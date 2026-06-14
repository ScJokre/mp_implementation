#!/usr/bin/env python3

from copy import deepcopy
from threading import Lock

import rclpy
from geometry_msgs.msg import Pose
from jaka_planning_interfaces.action import PlanMotion
from jaka_planning_interfaces.msg import EnvironmentModel, EnvironmentObject
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__("motion_planner")

        self.declare_parameter("environment_topic", "/known_environment")
        self.declare_parameter("planning_action", "/plan_motion")
        self.declare_parameter("move_group_action", "/move_action")
        self.declare_parameter("apply_scene_service", "/apply_planning_scene")
        self.declare_parameter("default_planning_group", "jaka_a5")
        self.declare_parameter("default_end_effector_link", "tool0")
        self.declare_parameter("default_planning_time", 5.0)
        self.declare_parameter("default_planning_attempts", 10)
        self.declare_parameter("default_velocity_scaling", 0.2)
        self.declare_parameter("default_acceleration_scaling", 0.2)

        callback_group = ReentrantCallbackGroup()
        environment_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._environment_lock = Lock()
        self._latest_environment = None
        self._applied_object_ids = set()
        self._task_lock = Lock()
        self._busy = False
        self._active_move_group_goal = None

        self.create_subscription(
            EnvironmentModel,
            self.get_parameter("environment_topic").value,
            self._environment_callback,
            environment_qos,
            callback_group=callback_group,
        )
        self._scene_client = self.create_client(
            ApplyPlanningScene,
            self.get_parameter("apply_scene_service").value,
            callback_group=callback_group,
        )
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            self.get_parameter("move_group_action").value,
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            PlanMotion,
            self.get_parameter("planning_action").value,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )

        self.get_logger().info(
            "Ready: environment=%s, planning_action=%s"
            % (
                self.get_parameter("environment_topic").value,
                self.get_parameter("planning_action").value,
            )
        )

    def _environment_callback(self, message):
        with self._environment_lock:
            if (
                self._latest_environment is not None
                and message.version < self._latest_environment.version
            ):
                self.get_logger().warning(
                    f"Ignoring older environment version {message.version}; "
                    f"latest is {self._latest_environment.version}."
                )
                return
            self._latest_environment = deepcopy(message)

        self.get_logger().info(
            f"Cached environment version {message.version} "
            f"with {len(message.objects)} object(s)."
        )

    def _goal_callback(self, goal_request):
        with self._task_lock:
            if self._busy:
                self.get_logger().warning(
                    f"Rejecting task '{goal_request.task_id}': planner is busy."
                )
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        if self._active_move_group_goal is not None:
            self._active_move_group_goal.cancel_goal_async()
        return CancelResponse.ACCEPT

    @staticmethod
    def _feedback(goal_handle, state):
        feedback = PlanMotion.Feedback()
        feedback.state = state
        goal_handle.publish_feedback(feedback)

    @staticmethod
    def _primitive_type(environment_object):
        mapping = {
            EnvironmentObject.BOX: (SolidPrimitive.BOX, 3),
            EnvironmentObject.SPHERE: (SolidPrimitive.SPHERE, 1),
            EnvironmentObject.CYLINDER: (SolidPrimitive.CYLINDER, 2),
        }
        if environment_object.primitive_type not in mapping:
            raise ValueError(
                f"Object '{environment_object.id}' has unsupported primitive type "
                f"{environment_object.primitive_type}."
            )
        return mapping[environment_object.primitive_type]

    def _make_collision_object(self, frame_id, environment_object):
        primitive_type, expected_dimensions = self._primitive_type(environment_object)
        if len(environment_object.dimensions) != expected_dimensions:
            raise ValueError(
                f"Object '{environment_object.id}' requires {expected_dimensions} "
                f"dimension(s), got {len(environment_object.dimensions)}."
            )
        if any(dimension <= 0.0 for dimension in environment_object.dimensions):
            raise ValueError(
                f"Object '{environment_object.id}' dimensions must be positive."
            )
        if not environment_object.id:
            raise ValueError("Every environment object requires a non-empty ID.")

        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = list(environment_object.dimensions)

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = environment_object.id
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(environment_object.pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    async def _apply_latest_environment(self, minimum_version):
        with self._environment_lock:
            environment = deepcopy(self._latest_environment)

        if environment is None:
            return False, "No known environment has been received."
        if environment.version < minimum_version:
            return (
                False,
                f"Environment version {environment.version} is older than required "
                f"version {minimum_version}.",
            )

        frame_id = environment.header.frame_id or "world"
        incoming_id_list = [item.id for item in environment.objects]
        incoming_ids = set(incoming_id_list)
        if len(incoming_ids) != len(incoming_id_list):
            return False, "Environment object IDs must be unique."
        scene = PlanningScene()
        scene.is_diff = True

        try:
            scene.world.collision_objects.extend(
                self._make_collision_object(frame_id, item)
                for item in environment.objects
            )
        except ValueError as error:
            return False, str(error)

        if environment.replace:
            for object_id in self._applied_object_ids - incoming_ids:
                removal = CollisionObject()
                removal.header.frame_id = frame_id
                removal.id = object_id
                removal.operation = CollisionObject.REMOVE
                scene.world.collision_objects.append(removal)

        if not self._scene_client.wait_for_service(timeout_sec=10.0):
            return False, "/apply_planning_scene is unavailable."

        request = ApplyPlanningScene.Request()
        request.scene = scene
        response = await self._scene_client.call_async(request)
        if response is None or not response.success:
            return False, "MoveIt rejected the planning scene update."

        if environment.replace:
            self._applied_object_ids = incoming_ids
        else:
            self._applied_object_ids.update(incoming_ids)
        return True, f"Applied environment version {environment.version}."

    def _request_value(self, value, parameter_name):
        if value > 0:
            return value
        return self.get_parameter(parameter_name).value

    @staticmethod
    def _make_joint_constraints(joint_state):
        if not joint_state.name or len(joint_state.name) != len(joint_state.position):
            raise ValueError("Joint goal names and positions must have equal lengths.")

        constraints = Constraints()
        constraints.name = "joint_goal"
        for name, position in zip(joint_state.name, joint_state.position):
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = position
            constraint.tolerance_above = 0.001
            constraint.tolerance_below = 0.001
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        return constraints

    @staticmethod
    def _make_pose_constraints(
        goal_pose, link_name, position_tolerance, orientation_tolerance
    ):
        frame_id = goal_pose.header.frame_id or "world"

        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [position_tolerance]
        region_pose = Pose()
        region_pose.position = goal_pose.pose.position
        region_pose.orientation.w = 1.0

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = link_name
        position.constraint_region.primitives.append(region)
        position.constraint_region.primitive_poses.append(region_pose)
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = frame_id
        orientation.link_name = link_name
        orientation.orientation = goal_pose.pose.orientation
        orientation.absolute_x_axis_tolerance = orientation_tolerance
        orientation.absolute_y_axis_tolerance = orientation_tolerance
        orientation.absolute_z_axis_tolerance = orientation_tolerance
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.name = "pose_goal"
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _make_move_group_goal(self, request):
        goal = MoveGroup.Goal()
        motion_request = goal.request
        motion_request.group_name = (
            request.planning_group
            or self.get_parameter("default_planning_group").value
        )
        motion_request.num_planning_attempts = int(
            self._request_value(
                request.num_planning_attempts, "default_planning_attempts"
            )
        )
        motion_request.allowed_planning_time = float(
            self._request_value(
                request.allowed_planning_time, "default_planning_time"
            )
        )
        motion_request.max_velocity_scaling_factor = float(
            self._request_value(
                request.velocity_scaling, "default_velocity_scaling"
            )
        )
        motion_request.max_acceleration_scaling_factor = float(
            self._request_value(
                request.acceleration_scaling, "default_acceleration_scaling"
            )
        )

        if request.use_current_start_state:
            motion_request.start_state.is_diff = True
        else:
            if not request.start_state.name:
                raise ValueError("An explicit start state requires joint names.")
            motion_request.start_state.joint_state = request.start_state

        if request.goal_type == PlanMotion.Goal.JOINT_GOAL:
            constraints = self._make_joint_constraints(request.joint_goal)
        elif request.goal_type == PlanMotion.Goal.POSE_GOAL:
            link_name = (
                request.end_effector_link
                or self.get_parameter("default_end_effector_link").value
            )
            position_tolerance = request.position_tolerance or 0.01
            orientation_tolerance = request.orientation_tolerance or 0.05
            constraints = self._make_pose_constraints(
                request.pose_goal,
                link_name,
                position_tolerance,
                orientation_tolerance,
            )
        else:
            raise ValueError(f"Unsupported goal type {request.goal_type}.")

        motion_request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = request.plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.replan = False
        return goal

    async def _execute_callback(self, goal_handle):
        request = goal_handle.request
        result = PlanMotion.Result()

        try:
            self._feedback(goal_handle, "applying_environment")
            scene_ok, scene_message = await self._apply_latest_environment(
                request.minimum_environment_version
            )
            if not scene_ok:
                result.message = scene_message
                goal_handle.abort()
                return result

            if goal_handle.is_cancel_requested:
                result.message = "Task canceled before planning."
                goal_handle.canceled()
                return result

            try:
                move_group_goal = self._make_move_group_goal(request)
            except ValueError as error:
                result.message = str(error)
                goal_handle.abort()
                return result

            self._feedback(goal_handle, "waiting_for_move_group")
            if not self._move_group_client.wait_for_server(timeout_sec=10.0):
                result.message = "/move_action is unavailable."
                goal_handle.abort()
                return result

            self._feedback(
                goal_handle,
                "planning" if request.plan_only else "planning_and_executing",
            )
            self._active_move_group_goal = await self._move_group_client.send_goal_async(
                move_group_goal,
                feedback_callback=lambda feedback: self._feedback(
                    goal_handle, feedback.feedback.state
                ),
            )
            if not self._active_move_group_goal.accepted:
                result.message = "MoveIt rejected the motion request."
                goal_handle.abort()
                return result

            move_group_result = (
                await self._active_move_group_goal.get_result_async()
            ).result
            result.moveit_error_code = move_group_result.error_code.val
            result.planning_time = move_group_result.planning_time
            result.planned_trajectory = move_group_result.planned_trajectory
            result.success = move_group_result.error_code.val == MoveItErrorCodes.SUCCESS
            result.message = (
                f"{scene_message} MoveIt completed task '{request.task_id}'."
                if result.success
                else f"MoveIt failed task '{request.task_id}' with error "
                f"{move_group_result.error_code.val}."
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as error:  # Keep the action server alive on integration errors.
            self.get_logger().error(f"Task '{request.task_id}' failed: {error}")
            result.message = str(error)
            goal_handle.abort()
            return result
        finally:
            self._active_move_group_goal = None
            with self._task_lock:
                self._busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
