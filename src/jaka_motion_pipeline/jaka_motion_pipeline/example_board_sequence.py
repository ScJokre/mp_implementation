#!/usr/bin/env python3

from time import sleep

import rclpy
from jaka_motion_pipeline.example_viewpoint_task import view_quaternion
from jaka_planning_interfaces.action import PlanMotion
from rclpy.action import ActionClient
from rclpy.node import Node


BOARD_CENTER = [0.48, 0.0, 0.30]
VIEWPOINTS = [
    ("above_board", [0.48, 0.0, 0.55], BOARD_CENTER),
    # Keep the same downward-facing orientation to make the lower IK goal easier.
    ("below_board", [0.48, 0.0, 0.20], [0.48, 0.0, 0.0]),
]


class ExampleBoardSequenceClient(Node):
    def __init__(self):
        super().__init__("example_board_sequence")
        self.declare_parameter("pause_seconds", 2.0)
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("end_effector_link", "tool0")
        self.client = ActionClient(self, PlanMotion, "/plan_motion")

    def make_goal(self, task_id, position, look_at):
        quaternion = view_quaternion(position, look_at)

        goal = PlanMotion.Goal()
        goal.task_id = task_id
        goal.minimum_environment_version = 2
        goal.planning_group = "jaka_a5"
        goal.end_effector_link = self.get_parameter("end_effector_link").value
        goal.use_current_start_state = True
        goal.goal_type = PlanMotion.Goal.POSE_GOAL
        goal.pose_goal.header.frame_id = "world"
        goal.pose_goal.header.stamp = self.get_clock().now().to_msg()
        (
            goal.pose_goal.pose.position.x,
            goal.pose_goal.pose.position.y,
            goal.pose_goal.pose.position.z,
        ) = position
        (
            goal.pose_goal.pose.orientation.x,
            goal.pose_goal.pose.orientation.y,
            goal.pose_goal.pose.orientation.z,
            goal.pose_goal.pose.orientation.w,
        ) = quaternion
        goal.position_tolerance = 0.02
        goal.orientation_tolerance = 0.15
        goal.plan_only = False
        goal.allowed_planning_time = 15.0
        goal.num_planning_attempts = 30
        goal.velocity_scaling = self.get_parameter("velocity_scaling").value
        goal.acceleration_scaling = 0.15
        return goal

    def execute_goal(self, task_id, position, look_at):
        self.get_logger().info(
            f"Sending '{task_id}' target at "
            f"x={position[0]:.2f}, y={position[1]:.2f}, z={position[2]:.2f}."
        )
        send_future = self.client.send_goal_async(
            self.make_goal(task_id, position, look_at),
            feedback_callback=lambda feedback: self.get_logger().info(
                f"{task_id}: {feedback.feedback.state}"
            ),
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"Task '{task_id}' was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        message = (
            f"{result.message} error_code={result.moveit_error_code}, "
            f"planning_time={result.planning_time:.3f}s"
        )
        if result.success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)
        return result.success

    def run_sequence(self):
        if not self.client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("/plan_motion is unavailable.")
            return False

        pause_seconds = self.get_parameter("pause_seconds").value
        for index, (task_id, position, look_at) in enumerate(VIEWPOINTS):
            if not self.execute_goal(task_id, position, look_at):
                self.get_logger().error("Stopping sequence after failed task.")
                return False
            if index < len(VIEWPOINTS) - 1:
                self.get_logger().info(
                    f"Reached board upper state; waiting {pause_seconds:.1f}s."
                )
                sleep(pause_seconds)

        self.get_logger().info("Completed above-board to below-board sequence.")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = ExampleBoardSequenceClient()
    try:
        node.run_sequence()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
