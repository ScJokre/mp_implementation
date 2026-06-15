# mp_implementation

Automated JAKA A5 motion-planning implementation for integrating environment
reconstruction, decision making, and MoveIt motion planning.

Repository: <https://github.com/ScJokre/mp_implementation>

This is a separate ROS 2 workspace. It reuses the installed
`jaka_a5_moveit_config` package but does not modify `jaka_a5_ros2_ws`.

## Intended project flow

```text
Gazebo RGB-D camera
  -> PointCloud2
  -> environment reconstruction / optimization
  -> /known_environment (versioned collision model)
  -> decision module (TSP + probability model)
  -> /plan_motion action (start state + target viewpoint)
  -> jaka_motion_pipeline
  -> MoveIt Planning Scene + MoveGroup
  -> planned trajectory / simulated execution
```

The motion-planning package intentionally does not consume raw point clouds.
The reconstruction module owns point-cloud processing and publishes a compact,
stable collision model. This keeps motion planning independent from the
specific RGB-D camera and reconstruction algorithm.

## Packages

### `jaka_planning_interfaces`

- `EnvironmentModel`: versioned known environment.
- `EnvironmentObject`: box, sphere, or cylinder collision primitive.
- `PlanMotion`: action used by the decision module to request planning or
  planning plus execution.

### `jaka_motion_pipeline`

- Caches the newest `/known_environment`.
- Rejects requests that require a newer environment version.
- Applies the environment through MoveIt's `/apply_planning_scene` service.
- Converts joint or pose goals into a MoveIt `/move_action` request.
- Returns the MoveIt error code, planning time, and planned trajectory.

Only one motion task is accepted at a time. This matches the sequential
viewpoint tasks produced by the decision module.

## Build in WSL

Build and source the existing JAKA workspace first:

```bash
cd ~/jaka_a5_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Then build this independent workspace:

```bash
cd ~/mp_implementation
source /opt/ros/jazzy/setup.bash
source ~/jaka_a5_ros2_ws/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Automated smoke test

This starts the existing JAKA demo, publishes a sample known environment, and
automatically plans and executes a joint-space task:

```bash
ros2 launch jaka_motion_pipeline automatic_demo.launch.py send_sample_task:=true
```

Use planning without execution:

```bash
ros2 launch jaka_motion_pipeline automatic_demo.launch.py \
  send_sample_task:=true plan_only:=true
```

No RViz interaction is required. RViz remains open only for visualization.

## Test the viewpoint interface

The decision module's target should ultimately be a camera pose. The included
viewpoint example accepts a camera/tool position and a point to look at, then
converts the viewing direction into an orientation quaternion:

```bash
ros2 run jaka_motion_pipeline example_viewpoint_task --ros-args \
  -p position:="[0.45, 0.0, 0.45]" \
  -p look_at:="[0.45, 0.0, 0.0]" \
  -p plan_only:=true
```

The current example assumes `tool0` looks forward along its local `+Z` axis.
This is only an integration placeholder. After adding the simulated camera,
set `end_effector_link` to the camera's controlled frame and verify its optical
axis convention.

## Test movement above and below the board

The sample environment contains a horizontal inspection board centered at
`[0.35, 0.0, 0.72]`. The board is placed above the initial robot state; its
bottom surface is approximately `z=0.705`. Start the system without the
automatic joint task:

```bash
ros2 launch jaka_motion_pipeline automatic_demo.launch.py
```

In another sourced terminal, run the two-state sequence:

```bash
ros2 run jaka_motion_pipeline example_board_sequence
```

The sequence automatically executes:

1. Test above-board targets around `[0.35, 0.0, 0.86]`, approximately `0.14 m`
   above the board.
2. Execute the first reachable above-board target.
3. Pause for two seconds.
4. Test below-board targets around `z=0.57` using plan-only requests.
5. Execute the first reachable below-board position.

MoveIt must plan around the board rather than passing through it. The sequence
uses a low velocity scaling of `0.15` so the movement is easy to observe.
Candidate probing is necessary because placing the complete arm directly under
the center of a horizontal board may not have a collision-free IK solution.
This demonstration uses a relaxed orientation tolerance so it primarily tests
position planning and collision avoidance. Camera-facing constraints will be
tightened after the simulated camera link is added.

## Export board poses as RViz named states

RViz named states contain joint angles, not Cartesian target positions. Start
the normal automatic demo, then calculate collision-free joint states for all
board candidates:

```bash
ros2 run jaka_motion_pipeline export_board_states
```

Successful candidates are written to:

```text
~/.ros/jaka_a5_board_states.srdf
```

Candidates that fail planning cannot become named states because MoveIt did
not find a valid joint configuration for them. The exporter also publishes
colored target-position markers on `/board_state_targets`. To inspect all
candidate positions while the normal demo is running:

1. In RViz, select `Add`.
2. Add a `MarkerArray` display.
3. Set its topic to `/board_state_targets`.
4. Run `export_board_states` again.

After exporting, stop the normal demo and launch MoveIt with the generated
states:

```bash
ros2 launch jaka_motion_pipeline board_states_demo.launch.py
```

In RViz, open `MotionPlanning -> Planning Request` and select generated states
such as `board_above_1` or `board_below_1` from the Goal State selector. This
launch uses the generated SRDF from `~/.ros` and does not modify
`jaka_a5_ros2_ws`.

## Integration contracts

### Environment reconstruction output

Publish `jaka_planning_interfaces/msg/EnvironmentModel` on
`/known_environment` with reliable, transient-local QoS:

- `header.frame_id`: normally `world`.
- `version`: increment whenever the optimized environment changes.
- `replace`: set true for a complete environment snapshot.
- `objects`: collision primitives with stable, unique IDs.

The current interface supports primitives because they are simple and reliable
for the first integrated simulation. If the optimizer produces a triangle
mesh, extend `EnvironmentObject` and the collision-object conversion. If it
produces dense occupancy data, use MoveIt's occupancy-map/OctoMap path instead
of generating thousands of primitive objects.

### Decision module output

Send `jaka_planning_interfaces/action/PlanMotion` goals to `/plan_motion`:

- `minimum_environment_version`: environment version used by the decision.
- `use_current_start_state`: normally true during execution.
- `goal_type`: `JOINT_GOAL` for testing or `POSE_GOAL` for viewpoints.
- `pose_goal`: target position and orientation of the camera/tool frame.
- `plan_only`: false to plan and execute in simulation.

The action result reports whether MoveIt succeeded and includes the planned
trajectory. A TSP-style decision module should wait for each result before
sending the next viewpoint.

## Gazebo connection work

The next integration stage is separate from this motion-planning node:

1. Create a simulation description package in this workspace that adds a
   fixed RGB-D camera link and Gazebo sensor to the JAKA model.
2. Bridge or publish the simulated depth point cloud as
   `sensor_msgs/msg/PointCloud2`.
3. Connect the reconstruction/optimization module to that point cloud.
4. Publish the optimized result as `/known_environment`.
5. Configure the decision module to send `/plan_motion` pose goals for the
   camera frame.

For real hardware, the mock `ros2_control` controller must later be replaced
with a verified JAKA ROS 2 hardware interface. The planning API can remain the
same.
