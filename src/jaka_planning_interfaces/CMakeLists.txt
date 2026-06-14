cmake_minimum_required(VERSION 3.8)
project(jaka_planning_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EnvironmentModel.msg"
  "msg/EnvironmentObject.msg"
  "action/PlanMotion.action"
  DEPENDENCIES geometry_msgs moveit_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
