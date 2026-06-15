#!/usr/bin/env python3

import rclpy
from jaka_planning_interfaces.msg import EnvironmentModel, EnvironmentObject
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class ExampleEnvironmentPublisher(Node):
    def __init__(self):
        super().__init__("example_environment")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            EnvironmentModel, "/known_environment", qos
        )
        self.publish_environment()

    @staticmethod
    def make_box(object_id, dimensions, position):
        item = EnvironmentObject()
        item.id = object_id
        item.primitive_type = EnvironmentObject.BOX
        item.dimensions = dimensions
        item.pose.position.x, item.pose.position.y, item.pose.position.z = position
        item.pose.orientation.w = 1.0
        return item

    def publish_environment(self):
        message = EnvironmentModel()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.version = 5
        message.replace = True
        message.objects = [
            self.make_box("table", [1.4, 1.4, 0.10], [0.35, 0.0, -0.08]),
            # Raised board placed farther from the robot base for a clear demo.
            self.make_box(
                "inspection_board",
                [0.30, 0.40, 0.03],
                [0.35, 0.0, 0.72],
            ),
        ]
        self.publisher.publish(message)
        self.get_logger().info("Published example environment version 5.")


def main(args=None):
    rclpy.init(args=args)
    node = ExampleEnvironmentPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
