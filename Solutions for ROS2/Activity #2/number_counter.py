#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64


class CounterNode(Node):
    
    def __init__(self):
        super().__init__("number_counter")

        self.subscriber = self.create_subscription(Int64,"number", self.callback_number, 10)
        self.get_logger().info("Sub to 'number' topic has been started!")

        self.publisher = self.create_publisher(Int64,"number_count", 10)

        self.count = 0

    def callback_number(self, msg):
        new_msg = Int64()
        new_msg.data = msg.data + self.count
        self.publisher.publish(new_msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = CounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()