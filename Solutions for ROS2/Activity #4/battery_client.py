#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed

from functools import partial

class BatteryClientNode(Node):
    
    def __init__(self):
        super().__init__("battery_client")
        self.st = False
        self.timer = self.create_timer(4.0, partial(self.call_set_led_server,a=2))

    def call_set_led_server(self, a):

        client = self.create_client(SetLed, "set_led")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server 'set_led'...")

        request = SetLed.Request()
        request.led_number = a
        request.state = self.st

        future = client.call_async(request)
        future.add_done_callback(self.callback_set_led)

        if self.st:
            self.st = False
        else:
            self.st = True

    def callback_set_led(self, future):
        try:
            response = future.result()
            if response.success:
                if self.st:
                    self.get_logger().info("Battery charged.")
                else:
                    self.get_logger().info("Battery is low.")

        except Exception as e:
            self.get_logger().error("Service call failed! %r",(e,))


def main(args=None):
    rclpy.init(args=args)
    node = BatteryClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()