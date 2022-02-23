#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.srv import SetLed
from my_robot_interfaces.msg import LedStatus

class LedPanelServerNode(Node):
    
    def __init__(self):
        super().__init__("set_led_server")
        self.create_service(SetLed, "set_led", self.callback_set_led_server)
        self.get_logger().info("set_led service is started")
        self.publisher = self.create_publisher(LedStatus, "led_states", 10)
        self.msg = LedStatus()
        self.timer = self.create_timer(1.0,self.publish_states)
        self.get_logger().info("publishing on led_states topic")


    def callback_set_led_server(self, request, response):
        if request.state:
            self.msg.led_status_array[request.led_number] = 1
            self.get_logger().info(f"State for the LED {request.led_number} is changed to 1.")
        else:
            self.msg.led_status_array[request.led_number] = 0
            self.get_logger().info(f"State for the LED {request.led_number} is changed to 0.")

        response.success = True
        return response

    def publish_states(self):
        self.publisher.publish(self.msg)  

def main(args=None):
    rclpy.init(args=args)
    node = LedPanelServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()