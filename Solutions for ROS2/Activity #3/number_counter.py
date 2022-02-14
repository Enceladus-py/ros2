#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

from functools import partial

class CounterNode(Node):
    
    def __init__(self):
        super().__init__("number_counter")

        self.subscriber = self.create_subscription(Int64,"number", self.callback_number, 10)
        self.get_logger().info("Sub to 'number' topic has been started!")

        self.publisher = self.create_publisher(Int64,"number_count", 10)

        self.count = 0

        # first way to do the activity 3
        # self.reset_counter_server = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)

    def callback_number(self, msg):
        new_msg = Int64()
        new_msg.data = msg.data + self.count
        self.publisher.publish(new_msg)

        # second way to do activity 3 (server is in cpp pkg)
        if self.count>20:
            self.call_reset_counter_server(True)

        self.count += 5

    # first way to do the activity 3
    """ 
    def callback_reset_counter(self, request, response):
        if request.data:
            self.count = 0

        response.success = True
        response.message = "Counter reset!"
        
        return response
    """

    # second way to do activity 3
    def call_reset_counter_server(self, data):

        client = self.create_client(SetBool, "reset_counter")

        """
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for the server 'reset_counter'...")
        """

        request = SetBool.Request()
        request.data = data

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_reset_counter, data=data))

    # second way to do activity 3
    def callback_reset_counter(self, future, data):

        try:
            response = future.result()
            if response.success:
                self.count = int(response.message)
                self.get_logger().info("Counter Reset!")
        except Exception as e:
            self.get_logger().error("Service call failed! %r",(e,))


def main(args=None):
    rclpy.init(args=args)
    node = CounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()