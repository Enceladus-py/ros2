#!/usr/bin/env python3
from turtle import position
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn
from my_robot_interfaces.msg import PosWithName

import random

"""
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name

"""

"""
float32 x
float32 y
float32 theta

float32 linear_velocity
float32 angular_velocity

"""

class SpawnTurtlesNode(Node):
    
    def __init__(self):
        super().__init__("spawn_turtles")
        self.x = 0
        self.y = 0
        self.publisher = self.create_publisher(PosWithName, "positions", 10)
        self.timer = self.create_timer(2.0, self.callback_spawn_turtles)

    def callback_spawn_turtles(self):
        client = self.create_client(Spawn, "spawn")

        request = Spawn.Request()
        self.x = random.uniform(0.0,10.0)
        self.y = random.uniform(0.0,10.0)
        request.x = self.x
        request.y = self.y

        future = client.call_async(request)
        future.add_done_callback(self.callback_spawn_client)

    def callback_spawn_client(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"{response.name} is spawned.")
            pos = PosWithName()
            pos.x = self.x
            pos.y = self.y
            pos.name = response.name
            self.publisher.publish(pos)

        except Exception as e:
            self.get_logger().error("Service call failed! %r",(e,))

def main(args=None):
    rclpy.init(args=args)
    node = SpawnTurtlesNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()