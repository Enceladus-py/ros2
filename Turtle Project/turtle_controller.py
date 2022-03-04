#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import PosWithName

from turtlesim.srv import Kill
from turtlesim.srv import TeleportAbsolute


"""

string name
---

"""

"""

float32 x
float32 y
float32 theta
---

"""

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.positions = []
        self.current_x = 5.544445 
        self.current_y = 5.544445 
        self.subscriber = self.create_subscription(PosWithName,"positions", self.callback_controller, 10)
        self.create_timer(5.0, self.callback_find_short_distance)
        self.get_logger().info("Turtle controller has been started!")

    def callback_controller(self, msg):
        self.positions.append((msg.x,msg.y,msg.name))
        self.get_logger().info(f"{msg.name} is added to positions.")


    def callback_find_short_distance(self):
        min = (self.current_x-self.positions[0][0])**2 + (self.current_y-self.positions[0][1])**2
        min_index = 0
        i = 1

        for pos in self.positions[1:]:
            if ( (pos[0]-self.current_x)**2 + (pos[1]-self.current_y)**2) < min:
                min = (pos[0]-self.current_x)**2 + (pos[1]-self.current_y)**2
                min_index = i
            i += 1

        self.current_x = self.positions[min_index][0]
        self.current_y = self.positions[min_index][1]
        self.get_logger().info(f"Turtle with the min. distance is {self.positions[min_index][2]}")

        client = self.create_client(Kill, "kill")
        request = Kill.Request()
        request.name = self.positions[min_index][2]
        future = client.call_async(request)
        future.add_done_callback(self.callback_kill_turtle)

        self.positions.pop(min_index)

        client = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        request = TeleportAbsolute.Request()
        request.x = self.current_x
        request.y = self.current_y
        future = client.call_async(request)
        future.add_done_callback(self.callback_teleport_turtle)

    def callback_kill_turtle(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed! %r",(e,))

    def callback_teleport_turtle(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed! %r",(e,))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()