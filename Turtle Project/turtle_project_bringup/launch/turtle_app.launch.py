from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    turtle_controller_node = Node(
        package="my_py_pkg",
        executable="turtle_controller",
    )
    spawn_turtle_node = Node(
        package="my_py_pkg",
        executable="turtle_spawner",                
    )
                      

    ld.add_action(turtlesim_node)
    ld.add_action(turtle_controller_node)
    ld.add_action(spawn_turtle_node)

    return ld