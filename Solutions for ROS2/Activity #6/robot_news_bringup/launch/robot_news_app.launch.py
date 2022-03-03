from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 pkg create robot_news_bringup  //creates the package (in src file)
# remove include and src files 
# create install folder and a python file with chmod +x
# colcon build --packages-select robot_news_bringup --symlink-install

def generate_launch_description(): 
    ld = LaunchDescription()

    robot_news_station_node1 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_1",  
        parameters=[
            {"robot_name":"I1"}
        ]                      
    )
    robot_news_station_node2 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_2",    
        parameters=[
            {"robot_name":"I2"}
        ]                            
    )
    robot_news_station_node3 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_3",    
        parameters=[
            {"robot_name":"I3"}
        ]                    
    )
    robot_news_station_node4 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_4",
        parameters=[
            {"robot_name":"I4"}
        ]                        
    )

    ld.add_action(robot_news_station_node1)
    ld.add_action(robot_news_station_node2)
    ld.add_action(robot_news_station_node3)
    ld.add_action(robot_news_station_node4)

    return ld