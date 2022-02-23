#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

using namespace std;

class RobotNewsNode: public rclcpp::Node{
    public:
        RobotNewsNode(): Node("robot_news_station"){

            this->declare_parameter("robot_name","Berat");
            robot_name = this->get_parameter("robot_name").as_string();
            
            publisher = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
            timer = this->create_wall_timer(chrono::milliseconds(500), bind(&RobotNewsNode::publishNews, this));
            RCLCPP_INFO(this->get_logger(),"Station is started.");
        }

    private:

        void publishNews(){
            auto msg = example_interfaces::msg::String();
            string d = "";
            d += "Hello, "; d += "I'm "; d += robot_name; d += " from the station!";
            msg.data = d;
            publisher->publish(msg);
        }

        string robot_name;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher;
        rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = make_shared<RobotNewsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}