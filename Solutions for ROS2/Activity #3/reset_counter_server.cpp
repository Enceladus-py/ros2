#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using namespace std;
using placeholders::_1;
using placeholders::_2;

class ResetCounterServerNode: public rclcpp::Node{
    public:
        ResetCounterServerNode(): Node("reset_counter_service"){

            server = this->create_service<example_interfaces::srv::SetBool>("reset_counter",
            bind(&ResetCounterServerNode::callbackResetCounter, this, _1, _2)); //for request and response
            RCLCPP_INFO(this->get_logger(), "Reset counter service has been started.");


        };

    private:

        void callbackResetCounter(const example_interfaces::srv::SetBool::Request::SharedPtr request,
        const example_interfaces::srv::SetBool::Response::SharedPtr response){
            if (request->data == true){
                response->message = "0"; // resets the self.counter in number_counter.py
                response->success = true;
            }
        }

        rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server;
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = make_shared<ResetCounterServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}