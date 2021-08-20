#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std;

class ParametersNode : public rclcpp::Node {
public:
    ParametersNode() : Node("parameter_node") {
        // create paramter
        this->declare_parameter<string>("my_param", "world");
        
        // create timer
        timer_ = this->create_wall_timer(1000ms, std::bind(&ParametersNode::Respond, this));
    }

    void Respond() {
        // get parameter value
        this->get_parameter("my_param", param_);
        RCLCPP_INFO(this->get_logger(), "Hello %s", param_.c_str());
    }

private:
    string param_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParametersNode>());
    rclcpp::shutdown();
    return 0;
}