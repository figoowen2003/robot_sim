/**
 *  spawn a new turtle
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

using namespace std;

int main(int argc, char **argv) {
    // init, start node, forward command line arguments to ROS
    rclcpp::init(argc, argv);

    // create spawn node
    auto node = rclcpp::Node::make_shared("turtle_spawner");
    RCLCPP_INFO(node->get_logger(), "create turtle spawner");

    // create spawn service client
    auto spawnerClient = node->create_client<turtlesim::srv::Spawn>("/spawn");

    // init a new turtle info
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = "turtle2";
    request->x = 1.5;
    request->y = 2.0;
    request->theta = 3.333;

    // make sure service has been connected
    while (!spawnerClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service");
            return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // call the spawn service
    auto result = spawnerClient->async_send_request(request);
    //wait for result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto r = result.get();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "result got, %s created sucessfully!", r->name.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "fail to get result, turtle created failed!");
    }

    // spin function
    rclcpp::spin(node);

    // shutdown
    rclcpp::shutdown();

    return 0;
}