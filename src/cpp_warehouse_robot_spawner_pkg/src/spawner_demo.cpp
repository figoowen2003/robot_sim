#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/executors.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
    // Start node, forward command line arguments to ROS
    rclcpp::init(argc, argv);

    // Create spawn node
    // rclcpp::Node node = rclcpp::Node("entity_spawner");
    auto node = rclcpp::Node::make_shared("entity_spawner");

    // Show progress in the terminal window
    RCLCPP_INFO(node->get_logger(), "Creating Service client to connect to '/spawn_entity'");
    // Create SpawnEntity
    // rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");

    // Get the spawn_entity service
    RCLCPP_INFO(node->get_logger(), "Connecting to '/spawn_entity' service...");
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_INFO(node->get_logger(), "spawn client was interrupted while \
                waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Service not available, waiting again...");
    }

    // Get the file path for the robot model
    // may throw PackageNotFoundError exception
    string package_share_directory = ament_index_cpp::get_package_share_directory("warehouse_robot_spawner_pkg");
    fs::path dir(package_share_directory);
    fs::path sdfFilePath = dir / fs::path("models") / fs::path("mobile_warehouse_robot") 
        / fs::path("model.sdf");
    cout << "robt_sdf = " << sdfFilePath << endl;
    // Get the content of robot sdf file
    ifstream sdfFile(sdfFilePath);
    string line;
    stringstream buffer;
    if (sdfFile.is_open()) {       
        buffer << sdfFile.rdbuf();
        // cout << "buffer = " << buffer.str() << endl;          
        sdfFile.close();
    } else {
        cout << "======== open sdf failed ========" << endl;
    }  

    // Set data for request, argvs come from gazebo_world.launch.py
    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = argv[1];
    cout << argv[1] << endl;
    request->xml = string(buffer.str());
    request->robot_namespace = string(argv[2]); // different from python 
    cout << "argc = " << argc << endl;
    cout << "argv[0] = " << argv[0] << endl;
    cout << "argv[1] = " << argv[1] << endl;
    cout << "argv[2] = " << argv[2] << endl;
    cout << "argv[3] = " << argv[3] << endl;
    cout << "argv[4] = " << argv[4] << endl;
    cout << "argv[5] = " << argv[5] << endl;
    cout << "argv[6] = " << argv[6] << endl;
    request->initial_pose.position.x = atof(argv[3]);
    request->initial_pose.position.y = atof(argv[4]);
    request->initial_pose.position.z = atof(argv[5]) ;

    RCLCPP_INFO(node->get_logger(), "Sending service request to '/spawn_entity'");
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        cout << "response: OK" << endl;
    } else {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return -1;
    }

    // auto future_result = client->async_send_request(request);

    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(node, future_result) ==
    //     rclcpp::executor::FutureReturnCode::SUCCESS)
    // {
    //     printf("Result of add_two_ints: %zd\n", future_result.get()->sum);
    // } else {
    //     printf("add_two_ints_client_async was interrupted. Exiting.\n");
    // }    

    RCLCPP_INFO(node->get_logger(), "Done! Shutting down node.");
    rclcpp::shutdown();
    return 0;                 
}