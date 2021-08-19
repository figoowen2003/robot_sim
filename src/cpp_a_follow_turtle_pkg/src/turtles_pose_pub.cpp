/**
 * subscribe the pose of the two turtles,then broadcast
 * their coordinates to the world base
 */

#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <iostream>

using namespace std;
using std::placeholders::_1;

rclcpp::Node::SharedPtr node;
string turtleName;

void CallBack(const turtlesim::msg::Pose::SharedPtr msg) {
    // create broadcastor for TF
    // static tf2_ros::TransformBroadcaster tfBr;
    static auto tfBr = std::make_shared<tf2_ros::TransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped tfStamped;

    // get turtle pose and change it to transformStamped info
    tfStamped.header.stamp = rclcpp::Time();
    tfStamped.header.frame_id = "world";   // father frame
    // tfStamped.child_frame_id = "turtle1";  // child frame
    tfStamped.child_frame_id = turtleName;
    // x,y,z info
    tfStamped.transform.translation.x = msg->x;
    tfStamped.transform.translation.y = msg->y;
    tfStamped.transform.translation.z = 0.0;
    // use euler angle to get quaternion
    tf2::Quaternion qtn;
    qtn.setRPY(0.0, 0.0, msg->theta);
    tfStamped.transform.rotation.x = qtn.x();
    tfStamped.transform.rotation.y = qtn.y();
    tfStamped.transform.rotation.z = qtn.z();
    tfStamped.transform.rotation.w = qtn.w();

    // publish tf
    tfBr->sendTransform(tfStamped);
}

int main(int argc, char **argv) {
    // init node, deliver args to node
    rclcpp::init(argc, argv);

    // create node
    node = rclcpp::Node::make_shared("turtle_pub");
    cout << "argc = " << argc << endl;
    cout << "argv[0] = " << argv[0] << endl;
    cout << "argv[1] = " << argv[1] << endl;
    cout << "argv[2] = " << argv[2] << endl;
    cout << "argv[3] = " << argv[3] << endl;
    cout << "argv[4] = " << argv[4] << endl;
    // parse the args from main
    if (argc != 5) {
        RCLCPP_INFO(node->get_logger(), "args not corrected!");
        return 0;
    } else {
        turtleName = argv[1];
        RCLCPP_INFO(node->get_logger(), "got turtle name: %s", turtleName.c_str());
    }

    // create subscribe to turtle pose
    auto pose_sub = node->create_subscription<turtlesim::msg::Pose>(turtleName + "/pose", 100,
        std::bind(&CallBack, _1));

    // ros2 spin function
    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}