/**
 * subsribe the broadcasting info from turtle1 and turtle2, search the nearest 
 * tf info, then change the pose of turtle1 relative to turtle2,
 * calculate the linear and angular speed 
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <memory>

using namespace std;

int main(int argc, char **argv) {
    // init ros node
    rclcpp::init(argc, argv);

    // create node
    auto node = rclcpp::Node::make_shared("turtle2_controller");

    // create tf subsciber object, traditional pointer, need delete
    // tf2_ros::Buffer *buffer;
    // buffer = new tf2_ros::Buffer(node->get_clock());
    // tf2_ros::TransformListener tfListener(*buffer);
    // use smart pointer, safe
    // shared_ptr<tf2_ros::Buffer> buffer(new tf2_ros::Buffer(node->get_clock()));
    shared_ptr<tf2_ros::Buffer> buffer = make_shared<tf2_ros::Buffer>(node->get_clock());
    tf2_ros::TransformListener tfListener(*buffer);

    // create speed publiser
    auto velPub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 1000);
    RCLCPP_INFO(node->get_logger(), "vel pub created!");

    // loop for speed pub
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        // get the coordinate of turtle1 related to turtle2
        try {
            geometry_msgs::msg::TransformStamped transStamped = buffer->lookupTransform("turtle2", "turtle1", tf2::TimePointZero);
            RCLCPP_INFO(node->get_logger(), "son1 related to son2: father %s, child %s offset(%.2f, %.2f, %.2f)",
                transStamped.header.frame_id.c_str(),
                transStamped.child_frame_id.c_str(),
                transStamped.transform.translation.x,
                transStamped.transform.translation.y,
                transStamped.transform.translation.z    
            );

            // compose twist info
            geometry_msgs::msg::Twist twist;
            twist.linear.x = 0.5 * sqrt(pow(transStamped.transform.translation.x, 2) + pow(transStamped.transform.translation.y, 2));
            twist.angular.z = 4 * atan2(transStamped.transform.translation.y, transStamped.transform.translation.x);

            // pub the twist info timely
            velPub->publish(twist);
        } catch (tf2::TransformException &e) {
            RCLCPP_WARN(node->get_logger(), "warning %s", e.what());
        }

        rate.sleep();
    }

    // spin
    rclcpp::spin(node); // if put spin into while loop, will throw exception

    rclcpp::shutdown();

    // delete buffer;

    return 0;
}