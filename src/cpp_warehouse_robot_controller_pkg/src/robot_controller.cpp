#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include<bits/stdc++.h>
#include<string>
#include<vector>

using namespace std;
using namespace std::placeholders;

class RobotController : public rclcpp::Node {
public:
    RobotController() : Node("Controller") {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        // This node subscribes to messages of type Float64MultiArray
        // over a topic named: /demo/state_est
        // The message represents the current estimated state:
        //      [x, y, yaw]
        est_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/demo/state_est", default_qos,
            std::bind(&RobotController::OnStateEstimatedMsg, this, _1));

        // Subscribe to sensor messages
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "laser_scan", default_qos,
            std::bind(&RobotController::OnSensorMsg, this, _1));

        // This node publishes the desired linear and angular velocity of the 
        // robot(in the robot chassis coordinate frame) to the /demo/cmd_vel topic
        // Using the diff_drive plugin enables the robot model to read this
        // /demo/cmd_vel topic and excute the motion accordingly
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/demo/cmd_vel", default_qos);        
    }
private:
    void OnStateEstimatedMsg(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        // Extract the position and orientation data
        // This callback is called each time a new messaege is received on 
        // "/demo/state_est" topic
        vector<double> currState(msg->data.begin(), msg->data.end());
        currentX_ = msg->data[0];
        currentY_ = msg->data[1];
        currentYaw_ = msg->data[2];
    
        // Command the robot to keep following the wall      
        FollowWall();
    }

    void OnSensorMsg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // This method gets called every time a LaserScan message is received on
        // the "/demo/laser/out" topic
        // Read the laser scan data that indicates distances
        // to obstacles in meters and extract 5 distinct laser readings to work with.
        // Each reading is separated by 45 degrees.
        // Assumes 181 laser readings, separated by 1 degree.
        // (e.g. -90 to 90 degrees.....0 to 180 degrees)

        // number of laser beams = str(len(msg.ranges))
        left_dist = msg->ranges[180];
        leftfront_dist = msg->ranges[135];
        front_dist = msg->ranges[90];
        rightfront_dist = msg->ranges[45];
        right_dist = msg->ranges[0];
    }

    void FollowWall() { 
    // This method causes the robot to follow the boundary of a wall.
    // Create a geometry_msgs/Twist message
    auto cmdMsg = std::make_unique<geometry_msgs::msg::Twist>();

    cmdMsg->linear.x = 0.0;
    cmdMsg->linear.y = 0.0;
    cmdMsg->linear.z = 0.0;
    cmdMsg->angular.x = 0.0;
    cmdMsg->angular.y = 0.0;
    cmdMsg->angular.z = 0.0;       
 
    // Logic for following the wall
    // >d means no wall detected by that laser beam
    // <d means an wall was detected by that laser beam
    double d = dist_thresh_wf;
    
    if (leftfront_dist > d && front_dist > d && rightfront_dist > d) {
        wall_following_state = "search for wall";
        cmdMsg->linear.x = forward_speed;
        cmdMsg->angular.z = (-1) * turning_speed_wf_slow;  //turn right to wall
    } else if (leftfront_dist > d && front_dist < d && rightfront_dist > d) {
        wall_following_state = "turn left";
        cmdMsg->angular.z = turning_speed_wf_fast;
    } else if (leftfront_dist > d && front_dist > d && rightfront_dist < d) {
        if (rightfront_dist < dist_too_close_to_wall) {
            // Getting too close to the wall
            wall_following_state = "turn left";
            cmdMsg->linear.x = forward_speed;
            cmdMsg->angular.z = turning_speed_wf_fast;
        } else {
            // Go straight ahead
            wall_following_state = "follow wall";
            cmdMsg->linear.x = forward_speed;
        }        
    } else if (leftfront_dist < d && front_dist > d && rightfront_dist > d) {
        wall_following_state = "search for wall";
        cmdMsg->linear.x = forward_speed;
        cmdMsg->angular.z = (-1) * turning_speed_wf_slow; // turn right to find wall
    } else if (leftfront_dist > d && front_dist < d && rightfront_dist < d) {
        wall_following_state = "turn left";
        cmdMsg->angular.z = turning_speed_wf_fast;
    } else if (leftfront_dist < d && front_dist < d && rightfront_dist > d) {
        wall_following_state = "turn left";
        cmdMsg->angular.z = turning_speed_wf_fast;
    } else if (leftfront_dist < d && front_dist < d && rightfront_dist < d) {
        wall_following_state = "turn left";
        cmdMsg->angular.z = turning_speed_wf_fast;
    } else if (leftfront_dist < d && front_dist > d && rightfront_dist < d) {
        wall_following_state = "search for wall";
        cmdMsg->linear.x = forward_speed;
        cmdMsg->angular.z = (-1) * turning_speed_wf_slow; // turn right to find wall
    } else {
        // do nothing
    }  
 
    // Send velocity command to the robot
    cmd_pub_->publish(std::move(cmdMsg));    
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr est_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Initialize the LaserScan sensor readings to some large value
    // Values are in meters.
    double left_dist = 999999.9; // Left
    double leftfront_dist = 999999.9; // Left-front
    double front_dist = 999999.9; // Front
    double rightfront_dist = 999999.9; // Right-front
    double right_dist = 999999.9; // Right
 
    /* ################### ROBOT CONTROL PARAMETERS ################## */
    // Maximum forward speed of the robot in meters per second
    // Any faster than this and the robot risks falling over.
    double forward_speed = 0.025;    

    // Current position and orientation of the robot in the global reference frame
    double currentX_{0.0};
    double currentY_{0.0};
    double currentYaw_{0.0};

    /* ############# WALL FOLLOWING PARAMETERS ####################### */     
    // Finite states for the wall following mode
    //  "turn left": Robot turns towards the left
    //  "search for wall": Robot tries to locate the wall        
    //  "follow wall": Robot moves parallel to the wall
    string wall_following_state = "turn left";
         
    // Set turning speeds (to the left) in rad/s 
    // These values were determined by trial and error.
    double turning_speed_wf_fast = 3.0;  // Fast turn
    double turning_speed_wf_slow = 0.05; // Slow turn
 
    // Wall following distance threshold.
    // We want to try to keep within this distance from the wall.
    double dist_thresh_wf = 0.50; // in meters  

    // We don't want to get too close to the wall though.
    double dist_too_close_to_wall = 0.19; // in meters    
};

int main(int argc, char * argv[])
{
  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<RobotController>();

  // Run node until it's exited
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}