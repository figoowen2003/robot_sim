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
        auto defaultQos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        // This node subscribes to messages of type Float64MultiArray
        // over a topic named: /demo/state_est
        // The message represents the current estimated state:
        //      [x, y, yaw]
        estStateSub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/demo/state_est", defaultQos,
            std::bind(&RobotController::OnStateEstimatedMsg, this, _1));

        // Subscribe to sensor messages
        laserSub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/demo/laser/out", defaultQos,
            std::bind(&RobotController::OnSensorMsg, this, _1));

        // This node publishes the desired linear and angular velocity of the 
        // robot(in the robot chassis coordinate frame) to the /demo/cmd_vel topic
        // Using the diff_drive plugin enables the robot model to read this
        // /demo/cmd_vel topic and excute the motion accordingly
        cmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/demo/cmd_vel", defaultQos);        
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
        leftDist_ = msg->ranges[180];
        leftFrontDist_ = msg->ranges[135];
        frontDist_ = msg->ranges[90];
        rightFrontDist_ = msg->ranges[45];
        rightDist_ = msg->ranges[0];
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
        double d = distThreshWf_;
        if (leftFrontDist_ > d && frontDist_ > d && rightFrontDist_ > d) {
            wallFollowingState_ = "search for wall";
            cmdMsg->linear.x = forwardSpeed_;
            // cout << "here is the 1 branch" << endl;
        } else if (leftFrontDist_ > d && frontDist_ < d && rightFrontDist_ > d) {
            wallFollowingState_ = "turn left";
            cmdMsg->angular.z = turningSpeedWfSlow_;
            // cout << "here is the 2 branch" << endl;
        } else if (leftFrontDist_ > d && frontDist_ > d && rightFrontDist_ < d) {
            cmdMsg->angular.z = turningSpeedWfFast_;
            // cout << "here is the 3 branch" << endl;
        } else if (leftFrontDist_ < d && frontDist_ > d && rightFrontDist_ > d) {
            cmdMsg->angular.z = -turningSpeedWfFast_; // turn right to find wall
            // cout << "here is the 4 branch" << endl;
        } else if (leftFrontDist_ > d && frontDist_ < d && rightFrontDist_ < d) {
            wallFollowingState_ = "turn left";
            cmdMsg->angular.z = turningSpeedWfFast_;
            // cout << "here is the 5 branch" << endl;
        } else if (leftFrontDist_ < d && frontDist_ < d && rightFrontDist_ > d) {
            wallFollowingState_ = "turn right";
            cmdMsg->angular.z = -turningSpeedWfFast_;
            // cout << "here is the 6 branch" << endl;
        } else if (leftFrontDist_ < d && frontDist_ < d && rightFrontDist_ < d) {
            wallFollowingState_ = "slow down";
            cmdMsg->linear.x = -forwardSpeed_;
            // cout << "here is the 7 branch" << endl;
        } else if (leftFrontDist_ < d && frontDist_ > d && rightFrontDist_ < d) {
            wallFollowingState_ = "search for wall";
            cmdMsg->linear.x = forwardSpeed_;
            // cout << "here is the 8 branch" << endl;
        } else if (leftFrontDist_ > d && leftDist_ < d && rightDist_ >d) {
            wallFollowingState_ = "turn right";
            cmdMsg->angular.z = -turningSpeedWfFast_;
            // cout << "here is the 9 branch" << endl;
        } else if (rightFrontDist_ > d && leftDist_ > d && rightDist_ < d) {
            cmdMsg->angular.z = turningSpeedWfFast_;
            // cout << "here is the 10 branch" << endl;
        } else {
            // do nothing
        }
    
        // Send velocity command to the robot
        cmdPub_->publish(std::move(cmdMsg));    
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr estStateSub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserSub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;

    // Initialize the LaserScan sensor readings to some large value
    // Values are in meters.
    double leftDist_ = 999999.9; // Left
    double leftFrontDist_ = 999999.9; // Left-front
    double frontDist_ = 999999.9; // Front
    double rightFrontDist_ = 999999.9; // Right-front
    double rightDist_ = 999999.9; // Right
 
    /* ################### ROBOT CONTROL PARAMETERS ################## */
    // Maximum forward speed of the robot in meters per second
    // Any faster than this and the robot risks falling over.
    double forwardSpeed_ = 0.01;    

    // Current position and orientation of the robot in the global reference frame
    double currentX_{0.0};
    double currentY_{0.0};
    double currentYaw_{0.0};

    /* ############# WALL FOLLOWING PARAMETERS ####################### */     
    // Finite states for the wall following mode
    //  "turn left": Robot turns towards the left
    //  "search for wall": Robot tries to locate the wall        
    //  "follow wall": Robot moves parallel to the wall
    string wallFollowingState_ = "turn left";
         
    // Set turning speeds (to the left) in rad/s 
    // These values were determined by trial and error.
    double turningSpeedWfFast_ = 0.05;  // Fast turn
    double turningSpeedWfSlow_ = 0.025; // Slow turn
 
    // Wall following distance threshold.
    // We want to try to keep within this distance from the wall.
    double distThreshWf_ = 0.50; // in meters  

    // We don't want to get too close to the wall though.
    double distTooCloseToWall_ = 0.1; // in meters    
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