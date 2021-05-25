#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include<bits/stdc++.h>
#include<tuple>

using namespace std;
using std::placeholders::_1;

class RobotEstimator: public rclcpp::Node
{
public:
    RobotEstimator() : Node("Estimator") {
        auto defaultQos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
        // Subscribe to messages of type nav_msgs/Odometry(positon and orientation of robot)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/demo/odom", defaultQos,
            std::bind(&RobotEstimator::OnOdomMsg, this, _1));

        // This node subscribes to messages of tpye geometry_msgs/Twist.msg
        // We are listening to the velocity commands here
        // The maximum number of queued messages is 10
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "demo/cmd_vel", defaultQos,
        std::bind(&RobotEstimator::OnVelocityMsg, this, _1));           

        // This node Pulbishes the estimated position (x, y, yaw),
        // the type of messages is std_msg/Float64MultiArray
        est_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/demo/state_est", defaultQos);

    }

private:
    void OnOdomMsg(const nav_msgs::msg::Odometry::SharedPtr msg) {
        tuple<float, float, float> result = GetEulerFromQuaternion(msg->pose.pose.orientation);
        float roll = get<0>(result);
        float pitch = get<1>(result);
        float yaw = get<2>(result);

        PublishEstimatedState(make_tuple(roll, pitch, yaw));
    }

    void PublishEstimatedState(const tuple<float, float, float> state) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.push_back(get<0>(state));
        msg.data.push_back(get<1>(state));
        msg.data.push_back(get<2>(state));

        est_state_pub_->publish(msg);      
    }

    tuple<float, float, float> GetEulerFromQuaternion(const geometry_msgs::msg::Quaternion &orientation) {
        double t0 = +2.0 * (orientation.w * orientation.x + orientation.y * orientation.z);
        double t1 = +1.0 - 2.0 * (orientation.x * orientation.x + orientation.y * orientation.y);
        double roll_x = atan2(t0, t1);
    
        double t2 = +2.0 * (orientation.w * orientation.y - orientation.z * orientation.x);
        t2 = (t2 > 1.0) ? 1.0 : t2;
        t2 = (t2 < -1.0) ? -1.0 : t2;
        double pitch_y = asin(t2);
    
        double t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
        double t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
        double yaw_z = atan2(t3, t4);
    
        return make_tuple(roll_x, pitch_y, yaw_z); // in radians       
    }

    void OnVelocityMsg(const geometry_msgs::msg::Twist::SharedPtr msg) {
        /*
            Listen to the velocity commands (linear forward velocity 
            in the x direction in the robot's reference frame and 
            angular velocity (yaw rate) around the robot's z-axis.
            [v,yaw_rate]
            [meters/second, radians/second]            
        */
        // Forward velocity in the robot's reference frame
        auto v = msg->linear.x;
    
        // Angular velocity around the robot's z axis
        auto yaw_rate = msg->angular.z;     
    }

    // odom messages subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // twist messages subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;

    // estimate state publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr est_state_pub_;   
};

int main(int argc, char * argv[])
{
  // Forward command line arguments to ROS
  rclcpp::init(argc, argv);

  // Create a node
  auto node = std::make_shared<RobotEstimator>();

  // Run node until it's exited
  rclcpp::spin(node);

  // Clean up
  rclcpp::shutdown();
  return 0;
}
