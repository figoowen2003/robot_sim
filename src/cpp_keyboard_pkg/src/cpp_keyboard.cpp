// Copyright 2021 Mu Wenfeng
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <utility>

#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#include <map>
#include <tuple>

class KeyboardController : public rclcpp::Node
{
public:
    ///KeyboardController node, which subscribes to laser scan messages and publishes
    /// velocity commands.
    KeyboardController()
    : Node("KeyboardController")
    {
        auto defaultQos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        // Advertise velocity commands
        cmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", defaultQos); 

        PubVelFromKeyboard();
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;

private:
    int GetCh(void)
    {
        int ch;
        struct termios oldt;
        struct termios newt;

        // Store old settings, and copy to new settings
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;

        // Make required changes and apply the settings
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_iflag |= IGNBRK;
        newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
        newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        tcsetattr(fileno(stdin), TCSANOW, &newt);

        // Get the current character
        ch = getchar();

        // Reapply old settings
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

        return ch;
    }

    void PubVelFromKeyboard() {
        // auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();        

        while(true){
            // Get the pressed key_
            key_ = GetCh();
            if (moveMap_.count(key_) == 1) {
                auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>(); 
                cmd_msg->linear.x = std::get<0>(moveMap_[key_]);
                cmd_msg->angular.z = std::get<1>(moveMap_[key_]);
                printf("\rCurrent: x %f\t %f | Last command: %c   ", cmd_msg->linear.x, cmd_msg->angular.z, key_);
                cmdPub_->publish(std::move(cmd_msg));                
            } else if (key_ == '\x03') {
                break;
            } else {
                printf("\rInvalid command! %c", key_);
            }
        }

        auto cmd_msg = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_msg->linear.x = 0;
        cmd_msg->linear.y = 0;
        cmd_msg->linear.z = 0;
        cmd_msg->angular.x = 0;
        cmd_msg->angular.y = 0;
        cmd_msg->angular.z = 0;

        cmdPub_->publish(std::move(cmd_msg));  
    }

    // Map for movement keys
    std::map<char, std::tuple<double, double>> moveMap_ {
        {'i', std::make_tuple(0.01, 0.0)},
        {'k', std::make_tuple(-0.01, 0.0)},
        {'j', std::make_tuple(0.0, 0.01)},
        {'l', std::make_tuple(0.0, -0.01)},
        {'s', std::make_tuple(0.0, 0.0)}        
    };   
    char key_{' '};               
};

int main(int argc, char * argv[])
{
    // Forward command line arguments to ROS
    rclcpp::init(argc, argv);

    // Create a node
    auto node = std::make_shared<KeyboardController>(); 

    // Run node until it's exited
    rclcpp::spin(node);

    // Clean up
    rclcpp::shutdown();
    return 0;
}