#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("dumb_teleop");

    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "cmd_vel", rmw_qos_profile_sensor_data);

    rclcpp::WallRate loop_rate(20);
    int count = 0;
    const int switch_count = 80;

    auto msg = std::make_shared<geometry_msgs::msg::Twist>();
    msg->linear.x = 0.0;
    msg->angular.z = 0.5;

    while (rclcpp::ok())
    {
        std::cout << "Publishing: (" << msg->linear.x << ", " << msg->angular.z << ")" << std::endl;
        cmd_vel_pub->publish(msg);
        if (++count >= switch_count)
        {
            msg->angular.z *= -1.0;
            count = 0;
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    return 0;
}