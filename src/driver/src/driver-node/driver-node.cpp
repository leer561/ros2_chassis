// 串口文件
#include "../serial-port/serial-port.h"

#include "driver-node.h"
#include "util.cpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

DriverNode::DriverNode(SerialPort *myPort) : Node("base_driver")
{
    auto callback = [this, myPort](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        //获取角速度,rad/s 线速度 m/s
        angular_temp = driverNodeUtil::GetCoef(msg->angular.z);
        linear_temp = driverNodeUtil::GetCoef(msg->linear.x);

        //将转换好的小车速度分量为左右轮速度
        float left_speed = linear_temp - 0.5f * angular_temp * D;
        float right_speed = linear_temp + 0.5f * angular_temp * D;

        int vl = (int)left_speed;
        int vr = (int)right_speed;

        //组合协议
        int speed_data[7] = {0xea, 0x05, 0x7e, vl, vr, 0x00, 0x0d};
        RCLCPP_INFO(this->get_logger(), "I heard: [%d]", angular_temp);
        RCLCPP_INFO(this->get_logger(), "speed_data vl [%d]", vl);

        RCLCPP_INFO(this->get_logger(), "speed_data vr [%d]", vr);

        // 串口写入 信息
        myPort->SendMsgToPort(speed_data, 7);
    };

    sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);
}