// 串口文件
#include "../serial-port/serial-controller.h"

#include "driver-node.h"
#include "util.cpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>

DriverNode::DriverNode(SerialController *myPort) : Node("base_driver")
{
    auto callback = [this, myPort](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        // 获取角速度,rad/s 线速度 m/s
        float angularTemp = msg->angular.z;
        float linearTemp = msg->linear.x;

        // 将转换好的小车速度分量为左右轮速度
        int leftSpeed = driverNodeUtil::GetCoef(linearTemp - 0.5f * angularTemp * D);
        int rightSpeed = driverNodeUtil::GetCoef(linearTemp + 0.5f * angularTemp * D);

        // 获取setSpeed 协议
        std::vector<int> speedData = {0xea, 0x05, 0x7e, 0x80, 0x80, 0x00, 0x0d};
        speedData[3] = leftSpeed;
        speedData[4] = rightSpeed;

        // 异或检验
        speedData[5] = (((speedData[0] ^ speedData[2]) ^ speedData[3]) ^ speedData[4]) ^ speedData[6];

        // 串口写入 信息
        myPort->SendMsgToPort(speedData);
    };

    sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);
}