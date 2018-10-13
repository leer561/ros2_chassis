// 串口文件
#include "./cmd-driver.h"
#include "util.cpp"

#include "rclcpp/rclcpp.hpp"

//包含 tf 以及 nav_msgs 相关的头文件
#include "geometry_msgs/msg/twist.hpp"

#include <QByteArray>
#include <QDebug>

CmdDriver::CmdDriver() : Node("cmd_driver")
{
    // 订阅并处理cmd_vel msg
    auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        // 获取角速度,rad/s 线速度 m/s
        float angularTemp = msg->angular.z;
        float linearTemp = msg->linear.x;

        // 将转换好的小车速度分量为左右轮速度
        float delta_t = linearTemp > 0 ? 5.9 : 4.45;
        float RightWheelV = linearTemp + angularTemp / delta_t / 2;
        float LeftWheelV = linearTemp - angularTemp / delta_t / 2;

        int leftSpeed = driverNodeUtil::GetCoef(LeftWheelV);
        int rightSpeed = driverNodeUtil::GetCoef(RightWheelV);

        // 获取setSpeed 协议
        QByteArray speedData;
        speedData.resize(7);
        speedData[0] = 0xea;
        speedData[1] = 0x05;
        speedData[2] = 0x7e;
        speedData[3] = leftSpeed;
        speedData[4] = rightSpeed;
        // 异或检验
        speedData[5] = (((speedData[0] ^ speedData[2]) ^ speedData[3]) ^ speedData[4]) ^ speedData[6];
        speedData[6] = 0x0d;

        // 串口写入信息
        qDebug() << "cmd speedData" << speedData;
        emit write(speedData);
    };

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);
}

CmdDriver::~CmdDriver()
{
}