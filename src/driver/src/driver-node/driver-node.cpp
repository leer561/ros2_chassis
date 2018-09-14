// 串口文件
#include "driver-node.h"
#include "../serial-port/serial-port.h"
#include "util.cpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <vector>
#include <iostream>

#include <QThread>
#include <QByteArray>
#include <QDebug>

DriverNode::DriverNode() : Node("base_driver")
{
    auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
        // 获取角速度,rad/s 线速度 m/s
        float angularTemp = msg->angular.z;
        float linearTemp = msg->linear.x;

        // 将转换好的小车速度分量为左右轮速度
        int leftSpeed = driverNodeUtil::GetCoef(linearTemp - 0.5f * angularTemp * D);
        int rightSpeed = driverNodeUtil::GetCoef(linearTemp + 0.5f * angularTemp * D);

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

    sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);
}

DriverNode::~DriverNode()
{
    workerThread.quit();
    workerThread.wait();
}

void DriverNode::init()
{
    SerialPort *port = new SerialPort;
    port->moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, port, &QObject::deleteLater);
    connect(this, &DriverNode::write, port, &SerialPort::write);
    connect(this, &DriverNode::start, port, &SerialPort::init);
    connect(port, &SerialPort::sendReadMsg, this, &DriverNode::getReadMsg);
    workerThread.start();
    emit start();
}

// 接收读取的数据
void DriverNode::getReadMsg(const QByteArray &data)
{
    // 读取判断数据 长度小于14不读
    if (data.size() < 14)
        return;
    // 转换为 unsigned char
    unsigned char *buffer = (unsigned char *)data.constData();
    std::vector<unsigned char> bufferToCompress(data.begin(), data.end());

    // 判断头部 不是0xEA 即234 返回
    if (data[0] != 234)
        return;
    qDebug() << "getReadMsg 2： " << data[2];
    qDebug() << "编码器 data11： " << data[11];
    qDebug() << "编码器 data12： " << data[12];
}