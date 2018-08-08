//以下为串口通讯需要的头文件
#include <QCoreApplication>
#include "serial-port/serial-port.h"

// 以下为ros node头文件
#include "driver-node/driver-node.h"
#include "rclcpp/rclcpp.hpp"

// 智能指针
#include <memory>

int main(int argc, char *argv[])

{
    rclcpp::init(argc, argv);
    QCoreApplication a(argc, argv);

    // 生成串口
    class SerialPort *myPort = new SerialPort();

    // Create a node.
    auto node = std::make_shared<DriverNode>(myPort);

    // spin will block until work comes in, execute work as it becomes
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);
    rclcpp::shutdown();

    return a.exec();
}