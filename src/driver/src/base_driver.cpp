//以下为串口通讯需要的头文件
#include <QCoreApplication>

// 以下为ros node头文件
#include "driver-node/driver-node.h"
#include "rclcpp/rclcpp.hpp"

// 智能指针
#include <memory>

int main(int argc, char *argv[])

{
    QCoreApplication a(argc, argv);

    rclcpp::init(argc, argv);
    // Create a node.
    auto node = std::make_shared<DriverNode>();
    node->init();

    // spin will block until work comes in, execute work as it becomes
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);
    rclcpp::shutdown();

    return a.exec();
}