//以下为串口通讯需要的头文件
#include <QCoreApplication>

// 以下为ros node头文件
#include "driver-node/driver-node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"

// 智能指针
#include <memory>

int main(int argc, char *argv[])

{
    QCoreApplication a(argc, argv);
    rclcpp::init(argc, argv);
    // Create a node.
    auto node = std::make_shared<DriverNode>();

    // 多态供tf2_ros构造初始化
    rclcpp::Node::SharedPtr driverNode;
    driverNode = node;

    tf2_ros::StaticTransformBroadcaster transformBroadcaster(driverNode);
    node->init(transformBroadcaster);

    // spin will block until work comes in, execute work as it becomes
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);
    rclcpp::shutdown();

    return a.exec();
}