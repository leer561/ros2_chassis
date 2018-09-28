// 以下为ros node头文件
#include "driver-node/pub-driver.h"
#include "driver-node/cmd-driver.h"
#include "serial-port/serial-port.h"

// ros
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/publisher.hpp"

#include <QCoreApplication>
#include <QThread>
#include <memory>

int main(int argc, char *argv[])

{
    rclcpp::init(argc, argv);
    QCoreApplication a(argc, argv);
    CmdDriver *subDriver = new CmdDriver();
    PubDriver *pubDriver = new PubDriver();

    // Createnode.
    std::shared_ptr<CmdDriver> subNode(subDriver);
    std::shared_ptr<PubDriver> pubNode(pubDriver);

    // 初始化串口管理类 并初始化另外的线程
    QThread workerThread;
    SerialPort *port = new SerialPort;
    port->moveToThread(&workerThread);
    QObject::connect(&workerThread, &QThread::finished, port, &QObject::deleteLater);
    QObject::connect(subDriver, &CmdDriver::write, port, &SerialPort::write);
    QObject::connect(pubDriver, &PubDriver::start, port, &SerialPort::init);
    QObject::connect(port, &SerialPort::sendReadMsg, pubDriver, &PubDriver::getReadMsg);
    workerThread.start();

    // 新开线程推送消息
    QThread pubThread;
    pubDriver->moveToThread(&pubThread);
    QObject::connect(&pubThread, &QThread::finished, port, &QObject::deleteLater);
    pubThread.start();

    // 多态供tf2_ros构造初始化以及add node
    rclcpp::Node::SharedPtr driverNode;
    driverNode = pubNode;
    tf2_ros::StaticTransformBroadcaster transformBroadcaster(driverNode);
    pubDriver->init(transformBroadcaster);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pubNode);
    exec.add_node(subNode);
    exec.spin();
    rclcpp::shutdown();

    return a.exec();
}