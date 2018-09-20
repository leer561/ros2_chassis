#ifndef DRIVERNODE_H
#define DRIVERNODE_H

// ROS
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

// transform broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

// 串口
#include <QByteArray>
#include <QObject>
#include <QThread>
#include <memory>

class DriverNode : public QObject
{
    Q_OBJECT
    QThread workerThread;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> publisher;

  public:
    DriverNode(rclcpp::Node::SharedPtr const &node, tf2_ros::StaticTransformBroadcaster &broadcast);
    ~DriverNode();
    void init(std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> const &p); // 初始化串口
  public slots:
    void getReadMsg(const QByteArray &);

  signals:
    void write(const QByteArray &);
    void start();

  private:
    double D = 0.39f; //两轮间距，单位是m
    double linearTemp = 0;
    double angularTemp = 0;       //暂存的线速度和角速度
    int lEncoderLast = 0;         //左轮编码器
    int rEncoderLast = 0;         //右轮编码器
    double wheelDiameter = 0.125; //右轮编码器
    int encoderTicks = 3300;      //一圈编码器值
    double x = 0;
    double y = 0;
    double theta = 0;
    double vx = 0;  // x 方向 0.1m/s
    double vy = 0;  //  y 方向 -0.1m/s
    double vth = 0; // 角速度为0.1rad/s

    rclcpp::Time lastTime = rclcpp::Time();
    tf2_ros::StaticTransformBroadcaster *transformBroadcaster = nullptr; // 广播成员
    tf2::Quaternion odomQ = tf2::Quaternion();                           // tf2 Quaternion
};

#endif // DriverNode