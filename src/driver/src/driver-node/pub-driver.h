#ifndef PUBDRIVER_H
#define PUBDRIVER_H

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"

// transform broadcaster
#include "tf2_ros/static_transform_broadcaster.h"

// 串口
#include <QByteArray>
#include <QObject>

class PubDriver : public QObject, public rclcpp::Node
{
    Q_OBJECT

  public:
    PubDriver();
    ~PubDriver();
    void init(tf2_ros::StaticTransformBroadcaster &broadcast); // 初始化串口
  public slots:
    void getReadMsg(const QByteArray &);

  signals:
    void start(); // 初始化串口

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

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
    rclcpp::Time lastTime = rclcpp::Time();
    tf2_ros::StaticTransformBroadcaster *transformBroadcaster = nullptr; // 广播成员
};

#endif // PubDriver