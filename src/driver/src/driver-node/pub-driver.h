#ifndef PUBDRIVER_H
#define PUBDRIVER_H

// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"

// transform broadcaster
#include "tf2_ros/static_transform_broadcaster.h"

// ecl
#include <ecl/geometry/legacy_pose2d.hpp>
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
  double linearTemp = 0;
  double angularTemp = 0;      //暂存的线速度和角速度
  int lEncoderLast = 0;        //左轮编码器
  int rEncoderLast = 0;        //右轮编码器
  double wheelDiameter = 0.08; //轮子直径
  double baseWidth = 0.25;     //两轮轮距
  int encoderTicks = 3300;     //一圈编码器值
  double x = 0;
  double y = 0;
  double theta = 0;
  double vx = 0;  // x 方向 0.1m/s
  double vy = 0;  //  y 方向 -0.1m/s
  double vth = 0; // 角速度为0.1rad/s

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher;
  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::Time lastTime = clock->now();
  tf2_ros::StaticTransformBroadcaster *transformBroadcaster = nullptr; // 广播成员
  ecl::LegacyPose2D<double> pose;
};

#endif // PubDriver