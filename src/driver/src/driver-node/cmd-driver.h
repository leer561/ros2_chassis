#ifndef CMDDRIVER_H
#define CMDDRIVER_H

// ROS
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <QObject>

class CmdDriver : public QObject, public rclcpp::Node
{
    Q_OBJECT
  public:
    CmdDriver();
    ~CmdDriver();
  signals:
    void write(const QByteArray &);

  private:
    double linearTemp = 0;
    double angularTemp = 0; //暂存的线速度和角速度

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

#endif // CmdDriver