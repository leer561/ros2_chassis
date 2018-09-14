#ifndef DRIVERNODE_H
#define DRIVERNODE_H

// 串口文件
#include "../serial-port/serial-port.h"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <QByteArray>
#include <QObject>
#include <QThread>

class DriverNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
    QThread workerThread;

  public:
    DriverNode();
    ~DriverNode();
    void init(); // 初始化串口

  public slots:
    void getReadMsg(const QByteArray &);

  signals:
    void write(const QByteArray &);
    void start();

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    float D = 0.39f; //两轮间距，单位是m
    float linearTemp = 0;
    float angularTemp = 0; //暂存的线速度和角速度
};

#endif // DriverNode