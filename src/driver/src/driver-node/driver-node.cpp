// 串口文件
#include "driver-node.h"
#include "../serial-port/serial-port.h"
#include "util.cpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "std_msgs/msg/string.hpp"

//包含 tf 以及 nav_msgs 相关的头文件
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>
#include <QThread>
#include <QByteArray>
#include <QDebug>

#include <cmath>
#include <memory>

DriverNode::DriverNode() : Node("base_driver")
{
    // 订阅并处理cmd_vel msg
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

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);

    // 创建发布
    publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom");
}

DriverNode::~DriverNode()
{
    workerThread.quit();
    workerThread.wait();
}

void DriverNode::init(tf2_ros::StaticTransformBroadcaster &broadcast)
{
    SerialPort *port = new SerialPort;
    port->moveToThread(&workerThread);
    connect(&workerThread, &QThread::finished, port, &QObject::deleteLater);
    connect(this, &DriverNode::write, port, &SerialPort::write);
    connect(this, &DriverNode::start, port, &SerialPort::init);
    connect(port, &SerialPort::sendReadMsg, this, &DriverNode::getReadMsg);
    workerThread.start();
    emit start();

    // 初始化TF广播
    transformBroadcaster = &broadcast;
}

// 接收读取的数据
void DriverNode::getReadMsg(const QByteArray &data)
{
    // 读取判断数据 长度小于14不读
    if (data.size() < 19)
        return;

    // 判断头部 不是0xEA 即234 返回
    if (data[0] != 234)
        return;

    // 左侧编码器
    QByteArray _lEncoder = data.mid(11, 4);
    QByteArray _rEncoder = data.mid(15, 4);
    bool qToInt;
    int lEncoder = _lEncoder.toInt(&qToInt, 10);
    int rEncoder = _rEncoder.toInt(&qToInt, 10);

    // 转换出错提示
    if (!qToInt)
    {
        qDebug() << "编码器值转换出错啦！！！";
    }

    // 解算里程
    double const PI = 3.14159265;
    double lMileage = PI * wheelDiameter * (lEncoder - lEncoderLast) / encoderTicks;
    double rMileage = PI * wheelDiameter * (rEncoder - rEncoderLast) / encoderTicks;

    // 赋值last编码器值
    lEncoderLast = lEncoder;
    rEncoderLast = rEncoder;

    // 航向
    double mileage = (lMileage + rMileage) / 2;
    double dtheta = (rMileage - lMileage) / D;

    if (mileage != 0)
    {
        double dx = std::cos(dtheta) * mileage;
        double dy = std::sin(dtheta) * mileage;
        x += dx * std::cos(theta) - dy * std::sin(theta);
        y += dx * std::sin(theta) + dy * std::cos(theta);
    }
    theta += dtheta;

    rclcpp::Time currentTime = rclcpp::Time();
    //以下为了兼容三维系统下的消息结构，将里程计的偏航角转换成四元数
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    //geometry_msgs::msg::Quaternion odomQuat = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(theta));

    odomQ.setRPY(theta, 0, 0);
    //geometry_msgs::msg::Quaternion odomQuat = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(odomQ);
    //first, we'll publish the transform over tf
    //TransformStamped 类型为tf 发布时需要的类型
    geometry_msgs::msg::TransformStamped odom_trans;
    //时间戳
    odom_trans.header.stamp = currentTime;
    //父参考坐标系 id
    odom_trans.header.frame_id = "odom";
    //子参考系 id
    odom_trans.child_frame_id = "base_link";
    //我们希望发布从odom到base_link的变换，因此这两个坐标系的关系不要搞错

    //填充变换数据
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = odomQuat;

    //发送变换
    //send the transform
    transformBroadcaster->sendTransform(odom_trans);
    //填充时间戳，发布nav_msgs/Odometry 里程计消息
    //以便于导航包可以获取速度信息
    //还需设置时间戳以及父子参考坐标系

    double dt = (currentTime.nanoseconds() - lastTime.nanoseconds()) / (1000 * 1000 * 1000);
    //next, we'll publish the odometry message over ROS
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    //最后填充机器人的位置以及速度信息，
    //并且发布该信息，因为是机器人本体，
    //所以参考坐标系为 base_link
    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odomQuat;
    //set the velocity
    odom.twist.twist.linear.x = mileage / dt;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = dtheta / dt;
    //publish the message
    lastTime = currentTime;
    publisher->publish(odom);
}