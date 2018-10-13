// 串口文件
#include "./pub-driver.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"

//包含 tf 以及 nav_msgs 相关的头文件
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <vector>
#include <QByteArray>
#include <QDebug>

#include <cmath>

enum Endian
{
    LittileEndian,
    BigEndian
};

int byteAraryToInt(QByteArray arr, Endian endian = BigEndian)
{
    if (arr.size() < 4)
        return 0;

    int res = 0;

    // 小端模式
    if (endian == LittileEndian)
    {
        res = arr.at(0) & 0x000000FF;
        res |= (arr.at(1) << 8) & 0x0000FF00;
        res |= (arr.at(2) << 16) & 0x00FF0000;
        res |= (arr.at(3) << 24) & 0xFF000000;
    }

    // 大端模式
    else if (endian == BigEndian)
    {
        res = (arr.at(0) << 24) & 0xFF000000;
        res |= (arr.at(1) << 16) & 0x00FF0000;
        res |= arr.at(2) << 8 & 0x0000FF00;
        res |= arr.at(3) & 0x000000FF;
    }
    return res;
}

PubDriver::PubDriver() : Node("pub_driver")
{
    // 创建发布
    publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom");
}

PubDriver::~PubDriver()
{
}

void PubDriver::init(tf2_ros::StaticTransformBroadcaster &broadcast)
{
    transformBroadcaster = &broadcast;
    emit start();
}

// 接收读取的数据
void PubDriver::getReadMsg(const QByteArray &data)
{
    // 读取判断数据 长度小于14不读
    if (data.size() < 19)
        return;

    // 编码器值
    QByteArray _lEncoder = data.mid(15, 4);
    QByteArray _rEncoder = data.mid(11, 4);
    int lEncoder = byteAraryToInt(_lEncoder);
    int rEncoder = byteAraryToInt(_rEncoder);
    qDebug() << "左编码器值" << lEncoder;
    qDebug() << "右编码器值" << rEncoder;

    // 解算里程
    double const PI = 3.14159265;
    double lMileage = PI * wheelDiameter * (lEncoder - lEncoderLast) / encoderTicks;
    double rMileage = PI * wheelDiameter * (rEncoder - rEncoderLast) / encoderTicks;
    // 赋值last编码器值
    lEncoderLast = lEncoder;
    rEncoderLast = rEncoder;

    // 航向
    double mileage = (lMileage + rMileage) / 2;
    double dtheta = (rMileage - lMileage) / baseWidth;
    qDebug() << "mileage： " << mileage;
    qDebug() << "dtheta： " << dtheta;
    if (mileage != 0)
    {
        double dx = std::cos(dtheta) * mileage;
        double dy = -(std::sin(dtheta) * mileage);
        x += (dx * std::cos(theta) - dy * std::sin(theta));
        y += (dx * std::sin(theta) + dy * std::cos(theta));
    }
    theta += dtheta;
    qDebug() << "theta： " << theta;
    rclcpp::Time currentTime = clock->now();
    //以下为了兼容三维系统下的消息结构，将里程计的偏航角转换成四元数
    //geometry_msgs::msg::Quaternion odomQuat = tf2::Quaternion(tf2Scalar(0), tf2Scalar(0), tf2Scalar(theta));
    tf2::Quaternion odomQ = tf2::Quaternion();
    odomQ.setRPY(0, 0, tf2Scalar(theta));
    geometry_msgs::msg::Quaternion odomQuat;
    odomQuat.x = odomQ.x();
    odomQuat.y = odomQ.y();
    odomQuat.z = odomQ.z();
    odomQuat.w = odomQ.w();

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
    odom_trans.transform.rotation = odomQuat;

    //发送变换
    //send the transform
    transformBroadcaster->sendTransform(odom_trans);
    //填充时间戳，发布nav_msgs/Odometry 里程计消息
    //以便于导航包可以获取速度信息
    //还需设置时间戳以及父子参考坐标系

    double dt = currentTime.nanoseconds() - lastTime.nanoseconds();
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
    odom.pose.pose.orientation.x = odomQ.x();
    odom.pose.pose.orientation.y = odomQ.y();
    odom.pose.pose.orientation.z = odomQ.z();
    odom.pose.pose.orientation.w = odomQ.w();

    for (unsigned int i = 0; i < odom.pose.covariance.size(); ++i)
    {
        odom.pose.covariance[i] = 0.0;
    }
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;

    //set the velocity
    odom.twist.twist.linear.x = mileage * 1000 * 1000 * 1000 / dt;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = dtheta * 1000 * 1000 * 1000 / dt;
    //publish the message
    lastTime = currentTime;
    publisher->publish(odom);
}