// 串口文件
#include "./pub-driver.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "rcutils/time.h"

//包含 tf 以及 nav_msgs 相关的头文件
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

// ecl packages
#include <ecl/geometry/legacy_pose2d.hpp>
#include <ecl/linear_algebra.hpp>

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

    if (mileage == 0)
        return;

    // time
    rclcpp::Time currentTime = clock->now();

    // use ecl pose 2d
    ecl::LegacyPose2D<double> pose_update;
    ecl::linear_algebra::Vector3d pose_update_rates;

    pose_update.translation(mileage, 0);
    pose_update.rotation(dtheta);

    double last_diff_time = (currentTime.nanoseconds() - lastTime.nanoseconds()) / 1000;
    if (last_diff_time < 20)
        return;
    double tx = pose_update.x();
    double ty = pose_update.y();
    double th = pose_update.heading();
    tx = tx * 1000 * 1000 / last_diff_time;
    ty = ty * 1000 * 1000 / last_diff_time;
    th = th * 1000 * 1000 / last_diff_time;
    qDebug() << "last_diff_time " << last_diff_time;
    pose_update_rates << tx, ty, th;

    pose *= pose_update;

    // publish the odometry message over ROS
    rcutils_time_point_value_t now;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp.sec = RCL_NS_TO_S(now);
    odom.header.stamp.nanosec = now - RCL_S_TO_NS(odom.header.stamp.sec);
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose.x();
    odom.pose.pose.position.y = pose.y();
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.heading());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    for (unsigned int i = 0; i < odom.pose.covariance.size(); ++i)
    {
        odom.pose.covariance[i] = 0.0;
    }
    // Pose covariance (required by robot_pose_ekf) TODO: publish realistic values
    odom.pose.covariance[0] = 0.1;
    odom.pose.covariance[7] = 0.1;
    odom.twist.twist.linear.x = pose_update_rates[0];
    odom.twist.twist.linear.y = pose_update_rates[1];
    odom.twist.twist.linear.z = 0.0;
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = pose_update_rates[2];

    //publish the message
    lastTime = currentTime;
    publisher->publish(odom);

    //TransformStamped 类型为tf 发布时需要的类型
    geometry_msgs::msg::TransformStamped odom_trans;
    //时间戳
    odom_trans.header.stamp = odom.header.stamp;
    //父参考坐标系 id
    odom_trans.header.frame_id = "odom";
    //子参考系 id
    odom_trans.child_frame_id = "base_link";
    //我们希望发布从odom到base_link的变换，因此这两个坐标系的关系不要搞错
    odom_trans.transform.translation.x = pose.x();
    odom_trans.transform.translation.y = pose.y();
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    transformBroadcaster->sendTransform(odom_trans);
}