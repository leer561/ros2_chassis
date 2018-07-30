#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

//以下为串口通讯需要的头文件
#define ASIO_STANDALONE
#include "asio.hpp"

using namespace asio; //定义一个命名空间，用于后面的读写操作
using namespace std;

unsigned char buf[2];

//转速转换比例，执行速度调整比例
float get_coef(float item)
{
    if (item > 255)
    {
        return 255;
    }
    else if (item < 0)
    {
        return 0;
    }
    else
    {
        return item;
    }
}
float D = 0.39f;                         //两轮间距，单位是m
float linear_temp = 0, angular_temp = 0; //暂存的线速度和角速度

class BaseDriver : public rclcpp::Node
{
  public:
    BaseDriver() : Node("base_driver")
    {
        auto callback = [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void {
            //获取角速度,rad/s 线速度 m/s
            angular_temp = get_coef(msg->angular.z);
            linear_temp = get_coef(msg->linear.x);

            //将转换好的小车速度分量为左右轮速度
            float left_speed = linear_temp - 0.5f * angular_temp * D;
            float right_speed = linear_temp + 0.5f * angular_temp * D;

            int vl = (int)left_speed;
            int vr = (int)right_speed;

            //组合协议
            int speed_data[7] = {0xea, 0x05, 0x7e, vl, vr, 0x00, 0x0d};

            //写入数据到串口
            //设置串口
            io_service iosev;
            serial_port sp(iosev, "/dev/motor_trd"); //定义传输的串口
            sp.set_option(serial_port::baud_rate(38400));
            sp.set_option(serial_port::flow_control());
            sp.set_option(serial_port::parity());
            sp.set_option(serial_port::stop_bits());
            sp.set_option(serial_port::character_size(8));
            write(sp, buffer(speed_data, 8));

            RCLCPP_INFO(this->get_logger(), "I heard: [%d]", angular_temp);
            RCLCPP_INFO(this->get_logger(), "speed_data vl [%d]", vl);
            RCLCPP_INFO(this->get_logger(), "speed_data vr [%d]", vr);
        };

        sub_ = create_subscription<geometry_msgs::msg::Twist>("cmd_vel", callback, rmw_qos_profile_sensor_data);
    }

  private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char *argv[])
{
    printf("test the node");
    //设置串口
    io_service iosev;
    serial_port sp(iosev, "/dev/motor_trd"); //定义传输的串口
    sp.set_option(serial_port::baud_rate(38400));
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    rclcpp::init(argc, argv);

    // Create a node.
    auto node = std::make_shared<BaseDriver>();

    // spin will block until work comes in, execute work as it becomes available, and keep blocking.
    // It will only be interrupted by Ctrl-C.
    rclcpp::spin(node);
    rclcpp::shutdown();

    while (rclcpp::ok())
    {
        read(sp, buffer(buf));
        string str(&buf[0], &buf[2]); //将数组转化为字符串
        printf("Read data %s", str.c_str());
    }

    iosev.run();

    return 0;
}