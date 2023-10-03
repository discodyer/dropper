#ifndef DROPPER_H
#define DROPPER_H

#include "ros/ros.h"
#include <string>
#include <libserial/SerialStream.h> // 请确保已安装 serial 库
#include <libserial/SerialPort.h>
#include <std_msgs/String.h>
#include "dropper/ansi_color.h"
#include "dropper/serial_base.h"

class Dropper : public SerialBase
{
public:
    Dropper(const std::string &port, ros::NodeHandle &nh, int baud_rate = 115200);
    ~Dropper();
    void dropAll();
    void pickAll();
    void drop(int num);
private:
    ros::NodeHandle nh_;         // ROS 节点句柄
    ros::Subscriber drop_sub_;  // 订阅器 - 投掷命令
    void subDropCallback(const std_msgs::String::ConstPtr &msg);
};

#endif // DROPPER_H
