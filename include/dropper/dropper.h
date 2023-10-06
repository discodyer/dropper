#ifndef DROPPER_H
#define DROPPER_H

#include "ros/ros.h"
#include <string>
#include <libserial/SerialStream.h> // 请确保已安装 serial 库
#include <libserial/SerialPort.h>
#include <std_msgs/String.h>
#include "dropper/ansi_color.h"
#include "dropper/serial_base.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

class Dropper : public SerialBase
{
public:
    Dropper(const std::string &port, ros::NodeHandle &nh, int baud_rate = 115200);
    ~Dropper();
    void dropAll();
    void pickAll();
    void drop(int num);
    void takeoffReader(int timeout_ms = 100);
    void loop();

private:
    ros::NodeHandle nh_;          // ROS 节点句柄
    ros::Publisher takeoff_pub_;  // 起飞指令发布器
    ros::Subscriber drop_sub_;    // 订阅器 - 投掷命令
    ros::Subscriber pose_sub_;    // 订阅器 - 心跳
    ros::Subscriber beep_sub_;    // 订阅器 - 蜂鸣器
    ros::Subscriber takeoff_sub_; // 订阅器 - 起飞指示
    std::string drop_msg;
    bool is_pose_sub_;
    bool is_beep_sub_;
    bool is_takeoff_sub_;
    bool is_drop_sub_;
    void subDropCallback(const std_msgs::String::ConstPtr &msg);
    void subTakeoffCallback(const std_msgs::String::ConstPtr &msg);
    void subPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void subBeepCallback(const std_msgs::String::ConstPtr &msg);
};

#endif // DROPPER_H
