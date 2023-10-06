#include "dropper/dropper.h"

Dropper::Dropper(const std::string &port, ros::NodeHandle &nh, int baud_rate)
    : SerialBase(port, baud_rate), nh_(nh), is_pose_sub_(false), is_beep_sub_(false), is_takeoff_sub_(false), is_drop_sub_(false)
{
    if (this->open())
    {
        takeoff_pub_ = nh_.advertise<std_msgs::String>("dropper/takeoff", 1);
        drop_sub_ = nh_.subscribe<std_msgs::String>("dropper", 10, &Dropper::subDropCallback, this);
        beep_sub_ = nh_.subscribe<std_msgs::String>("dropper/beep", 10, &Dropper::subBeepCallback, this);
        takeoff_sub_ = nh_.subscribe<std_msgs::String>("dropper/takeoff", 10, &Dropper::subTakeoffCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10, &Dropper::subPoseCallback, this);
        this->pickAll();
    }
}

Dropper::~Dropper() {}

void Dropper::loop()
{
    if (is_drop_sub_)
    {
        if (drop_msg == std::string("pick all"))
        {
            this->pickAll();
        }
        else if (drop_msg == std::string("drop all"))
        {
            this->dropAll();
        }
        else if (drop_msg == std::string("drop 1"))
        {
            this->drop(1);
        }
        else if (drop_msg == std::string("drop 2"))
        {
            this->drop(2);
        }
        else if (drop_msg == std::string("drop 3"))
        {
            this->drop(3);
        }
        is_drop_sub_ = false;
        return;
    }

    if(is_takeoff_sub_)
    {
        this->write("d");
        is_takeoff_sub_ = false;
        return;
    }

    if(is_beep_sub_)
    {
        this->write("p");
        is_beep_sub_ = false;
        return;
    }

    if(is_pose_sub_)
    {
        this->write("c");
        is_pose_sub_ = false;
        return;
    }

}

void Dropper::dropAll()
{
    this->write("a");
}

void Dropper::pickAll()
{
    this->write("b");
}

void Dropper::drop(int num)
{
    switch (num)
    {
    case 1:
        this->write("1");
        break;

    case 2:
        this->write("2");
        break;

    case 3:
        this->write("3");
        break;

    default:
        break;
    }
}

void Dropper::subDropCallback(const std_msgs::String::ConstPtr &msg)
{
    drop_msg = msg->data;
    this->is_drop_sub_ = true;
}

void Dropper::subPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    this->is_pose_sub_ = true;
}

void Dropper::subBeepCallback(const std_msgs::String::ConstPtr &msg)
{
    this->is_beep_sub_ = true;
}

void Dropper::subTakeoffCallback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data == std::string("ok"))
    {
        this->is_takeoff_sub_ = true;
    }
}

void Dropper::takeoffReader(int timeout_ms)
{
    std::vector<uint8_t> buffer;
    if (!read(buffer, 3, timeout_ms))
    {
        // ROS_WARN("Serial read timeout or read error.");
        return;
    }
    // 检查返回值是否符合预期
    if (buffer.size() != 3 || buffer[0] != 't')
    {
        ROS_WARN("Received unexpected response");
        return;
    }
    std_msgs::String msg;

    switch (buffer[1])
    {
    case '0':
        msg.data = std::string("takeoff normal");
        break;
    case '1':
        msg.data = std::string("takeoff fallback");
        break;
    case '2':
        msg.data = std::string("takeoff 1");
        break;
    case '3':
        msg.data = std::string("takeoff 2");
        break;
    default:
        ROS_WARN("Received unexpected response");
        return;
        break;
    }
    takeoff_pub_.publish(msg);
}