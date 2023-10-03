#include "dropper/dropper.h"

Dropper::Dropper(const std::string &port, ros::NodeHandle &nh, int baud_rate)
    : SerialBase(port, baud_rate), nh_(nh)
{
    if (this->open())
    {
        drop_sub_ = nh_.subscribe<std_msgs::String>("dropper", 10, &Dropper::subDropCallback, this);
        this->pickAll();
    }
}

Dropper::~Dropper() {}

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
    if (msg->data == std::string("pick all"))
    {
        this->pickAll();
    }
    else if (msg->data == std::string("drop all"))
    {
        this->dropAll();
    }
    else if (msg->data == std::string("drop 1"))
    {
        this->drop(1);
    }
    else if (msg->data == std::string("drop 2"))
    {
        this->drop(2);
    }
    else if (msg->data == std::string("drop 3"))
    {
        this->drop(3);
    }
}
