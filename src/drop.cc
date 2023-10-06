#include "ros/ros.h"
#include "dropper/serial_base.h"
#include "dropper/dropper.h"
#include "dropper/ansi_color.h"
#include <libserial/SerialStream.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dropper_node");
    ros::NodeHandle nh;

    std::string serial_port_name = "/dev/ttyS4";
    int serial_baud_rate = 115200;

    // Read parameters from launch file, including: serial_port_name, serial_baud_rate
    {
        if (nh.getParam("/dropper_node/serial_port_name", serial_port_name))
        {
            ROS_INFO("Get serial_port_name parameter: %s", serial_port_name.c_str());
        }
        else
        {
            ROS_WARN("Using default serial_port_name: %s", serial_port_name.c_str());
        }

        if (nh.getParam("/dropper_node/serial_baud_rate", serial_baud_rate))
        {
            ROS_INFO("Get serial_baud_rate parameter: %d", serial_baud_rate);
        }
        else
        {
            ROS_WARN("Using default serial_baud_rate: %d", serial_baud_rate);
        }
    }

    Dropper dropper(serial_port_name, nh, serial_baud_rate);

    ros::Rate rate(10);

    while (ros::ok())
    {
        dropper.takeoffReader(50);
        dropper.loop();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}