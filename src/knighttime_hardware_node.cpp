#include <ros/ros.h>

// #include "hardware_interface.hpp"
#include "hardware_interface.hpp"
#include <controller_manager/controller_manager.h>

ros::NodeHandle* node;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar3_hardware");

    ros::NodeHandle n;
    node = &n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    AR3_Robot robot(n);
    robot.init();
    // hardware_interface::RobotHW robot;
    controller_manager::ControllerManager cm(&robot, n);

    ros::Time prev_time = ros::Time::now();
    ros::Duration period = ros::Time::now() - prev_time;

    ros::Rate rate(50);

    while (ros::ok())
    {
        ros::Time time = ros::Time::now();
        period = time - prev_time;
        prev_time = time;

        robot.read(time, period);
        cm.update(time, period);
        robot.write(time, period);
        // ROS_WARN("Joint 1: %f", robot.get_j1_cmd());
        // ROS_WARN("Test");

        rate.sleep();
    }

    return 0;
}