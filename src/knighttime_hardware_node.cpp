#include <ros/ros.h>

// #include "hardware_interface.hpp"
#include "knighttime_interface.hpp"
#include <controller_manager/controller_manager.h>

#include <trajectory_msgs/JointTrajectory.h>

ros::NodeHandle* node;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar3_hardware");

    ros::NodeHandle n;
    node = &n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    KnighttimeRobotHW robot(n);
    robot.init();
    // hardware_interface::RobotHW robot;
    controller_manager::ControllerManager cm(&robot, n);

    ros::Time start_time = ros::Time::now();
    ros::Time prev_time = ros::Time::now();
    ros::Duration period = ros::Time::now() - prev_time;

    ros::Rate rate(100);

    trajectory_msgs::JointTrajectory traj;

    for (std::string joint_name : robot.get_joint_names())
    {
        traj.joint_names.push_back(joint_name);
    }

    traj.points.resize(2);

    traj.points[0].positions.resize(3);
    traj.points[0].positions[0] = -0.75;
    traj.points[0].positions[1] = 0.75;
    traj.points[0].positions[2] = 0.0;
    traj.points[0].velocities.resize(3);
    traj.points[0].velocities[0] = 0.0;
    traj.points[0].velocities[1] = 0.0;
    traj.points[0].velocities[2] = 0.0;
    traj.points[0].time_from_start = ros::Duration(0.5);

    traj.points[1].positions.resize(3);
    traj.points[1].positions[0] = 0.0;
    traj.points[1].positions[1] = 0.0;
    traj.points[1].positions[2] = 0.0;
    traj.points[1].velocities.resize(3);
    traj.points[1].velocities[0] = 0.0;
    traj.points[1].velocities[1] = 0.0;
    traj.points[1].velocities[2] = 0.0;
    traj.points[1].time_from_start = ros::Duration(1.0);

    // static ros::Publisher goal_pub = n.advertise<trajectory_msgs::JointTrajectory>("/velocity_trajectory_controller/command", 10);
    static ros::Publisher goal_pub = n.advertise<trajectory_msgs::JointTrajectory>("/position_trajectory_controller/command", 10);

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

        if (time.toSec() - start_time.toSec() > 10.0)
        {
            // ROS_ERROR("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
            traj.header.stamp = time;
            // goal_pub.publish(traj);
            start_time = ros::Time::now();
        }

        rate.sleep();
    }

    return 0;
}