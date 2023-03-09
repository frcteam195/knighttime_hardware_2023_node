#include <ros/ros.h>

// #include "hardware_interface.hpp"
#include "knighttime_interface.hpp"
#include <controller_manager/controller_manager.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

ros::NodeHandle* node;

// static const std::string PLANNING_GROUP = "arm_group";
// static moveit::planning_interface::MoveGroupInterface *move_group;

void joint_states_cb(const sensor_msgs::JointState& state)
{
    // ROS_ERROR("Start CB");
    (void)state;
    // if (move_group == nullptr)
    // {
        // move_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    // }
    // ROS_ERROR("Finish CB");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar3_hardware");

    ros::NodeHandle n;
    node = &n;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    KnighttimeRobotHW robot(n);
    robot.init();
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
    static ros::Subscriber joint_state_sub = n.subscribe("/joint_states", 10, joint_states_cb, ros::TransportHints().tcpNoDelay());

    while (ros::ok())
    {
        ros::Time time = ros::Time::now();
        period = time - prev_time;
        prev_time = time;

        robot.read(time, period);
        cm.update(time, period);
        robot.write(time, period);

        if (time.toSec() - start_time.toSec() > 10.0)
        {
            // ROS_ERROR("Planning path!");
            // start_time = ros::Time::now();

            // std::vector<double> target = {-0.75, 0.75, 0.1};
            // move_group->setJointValueTarget(target);
            // std::vector<double> target2 = {0.0, 1.5, 0.0};
            // move_group->setJointValueTarget(target2);
            // moveit::core::RobotState start_state(move_group->getRobotModel());
            // start_state.setJointGroupPositions(PLANNING_GROUP, std::vector<double>{0.0, 0.75, 0.0});
            // move_group->setStartState(start_state);

            // moveit::planning_interface::MoveGroupInterface::Plan plan;
            // moveit::core::MoveItErrorCode error = move_group->plan(plan);
            // ROS_ERROR("Planning time: %f", plan.planning_time_);

            // for (auto joint_state : plan.start_state_.joint_state.position)
            // {
            //     ROS_ERROR("JOINT start state: %f", joint_state);
            // }

            // ROS_ERROR("TRAJ SIZE: %ld", plan.trajectory_.joint_trajectory.points.size());

            // for (size_t i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
            // {
            //     auto traj_point = plan.trajectory_.joint_trajectory.points[i];
            //     std::cout << traj_point.time_from_start << ",";
            //     std::cout << traj_point.positions[1] << ",";
            //     std::cout << traj_point.velocities[1] << ",";
            //     std::cout << traj_point.accelerations[1] << std::endl;
            // }
            // std::cout << "--------------------" << std::endl;

            // ROS_ERROR_STREAM("Error code: " << moveit::core::MoveItErrorCode::toString(error));
            // // bool success = (error == moveit::core::MoveItErrorCode::SUCCESS);
            // bool success = (bool)error;

            // ROS_ERROR_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
        }

        rate.sleep();
    }

    return 0;
}
