#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
// #include <urdf/model.h>

// #include <rosparam_shortcuts/rosparam_shortcuts.h>
// #include <moveit_ros_control_interface/ControllerHandle.h>

#include "ck_ros_msgs_node/Arm_Status.h"
#include "ck_ros_msgs_node/Arm_Control.h"
#include <std_msgs/Float64.h>

#include <ck_utilities/CKMath.hpp>

#include <ros/ros.h>
#include <string>
#include <iostream>
#include <math.h>

class KnighttimeRobotHW : public hardware_interface::RobotHW
{
public:
    KnighttimeRobotHW(const ros::NodeHandle& nh);

    void init();
    void arm_status_cb(const ck_ros_msgs_node::Arm_Status& status);

    void read(const ros::Time& time, const ros::Duration& period);

    void write(const ros::Time& time, const ros::Duration& period);

private:
    const ros::NodeHandle& nh_;
    const std::string name_;

    ros::Subscriber arm_status_sub;
    ros::Publisher arm_control_pub;

    ck_ros_msgs_node::Arm_Status arm_status;

    // hardware_interface::JointStateInterface jnt_state_interface;
    // hardware_interface::PositionJointInterface jnt_pos_interface;
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;
    hardware_interface::EffortJointInterface effort_joint_interface_;

    // double cmd[6];
    // double pos[6];
    // double vel[6];
    // double eff[6];
    std::vector<std::string> joint_names_;
    std::size_t num_joints_;
    // urdf::Model* urdf_model_;

    // Modes
    bool use_rosparam_joint_limits_;
    bool use_soft_limits_if_available_;

    // States
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;

    // Commands
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocity_command_;
    std::vector<double> joint_effort_command_;
};