#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>
// #include <moveit_ros_control_interface/ControllerHandle.h>

#include "ck_ros_msgs_node/Arm_Status.h"
#include "ck_ros_msgs_node/Arm_Control.h"

#include <string>
#include <iostream>
#include <std_msgs/Float64.h>
#include <math.h>

class AR3_Robot : public hardware_interface::RobotHW
{
public:
    AR3_Robot(const ros::NodeHandle& nh) : nh_(nh), name_("knighttime_hardware")
    {
        // std::string joint_names[6];

        // for (int i = 0; i < 6; i++) {
        //     joint_names[i] = "joint_" + std::to_string(i+1);
        // }

        // for (int i = 0; i < 6; i++)
        // {
        //     hardware_interface::JointStateHandle state_handle(joint_names[i], &pos[i], &vel[i], &eff[i]);
        //     jnt_state_interface.registerHandle(state_handle);
        // }


        // hardware_interface::JointStateHandle state_handle_1("joint_1", &pos[0], &vel[0], &eff[0]);
        // jnt_state_interface.registerHandle(state_handle_1);

        // registerInterface(&jnt_state_interface);

        // hardware_interface::JointHandle pos_handle_1(jnt_state_interface.getHandle("joint_1"), &cmd[0]);
        // jnt_pos_interface.registerHandle(pos_handle_1);

        // registerInterface(&jnt_pos_interface);
        ros::NodeHandle rpnh(
            nh_, "hardware_interface");  // TODO(davetcoleman): change the namespace to "generic_hw_interface" aka name_

        pub = rpnh.advertise<std_msgs::Float64>("/test_pub", 10);
        arm_status_sub = rpnh.subscribe("/ArmStatus", 10, &AR3_Robot::arm_status_cb, this, ros::TransportHints().tcpNoDelay());
        arm_control_pub = rpnh.advertise<ck_ros_msgs_node::Arm_Control>("/ArmControl", 10);
        // arm_status_sub = rpnh.subscribe("/ArmStatus", 10, &AR3_Robot::arm_status_cb, this);
	    // odometry_subscriber = node->subscribe("/odometry/filtered", 10, robot_odometry_subscriber, ros::TransportHints().tcpNoDelay());

        std::size_t error = 0;
        error += !rosparam_shortcuts::get(name_, rpnh, "joints", joint_names_);
        ROS_ERROR("ERROR CODE: %ld - size: %ld", error, joint_names_.size());
        rosparam_shortcuts::shutdownIfError(name_, error);
    }

    void init()
    {
        num_joints_ = joint_names_.size();

        // Status
        joint_position_.resize(num_joints_, 0.0);
        joint_velocity_.resize(num_joints_, 0.0);
        joint_effort_.resize(num_joints_, 0.0);

        // Command
        joint_position_command_.resize(num_joints_, 0.0);
        joint_velocity_command_.resize(num_joints_, 0.0);
        joint_effort_command_.resize(num_joints_, 0.0);

        // for (size_t i = 0; i < num_joints_; i++)
        // {
        //     joint_names_.push_back("joint_" + std::to_string(i+1));
        // }

        for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
        {
            ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

            // Create joint state interface
            joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
                joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

            // Add command interfaces to joints
            // TODO: decide based on transmissions?
            hardware_interface::JointHandle joint_handle_position = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_position_command_[joint_id]);
            position_joint_interface_.registerHandle(joint_handle_position);

            // hardware_interface::JointHandle joint_handle_velocity = hardware_interface::Joi

            hardware_interface::JointHandle joint_handle_velocity = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_velocity_command_[joint_id]);
            velocity_joint_interface_.registerHandle(joint_handle_velocity);

            hardware_interface::JointHandle joint_handle_effort = hardware_interface::JointHandle(
                joint_state_interface_.getHandle(joint_names_[joint_id]), &joint_effort_command_[joint_id]);
            effort_joint_interface_.registerHandle(joint_handle_effort);

            // Load the joint limits
            // registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id);
        }  // end for each joint

        registerInterface(&joint_state_interface_);     // From RobotHW base class.
        registerInterface(&position_joint_interface_);  // From RobotHW base class.
        registerInterface(&velocity_joint_interface_);  // From RobotHW base class.
        registerInterface(&effort_joint_interface_);    // From RobotHW base class.

        // ROS_INFO_STREAM_NAMED(name_, "GenericHWInterface Ready.");
    }

    void read(const ros::Time& time, const ros::Duration& period)
    {
        (void)time;
        (void)period;
        // joint_position_[0] = arm_status.arm_base_actual_position * 2 * M_PI;
        // joint_position_[1] = -1 * arm_status.arm_upper_actual_position * 2 * M_PI;
        // joint_position_[2] = 0.0;

        // // status is RPM
        // joint_velocity_[0] = arm_status.arm_base_velocity / 60.0 * 2.0 * M_PI;
        // joint_velocity_[1] = -1 * arm_status.arm_upper_velocity / 60.0 * 2.0 * M_PI;
        // joint_velocity_[2] = 0.0;
    }

    void write(const ros::Time& time, const ros::Duration& period)
    {
        (void)time;
        (void)period;
        for (size_t i = 0; i < num_joints_; i++)
        {
            ROS_WARN("Joint %ld: %f", i+1, joint_velocity_command_[i]);
            joint_position_[i] += joint_velocity_command_[i] * period.toSec();
            joint_velocity_[i] = joint_velocity_command_[i];
            // joint_position_[i] = joint_position_command_[i];
        }
        // ROS_WARN("Joint 1 Velocity command: %f", joint_velocity_command_[0]);
        // ROS_WARN("Joint 1 Position command: %f", joint_position_command_[0]);
        std::cout << std::endl;

        ck_ros_msgs_node::Arm_Control arm_control;
        arm_control.arm_base_requested_position = joint_velocity_command_[0] * 60.0 / 2.0 * M_PI;
        arm_control.arm_upper_requested_position = -1 * joint_velocity_command_[1] * 60.0 / 2.0 * M_PI;
        arm_control.arm_wrist_requested_position = 0.0;
        arm_control.extend = 0.0;

        arm_control_pub.publish(arm_control);
    }

    void arm_status_cb(const ck_ros_msgs_node::Arm_Status& status)
    {
        this->arm_status = status;
        // ROS_ERROR("GOT ARM STATUS");
    }

private:
    const ros::NodeHandle& nh_;
    std::string name_;

    ros::Publisher pub;
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
    urdf::Model* urdf_model_;

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