#include "knighttime_interface.hpp"

KnighttimeRobotHW::KnighttimeRobotHW(const ros::NodeHandle& nh)
    : nh_(nh),
      name_("knighttime_hardware")
    //   baseMotor(9, Motor::Motor_Type::TALON_FX),
    //   upperMotor(11, Motor::Motor_Type::TALON_FX)
{
    ros::NodeHandle rpnh(nh_, "hardware_interface");

    arm_status_sub = rpnh.subscribe("/ArmStatus", 10, &KnighttimeRobotHW::arm_status_cb, this, ros::TransportHints().tcpNoDelay());
    arm_control_pub = rpnh.advertise<ck_ros_msgs_node::Arm_Control>("/ArmControl", 10);

    vel_cmd_pub = rpnh.advertise<std_msgs::Float64>("/real_vel_cmd", 10);

    if (!rpnh.getParam("joints", joint_names_))
    {
        ROS_ERROR_NAMED(name_, "COULD NOT FIND JOINT NAMES PARAM! - EXITING");
        exit(1);
    }
}

void KnighttimeRobotHW::init()
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

    for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
    {
        ROS_ERROR_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

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

void KnighttimeRobotHW::arm_status_cb(const ck_ros_msgs_node::Arm_Status& status)
{
    this->arm_status = status;
}

void KnighttimeRobotHW::read(const ros::Time& time, const ros::Duration& period)
{
    (void)time;
    (void)period;

    // joint_position_[0] = ck::math::deg2rad(arm_status.arm_base_angle);
    // joint_position_[1] = ck::math::deg2rad(arm_status.arm_upper_angle);
    // joint_position_[2] = 0.0;

    // // // // status is RPM
    // joint_velocity_[0] = arm_status.arm_base_velocity / 60.0 * 2.0 * M_PI;
    // joint_velocity_[1] = arm_status.arm_upper_velocity / 60.0 * 2.0 * M_PI;
    // joint_velocity_[2] = 0.0;
}

void KnighttimeRobotHW::write(const ros::Time& time, const ros::Duration& period)
{
    (void)time;
    (void)period;
    for (size_t i = 0; i < num_joints_; i++)
    {
        // ROS_WARN("Joint %ld: %f", i+1, joint_velocity_[i]);
        joint_position_[i] += joint_velocity_command_[i] * period.toSec();
        joint_velocity_[i] = joint_velocity_command_[i];
        // joint_position_[i] = joint_position_command_[i];
        joint_effort_[i] = joint_effort_command_[i];
    }
    std_msgs::Float64 real_vel;
    real_vel.data = joint_velocity_command_[1];

    // vel_cmd_pub.publish(real_vel);
    // ROS_WARN("Joint 1 Velocity command: %f", joint_velocity_command_[0]);
    // ROS_WARN("Joint 1 Position command: %f", joint_position_command_[0]);
    // ROS_WARN("Joint 1 Effort Command: %f", joint_effort_command_[0]);
    // std::cout << std::endl;

    ck_ros_msgs_node::Arm_Control arm_control;
    // arm_control.arm_base_requested_position = joint_velocity_command_[0] * 60.0 / 2.0 * M_PI;
    // arm_control.arm_upper_requested_position = joint_velocity_command_[1] * 60.0 / 2.0 * M_PI;
    // arm_control.arm_wrist_requested_position = 0.0;
    arm_control.arm_base_requested_position = joint_effort_command_[0];
    arm_control.arm_upper_requested_position = joint_effort_command_[1];
    arm_control.arm_wrist_requested_position = 0.0;

    // baseMotor

    // arm_control.arm_base_requested_position = joint_position_command_[0];
    // arm_control.arm_upper_requested_position = joint_position_command_[1];
    // arm_control.arm_wrist_requested_position = 0.0;
    arm_control.extend = 0.0;

    arm_control_pub.publish(arm_control);
}

std::vector<std::string> KnighttimeRobotHW::get_joint_names()
{
    return joint_names_;
}
