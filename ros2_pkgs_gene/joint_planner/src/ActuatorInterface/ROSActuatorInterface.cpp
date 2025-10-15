#include "ROSActuatorInterface.h"

ROSActuatorInterface::ROSActuatorInterface(rclcpp::Node::SharedPtr node)
: _node(node)
{
    /* Initialize the publisher and message */
    _filtered_action_publisher = _node->create_publisher<interfaces::msg::CustomJointState>("joint_cmds", 10);
    _filtered_action_msg.state.position.resize(10);
    _filtered_action_msg.kp.resize(10);
    _filtered_action_msg.kd.resize(10);

    /* Initialize the publisher and message */
    _action_publisher = _node->create_publisher<sensor_msgs::msg::JointState>("joint_cmd_debug", 10);
    _action_msg.position.resize(10);
}

void ROSActuatorInterface::set_joint_names(std::vector<std::string> &joint_names)
{
    _filtered_action_msg.state.name = joint_names;
    _action_msg.name = joint_names;
}

void ROSActuatorInterface::send_command(std::vector<float> &joint_positions)
{
    _filtered_action_msg.state.header.stamp = _node->now();
    std::copy(joint_positions.begin(), joint_positions.end(), _filtered_action_msg.state.position.begin());
    _filtered_action_publisher->publish(_filtered_action_msg);
}

void ROSActuatorInterface::set_gains(std::vector<float> &kp, std::vector<float> &kd)
{
    std::copy(kp.begin(), kp.end(), _filtered_action_msg.kp.begin());
    std::copy(kd.begin(), kd.end(), _filtered_action_msg.kd.begin());
}

/* For debugging only */
void ROSActuatorInterface::send_command_debug(std::vector<float> &joint_positions)
{
    _action_msg.header.stamp = _node->now();
    std::copy(joint_positions.begin(), joint_positions.end(), _action_msg.position.begin());
    _action_publisher->publish(_action_msg);
}