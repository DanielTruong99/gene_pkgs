#ifndef ROS_ACTUATOR_INTERFACE_H
#define ROS_ACTUATOR_INTERFACE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "interfaces/msg/custom_joint_state.hpp"



class ROSActuatorInterface
{
    public:
        ROSActuatorInterface(rclcpp::Node::SharedPtr node);

        void send_command(std::vector<float> &joint_positions);
        void set_gains(std::vector<float> &kp, std::vector<float> &kd);
        void set_joint_names(std::vector<std::string> &joint_names);
        void send_command_debug(std::vector<float> &joint_positions);

    private:
        rclcpp::Node::SharedPtr _node;

        rclcpp::Publisher<interfaces::msg::CustomJointState>::SharedPtr _filtered_action_publisher;
        interfaces::msg::CustomJointState _filtered_action_msg;

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _action_publisher;
        sensor_msgs::msg::JointState _action_msg;
};

#endif // ROS_ACTUATOR_INTERFACE_H