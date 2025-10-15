#ifndef ROS_DATA_INTERFACE_H
#define ROS_DATA_INTERFACE_H

#include <vector>
#include <unordered_map>
#include <chrono>
#include "Robot.h"
#include "../EventManager/EventManager.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <vectornav_msgs/msg/common_group.hpp>
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/key_info.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#define DEBUG_IMU 1

using namespace std::chrono_literals;

class ROSDataInterface : public Robot
{
    public:
        ROSDataInterface(rclcpp::Node::SharedPtr node) : Robot(), _node(node)
        {
            //! Debug only
            #if DEBUG_IMU
                _pose_stamp_pub = _node->create_publisher<geometry_msgs::msg::PoseStamped>("/est_pose", 10);
            #endif

            _joint_state_sub = _node->create_subscription<sensor_msgs::msg::JointState>(
                "joint_states", 10, std::bind(&ROSDataInterface::joint_state_callback, this, std::placeholders::_1)
            );

            _imu_sub = _node->create_subscription<vectornav_msgs::msg::CommonGroup>(
                "vectornav/raw/common", 10, std::bind(&ROSDataInterface::imu_callback, this, std::placeholders::_1)
            );

            _joystick_sub = _node->create_subscription<sensor_msgs::msg::Joy>(
                "joy", 10, std::bind(&ROSDataInterface::joy_stick_callback, this, std::placeholders::_1)
            );

            _joystick_keyinfo_sub = _node->create_subscription<interfaces::msg::KeyInfo>(
                "key_info", 10, std::bind(&ROSDataInterface::joystick_keyinfo_callback, this, std::placeholders::_1)
            );

            _sensor_state_map = {
                {"joint_states", 0},
                {"imu", 1},
            };
            _are_all_sensors_recieved = std::vector<bool>(_sensor_state_map.size(), false);
        }
        
        bool is_ready() 
        {
            return std::all_of(
                _are_all_sensors_recieved.begin(), 
                _are_all_sensors_recieved.end(), 
                [](bool recieved) {return recieved;}
            );
        }

    private:
        std::vector<bool> _are_all_sensors_recieved;
        std::unordered_map<std::string, uint8_t> _sensor_state_map;
        std::unordered_map<std::string, rclcpp::Time> _last_rx;

    private:
        rclcpp::Node::SharedPtr _node;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
        rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr _imu_sub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joystick_sub;
        rclcpp::Subscription<interfaces::msg::KeyInfo>::SharedPtr _joystick_keyinfo_sub;
        //! Debug only
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pose_stamp_pub;

    private: 
        /* Callback function for joint state subscriber */
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

        /* Callback function for IMU subscriber */
        void imu_callback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);

        /* Callback function for Joy stick subscriber */
        void joy_stick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

        /* Callback function for Joy stick key info subscriber */
        void joystick_keyinfo_callback(const interfaces::msg::KeyInfo::SharedPtr msg);
};



#endif // ROS_DATA_INTERFACE_H