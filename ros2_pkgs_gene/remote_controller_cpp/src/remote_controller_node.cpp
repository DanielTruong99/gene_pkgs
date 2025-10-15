#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "interfaces/msg/key_info.hpp"

using namespace std::chrono_literals;

class RemoteControllerNode : public rclcpp::Node 
{
    public:
        RemoteControllerNode()
        : rclcpp::Node("remote_controller_node"),
        _last_key_value("NONE"),
        _key_value("NONE"),
        _start_timer(false),
        _timer_counter(0)
    {
        // Initialize key map
        _key_map = {
            {1u << 0, "A"},
            {1u << 1, "B"},
            {1u << 2, "X"},
            {1u << 3, "Y"},
            {1u << 4, "LB"},
            {1u << 5, "RB"},
            {1u << 6, "BACK"},
            {1u << 7, "START"},
            {1u << 8, "LOGITECH"}
        };

        _joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&RemoteControllerNode::joystick_callback, this, std::placeholders::_1)
        );

        _key_info_pub = this->create_publisher<interfaces::msg::KeyInfo>("key_info", 10);

        _timer = this->create_wall_timer(100ms, std::bind(&RemoteControllerNode::timer_callback, this));
    }

    private:
        void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            const auto & buttons = msg->buttons;

            // Calculate the key value as bitmask
            uint32_t bitmask = 0u;
            for (size_t i = 0; i < buttons.size(); ++i) if (buttons[i] == 1) bitmask += (1u << i);

            const std::string key_value = (_key_map.count(bitmask) ? _key_map.at(bitmask) : std::string("NONE"));

            if (_last_key_value != key_value) 
            {
                interfaces::msg::KeyInfo key_info_msg;

                if (_last_key_value == "NONE") 
                {
                    // Key is pressed
                    key_info_msg.key_event = "KEY_PRESSED";
                    key_info_msg.key_value = key_value;
                    _key_value = key_value;
                    _key_info_pub->publish(key_info_msg);
                } 
                else 
        {
                    // Key is released
                    key_info_msg.key_value = _key_value;
                    key_info_msg.key_event = "KEY_RELEASED";
                    _key_info_pub->publish(key_info_msg);
                    if (_start_timer) 
                    {
                        _start_timer = false;
                        _timer_counter = 0;
                    }
                }

                // If pressed now, enable timer
                if (!_start_timer && key_info_msg.key_event == "KEY_PRESSED") 
                {
                    _start_timer = true;
                    _timer_counter = 0;
                }

                _last_key_value = key_value;
            }
        }

    private:
        void timer_callback()
        {
            if (!_start_timer) return;

            ++_timer_counter;
            if (_timer_counter == 30) 
            {   
                // 3 seconds at 10Hz
                interfaces::msg::KeyInfo key_info_msg;
                key_info_msg.key_value = _key_value;
                key_info_msg.key_event = "KEY_HOLDING_3S";
                _key_info_pub->publish(key_info_msg);
            } 
            else if (_timer_counter == 50) 
            { 
                // 5 seconds at 10Hz
                interfaces::msg::KeyInfo key_info_msg;
                key_info_msg.key_value = _key_value;
                key_info_msg.key_event = "KEY_HOLDING_5S";
                _key_info_pub->publish(key_info_msg);
            }
        }

    private:
        std::unordered_map<uint32_t, std::string> _key_map;
        std::string _last_key_value;
        std::string _key_value;
        bool _start_timer;
        int _timer_counter;

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_sub;
        rclcpp::Publisher<interfaces::msg::KeyInfo>::SharedPtr _key_info_pub;
        rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
