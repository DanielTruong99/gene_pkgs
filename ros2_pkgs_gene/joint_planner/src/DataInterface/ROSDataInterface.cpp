#include "ROSDataInterface.h"

void ROSDataInterface::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    /* Check nan */
    bool is_pos_nan = std::any_of(msg->position.begin(), msg->position.end(), [](float val) { return std::isnan(val); });
    bool is_vel_nan = std::any_of(msg->velocity.begin(), msg->velocity.end(), [](float val) { return std::isnan(val); });
    if(is_pos_nan || is_vel_nan) return;

    /* Check infinite */
    bool is_pos_inf = std::any_of(msg->position.begin(), msg->position.end(), [](float val) { return std::isinf(val); });
    bool is_vel_inf = std::any_of(msg->position.begin(), msg->position.end(), [](float val) { return std::isinf(val); });
    if(is_pos_inf || is_vel_inf) return;

    /* Data are safe, cache the joint data */
    std::copy(msg->position.begin(), msg->position.end(), _joint_positions.begin());
    std::copy(msg->velocity.begin(), msg->velocity.end(), _joint_velocities.begin());
    std::copy(msg->effort.begin(), msg->effort.end() - 1, _joint_torques.begin());
    _are_all_sensors_recieved[_sensor_state_map["joint_states"]] = true;
}

void ROSDataInterface::imu_callback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg)
{
    /* Check nan */
    bool is_angular_nan = std::isnan(msg->angularrate.x) || std::isnan(msg->angularrate.y) || std::isnan(msg->angularrate.z);
    bool is_quat_nan = std::isnan(msg->quaternion.x) || std::isnan(msg->quaternion.y) || std::isnan(msg->quaternion.z) || std::isnan(msg->quaternion.w);
    if(is_angular_nan || is_quat_nan) return;

    /* Check infinite */
    bool is_angular_inf = std::isinf(msg->angularrate.x) || std::isinf(msg->angularrate.y) || std::isinf(msg->angularrate.z);
    bool is_quat_inf = std::isinf(msg->quaternion.x) || std::isinf(msg->quaternion.y) || std::isinf(msg->quaternion.z) || std::isinf(msg->quaternion.w);
    if(is_angular_inf || is_quat_inf) return;

    static bool is_first_run = true;
    static const Eigen::Quaternionf qx_pi(
        Eigen::AngleAxisf(float(M_PI), Eigen::Vector3f::UnitX())
    );
    static Eigen::Quaternionf q_glob_init;
    if(is_first_run)
    {
        is_first_run = false;
        const float w = msg->quaternion.w, x = msg->quaternion.x, y =msg->quaternion.y, z = msg->quaternion.z;
        static Eigen::Quaternionf q_imu_init(w, x, y, z);
        q_imu_init.normalize(); if(q_imu_init.w() < 0.0f) q_imu_init.coeffs() *= -1.0f;
        float yaw = q_imu_init.toRotationMatrix().eulerAngles(0, 1, 2).z();
        q_glob_init = Eigen::Quaternionf(
            Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
        );
        q_glob_init = q_glob_init * qx_pi;
        q_glob_init.normalize(); if(q_glob_init.w() < 0.0f) q_glob_init.coeffs() *= -1.0f;
    }

    /* Convert quaternion */
    const float w = msg->quaternion.w, x = msg->quaternion.x, y =msg->quaternion.y, z = msg->quaternion.z;
    Eigen::Quaternionf q_imu(w, x, y, z);
    q_imu.normalize(); if(q_imu.w() < 0.0f) q_imu.coeffs() *= -1.0f;

    
    
    /* e0 = qimu e2
       e0 = q_glob e1
       e1 = q_glob^-1 * qimu
    */
    // Eigen::Quaternionf q_robot = q_imu * qx_pi;
    Eigen::Quaternionf q_robot = q_glob_init.inverse() * q_imu * qx_pi;
    q_robot.normalize(); if(q_robot.w() < 0.0f) q_robot.coeffs() *= -1.0f;

    /* Convert angular velocity */
    Eigen::Vector3f w_imu(msg->angularrate.x, msg->angularrate.y, msg->angularrate.z);
    Eigen::Vector3f w_robot = qx_pi.inverse() * w_imu;

    // //! Debug imu orientation only
    // #if DEBUG_IMU
    //     geometry_msgs::msg::PoseStamped imu_msg;
    //     imu_msg.header.stamp = _node->get_clock()->now();
    //     imu_msg.header.frame_id = "base";
    //     imu_msg.pose.orientation.x = q_robot.x();
    //     imu_msg.pose.orientation.y = q_robot.y();
    //     imu_msg.pose.orientation.z = q_robot.z();
    //     imu_msg.pose.orientation.w = q_robot.w();
    //     _pose_stamp_pub->publish(imu_msg);
    // #endif

    /* Data are safe, cache the data */
    _wb[0] = w_robot.x(); _wb[1] = w_robot.y(); _wb[2] = w_robot.z();
    _qw[0] = q_robot.w(); _qw[1] = q_robot.x(); _qw[2] = q_robot.y(); _qw[3] = q_robot.z();
    _are_all_sensors_recieved[_sensor_state_map["imu"]] = true;
}

void ROSDataInterface::joy_stick_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    _raw_v_cmds[1] = -msg->axes[0]; // vy = left x
    _raw_v_cmds[0] = msg->axes[1]; // vx = left y
    _raw_v_cmds[2] = msg->axes[3]; // w = right x
}


void ROSDataInterface::joystick_keyinfo_callback(const interfaces::msg::KeyInfo::SharedPtr msg)
{
    auto key_value = msg->key_value;
    auto key_event = msg->key_event;
    if (key_value == "START" && key_event == "KEY_HOLDING_3S")
    {
        static event_manager::Event const start_button_3s_event(static_cast<uint8_t>(event_manager::Signal::START_BUTTON_3S));
        event_manager::g_event_manager.post_event(&start_button_3s_event); 
    }
    else if (key_value == "BACK" && key_event == "KEY_PRESSED")
    {
        static event_manager::Event const back_button_pressed_event(static_cast<uint8_t>(event_manager::Signal::BACK_BUTTON_PRESSED));
        event_manager::g_event_manager.post_event(&back_button_pressed_event);
    }
}