#ifndef ROBOT_INTERFACE_H
#define ROBOT_INTERFACE_H
#pragma once

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <cmath>



struct TiltResult 
{
  double tilt_rad;
  double tilt_deg;
};

inline TiltResult tilt_from_quat(double w, double x, double y, double z) 
{
    /* Z pointing downward! */

    // Normalize
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n == 0.0) return {NAN, NAN};
    w /= n; x /= n; y /= n; z /= n;

    // s = magnitude of XY part (yaw-invariant)
    double s = std::sqrt(x*x + y*y);

    // Tilt wrt world +Z (your original)
    double tilt_up = 2.0 * std::atan2(s, w);

    // For Z-down convention: angle between body +Z and world -Z
    double tilt_down = M_PI - tilt_up;

    // Clamp to [0, π] (numerical safety)
    if (tilt_down < 0.0) tilt_down = 0.0;
    if (tilt_down > M_PI) tilt_down = M_PI;

    return {tilt_down, tilt_down * 180.0 / M_PI};
}

// Example check
inline bool is_tilt_safe(double w, double x, double y, double z, double limit_deg = 20.0) 
{
  auto t = tilt_from_quat(w, x, y, z);
  return std::isfinite(t.tilt_deg) && t.tilt_deg <= limit_deg;
}

// Example check
inline bool is_tilt_safe(std::vector<float>& projected_g) 
{
  
  return std::abs(projected_g[0]) < 0.7f || std::abs(projected_g[1]) < 0.7f;
}

/* Utility functions */
inline std::unordered_map<std::string,int>
make_index(const std::vector<std::string>& names) {
    std::unordered_map<std::string,int> m;
    m.reserve(names.size());
    for (int i = 0; i < (int)names.size(); ++i) m.emplace(names[i], i);
    return m;
}

inline std::vector<int>
indices_of(const std::vector<std::string>& keys,
           const std::unordered_map<std::string,int>& dict,
           int missing = -1) {
    std::vector<int> idx; idx.reserve(keys.size());
    for (auto& k : keys) {
        if (auto it = dict.find(k); it != dict.end()) idx.push_back(it->second);
        else idx.push_back(missing);
    }
    return idx;
}

template <class T>
inline std::vector<T> gather(const std::vector<T>& src,
                             const std::vector<int>& idx,
                             T fill = T{}) {
    std::vector<T> out(idx.size(), fill);
    for (size_t i = 0; i < idx.size(); ++i)
        if (int j = idx[i]; j >= 0 && j < (int)src.size()) out[i] = src[j];
    return out;
}

template <class T>
inline std::vector<T> scatter_rl_to_hw(const std::vector<T>& rl_vals,
                                       const std::vector<int>& rl_to_hw,
                                       size_t hw_size,
                                       T fill = T{}) {
    std::vector<T> out(hw_size, fill);
    for (size_t rl = 0; rl < rl_to_hw.size(); ++rl)
        if (int hw = rl_to_hw[rl]; hw >= 0 && hw < (int)hw_size) out[hw] = rl_vals[rl];
    return out;
}


class Robot 
{
    public:
        Robot()
        {
            _joint_positions = std::vector<float>(10, 0.0f);
            _joint_velocities = std::vector<float>(10, 0.0f);
            _joint_torques = std::vector<float>(10, 0.0f);
            _vb = std::vector<float>(3, 0.0f);
            _wb = std::vector<float>(3, 0.0f);
            _rw = std::vector<float>(3, 0.0f);
            _qw = std::vector<float>(4, 0.0f);
            _proj_g = std::vector<float>(3, 0.0f);
            _raw_v_cmds = std::vector<float>(3, 0.0f);
        };

    public:
        /* Getters */
        std::vector<float> get_joint_positions() { return _joint_positions; }

        std::vector<std::string> get_joint_names() { return _joint_names; }
        void set_joint_names(std::vector<std::string> &joint_names) {_joint_names = joint_names;}

        std::vector<float> get_joint_velocities() { return _joint_velocities; }

        std::vector<float> get_joint_torques() { return _joint_torques; }

        std::vector<float> get_base_velocity() { return _vb; }

        std::vector<float> get_base_angular_velocity() 
        { 
            // /* Z up */ 
            // // _wb is [wx, wy, wz] in Z-down body frame
            // const float wx = _wb[0];
            // const float wy = _wb[1];
            // const float wz = _wb[2];

            // // Convert to Z-up body frame (no qx_pi, no quaternion math)
            // std::vector<float> out(3);
            // out[0] = wx;
            // out[1] = -wy;
            // out[2] = -wz;
            return _wb;
        }

        std::vector<float> get_base_position() { return _rw; }

        std::vector<float> get_base_quaternion() { return _qw; }

        std::vector<float> get_proj_g()
        {
            // _qw is quaternion in Z-down convention [w,x,y,z]
            const float w = _qw[0], x = _qw[1], y = _qw[2], z = _qw[3];
            Eigen::Quaternionf q_zup(w, x, y, z);
            q_zup.normalize();

            // Gravity in world (Z-up convention: down is -Z)
            Eigen::Vector3f g_world(0.f, 0.f, -1.f);

            // Express gravity in body Z-up frame
            Eigen::Vector3f g_body = q_zup.conjugate() * g_world;

            // Copy into _proj_g
            _proj_g.resize(3);
            std::copy_n(g_body.data(), 3, _proj_g.begin());
            return _proj_g;
        }
        std::vector<float> get_raw_joystick_cmds() {return _raw_v_cmds;}


        struct SafetyReport
        {
            bool is_safe = true;
            std::vector<std::string> error_msgs;
        };
        SafetyReport is_safe()
        {
            SafetyReport report;

            if(!is_tilt_safe(_proj_g))
            {
                report.is_safe = false;
                report.error_msgs.push_back("Tilt larger than limit");
            }
            
            return report;
        }

    protected:
        /* Low level states */
        std::vector<std::string> _joint_names;
        std::vector<float> _joint_positions;
        std::vector<float> _joint_velocities;
        std::vector<float> _joint_torques;

    protected:
        /* High level states */
        std::vector<float> _vb; // Base velocity in base frame
        std::vector<float> _wb; // Base angular velocity in base frame
        std::vector<float> _rw; // Base position in world frame
        std::vector<float> _qw; // Base quaternion in world frame
        std::vector<float> _proj_g;

    protected:
        /* Raw command from joystick */
        std::vector<float> _raw_v_cmds; // raw [vx, vy, wz]
};


#endif // ROBOT_INTERFACE_H

