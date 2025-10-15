#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <dirent.h>
#include <chrono>
#include <array>

#include "../DataInterface/ROSDataInterface.h"

namespace planner 
{
    struct JointTrajectory
    {
        std::vector<std::vector<float>> joint_positions;
        std::vector<float> time;
    };

    using JointTrajectory = struct JointTrajectory;

    class Planner 
    {
        public:
            Planner(std::shared_ptr<ROSDataInterface> robot_interface) : 
                _robot_interface(robot_interface)
            {
                _initialize();
                _joint_positions_cmd.resize(5);
                _filtered_position_cmds.resize(5);
                // constexpr float pi = 3.14159265358979323846f;
                _offset_positions = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Joint position offset
            }

        private:
            /* Keep track time in the loaded motion file */
            float _current_time;
            std::vector<float> _offset_positions;   
        public:
            /* Compute the joint position cmds from the trajectory */
            std::vector<float> compute(float dt);

            /* Reset the planner */
            void reset();
            
            /* Check if the first joint position cmds is reached */
            bool is_reached() const;

            /* Check if the trajectory is completed */
            bool is_trajectory_completed() const;
            

            /* Change the index of trajectory */
            void change_trajectory();

            /* Get the current joint position cmds */
            std::vector<float> get_actions() const { return _joint_positions_cmd; }


        private:
            /* Variable related low pass filter */
            std::vector<float> _filtered_position_cmds;
            bool _is_first_time = true;
            const float _alpha = 0.9105f;
        public:
            /* Apply low pass filter to the joint position commands */
            std::vector<float> apply_filter(std::vector<float> &joint_positions_cmds);

            /* Reset the filter */
            void reset_filter();


        private:
            /* Robot interface */
            std::shared_ptr<ROSDataInterface> _robot_interface;

            
        private:
            /* Joint trajectory data */
            std::vector<float> _joint_positions_cmd;
            std::vector<JointTrajectory> _joint_trajectories;
            std::array<bool, 100> _joint_trajectory_keys;
            std::vector<std::string> _joint_trajectory_files;
            uint8_t _current_trajectory_index = 0;

            /* Enumerate motion files in specified directory*/
            void _initialize();

            /* Load trajectory from file */
            void _load_trajectory(const std::string &file_path);
    };

    /* Helper functions */
    inline float clamp(float x, float lo, float hi) {
        return std::max(lo, std::min(x, hi));
    }

    // Broadcast Amin/Amax to a common size
    inline size_t resolve_size(const std::vector<float>& Amin,
                            const std::vector<float>& Amax) {
        if (Amin.empty() || Amax.empty())
            throw std::runtime_error("Amin/Amax must be non-empty.");
        if (Amin.size() == Amax.size()) return Amin.size();
        if (Amin.size() == 1) return Amax.size();
        if (Amax.size() == 1) return Amin.size();
        throw std::runtime_error("Amin/Amax sizes must match or one must be size 1.");
    }

    /**
     * Linear-chirp with linear amplitude envelope.
     * @param t0  start time
     * @param t   current time
     * @param T   duration (seconds), T>0
     * @param f0  start freq (Hz)
     * @param fT  end   freq (Hz)
     * @param Amin  min amplitude  (size 1 or N)
     * @param Amax  max amplitude  (size 1 or N)
     * @return vector<float> y of size N
     */
    inline std::vector<float> chirp_signal(float t0, float t, float T,
                                            float f0, float fT,
                                            const std::vector<float>& Amin,
                                            const std::vector<float>& Amax)
    {
        if (T <= 0.0) throw std::runtime_error("T must be > 0.");
        const size_t N = resolve_size(Amin, Amax);

        // local time within [0, T]
        const float tau = clamp(t - t0, 0.0, T);

        // linear freq sweep: f(t) = f0 + k*tau
        const float k = (fT - f0) / T;

        // phase(tau) = 2π * ( f0 * tau + 0.5 * k * tau^2 )
        const float phase = 2.0 * M_PI * (f0 * tau + 0.5 * k * tau * tau);

        // amplitude ramp: A(tau) = Amin + r*(Amax - Amin), r in [0,1]
        const float r = (T > 0.0) ? (tau / T) : 0.0;

        std::vector<float> y(N);
        for (size_t i = 0; i < N; ++i) {
            const float a_min = (Amin.size() == 1 ? Amin[0] : Amin[i]);
            const float a_max = (Amax.size() == 1 ? Amax[0] : Amax[i]);
            const float A = a_min + r * (a_max - a_min);  // envelope
            y[i] = A * std::sin(phase);                    // starts at 0 at t=t0
        }
        return y;
    }


} // namespace Planner

#endif // PLANNER_H