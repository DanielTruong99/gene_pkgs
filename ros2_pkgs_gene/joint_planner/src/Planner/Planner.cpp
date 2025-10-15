#include "Planner.h"

namespace planner
{

    std::vector<float> Planner::compute(float dt)
    {
        /* Get the lower bound index in times of _current_time */
        auto it = std::lower_bound(_joint_trajectories[_current_trajectory_index].time.begin(),
                       _joint_trajectories[_current_trajectory_index].time.end(),
                       _current_time);
        size_t index = std::distance(_joint_trajectories[_current_trajectory_index].time.begin(), it);
        if(_current_time >= _joint_trajectories[_current_trajectory_index].time[index - 1])
        {
            index = index - 1;
        }
        

        /* Get the upper bound index in times of _current_time */
        auto it_next = std::upper_bound(_joint_trajectories[_current_trajectory_index].time.begin(),
                        _joint_trajectories[_current_trajectory_index].time.end(),
                        _current_time);
        size_t index_next = std::distance(_joint_trajectories[_current_trajectory_index].time.begin(), it_next);

        /* 
            Case 1: t0 <= current_time <= t1, t0 = t1
        */
        if (index == index_next)
        {
            std::copy(_joint_trajectories[_current_trajectory_index].joint_positions[index].begin(),
                      _joint_trajectories[_current_trajectory_index].joint_positions[index].end(),
                      _joint_positions_cmd.begin());
        }

        /* Case: t0 <= current_time <= t1 */
        /* Interpolate the joint positions */
        if (index < _joint_trajectories[_current_trajectory_index].joint_positions.size() - 1)
        {
            float t0 = _joint_trajectories[_current_trajectory_index].time[index];
            float t1 = _joint_trajectories[_current_trajectory_index].time[index_next];
            float alpha = (_current_time - t0) / (t1 - t0);

            for (size_t i = 0; i < _joint_trajectories[_current_trajectory_index].joint_positions[0].size(); ++i)
            {
            _joint_positions_cmd[i] = _joint_trajectories[_current_trajectory_index].joint_positions[index][i] +
                          alpha * (_joint_trajectories[_current_trajectory_index].joint_positions[index_next][i] -
                               _joint_trajectories[_current_trajectory_index].joint_positions[index][i]);
            }
        }
        else
        {
            _joint_positions_cmd = _joint_trajectories[_current_trajectory_index].joint_positions.back();
        }

        _current_time += dt;

        /* Add with the joint position offset, 
            Because the reference motion come from the simulation,
            the joint position offset is needed to be added to the joint position commands to 
            make the robot move in the real world.
        */
       std::transform(_joint_positions_cmd.begin(), _joint_positions_cmd.end(), _offset_positions.begin(), _joint_positions_cmd.begin(),
            [this](float joint_position_cmd, float offset_position) 
            { 
                return joint_position_cmd + offset_position; 
            }
        );
        return _joint_positions_cmd;
    }
    

    void Planner::reset_filter()
    {
        _is_first_time = true;
    }

    std::vector<float> Planner::apply_filter(std::vector<float> &joint_positions_cmds)
    {
        if(_is_first_time)
        {
            std::vector<float> current_joint_positions = _robot_interface->get_joint_positions();
            std::copy(current_joint_positions.begin(), current_joint_positions.end(), _filtered_position_cmds.begin());
            _is_first_time = false;
        }

        /* Apply simple low pass filter */
        std::transform(_filtered_position_cmds.begin(), _filtered_position_cmds.end(), joint_positions_cmds.begin(), _filtered_position_cmds.begin(),
            [this](float filtered_position_cmd, float joint_positions_cmd)
            {
                return _alpha * filtered_position_cmd + (1.0f - _alpha) * joint_positions_cmd;
            }
        );

        return _filtered_position_cmds;
    }


    bool Planner::is_reached() const
    {
        // Check if the  joint position is reached
        float joint_pos_error_norm = 0.0f;
        float joint_pos_error;
        std::vector<float> joint_pos = _robot_interface->get_joint_positions();
        // ! Temporarily set command for 2 joint positions
        for (size_t index = 0; index < 2; ++index)
        {
            joint_pos_error = _joint_positions_cmd[index] - joint_pos[index];
            joint_pos_error_norm += std::pow(joint_pos_error, 2);
        }
        joint_pos_error_norm = std::sqrt(joint_pos_error_norm) / 2.0f;
        return joint_pos_error_norm < 0.01f; // Threshold for reaching the position
    }

    bool Planner::is_trajectory_completed() const
    {
        // Check if the trajectory is reached
        return _current_time >= _joint_trajectories[_current_trajectory_index].time.back();
    }

    void Planner::change_trajectory()
    {
        _current_time = 0.0f; // Reset time for new trajectory
        _current_trajectory_index = ++_current_trajectory_index >= _joint_trajectory_files.size() ? 0 : _current_trajectory_index;

        if (_joint_trajectory_keys[_current_trajectory_index]) return;
        _load_trajectory(_joint_trajectory_files[_current_trajectory_index]);
        _joint_trajectory_keys[_current_trajectory_index] = true;
    }

    void Planner::reset()
    {
        _initialize();
    }

    void Planner::_initialize()
    {
        _current_time = 0.0f;

        /* Loop over the specified directory and find all the file .csv and save into  _joint_trajectory_files */
        const std::string path = "/home/gene/DanielWorkspace/ros2_ws/src/joint_planner/motion_files"; // current directory
        DIR *dir = opendir(path.c_str());
        if (!dir)
        {
            std::cerr << "Cannot open directory: " << path << std::endl;
            return;
        }

        struct dirent *entry;
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string name = entry->d_name;
            if (name == "." || name == "..") continue;
            if (name.substr(name.find_last_of(".") + 1) == "csv")
            {
                _joint_trajectory_files.push_back(path + "/" + name);
            }
        }

        closedir(dir);

        /* Read the csv file and store the trajectory */
        _load_trajectory(_joint_trajectory_files[_current_trajectory_index]);
        _joint_trajectory_keys[_current_trajectory_index] = true;
    }

    void Planner::_load_trajectory(const std::string &file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open())
        {
            std::cerr << "Error opening file: " << file_path << std::endl;
            return;
        }
        std::string line;
        JointTrajectory trajectory;
        bool is_first_line = true;
        while (std::getline(file, line))
        {
            if (is_first_line)
            {
                is_first_line = false;
                continue; // Skip the header line
            }

            std::istringstream ss(line);
            std::string value;
            /*
                Each row of the csv file contains:
                joint_positions, time
                t, q1, q2, q3, q4, q5
            */
            std::vector<float> row_values;
            while(std::getline(ss, value, ','))
            {
                row_values.push_back(std::stof(value));
            }

            // Read joint positions
            std::vector<float> joint_positions;
            joint_positions.resize(5);
            std::copy(row_values.begin() + 1, row_values.end(), joint_positions.begin());

            // Read time
            float time;
            time = *row_values.begin();

            trajectory.joint_positions.push_back(joint_positions);
            trajectory.time.push_back(time);
        }
        file.close();
        _joint_trajectories.push_back(trajectory);
    }
} // namespace planner