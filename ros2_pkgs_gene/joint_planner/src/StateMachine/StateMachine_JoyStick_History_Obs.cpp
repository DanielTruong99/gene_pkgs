#include "StateMachine_ControlledYaw_History_Obs.h"

namespace planner_sm
{
    Status PlannerStateMachine::initial_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Initial state");

                /* get default gains set up from ros parameters */
                _node->declare_parameter("default_gains.kps", std::vector<double>(_rl->cfg()->action_dim, 0.0f));
                std::vector<double> tmp;
                _default_kps.reserve(_rl->cfg()->action_dim);
                _node->get_parameter("default_gains.kps", tmp);
                _default_kps.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("default_gains.kds", std::vector<double>(_rl->cfg()->action_dim, 0.0f));
                _default_kds.reserve(_rl->cfg()->action_dim);
                _node->get_parameter("default_gains.kds", tmp);
                _default_kds.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("positions.standing", std::vector<double>(_rl->cfg()->action_dim, 0.0f));
                _standing_joint_pos.reserve(_rl->cfg()->action_dim);
                _node->get_parameter("positions.standing", tmp);
                _standing_joint_pos.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("positions.default", std::vector<double>(_rl->cfg()->action_dim, 0.0f));
                _default_joint_pos.reserve(_rl->cfg()->action_dim);
                _node->get_parameter("positions.default", tmp);
                _default_joint_pos.assign(tmp.begin(), tmp.end());

                /* get hardware joint names set up from ros parameters */
                auto joint_names = _node->declare_parameter<std::vector<std::string>>("hw.joint_names", {});
                if (joint_names.empty()) 
                {
                    RCLCPP_ERROR(_node->get_logger(), "Parameter 'hw.joint_names' is empty. Please set it in YAML.");
                }

                /* set default gains */
                _actuator_interface->set_gains(_default_kps, _default_kds);

                /* set joint names */
                _actuator_interface->set_joint_names(joint_names);
                _robot_interface->set_joint_names(joint_names);

                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_3S_SIG:
            {
                /* transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }
            
        }

        return status;
    }

    Status PlannerStateMachine::configuration_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Configuration state");
                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                /* all sensors have been received */
                // auto joint_names = _robot_interface->get_joint_names();
                // _actuator_interface->set_joint_names(joint_names);
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_3S_SIG:
            {
               if(_robot_interface->is_ready())
               {
                    _state = (fsm::FSM::StateHandler)&PlannerStateMachine::pre_planning_state;
                    status = fsm::Status::TRAN_STATUS;
                    RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: All sensors OK!");
                    break;
               }
               else
               {
                    status = fsm::Status::HANDLED_STATUS;
                    RCLCPP_WARN(_node->get_logger(), "PlannerStateMachine: Sensor non-OK!");
                    break;
               }
                // _state = (fsm::FSM::StateHandler)&PlannerStateMachine::planning_state;
                // status = fsm::Status::TRAN_STATUS;
                // break;
            }
        }

        return status;
    }

    Status PlannerStateMachine::pre_planning_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Pre Planning state");
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Moving to zeros state");
                _start_timer();
                _counter = 0;
                _time_end = 7; // seconds
                _max_counter = static_cast<uint16_t>(_time_end / _sampling_time);
                // std::fill(
                //     _first_joint_positions_cmds.begin(), 
                //     _first_joint_positions_cmds.end(),
                //     0.0f
                // );
                _first_joint_positions_cmds.reserve(10);
                std::copy(
                    _default_joint_pos.begin(),
                    _default_joint_pos.end(),
                    _first_joint_positions_cmds.begin()
                );

                // RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Pre Planning state: OK");

                _first_joint_pos = _robot_interface->get_joint_positions();


                /* set default gains */
                _actuator_interface->set_gains(_default_kps, _default_kds);

                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                _counter++; // increment until reach time target
                // RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Pre Planning state: OK 5MS");
                
                std::vector<float> joint_pos_cmds; joint_pos_cmds.resize(_first_joint_pos.size());
                float alpha = static_cast<float>(_counter) / _max_counter;
                for(size_t index = 0; index < _first_joint_pos.size(); index++)
                {
                    joint_pos_cmds[index] = (1 - alpha) * _first_joint_pos[index] + alpha * _first_joint_positions_cmds[index];
                }

                /* Send the filtered command to the actuator interface */
                _actuator_interface->send_command(joint_pos_cmds);
                _actuator_interface->send_command_debug(joint_pos_cmds);

                if(_counter < _max_counter)
                {
                    status = fsm::Status::HANDLED_STATUS;
                    break;
                }
                _counter = 0; // reset wait counter
           

                /* First configuration of the Planner plan is reached, transient to planning state*/
               _state = (fsm::FSM::StateHandler)&PlannerStateMachine::planning_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }

        }

        return status;
    }    

    Status PlannerStateMachine::planning_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Planning state");
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Waiting for Starting RL!");
                _start_timer();
                _rl_tick = 0;
                _rl_decim = static_cast<uint16_t>( (1.0 / _sampling_time) / _rl->cfg()->control_rate_hz) + 1;
                _rl->reset();
                _start_rl = false;
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::START_BUTTON_3S:
            {
                status = fsm::Status::HANDLED_STATUS;
                _start_rl = true; 

                std::string msg = _is_walk == true ? "Walk" : "Stand";
                RCLCPP_INFO(_node->get_logger(), "RL enabled!");
                RCLCPP_INFO(_node->get_logger(), "RL is %s", msg.c_str());
                break;
            }

            case Signal::BACK_BUTTON_PRESSED:
            {
                status = fsm::Status::HANDLED_STATUS;
                _is_walk = _is_walk ^ true; 

                std::string msg = _is_walk == true ? "Walk" : "Stand";
                RCLCPP_INFO(_node->get_logger(), "RL is %s", msg.c_str());
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                // wait for start button
                if(!_start_rl)
                {
                    status = fsm::Status::HANDLED_STATUS;
                    break;
                }

                // check safety    
                ROSDataInterface::SafetyReport report = _robot_interface->is_safe();
                if(!report.is_safe)
                {
                    for(auto& error_msg : report.error_msgs) RCLCPP_WARN(_node->get_logger(), error_msg.c_str());
                    status = fsm::Status::TRAN_STATUS;
                    _state = (fsm::FSM::StateHandler)&PlannerStateMachine::finished_state;
                    break;
                }

                // check all sensor data available
                // if(!_rl || !_rl->is_ready())
                // {
                //     status = fsm::Status::HANDLED_STATUS;
                //     break;
                // }

                // check timing for rl policy
                _rl_tick++;
                if(_rl_tick % _rl_decim != 0)
                {
                    status = fsm::Status::HANDLED_STATUS;
                    break;
                }


                /* Normal case, perform planning
                    Compute joint commands and send joint commands
                */                
                auto q = _robot_interface->get_joint_positions();
                auto dq = _robot_interface->get_joint_velocities();
                static bool is_first = true;
                static std::vector<float> filtered_qcmds(_rl->cfg()->action_dim, 0.0f);
                static LPF qcmds_filter(3.5, 1.0f/_rl->cfg()->control_rate_hz, _rl->cfg()->action_dim);

                // change hw to rl joint ordered name list
                static auto hw_joint_names = _robot_interface->get_joint_names();
                static auto rl_joint_names = _rl->cfg()->joint_names;
                static auto hw_index = make_index(hw_joint_names);
                static auto rl_to_hw  = indices_of(rl_joint_names, hw_index, -1);
                q = gather<float>(q, rl_to_hw, 0.0);  
                dq = gather<float>(dq, rl_to_hw, 0.0);

                if (is_first)
                {
                    filtered_qcmds = q;
                    is_first = false;
                }

                RLPolicy::Inputs obs;
                obs.base_ang_vel = _robot_interface->get_base_angular_velocity();
                obs.projected_gravity = _robot_interface->get_proj_g();
                obs.v_cmds = _robot_interface->get_raw_joystick_cmds();
                obs.q = q;
                obs.dq = dq;

                _is_walk = obs.v_cmds[0] > 0.1f ? false : true;
                if(!_is_walk)
                {
                    obs.v_cmds = std::vector<float>{0.0f, 0.0f, 0.0f};
                    obs.gait_parameters = std::vector<float>{0.0f, 0.0f, 1.0f, 0.0f};
                }
                else 
                {
                    obs.gait_parameters = std::vector<float>{1.5f, 0.5f, 0.5f, 0.15f};
                }


                std::vector<float> joint_position_cmds = _rl->run(obs);


                // change joint order from rl to hw
                joint_position_cmds = scatter_rl_to_hw<float>(joint_position_cmds, rl_to_hw, hw_joint_names.size(), 0.0);
                static auto rl_kps = _rl->cfg()->rl_kps;
                static auto rl_kds = _rl->cfg()->rl_kds;
                _actuator_interface->set_gains(rl_kps, rl_kds);
                _actuator_interface->send_command(joint_position_cmds);
                _actuator_interface->send_command_debug(joint_position_cmds);
                status = fsm::Status::HANDLED_STATUS;
                break;
            }
        }

        return status;
    }

    Status PlannerStateMachine::finished_state(const Event *const event)
    {
        Status status = fsm::Status::IGNORED_STATUS;
        switch (event->signal)
        {
            case Signal::ENTRY_SIG:
            {
                RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Finished state");

                /* Abnormal stated detected! */
                // stop all motor
                static std::vector<float> zeros_kp(_robot_interface->get_joint_names().size(), 0.0f);
                static std::vector<float> zeros_kd(_robot_interface->get_joint_names().size(), 0.0f);
                static std::vector<float> zeros_joint_pos_cmds(_robot_interface->get_joint_names().size(), 0.0f);
                _actuator_interface->set_gains(zeros_kp, zeros_kd);
                _actuator_interface->send_command(zeros_joint_pos_cmds);

                _start_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            // case Signal::TIMEOUT_1S_SIG:
            // {
            //     /* Transient to configuration state */
            //     _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
            //     status = fsm::Status::TRAN_STATUS;
            //     break;
            // }
            case Signal::START_BUTTON_3S:
            {
                /* Transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::TRAN_STATUS;
                break;
            }
        }

        return status;
    }
}