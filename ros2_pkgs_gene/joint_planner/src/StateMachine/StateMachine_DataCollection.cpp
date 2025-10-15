#include "StateMachine.h"

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
                _node->declare_parameter("default_gains.kps", std::vector<double>(_rl->cfg().action_dim, 0.0f));
                std::vector<double> tmp;
                _default_kps.reserve(_rl->cfg().action_dim);
                _node->get_parameter("default_gains.kps", tmp);
                _default_kps.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("default_gains.kds", std::vector<double>(_rl->cfg().action_dim, 0.0f));
                _default_kds.reserve(_rl->cfg().action_dim);
                _node->get_parameter("default_gains.kds", tmp);
                _default_kds.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("positions.standing", std::vector<double>(_rl->cfg().action_dim, 0.0f));
                _standing_joint_pos.reserve(_rl->cfg().action_dim);
                _node->get_parameter("positions.standing", tmp);
                _standing_joint_pos.assign(tmp.begin(), tmp.end());

                _node->declare_parameter("positions.default", std::vector<double>(_rl->cfg().action_dim, 0.0f));
                _default_joint_pos.reserve(_rl->cfg().action_dim);
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
                _planner->compute(_sampling_time); // call the compute to get the first position in the trajectory, saved internally
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
                    break;
               }
               else
               {
                    status = fsm::Status::HANDLED_STATUS;
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
                _start_timer();
                _counter = 0;
                _time_end = 7; // seconds
                _max_counter = static_cast<uint16_t>(_time_end / _sampling_time);

                _planner->compute(_sampling_time); 
                _first_joint_positions_cmds = _planner->get_actions(); //! shape 5

                // _first_joint_positions_cmds.reserve(10);
                // std::fill(
                //     _first_joint_positions_cmds.begin(), 
                //     _first_joint_positions_cmds.end(),
                //     0.0f
                // );

                // _first_joint_positions_cmds.reserve(10);
                // std::copy(
                //     _default_joint_pos.begin(),
                //     _default_joint_pos.end(),
                //     _first_joint_positions_cmds.begin()
                // );

                // RCLCPP_INFO(_node->get_logger(), "PlannerStateMachine: Pre Planning state: OK");

                _first_joint_pos = _robot_interface->get_joint_positions(); //! shape 10


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
                
                std::vector<float> joint_pos_cmds(10, 0.0); //! shape 10
                float alpha = static_cast<float>(_counter) / _max_counter;
                static const std::vector<uint8_t> observered_joint_ids = {0, 1, 2, 3, 9}; // left leg joints
                // for(size_t index = 0; index < _first_joint_pos.size(); index++)
                // {
                //     joint_pos_cmds[index] = (1 - alpha) * _first_joint_pos[index] + alpha * _first_joint_positions_cmds[index];
                // }
                for(size_t index = 0; index < observered_joint_ids.size(); index++)
                {
                    joint_pos_cmds[observered_joint_ids[index]] = (1 - alpha) * _first_joint_pos[observered_joint_ids[index]] + alpha * _first_joint_positions_cmds[index];
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
                _start_timer();
                _counter = 0.0;
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                _counter = 0.0;
                _start_rl = false;
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::START_BUTTON_3S:
            {
                status = fsm::Status::HANDLED_STATUS;
                _start_rl = true; 
                break;
            }

            case Signal::TIMEOUT_5MS_SIG:
            {
                if(!_start_rl)
                {
                    status = fsm::Status::HANDLED_STATUS;
                    break;
                }

                /* Eclipsoid trajectory */
                if(_planner->is_trajectory_completed())
                {
                    /* Planner plan for 1 trajectory is reached, transient to finished state */
                    _state = (fsm::FSM::StateHandler)&PlannerStateMachine::finished_state;
                    status = fsm::Status::TRAN_STATUS;
                    break;
                }


                /* Normal case, perform planning
                    Compute joint commands and send joint commands
                */                
                std::vector<float> joint_position_cmds = _planner->compute(_sampling_time);

                /* Chirp signal */
                // static float t0 = _counter * _sampling_time;
                // float t = _counter * _sampling_time;
                // static float f0 = 0.01;
                // static float fT = 10.0;
                // static float T = 20.0;
                // static std::vector<float> A_min = {-0.1, -0.2, 0.3, -1.3, 0.25};
                // static std::vector<float> A_max = {0.1, 0.2, 0.95, -0.7, 0.7};
                // // ! joint_position_cmds shape = 5
                // std::vector<float> joint_position_cmds = planner::chirp_signal(
                //     t0, 
                //     t,
                //     T, 
                //     f0,
                //     fT, 
                //     A_min,
                //     A_max
                // );


                /* Check valid joint cmds */
                bool is_nan = std::any_of(joint_position_cmds.begin(), joint_position_cmds.end(), [](float val) { return std::isnan(val); });
                bool is_inf = std::any_of(joint_position_cmds.begin(), joint_position_cmds.end(), [](float val) { return std::isinf(val); });
                if(is_nan || is_inf)
                {
                    RCLCPP_ERROR(_node->get_logger(), "PlannerStateMachine: Joint position commands contain NaN or Inf values");
                    status = fsm::Status::IGNORED_STATUS;
                    break;
                }

                // /* Update counter - Chirp signal*/
                // _counter++;
                // if(t > T)
                // {
                //     /* Planner plan for 1 trajectory is reached, transient to finished state */
                //     _state = (fsm::FSM::StateHandler)&PlannerStateMachine::finished_state;
                //     status = fsm::Status::TRAN_STATUS;
                //     break;
                // }

                /* Send the filtered command to the actuator interface */
                static const std::vector<uint8_t> observered_joint_ids = {0, 1, 2, 3, 9}; // left leg joints
                static std::vector<float> hw_joint_position_cmds(10, 0.0); //! shape 10
                for(size_t index = 0; index < observered_joint_ids.size(); index++)
                {
                    hw_joint_position_cmds[observered_joint_ids[index]] = joint_position_cmds[index];
                }
                
                _actuator_interface->send_command(hw_joint_position_cmds);
                _actuator_interface->send_command_debug(hw_joint_position_cmds);
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
                _start_timer();
                _planner->change_trajectory();
                status = fsm::Status::HANDLED_STATUS;
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::EXIT_SIG:
            {
                _stop_timer();
                status = fsm::Status::HANDLED_STATUS;
                break;
            }

            case Signal::TIMEOUT_1S_SIG:
            {
                /* Transient to configuration state */
                _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
                status = fsm::Status::HANDLED_STATUS;
                break;
            }
            // case Signal::START_BUTTON_3S:
            // {
            //     /* Transient to configuration state */
            //     _state = (fsm::FSM::StateHandler)&PlannerStateMachine::configuration_state;
            //     status = fsm::Status::TRAN_STATUS;
            //     break;
            // }
        }

        return status;
    }
}