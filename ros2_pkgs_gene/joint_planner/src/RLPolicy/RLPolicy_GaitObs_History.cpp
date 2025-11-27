#include "RLPolicy_GaitObs_History.h"

#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace 
{
    constexpr float PI = 3.14159265358979323846f;
}

RLPolicy::RLPolicy(const rclcpp::Node::SharedPtr node) : _node(node)
{
    _load_model_config();

    // ORT session options
    _opts.SetIntraOpNumThreads(1);
    _opts.SetInterOpNumThreads(1);

    // Load model
    _ready = _load_model(_cfg.policy_file_path);
    RCLCPP_INFO(_node->get_logger(), "RLPolicy: model load %s (%s)", _ready ? "ok" : "failed", _cfg.policy_file_path.c_str());

    // Reset model internal variables
    reset();

    RCLCPP_INFO(node->get_logger(), "Initialize RL Done!");
}

std::vector<float> RLPolicy::run(const Inputs& in)
{
    _compute_observation(in);
    _run_onnx();
    return _preprocess_actions();
}

void RLPolicy::reset()
{
    _prev_action.assign(static_cast<size_t>(_cfg.action_dim), 0.0f);
    _action.assign(static_cast<size_t>(_cfg.action_dim), 0.0f);
}

void RLPolicy::_load_model_config()
{       
    // core
    _node->declare_parameter("model.action_dim", 0);
    _node->get_parameter("model.action_dim", _cfg.action_dim);

    _node->declare_parameter("model.observation_dim", 0);
    _node->get_parameter("model.observation_dim", _cfg.observation_dim);

    _cfg.observation_history_length = _node->declare_parameter("model.observation_history_length", 0);
    _node->get_parameter("model.observation_history_length", _cfg.observation_history_length);

    std::vector<double> tmp(_cfg.action_dim, 0.0f);
    _node->declare_parameter("model.action_scale", std::vector<double>(_cfg.action_dim, 1.0f));
    _node->get_parameter("model.action_scale", tmp);
    _cfg.action_scales.reserve(_cfg.action_dim);
    _cfg.action_scales.assign(tmp.begin(), tmp.end());
    
    _node->declare_parameter("model.default_joint_positions", std::vector<double>(_cfg.action_dim, 0.0f));
    _node->get_parameter("model.default_joint_positions", tmp);
    _cfg.default_joint_positions.reserve(_cfg.action_dim);
    _cfg.default_joint_positions.assign(tmp.begin(), tmp.end());

    _node->declare_parameter("model.joint_names", std::vector<std::string>{});
    _node->get_parameter("model.joint_names", _cfg.joint_names);
    _cfg.policy_file_path = _node->declare_parameter("model.policy_file_path", std::string(""));
    _node->get_parameter("model.policy_file_path", _cfg.policy_file_path);
    
    _node->declare_parameter("model.gains.kps", std::vector<double>(_cfg.action_dim, 0.0f));
    _node->get_parameter("model.gains.kps", tmp);
    _cfg.rl_kps.reserve(_cfg.action_dim);
    _cfg.rl_kps.assign(tmp.begin(), tmp.end());

    _node->declare_parameter("model.gains.kds", std::vector<double>(_cfg.action_dim, 0.0f));
    _node->get_parameter("model.gains.kds", tmp);
    _cfg.rl_kds.reserve(_cfg.action_dim);
    _cfg.rl_kds.assign(tmp.begin(), tmp.end());

    // limits
    _node->declare_parameter("model.cmd_limits.max_linear_velocity_x", 0.0);
    _node->get_parameter("model.cmd_limits.max_linear_velocity_x", _cfg.max_linear_velocity_x);
    _node->declare_parameter("model.cmd_limits.max_linear_velocity_y", 0.0);
    _node->get_parameter("model.cmd_limits.max_linear_velocity_y", _cfg.max_linear_velocity_y);
    _node->declare_parameter("model.cmd_limits.max_angular_velocity", 0.0);
    _node->get_parameter("model.cmd_limits.max_angular_velocity", _cfg.max_angular_velocity);

    // io names
    _node->declare_parameter("model.io.obs_name",    std::string("obs"));
    _node->get_parameter("model.io.obs_name", _cfg.obs_name);
    _node->declare_parameter("model.io.action_name", std::string("actions"));
    _node->get_parameter("model.io.action_name", _cfg.action_name);

    // timing (for phase auto-advance)
    _node->declare_parameter("model.control_rate", 50.0);
    _node->get_parameter("model.control_rate", _cfg.control_rate_hz);
}

bool RLPolicy::_load_model(const std::string& onnx_path)
{
    try 
    {
        _session = std::make_unique<Ort::Session>(_env, onnx_path.c_str(), _opts);
        _obs_name = _cfg.obs_name;
        _action_name = _cfg.action_name;
        _prepare_obs_buffer();
        return true;
    }
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to load ONNX model: %s", e.what());
    } 
    catch (...) 
    {
        RCLCPP_ERROR(_node->get_logger(), "Failed to load ONNX model: unknown error");
    }
    _session.reset();
    _obs_tensor.reset();
    return false;
}

void RLPolicy::_prepare_obs_buffer()
{
    _input_obs.assign(static_cast<size_t>(_cfg.observation_dim), 0.0f);
    _obs_shape = {1, static_cast<int64_t>(_input_obs.size())};
    _obs_tensor = std::make_unique<Ort::Value>(
        Ort::Value::CreateTensor<float>(
            _mem_info,
            _input_obs.data(),
            _input_obs.size(),
            _obs_shape.data(),
            _obs_shape.size())
    );

    _input_names.clear();
    for(int index = 0; index < _obs_name.size(); index++)
    {
        _input_names.push_back(_obs_name.c_str() + index);
    }

    _output_names.clear();
    for(int index = 0; index < _action_name.size(); index++)
    {
        _output_names.push_back(_action_name.c_str() + index);
    }

    _action.reserve(_cfg.action_dim);
    _prev_action.reserve(_cfg.action_dim);

    /* circular buffers */
    uint8_t window_length = _cfg.observation_history_length;
    uint8_t num_obs_term = 8;
    for(size_t index = 0; index < num_obs_term; index++)
    {
        _buffers.push_back(boost::circular_buffer<std::vector<float>>(window_length));
    } 
}

void RLPolicy::_compute_observation(const Inputs& in)
{
    _input_obs.clear();
    _input_obs.reserve(_cfg.observation_dim);

    /*
        obs (example total 45):
        [ 
            base_ang_vel * 0.25 (3),
            projected_gravity (3),
            joint_pos_rel (10),
            joint_vel * 0.05 (10),
            last_action (10),
            vel_cmds [vx, vy, wz] scaled by max limits (3),
            gait_phase (2)
            gait_commands (4)
        ]    
    */
    std::vector<float> scaled_base_ang_vel; scaled_base_ang_vel.reserve(in.base_ang_vel.size());
    for(auto w : in.base_ang_vel) scaled_base_ang_vel.push_back(w * 0.25f);

    std::vector<float> scaled_projected_gravity; scaled_projected_gravity.reserve(in.projected_gravity.size());
    for(auto g : in.projected_gravity) scaled_projected_gravity.push_back(g);

    std::vector<float> scaled_q; scaled_q.reserve(in.q.size());
    for(size_t index = 0; index < in.q.size(); index++)
        scaled_q.push_back(in.q[index] - _cfg.default_joint_positions[index]);

    std::vector<float> scaled_dq; scaled_dq.reserve(in.dq.size());
    for(auto dq : in.dq) scaled_dq.push_back(dq * 0.05f);

    
    std::vector<float> scaled_v_cmds; scaled_v_cmds.reserve(in.v_cmds.size());
    static std::vector<float> cmd_scales{
        _cfg.max_linear_velocity_x,
        _cfg.max_linear_velocity_y,
        _cfg.max_angular_velocity
    };
    for(size_t index = 0; index < in.v_cmds.size(); index++) 
        scaled_v_cmds.push_back(in.v_cmds[index] * cmd_scales[index]);


    std::vector<float> scaled_prev_action; scaled_prev_action.reserve(_prev_action.size());
    scaled_prev_action = _prev_action;

    std::vector<float> scaled_gait_params; scaled_gait_params.reserve(in.gait_parameters.size());
    scaled_gait_params = in.gait_parameters;

    std::vector<float> gait_phase; gait_phase.reserve(2);
    const float control_dt = 1.0f / _cfg.control_rate_hz;
    float gait_index = std::fmod(_policy_counter * control_dt * in.gait_parameters[0], 1.0f);
    // if (gait_index < 0.0f) gait_index += 1.0f;
    float angle = 2.0f * static_cast<float>(M_PI) * gait_index;
    gait_phase.push_back(std::sin(angle)); // sin_phase
    gait_phase.push_back(std::cos(angle)); // cos_phase
    

    /* First push */
    std::array<std::vector<float>*, 8> inputs = {
        &scaled_base_ang_vel,
        &scaled_projected_gravity,
        &scaled_q,
        &scaled_dq,
        &scaled_prev_action,
        &scaled_v_cmds,
        &gait_phase,
        &scaled_gait_params
    };

    if(_buffers[0].empty())
    {
        for(size_t index = 0; index < 8; index++) _buffers[index].resize(_cfg.observation_history_length, *inputs[index]);
    }
    else
    {
        for(size_t index = 0; index < 8; index++) _buffers[index].push_back(std::move(*inputs[index]));
    }


    /* Flatten the buffer and fill to the _input_obs */
    for (const auto& buffer : _buffers) 
    {
        for (const auto& frame : buffer) 
        {
            _input_obs.insert(_input_obs.end(), frame.begin(), frame.end());
        }
    }

    /* Check obs size */
    if(_input_obs.size() != _cfg.observation_dim)
    {
        RCLCPP_ERROR(_node->get_logger(), "ORT run failed: obs dim mistmatch!");
    }

}

void RLPolicy::_run_onnx()
{
    static const char* in_names[]  = { _obs_name.c_str() };
    static const char* out_names[] = { _action_name.c_str() };
    std::vector<Ort::Value> input_values;
    input_values.push_back(
        Ort::Value::CreateTensor<float>(
            _mem_info, _input_obs.data(), _input_obs.size(),
            _obs_shape.data(), _obs_shape.size()
        )
    );

    Ort::RunOptions run_options;
    try
    {        
        std::vector<Ort::Value> outputs = _session->Run(
            run_options,
            in_names, input_values.data(), 1,
            out_names, 1
        );    

        for (size_t index = 0; index < _cfg.action_dim; index++)
        {
            _action[index] = *(outputs[0].GetTensorMutableData<float>() + index);
        }
    }
    catch (const std::exception& e) 
    {
        RCLCPP_ERROR(_node->get_logger(), "ORT run failed: %s", e.what());
    }
    catch (...) 
    {
        RCLCPP_ERROR(_node->get_logger(), "ORT run failed: unknown error");
    }
} 

std::vector<float> RLPolicy::_preprocess_actions()
{
    _policy_counter++;
    std::vector<float> processed_action(_cfg.action_dim, 0.0f);

    // Scale and offset by default joints
    for (size_t index = 0; index < _cfg.default_joint_positions.size(); index++)
        processed_action[index] = _action[index] * _cfg.action_scales[index] + _cfg.default_joint_positions[index];

    _prev_action = _action;
    return processed_action;
}
