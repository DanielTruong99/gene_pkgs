#include "RLPolicy.h"

#include <algorithm>
#include <stdexcept>
#include <cmath>

namespace 
{
    constexpr float PI = 3.14159265358979323846f;
}

RLPolicy::RLPolicy(const rclcpp::Node::SharedPtr node) : _node(node)
{
    _cfg = _load_model_config(_node.get());


    if (_cfg.control_rate_hz > 0.0 && _cfg.gait_period_s > 0.0) 
    {
        double dt = 1.0 / _cfg.control_rate_hz;
        _dphi_tick = static_cast<float>(dt / _cfg.gait_period_s);
    } 
    else 
    {
        _dphi_tick = 0.f;
    }

    // ORT session options
    _opts.SetIntraOpNumThreads(1);
    _opts.SetInterOpNumThreads(1);
    _opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    _opts.DisableMemPattern();

    _ready = _load_model(_cfg.policy_file_path);
    RCLCPP_INFO(_node->get_logger(), "RLPolicy: model load %s (%s)", _ready ? "ok" : "failed", _cfg.policy_file_path.c_str());

    // Reset model internal variables
    reset();
}

std::vector<float> RLPolicy::run(const Inputs& in)
{
    // Check initialization fail
    if (!_ready || !_session) return std::vector<float>(static_cast<size_t>(_cfg.action_dim), 0.0f);


    // Pre process
    _compute_observation(in);

    // Compute
    std::vector<float> raw = _run_onnx();

    // Post process
    _prev_action = raw;
    std::vector<float> act = _preprocess_actions(raw);

    _advance_phase();
    return act;
}

void RLPolicy::reset()
{
    _prev_action.assign(static_cast<size_t>(_cfg.action_dim), 0.0f);
    if (!_input_obs.empty()) std::fill(_input_obs.begin(), _input_obs.end(), 0.0f);
    if (!_h_in_obs.empty())  std::fill(_h_in_obs.begin(),  _h_in_obs.end(),  0.0f);
    _phase = 0.f;
}

void RLPolicy::set_phase(float phase)
{
    // wrap into [0,1)
    float p = std::fmod(std::max(phase, 0.f), 1.f);
    _phase = (p < 0.f) ? p + 1.f : p;
}

RLPolicy::ModelConfig RLPolicy::_load_model_config(rclcpp::Node* node)
{
    ModelConfig cfg;
    // core
    cfg.action_dim       = node->declare_parameter("model.action_dim", 0);
    cfg.observation_dim  = node->declare_parameter("model.observation_dim", 0);
    cfg.action_scale     = node->declare_parameter("model.action_scale", 1.0);
    node->declare_parameter("model.default_joint_postitions", std::vector<double>(cfg.action_dim, 0.0f));
    std::vector<double> tmp;
    cfg.default_joint_postitions.reserve(cfg.action_dim);
    node->get_parameter("model.default_joint_postitions", tmp);
    cfg.default_joint_postitions.assign(tmp.begin(), tmp.end());
    cfg.joint_names      = node->declare_parameter("model.joint_names", std::vector<std::string>{});
    cfg.policy_file_path = node->declare_parameter("model.policy_file_path", std::string(""));
    
    node->declare_parameter("model.gains.kps", std::vector<double>(cfg.action_dim, 0.0f));
    cfg.rl_kps.reserve(cfg.action_dim);
    node->get_parameter("model.gains.kps", tmp);
    cfg.rl_kps.assign(tmp.begin(), tmp.end());

    node->declare_parameter("model.gains.kds", std::vector<double>(cfg.action_dim, 0.0f));
    cfg.rl_kds.reserve(cfg.action_dim);
    node->get_parameter("model.gains.kds", tmp);
    cfg.rl_kds.assign(tmp.begin(), tmp.end());

    // limits
    cfg.max_linear_velocity_x = node->declare_parameter("model.cmd_limits.max_linear_velocity_x", 0.0);
    cfg.max_linear_velocity_y = node->declare_parameter("model.cmd_limits.max_linear_velocity_y", 0.0);
    cfg.max_angular_velocity  = node->declare_parameter("model.cmd_limits.max_angular_velocity", 0.0);

    // io names
    cfg.obs_name    = node->declare_parameter("model.io.obs_name",    std::string("obs"));
    cfg.action_name = node->declare_parameter("model.io.action_name", std::string("actions"));

    // timing (for phase auto-advance)
    cfg.control_rate_hz = node->declare_parameter("model.control_rate", 50.0);
    cfg.gait_period_s   = node->declare_parameter("model.gait_period",  0.8);
    return cfg;
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

    _h_in_obs.assign(64, 0.0f);
    _h_in_shape = {1, 1, 64};
    _h_in_tensor = std::make_unique<Ort::Value>(
        Ort::Value::CreateTensor<float>(
            _mem_info,
            _h_in_obs.data(),
            _h_in_obs.size(),
            _h_in_shape.data(),
            _h_in_shape.size())
    );
}

void RLPolicy::_compute_observation(const Inputs& in)
{
    // sanaty check
    if (_input_obs.size() != static_cast<size_t>(_cfg.observation_dim)) _prepare_obs_buffer();


    /*
        obs (total 41):
        [ 
            base_ang_vel * 0.25 (3),
            projected_gravity (3),
            vel_cmds [vx, vy, wz] (3),
            joint_pos_rel (10),
            joint_vel * 0.05 (10),
            last_action (10),
            sin(2πφ), cos(2πφ) (2) 
        ]    
    */
    size_t off = 0;
    auto cat = [&](const float* data, size_t n)
    {
        if (off + n > _input_obs.size()) return;
        std::copy(data, data + n, _input_obs.begin() + static_cast<long>(off));
        off += n;
    };
    auto cat_vec = [&](const std::vector<float>& v){ cat(v.data(), v.size()); };

    // base_ang_vel * 0.25
    std::vector<float> w_scaled = {
        in.base_ang_vel[0] * 0.25f,
        in.base_ang_vel[1] * 0.25f,
        in.base_ang_vel[2] * 0.25f
    };
    cat_vec(w_scaled);

    // projected_gravity
    std::vector<float> gvec = { in.projected_gravity[0], in.projected_gravity[1], in.projected_gravity[2] };
    cat_vec(gvec);

    // pose_cmd [vx, vy, wz]
    std::vector<float> cmds = { 
        in.v_cmds[0] * _cfg.max_linear_velocity_x, 
        in.v_cmds[1] * _cfg.max_linear_velocity_y, 
        in.v_cmds[2] * _cfg.max_angular_velocity 
    };
    cat_vec(cmds);

    // joint_pos_rel (q)
    std::vector<float> q_rel(in.q.begin(), in.q.end());
    for(auto index = 0; index < q_rel.size(); index++) q_rel[index] = q_rel[index] - _cfg.default_joint_postitions[index];
    cat_vec(q_rel);

    // joint_vel * 0.05
    std::vector<float> dq_scaled(in.dq.begin(), in.dq.end());
    for (auto& v : dq_scaled) v = v * 0.05f;
    cat_vec(dq_scaled);

    // last_action
    if (_prev_action.size() != static_cast<size_t>(_cfg.action_dim))
        _prev_action.assign(static_cast<size_t>(_cfg.action_dim), 0.0f);
    cat_vec(_prev_action);

    // phase sin/cos
    float s = std::sin(2.f * PI * _phase);
    float c = std::cos(2.f * PI * _phase);
    float phase2[2] = { s, c };
    cat(phase2, 2);
}

std::vector<float> RLPolicy::_run_onnx()
{
    const char* in_names[]  = { _obs_name.c_str(), "h_in" };
    const char* out_names[] = { _action_name.c_str(), "h_out" };

    // Build input tensors (wrapping existing buffers; no copy of data)
    Ort::Value obs  = Ort::Value::CreateTensor<float>(
        _mem_info, _input_obs.data(), _input_obs.size(),
        _obs_shape.data(), _obs_shape.size());

    Ort::Value h_in = Ort::Value::CreateTensor<float>(
        _mem_info, _h_in_obs.data(), _h_in_obs.size(),
        _h_in_shape.data(), _h_in_shape.size());

    // Put them in a contiguous array (move-only values)
    std::array<Ort::Value, 2> inputs{ std::move(obs), std::move(h_in) };

    try {
        auto outputs = _session->Run(
            Ort::RunOptions{nullptr},
            in_names,  inputs.data(), inputs.size(),
            out_names, 2
        );

        if (outputs.size() < 2 || !outputs[0].IsTensor() || !outputs[1].IsTensor()) {
            return std::vector<float>(static_cast<size_t>(_cfg.action_dim), 0.0f);
        }

        // actions: [1, 10]
        const Ort::Value& o_act = outputs[0];
        auto act_info = o_act.GetTensorTypeAndShapeInfo();
        size_t act_n = act_info.GetElementCount();
        const float* act_ptr = o_act.GetTensorData<float>();

        std::vector<float> act(static_cast<size_t>(_cfg.action_dim), 0.0f);
        std::copy(act_ptr, act_ptr + std::min(act_n, act.size()), act.begin());

        // h_out -> next h_in
        const Ort::Value& o_h = outputs[1];
        auto h_info = o_h.GetTensorTypeAndShapeInfo();
        size_t h_n = h_info.GetElementCount(); // expect 64
        const float* h_ptr = o_h.GetTensorData<float>();
        std::copy(h_ptr, h_ptr + std::min(h_n, _h_in_obs.size()), _h_in_obs.begin());

        return act;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(_node->get_logger(), "ORT run failed: %s", e.what());
    }
    catch (...) {
        RCLCPP_ERROR(_node->get_logger(), "ORT run failed: unknown error");
    }
    return std::vector<float>(static_cast<size_t>(_cfg.action_dim), 0.0f);
}

std::vector<float> RLPolicy::_preprocess_actions(const std::vector<float> raw)
{
    // Check size
    std::vector<float> out = raw;
    if (out.size() != static_cast<size_t>(_cfg.action_dim)) out.assign(static_cast<size_t>(_cfg.action_dim), 0.0f);

    // Scale action
    std::transform(
        _cfg.default_joint_postitions.begin(), 
        _cfg.default_joint_postitions.end(), 
        out.begin(),
        out.begin(),
        [&](auto& default_pos, auto& action)
        {
            return action * _cfg.action_scale + default_pos;
        }
    );

    // If all zero, there maybe error somewhere, keep the previous action
    bool all_zero = std::all_of(out.begin(), out.end(), [](float v){ return v == 0.0f; });
    if (all_zero && !_prev_action.empty()) return _prev_action;
    return out;
}

inline void RLPolicy::_advance_phase()
{
    _phase = _phase + _dphi_tick;
    // wrap into [0,1)
    _phase = _phase - std::floor(_phase);
}
