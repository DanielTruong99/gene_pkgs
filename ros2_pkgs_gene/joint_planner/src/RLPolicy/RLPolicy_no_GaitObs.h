#ifndef RL_POLICY2_H
#define RL_POLICY2_H

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <unordered_map>
#include <tuple>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>

class RLPolicy 
{
public:
    struct ModelConfig 
    {

        int action_dim{0};
        int observation_dim{0};

        // limits
        float max_linear_velocity_x{0.0};
        float max_linear_velocity_y{0.0};
        float max_angular_velocity{0.0};

        // timing for internal phase update
        float control_rate_hz{50.0};
        
        // model / io
        std::string policy_file_path;
        std::string obs_name{"obs"};
        std::string action_name{"action"};
        std::vector<std::string> joint_names;
        std::vector<float> action_scales;
        std::vector<float> default_joint_positions;
        std::vector<float> rl_kps;
        std::vector<float> rl_kds;
    };

    struct Inputs 
    {
        std::vector<float> base_ang_vel{0,0,0};     // scaled 0.25 in obs
        std::vector<float> projected_gravity{0,0,-1};
        std::vector<float> v_cmds{0,0,0};           // [vx, vy, wz] in [-1,1]
        std::vector<float> q;                        // 10
        std::vector<float> dq;                       // 10 (scaled 0.05 in obs)
        std::vector<float> gait_parameters{0.0f, 0.0f, 1.0f, 0.0f};
    };

    explicit RLPolicy(const rclcpp::Node::SharedPtr node);

public:
    std::vector<float> run(const Inputs& in);

    bool is_ready() const { return _ready; }
    void reset();
    ModelConfig* cfg() { return &_cfg; }

private:
    rclcpp::Node::SharedPtr _node;

    ModelConfig _cfg;
    std::vector<float> _action_scale;
    bool _ready{false};

    // ONNX Runtime
    Ort::Env _env{ORT_LOGGING_LEVEL_WARNING, "rl_policy"};
    Ort::SessionOptions _opts{};
    std::unique_ptr<Ort::Session> _session;
    Ort::MemoryInfo _mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    // I/O names
    std::string _obs_name{"obs"};
    std::string _action_name{"action"};

    // Buffers/tensors
    std::vector<const char *> _input_names;
    std::vector<const char *> _output_names;
    std::vector<float> _prev_action;
    std::vector<float> _action;
    std::vector<float> _input_obs;
    std::unique_ptr<Ort::Value> _obs_tensor;
    std::array<int64_t, 2> _obs_shape;


private:
    void _load_model_config();
    bool _load_model(const std::string& onnx_path);

    void _prepare_obs_buffer();
    void _compute_observation(const Inputs& in);

    void _run_onnx();
    std::vector<float> _preprocess_actions();
};

#endif // RL_POLICY2_H
