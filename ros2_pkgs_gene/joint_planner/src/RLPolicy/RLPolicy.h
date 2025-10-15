#ifndef RL_POLICY_H
#define RL_POLICY_H

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <onnxruntime_cxx_api.h>
#include <rclcpp/rclcpp.hpp>

class RLPolicy 
{
    public:
        struct ModelConfig 
        {
            int action_dim{0};               // 10 dims
            int observation_dim{0};          // 41 dims
            float action_scale{1.0};
            std::vector<float> default_joint_postitions;
            std::vector<std::string> joint_names;
            std::vector<float> rl_kps;
            std::vector<float> rl_kds;

            // limits
            float max_linear_velocity_x{0.0};
            float max_linear_velocity_y{0.0};
            float max_angular_velocity{0.0};

            // model / io
            std::string policy_file_path;
            std::string obs_name{"obs"};
            std::string action_name{"action"};

            // timing for internal phase update
            float control_rate_hz{50.0};    // 
            float gait_period_s{0.8};       // default period (s)
        };

        struct Inputs 
        {
            std::vector<float> base_ang_vel{0,0,0};     // scaled 0.25 in obs
            std::vector<float> projected_gravity{0,0,-1};
            std::vector<float> v_cmds{0,0,0};         // [vx, vy, wz]
            std::vector<float> q;                        // 10
            std::vector<float> dq;                       // 10 (scaled 0.05 in obs)
        };

        explicit RLPolicy(const rclcpp::Node::SharedPtr node);

    public:
        std::vector<float> run(const Inputs& in);

        bool is_ready() const { return _ready; }
        void reset();
        ModelConfig cfg() const { return _cfg; }

        // Phase is internal; allow only explicit set
        void set_phase(float phase);  // set to [0,1)

    private:
        rclcpp::Node::SharedPtr _node;

        ModelConfig _cfg;
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
        std::vector<float> _prev_action;
        std::vector<float> _input_obs;
        std::vector<float> _h_in_obs;
        std::unique_ptr<Ort::Value> _obs_tensor;
        std::unique_ptr<Ort::Value> _h_in_tensor;
        std::array<int64_t, 2> _obs_shape{1, 0};
        std::array<int64_t, 3> _h_in_shape{1, 1, 64};

        // Internal phase in [0,1)
        float _phase{0.f};
        // Per-tick phase increment: dphi = (1/control_rate)/gait_period
        float _dphi_tick{0.f};

    private:
        ModelConfig _load_model_config(rclcpp::Node* node);
        bool _load_model(const std::string& onnx_path);

        void _prepare_obs_buffer();
        void _compute_observation(const Inputs& in);

        std::vector<float> _run_onnx();
        std::vector<float> _preprocess_actions(const std::vector<float> raw);

        // advance and wrap phase each tick
        inline void _advance_phase();
};

#endif // RL_POLICY_H
