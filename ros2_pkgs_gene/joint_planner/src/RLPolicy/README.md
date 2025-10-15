# RLPolicy (ONNX)

A minimal ONNX Runtime wrapper to execute a learned policy that outputs joint position commands.

Inputs (concatenated):
- q: joint positions (Nq)
- dq: joint velocities (Nq)
- pose_cmd: task/pose command (Np)
- prev_action: previous action (Nq)

Output:
- joint position commands (Nq)

Parameters:
- rl_policy.model_path (string): path to .onnx model. If empty or ONNX Runtime not found, a stub returns prev_action (hold) or zeros.

CMake:
- Set environment variable ONNXRUNTIME_ROOT to your install prefix containing include/ and lib/ for ONNX Runtime.

Example env on macOS (Homebrew):
- export ONNXRUNTIME_ROOT="/opt/homebrew/opt/onnxruntime"
