/**************************************************
---------------- Pre-processing -------------------
**************************************************/
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include <fcntl.h>
#include <sys/mman.h>

#include <eigen3/Eigen/Dense>

#include "ros/ros.h"

/* ---- Xenomai 3 ---- */
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>

/* ---- EtherCAT ---- */
#include "Whole_EtherCAT_DC_2.h"

/**************************************************
---------------- Project Configuration ------------
**************************************************/
// These should come from your EtherCAT header:
#ifndef NUM_ELMO
#  define NUM_ELMO 8
#endif
#ifndef NUM_MY
#  define NUM_MY 2
#endif
#ifndef NUM_ACT
#  define NUM_ACT (NUM_ELMO + NUM_MY)
#endif

namespace cfg 
{
    constexpr double kPi  = 3.14159265358979323846;
    constexpr double kPi2 = 2.0 * kPi;

    // Timing
    constexpr double kDt = 0.001; // 1kHz
    constexpr double kPrintDt = 0.010; // 100Hz

    // Hardware constants
    constexpr double kGearRatio = 10.0;
    constexpr double kEncoderElmo = 131072.0;   // 17-bit
    constexpr double kEncoderMy   = 262144.0;   // 18-bit

    // Elmo current model
    constexpr double kElmoTorqueConstant = 1.65; // Nm/A
    constexpr double kElmoRatedCurrent   = (45.0 * std::sqrt(2.0)); // A (peak)

    // MY current model
    constexpr double kMyTorqueConstant = 1.9; // Nm/A
    constexpr double kMyRatedCurrent   = 15.0; // A (assumed peak reference)

    // Per-joint torque limits (Nm)
    constexpr double kMaxTorque[NUM_ACT] = {80, 80, 80, 100, 80, 80, 80, 100, 28, 28};

    // Per-joint transmission factor (divide cmd/vel/pos by this if true)
    /*
        The motors already have the gearbox.
            kTransmission = 1.0, means the motor+gearbox attached to the joint directly.
            kTransmission > 1.0, means there is an additional transmission between the motor+gearbox and the joint.
        
        For example, the calf joints were drived by belt transmission with ratio 1.52.
        In addition, the toe joints were drived by 4-bar linkage mechanism with ratio 1.0.
    */
    constexpr double kTransmission[NUM_ACT] = 
    {
        1.0, 1.0, 1.0, 1.52, 1.0, 1.0, 1.0, 1.52, 1.0, 1.0
    };

    // Joint names
    static const char* kJointNames[NUM_ACT] = 
    {
        "R_hip_joint","R_hip2_joint","R_thigh_joint","R_calf_joint",
        "L_hip_joint","L_hip2_joint","L_thigh_joint","L_calf_joint",
        "L_toe_joint","R_toe_joint"
    };

    // Soft limits [rad]
    constexpr double kPosLimit[NUM_ACT][2] = 
    {
        {-0.52,  0.52},  // R_hip_yaw
        {-0.25,  0.52},  // R_hip_roll
        {-0.95,  0.95},  // R_hip_pitch
        {-1.50,  0.45},  // R_knee_pitch
        {-0.52,  0.52},  // L_hip_yaw
        {-0.25,  0.52},  // L_hip_roll
        {-0.95,  0.95},  // L_hip_pitch
        {-1.50,  0.45},  // L_knee_pitch
        {-0.30,  1.50},  // L_ankle_pitch
        {-0.30,  1.50}   // R_ankle_pitch
    };

    // Default PD gains
    constexpr double kKpDefault[NUM_ACT] = { 0,  0,   0,   0,  0,  0,  0,   0,   0,  0};
    constexpr double kKdDefault[NUM_ACT] = { 0,  0,   0,   0,  0,  0,  0,   0,   0,  0};
    constexpr double kKiDefault[NUM_ACT] = { 0,  0,   0,   0,  0,  0,  0,   0,   0,  0};

    // Feature toggles
    constexpr bool kPrintOn = true;

    // EtherCAT network interface
    static const char* kIfName = "rteth0";
} // namespace cfg

/**************************************************
----------------- Helpers & Utilities -------------
**************************************************/
template <typename T>
inline T clamp(T v, T lo, T hi) 
{
    return std::max(lo, std::min(v, hi));
}

inline bool in_range(double v, double lo, double hi) 
{
    return (v >= lo) && (v <= hi);
}


/**************************************************
---------------- Global State ---------------------
**************************************************/
static int g_run = 1;

/* Xenomai tasks */
RT_TASK task_joint_state_pub;
RT_TASK task_joint_cmd_sub;
RT_TASK task_control;
RT_TASK task_print;
RT_TASK task_save;
RT_TASK task_ecat;

/* EtherCAT runtime */
int expectedWKC;
volatile int wkc;
bool inOP;
uint8 currentgroup = 0;
EthercatMaster ethercatMaster;

/* PDO buffers (READ) */
uint16_t StatusWord[NUM_ELMO] = {0};
int32_t  ActualPos [NUM_ELMO] = {0};
int32_t  ActualVel [NUM_ELMO] = {0};
int16_t  ActualTor [NUM_ELMO] = {0};
uint8_t  ModeOfOperationDisplayElmo[NUM_ELMO] = {0};

uint16_t StatusWord_my[NUM_MY] = {0};
int32_t  ActualPos_my [NUM_MY] = {0};
int32_t  ActualVel_my [NUM_MY] = {0};
int16_t  ActualTor_my [NUM_MY] = {0};
uint8_t  ModeOfOperationDisplay_my[NUM_MY] = {0};

/* PDO buffers (WRITE) */
int32_t  TargetPos [NUM_ELMO] = {0};
int32_t  TargetVel [NUM_ELMO] = {0};
int16_t  TargetTor [NUM_ELMO] = {0};
uint16_t ControlWord[NUM_ELMO] = {0};

int32_t  TargetPos_my [NUM_MY] = {0};
int32_t  TargetVel_my [NUM_MY] = {0};
int16_t  TargetTor_my [NUM_MY] = {0};
uint16_t ControlWord_my[NUM_MY] = {0};

/* Controller state */
double Kp[NUM_ACT] = {0};
double Kd[NUM_ACT] = {0};
double Ki[NUM_ACT] = {0};

double error_old[NUM_ACT] = {0};
double P_term[NUM_ACT] = {0};
double I_term[NUM_ACT] = {0};
double D_term[NUM_ACT] = {0};

double pid_target[NUM_ACT] = {0};
double init_pos  [NUM_ACT] = {0};
bool   init_ok = false;

double posRad[NUM_ACT] = {0};
double velRad[NUM_ACT] = {0};
double torqueCmd[NUM_ACT] = {0};

double actualTorqueNm[NUM_ACT] = {0};
double raw_pos[NUM_ACT] = {0};
double raw_vel[NUM_ACT] = {0};
double raw_pid[NUM_ACT] = {0};

bool is_controller_ready = false;
bool is_planner_activated = false;
bool motor_off = false;


/**************************************************
---------------- Shared Memory Layout -------------
**************************************************/
struct MotorState 
{
    const char*  joint_names[NUM_ACT] = 
    {
        "R_hip_joint","R_hip2_joint","R_thigh_joint","R_calf_joint",
        "L_hip_joint","L_hip2_joint","L_thigh_joint","L_calf_joint",
        "L_toe_joint","R_toe_joint",
    };
    double position[NUM_ACT] = {0};
    double angular_velocity[NUM_ACT] = {0};
    double torque[NUM_ACT] = {0};
    double elapsed_time_ns = 0.0;
    bool   is_controller_ready = false;
};
using MotorCommand = struct MotorCommand 
{
    double position[NUM_ACT] = {0};
    double angular_velocity[NUM_ACT] = {0};
    double torque[NUM_ACT] = {0};
    double kp[NUM_ACT] = {0};
    double kd[NUM_ACT] = {0};
    bool   is_activated = false;
};

/* SHM config */
#define SHM_MOTOR_STATE_NAME "/motor_state"
#define SHM_MOTOR_CMD_NAME   "/motor_command"
#define SHM_MOTOR_STATE_SIZE sizeof(MotorState)
#define SHM_MOTOR_CMD_SIZE   sizeof(MotorCommand)

/**************************************************
---------------- Initialization -------------------
**************************************************/
void loadDefaultGains() 
{
    std::copy(std::begin(cfg::kKpDefault), std::end(cfg::kKpDefault), Kp);
    std::copy(std::begin(cfg::kKdDefault), std::end(cfg::kKdDefault), Kd);
    std::copy(std::begin(cfg::kKiDefault), std::end(cfg::kKiDefault), Ki);
}

inline double applyTransmissionIn(double v, int idx) 
{
    const double t = cfg::kTransmission[idx];
    return t == 1.0 ? v : v / t;
}
inline double applyTransmissionOut(double v, int idx) 
{
    const double t = cfg::kTransmission[idx];
    return t == 1.0 ? v : v / t; // dividing torque to motor-side if linkage amplifies
}

/**************************************************
---------------- Conversions ----------------------
**************************************************/
void convertUnits() 
{
    // ELMO
    for (int i = 0; i < NUM_ELMO; ++i) 
    {
        // Position [rad]
        raw_pos[i] = static_cast<double>(ActualPos[i]) / cfg::kEncoderElmo / cfg::kGearRatio * cfg::kPi2;
        posRad[i]  = applyTransmissionIn(raw_pos[i], i);

        // Velocity [rad/s]
        const double rpm = static_cast<double>(ActualVel[i]) / cfg::kEncoderElmo;
        raw_vel[i]       = rpm / cfg::kGearRatio * cfg::kPi2;
        velRad[i]        = applyTransmissionIn(raw_vel[i], i);

        // Commanded torque -> driver command
        const double current_A = torqueCmd[i] / cfg::kElmoTorqueConstant;
        const double permille  = (current_A / cfg::kElmoRatedCurrent) * 1000.0;
        TargetTor[i] = static_cast<int16_t>(permille);

        // Actual torque from driver counts
        actualTorqueNm[i] = (static_cast<double>(ActualTor[i]) / 1000.0) * cfg::kElmoRatedCurrent * cfg::kElmoTorqueConstant;
    }

    // MY
    for (int i = 0; i < NUM_MY; ++i) 
    {
        const int idx = i + NUM_ELMO;

        // Position [rad]
        raw_pos[idx] = static_cast<double>(ActualPos_my[i]) / cfg::kEncoderMy * cfg::kPi2;
        posRad[idx]  = raw_pos[idx];

        // Velocity [rad/s]
        const double rpm = static_cast<double>(ActualVel_my[i]) / cfg::kEncoderMy;
        raw_vel[idx]     = rpm * cfg::kPi2;
        velRad[idx]      = raw_vel[idx];

        // Commanded torque -> driver command
        const double current_A = torqueCmd[idx] / cfg::kMyTorqueConstant;
        const double permille  = (current_A / cfg::kMyRatedCurrent) * 1000.0;
        TargetTor_my[i] = static_cast<int16_t>(permille);

        // Actual torque
        actualTorqueNm[idx] = (static_cast<double>(ActualTor_my[i]) / 1000.0) * cfg::kMyRatedCurrent * cfg::kMyTorqueConstant;
    }
}

/**************************************************
---------------- Init Position Capture ------------
**************************************************/
bool allStatusOK_Elmo() 
{
    for (int i = 0; i < NUM_ELMO; ++i) if (StatusWord[i] != 4663) return false;
    return true;
}
bool allStatusOK_My() 
{
    for (int i = 0; i < NUM_MY; ++i) if (StatusWord_my[i] != 4663) return false;
    return true;
}
void captureInitialPositionsIfReady() 
{
    static bool elmo_init = false, my_init = false;

    if (!elmo_init && allStatusOK_Elmo()) 
    {
        for (int i = 0; i < NUM_ELMO; ++i) init_pos[i] = posRad[i];
        elmo_init = true;
        rt_printf("[Init] ELMO StatusWord = 4663 → InitPos saved\n");
    }
    if (!my_init && allStatusOK_My()) 
    {
        for (int i = 0; i < NUM_MY; ++i) init_pos[i + NUM_ELMO] = posRad[i + NUM_ELMO];
        my_init = true;
        rt_printf("[Init] MY StatusWord = 4663 → InitPos saved\n");
    }
    init_ok = elmo_init && my_init;
}

/**************************************************
---------------- Safety / Stop Helpers ------------
**************************************************/
void stopAllMotors() 
{
    for (int i = 0; i < NUM_ELMO; ++i) 
    {
        torqueCmd[i] = 0.0;
        TargetTor[i] = 0;
    }
    for (int i = 0; i < NUM_MY; ++i) 
    {
        const int idx = i + NUM_ELMO;
        torqueCmd[idx] = 0.0;
        TargetTor_my[i] = 0;
    }
}

bool anyStatusAbnormal() 
{
    bool bad = false;
    for (int i = 0; i < NUM_ELMO; ++i) 
    {
        if (StatusWord[i] != 4663) 
        {
            rt_printf("Abnormal StatusWord (ELMO %d): %d\n", i + 1, StatusWord[i]);
            bad = true;
        }
    }
    for (int i = 0; i < NUM_MY; ++i) 
    {
        if (StatusWord_my[i] != 4663) 
        {
            rt_printf("Abnormal StatusWord (MY %d): %d\n", i + 1, StatusWord_my[i]);
            bad = true;
        }
    }
    return bad;
}

bool anyPositionOutOfLimits() 
{
    for (int i = 0; i < NUM_ACT; ++i) 
    {
        if (!in_range(posRad[i], cfg::kPosLimit[i][0], cfg::kPosLimit[i][1])) 
        {
            rt_printf(
                "Position limit exceeded (motor %d): q=%.3f, lim=[%.3f, %.3f]\n",
                i, posRad[i], cfg::kPosLimit[i][0], cfg::kPosLimit[i][1]
            );
            return true;
        }
    }
    return false;
}

/**************************************************
---------------- Controllers ----------------------
**************************************************/
void pidControl(const int idx, const double q_des) 
{
    const double q  = posRad[idx];
    const double dq = velRad[idx];

    const double q_err  = q_des - q;
    const double dq_err = -dq;  // desired vel = 0

    P_term[idx]  = Kp[idx] * q_err;
    I_term[idx] += Ki[idx] * q_err * cfg::kDt;
    D_term[idx]  = Kd[idx] * dq_err;

    double u = P_term[idx] + I_term[idx] + D_term[idx];
    raw_pid[idx] = u;

    // zero if essentially at target (optional)
    if (std::abs(q_err) < 1e-4) u = 0.0;

    // apply torque limit
    u = clamp(u, -cfg::kMaxTorque[idx], cfg::kMaxTorque[idx]);

    // transmission compensation (motor-side torque)
    u = applyTransmissionOut(u, idx);

    torqueCmd[idx] = u;
    error_old[idx] = q_err;
}

/**************************************************
----------------- Tasks ---------------------------
**************************************************/
void joint_state_publisher(void* arg) 
{
    auto* state = static_cast<MotorState*>(arg);
    state->is_controller_ready = false;

    rt_task_set_periodic(nullptr, TM_NOW, ms(1.0));
    RTIME last = rt_timer_read();

    while (g_run) 
    {
        rt_task_wait_period(nullptr);
        const RTIME now = rt_timer_read();

        for (int i = 0; i < NUM_ACT; ++i) 
        {
            state->position[i]         = posRad[i];
            state->angular_velocity[i] = velRad[i];
            state->torque[i]           = torqueCmd[i];
        }
        state->is_controller_ready = is_controller_ready;
        state->elapsed_time_ns = static_cast<double>(now - last);
        last = now;
    }
}

void joint_command_sub(void* arg) 
{
    auto* cmd = static_cast<MotorCommand*>(arg);
    cmd->is_activated = false;

    rt_task_set_periodic(nullptr, TM_NOW, ms(1.0));
    while (g_run) 
    {
        rt_task_wait_period(nullptr);
        if (!cmd->is_activated) continue;
        is_planner_activated = true;

        for (int i = 0; i < NUM_ACT; ++i) 
        {
            pid_target[i] = cmd->position[i];
            if (cmd->kp[i] > 0) Kp[i] = cmd->kp[i];
            if (cmd->kd[i] > 0) Kd[i] = cmd->kd[i];
        }
    }
}

void control(void* /*arg*/) 
{
    rt_task_set_periodic(nullptr, TM_NOW, ms(1.0));
    ROS_INFO("Control task started");

    loadDefaultGains();
    uint16_t warmup_counter = 0;
    const uint16_t warmup_max = static_cast<uint16_t>(5.0 / cfg::kDt);

    while (g_run) 
    {
        rt_task_wait_period(nullptr);

        /* Wait until initial positions captured */
        if(!init_ok) continue; 

        /* Warmup period to let things settle */
        if(warmup_counter < warmup_max) 
        { 
            ++warmup_counter;
            continue;
        }

        /* Check abnormal status from motor driver */
        if(anyStatusAbnormal()) 
        {
            rt_printf("Motor error → stopping all motors.\n");
            stopAllMotors();
            break;
        }

        /* Check position limits */
        if(anyPositionOutOfLimits()) 
        {
            rt_printf("Limit error → stopping all motors.\n");
            stopAllMotors();
            break;
        }

        /* Broadcast the controller status to the user */
        is_controller_ready = true;

        /* If planner not activated, hold initial positions */
        if (!is_planner_activated) 
        {
            for (int i = 0; i < NUM_ACT; ++i) pid_target[i] = init_pos[i];
            continue;
        }

        /* If motor off signal received, stop all motors */
        if (motor_off) 
        {
            stopAllMotors();
            continue;
        }

        /* Normal operation: run PID control */
        for (int i = 0; i < NUM_ACT; ++i) 
        {
            pidControl(i, pid_target[i]);
        }
    }
}

void print_task(void* /*arg*/) {
  rt_task_set_periodic(nullptr, TM_NOW, ms(cfg::kPrintDt * 1000.0));
  ROS_INFO("Print task started");

  while (g_run) {
    rt_task_wait_period(nullptr);
    if (!cfg::kPrintOn) continue;

    rt_printf("\n================= STATE =================\n");
    rt_printf("%6s | %8s | %4s | %10s | %10s | %10s | %10s | %10s | %10s\n",
              "ID","Status","Mode","Act.Pos","Act.Vel","PosRad","VelRad/s",
              "Act.TorNm","Cmd.Tor");

    for (int i = 0; i < NUM_ELMO; ++i) {
      rt_printf("%6d | %8d | %4d | %10d | %10d | %10.4f | %10.3f | %10.3f | %10.3f\n",
        i+1, StatusWord[i], ModeOfOperationDisplayElmo[i],
        ActualPos[i], ActualVel[i], posRad[i], velRad[i],
        actualTorqueNm[i], torqueCmd[i]);
    }
    for (int i = 0; i < NUM_MY; ++i) {
      const int idx = i + NUM_ELMO;
      rt_printf("%6d | %8d | %4d | %10d | %10d | %10.4f | %10.3f | %10.3f | %10.3f\n",
        idx+1, StatusWord_my[i], ModeOfOperationDisplay_my[i],
        ActualPos_my[i], ActualVel_my[i], posRad[idx], velRad[idx],
        actualTorqueNm[idx], torqueCmd[idx]);
    }
    rt_print_flush_buffers();
  }
}


void EtherCAT(void* /*arg*/) 
{
#if DCMODE == 1
    ethercatMaster.InitDCMODE(1.0); // 1 ms
#else
    rt_task_set_periodic(nullptr, TM_NOW, ms(1.0));
#endif

    ROS_INFO("EtherCAT task started");

    while (g_run) 
    {
#if DCMODE == 1
        ethercatMaster.WaitTime();
#else
        rt_task_wait_period(nullptr);
#endif

        wkc = ethercatMaster.processTxDomain();

        // READ
        ethercatMaster.readBuffer(POSITION_ACTUAL_VALUE,      ActualPos);
        ethercatMaster.readBuffer(VELOCITY_ACTUAL_VALUE,      ActualVel);
        ethercatMaster.readBuffer(TORQUE_ACTUAL_VALUE,        ActualTor);
        ethercatMaster.readBuffer(STATUS_WORD,                StatusWord);
        ethercatMaster.readBuffer(MODE_OF_OPERATION_DISPLAY,  ModeOfOperationDisplayElmo);

        ethercatMaster.readBuffer(POSITION_ACTUALVALUE_my,    ActualPos_my);
        ethercatMaster.readBuffer(VELOCITY_ACTUALVALUE_my,    ActualVel_my);
        ethercatMaster.readBuffer(TORQUE_ACTUALVALUE_my,      ActualTor_my);
        ethercatMaster.readBuffer(STATUSWORD_my,              StatusWord_my);
        ethercatMaster.readBuffer(MODEofOPERATION_DISPLAY_my, ModeOfOperationDisplay_my);

        // Convert & init
        convertUnits();
        captureInitialPositionsIfReady();

        // WRITE
        ethercatMaster.writeBuffer(TARGET_POSITION, TargetPos);
        ethercatMaster.writeBuffer(TARGET_VELOCITY, TargetVel);
        ethercatMaster.writeBuffer(TARGET_TORQUE,   TargetTor);
        ethercatMaster.writeBuffer(CONTROL_WORD,    ControlWord);

        ethercatMaster.writeBuffer(TARGETTORQUE_my, TargetTor_my);
        ethercatMaster.writeBuffer(CONTROLWORD_my,  ControlWord_my);

#if DCMODE == 1
        if (ec_slave[0].hasdc) {
            ethercatMaster.ec_sync(ec_DCtime, (int64)ethercatMaster.cycletime, &ethercatMaster.toff);
        }
#endif
        ethercatMaster.processRxDomain();
    }
}

/**************************************************
---------------- Signal / Shutdown ----------------
**************************************************/
void gracefulShutdown() {
  motor_off = true;

  // Multiple zero-torque frames
  for (int rep = 0; rep < 20; ++rep) {
    for (int i = 0; i < NUM_MY; ++i) {
      const int idx = i + NUM_ELMO;
      torqueCmd[idx] = 0.0;
      TargetTor_my[i] = 0;
      ControlWord_my[i] = 0x00;
    }
    ethercatMaster.writeBuffer(TARGETTORQUE_my, TargetTor_my);
    ethercatMaster.writeBuffer(CONTROLWORD_my,  ControlWord_my);
    ethercatMaster.processRxDomain();
    usleep(2000);
  }

  // One final confirmation frame
  ethercatMaster.writeBuffer(TARGETTORQUE_my, TargetTor_my);
  ethercatMaster.writeBuffer(CONTROLWORD_my,  ControlWord_my);
  ethercatMaster.processRxDomain();
  usleep(3000);

  if (NUM_MY > 0) rt_printf("[Shutdown] MY drives commanded to zero torque.\n");
}

void signal_handler(int sig) {
  (void)sig;

  rt_task_delete(&task_control);
  gracefulShutdown();
  rt_task_delete(&task_ecat);
  rt_task_delete(&task_print);
  rt_task_delete(&task_joint_state_pub);
  rt_task_delete(&task_joint_cmd_sub);
  g_run = 0;

  ROS_INFO("Program terminated");
  std::exit(EXIT_SUCCESS);
}

/**************************************************
------------------------------ main --------------
**************************************************/
int main(int argc, char** argv) {
  ros::init(argc, argv, "Combine_test_clean");
  rt_printf("SOEM (Simple Open EtherCAT Master) — Clean Version\n");

  std::signal(SIGTERM, signal_handler);
  std::signal(SIGINT,  signal_handler);

  mlockall(MCL_CURRENT | MCL_FUTURE);

  // Shared memory (state)
  int shm_state_fd = shm_open(SHM_MOTOR_STATE_NAME, O_CREAT | O_RDWR, 0666);
  ftruncate(shm_state_fd, SHM_MOTOR_STATE_SIZE);
  auto* state = static_cast<MotorState*>(
      mmap(nullptr, SHM_MOTOR_STATE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_state_fd, 0));
  mlock(state, SHM_MOTOR_STATE_SIZE);

  // Shared memory (command)
  int shm_cmd_fd = shm_open(SHM_MOTOR_CMD_NAME, O_CREAT | O_RDWR, 0666);
  ftruncate(shm_cmd_fd, SHM_MOTOR_CMD_SIZE);
  auto* motor_cmd = static_cast<MotorCommand*>(
      mmap(nullptr, SHM_MOTOR_CMD_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_cmd_fd, 0));
  mlock(motor_cmd, SHM_MOTOR_CMD_SIZE);

  inOP = ethercatMaster.init(TORQUE_MODE, cfg::kIfName);
  if (!inOP) {
    rt_printf("System Initialization Failed\n");
    return EXIT_FAILURE;
  }

  // Tasks
  rt_task_create(&task_ecat, "EtherCAT", 0, 67, 0);
  rt_task_start (&task_ecat, &EtherCAT, nullptr);

  rt_task_create(&task_control, "Control", 0, 65, 0);
  rt_task_start (&task_control, &control, nullptr);

  rt_task_create(&task_print, "Print", 0, 60, 0);
  rt_task_start (&task_print, &print_task, nullptr);

  // ROS2–Xenomai bridge via SHM
  rt_task_create(&task_joint_state_pub, "JointStatePub", 0, 61, 0);
  rt_task_start (&task_joint_state_pub, &joint_state_publisher, state);

  rt_task_create(&task_joint_cmd_sub, "JointCmdSub", 0, 62, 0);
  rt_task_start (&task_joint_cmd_sub, &joint_command_sub, motor_cmd);

  while (g_run) sched_yield();

  // Cleanup
  rt_task_delete(&task_control);
  rt_task_delete(&task_print);
  rt_task_delete(&task_ecat);
  rt_task_delete(&task_joint_state_pub);
  rt_task_delete(&task_joint_cmd_sub);
  g_run = 0;

  rt_printf("End of Program\n");
  return EXIT_SUCCESS;
}
