/**************************************************
---------------- Pre processing ------------------
**************************************************/
/* ---- Basic setting ---- */ 
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <ctime>
/* ---- Basic setting of Eigen start ---- */ 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
/* ---- Basic setting of Xenomai start ---- */
#include <fcntl.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>
////// XENOMAI3 //////
#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <trank/rtdk.h>
////// XENOMAI3 //////
/* ---- Basic setting of EtherCAT start ---- */ 
#include "Humanoid_EtherCAT_DC.h"
/* ---- Macro setting ---- */ 
// #define dt 0.001
// #define millisec(x) (x*1000000)

/**************************************************
---------------- Global Variable ------------------
**************************************************/
/* ---- Global variable of Xenomai start ---- */
RT_TASK joint_state_publisher_task; // 주기: 1ms
RT_TASK joint_command_subscriber_task; // 주기: 1ms 
RT_TASK control_task;      // 주기: 1ms
RT_TASK print_task;        // 주기: 1ms
RT_TASK save_task;         // 주기: 1ms
RT_TASK save_sin;
RT_TASK EtherCAT_task;     // 주기: 1ms
RT_TASK ecat_ch;           // 주기: 250µs
static int run = 1;

/* ---- Global variable of EtherCAT start ---- */ 
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
EthercatMaster ethercatMaster;

/* ---- Global variable of ACTUATORS ---- */ 
// READ PDO: SEA
int32_t ActualPos[NUM_ELMO] = {0,};
int16_t ActualTor[NUM_ELMO] = {0,};
uint16_t StatusWord[NUM_ELMO] = {0,};
int32_t ActualVel[NUM_ELMO] = {0,};
uint8_t ModeOfOperationDisplayElmo[NUM_ELMO] = {0,};

// WRITE PDO: SEA
int32_t TargetPos[NUM_ELMO] = {0,};
int32_t TargetVel[NUM_ELMO] = {0,};
int16_t TargetTor[NUM_ELMO] = {0,};
uint16_t ControlWord[NUM_ELMO] = {0,};

/* ---- PID control variables ---- */ 
double error[NUM_ELMO] = {0,};

double Kp[NUM_ELMO] = {0,};  
double Kd[NUM_ELMO] = {0,};
double Ki[NUM_ELMO] = {0,};

double P_control[NUM_ELMO] = {0,};
double I_control[NUM_ELMO] = {0,};
double D_control[NUM_ELMO] = {0,};

/* ---- Velocity PID control variables ---- */ 
double pid_vel_target[NUM_ELMO] = {0,};     //rad/s
RTIME vel_step_start_time = 0;
bool vel_step_started = false;
double rpm_setpoint = -100.0;  // 시작 rpm
const double rpm_end = 100.0;  // 끝 rpm
const double rpm_step = 1.0;   // 증가 단위
const double step_interval_sec = 4.0; // 5초마다 변경

/*---target position && init position--------*/
double pid_target[NUM_ELMO] = {0,};
double init_pos[NUM_ELMO] = {0,}; 
bool init_ck = false;

/* ---- Global variable of residure start ---- */
double sampleing_time = 0.001;
double max_torque = 8.0;
// double min_torque = 1.0;
std::ofstream data_file; 
std::ofstream sin_file;

/* ---- Global variable ---- */
#define PI 3.14159265358979323846
#define PI2 (2.0 * PI) // 2π
#define GEAR_RATIO 10.0 // gear ratio
#define ENCODER_RESOLUTION 131072.0 // Encoder resolution
#define ELMO_TORQUE_CONSTANT 1.65     // Nm/A
#define ELMO_RATED_CURRENT (20.0 * sqrt(2))  // rated current [rms]
#define printOn 1
#define saveOn 1
#define TEST_Pos 0    //Test는 하나가 1이면 하나는 0이어야 함. if one test is 1, the others have to be 0
#define TEST_sin 0
#define TEST_VEL 1
#define sin_id 1

// Convert Unit
double posRad[NUM_ELMO] = {0,};
double velRadPerSec[NUM_ELMO] = {0,};
double torqueCmd[NUM_ELMO] = {0,};
double actualTorqueNm[NUM_ELMO] = {0,}; 
double zero_pos[NUM_ELMO] = {0,};
double raw_pos[NUM_ELMO] = {0,};
double raw_vel[NUM_ELMO] = {0,};
double raw_pid[NUM_ELMO] = {0,};
bool motor_off = false;
bool sin_go = true;
int sin_ck[NUM_ELMO] = {0,};
double total_current_A[NUM_ELMO] = {0,}; // 누적 전류 (A)
int total_samples[NUM_ELMO] = {0,};      // 누적 샘플 개수

/**************************************************
------------- Function declaration ---------------
**************************************************/
void parameter(void *arg);
void pidControl(int actuator_index, double target_pos, double* error_old);
void velControl(int actuator_index, double target_vel, double* error_old);
void Convertunit(void *arg);
void initialPositionCapture(void *arg);
/**************************************************
--------------- Normal functions -----------------
**************************************************/
void parameter(void *arg)
{
    ///////////////////Right////////////////
    // Hip yaw
    Kp[0] = 0.0; Kd[0] = 0.0; Ki[0] = 0.0;
    // //  Hip roll
    Kp[1] = 0.0; Kd[1] = 0.0; Ki[1] = 0.0;   ///750
    // //  Hip pitch
    // Kp[2] = 900.0; Kd[2] = 8.0; Ki[2] = 0.0;
    // //  knee pitch 
    // Kp[3] = 450.0; Kd[3] = 8.0; Ki[3] = 0.0; //Best: 300 / 8.0
    // // ankle pitch
    // Kp[4] = 250.0; Kd[4] = 3.0; Ki[4] = 0.0; //Best: 250 / 3.0
    // // Hip yaw
    // Kp[0] = 0.0; Kd[0] = 0.0; Ki[0] = 0.0;
    // // Hip roll
    // Kp[1] = 0.0; Kd[1] = 0.0; Ki[1] = 0.0;
    // // Hip pitch
    // Kp[2] = 0.0; Kd[2] = 0.0; Ki[2] = 0.0;
    // // knee pitch 
    // Kp[3] = 0.0; Kd[3] = 0.0; Ki[3] = 0.0;
    // // ankle pitch
    // Kp[4] = 0.0; Kd[4] = 0.0; Ki[4] = 0.0;
    ///////////////////LEFT//////////////////
    // Hip Yaw
    // Kp[5] = 0.0; Kd[5] = 0.0; Ki[5] = 0.0;
    // //  Hip roll
    // Kp[6] = 0.0; Kd[6] = 0.0; Ki[6] = 0.0;
    // // //  Hip pitch
    // Kp[7] = 0.0; Kd[7] = 0.0; Ki[7] = 0.0;
    // //  knee pitch
    // Kp[8] = 0.0; Kd[8] = 0.0; Ki[8] = 0.0;
    // // ankle pitch
    // Kp[9] = 0.0; Kd[9] = 0.0; Ki[9] = 0.0; 
}

void pidControl(int actuator_index, double target_pos, double* error_old)
{
    double errorlimit = 0.2;
    double error_now = target_pos - posRad[actuator_index];

    parameter(NULL);  // Gain 초기화

    // Clamp error
    if (error_now > errorlimit) error_now = errorlimit;
    else if (error_now < -errorlimit) error_now = -errorlimit;

    // PID 연산
    P_control[actuator_index] = Kp[actuator_index] * error_now;
    I_control[actuator_index] += Ki[actuator_index] * error_now * sampleing_time;
    D_control[actuator_index] = Kd[actuator_index] * (error_now - *error_old) / sampleing_time;
    // D_control[actuator_index] = -Kd[actuator_index] * velRadPerSec[actuator_index];

    double PID_control = P_control[actuator_index] + I_control[actuator_index] + D_control[actuator_index];
    raw_pid[actuator_index] = PID_control;

    // Clamp output torque (MAX)
    if (PID_control > max_torque) PID_control = max_torque;
    else if (PID_control < -max_torque) PID_control = -max_torque;
    // Clamp output torque (MIN)
    // if (std::abs(PID_control) <= min_torque && std::abs(error_now) > 0.001)
    // {
    //     PID_control = (PID_control > 0) ? min_torque : -min_torque;
    // }
    // Clamp output torque (Zero)
    if (std::abs(error_now) < 0.0001) PID_control = 0;

    // rt_printf("Motor %d : pid = %f \n", actuator_index, PID_control);
    // Save result in torqueCmd only
    torqueCmd[actuator_index] = PID_control;

    // rt_printf("Motor %d : pid = %f \n", actuator_index, torqueCmd[actuator_index]);
    *error_old = error_now;
}

void velControl(int actuator_index, double target_vel, double* error_old)
{
    double error_now = target_vel - velRadPerSec[actuator_index];

    double Kp_vel = 10.0;  // 임의값, 실험하면서 조절
    double Kd_vel = 0.0;
    double Ki_vel = 0.0;

    double P = Kp_vel * error_now;
    double D = Kd_vel * (error_now - *error_old) / sampleing_time;
    double I = 0; // Ki_vel * error 누적은 이후 추가

    double torque = P + D + I;

    // 제한
    if (torque > max_torque) torque = max_torque;
    else if (torque < -max_torque) torque = -max_torque;

    torqueCmd[actuator_index] = torque;
    *error_old = error_now;
}

void Convertunit(void *arg)
{
    for (int i = 0; i < NUM_ELMO; i++) {
        // Position [rad]
        raw_pos[i] = static_cast<double>(ActualPos[i]) / ENCODER_RESOLUTION / GEAR_RATIO * PI2;
        posRad[i] = raw_pos[i];
        

        // Velocity [rad/s]
        double rpm = static_cast<double>(ActualVel[i]) / ENCODER_RESOLUTION;
        raw_vel[i] = rpm / GEAR_RATIO * PI2;
        velRadPerSec[i] = raw_vel[i];

        // TorqueCmd [Nm] → Current [%] → TargetTor
        double current_A = torqueCmd[i] / ELMO_TORQUE_CONSTANT;

        // for calculate total current //////////////////////
        total_current_A[i] += std::abs(current_A);
        total_samples[i]++; 
        /////////////////////////////////////////////////////

        double percent = current_A / ELMO_RATED_CURRENT * 1000.0;
        TargetTor[i] = static_cast<int16_t>(percent);

        // Actual Torque
        actualTorqueNm[i] = ((double)ActualTor[i] / 1000.0) * ELMO_RATED_CURRENT * ELMO_TORQUE_CONSTANT;
    }
}

void initialPositionCapture(void *arg)
{
    static bool elmo_initialized = false;

    // ELMO 초기화: StatusWord == 4663
    if (!elmo_initialized) {
        bool all_elmo_ready = true;
        for (int i = 0; i < NUM_ELMO; i++) {
            if (StatusWord[i] != 4663) {
                all_elmo_ready = false;
                break;
            }
        }
        if (all_elmo_ready) {
            for (int i = 0; i < NUM_ELMO; i++) {
                init_pos[i] = posRad[i];
            }
            elmo_initialized = true;
            rt_printf("[Init] ELMO StatusWord = 4663 → InitPos 저장 완료\n");
        }
    }

    // 모두 완료되면 flag ON
    if (elmo_initialized) {
        init_ck = true;
    }
}
/**************************************************
--------------- Xenomai functions ----------------
**************************************************/
struct MotorState
{
    double position[NUM_ELMO + NUM_EVO];         // Radians
    double angular_velocity[NUM_ELMO + NUM_EVO]; // Rad/s
    double torque[NUM_ELMO + NUM_EVO];           // Nm
    double elapsed_time;    // ns
};

using MotorState = struct MotorState;

#define SHM_MOTOR_STATE_NAME "/motor_state"
#define SHM_MOTOR_STATE_SIZE sizeof(MotorState)

void joint_state_publisher(void *arg)
{
    MotorState *state = (MotorState *)arg;
    RTIME now, start_time;

    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0));       // 1ms
    start_time = rt_timer_read(); // 시작 시간 기록
    while (true) 
    {
        rt_task_wait_period(NULL);
        
        /* Loop over the ELMO drivers  */
        for (int index = 0; index < NUM_ELMO + NUM_EVO; index++)
        {
            // fake podRad to be sine function
            // state->position[index] = std::sin(2 * PI * 0.5 * (now) / 1e9); // 0.5Hz sine wave
            state->position[index] = posRad[index];
            state->angular_velocity[index] = velRadPerSec[index];
            state->torque[index] = ActualTor[index];
        }

        /* Update elapsed time */
        now = rt_timer_read();
        state->elapsed_time = (double)(now - start_time); //ns
        start_time = now;
    }
}

struct MotorCommand
{
    double position[NUM_ELMO + NUM_EVO];         // Radians
    double angular_velocity[NUM_ELMO + NUM_EVO]; // Rad/s
    double torque[NUM_ELMO + NUM_EVO];           // Nm
};

using MotorCommand = struct MotorCommand;

#define SHM_MOTOR_CMD_NAME "/motor_command"
#define SHM_MOTOR_CMD_SIZE sizeof(MotorCommand)

void joint_command_sub(void *arg)
{
    MotorCommand *motor_cmd = (MotorCommand *)arg;
    RTIME now, start_time;

    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0)); // 1ms
    start_time = rt_timer_read();                     // 시작 시간 기록
    while (true)
    {
        rt_task_wait_period(NULL);

        /* Copy joint cmds */
        for (int index = 0; index < NUM_ELMO + NUM_EVO; index++)
        {
            pid_target[index] += motor_cmd->position[index];
            
        }

        // rt_printf("Motor %d: Position Command = %f\n", 0, motor_cmd->position[0]);
    }
}

void control(void *arg)
{
    RTIME now, previous;
    RTIME start_time = rt_timer_read(); 
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0));       

    rt_task_sleep((RTIME)ms(30.0));
    previous = rt_timer_read();

    static RTIME sin_start_time = 0;
    static bool sin_started = false;

    while (run) {
        rt_task_wait_period(NULL);
        now = rt_timer_read();

        if (!run)
            break;

        double A = 0.5;   // 진폭
        double f = 0.5;   // 주파수

        static double prev_sin_val[NUM_ELMO] = {0,}; // 이전 sin 값 저장

        for (int i = 0; i < NUM_ELMO; i++) 
        {
            if (init_ck == true && motor_off == false) {
                pidControl(i, pid_target[i], &error[i]);
            }
            else
            {
                for (int i = 0; i < NUM_ELMO; i++) {
                    torqueCmd[i] = 0.0;
                    TargetTor[i] = 0;
                }
            }
        }
        previous = now;
    }
}

void print(void *arg)
{
    RTIME now, previous;
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0));       // 1ms


    previous = rt_timer_read();

    while (run) {
        rt_task_wait_period(NULL);
        now = rt_timer_read();

        rt_printf("\n====================================================================================================\n");
        rt_printf("\e[33;1m%6s | %8s | %4s | %10s | %10s | %10s | %10s | %10s | %10s | %10.3s | %10s | %10s | %10s | %10s | %10s | %10s \e[0m\n",
                    "ID", "Status", "Mode", "Act.Pos", "Act.Vel", "Tgt.Pos", "PosRad", "VelRad/s", "Act.Tor", "Act.Tor(Nm)", "Tgt.Tor", "Cmd.Tor", "RawForce", "InitPos", "RPM SET", "Tat.Vel");

        for (int i = 0; i < NUM_ELMO; i++) {
            rt_printf("\e[32;1m%6d | %8d | %4d | %10d | %10d | %10.4f | %10.4f | %10.3f | %10d | %10.3f | %10d | %10.3f | %10.3f | %10.4f | %10f | %10f \e[0m\n",
                    i + 1, StatusWord[i], ModeOfOperationDisplayElmo[i],
                    ActualPos[i], ActualVel[i], pid_target[i], posRad[i], velRadPerSec[i],
                    ActualTor[i], actualTorqueNm[i], TargetTor[i], torqueCmd[i], raw_pid[i], init_pos[i], rpm_setpoint, pid_vel_target[i]);
        }
        rt_printf("====================================================================================================\n");

        previous = now;
    }
}

void saveData(void *arg)
{
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0));       // 1ms

    time_t now = time(0);
    struct tm sbc_tm;
    localtime_r(&now, &sbc_tm);

    char filename[256];
    sprintf(filename, "/home/gene/catkin_ws/DATA/Elmo_data/data_%d_%02d_%02d_%02dh_%02dm_%02ds.csv",
            sbc_tm.tm_year + 1900, sbc_tm.tm_mon + 1, sbc_tm.tm_mday,
            sbc_tm.tm_hour, sbc_tm.tm_min, sbc_tm.tm_sec);

    data_file.open(filename);

    if (!data_file.is_open()) {
        rt_printf("Failed to open file for saving data\n");
        return;
    }

    rt_printf("Data saving task started, file: %s\n", filename);

    data_file << "Time,MotorID,Status,Mode,ActualPos,ActualVel,TargetPos,PosRad,VelRadPerSec,ActualTor,ActualTorNm,TargetTor,TorqueCmd,RawForce,InitPos,RawPos,RawVel,RpmSetpoint\n";

    RTIME start_time = rt_timer_read();
    // WTF professor fucking LAB Fucking PCB
    while (run) {
        rt_task_wait_period(NULL);

        RTIME current_time = rt_timer_read();
        double elapsed_time = (double)(current_time - start_time) / 1e9;

        // ELMO
        for (int i = 0; i < NUM_ELMO; i++) {
            data_file << elapsed_time << ","
                    << i + 1 << ","
                    << StatusWord[i] << ","
                    << static_cast<int>(ModeOfOperationDisplayElmo[i]) << ","
                    << ActualPos[i] << ","
                    << ActualVel[i] << ","
                    << pid_target[i] << ","
                    << posRad[i] << ","
                    << velRadPerSec[i] << ","
                    << ActualTor[i] << ","
                    << actualTorqueNm[i] << ","
                    << TargetTor[i] << ","
                    << torqueCmd[i] << ","
                    << raw_pid[i] << ","
                    << init_pos[i] << ","
                    << raw_pos[i] << ","
                    << raw_vel[i] << ","
                    << rpm_setpoint << "\n";
        }
        data_file.flush();
    }

    data_file.close();
    rt_printf("Data saving task ended\n");
}

void saveSin(void *arg)
{
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(1000.0));       // 1ms

    time_t now = time(0);
    struct tm sbc_tm;
    localtime_r(&now, &sbc_tm);

    char filename[256];
    sprintf(filename, "/home/gene/catkin_ws/DATA/sin/sin_%d_%02d_%02d_%02dh_%02dm_%02ds.csv",
            sbc_tm.tm_year + 1900, sbc_tm.tm_mon + 1, sbc_tm.tm_mday,
            sbc_tm.tm_hour, sbc_tm.tm_min, sbc_tm.tm_sec);

    sin_file.open(filename);

    if (!sin_file.is_open()) {
        rt_printf("Failed to open file for saving data\n");
        return;
    }

    rt_printf("Data saving task started, file: %s\n", filename);

    sin_file << "Time,MotorID,Sin,MeanCurrentA\n";

    RTIME start_time = rt_timer_read();
    // WTF professor fucking LAB Fucking PCB
    while (run) {
        rt_task_wait_period(NULL);

        RTIME current_time = rt_timer_read();
        double elapsed_time = (double)(current_time - start_time) / 1e9;

        for (int i = 0; i < NUM_ELMO; i++) {
            double avg_current = (total_samples[i] > 0) ? total_current_A[i] / total_samples[i] : 0.0;
            sin_file << elapsed_time << ","
                      << i + 1 << ","
                      << sin_ck[i] << ","
                      << avg_current << "\n";
        }

        sin_file.flush();
    }

    sin_file.close();
    rt_printf("Data saving task ended\n");
}

void EtherCAT(void *arg)
{
    RTIME now, previous;
    #if DCMODE == 1
        // DCMODE일 경우 주기 설정
        ethercatMaster.InitDCMODE(1.0); // 1ms
    #else
        rt_task_set_periodic(NULL, TM_NOW, (int)ms(1.0));       // 1ms
    #endif


    previous = rt_timer_read();

    while (run) {
        // rt_task_wait_period(NULL);
        ethercatMaster.WaitTime();
        now = rt_timer_read();

        wkc = ethercatMaster.processTxDomain(); 

        // ELMO 데이터 읽기
        ethercatMaster.readBuffer(POSITION_ACTUAL_VALUE, ActualPos);
        ethercatMaster.readBuffer(VELOCITY_ACTUAL_VALUE, ActualVel);
        ethercatMaster.readBuffer(TORQUE_ACTUAL_VALUE, ActualTor);
        ethercatMaster.readBuffer(STATUS_WORD, StatusWord);
        ethercatMaster.readBuffer(MODE_OF_OPERATION_DISPLAY, ModeOfOperationDisplayElmo);

        ///////////////////////////////////
        Convertunit(NULL);
        initialPositionCapture(NULL);
        //////////////////////////////////

        // ELMO 데이터 쓰기
        ethercatMaster.writeBuffer(TARGET_POSITION, TargetPos);
        ethercatMaster.writeBuffer(TARGET_VELOCITY, TargetVel);
        ethercatMaster.writeBuffer(TARGET_TORQUE, TargetTor);
        ethercatMaster.writeBuffer(CONTROL_WORD, ControlWord);
        
        #if DCMODE == 1
            if ( ec_slave[0].hasdc ) 
            {
                ////// Calulate toff to get linux time and DC synced
                ////// ethercatMaster.toff : predicted delay time of EtherCAT communication
                ethercatMaster.ec_sync(ec_DCtime, (int64)ethercatMaster.cycletime, &ethercatMaster.toff);
            }
        #endif

        ethercatMaster.processRxDomain();

        previous = now;
    }
}

void ecatcheck( void *ptr ) // For check the EtherCAT status
{
    int slave;

    ethercatMaster.SetLoopTimeCheck(1000);

    while(1)
    {
        #if DCMODE == 1
            ethercatMaster.ecatcheckTime.tv_nsec += ethercatMaster.cycletimeCheck; // frequency : 1kHz (1 ms)
            if( ethercatMaster.ecatcheckTime.tv_nsec > SEC_IN_NSEC )
            {
                ethercatMaster.ecatcheckTime.tv_nsec -= SEC_IN_NSEC;
                ethercatMaster.ecatcheckTime.tv_sec++;
            }
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ethercatMaster.ecatcheckTime, NULL);
        #endif

        ethercatMaster.previousCheck = rt_timer_read();
        
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               rt_printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        
        ethercatMaster.nowCheck = rt_timer_read();
        ethercatMaster.timet_ecat = (double)((ethercatMaster.nowCheck - ethercatMaster.previousCheck)/1e9);

        #if DCMODE == 0
            rt_task_wait_period(NULL);
        #endif
        
        // usleep(250);
    }
}

void signal_handler(int sig)
{
    
    rt_task_delete(&control_task);
    rt_task_delete(&EtherCAT_task);
    rt_task_delete(&ecat_ch);
    rt_task_delete(&save_sin);
    #if saveOn == 1
        rt_task_delete(&save_task);
    #endif
    #if printOn == 1
        rt_task_delete(&print_task);
    #endif

    run = 0;

    exit(true);
}

int main(int argc, char **argv)
{
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Shared Memory configuration
        - shm_open: Create a new shared memory object
        - ftruncate: Set the size of the shared memory object
        - mmap: Map the shared memory object into the process's address space
        - mlock: Lock the memory to prevent page faults
    */
    // Create shared memory for motor state
    // int shm_motor_state_fd = shm_open(SHM_MOTOR_STATE_NAME, O_CREAT | O_RDWR, 0666);
    // ftruncate(shm_motor_state_fd, SHM_MOTOR_STATE_SIZE);

    // MotorState *state = (MotorState *)mmap(NULL, SHM_MOTOR_STATE_SIZE,
    //                                        PROT_READ | PROT_WRITE,
    //                                        MAP_SHARED, shm_motor_state_fd, 0);
    // mlock(state, SHM_MOTOR_STATE_SIZE);

    // // Create shared memory for motor command
    // int shm_motor_cmd_fd = shm_open(SHM_MOTOR_CMD_NAME, O_CREAT | O_RDWR, 0666);
    // ftruncate(shm_motor_cmd_fd, SHM_MOTOR_CMD_SIZE);

    // MotorCommand *motor_cmd = (MotorCommand *)mmap(NULL, SHM_MOTOR_CMD_SIZE,
    //                                        PROT_READ | PROT_WRITE,
    //                                        MAP_SHARED, shm_motor_cmd_fd, 0);
    // mlock(motor_cmd, SHM_MOTOR_CMD_SIZE);

    /* Ethercat initialization */
    inOP = ethercatMaster.init(TORQUE_MODE, "rteth0");

    if (!inOP) {
        printf("System Initialization Failed\n");
        return 0;
    }


    // rt_task_create(&ecat_ch, "EtherCAT_checking", 0, 50, 0);   
    // rt_task_start(&ecat_ch, &ecatcheck, NULL);

    // rt_task_create(&EtherCAT_task, "EtherCAT_tasking", 0, 80, 0);
    // rt_task_start(&EtherCAT_task, &EtherCAT, NULL);

    rt_task_create(&control_task, "controling", 0, 70, 0);
    rt_task_start(&control_task, &control, NULL);

    /* Create ros2 xenomai bridge via shared memory mechanism 
        - joint_state_publisher: Publish the joint state
        - joint_command_sub: Subscribe to the joint command
    */
//    rt_task_create(&joint_state_publisher_task, "joint_state_publisher", 0, 40, 0);
//    rt_task_start(&joint_state_publisher_task, &joint_state_publisher, state);

//    rt_task_create(&joint_command_subscriber_task, "joint_command_subscriber", 0, 71, 0);
//    rt_task_start(&joint_command_subscriber_task, &joint_command_sub, motor_cmd);


    // while (run) {
    //     sched_yield();
    // }

    // rt_task_delete(&control_task);
    // rt_task_delete(&ecat_ch);
    // rt_task_delete(&EtherCAT_task);
    // run = 0;

    rt_printf("End of Program\n");

    return 0;
}