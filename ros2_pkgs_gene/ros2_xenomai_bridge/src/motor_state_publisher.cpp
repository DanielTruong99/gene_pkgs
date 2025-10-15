#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#define SHM_NAME "/motor_state"
#define SHM_SIZE sizeof(MotorState)
#define NUM_MOTORS 10
struct MotorState
{
    char* joint_names[NUM_MOTORS];
    double position[NUM_MOTORS];
    double angular_velocity[NUM_MOTORS];
    double torque[NUM_MOTORS];
    double elapsed_time; // ms
    bool is_controller_ready = false;
};

using MotorState = struct MotorState;

class MotorPublisher : public rclcpp::Node
{
public:
    MotorPublisher() : Node("motor_publisher")
    {
        std::cout << "MotorPublisher node started" << std::endl;
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", 15);

        init_shared_memory();
        init_buffers();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            [this]()
            { this->publish_state(); });
    }

    ~MotorPublisher()
    {
        if (state_) munmap(state_, SHM_SIZE);
        if (shm_fd_ >= 0) close(shm_fd_);
    }

private:
    void init_buffers()
    {
        joint_state_.position.resize(NUM_MOTORS);
        joint_state_.velocity.resize(NUM_MOTORS);
        joint_state_.effort.resize(NUM_MOTORS + 1); // +1 for elapsed_time
        joint_state_.name = {
            "R_hip_joint", 
            "R_hip2_joint", 
            "R_thigh_joint", 
            "R_calf_joint", 

            "L_hip_joint", 
            "L_hip2_joint", 
            "L_thigh_joint", 
            "L_calf_joint", 

            "L_toe_joint",
            "R_toe_joint",
        
        };
    }

    void init_shared_memory()
    {
        shm_fd_ = shm_open(SHM_NAME, O_RDONLY, 0666);
        if (shm_fd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open shared memory");
            rclcpp::shutdown();
            return;
        }
        state_ = (MotorState *)mmap(NULL, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd_, 0);
    }

    void publish_state()
    {
        bool is_motor_driver_controller_ready = state_->is_controller_ready;
        if(!is_motor_driver_controller_ready) return;

        for (int i = 0; i < NUM_MOTORS; ++i)
        {
            joint_state_.position[i] = state_->position[i];
            joint_state_.velocity[i] = state_->angular_velocity[i];
            joint_state_.effort[i] = state_->torque[i];

            // joint_state_.name[i] = state_->joint_names[i];
        }
        joint_state_.effort[NUM_MOTORS] = state_->elapsed_time; // ms elapsed time
        joint_state_.header.stamp = this->now();
        publisher_->publish(joint_state_);
    }

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;
    int shm_fd_;
    MotorState *state_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorPublisher>());
    rclcpp::shutdown();
    return 0;
}