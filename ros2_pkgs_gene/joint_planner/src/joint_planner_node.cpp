#include "rclcpp/rclcpp.hpp"

#include "ActuatorInterface/ROSActuatorInterface.h"
#include "Planner/Planner.h"
#include "DataInterface/ROSDataInterface.h"
#include "StateMachine/StateMachine.h"


int main(int argc, char *argv[])
{
    /* Initialize ros2 node */
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rl_controller_node");

    /* Create the actuator interface */
    auto actuator_interface = std::make_shared<ROSActuatorInterface>(node);

    /* Create the data interface */
    auto data_interface = std::make_shared<ROSDataInterface>(node);


    /* Create the planner */
    auto planner = std::make_shared<planner::Planner>(data_interface);  


    /* Create the state machine */
    auto state_machine = std::make_shared<planner_sm::PlannerStateMachine>(node, actuator_interface, data_interface, planner);

    while(rclcpp::ok())
    {
        state_machine->event_loop(); // Process the state machine events
        rclcpp::spin_some(node); // Process any incoming messages
    }

    rclcpp::shutdown();
    return 0;
}