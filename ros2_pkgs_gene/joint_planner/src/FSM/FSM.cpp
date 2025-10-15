#include "FSM.h"

namespace fsm
{

    FSM::FSM()
    {
        _state = &FSM::initial_state;
    }

    void FSM::event_loop()
    {
        /* check if event queue is empty */
        if (event_manager::g_event_manager.is_empty()) return;
        
        /* there are events in event queue, process it */
        event_manager::Event event = event_manager::g_event_manager.query_event();
        _dispatch(&event);
    }

    void FSM::_dispatch(const event_manager::Event *const event)
    {
        /* cache the previous state */
        StateHandler previous_state = _state;

        /* dispatch event to current state 
            Inside the state handler, the _trainsition_to() method can be called to change the _state.
        */
        Status status = (this->*_state)(event);

        /* check if event is tran */
        static event_manager::Event const entry_event(static_cast<uint8_t>(event_manager::Signal::ENTRY_SIG));
        static event_manager::Event const exit_event(static_cast<uint8_t>(event_manager::Signal::EXIT_SIG));
        if( status == Status::TRAN_STATUS )
        {
            (this->*previous_state)((event_manager::Event *)&exit_event);
            (this->*_state)((event_manager::Event *)&entry_event);
        }
    }
}