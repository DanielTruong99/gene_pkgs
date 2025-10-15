#ifndef FSM_H
#define FSM_H

#include <queue>
#include <cstdint>
#include <functional>

#include "../EventManager/EventManager.h"

namespace fsm 
{

    enum Status
    {
        TRAN_STATUS = 1,
        HANDLED_STATUS = 2,
        IGNORED_STATUS = 3,
        INIT_STATUS = 4,
    };

    class FSM
    {
        public:
            FSM();

            /* event loop */
            void event_loop();

            /* state handlers */
            virtual Status initial_state(const event_manager::Event *const event) = 0;

        protected:
            using StateHandler = Status (FSM::*)(const event_manager::Event *const event);

            /* current state handler */
            StateHandler _state;

        private:
            /* dispatch event */
            void _dispatch(const event_manager::Event * const event);
    };


}// namespace fsm

#endif // FSM_H