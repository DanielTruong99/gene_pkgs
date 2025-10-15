#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include <queue>
#include <cstdint>
namespace event_manager
{

    enum Signal
    {
        INIT_SIG = 0,
        ENTRY_SIG,
        EXIT_SIG,
        DEFAULT_SIG,
        TIMEOUT_5MS_SIG,
        TIMEOUT_1S_SIG,
        TIMEOUT_3S_SIG,
        ABNORMAL_STATE_DETECTED,
        BACK_BUTTON_PRESSED,
        START_BUTTON_3S
    };

    class Event
    {
        public:
            Event(uint8_t signal) : signal(signal) {}
            uint8_t signal;
    };

    class EventManager 
    {
        public:
            EventManager();

            /* post event */
            void post_event(const Event * const event)
            {
                /* check if event queue is full */
                if (_event_queue.size() >= _max_event_queue_size) _event_queue.pop();

                /* push event to event queue */
                _event_queue.push(Event(event->signal));
            }

            /* query event */
            inline Event query_event()
            {
                static const Event default_event(static_cast<uint8_t>(Signal::DEFAULT_SIG));

                /* check if event queue is empty */
                if (_event_queue.empty()) return default_event;

                /* pop event from event queue */
                Event event = _event_queue.front();
                _event_queue.pop();
                return event;
            }

            /* check if event queue is empty */
            inline bool is_empty()
            {
                return _event_queue.empty();
            }

        private:
            /* event queue */
            uint8_t _max_event_queue_size = 200;
            std::queue<Event> _event_queue;
    };

    extern EventManager g_event_manager;
}
#endif // EVENT_MANAGER_H