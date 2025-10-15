#include "EventManager.h"

namespace event_manager
{
    EventManager g_event_manager;

    EventManager::EventManager()
        : _event_queue(std::queue<Event>())
    {
    }
}