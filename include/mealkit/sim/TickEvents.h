#pragma once

#include "mealkit/core/Log.h"
#include "mealkit/core/Types.h"

#include <type_traits>

// Simulation → view notifications (docs/CODING_MODEL.md "Notifying the view from the simulation").
// sim/ only writes with world.events.push(...) and never reads. World::step() clear()s it at tick start.
// GameSession moves them to the view-side queue after every step(), so nothing is lost even when several ticks run per frame.
// The simulation doesn't read them, so they're not in the state hash. On overflow the event is dropped (only a view effect is lost).
namespace mealkit::sim {

template <class T, u32 Capacity>
class TickEvents {
    static_assert(std::is_trivially_copyable_v<T>);

public:
    void push(const T& event) {
        MEALKIT_ASSERT(m_count < Capacity); // one tick's events overflowed: increase capacity
        if (m_count < Capacity) m_items[m_count++] = event;
    }
    void clear() { m_count = 0; }

    // Read only by the view side (app/, view/)
    u32 size() const { return m_count; }
    const T* begin() const { return m_items; }
    const T* end() const { return m_items + m_count; }

private:
    T m_items[Capacity] = {};
    u32 m_count = 0;
};

} // namespace mealkit::sim
