#pragma once

#include "mealkit/core/Reflect.h"

// "After N ticks" timer (docs/CODING_MODEL.md "Things that take time"). Stores one tick number when it becomes ready instead of real time or a per-tick countdown.
// State is a single u32, so hashing/snapshots are easy (it's a field declaration, so putting it in a state struct includes it in the hash/snapshots automatically).
namespace mealkit::sim {

MEALKIT_STRUCT(TickTimer) {
    MEALKIT_FIELD(u32, readyTick, 0, "Ready tick"); // ready from this tick (0 = ready from the start)

    void start(u32 now, u32 ticks) { readyTick = now + ticks; }
    bool ready(u32 now) const { return now >= readyTick; }
    u32 remaining(u32 now) const { return now >= readyTick ? 0u : readyTick - now; }
};

} // namespace mealkit::sim
