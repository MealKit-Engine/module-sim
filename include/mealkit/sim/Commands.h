#pragma once

#include "mealkit/core/Hash.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/core/Types.h"
#include "mealkit/net/Lockstep.h"

#include <cstring>
#include <type_traits>

// Confirmed commands (docs/CODING_MODEL.md "Non-input commands into the simulation"): separate from tick input (8 bytes per player),
// occasionally sent commands — debug cheats, mid-match setting changes, votes. Modifying state directly would desync other peers/the shadow simulation, so
// the view side (app/, view/) only sends commands, and the simulation processes them on the confirmed tick.
//
//   MEALKIT_STRUCT(Shockwave) { MEALKIT_FIELD(u32, power, 1, "Power"); };   // command type in sim/ (no strings/containers)
//   sim::sendCommand(ctx.session, Shockwave{});                            // view side: send
//   sim::forEachCommand(tickInputs, [&](const sim::Command& c) {           // at the start of World::step(): process
//       if (Shockwave s; c.read(s)) shockwave(world, c.player, s);
//   });
//
// The host decides the tick to apply it on and sends it to everyone with that tick's inputs. Order within a tick is player index → send order.
// They go into input recordings/rewind/shadow simulation together with inputs. Commands are sent via memcpy, so they only mean the same between identical builds (same as the lockstep premise).
// Types are distinguished by a hash of the name (4 bytes): one command is type + contents up to net::kMaxCommandBytes.
namespace mealkit::sim {

template <class T>
constexpr void checkCommandType() {
    static_assert(std::is_trivially_copyable_v<T>, "a command is sent with memcpy: no std::string, containers or pointers");
    static_assert(sizeof(T) + 4 <= net::kMaxCommandBytes, "command too large (net::kMaxCommandBytes)");
}

template <class T>
u32 commandType() {
    checkCommandType<T>();
    static const u32 id = static_cast<u32>(fnv1a(reflect::typeOf<T>().name));
    return id;
}

struct Command {
    u8 player = 0; // sending player slot
    u32 type = 0;
    const u8* data = nullptr;
    u32 size = 0;

    template <class T>
    bool is() const {
        return type == commandType<T>() && size == sizeof(T);
    }
    // If it's this type, copies into out and returns true
    template <class T>
    bool read(T& out) const {
        if (!is<T>()) return false;
        std::memcpy(&out, data, sizeof(T));
        return true;
    }
};

// This tick's confirmed commands in order: f(const Command&)
template <class F>
void forEachCommand(const net::TickInputs& inputs, F&& f) {
    net::forEachCommand(inputs, [&](u8 player, const u8* data, u32 size) {
        if (size < 4) return;
        Command c;
        c.player = player;
        c.type = static_cast<u32>(data[0]) | static_cast<u32>(data[1]) << 8 | static_cast<u32>(data[2]) << 16 |
                 static_cast<u32>(data[3]) << 24;
        c.data = data + 4;
        c.size = size - 4;
        f(static_cast<const Command&>(c));
    });
}

// Called by the view side (app/, view/). false if not mid-match (Running) or the queue is full
template <class T>
bool sendCommand(net::LockstepSession& session, const T& command) {
    const u32 type = commandType<T>();
    u8 buffer[4 + sizeof(T)];
    buffer[0] = static_cast<u8>(type);
    buffer[1] = static_cast<u8>(type >> 8);
    buffer[2] = static_cast<u8>(type >> 16);
    buffer[3] = static_cast<u8>(type >> 24);
    std::memcpy(buffer + 4, &command, sizeof(T));
    return session.submitCommand(buffer, sizeof(buffer));
}

// Engine command: a player joined mid-match (first time or again) or disconnected (net::kPresenceCommand, sent by the host).
// All peers receive it on the same tick. For a new slot the game creates the slot/controller; loadout is that person's match start data (same bytes as game::setLoadout)
struct Presence {
    u8 player = 0;
    bool connected = false;
    const u8* loadout = nullptr;
    u32 loadoutSize = 0;

    // Reads the loadout as T (same format as game::readLoadout: 4-byte type hash + contents)
    template <class T>
    bool readLoadout(T& out) const {
        if (loadoutSize != 4 + sizeof(T)) return false;
        u32 type = 0;
        std::memcpy(&type, loadout, 4);
        if (type != static_cast<u32>(fnv1a(reflect::typeOf<T>().name))) return false;
        std::memcpy(&out, loadout + 4, sizeof(T));
        return true;
    }
};

inline bool readPresence(const Command& c, Presence& out) {
    if (c.type != static_cast<u32>(fnv1a(net::kPresenceCommand)) || c.size < 1) return false;
    out.player = c.player;
    out.connected = c.data[0] != 0;
    out.loadout = c.data + 1;
    out.loadoutSize = c.size - 1;
    return true;
}

} // namespace mealkit::sim
