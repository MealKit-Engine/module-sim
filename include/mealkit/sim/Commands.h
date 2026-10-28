#pragma once

#include "mealkit/core/Hash.h"
#include "mealkit/core/Reflect.h"
#include "mealkit/core/Types.h"
#include "mealkit/net/Lockstep.h"

#include <cstring>
#include <type_traits>

// 확정 명령 (docs/CODING_MODEL.md "시뮬레이션에 넣는 입력 외 명령"): 틱 입력(플레이어당 8바이트)과 별도로
// 가끔 보내는 명령 — 디버그 치트, 매치 중 설정 변경, 투표. 상태를 직접 고치면 다른 피어·그림자 시뮬레이션이 디싱크되므로
// 화면 쪽(app/, view/)은 명령을 보내기만 하고, 시뮬레이션이 확정된 틱에 처리한다.
//
//   MEALKIT_STRUCT(Shockwave) { MEALKIT_FIELD(u32, power, 1, "세기"); };   // sim/의 명령 타입 (문자열·컨테이너 없음)
//   sim::sendCommand(ctx.session, Shockwave{});                            // 화면 쪽: 보낸다
//   sim::forEachCommand(tickInputs, [&](const sim::Command& c) {           // World::step() 맨 앞: 처리한다
//       if (Shockwave s; c.read(s)) shockwave(world, c.player, s);
//   });
//
// 호스트가 적용할 틱을 정하고 그 틱의 입력과 함께 모두에게 보낸다. 한 틱 안의 순서는 플레이어 번호 → 보낸 순서.
// 입력 기록·되감기·그림자 시뮬레이션에 입력과 같이 들어간다. 명령은 memcpy로 보내므로 같은 빌드끼리만 뜻이 같다(락스텝 전제와 같다).
// 타입은 이름의 해시(4바이트)로 구분한다: 명령 하나는 타입 + 내용이 net::kMaxCommandBytes 이하.
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
    u8 player = 0; // 보낸 플레이어 슬롯
    u32 type = 0;
    const u8* data = nullptr;
    u32 size = 0;

    template <class T>
    bool is() const {
        return type == commandType<T>() && size == sizeof(T);
    }
    // 이 타입이면 out에 복사하고 true
    template <class T>
    bool read(T& out) const {
        if (!is<T>()) return false;
        std::memcpy(&out, data, sizeof(T));
        return true;
    }
};

// 이 틱의 확정 명령을 차례로: f(const Command&)
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

// 화면 쪽(app/, view/)이 부른다. 매치 중(Running)이 아니거나 대기열이 차면 false
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

// 엔진 명령: 플레이어가 매치 중에 들어오거나(처음 또는 다시) 끊겼다 (net::kPresenceCommand, 호스트가 보낸다).
// 모든 피어가 같은 틱에 받는다. 새 슬롯이면 게임이 슬롯·컨트롤러를 만들고, loadout은 그 사람의 매치 시작 데이터(game::setLoadout과 같은 바이트)
struct Presence {
    u8 player = 0;
    bool connected = false;
    const u8* loadout = nullptr;
    u32 loadoutSize = 0;

    // 로드아웃을 T로 읽는다 (game::readLoadout과 같은 형식: 타입 해시 4바이트 + 내용)
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
