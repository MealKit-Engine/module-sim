#pragma once

#include "mealkit/core/Types.h"

// "N틱 뒤" 타이머 (docs/CODING_MODEL.md "시간이 걸리는 일"). 실수 시간이나 매 틱 감소 대신 준비되는 틱 번호 하나를 저장한다.
// 상태가 u32 하나라 해시·스냅샷이 쉽다: World::hash()에 readyTick을 넣는다.
namespace mealkit::sim {

struct TickTimer {
    u32 readyTick = 0; // 이 틱부터 준비됨 (0 = 처음부터 준비)

    void start(u32 now, u32 ticks) { readyTick = now + ticks; }
    bool ready(u32 now) const { return now >= readyTick; }
    u32 remaining(u32 now) const { return now >= readyTick ? 0u : readyTick - now; }
};

} // namespace mealkit::sim
