#pragma once

#include "mealkit/core/Log.h"
#include "mealkit/core/Types.h"

#include <type_traits>

// 시뮬레이션 → 화면 알림 (docs/CODING_MODEL.md "시뮬레이션 → 화면 알리기").
// sim/은 world.events.push(...)로 쓰기만 하고 읽지 않는다. World::step()이 틱 시작 때 clear()한다.
// GameSession이 step() 뒤마다 화면 쪽 큐로 옮기므로, 한 프레임에 틱이 여러 번 돌아도 빠지지 않는다.
// 시뮬레이션이 읽지 않으므로 상태 해시에 넣지 않는다. 넘치면 그 이벤트는 버린다(화면 효과만 빠진다).
namespace mealkit::sim {

template <class T, u32 Capacity>
class TickEvents {
    static_assert(std::is_trivially_copyable_v<T>);

public:
    void push(const T& event) {
        MEALKIT_ASSERT(m_count < Capacity); // 한 틱 이벤트가 넘쳤다: 용량을 늘린다
        if (m_count < Capacity) m_items[m_count++] = event;
    }
    void clear() { m_count = 0; }

    // 화면 쪽(app/, view/)에서만 읽는다
    u32 size() const { return m_count; }
    const T* begin() const { return m_items; }
    const T* end() const { return m_items + m_count; }

private:
    T m_items[Capacity] = {};
    u32 m_count = 0;
};

} // namespace mealkit::sim
