#pragma once

#include "mealkit/core/Hash.h"
#include "mealkit/core/Log.h"

#include <type_traits>
#include <vector>

// 시뮬레이션 객체 저장소 (docs/CODING_MODEL.md "생성과 파괴").
//
// - 용량은 매치를 시작할 때 init()으로 한 번 정한다. 그 뒤로는 할당하지 않는다(틱 도중 할당 금지).
// - 생성은 즉시: create()가 돌려준 id로 이번 틱에도 get()할 수 있다. 다만 만든 틱에는 순회(for)에 나오지 않는다.
// - 삭제는 표시만: destroy() 뒤로는 get()과 순회에서 빠지고, endTick()이 슬롯 순서로 실제로 비운다.
//   그래서 순회 도중에 만들거나 지워도 안전하고, 같은 틱 안에서 슬롯이 다시 쓰이지 않는다.
// - id에는 세대 번호가 붙어 있어, 지워진 객체의 id로 get()하면 nullptr이다(슬롯이 다시 쓰여도).
//
// 결정론: 순회는 슬롯 번호 순서, 빈 슬롯은 정해진 순서로 다시 쓴다. 슬롯 상태(세대, 빈 슬롯 목록)는 hashState()로
// World::hash()에 넣고, 항목의 필드는 게임이 직접 하나씩 넣는다(구조체 패딩이 섞이지 않게).
namespace mealkit::sim {

// Pool<T> 항목 id. generation 0은 무효(슬롯 세대는 1부터 시작한다).
template <class T>
struct Id {
    u32 slot = 0;
    u32 generation = 0;

    bool valid() const { return generation != 0; }
    friend bool operator==(Id a, Id b) { return a.slot == b.slot && a.generation == b.generation; }
};

template <class T>
class Pool {
    static_assert(std::is_trivially_copyable_v<T>, "Pool items must be plain data (no pointers to owned memory)");

    enum class Slot : u8 { Free, New, Alive, Dying };

public:
    // 매치 시작 때 한 번. 이전 항목은 모두 사라진다.
    void init(u32 capacity) {
        m_items.assign(capacity, T{});
        m_generations.assign(capacity, 1u);
        m_slots.assign(capacity, Slot::Free);
        m_free.clear();
        m_free.reserve(capacity);
        for (u32 i = capacity; i > 0; --i) m_free.push_back(i - 1); // 0번 슬롯부터 쓴다
        m_dying.clear();
        m_dying.reserve(capacity);
        m_count = 0;
    }

    u32 capacity() const { return static_cast<u32>(m_items.size()); }
    u32 count() const { return m_count; } // 쓰는 중인 슬롯 (이번 틱에 만든 것과 지울 것 포함)

    // 꽉 찼으면 무효 id를 돌려준다 (모든 피어에서 똑같이 실패한다). 용량을 늘려야 한다는 뜻이라 assert도 건다.
    Id<T> create(const T& value = T{}) {
        MEALKIT_ASSERT(!m_free.empty()); // Pool이 꽉 찼다: World::reset()에서 용량을 늘린다
        if (m_free.empty()) return {};
        const u32 slot = m_free.back();
        m_free.pop_back();
        m_items[slot] = value;
        m_slots[slot] = Slot::New;
        ++m_count;
        return {slot, m_generations[slot]};
    }

    // 표시만 한다. 이미 지웠거나 무효인 id는 무시한다.
    void destroy(Id<T> id) {
        if (!live(id)) return;
        m_slots[id.slot] = Slot::Dying;
        m_dying.push_back(id.slot);
    }

    bool alive(Id<T> id) const { return live(id); }
    T* get(Id<T> id) { return live(id) ? &m_items[id.slot] : nullptr; }
    const T* get(Id<T> id) const { return live(id) ? &m_items[id.slot] : nullptr; }

    // 틱 끝에 한 번 (World::step 마지막). 지울 항목을 슬롯 순서로 onDestroy(T&)에 넘긴 뒤 비우고,
    // 이번 틱에 만든 항목을 다음 틱부터 순회에 넣는다. onDestroy에서는 물리 바디 삭제 같은 정리만 한다.
    template <class OnDestroy>
    void endTick(OnDestroy&& onDestroy) {
        if (!m_dying.empty()) {
            sortSlots(m_dying);
            for (u32 slot : m_dying) {
                onDestroy(m_items[slot]);
                m_items[slot] = T{};
                m_slots[slot] = Slot::Free;
                if (++m_generations[slot] == 0) m_generations[slot] = 1;
                m_free.push_back(slot);
                --m_count;
            }
            m_dying.clear();
        }
        for (Slot& s : m_slots)
            if (s == Slot::New) s = Slot::Alive;
    }
    void endTick() {
        endTick([](T&) {});
    }

    // 슬롯 상태만 넣는다. 항목 필드는 게임이 순회하며 넣는다.
    void hashState(Hasher& h) const {
        h.add(capacity());
        h.add(m_count);
        for (u32 i = 0; i < capacity(); ++i) {
            h.add(m_generations[i]);
            h.add(static_cast<u8>(m_slots[i]));
        }
        h.add(static_cast<u32>(m_free.size()));
        for (u32 slot : m_free) h.add(slot);
    }

    // ---- 순회: for (auto [id, item] : pool) — 슬롯 순서, 살아 있고 이번 틱에 만들지 않은 항목만
    template <class Ref>
    struct Entry {
        Id<T> id;
        Ref value;
    };

    template <class PoolT, class Ref>
    class Iterator {
    public:
        Iterator(PoolT* pool, u32 slot) : m_pool(pool), m_slot(slot) { skip(); }
        Entry<Ref> operator*() const { return {{m_slot, m_pool->m_generations[m_slot]}, m_pool->m_items[m_slot]}; }
        Iterator& operator++() {
            ++m_slot;
            skip();
            return *this;
        }
        bool operator!=(const Iterator& other) const { return m_slot != other.m_slot; }

    private:
        void skip() {
            while (m_slot < m_pool->capacity() && m_pool->m_slots[m_slot] != Slot::Alive) ++m_slot;
        }
        PoolT* m_pool;
        u32 m_slot;
    };

    Iterator<Pool, T&> begin() { return {this, 0}; }
    Iterator<Pool, T&> end() { return {this, capacity()}; }
    Iterator<const Pool, const T&> begin() const { return {this, 0}; }
    Iterator<const Pool, const T&> end() const { return {this, capacity()}; }

private:
    bool live(Id<T> id) const {
        return id.valid() && id.slot < capacity() && m_generations[id.slot] == id.generation &&
               (m_slots[id.slot] == Slot::New || m_slots[id.slot] == Slot::Alive);
    }

    // 지운 순서와 관계없이 슬롯 순서로 정리한다 (삽입 정렬: 한 틱에 지우는 수는 적다)
    static void sortSlots(std::vector<u32>& slots) {
        for (size_t i = 1; i < slots.size(); ++i) {
            const u32 v = slots[i];
            size_t j = i;
            for (; j > 0 && slots[j - 1] > v; --j) slots[j] = slots[j - 1];
            slots[j] = v;
        }
    }

    std::vector<T> m_items;
    std::vector<u32> m_generations;
    std::vector<Slot> m_slots;
    std::vector<u32> m_free;  // 다음에 쓸 슬롯이 맨 뒤
    std::vector<u32> m_dying; // 이번 틱에 지운 슬롯
    u32 m_count = 0;
};

} // namespace mealkit::sim
