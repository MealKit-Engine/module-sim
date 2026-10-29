#pragma once

#include "mealkit/core/Hash.h"
#include "mealkit/core/Log.h"
#include "mealkit/core/Reflect.h"

#include <type_traits>
#include <vector>

// Simulation object storage (docs/CODING_MODEL.md "Creation and destruction").
//
// - Capacity is set once with init() when a match starts. No allocation after that (no allocation mid-tick).
// - Creation is immediate: the id returned by create() can be get() in the same tick. But it doesn't appear in iteration (for) on the tick it was created.
// - Deletion only marks: after destroy() it's excluded from get() and iteration, and endTick() actually clears it in slot order.
//   So creating or deleting during iteration is safe, and slots aren't reused within the same tick.
// - ids carry a generation number, so get() with a deleted object's id returns nullptr (even if the slot is reused).
//
// Determinism: iteration is in slot order, and empty slots are reused in a fixed order. Slot state (generations, free list) goes into
// World::hash() via hashState(), and the game adds item fields one by one itself (so struct padding doesn't mix in).
namespace mealkit::sim {

// Pool<T> item id. generation 0 is invalid (slot generations start at 1).
template <class T>
struct Id {
    static constexpr bool kReflectId = true;
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
    using value_type = T;

    // Once at match start. All previous items disappear.
    void init(u32 capacity) {
        m_items.assign(capacity, T{});
        m_generations.assign(capacity, 1u);
        m_slots.assign(capacity, Slot::Free);
        m_free.clear();
        m_free.reserve(capacity);
        for (u32 i = capacity; i > 0; --i) m_free.push_back(i - 1); // slot 0 is used first
        m_dying.clear();
        m_dying.reserve(capacity);
        m_count = 0;
    }

    // Grows capacity mid-match (more players from late joins). New slots are used after the current empty slots are used up. All peers call it on the same tick
    void grow(u32 capacity) {
        const u32 old = this->capacity();
        if (capacity <= old) return;
        m_items.resize(capacity, T{});
        m_generations.resize(capacity, 1u);
        m_slots.resize(capacity, Slot::Free);
        std::vector<u32> free;
        free.reserve(capacity);
        for (u32 i = capacity; i > old; --i) free.push_back(i - 1);
        free.insert(free.end(), m_free.begin(), m_free.end());
        m_free = std::move(free);
        m_dying.reserve(capacity);
    }

    u32 capacity() const { return static_cast<u32>(m_items.size()); }
    u32 count() const { return m_count; } // slots in use (including ones created this tick and ones to delete)

    // Returns an invalid id when full (fails the same way on all peers). It means capacity should grow, so it also asserts.
    Id<T> create(const T& value = T{}) {
        MEALKIT_ASSERT(!m_free.empty()); // the Pool is full: increase capacity in World::reset()
        if (m_free.empty()) return {};
        const u32 slot = m_free.back();
        m_free.pop_back();
        m_items[slot] = value;
        m_slots[slot] = Slot::New;
        ++m_count;
        return {slot, m_generations[slot]};
    }

    // Only marks. Already deleted or invalid ids are ignored.
    void destroy(Id<T> id) {
        if (!live(id)) return;
        m_slots[id.slot] = Slot::Dying;
        m_dying.push_back(id.slot);
    }

    bool alive(Id<T> id) const { return live(id); }
    T* get(Id<T> id) { return live(id) ? &m_items[id.slot] : nullptr; }
    const T* get(Id<T> id) const { return live(id) ? &m_items[id.slot] : nullptr; }

    // Once at the end of a tick (end of World::step). Passes items to delete in slot order to onDestroy(T&) and clears them,
    // and puts items created this tick into iteration from the next tick. onDestroy only does cleanup like deleting physics bodies.
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

    // Only slot state is added. Item fields are added by the game while iterating.
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

    // ---- Iteration: for (auto [id, item] : pool) — slot order, only items alive and not created this tick
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

    // Description used by field declarations (MEALKIT_FIELD): hash, snapshots, editor state view
    static const reflect::ContainerOps& reflectOps() {
        static const reflect::ContainerOps ops{
            [](const void* p) { return static_cast<const Pool*>(p)->capacity(); },
            [](const void* p, u32 slot) {
                const Slot s = static_cast<const Pool*>(p)->m_slots[slot];
                return s == Slot::New || s == Slot::Alive;
            },
            [](const void* p, u32 slot) -> const void* { return &static_cast<const Pool*>(p)->m_items[slot]; },
            [](const void* p, u32 slot) { return static_cast<const Pool*>(p)->m_generations[slot]; },
            [](const void* p, Hasher& h) { static_cast<const Pool*>(p)->hashState(h); },
            [](const void* p, reflect::ByteWriter& w) { static_cast<const Pool*>(p)->save(w); },
            [](void* p, reflect::ByteReader& r) { return static_cast<Pool*>(p)->load(r); },
        };
        return ops;
    }

private:
    template <class V>
    static void saveVector(const std::vector<V>& v, reflect::ByteWriter& w) {
        w.value(static_cast<u32>(v.size()));
        if (!v.empty()) w.bytes(v.data(), v.size() * sizeof(V));
    }
    template <class V>
    static bool loadVector(std::vector<V>& v, reflect::ByteReader& r) {
        u32 n = 0;
        if (!r.value(n)) return false;
        v.resize(n);
        return n == 0 || r.bytes(v.data(), n * sizeof(V));
    }
    void save(reflect::ByteWriter& w) const {
        w.value(m_count);
        saveVector(m_items, w);
        saveVector(m_generations, w);
        saveVector(m_slots, w);
        saveVector(m_free, w);
        saveVector(m_dying, w);
    }
    bool load(reflect::ByteReader& r) {
        // Capacity is reserved up front so there's no mid-tick allocation even after loading
        if (!r.value(m_count) || !loadVector(m_items, r) || !loadVector(m_generations, r) || !loadVector(m_slots, r) ||
            !loadVector(m_free, r) || !loadVector(m_dying, r))
            return false;
        m_free.reserve(m_items.size());
        m_dying.reserve(m_items.size());
        return true;
    }

    bool live(Id<T> id) const {
        return id.valid() && id.slot < capacity() && m_generations[id.slot] == id.generation &&
               (m_slots[id.slot] == Slot::New || m_slots[id.slot] == Slot::Alive);
    }

    // Cleans up in slot order regardless of deletion order (insertion sort: few deletions per tick)
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
    std::vector<u32> m_free;  // the next slot to use is at the back
    std::vector<u32> m_dying; // slots deleted this tick
    u32 m_count = 0;
};

} // namespace mealkit::sim
