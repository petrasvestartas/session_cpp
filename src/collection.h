#pragma once
#include "fmt/core.h"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace session_cpp {

class Tomb;

/// The guid a Collection indexes a shared entry by.
template <class T> const std::string& guid_of(const std::shared_ptr<T>& item) {
    return item->guid();
}

/// The guid a Collection indexes a value entry by, a Component.
template <class E> const std::string& guid_of(const E& item) {
    return item.guid();
}

/// A list of objects that can hold dead slots: every public view skips them, in slot order.
template <class E> class Collection {
private:
    std::vector<E> _items;                                   // Raw slots in canonical order, dead ones included.
    std::vector<bool> _dead;                                 // One flag per slot.
    std::unordered_map<std::string, size_t> _slots;          // Live guid -> slot.
    std::unordered_map<size_t, std::vector<std::weak_ptr<Tomb>>> _tombs; // Weak pins per slot, newest last.
    size_t _live = 0;                                        // Live count.
    size_t _count = 0;                                       // Dead slots not yet purged.
    size_t _low = 0;                                         // Lowest dead slot, where compaction starts.
    std::optional<std::pair<size_t, size_t>> _cursor;        // (read, write) while a compaction is part way.
    mutable std::optional<std::vector<size_t>> _positions;   // Live slot positions, built lazily.

    /// The raw slot of live position i, O(n) once after an edit while dead slots exist, then O(1).
    size_t _position(size_t i) const {

        if (i >= _live)
            throw std::out_of_range(fmt::format("Collection index {} out of range for {} live entries", i, _live));

        if (_count == 0 && !_cursor)
            return i;

        if (!_positions) {

            std::vector<size_t> live;
            live.reserve(_live);

            for (size_t slot = 0; slot < _dead.size(); ++slot)
                if (!_dead[slot])
                    live.push_back(slot);

            _positions = std::move(live);
        }

        return (*_positions)[i];
    }

    /// Point a tomb at the slot it pins; a template so Tomb may still be incomplete here.
    template <class T> static void _repin(const std::shared_ptr<T>& tomb, size_t slot) {
        tomb->slot = slot;
    }

    /// The tombs of a slot's pins that a record still holds, oldest first.
    std::vector<std::shared_ptr<Tomb>> _held(size_t slot) const {

        std::vector<std::shared_ptr<Tomb>> held;
        auto it = _tombs.find(slot);

        if (it == _tombs.end())
            return held;

        for (const std::weak_ptr<Tomb>& pin : it->second)
            if (std::shared_ptr<Tomb> tomb = pin.lock())
                held.push_back(std::move(tomb));

        return held;
    }

public:
    using value_type = E;

    /// Forward iterator over the live entries, in slot order.
    template <bool CONST> class Iterator {
    private:
        using Owner = std::conditional_t<CONST, const Collection*, Collection*>;
        Owner _owner = nullptr; // The collection walked.
        size_t _slot = 0;       // The current raw slot.

        /// Move to the first live slot at or after the current one.
        void _skip() {

            while (_slot < _owner->_dead.size() && _owner->_dead[_slot])
                ++_slot;
        }

    public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = E;
        using difference_type = std::ptrdiff_t;
        using pointer = std::conditional_t<CONST, const E*, E*>;
        using reference = std::conditional_t<CONST, const E&, E&>;

        /// Construct an iterator that points nowhere.
        Iterator() = default;

        /// Construct an iterator at the first live slot from slot on.
        Iterator(Owner owner, size_t slot) : _owner(owner), _slot(slot) {
            _skip();
        }

        /// Return the live entry.
        reference operator*() const {
            return _owner->_items[_slot];
        }

        /// Return the address of the live entry.
        pointer operator->() const {
            return &_owner->_items[_slot];
        }

        /// Advance to the next live entry.
        Iterator& operator++() {

            ++_slot;
            _skip();

            return *this;
        }

        /// Advance to the next live entry, returning the previous position.
        Iterator operator++(int) {

            Iterator previous = *this;
            ++*this;

            return previous;
        }

        /// Compare positions.
        bool operator==(const Iterator& other) const {
            return _slot == other._slot;
        }

        /// Compare positions.
        bool operator!=(const Iterator& other) const {
            return _slot != other._slot;
        }
    };

    using iterator = Iterator<false>;
    using const_iterator = Iterator<true>;

    // ═══════════════════════════════════════════════════════════════════════════
    // Constructors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Construct an empty collection.
    Collection() = default;

    /// Take every entry of a vector as live.
    explicit Collection(std::vector<E> items) : _items(std::move(items)), _dead(_items.size(), false), _live(_items.size()) {

        _slots.reserve(_items.size());

        for (size_t slot = 0; slot < _items.size(); ++slot)
            _slots[guid_of(_items[slot])] = slot;
    }

    /// Copy the live entries, compacted and without pins.
    Collection(const Collection& other) {

        if (other._count > 0 || other._cursor) {

            _items.reserve(other._live);

            for (const E& item : other)
                push_back(item);

            return;
        }

        _items = other._items;
        _dead = other._dead;
        _slots = other._slots;
        _live = other._live;
    }

    /// Copy-assign the live entries, compacted and without pins.
    Collection& operator=(const Collection& other) {

        if (this != &other) {
            Collection copy(other);
            *this = std::move(copy);
        }

        return *this;
    }

    /// Move every slot and pin as they are, leaving other empty.
    Collection(Collection&& other) noexcept {
        *this = std::move(other);
    }

    /// Move-assign every slot and pin as they are, leaving other empty so its counts still match its slots.
    Collection& operator=(Collection&& other) noexcept {

        if (this != &other) {
            _items = std::move(other._items);
            _dead = std::move(other._dead);
            _slots = std::move(other._slots);
            _tombs = std::move(other._tombs);
            _live = other._live;
            _count = other._count;
            _low = other._low;
            _cursor = other._cursor;
            _positions = std::move(other._positions);
            other.clear();
        }

        return *this;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Accessors
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the number of live entries.
    size_t size() const {
        return _live;
    }

    /// Return whether no entry is live.
    bool empty() const {
        return _live == 0;
    }

    /// Return an iterator at the first live entry.
    iterator begin() {
        return iterator(this, 0);
    }

    /// Return the iterator past the last slot.
    iterator end() {
        return iterator(this, _items.size());
    }

    /// Return an iterator at the first live entry.
    const_iterator begin() const {
        return const_iterator(this, 0);
    }

    /// Return the iterator past the last slot.
    const_iterator end() const {
        return const_iterator(this, _items.size());
    }

    /// Return the live entry at position i; throws std::out_of_range past the end.
    E& at(size_t i) {
        return _items[_position(i)];
    }

    /// Return the live entry at position i; throws std::out_of_range past the end.
    const E& at(size_t i) const {
        return _items[_position(i)];
    }

    /// Return the live entry at position i; throws std::out_of_range past the end.
    E& operator[](size_t i) {
        return at(i);
    }

    /// Return the live entry at position i; throws std::out_of_range past the end.
    const E& operator[](size_t i) const {
        return at(i);
    }

    /// Return the first live entry.
    E& front() {
        return at(0);
    }

    /// Return the first live entry.
    const E& front() const {
        return at(0);
    }

    /// Return the last live entry.
    E& back() {
        return at(_live - 1);
    }

    /// Return the last live entry.
    const E& back() const {
        return at(_live - 1);
    }

    /// Return the live entries as a vector.
    std::vector<E> to_vector() const {
        return std::vector<E>(begin(), end());
    }

    /// Return the live entries as a vector.
    operator std::vector<E>() const {
        return to_vector();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Kernel slots: raw access for Session, History and tests
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return the slot of a live guid.
    std::optional<size_t> get_slot(const std::string& guid) const {

        auto it = _slots.find(guid);

        if (it == _slots.end())
            return std::nullopt;

        return it->second;
    }

    /// Return the entry in a slot, dead or alive.
    const E& get_item(size_t slot) const {
        return _items[slot];
    }

    /// Put an entry in a slot; a live slot re-indexes its guid.
    void set_item(size_t slot, E item) {

        if (!_dead[slot]) {

            auto it = _slots.find(guid_of(_items[slot]));

            if (it != _slots.end() && it->second == slot)
                _slots.erase(it);

            _slots[guid_of(item)] = slot;
        }

        _items[slot] = std::move(item);
    }

    /// Return whether a slot is dead.
    bool is_dead(size_t slot) const {
        return _dead[slot];
    }

    /// Kill or revive a slot in O(1); a kill unindexes the guid only when it points at this slot.
    void set_dead(size_t slot, bool dead) {

        if (_dead[slot] == dead)
            return;

        if (_cursor && _cursor->second <= slot && slot < _cursor->first)
            return;

        const std::string& key = guid_of(_items[slot]);
        _dead[slot] = dead;
        _positions.reset();

        if (!dead) {
            _slots[key] = slot;
            ++_live;
            --_count;

            return;
        }

        auto it = _slots.find(key);

        if (it != _slots.end() && it->second == slot)
            _slots.erase(it);

        if (_count == 0 && !_cursor)
            _low = slot;

        --_live;
        ++_count;
        _low = std::min(_low, slot);
    }

    /// Return the newest tomb pinning a slot while a record still holds it.
    std::shared_ptr<Tomb> get_tomb(size_t slot) const {

        const std::vector<std::shared_ptr<Tomb>> held = _held(slot);

        return held.empty() ? nullptr : held.back();
    }

    /// Pin a slot weakly to a tomb and point the tomb at the slot; older pins a record still holds stay.
    void set_tomb(size_t slot, const std::shared_ptr<Tomb>& tomb) {

        std::vector<std::weak_ptr<Tomb>> pins;

        for (const std::shared_ptr<Tomb>& held : _held(slot))
            if (held != tomb)
                pins.push_back(held);

        pins.push_back(tomb);
        this->_repin(tomb, slot);
        _tombs[slot] = std::move(pins);
    }

    /// Return the number of dead slots not yet purged.
    size_t number_of_dead() const {
        return _count;
    }

    /// Return the number of raw slots, dead ones included.
    size_t number_of_slots() const {
        return _items.size();
    }

    /// Return whether a compaction is part way.
    bool is_compacting() const {
        return _cursor.has_value();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // Mutators
    // ═══════════════════════════════════════════════════════════════════════════
    /// Append a live entry and index its guid, O(1) amortised.
    void push_back(E item) {

        const size_t slot = _items.size();
        _slots[guid_of(item)] = slot;
        _items.push_back(std::move(item));
        _dead.push_back(false);
        ++_live;

        if (_positions)
            _positions->push_back(slot);
    }

    /// Drop every entry and pin.
    void clear() {

        _items.clear();
        _dead.clear();
        _slots.clear();
        _tombs.clear();
        _live = 0;
        _count = 0;
        _low = 0;
        _cursor.reset();
        _positions.reset();
    }

    /// Purge unpinned dead slots for at most work slots, resuming where the last call stopped; returns the slots examined.
    size_t compact_step(size_t work) {

        if (work == 0 || (!_cursor && _count == 0))
            return 0;

        if (!_cursor) {
            const size_t start = std::min(_low, _items.size());
            _cursor = std::make_pair(start, start);
            _low = SIZE_MAX;
        }

        size_t r = _cursor->first;
        size_t w = _cursor->second;
        size_t examined = 0;
        _positions.reset();

        while (examined < work && r < _items.size()) {

            const std::vector<std::shared_ptr<Tomb>> held = _held(r);

            if (_dead[r] && held.empty()) {
                _tombs.erase(r);
                --_count;
            } else {
                if (w != r) {
                    std::swap(_items[w], _items[r]);
                    const bool dead = _dead[r];
                    _dead[r] = _dead[w];
                    _dead[w] = dead;
                    _tombs.erase(r);

                    for (const std::shared_ptr<Tomb>& tomb : held)
                        this->_repin(tomb, w);

                    if (!held.empty())
                        _tombs[w] = std::vector<std::weak_ptr<Tomb>>(held.begin(), held.end());

                    if (!_dead[w])
                        _slots[guid_of(_items[w])] = w;
                }

                if (_dead[w])
                    _low = std::min(_low, w);

                ++w;
            }

            ++r;
            ++examined;
        }

        if (r < _items.size()) {
            _cursor = std::make_pair(r, w);

            return examined;
        }

        _items.erase(_items.begin() + w, _items.end());
        _dead.erase(_dead.begin() + w, _dead.end());
        _cursor.reset();
        _low = std::min(_low, w);

        return examined;
    }

    /// Finish a running compaction, then purge every unpinned dead slot.
    void compact() {

        if (_cursor)
            compact_step(SIZE_MAX);

        compact_step(SIZE_MAX);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // String
    // ═══════════════════════════════════════════════════════════════════════════
    /// Return "Collection(3 live, 0 dead)".
    std::string str() const {
        return fmt::format("Collection({} live, {} dead)", _live, _count);
    }

    /// Return "Collection(3 live, 0 dead)".
    std::string repr() const {
        return str();
    }
};

/// Write the collection string to a stream.
template <class E> std::ostream& operator<<(std::ostream& os, const Collection<E>& collection) {
    return os << collection.str();
}

} // namespace session_cpp
