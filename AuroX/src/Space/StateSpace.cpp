// ============================================================================
// AuroX - StateSpace implementation (non-template only)
// ----------------------------------------------------------------------------
// Mirrors include/AuroX/Space/StateSpace.hpp one-to-one: the header holds
// declarations, the .ipp holds template bodies, and every non-template function
// lives here. Responsibilities: unique value creation, validated update,
// numeric validation, vector packing and layout computation.
//
// Dependencies: standard library only (no json, no other space).
// ============================================================================

#include "AuroX/Space/StateSpace.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace AuroX {
namespace Space {

namespace {

// Number of doubles this value contributes to vec(); 0 when it is excluded.
// A string can never be vectorized, and an empty Vec would silently occupy no
// space while still claiming to be numeric, so both are rejected.
std::size_t components(const StateValue& value, bool vectorized) {
    if (!vectorized) return 0;
    if (std::holds_alternative<Str>(value))
        throw std::invalid_argument("StateSpace: a string state cannot be vectorized");
    if (const auto* v = std::get_if<Vec>(&value)) {
        if (v->empty())
            throw std::invalid_argument("StateSpace: a vectorized Vec must not be empty");
        return v->size();
    }
    return 1;
}

// Reject values that cannot take part in a numeric view (NaN / +-Inf).
// bool / int / int64 / Str are always representable, so they pass untouched.
void validate(const StateValue& value) {
    if (const auto* f = std::get_if<float>(&value)) {
        if (!std::isfinite(*f))
            throw std::invalid_argument("StateSpace: non-finite float state");
    } else if (const auto* d = std::get_if<double>(&value)) {
        if (!std::isfinite(*d))
            throw std::invalid_argument("StateSpace: non-finite double state");
    } else if (const auto* v = std::get_if<Vec>(&value)) {
        for (double x : *v)
            if (!std::isfinite(x))
                throw std::invalid_argument("StateSpace: non-finite Vec state");
    }
}

// 2^53: the largest magnitude up to which every int64 is exactly representable
// as a double. An int64 state that is vectorized must stay inside this range, so
// round-tripping vec() -> unvec-style pipelines cannot lose integer precision.
constexpr std::int64_t kMaxExactInt64 = 9007199254740992LL;

void validateInt64Range(const StateValue& value) {
    if (const auto* n = std::get_if<std::int64_t>(&value)) {
        if (*n > kMaxExactInt64 || *n < -kMaxExactInt64)
            throw std::invalid_argument(
                "StateSpace: int64 state is not exactly representable in double");
    }
}

const char* typeName(const StateValue& value) {
    if (std::holds_alternative<bool>(value))          return "bool";
    if (std::holds_alternative<int>(value))           return "int";
    if (std::holds_alternative<std::int64_t>(value))  return "int64";
    if (std::holds_alternative<float>(value))         return "float";
    if (std::holds_alternative<double>(value))        return "double";
    if (std::holds_alternative<Str>(value))           return "string";
    return "vector<double>";
}

// Append this value's double components to the numeric view. Values were
// validated (finite, int64 range) on the way in, so this never throws for a
// well-formed space.
void pack(const StateValue& value, Vec& out) {
    if (const auto* v = std::get_if<Vec>(&value)) {
        out.insert(out.end(), v->begin(), v->end());
    } else if (const auto* b = std::get_if<bool>(&value)) {
        out.push_back(*b ? 1.0 : 0.0);            // only when explicitly enabled
    } else if (const auto* i = std::get_if<int>(&value)) {
        out.push_back(static_cast<double>(*i));
    } else if (const auto* n = std::get_if<std::int64_t>(&value)) {
        out.push_back(static_cast<double>(*n));   // range already checked
    } else if (const auto* f = std::get_if<float>(&value)) {
        out.push_back(static_cast<double>(*f));
    } else if (const auto* d = std::get_if<double>(&value)) {
        out.push_back(*d);
    } else {
        throw std::logic_error("StateSpace: a string state reached the numeric layout");
    }
}

} // namespace

// ============================================================================
// Non-template core
// ============================================================================

std::size_t StateSpace::addRaw(Str name, StateValue value, Str desc, bool vectorized) {
    if (frozen_)       throw std::logic_error("StateSpace: schema is frozen");
    if (name.empty())  throw std::invalid_argument("StateSpace: state name must not be empty");
    if (contains(name))
        throw std::invalid_argument("StateSpace: duplicate state name '" + name + "'");

    validate(value);
    const std::size_t n = components(value, vectorized);
    if (vectorized) validateInt64Range(value);
    if (n > std::numeric_limits<std::size_t>::max() - vectorDim_)
        throw std::length_error("StateSpace: vec() dimension overflow");

    const std::size_t i = items_.size();
    items_.push_back(std::make_unique<State>(
        State{ std::move(name), std::move(desc), std::move(value), vectorized }));

    // Map insert may allocate; roll the item back if it fails so the space is
    // left exactly as it was before the call.
    try {
        index_.emplace(items_[i]->name, i);
    } catch (...) {
        items_.pop_back();
        throw;
    }

    vectorDim_ += n;
    return i;
}

void StateSpace::setRaw(std::size_t index, StateValue value) {
    auto& slot = items_.at(index)->value;

    // A state's declared type and a Vec's length are part of the schema: a
    // downstream fixed-index layout would misread the data if they changed.
    if (slot.index() != value.index())
        throw std::invalid_argument("StateSpace: changing a state type is prohibited");
    if (const auto* old = std::get_if<Vec>(&slot)) {
        if (std::get<Vec>(value).size() != old->size())
            throw std::invalid_argument("StateSpace: changing a Vec length is prohibited");
    }

    validate(value);
    if (items_[index]->vectorized) validateInt64Range(value);

    slot = std::move(value);
}

std::size_t StateSpace::indexOf(const Str& name) const {
    const auto it = index_.find(name);
    if (it == index_.end())
        throw std::out_of_range("StateSpace: unknown state '" + name + "'");
    return it->second;
}

// ============================================================================
// Inspection
// ============================================================================

const State& StateSpace::state(const Str& name) const { return *items_.at(indexOf(name)); }
bool        StateSpace::contains(const Str& name) const { return index_.find(name) != index_.end(); }
std::size_t StateSpace::size() const noexcept { return items_.size(); }
std::size_t StateSpace::dim()  const noexcept { return vectorDim_; }
void        StateSpace::freeze() noexcept { frozen_ = true; }
bool        StateSpace::frozen() const noexcept { return frozen_; }

// A fresh vector every call: a stable, alias-free input for the optimizer. Only
// states that were registered vectorized participate, in registration order.
Vec StateSpace::vec() const {
    Vec out;
    out.reserve(vectorDim_);
    for (const auto& item : items_)
        if (item->vectorized) pack(item->value, out);
    return out;
}

// vec() order + offsets, computed from registration order (never from the
// unordered_map), so callers can address a component without guessing indices.
std::vector<StateSpace::ComponentMeta> StateSpace::layout() const {
    std::vector<ComponentMeta> out;
    std::size_t offset = 0;
    for (const auto& item : items_) {
        if (!item->vectorized) continue;
        const std::size_t n = components(item->value, true);
        out.push_back(ComponentMeta{ item->name, typeName(item->value), offset, n });
        offset += n;
    }
    return out;
}

} // namespace Space
} // namespace AuroX
