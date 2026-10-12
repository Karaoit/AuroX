// ============================================================================
// AuroX - StateSpace template implementations
// ----------------------------------------------------------------------------
// This file is included automatically at the end of StateSpace.hpp.
// Do NOT include it directly and do NOT add non-template code here.
//
// Why templates live here instead of StateSpace.cpp:
//   template code must be visible to the compiler in every translation unit
//   that instantiates it, so it has to ship with the header. Everything that
//   is NOT a template lives in StateSpace.cpp.
// ============================================================================

#pragma once

#include <stdexcept>
#include <utility>

namespace AuroX {
namespace Space {

// ============================================================================
// Registration
// ============================================================================

template <class T>
std::size_t StateSpace::add(Str name, T initial, Str desc, bool vectorized) {
    static_assert(detail::supported_state_v<T>, "StateSpace: unsupported value type");
    return addRaw(std::move(name), StateValue{ std::move(initial) },
                  std::move(desc), vectorized);
}

template <class T>
void StateSpace::bind(SField<T>& field, Str name, T initial, Str desc, bool vectorized) {
    static_assert(detail::supported_state_v<T>, "StateSpace: unsupported value type");
    if (field.bound())
        throw std::logic_error("StateSpace: SField is already bound");
    const std::size_t id =
        add<T>(std::move(name), std::move(initial), std::move(desc), vectorized);
    field.space_ = this;
    field.index_ = id;
}

// ============================================================================
// Dynamic access
// ============================================================================

template <class T>
T StateSpace::get(const Str& name) const {
    static_assert(detail::supported_state_v<T>, "StateSpace: unsupported value type");
    return getRaw<T>(indexOf(name));
}

template <class T>
void StateSpace::set(const Str& name, T value) {
    static_assert(detail::supported_state_v<T>, "StateSpace: unsupported value type");
    setRaw(indexOf(name), StateValue{ std::move(value) });
}

// Type-erased read by stable index. Throws on a type mismatch instead of
// silently returning a default value, so a wrong get<T>() is never mistaken for
// a legitimate 0/false state.
template <class T>
T StateSpace::getRaw(std::size_t index) const {
    static_assert(detail::supported_state_v<T>, "StateSpace: unsupported value type");
    const auto* p = std::get_if<T>(&items_.at(index)->value);
    if (!p)
        throw std::invalid_argument("StateSpace: state type mismatch");
    return *p;
}

// ============================================================================
// SField<T>
// ============================================================================

template <class T>
T SField<T>::get() const {
    if (!space_) throw std::logic_error("StateSpace: SField is not bound");
    return space_->template getRaw<T>(index_);
}

template <class T>
SField<T>::operator T() const { return get(); }

template <class T>
SField<T>& SField<T>::operator=(T value) {
    if (!space_) throw std::logic_error("StateSpace: SField is not bound");
    space_->setRaw(index_, StateValue{ std::move(value) });
    return *this;
}

template <class T>
bool SField<T>::bound() const noexcept { return space_ != nullptr; }

} // namespace Space
} // namespace AuroX
