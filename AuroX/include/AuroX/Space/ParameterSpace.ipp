// ============================================================================
// AuroX - ParameterSpace template implementations
// ----------------------------------------------------------------------------
// This file is included automatically at the end of ParameterSpace.hpp.
// Do NOT include it directly and do NOT add non-template code here.
//
// Why templates live here instead of ParameterSpace.cpp:
//   template code must be visible to the compiler in every translation unit
//   that instantiates it, so it has to ship with the header. Everything that
//   is NOT a template lives in ParameterSpace.cpp.
// ============================================================================

#include <cmath>
#include <limits>
#include <utility>
#include <cassert>
#include <fstream>
#include <stdexcept>

namespace AuroX {
namespace Space {

// ============================================================================
// NumericTraits specializations
// ----------------------------------------------------------------------------
// Each `fits(double)` answers "is this double representable as T?" and MUST be
// checked before `back()`. P2: back() previously ran std::llround plus a plain
// cast with no range check, so a candidate like 1e30 could overflow the target
// type (undefined behaviour in llround) instead of being rejected.
// ============================================================================

template <> struct NumericTraits<bool> {
    static constexpr bool vectorizable = true;
    static double conv(bool v) { return v ? 1.0 : 0.0; }
    static bool fits(double d) { return std::isfinite(d); }
    static bool back(double d) { return d != 0.0; }
};

template <> struct NumericTraits<int> {
    static constexpr bool vectorizable = true;
    static double conv(int v) { return static_cast<double>(v); }
    // Reject anything that cannot be represented by int BEFORE rounding.
    static bool fits(double d) {
        if (!std::isfinite(d)) return false;
        if (d < static_cast<double>(std::numeric_limits<int>::min())) return false;
        if (d > static_cast<double>(std::numeric_limits<int>::max())) return false;
        return true;
    }
    static int back(double d) { return static_cast<int>(std::llround(d)); }
};

template <> struct NumericTraits<float> {
    static constexpr bool vectorizable = true;
    static double conv(float v) { return static_cast<double>(v); }
    // A finite double can still overflow float; reject it before the cast.
    static bool fits(double d) {
        if (!std::isfinite(d)) return false;
        constexpr double hi = static_cast<double>(std::numeric_limits<float>::max());
        return d >= -hi && d <= hi;
    }
    static float back(double d) { return static_cast<float>(d); }
};

template <> struct NumericTraits<double> {
    static constexpr bool vectorizable = true;
    static double conv(double v) { return v; }
    static bool fits(double d) { return std::isfinite(d); }
    static double back(double d) { return d; }
};

// ============================================================================
// TypedParameter<T>
// ============================================================================

template <typename T>
TypedParameter<T>::TypedParameter(std::string name, T def, T lo, T hi)
    : name_(std::move(name)), value(def), lower(lo), upper(hi), hasBounds(true) {}

template <typename T>
TypedParameter<T>::TypedParameter(std::string name, T def)
    : name_(std::move(name)), value(def), hasBounds(false) {}

template <typename T>
TypedParameter<T>::TypedParameter(std::string name, T def, std::vector<T> ch)
    : name_(std::move(name)), value(def), hasBounds(false), choices(std::move(ch)) {}

template <typename T>
const std::string& TypedParameter<T>::name() const { return name_; }

template <typename T>
ParameterKind TypedParameter<T>::kind() const {
    return choices.empty() ? ParameterKind::Continuous : ParameterKind::Discrete;
}

template <typename T>
ParameterTypeId TypedParameter<T>::typeId() const { return ParameterTypeInfo<T>::id; }

template <typename T>
const char* TypedParameter<T>::typeName() const { return ParameterTypeInfo<T>::name; }

template <typename T>
std::unique_ptr<ParameterBase> TypedParameter<T>::clone() const {
    return std::make_unique<TypedParameter<T>>(*this);
}

template <typename T>
bool TypedParameter<T>::isBounded() const { return hasBounds; }

// P1 (unified write validation): the ONE place that knows what a legal value
// is. Discrete parameters accept only a listed candidate; bounded continuous
// parameters accept [lower, upper]; everything else accepts any value.
template <typename T>
bool TypedParameter<T>::accepts(const T& cand) const {
    if (!choices.empty()) {
        for (const auto& c : choices)
            if (c == cand) return true;
        return false;
    }
    if (!hasBounds) return true;
    return cand >= lower && cand <= upper;
}

template <typename T>
bool TypedParameter<T>::isWithinBounds() const { return accepts(value); }

template <typename T>
json TypedParameter<T>::toJson() const {
    json j = json::object();
    j["name"] = name_;
    j["type"] = typeName();
    j["kind"] = choices.empty() ? "continuous" : "discrete";
    j["value"] = value;
    if (choices.empty()) {
        j["bounded"] = hasBounds;
        if (hasBounds) { j["lower"] = lower; j["upper"] = upper; }
    } else {
        j["choices"] = choices;
    }
    return j;
}

template <typename T>
void TypedParameter<T>::fromJson(const json& j) {
    if (j.contains("value")) value = j["value"].get<T>();
    if (j.contains("choices") && j["choices"].is_array() && !j["choices"].empty()) {
        choices = j["choices"].get<std::vector<T>>();
        hasBounds = false;
    } else if (j.contains("lower") && j.contains("upper")) {
        lower = j["lower"].get<T>();
        upper = j["upper"].get<T>();
        hasBounds = true;
    } else {
        hasBounds = false;
    }
}

template <typename T>
json TypedParameter<T>::valueToJson() const { return value; }

template <typename T>
void TypedParameter<T>::setValueFromJson(const json& v) { value = v.get<T>(); }

template <typename T>
bool TypedParameter<T>::packDouble(double& out) const {
    if constexpr (NumericTraits<T>::vectorizable) {
        out = NumericTraits<T>::conv(value);
        return true;
    } else {
        (void)out;
        return false;
    }
}

template <typename T>
bool TypedParameter<T>::unpackDouble(double v) {
    if constexpr (NumericTraits<T>::vectorizable) {
        value = NumericTraits<T>::back(v);
        return true;
    } else {
        (void)v;
        return false;
    }
}

template <typename T>
std::size_t TypedParameter<T>::componentCount() const {
    return NumericTraits<T>::vectorizable ? 1u : 0u;
}

// P-04: validate a candidate vector component before commit: it must be finite,
// representable by the target type, and inside the discrete candidate set (if
// any) or the [lower, upper] bounds (if bounded). Shares accepts() with the
// typed unvectorize<T>() path so both write entry points use the same rule.
template <typename T>
bool TypedParameter<T>::acceptsDouble(double v) const {
    if constexpr (NumericTraits<T>::vectorizable) {
        if (!std::isfinite(v)) return false;
        if (!NumericTraits<T>::fits(v)) return false;   // P2: range before conversion
        return accepts(NumericTraits<T>::back(v));
    } else {
        (void)v;
        return false;
    }
}

// ============================================================================
// ParameterSpace - template members
// ============================================================================

template <typename T>
TypedParameter<T>* ParameterSpace::getAs(const std::string& name) {
    auto* p = get(name);
    return (p && dynamic_cast<TypedParameter<T>*>(p)) ? static_cast<TypedParameter<T>*>(p) : nullptr;
}

template <typename T>
const TypedParameter<T>* ParameterSpace::getAs(const std::string& name) const {
    const auto* p = get(name);
    return (p && dynamic_cast<const TypedParameter<T>*>(p)) ? static_cast<const TypedParameter<T>*>(p) : nullptr;
}

template <typename T>
std::vector<T> ParameterSpace::vectorize() const {
    std::vector<T> out;
    for (const auto& p : items_) {
        if (auto* tp = dynamic_cast<TypedParameter<T>*>(p.get()))
            out.push_back(tp->value);
    }
    return out;
}

template <typename T>
void ParameterSpace::unvectorize(const std::vector<T>& v) {
    // P1 (unified write validation): mirror the unvectorizeDouble() contract -
    // require the full length, validate EVERY candidate with the parameter's own
    // accepts() rule, commit only after the whole vector passed (all-or-nothing).
    // Previously this wrote TypedParameter<T>::value directly, so an out-of-range
    // value such as 100 could overwrite a parameter bounded to [0, 3].
    std::size_t expected = 0;
    for (const auto& p : items_)
        if (dynamic_cast<TypedParameter<T>*>(p.get())) ++expected;
    if (v.size() != expected)
        throw std::invalid_argument("ParameterSpace: vector length mismatch");

    // Pass 1: validate.
    std::size_t i = 0;
    for (const auto& p : items_) {
        if (auto* tp = dynamic_cast<TypedParameter<T>*>(p.get())) {
            if (!tp->accepts(v[i]))
                throw std::invalid_argument(
                    "ParameterSpace: value out of range for parameter '" + tp->name() + "'");
            ++i;
        }
    }

    // Pass 2: commit - every value passed validation.
    i = 0;
    for (auto& p : items_)
        if (auto* tp = dynamic_cast<TypedParameter<T>*>(p.get())) tp->value = v[i++];
}

// ============================================================================
// Param<T>
// ============================================================================

template <typename T>
void Param<T>::bind(ParameterSpace& space, std::string_view name, T def, T lo, T hi) {
    name_ = std::string(name);
    value = def;
    raw_ = space.declare(name_, def, lo, hi);
    sync_to_raw();
}

template <typename T>
void Param<T>::bind(ParameterSpace& space, std::string_view name, T def) {
    name_ = std::string(name);
    value = def;
    raw_ = space.declare(name_, def);
    sync_to_raw();
}

template <typename T>
void Param<T>::bind(ParameterSpace& space, std::string_view name, T def, std::vector<T> choices) {
    name_ = std::string(name);
    value = def;
    raw_ = space.declareDiscrete(name_, def, std::move(choices));
    sync_to_raw();
}

template <typename T>
bool Param<T>::rebind(ParameterSpace& space) {
    if (name_.empty()) { raw_ = nullptr; return false; }
    raw_ = space.template getAs<T>(name_);
    return raw_ != nullptr;
}

template <typename T>
void Param<T>::sync_to_raw() { if (raw_) raw_->value = value; }

template <typename T>
void Param<T>::sync_from_raw() { if (raw_) value = raw_->value; }

template <typename T>
Param<T>& Param<T>::operator=(const T& v) { value = v; sync_to_raw(); return *this; }

template <typename T>
Param<T>& Param<T>::operator=(T&& v) { value = std::move(v); sync_to_raw(); return *this; }

template <typename T>
Param<T>::operator const T&() const { return value; }

template <typename T>
const T& Param<T>::get() const { return value; }

template <typename T>
const std::string& Param<T>::name() const { return name_; }

template <typename T>
bool Param<T>::isBound() const { return raw_ != nullptr; }

template <typename T>
bool Param<T>::isValid() const { return raw_ ? raw_->isValid() : false; }

// P1: only the const overload remains - see the header note. Direct writes to
// the TypedParameter<T> are no longer possible through a Param<T> proxy.
template <typename T>
const TypedParameter<T>* Param<T>::raw() const { return raw_; }

// ============================================================================
// PStruct<Derived>
// ============================================================================

template <typename Derived>
template <typename Fn>
void PStruct<Derived>::registerParams(Fn&& fn) {
    fn(space_);
    sync_from_space();
}

template <typename Derived>
void PStruct<Derived>::sync_to_space() {
    derived().for_each_param([](auto& p) { p.sync_to_raw(); });
}

template <typename Derived>
void PStruct<Derived>::sync_from_space() {
    derived().for_each_param([](auto& p) { p.sync_from_raw(); });
}

template <typename Derived>
bool PStruct<Derived>::rebind() {
    bool ok = true;
    derived().for_each_param([&](auto& p) { if (!p.rebind(space_)) ok = false; });
    return ok;
}

template <typename Derived>
ParameterSpace& PStruct<Derived>::space() {
    sync_to_space();
    return space_;
}

template <typename Derived>
const ParameterSpace& PStruct<Derived>::space() const {
    const_cast<PStruct*>(this)->sync_to_space();
    return space_;
}

template <typename Derived>
json PStruct<Derived>::toJson() const {
    auto* self = const_cast<PStruct*>(this);
    self->sync_to_space();
    return space_.toJson();
}

template <typename Derived>
void PStruct<Derived>::fromJson(const json& j) {
    space_.fromJson(j);
    rebind();
    sync_from_space();
}
// P-05: definition was declared in the header but left commented out here, so
// any instantiation produced a missing definition / link error. Restored.
template <typename Derived>
bool PStruct<Derived>::tryFromJson(const json& j) noexcept {
    try {
        fromJson(j);
        return true;
    } catch (...) {
        return false;
    }
}

template <typename Derived>
std::vector<double> PStruct<Derived>::vectorize() const {
    auto* self = const_cast<PStruct*>(this);
    self->sync_to_space();
    return space_.vectorizeDouble();
}

template <typename Derived>
std::vector<double> PStruct<Derived>::vectorizeDouble() const { return vectorize(); }

template <typename Derived>
void PStruct<Derived>::unvectorize(const std::vector<double>& v) {
    space_.unvectorizeDouble(v);
    sync_from_space();
}

template <typename Derived>
void PStruct<Derived>::unvectorizeDouble(const std::vector<double>& v) { unvectorize(v); }

template <typename Derived>
bool PStruct<Derived>::validate() const {
    auto* self = const_cast<PStruct*>(this);
    self->sync_to_space();
    bool valid = true;
    for (std::size_t i = 0; i < space_.size(); ++i) {
        const auto* p = space_.at(i);
        if (p && !p->isValid()) { valid = false; break; }
    }
#ifndef NDEBUG
    // Debug assertion: every declared Param/EnumParam must be bound via
    // PARAM_BIND inside PARAM_REG. A forgotten PARAM_BIND leaves the member
    // unbound and unsynced, which is a silent user error we want to catch.
    derived().for_each_param([&](const auto& p) {
        assert(p.isBound() && "PStruct::validate: a Param/EnumParam was never "
               "bind()-ed - did you forget a PARAM_BIND for it inside PARAM_REG?");
    });
#endif
    return valid;
}

template <typename Derived>
std::size_t PStruct<Derived>::size() const { return space_.size(); }

template <typename Derived>
bool PStruct<Derived>::empty() const { return space_.empty(); }

template <typename Derived>
std::vector<std::string> PStruct<Derived>::names() const { return space_.names(); }

template <typename Derived>
std::vector<ParameterSpace::ComponentMeta> PStruct<Derived>::componentLayout() const {
    return space_.componentLayout();
}

template <typename Derived>
ParameterBase* PStruct<Derived>::get(const std::string& name) {
    sync_to_space();
    return space_.get(name);
}

template <typename Derived>
const ParameterBase* PStruct<Derived>::get(const std::string& name) const {
    auto* self = const_cast<PStruct*>(this);
    self->sync_to_space();
    return space_.get(name);
}

template <typename Derived>
Derived& PStruct<Derived>::derived() { return static_cast<Derived&>(*this); }

template <typename Derived>
const Derived& PStruct<Derived>::derived() const {
    return static_cast<const Derived&>(*this);
}

// Non-throwing file loader: open `path`, parse JSON, then call tryFromJson.
// Works for both ParameterSpace and PStruct<Derived> (both expose tryFromJson).
template <typename SpaceT>
bool tryLoadJson(SpaceT& s, const std::string& path) noexcept {
    try {
        std::ifstream f(path);
        if (!f) return false;
        json j;
        f >> j;
        return s.tryFromJson(j);
    } catch (...) {
        return false;
    }
}

} // namespace Space
} // namespace AuroX
