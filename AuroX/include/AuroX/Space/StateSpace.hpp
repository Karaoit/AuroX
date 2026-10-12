#pragma once

// ============================================================================
// AuroX - StateSpace
// ----------------------------------------------------------------------------
// Purpose:
//   One owned, readable and numerically consistent container for the current
//   state. It is the single source of truth for state values; nothing here
//   predicts, plans or estimates.
//
// Design contract (see SSpace重构.md ch. 4.3 / 5.1 / 11):
//   * ONE data source  - every value lives in exactly one State::value.
//                        SField<T> is a non-owning index proxy, never a copy,
//                        so the two access styles below always hit the SAME
//                        value and never need a sync_to_space()/sync_from_space()
//                        call.
//   * TWO access styles- dynamic : states.get<T>("velocity") / set<T>(...)
//                        typed   : robot.velocity = 1.2 / robot.velocity.get()
//   * NUMERIC view     - vec() emits the continuous double vector consumed by the
//                        optimizer and Planner::bfs(); layout() explains the
//                        element order and offsets instead of letting callers
//                        guess indices from map iteration order.
//   * FROZEN schema    - freeze() locks the SET of states (names/types/shape),
//                        not their values; set() keeps working afterwards.
//
// Supported value types (v1):
//   bool, int, std::int64_t, float, double, Str, Vec
//   vectorized by default : int, std::int64_t, float, double, Vec
//   excluded by default   : bool, Str  (opt in explicitly when required)
//
// Non-goals (design doc ch. 3.2): predict/transition, plan/replan,
// estimate/sensor fusion, observation channels, history/persistence, gradients,
// implicit unvec(), snapshot(), setBatch(), remove()/replace(), threading.
//
// File layout (declaration / implementation split, same as ParameterSpace):
//   StateSpace.hpp - declarations, comments and examples only (this file)
//   StateSpace.ipp - template implementations, included at the end of this
//                    header; do NOT include it directly
//   StateSpace.cpp - all non-template implementations
//
// ----------------------------------------------------------------------------
// Usage examples
// ----------------------------------------------------------------------------
// 1) Simple - dynamic, name based:
//
//      SSp states;
//      states.add("position_x", 1.5, "X coordinate");
//      states.add("position_y", 2.0, "Y coordinate");
//      states.add("velocity",   0.8, "Speed");
//      states.add("mode", Str{"auto"}, "Control mode");   // excluded from vec()
//      states.freeze();
//
//      states.set("velocity", 1.2);
//      double v  = states.get<double>("velocity");         // 1.2
//      Vec    in = states.vec();                           // {1.5, 2.0, 1.2}
//
// 2) Standard - strong-typed members alongside dynamic access:
//
//      struct RobotState : SSp {
//          SField<double> x, y, speed;
//          SField<Str>    mode;
//          RobotState() {
//              bind(x,     "x",     0.0, "X coordinate");
//              bind(y,     "y",     0.0, "Y coordinate");
//              bind(speed, "speed", 0.0, "Linear speed");
//              bind(mode,  "mode",  Str{"idle"}, "Mode");
//              freeze();
//          }
//      };
//
//      RobotState robot;
//      robot.x = 1.5;                       // typed write
//      robot.set("speed", 1.2);             // dynamic write, same value as below
//      double s = robot.speed.get();        // 1.2
//      double x = robot.get<double>("x");   // 1.5
//
// 3) Advanced - stable layout handed to the action layer:
//
//      RobotState robot;
//      auto meta = robot.layout();   // x: offset 0, y: offset 1, speed: offset 2
//      ActionSpace actions;
//      // Hand the numeric view straight to planning (the space stores no copy):
//      auto chain = Planner::bfs(actions, robot.vec(), goal);
//
// Note: `auto v = robot.speed;` is intentionally not supported - SField is a
// proxy, not a value, and copying it is deleted. Write `robot.speed.get()` or
// `double v = robot.speed;` (the implicit read conversion returns by value).
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#ifndef AUROX_API
#  if defined(_WIN32) && defined(AUROX_BUILD_DLL)
#    define AUROX_API __declspec(dllexport)
#  elif defined(_WIN32) && defined(AUROX_USE_DLL)
#    define AUROX_API __declspec(dllimport)
#  else
#    define AUROX_API
#  endif
#endif

namespace AuroX {
namespace Space {

using Str = std::string;
using Vec = std::vector<double>;

// Heterogeneous state value: the fixed set of types supported in v1.
using StateValue = std::variant<bool, int, std::int64_t, float, double, Str, Vec>;

namespace detail {

// Compile-time type filters. They stay in the header (data-only traits, like
// ParameterTypeInfo in ParameterSpace.hpp).
template <class T>
inline constexpr bool supported_state_v =
    std::is_same_v<T, bool> || std::is_same_v<T, int> ||
    std::is_same_v<T, std::int64_t> || std::is_same_v<T, float> ||
    std::is_same_v<T, double> || std::is_same_v<T, Str> ||
    std::is_same_v<T, Vec>;

// bool and Str are excluded from vec() unless the caller opts in explicitly.
template <class T>
inline constexpr bool default_vectorized_v =
    std::is_same_v<T, int> || std::is_same_v<T, std::int64_t> ||
    std::is_same_v<T, float> || std::is_same_v<T, double> ||
    std::is_same_v<T, Vec>;

} // namespace detail

// ----------------------------------------------------------------------------
// State: the record stored (and owned) by the space.
// ----------------------------------------------------------------------------
struct State {
    Str        name;              // unique, non-empty
    Str        desc;              // optional human-readable description
    StateValue value;             // the ONE real value
    bool       vectorized = false; // does it contribute to vec()?
};

template <class T> class SField;   // strong-typed index proxy (defined below)

// ============================================================================
// StateSpace
// ============================================================================

class AUROX_API StateSpace {
public:
    // Layout entry for one state inside vec(): where it starts and how many
    // doubles it occupies. Nested type - no extra polymorphic layer is added.
    struct ComponentMeta {
        Str         name;
        Str         type;         // "bool"|"int"|"int64"|"float"|"double"|
                                  // "string"|"vector<double>"
        std::size_t offset = 0;   // first index of this state inside vec()
        std::size_t components = 0;
    };

    StateSpace() = default;
    // One space owns its State objects; SField<T> proxies store its address plus
    // a stable index. Copy/move are disabled so a proxy can never be silently
    // re-pointed at a different (or moved-from) container.
    StateSpace(const StateSpace&)            = delete;
    StateSpace& operator=(const StateSpace&) = delete;
    StateSpace(StateSpace&&)                 = delete;
    StateSpace& operator=(StateSpace&&)      = delete;

    // ---- registration ------------------------------------------------------
    // Register a state, return its stable index. `vectorized` decides whether it
    // joins vec(); the default is true for numeric types and false for bool/Str.
    // Throws on: empty name, duplicate name, frozen schema, unsupported value
    // (non-finite number, empty Vec, out-of-range int64 when vectorized).
    template <class T>
    std::size_t add(Str name, T initial, Str desc = {},
                    bool vectorized = detail::default_vectorized_v<T>);

    // Register a state and bind it to `field` in one step (typed-member usage).
    // Throws if `field` is already bound.
    template <class T>
    void bind(SField<T>& field, Str name, T initial, Str desc = {},
              bool vectorized = detail::default_vectorized_v<T>);

    // ---- dynamic access ----------------------------------------------------
    // get<T>() returns a COPY (no writable reference can escape and bypass the
    // validation in set()). T must match the registered type exactly.
    template <class T> T    get(const Str& name) const;
    template <class T> void set(const Str& name, T value);

    // ---- inspection --------------------------------------------------------
    const State& state(const Str& name) const;   // full record; throws if unknown
    bool         contains(const Str& name) const;
    std::size_t  size() const noexcept;          // entries (incl. non-vectorized)
    std::size_t  dim()  const noexcept;          // length of vec()
    Vec          vec()  const;                   // numeric view (a fresh copy)
    std::vector<ComponentMeta> layout() const;   // vec() order + offsets

    // ---- schema lock -------------------------------------------------------
    void freeze() noexcept;                      // one way; no unfreeze
    bool frozen() const noexcept;

private:
    template <class T> friend class SField;

    // Non-template core: type-erased creation / update / validation.
    std::size_t addRaw(Str name, StateValue value, Str desc, bool vectorized);
    void        setRaw(std::size_t index, StateValue value);
    std::size_t indexOf(const Str& name) const;
    template <class T> T getRaw(std::size_t index) const;

    // unique_ptr keeps every State address stable even as the vector grows, so
    // SField indices stay valid for the lifetime of the space.
    std::vector<std::unique_ptr<State>> items_;
    std::unordered_map<Str, std::size_t> index_;   // name -> stable index
    std::size_t vectorDim_ = 0;                    // cached vec() dimension
    bool        frozen_    = false;
};

using SSp = StateSpace;   // convenience alias

// ============================================================================
// SField<T>: a lightweight, non-owning proxy to one state inside a StateSpace.
// ----------------------------------------------------------------------------
//   robot.speed.get()        -> read (copy)
//   double v = robot.speed;  -> implicit read (copy)
//   robot.speed = 1.2;       -> validated write
// It stores NO value of its own, so it can never diverge from the space.
// Copy/move are deleted; the space must outlive its fields.
// ============================================================================

template <class T>
class SField {
public:
    SField() = default;
    SField(const SField&)            = delete;
    SField& operator=(const SField&) = delete;
    SField(SField&&)                 = delete;
    SField& operator=(SField&&)      = delete;

    T       get() const;         // read the owned value (by copy)
    operator T() const;          // implicit read: double v = robot.speed;
    SField& operator=(T value);  // validated write to the owned value
    bool    bound() const noexcept;

private:
    friend class StateSpace;
    StateSpace* space_ = nullptr;
    std::size_t index_ = 0;
};

} // namespace Space
} // namespace AuroX

// Template implementations (add/bind/get/set/getRaw, SField<T>).
#include "StateSpace.ipp"
