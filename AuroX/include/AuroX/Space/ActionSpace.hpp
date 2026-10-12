#pragma once

// ============================================================================
// AuroX - ActionSpace
// ----------------------------------------------------------------------------
// Design rule: add no entity unless it is necessary.
//
//   3 types     Condition / Action / ActionSpace
//   1 free fn   hashOf
//   0 macros
//   standard library only
//
// ActionSpace is an ACTION CONTAINER + EXECUTION GATE. It owns no system state
// and performs no search:
//   * no cur_ / cur()      - the real state lives in StateSpace (single source)
//   * no plan()/replan()   - planning lives in AuroX::Planner (Planner.hpp)
//   * run() only checks the conditions on the state YOU pass, then executes; it
//     never predicts and never advances any internal state.
//
// Dependency is ONE-WAY: Planner includes ActionSpace, never the reverse.
//
// File layout (declaration / implementation split):
//   include/AuroX/Space/ActionSpace.hpp  - declarations only (this file)
//   src/Space/ActionSpace.cpp            - all implementations
//
// The three usage tiers are NESTED, not parallel:
//   tier 3 simple  : Action{name,type,run} + reg + run     -> a bag of actions
//   tier 1 standard: + conditions + nxt + Planner::bfs     -> pick a chain
//   tier 2 advanced: + ban + Planner::bfs                  -> replan on anomaly
//
// Contract notes:
//   * Action::nxt is a PURE prediction (same input -> same output, no side
//     effects, never touches real state). Its result must keep the input
//     dimension and hold finite values only; Planner::bfs enforces this.
//   * Action::next() FAILS (throws std::logic_error) when nxt is missing: an
//     unknown effect must never be mistaken for "state unchanged".
//   * ActionSpace::run() returning true means the LOCAL CLOSURE returned
//     normally, NOT that a device executed successfully. Confirm the real
//     result through StateSpace and feed the measured state back yourself.
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
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
using Vec = std::vector<double>;        // Neutral state vector: contents are up to the user.

// -- free function -----------------------------------------------------------
AUROX_API std::uint64_t hashOf(std::string_view s);   // FNV-1a 64 -> global key

// -- 1. Condition: a single predicate ---------------------------------------
struct AUROX_API Condition {
    Str                             desc;   // human-readable
    std::function<bool(const Vec&)> ok;     // machine-checkable; empty = always true

    bool test(const Vec& x) const;
};

// -- 2. Action: a plain data struct -----------------------------------------
using RunFn = std::function<void()>;               // body: no args; wrap external fns in a closure
using NxtFn = std::function<Vec(const Vec&)>;      // end-state prediction (pure)

struct AUROX_API Action {
    Str                    name;        // action name
    Str                    type;        // action family (with name -> hash)
    std::uint64_t          hash = 0;    // global key; 0 -> derived automatically by reg()
    RunFn                  run;         // execution body
    std::vector<Condition> conditions;  // guards tested against the state passed to run()
    NxtFn                  nxt;         // end-state prediction; empty = NOT predictable

    bool can (const Vec& x) const;      // are all conditions satisfied?
    Vec  next(const Vec& x) const;      // predict; throws std::logic_error when nxt is empty
    void exec() const;                  // execute (no-op when run is empty)
};
using Act = Action;                     // convenience alias

// -- 3. ActionSpace ----------------------------------------------------------
class AUROX_API ActionSpace {
public:
    ActionSpace() = default;

    // The space stores Action* pointing into the derived type's OWN members; a
    // copy would point into a different object -> copying is disabled.
    ActionSpace(const ActionSpace&)            = delete;
    ActionSpace& operator=(const ActionSpace&) = delete;

    // Register: hand all Action members of the derived type to the space at
    // once (not one add() at a time). This is the only template here; it just
    // forwards, so its body must stay in the header.
    template <class... A> void reg(A&... as) { (reg_(as), ...); }

    // Lookup (read/write), read-only lookup for the Planner, and hash-name lookup.
    Action*       get(std::uint64_t h);
    const Action* get(std::uint64_t h) const;         // const: for Planner's read-only scan
    Action*       get(const Str& nm);                 // by name: ambiguous when two
                                                      // families share the name -> first wins
    Action*       get(const Str& type, const Str& nm);// type+name: hash-disambiguated

    // Registration order, read-only. The Planner enumerates candidates through
    // this so BFS results stay reproducible. Do not mutate the space while
    // planning.
    const std::vector<std::uint64_t>& ids() const;

    std::size_t size() const;

    // Execute: gate on the conditions tested against the state you pass, then
    // run the body. Returns false for an unknown/banned action or an empty run
    // closure. It NEVER predicts or advances any state.
    bool run(std::uint64_t h, const Vec& actualState);

    // No-state shorthand: without a state there is nothing to test conditions
    // against, so it is valid ONLY for actions that have no conditions; an
    // action that has conditions returns false here (never bypassed).
    bool run(std::uint64_t h);

    // Ban set: both planning and execution honor it.
    void ban   (std::uint64_t h);
    void unban (std::uint64_t h);
    bool banned(std::uint64_t h) const;

    virtual ~ActionSpace() = default;

private:
    void reg_(Action& a);

    std::unordered_map<std::uint64_t, Action*> byHash_; // hash -> action (zero copy)
    std::vector<std::uint64_t>                 order_;  // declaration order (reproducible)
    std::unordered_set<std::uint64_t>          ban_;    // ban set
};

using ASpace = ActionSpace;             // convenience alias

} // namespace Space
} // namespace AuroX
