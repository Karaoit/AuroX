#pragma once

// ============================================================================
// AuroX - Planner
// ----------------------------------------------------------------------------
// Design rule: add no entity unless it is necessary.
//
//   1 struct   Chain               (the planning product: hashes only)
//   2 aliases  PlanFn / PredictFn  (std::function - NOT extra classes)
//   1 function bfs()               (the only built-in planner, a minimal stub)
//   0 macros
//   standard library only
//
// Dependency direction is ONE-WAY: Planner -> Space. ActionSpace never includes
// Planner. Planning only READS an ActionSpace and returns registered action
// hashes; it never executes an action, never writes a StateSpace and never
// keeps the start state.
//
// File layout (declaration / implementation split):
//   include/AuroX/Planner/Planner.hpp  - declarations only (this file)
//   src/Planner/BFS.cpp                - the bfs() implementation
//
// A predicted state is a CANDIDATE, not a device result. Running an action is
// ActionSpace's job and must be confirmed by real feedback.
//
// Usage tiers:
//   standard : Planner::bfs(actions, start, goal)                    -> a chain
//   advanced : Planner::bfs(actions, start, goal, predict)           -> world model
//              or a user PlanFn that returns a Chain directly         -> custom/LLM
// ============================================================================

#include "AuroX/Space/ActionSpace.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
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
namespace Planner {

// The planning product: a sequence of REGISTERED action hashes. V0.5 returns
// hashes only - no per-step parameter struct and no search graph.
struct AUROX_API Chain {
    bool                       ok  = false;
    std::string                why;     // failure reason (ok == false)
    std::vector<std::uint64_t> acts;    // action hash sequence (the chain)
};

// State-prediction contract supplied by the caller (e.g. a world model).
//   returns std::nullopt -> "this action cannot be predicted in this state",
//                           which is NOT the same as "the state is unchanged".
// When a PredictFn is provided, bfs() uses ONLY it and never falls back to
// Action::nxt.
using PredictFn = std::function<std::optional<Space::Vec>(
    const Space::Vec& state, const Space::Action& action)>;

// The custom-algorithm / language-model entry point. Implement any function or
// lambda with this shape and return a Chain; no base class is required. AuroX
// does not implement model calls, prompt protocols or networking in V0.5 - the
// caller parses model output, maps it to registered hashes, validates them and
// returns a Chain.
using PlanFn = std::function<Chain(
    const Space::ActionSpace& actions,
    const Space::Vec& start,
    const Space::Vec& goal)>;

// Minimal forward BFS over DISCRETE actions with a deterministic single-step
// prediction. It is a placeholder for later planners, not a claim of
// optimality for continuous or industrial processes.
//
//   predict  - when non-empty, the ONLY transition source; returning nullopt
//              skips the action. When empty, only actions that carry a `nxt`
//              are used (actions without a prediction are skipped).
//   maxDepth - layer budget (>= 1)
//   eps      - SQUARED-distance tolerance (>= 0, finite)
//   maxNodes - total node budget INCLUDING the start (>= 1)
//
// All input problems are reported as Chain{ok=false, why=...}; bfs() does not
// throw for bad arguments. It never runs an action and never mutates its input.
AUROX_API Chain bfs(
    const Space::ActionSpace& actions,
    const Space::Vec& start,
    const Space::Vec& goal,
    const PredictFn& predict = {},
    int maxDepth = 8,
    double eps = 1e-6,
    std::size_t maxNodes = 10000);

} // namespace Planner
} // namespace AuroX
