// ============================================================================
// AuroX - Planner::bfs implementation
// ----------------------------------------------------------------------------
// Mirrors include/AuroX/Planner/Planner.hpp. This file implements the ONLY
// built-in planner: a minimal forward BFS over discrete actions with a
// deterministic single-step prediction.
//
// It READS an ActionSpace through its public read-only surface:
//   ids()    - candidate order (registration order -> reproducible results)
//   get(id)  - const lookup -> const Action*
//   banned() - the ban set is honored by both planning and execution
//   Action::can() / Action::nxt - condition gate and pure prediction
//
// It NEVER executes an action, NEVER writes a StateSpace and NEVER keeps the
// start state. Distance is a private detail here: Space no longer exposes a
// distance function, so task code does its own prediction/actual comparison.
//
// Dependencies: ActionSpace.hpp + the standard library only.
// ============================================================================

#include "AuroX/Planner/Planner.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace AuroX {
namespace Planner {

namespace {

// A usable state keeps its dimension and holds finite values only.
bool finiteState(const Space::Vec& x) {
    for (double v : x)
        if (!std::isfinite(v)) return false;
    return true;
}

// Squared Euclidean distance. Private to the Planner; `eps` is expressed in
// squared-error units (no sqrt).
double sqDist(const Space::Vec& a, const Space::Vec& b) {
    double d = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double delta = a[i] - b[i];
        d += delta * delta;
    }
    return d;
}

} // namespace

Chain bfs(const Space::ActionSpace& actions,
          const Space::Vec& start,
          const Space::Vec& goal,
          const PredictFn& predict,
          int maxDepth,
          double eps,
          std::size_t maxNodes) {
    Chain out;

    // (1) Argument validation -> Chain{ok=false, why=...}, never a throw.
    if (maxDepth < 1)                     { out.why = "invalid maxDepth";          return out; }
    if (maxNodes < 1)                     { out.why = "invalid maxNodes";          return out; }
    if (!std::isfinite(eps) || eps < 0.0) { out.why = "invalid eps";               return out; }
    if (start.size() != goal.size())      { out.why = "state dimension mismatch";  return out; }
    if (!finiteState(start))              { out.why = "non-finite start";         return out; }
    if (!finiteState(goal))               { out.why = "non-finite goal";          return out; }

    const std::size_t dim = start.size();

    struct Node { Space::Vec x; int par; std::uint64_t act; };

    // (2) The start already satisfies the goal.
    if (sqDist(start, goal) <= eps) { out.ok = true; return out; }

    std::vector<Node> seen{ { start, -1, 0 } };   // start counts toward the budget
    std::size_t explored = 1;

    std::vector<int> layer{ 0 };

    // (3) Forward BFS, layer by layer.
    for (int depth = 0; depth < maxDepth && !layer.empty(); ++depth) {
        std::vector<int> next;

        for (int i : layer) {
            // (3b) Enumerate candidates in registration order for reproducibility.
            for (std::uint64_t id : actions.ids()) {
                // (8) Node budget (start included).
                if (explored >= maxNodes) { out.why = "node budget exhausted"; return out; }
                // (4) Skip banned, non-executable, or condition-blocked candidates.
                if (actions.banned(id)) continue;
                const Space::Action* a = actions.get(id);
                if (!a)          continue;
                if (!a->run)     continue;
                if (!a->can(seen[i].x)) continue;

                // (5) Transition source: predict() EXCLUSIVELY when provided,
                // otherwise only actions that carry a nxt. No fallback between
                // the two, and a nullopt prediction skips the action.
                Space::Vec nx;
                bool predicted = false;
                if (predict) {
                    if (auto p = predict(seen[i].x, *a)) { nx = std::move(*p); predicted = true; }
                } else if (a->nxt) {
                    nx = a->nxt(seen[i].x);
                    predicted = true;
                }
                if (!predicted) continue;

                // (6) A predicted state must keep the dimension and stay finite.
                if (nx.size() != dim || !finiteState(nx)) {
                    out.why = "action '" + a->name + "' produced an invalid predicted state";
                    return out;
                }

                // (7) Simple exact/eps de-duplication.
                bool dup = false;
                for (const Node& n : seen)
                    if (sqDist(n.x, nx) <= eps) { dup = true; break; }
                if (dup) continue;

                seen.push_back({ nx, i, id });
                const int k = static_cast<int>(seen.size()) - 1;
                ++explored;

                if (sqDist(nx, goal) <= eps) {                 // hit -> backtrack
                    std::vector<std::uint64_t> r;
                    for (int p = k; p > 0; p = seen[p].par) r.push_back(seen[p].act);
                    out.acts.assign(r.rbegin(), r.rend());
                    out.ok = true;
                    return out;
                }
                next.push_back(k);
            }
        }
        layer = std::move(next);
    }

    out.why = "unreachable within depth " + std::to_string(maxDepth);
    return out;
}

} // namespace Planner
} // namespace AuroX
