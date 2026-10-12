// ============================================================================
// AuroX - ActionSpace implementation
// ----------------------------------------------------------------------------
// Mirrors include/AuroX/Space/ActionSpace.hpp one-to-one:
//   the header holds declarations only; every implementation lives here (this
//   is NOT header-only).
//
// One fact you must know: the space stores Action*, pointing at the DERIVED
// type's own members. A copied space would point at another object's members,
// so copying is deleted in the header.
//
// This file contains NO search and NO state: planning moved to AuroX::Planner
// (src/Planner/BFS.cpp) and the current state lives in StateSpace.
//
// Dependencies: standard library only. Parameter / state / observation spaces
// are not included here - capture them in closures when needed.
// ============================================================================

#include "AuroX/Space/ActionSpace.hpp"

#include <stdexcept>
#include <unordered_map>

namespace AuroX {
namespace Space {

// -- free function -----------------------------------------------------------
std::uint64_t hashOf(std::string_view s) {
    std::uint64_t h = 14695981039346656037ULL;            // FNV-1a 64 offset basis
    for (char c : s) {
        h ^= static_cast<std::uint64_t>(static_cast<unsigned char>(c));
        h *= 1099511628211ULL;                            // FNV-1a 64 prime
    }
    return h;
}

// -- Condition ---------------------------------------------------------------
bool Condition::test(const Vec& x) const {
    return !ok || ok(x);                                  // empty predicate = always true
}

// -- Action ------------------------------------------------------------------
bool Action::can(const Vec& x) const {
    for (const Condition& c : conditions)
        if (!c.test(x)) return false;
    return true;
}

// An unknown effect is NOT "the state is unchanged": fail loudly instead of
// silently pretending a transition exists (see the header contract).
Vec Action::next(const Vec& x) const {
    if (!nxt) throw std::logic_error("ActionSpace: action has no nxt prediction");
    return nxt(x);
}

void Action::exec() const {
    if (run) run();
}

// -- ActionSpace -------------------------------------------------------------
// Register: auto-fill hash; re-registering the same object is ignored; a hash
// collision throws (never silently overwrite).
void ActionSpace::reg_(Action& a) {
    if (a.hash == 0) a.hash = hashOf(a.type + "/" + a.name);

    const auto it = byHash_.find(a.hash);
    if (it != byHash_.end()) {
        if (it->second == &a) return;
        throw std::runtime_error("ActionSpace: hash collision -> " + a.type + "/" + a.name);
    }
    byHash_.emplace(a.hash, &a);
    order_.push_back(a.hash);                             // keep declaration order
}

Action* ActionSpace::get(std::uint64_t h) {
    const auto it = byHash_.find(h);
    return it == byHash_.end() ? nullptr : it->second;
}

// Read-only lookup used by the Planner: never hands out a mutable handle.
const Action* ActionSpace::get(std::uint64_t h) const {
    const auto it = byHash_.find(h);
    return it == byHash_.end() ? nullptr : it->second;
}

// Name-only lookup: linear scan, first match wins. Ambiguous when two action
// families share a name - use get(type, name) or get(hash) in that case.
Action* ActionSpace::get(const Str& nm) {
    for (std::uint64_t h : order_) {
        Action* a = get(h);
        if (a && a->name == nm) return a;
    }
    return nullptr;
}

// type+name lookup: the registration hash is derived from "type/name", so the
// pair resolves in O(1) through byHash_ - no linear scan and no first-match
// ambiguity between same-named actions of different families.
// Falls back to a scan for actions whose hash was supplied manually.
Action* ActionSpace::get(const Str& type, const Str& nm) {
    const std::uint64_t h = hashOf(type + "/" + nm);
    if (Action* a = get(h))
        if (a->type == type && a->name == nm) return a;
    for (std::uint64_t hh : order_) {
        Action* a = get(hh);
        if (a && a->type == type && a->name == nm) return a;
    }
    return nullptr;
}

const std::vector<std::uint64_t>& ActionSpace::ids() const { return order_; }

std::size_t ActionSpace::size() const { return byHash_.size(); }

// Execute: test the conditions against the state YOU pass, then run the body.
// The state is NOT advanced and NOT predicted; an empty run closure is not a
// success, and exceptions from the body propagate (never swallowed here).
bool ActionSpace::run(std::uint64_t h, const Vec& actualState) {
    if (banned(h))            return false;               // banned
    Action* a = get(h);
    if (!a)                   return false;               // unknown
    if (!a->run)              return false;               // nothing to execute
    if (!a->can(actualState)) return false;               // conditions not all satisfied
    a->exec();                                            // closure captures everything
    return true;                                          // local closure returned normally
}

// No-state shorthand: there is no state to test conditions against, so only
// actions WITHOUT conditions are valid here.
bool ActionSpace::run(std::uint64_t h) {
    if (banned(h))              return false;
    Action* a = get(h);
    if (!a)                     return false;
    if (!a->run)                return false;
    if (!a->conditions.empty()) return false;             // never bypass conditions
    a->exec();
    return true;
}

void ActionSpace::ban   (std::uint64_t h) { ban_.insert(h); }
void ActionSpace::unban (std::uint64_t h) { ban_.erase(h);  }
bool ActionSpace::banned(std::uint64_t h) const { return ban_.count(h) != 0; }

} // namespace Space
} // namespace AuroX
