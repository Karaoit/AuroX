#include "AuroX/Space/ActionSpace.hpp"

#include <algorithm>

namespace AuroX {
namespace Space {

namespace {
    // Fallback learning rate when no optimizer is wired (standalone use).
    constexpr double kFallbackLr = 0.01;
}  // namespace

// ---------------------------------------------------------------------------
// ParameterUpdateAction
// ---------------------------------------------------------------------------
void ParameterUpdateAction::apply(ParameterSpace& space,
                                  const std::vector<double>& gradient,
                                  const ParameterUpdateFn& updateSource) const {
    std::vector<double> delta;
    if (updateSource) {
        // Basis from the optimizer: ask the wired update source for the delta.
        delta = updateSource(space, gradient);
    } else {
        // Standalone fallback: a plain gradient step so the action is still
        // meaningful without a live optimizer.
        auto theta = space.vectorizeDouble();
        delta.assign(theta.size(), 0.0);
        const size_t n = std::min(theta.size(), gradient.size());
        for (size_t i = 0; i < n; ++i) delta[i] = -kFallbackLr * gradient[i];
    }

    auto theta = space.vectorizeDouble();
    const size_t n = std::min(theta.size(), delta.size());
    for (size_t i = 0; i < n; ++i) theta[i] += delta[i];
    space.unvectorizeDouble(theta);
}

// ---------------------------------------------------------------------------
// CustomAction
// ---------------------------------------------------------------------------
bool CustomAction::feasible(const ParameterSpace& space) const {
    if (paramDelta_.empty()) return true;  // pure placeholder is always feasible
    return paramDelta_.size() == space.vectorizeDouble().size();
}

void CustomAction::apply(ParameterSpace& space,
                         const std::vector<double>& gradient,
                         const ParameterUpdateFn& updateSource) const {
    (void)gradient;
    (void)updateSource;
    // Placeholder: no real hardware / state transition is executed. If a
    // demonstration delta was provided, apply it to the parameter vector so the
    // data-flow (and serial stacking) is observable end-to-end.
    if (paramDelta_.empty()) return;
    auto theta = space.vectorizeDouble();
    const size_t n = std::min(theta.size(), paramDelta_.size());
    for (size_t i = 0; i < n; ++i) theta[i] += paramDelta_[i];
    space.unvectorizeDouble(theta);
}

json CustomAction::config() const {
    json c = json::object();
    if (!name_.empty()) c["name"] = name_;
    if (!params_.is_null()) c["params"] = params_;
    if (!state_.is_null()) c["state"] = state_;
    if (!transition_.is_null()) c["transition"] = transition_;
    if (!paramDelta_.empty()) c["paramDelta"] = paramDelta_;
    return c;
}

void CustomAction::setConfig(const json& j) {
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("params")) params_ = j["params"];
    if (j.contains("state")) state_ = j["state"];
    if (j.contains("transition")) transition_ = j["transition"];
    if (j.contains("paramDelta")) paramDelta_ = j["paramDelta"].get<std::vector<double>>();
}

// ---------------------------------------------------------------------------
// ActionSpace
// ---------------------------------------------------------------------------
ActionSpace::ActionSpace() {
    // Default action set: ONLY the parameter update.
    actions_.push_back(std::make_unique<ParameterUpdateAction>());
}

void ActionSpace::addAction(std::unique_ptr<Action> a) {
    if (a) actions_.push_back(std::move(a));
}

void ActionSpace::apply(ParameterSpace& space, const std::vector<double>& gradient) const {
    for (const auto& a : actions_)
        a->apply(space, gradient, updateSource_);
}

void ActionSpace::clearCustom() {
    if (actions_.empty()) {
        actions_.push_back(std::make_unique<ParameterUpdateAction>());
        return;
    }
    auto def = std::move(actions_[0]);
    actions_.clear();
    actions_.push_back(std::move(def));
}

json ActionSpace::toJson() const {
    json arr = json::array();
    for (const auto& a : actions_) {
        json aj = json::object();
        aj["type"] = a->type();
        aj["name"] = a->name();
        aj["config"] = a->config();
        arr.push_back(std::move(aj));
    }
    return json{{"version", 1}, {"actions", arr}};
}

void ActionSpace::fromJson(const json& j) {
    actions_.clear();
    if (!j.contains("actions") || !j["actions"].is_array()) {
        actions_.push_back(std::make_unique<ParameterUpdateAction>());
        return;
    }
    for (const auto& aj : j["actions"]) {
        auto a = createAction(aj);
        if (a) actions_.push_back(std::move(a));
    }
    // Guarantee the default action set is never empty.
    if (actions_.empty())
        actions_.push_back(std::make_unique<ParameterUpdateAction>());
}

// ---------------------------------------------------------------------------
// Factory
// ---------------------------------------------------------------------------
std::unique_ptr<Action> createAction(const json& serialized) {
    const std::string t = serialized.value("type", "parameter_update");
    std::unique_ptr<Action> a;
    if (t == "custom") {
        a = std::make_unique<CustomAction>();
    } else {
        // Unknown / default -> parameter update (safe fallback).
        a = std::make_unique<ParameterUpdateAction>();
    }
    if (serialized.contains("config")) a->setConfig(serialized["config"]);
    return a;
}

}  // namespace Space
}  // namespace AuroX
