#include "AuroX/Dynamics/Dynamics.hpp"

#include <vector>

namespace AuroX {
namespace Dynamics {

Dynamics::Dynamics(Space::ObservationSpace* obs, Space::StateSpace* state,
                   std::unique_ptr<StateEstimator> estimator,
                   std::unique_ptr<TransitionModel> model,
                   std::unique_ptr<SystemIdentifier> identifier)
    : obs_(obs), state_(state),
      estimator_(std::move(estimator)), model_(std::move(model)),
      identifier_(std::move(identifier)), initialized_(false) {}

// One tick. `u` is the control vector for THIS step (from ActionSpace /
// Controller). Returns false if ObservationSpace / StateSpace are missing.
bool Dynamics::step(const std::vector<double>& u, double ts) {
    if (!obs_ || !state_) return false;
    auto bundle = obs_->collect();
    const std::vector<double>& y = bundle.vector;
    std::vector<double> x_pred = initialized_ ? model_->predict(x_prev_, u) : y;
    std::vector<double> xhat = estimator_->estimate(y, u, x_pred);
    state_->setState(xhat, ts);
    if (initialized_) identifier_->addSample(x_prev_, u, xhat);
    x_prev_ = xhat;
    initialized_ = true;
    return true;
}

// Overload with no control (control defaults to zeros of model's dim).
bool Dynamics::step(double ts) {
    size_t m = model_ ? model_->controlDim() : 0;
    return step(std::vector<double>(m, 0.0), ts);
}

// Lookahead: predicted next state for a candidate control (no execution).
std::vector<double> Dynamics::simulate(const std::vector<double>& u) const {
    if (state_ && state_->hasState()) return model_->predict(state_->getState(), u);
    return {};
}

// Multi-step rollout from an explicit initial state.
std::vector<std::vector<double>> Dynamics::rollout(
    const std::vector<double>& x0,
    const std::vector<std::vector<double>>& us) const {
    return model_->rollout(x0, us);
}

// Run identification; on success the fitted model replaces the active one.
bool Dynamics::identify() {
    if (!identifier_) return false;
    if (!identifier_->fit()) return false;
    if (auto* li = dynamic_cast<LinearIdentifier*>(identifier_.get())) {
        if (li->model()) {
            model_ = std::make_unique<LinearTransitionModel>(*li->model());
            return true;
        }
    }
    return false;
}

// Swap in a custom white-box model at runtime.
void Dynamics::setModel(std::unique_ptr<TransitionModel> m) { model_ = std::move(m); }
void Dynamics::useWhiteBox(size_t n, size_t m, WhiteBoxModel::Fn f) {
    model_ = std::make_unique<WhiteBoxModel>(n, m, std::move(f));
}

const StateEstimator*   Dynamics::estimator()   const { return estimator_.get(); }
const TransitionModel*  Dynamics::model()       const { return model_.get(); }
const SystemIdentifier* Dynamics::identifier()  const { return identifier_.get(); }
Space::StateSpace*      Dynamics::stateSpace()        { return state_; }

void Dynamics::reset() {
    initialized_ = false;
    x_prev_.clear();
    if (estimator_)  estimator_->reset();
    if (identifier_) identifier_->reset();
}

}  // namespace Dynamics
}  // namespace AuroX
