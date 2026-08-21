#pragma once

// ============================================================================
// AuroX - Dynamics / Dynamics  (orchestrator)
// ----------------------------------------------------------------------------
// Wires the three Dynamics modules together and connects the ObservationSpace
// (input) to the StateSpace (output):
//
//    ObservationSpace --(y)--> StateEstimator --> StateSpace (holds x_hat)
//                                            |
//                                            v
//                                 TransitionModel (predict / rollout)
//                                            ^
//                                 SystemIdentifier (fits LinearTransitionModel)
//
// Dynamics does NOT own ObservationSpace / StateSpace; the caller wires them.
// It also exposes:
//   - step(u)        : one tick (pull obs, estimate, push state, feed identifier)
//   - simulate(u)    : predict next state for a candidate control (Optimizer/
//                      Policy lookahead, no execution)
//   - rollout(us)    : multi-step trajectory
//   - identify()     : run the identifier and adopt the fitted model
//   - useWhiteBox()  : swap in a user-supplied analytic model at runtime
//
// Declarations only; method definitions live in Dynamics.cpp.
// ============================================================================

#include "AuroX/Dynamics/StateEstimator.hpp"
#include "AuroX/Dynamics/SystemIdentifier.hpp"
#include "AuroX/Dynamics/TransitionModel.hpp"
#include "AuroX/Space/ObservationSpace.hpp"
#include "AuroX/Space/StateSpace.hpp"

#include <memory>
#include <vector>

namespace AuroX {
namespace Dynamics {

class Dynamics {
public:
    Dynamics(Space::ObservationSpace* obs, Space::StateSpace* state,
             std::unique_ptr<StateEstimator> estimator,
             std::unique_ptr<TransitionModel> model,
             std::unique_ptr<SystemIdentifier> identifier);

    // One tick. `u` is the control vector for THIS step (from ActionSpace /
    // Controller). Returns false if ObservationSpace / StateSpace are missing.
    bool step(const std::vector<double>& u, double ts = 0.0);

    // Overload with no control (control defaults to zeros of model's dim).
    bool step(double ts = 0.0);

    // Lookahead: predicted next state for a candidate control (no execution).
    std::vector<double> simulate(const std::vector<double>& u) const;

    // Multi-step rollout from an explicit initial state.
    std::vector<std::vector<double>> rollout(const std::vector<double>& x0,
                                             const std::vector<std::vector<double>>& us) const;

    // Run identification; on success the fitted model replaces the active one.
    bool identify();

    // Swap in a custom white-box model at runtime.
    void setModel(std::unique_ptr<TransitionModel> m);
    void useWhiteBox(size_t n, size_t m, WhiteBoxModel::Fn f);

    const StateEstimator*   estimator()   const;
    const TransitionModel*  model()       const;
    const SystemIdentifier* identifier()  const;
    Space::StateSpace*      stateSpace();

    void reset();

private:
    Space::ObservationSpace* obs_;
    Space::StateSpace* state_;
    std::unique_ptr<StateEstimator>   estimator_;
    std::unique_ptr<TransitionModel>  model_;
    std::unique_ptr<SystemIdentifier> identifier_;
    std::vector<double> x_prev_;
    bool initialized_;
};

}  // namespace Dynamics
}  // namespace AuroX
