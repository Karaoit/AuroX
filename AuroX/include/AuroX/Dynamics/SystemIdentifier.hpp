#pragma once

// ============================================================================
// AuroX - Dynamics / SystemIdentifier
// ----------------------------------------------------------------------------
// Corrects / fits the transition model f from closed-loop history.
//   Default LinearIdentifier: accumulates (x, u, x') samples and solves the
//   linear least-squares problem  [x; u] * [A; B]^T = x'  via normal equations
//   (Gaussian elimination, defined in SystemIdentifier.cpp).
//
// Online vs offline: addSample() buffers every step (in the hot loop, cheap);
// fit() runs the (heavier) solve and is meant to be called periodically /
// off the hot path. The buffer is capped to maxSamples (ring) to bound memory.
//
// Declarations only; method definitions live in SystemIdentifier.cpp.
// ============================================================================

#include <memory>
#include <vector>

#include "AuroX/Dynamics/TransitionModel.hpp"

namespace AuroX {
namespace Dynamics {

class SystemIdentifier {
public:
    virtual ~SystemIdentifier() = default;
    virtual void reset() = 0;
    virtual void addSample(const std::vector<double>& x, const std::vector<double>& u,
                           const std::vector<double>& x_next) = 0;
    virtual bool fit() = 0;             // produce a model; true on success
    virtual bool hasModel() const = 0;
};

class LinearIdentifier : public SystemIdentifier {
public:
    explicit LinearIdentifier(size_t n = 0, size_t m = 0, size_t maxSamples = 4096);

    void reset() override;
    void setDims(size_t n, size_t m);

    void addSample(const std::vector<double>& x, const std::vector<double>& u,
                   const std::vector<double>& x_next) override;

    // Solve  min_{A,B} sum || A x_t + B u_t - x_{t+1} ||^2 via normal equations.
    bool fit() override;

    bool hasModel() const override;

    LinearTransitionModel* model() { return model_.get(); }
    const LinearTransitionModel* model() const { return model_.get(); }

private:
    size_t n_, m_, maxSamples_;
    bool fitted_;
    std::unique_ptr<LinearTransitionModel> model_;
    std::vector<std::vector<double>> xs_, us_, xns_;
};

}  // namespace Dynamics
}  // namespace AuroX
