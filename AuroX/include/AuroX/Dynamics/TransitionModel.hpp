#pragma once

// ============================================================================
// AuroX - Dynamics / TransitionModel
// ----------------------------------------------------------------------------
// The "state predictor": x_{t+1} = f(x_t, u_t).
//   - predict() is the only required method.
//   - jacobian() defaults to a numeric central difference (handy for finite-
//     difference gradients inside the Optimizer / Policy); override for speed.
//   - rollout() chains predict() for multi-step (MPC) lookahead.
//
// Two concrete models ship by default:
//   * LinearTransitionModel : x' = A x + B u  (the model the identifier fits)
//   * WhiteBoxModel         : USER-SUPPLIED analytic / physics f(x,u) -> x'
//
// Declarations only; method definitions live in TransitionModel.cpp.
// ============================================================================

#include <cstddef>
#include <functional>
#include <vector>

namespace AuroX {
namespace Dynamics {

class TransitionModel {
public:
    virtual ~TransitionModel() = default;

    virtual size_t stateDim()  const = 0;
    virtual size_t controlDim() const = 0;

    virtual std::vector<double> predict(const std::vector<double>& x,
                                        const std::vector<double>& u) const = 0;

    // Numeric central-difference Jacobians [A = df/dx (n x n), B = df/du (n x m)].
    virtual void jacobian(const std::vector<double>& x, const std::vector<double>& u,
                          std::vector<double>& A, std::vector<double>& B) const;

    // Multi-step rollout: traj[0] = x0, traj[k+1] = predict(traj[k], us[k]).
    std::vector<std::vector<double>> rollout(const std::vector<double>& x0,
                                             const std::vector<std::vector<double>>& us) const;
};

// ----------------------------------------------------------------------------
// LinearTransitionModel : DEFAULT identified model, x' = A x + B u.
//   Matrices are row-major. The SystemIdentifier writes directly into A()/B().
// ----------------------------------------------------------------------------
class LinearTransitionModel : public TransitionModel {
public:
    LinearTransitionModel(size_t n = 0, size_t m = 0);

    size_t stateDim()  const override;
    size_t controlDim() const override;

    void resize(size_t n, size_t m);

    std::vector<double> predict(const std::vector<double>& x,
                                const std::vector<double>& u) const override;

    std::vector<double>& A() { return A_; }
    std::vector<double>& B() { return B_; }
    const std::vector<double>& A() const { return A_; }
    const std::vector<double>& B() const { return B_; }

    bool valid() const;

private:
    size_t n_, m_;
    std::vector<double> A_;  // n*n row-major
    std::vector<double> B_;  // n*m row-major
};

// ----------------------------------------------------------------------------
// WhiteBoxModel : USER-SUPPLIED physics / analytic dynamics.
//   Supply a callable f(x, u) -> x'. It satisfies the TransitionModel
//   interface, so it can be swapped in at any time via Dynamics::useWhiteBox().
// ----------------------------------------------------------------------------
class WhiteBoxModel : public TransitionModel {
public:
    using Fn = std::function<std::vector<double>(const std::vector<double>&,
                                                const std::vector<double>&)>;

    WhiteBoxModel(size_t n, size_t m, Fn f);
    WhiteBoxModel(Fn f, size_t n, size_t m);

    size_t stateDim()  const override;
    size_t controlDim() const override;

    std::vector<double> predict(const std::vector<double>& x,
                                const std::vector<double>& u) const override;

private:
    size_t n_, m_;
    Fn f_;
};

}  // namespace Dynamics
}  // namespace AuroX
