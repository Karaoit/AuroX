#pragma once

// ============================================================================
// AuroX - Dynamics / StateEstimator  (the "Observer")
// ----------------------------------------------------------------------------
// Maps ObservationSpace output y -> state estimate x_hat.
// estimate() is called every step with:
//     y      : current observation vector (from ObservationSpace::collect())
//     u      : current control vector
//     x_pred : model prior prediction A x_prev + B u (equals y on the 1st step)
//
// Declarations only; method definitions live in StateEstimator.cpp.
// ============================================================================

#include <vector>

namespace AuroX {
namespace Dynamics {

class StateEstimator {
public:
    virtual ~StateEstimator() = default;
    virtual void reset() = 0;
    virtual size_t stateDim() const = 0;
    virtual std::vector<double> estimate(const std::vector<double>& y,
                                         const std::vector<double>& u,
                                         const std::vector<double>& x_pred) = 0;
};

// ----------------------------------------------------------------------------
// PassThroughEstimator : x_hat = y. Minimal default; requires obsDim == stateDim.
// ----------------------------------------------------------------------------
class PassThroughEstimator : public StateEstimator {
public:
    PassThroughEstimator(size_t dim = 0);
    void reset() override;
    size_t stateDim() const override;
    void setDim(size_t d);
    std::vector<double> estimate(const std::vector<double>& y, const std::vector<double>&,
                                 const std::vector<double>&) override;
private:
    size_t dim_;
};

// ----------------------------------------------------------------------------
// LowPassEstimator : exponential smoothing, x = (1-a) x_prev + a y.
//   Requires obsDim == stateDim. `a` in (0,1]; smaller = heavier smoothing.
//   A trivial, robust denoiser when you don't want to tune a Kalman filter.
// ----------------------------------------------------------------------------
class LowPassEstimator : public StateEstimator {
public:
    LowPassEstimator(size_t dim = 0, double a = 0.3);
    void reset() override;
    size_t stateDim() const override;
    void setDim(size_t d);
    void setAlpha(double a);
    std::vector<double> estimate(const std::vector<double>& y, const std::vector<double>&,
                                 const std::vector<double>&) override;
private:
    size_t dim_;
    double a_;
    std::vector<double> x_;
    bool init_;
};

// ----------------------------------------------------------------------------
// KalmanEstimator : linear observer (optimal linear filter).
//   predict : x_pred = A x + B u
//   update  : x = x_pred + K (y - C x_pred)
//   A, B come from the transition model; C maps state -> observation.
//   Q (process cov), R (measurement cov) are tunable; S = C P C^T + R.
// ----------------------------------------------------------------------------
class KalmanEstimator : public StateEstimator {
public:
    KalmanEstimator();

    void configure(size_t stateDim, size_t obsDim, size_t controlDim,
                   const std::vector<double>& A, const std::vector<double>& B,
                   const std::vector<double>& C, const std::vector<double>& Q,
                   const std::vector<double>& R);

    void reset() override;
    size_t stateDim() const override;

    std::vector<double> estimate(const std::vector<double>& y, const std::vector<double>& u,
                                 const std::vector<double>&) override;

private:
    size_t n_, o_, m_;
    std::vector<double> A_, B_, C_, Q_, R_;
    std::vector<double> x_;
    std::vector<double> P_;
    bool init_;
};

}  // namespace Dynamics
}  // namespace AuroX
