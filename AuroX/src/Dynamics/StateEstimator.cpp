#include "AuroX/Dynamics/StateEstimator.hpp"
#include "AuroX/Dynamics/LinAlg.hpp"

#include <vector>

namespace AuroX {
namespace Dynamics {

// ----------------------------------------------------------------------------
// PassThroughEstimator
// ----------------------------------------------------------------------------
PassThroughEstimator::PassThroughEstimator(size_t dim) : dim_(dim) {}

void PassThroughEstimator::reset() {}
size_t PassThroughEstimator::stateDim() const { return dim_; }
void PassThroughEstimator::setDim(size_t d) { dim_ = d; }

std::vector<double> PassThroughEstimator::estimate(const std::vector<double>& y,
                                                   const std::vector<double>&,
                                                   const std::vector<double>&) {
    return y;
}

// ----------------------------------------------------------------------------
// LowPassEstimator
// ----------------------------------------------------------------------------
LowPassEstimator::LowPassEstimator(size_t dim, double a) : dim_(dim), a_(a), init_(false) {}

void LowPassEstimator::reset() { init_ = false; x_.clear(); }
size_t LowPassEstimator::stateDim() const { return dim_; }
void LowPassEstimator::setDim(size_t d) { dim_ = d; init_ = false; }
void LowPassEstimator::setAlpha(double a) { a_ = a; }

std::vector<double> LowPassEstimator::estimate(const std::vector<double>& y,
                                               const std::vector<double>&,
                                               const std::vector<double>&) {
    if (!init_ || x_.size() != y.size()) { x_ = y; init_ = true; return x_; }
    for (size_t i = 0; i < y.size(); ++i) x_[i] = (1.0 - a_) * x_[i] + a_ * y[i];
    return x_;
}

// ----------------------------------------------------------------------------
// KalmanEstimator
// ----------------------------------------------------------------------------
KalmanEstimator::KalmanEstimator() : n_(0), o_(0), m_(0), init_(false) {}

void KalmanEstimator::configure(size_t stateDim, size_t obsDim, size_t controlDim,
                                const std::vector<double>& A, const std::vector<double>& B,
                                const std::vector<double>& C, const std::vector<double>& Q,
                                const std::vector<double>& R) {
    n_ = stateDim; o_ = obsDim; m_ = controlDim;
    A_ = A; B_ = B; C_ = C; Q_ = Q; R_ = R;
    init_ = false; x_.clear(); P_.clear();
}

void KalmanEstimator::reset() { init_ = false; x_.clear(); P_.clear(); }
size_t KalmanEstimator::stateDim() const { return n_; }

std::vector<double> KalmanEstimator::estimate(const std::vector<double>& y,
                                              const std::vector<double>& u,
                                              const std::vector<double>&) {
    if (n_ == 0) return y;
    if (!init_) {
        x_.assign(n_, 0.0);
        for (size_t i = 0; i < n_; ++i) x_[i] = (i < y.size()) ? y[i] : 0.0;
        P_ = LinAlg::eye(n_);
        init_ = true;
        return x_;
    }
    // predict
    std::vector<double> xpred = LinAlg::matVec(A_, x_, n_, n_);
    if (m_ > 0) {
        std::vector<double> Bu = LinAlg::matVec(B_, u, n_, m_);
        for (size_t i = 0; i < n_; ++i) xpred[i] += Bu[i];
    }
    // P = A P A^T + Q
    std::vector<double> Ppred = LinAlg::matMul(LinAlg::matMul(A_, P_, n_, n_, n_),
                                               LinAlg::matTranspose(A_, n_, n_), n_, n_, n_);
    Ppred = LinAlg::matAdd(Ppred, Q_, n_ * n_);
    // S = C Ppred C^T + R  (o x o)
    std::vector<double> S = LinAlg::matMul(LinAlg::matMul(C_, Ppred, o_, n_, n_),
                                           LinAlg::matTranspose(C_, o_, n_), o_, n_, o_);
    S = LinAlg::matAdd(S, R_, o_ * o_);
    // K = Ppred C^T S^{-1}
    std::vector<double> CT = LinAlg::matTranspose(C_, o_, n_);
    std::vector<double> Sinv;
    if (!LinAlg::matInv(S, Sinv, o_)) return xpred;   // S singular: keep prior
    std::vector<double> K = LinAlg::matMul(LinAlg::matMul(Ppred, CT, n_, n_, o_),
                                           Sinv, n_, o_, o_);
    // x = xpred + K (y - C xpred)
    std::vector<double> yhat = LinAlg::matVec(C_, xpred, o_, n_);
    std::vector<double> innov = LinAlg::matSub(y, yhat, o_);
    std::vector<double> Kinn = LinAlg::matVec(K, innov, n_, o_);
    for (size_t i = 0; i < n_; ++i) x_[i] = xpred[i] + Kinn[i];
    // P = (I - K C) Ppred
    std::vector<double> KC = LinAlg::matMul(K, C_, n_, o_, n_);
    std::vector<double> I = LinAlg::eye(n_);
    std::vector<double> IKC = LinAlg::matSub(I, KC, n_ * n_);
    P_ = LinAlg::matMul(IKC, Ppred, n_, n_, n_);
    return x_;
}

}  // namespace Dynamics
}  // namespace AuroX
