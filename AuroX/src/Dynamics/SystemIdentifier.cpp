#include "AuroX/Dynamics/SystemIdentifier.hpp"
#include "AuroX/Dynamics/TransitionModel.hpp"
#include "AuroX/Dynamics/LinAlg.hpp"

#include <vector>

namespace AuroX {
namespace Dynamics {

LinearIdentifier::LinearIdentifier(size_t n, size_t m, size_t maxSamples)
    : n_(n), m_(m), maxSamples_(maxSamples), fitted_(false) {
    model_ = std::make_unique<LinearTransitionModel>(n_, m_);
}

void LinearIdentifier::reset() {
    xs_.clear(); us_.clear(); xns_.clear(); fitted_ = false;
}

void LinearIdentifier::setDims(size_t n, size_t m) {
    n_ = n; m_ = m;
    model_ = std::make_unique<LinearTransitionModel>(n_, m_);
    reset();
}

void LinearIdentifier::addSample(const std::vector<double>& x, const std::vector<double>& u,
                                const std::vector<double>& x_next) {
    if (x.size() != n_ || x_next.size() != n_ || u.size() != m_) return;
    xs_.push_back(x); us_.push_back(u); xns_.push_back(x_next);
    if (xs_.size() > maxSamples_) {
        xs_.erase(xs_.begin());
        us_.erase(us_.begin());
        xns_.erase(xns_.begin());
    }
}

// Solve  min_{A,B} sum || A x_t + B u_t - x_{t+1} ||^2.
// Build regression matrix Phi=[x;u] (k x p, p=n+m) and targets Y (k x n);
// normal equations M=Phi^T Phi (p x p), b=Phi^T Y (p x n); solve M Theta=b.
bool LinearIdentifier::fit() {
    const size_t k = xs_.size();
    if (k < n_ + m_ + 1) return false;     // need more than p samples
    const size_t p = n_ + m_;
    std::vector<double> Phi(k * p, 0.0), Y(k * n_, 0.0);
    for (size_t r = 0; r < k; ++r) {
        for (size_t j = 0; j < n_; ++j) Phi[r * p + j] = xs_[r][j];
        for (size_t j = 0; j < m_; ++j) Phi[r * p + n_ + j] = us_[r][j];
        for (size_t j = 0; j < n_; ++j) Y[r * n_ + j] = xns_[r][j];
    }
    std::vector<double> Phit = LinAlg::matTranspose(Phi, k, p);
    std::vector<double> M = LinAlg::matMul(Phit, Phi, p, k, p);     // p x p
    std::vector<double> b = LinAlg::matMul(Phit, Y, p, k, n_);      // p x n
    if (!LinAlg::solveLinearSystem(M, b, p, n_)) return false;      // Theta in b
    model_->resize(n_, m_);
    auto& A = model_->A(); auto& B = model_->B();
    // A[i][j] = coefficient of state j in x'_i = Theta[regressor j][target i]
    //         = b[j*n_ + i]   (b is row-major p x n_, row=regressor, col=target)
    for (size_t i = 0; i < n_; ++i)
        for (size_t j = 0; j < n_; ++j) A[i * n_ + j] = b[j * n_ + i];
    // B[i][j] = coefficient of control j in x'_i = Theta[regressor (n+j)][target i]
    //         = b[(n_+j)*n_ + i]
    for (size_t i = 0; i < n_; ++i)
        for (size_t j = 0; j < m_; ++j)
            B[i * m_ + j] = b[(n_ + j) * n_ + i];
    fitted_ = true;
    return true;
}

bool LinearIdentifier::hasModel() const { return fitted_; }

}  // namespace Dynamics
}  // namespace AuroX
