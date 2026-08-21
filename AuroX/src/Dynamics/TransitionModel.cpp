#include "AuroX/Dynamics/TransitionModel.hpp"

#include <vector>

namespace AuroX {
namespace Dynamics {

// ----------------------------------------------------------------------------
// TransitionModel
// ----------------------------------------------------------------------------
void TransitionModel::jacobian(const std::vector<double>& x, const std::vector<double>& u,
                               std::vector<double>& A, std::vector<double>& B) const {
    const double h = 1e-6;
    const size_t n = stateDim(), m = controlDim();
    A.assign(n * n, 0.0);
    B.assign(n * m, 0.0);
    std::vector<double> xp = predict(x, u);
    for (size_t j = 0; j < n; ++j) {
        std::vector<double> xh = x; xh[j] += h;
        std::vector<double> fp = predict(xh, u);
        for (size_t i = 0; i < n; ++i) A[i * n + j] = (fp[i] - xp[i]) / h;
    }
    for (size_t j = 0; j < m; ++j) {
        std::vector<double> uh = u; uh[j] += h;
        std::vector<double> fp = predict(x, uh);
        for (size_t i = 0; i < n; ++i) B[i * m + j] = (fp[i] - xp[i]) / h;
    }
}

std::vector<std::vector<double>> TransitionModel::rollout(
    const std::vector<double>& x0,
    const std::vector<std::vector<double>>& us) const {
    std::vector<std::vector<double>> traj;
    traj.push_back(x0);
    std::vector<double> x = x0;
    for (const auto& u : us) { x = predict(x, u); traj.push_back(x); }
    return traj;
}

// ----------------------------------------------------------------------------
// LinearTransitionModel
// ----------------------------------------------------------------------------
LinearTransitionModel::LinearTransitionModel(size_t n, size_t m)
    : n_(n), m_(m), A_(n * n, 0.0), B_(n * m, 0.0) {}

size_t LinearTransitionModel::stateDim()  const { return n_; }
size_t LinearTransitionModel::controlDim() const { return m_; }

void LinearTransitionModel::resize(size_t n, size_t m) {
    n_ = n; m_ = m;
    A_.assign(n * n, 0.0);
    B_.assign(n * m, 0.0);
}

std::vector<double> LinearTransitionModel::predict(const std::vector<double>& x,
                                                  const std::vector<double>& u) const {
    std::vector<double> xp(n_, 0.0);
    for (size_t i = 0; i < n_; ++i) {
        double s = 0.0;
        for (size_t j = 0; j < n_; ++j) s += A_[i * n_ + j] * x[j];
        for (size_t j = 0; j < m_; ++j) s += B_[i * m_ + j] * u[j];
        xp[i] = s;
    }
    return xp;
}

bool LinearTransitionModel::valid() const { return n_ > 0; }

// ----------------------------------------------------------------------------
// WhiteBoxModel
// ----------------------------------------------------------------------------
WhiteBoxModel::WhiteBoxModel(size_t n, size_t m, Fn f) : n_(n), m_(m), f_(std::move(f)) {}
WhiteBoxModel::WhiteBoxModel(Fn f, size_t n, size_t m) : n_(n), m_(m), f_(std::move(f)) {}

size_t WhiteBoxModel::stateDim()  const { return n_; }
size_t WhiteBoxModel::controlDim() const { return m_; }

std::vector<double> WhiteBoxModel::predict(const std::vector<double>& x,
                                           const std::vector<double>& u) const {
    return f_ ? f_(x, u) : x;
}

}  // namespace Dynamics
}  // namespace AuroX
