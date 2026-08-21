#include "AuroX/Dynamics/LinAlg.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace AuroX {
namespace Dynamics {
namespace LinAlg {

// y = A x   (A: n x m, x: m) -> y: n
std::vector<double> matVec(const std::vector<double>& A,
                           const std::vector<double>& x,
                           size_t n, size_t m) {
    std::vector<double> y(n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        double s = 0.0;
        for (size_t j = 0; j < m; ++j) s += A[i * m + j] * x[j];
        y[i] = s;
    }
    return y;
}

// C = A B   (A: n x k, B: k x m) -> C: n x m
std::vector<double> matMul(const std::vector<double>& A,
                           const std::vector<double>& B,
                           size_t n, size_t k, size_t m) {
    std::vector<double> C(n * m, 0.0);
    for (size_t i = 0; i < n; ++i)
        for (size_t p = 0; p < k; ++p) {
            double a = A[i * k + p];
            if (a == 0.0) continue;
            for (size_t j = 0; j < m; ++j) C[i * m + j] += a * B[p * m + j];
        }
    return C;
}

// T = A^T   (A: n x m) -> T: m x n
std::vector<double> matTranspose(const std::vector<double>& A,
                                 size_t n, size_t m) {
    std::vector<double> T(m * n, 0.0);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < m; ++j) T[j * n + i] = A[i * m + j];
    return T;
}

std::vector<double> matAdd(const std::vector<double>& A,
                           const std::vector<double>& B, size_t p) {
    std::vector<double> C(p, 0.0);
    for (size_t i = 0; i < p; ++i) C[i] = A[i] + B[i];
    return C;
}

std::vector<double> matSub(const std::vector<double>& A,
                           const std::vector<double>& B, size_t p) {
    std::vector<double> C(p, 0.0);
    for (size_t i = 0; i < p; ++i) C[i] = A[i] - B[i];
    return C;
}

std::vector<double> eye(size_t n) {
    std::vector<double> I(n * n, 0.0);
    for (size_t i = 0; i < n; ++i) I[i * n + i] = 1.0;
    return I;
}

// Invert square matrix A (n x n) via Gauss-Jordan with partial pivoting.
// Returns false if A is (near) singular.
bool matInv(const std::vector<double>& A, std::vector<double>& Ainv, size_t n) {
    Ainv.assign(n * n, 0.0);
    std::vector<double> aug(n * 2 * n, 0.0);
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) aug[i * 2 * n + j] = A[i * n + j];
        aug[i * 2 * n + n + i] = 1.0;
    }
    const double eps = 1e-12;
    for (size_t col = 0; col < n; ++col) {
        size_t piv = col;
        double best = std::fabs(aug[col * 2 * n + col]);
        for (size_t r = col + 1; r < n; ++r) {
            double v = std::fabs(aug[r * 2 * n + col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < eps) return false;
        if (piv != col)
            for (size_t j = 0; j < 2 * n; ++j)
                std::swap(aug[col * 2 * n + j], aug[piv * 2 * n + j]);
        double d = aug[col * 2 * n + col];
        for (size_t j = 0; j < 2 * n; ++j) aug[col * 2 * n + j] /= d;
        for (size_t r = 0; r < n; ++r) {
            if (r == col) continue;
            double f = aug[r * 2 * n + col];
            if (f == 0.0) continue;
            for (size_t j = 0; j < 2 * n; ++j)
                aug[r * 2 * n + j] -= f * aug[col * 2 * n + j];
        }
    }
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j) Ainv[i * n + j] = aug[i * 2 * n + n + j];
    return true;
}

// Solve M X = B, M is p x p (row-major), B is p x nrhs (row-major).
// Solution is returned in-place in B. Returns false if M is (near) singular.
bool solveLinearSystem(const std::vector<double>& M,
                       std::vector<double>& B,
                       size_t p, size_t nrhs) {
    const size_t w = p + nrhs;
    std::vector<double> aug(p * w, 0.0);
    for (size_t i = 0; i < p; ++i) {
        for (size_t j = 0; j < p; ++j) aug[i * w + j] = M[i * p + j];
        for (size_t j = 0; j < nrhs; ++j) aug[i * w + p + j] = B[i * nrhs + j];
    }
    const double eps = 1e-12;
    for (size_t col = 0; col < p; ++col) {
        size_t piv = col;
        double best = std::fabs(aug[col * w + col]);
        for (size_t r = col + 1; r < p; ++r) {
            double v = std::fabs(aug[r * w + col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < eps) return false;
        if (piv != col)
            for (size_t j = 0; j < w; ++j) std::swap(aug[col * w + j], aug[piv * w + j]);
        double d = aug[col * w + col];
        for (size_t j = 0; j < w; ++j) aug[col * w + j] /= d;
        for (size_t r = 0; r < p; ++r) {
            if (r == col) continue;
            double f = aug[r * w + col];
            if (f == 0.0) continue;
            for (size_t j = 0; j < w; ++j) aug[r * w + j] -= f * aug[col * w + j];
        }
    }
    for (size_t i = 0; i < p; ++i)
        for (size_t j = 0; j < nrhs; ++j) B[i * nrhs + j] = aug[i * w + p + j];
    return true;
}

}  // namespace LinAlg
}  // namespace Dynamics
}  // namespace AuroX
