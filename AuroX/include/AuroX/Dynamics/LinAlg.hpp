#pragma once

// ============================================================================
// AuroX - Dynamics / LinAlg  (linear-algebra helpers)
// ----------------------------------------------------------------------------
// Tiny row-major matrix helpers used by the Dynamics layer (Kalman filter,
// least-squares identification). They are dependency-free and defined in the
// matching LinAlg.cpp translation unit; this header only declares them.
//
// All matrices are std::vector<double>, row-major, with explicit dimensions
// passed alongside the data. Kept in their own AuroX::Dynamics::LinAlg
// namespace (mirroring the file name) so the internal math helpers are clearly
// separated from the public model classes.
// ============================================================================

#include <vector>

namespace AuroX {
namespace Dynamics {
namespace LinAlg {

// y = A x   (A: n x m, x: m) -> y: n
std::vector<double> matVec(const std::vector<double>& A,
                           const std::vector<double>& x,
                           size_t n, size_t m);

// C = A B   (A: n x k, B: k x m) -> C: n x m
std::vector<double> matMul(const std::vector<double>& A,
                           const std::vector<double>& B,
                           size_t n, size_t k, size_t m);

// T = A^T   (A: n x m) -> T: m x n
std::vector<double> matTranspose(const std::vector<double>& A,
                                 size_t n, size_t m);

std::vector<double> matAdd(const std::vector<double>& A,
                            const std::vector<double>& B, size_t p);

std::vector<double> matSub(const std::vector<double>& A,
                            const std::vector<double>& B, size_t p);

std::vector<double> eye(size_t n);

// Invert square matrix A (n x n) via Gauss-Jordan with partial pivoting.
// Returns false if A is (near) singular.
bool matInv(const std::vector<double>& A, std::vector<double>& Ainv, size_t n);

// Solve M X = B, M is p x p (row-major), B is p x nrhs (row-major).
// Solution is returned in-place in B. Returns false if M is (near) singular.
bool solveLinearSystem(const std::vector<double>& M,
                       std::vector<double>& B,
                       size_t p, size_t nrhs);

}  // namespace LinAlg
}  // namespace Dynamics
}  // namespace AuroX
