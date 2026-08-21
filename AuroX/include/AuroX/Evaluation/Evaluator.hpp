#pragma once

// ============================================================================
// AuroX - Evaluator  (read-data / evaluation half)
// ----------------------------------------------------------------------------
// This is Part 1 of the "decouple reading data from optimizing" design: it is
// only responsible for reading data and computing the objective value and its
// gradient. It never touches parameter updates. It is decoupled from
// ParameterSpace (it only receives a std::vector<double> parameter vector), so
// the data-reading logic can be swapped freely -- implement a custom Evaluator
// to plug into the optimization loop without touching the optimizer or space.
//
// Design notes:
//   - EvaluatorResult { value, gradient }: the output of one evaluation;
//   - evaluate(params): full-batch evaluation (used by Gradient Descent);
//   - evaluateBatch(params, indices) + nextBatchIndices(batchSize) + beginEpoch():
//     mini-batch evaluation (used by SGD); the evaluator owns sampling/shuffling;
//   - custom interface: subclass Evaluator and implement evaluate() (plus the
//     optional batch interface).
//
// Reference implementation RegressionEvaluator: MVP multi-dimensional linear
// regression (minimizes MSE), supporting both GD (full batch) and SGD (mini-batch).
// ============================================================================

#include "json.hpp"

#include <vector>

// DLL export macro: empty when compiled as a static library; define
// AUROX_BUILD_DLL (export) or AUROX_USE_DLL (import) to enable when building
// AuroX.dll.
#ifndef AUROX_API
#  if defined(_WIN32) && defined(AUROX_BUILD_DLL)
#    define AUROX_API __declspec(dllexport)
#  elif defined(_WIN32) && defined(AUROX_USE_DLL)
#    define AUROX_API __declspec(dllimport)
#  else
#    define AUROX_API
#  endif
#endif

namespace AuroX {
namespace Evaluation {

using json = nlohmann::json;

// Output of one evaluation: objective value + gradient w.r.t. the parameter vector.
struct EvaluatorResult {
    double value = 0.0;              // objective / loss (smaller is better by default)
    std::vector<double> gradient;    // gradient w.r.t. the parameter vector
};

// Abstract evaluator base class (the "read data" half).
// Subclasses only care about: given a parameter vector, compute loss and gradient;
// sampling / shuffling details are owned by the subclass.
// It is decoupled from both the optimizer and the parameter space -- this is the
// core of "decoupling data reading from optimization".
class AUROX_API Evaluator {
public:
    virtual ~Evaluator() = default;

    // Full-batch evaluation: given the parameter vector, return loss and gradient.
    virtual EvaluatorResult evaluate(const std::vector<double>& params) const = 0;

    // Dimension of the parameter vector (used to align/validate against
    // ParameterSpace::vectorizeDouble()).
    virtual size_t dimension() const = 0;

    // Total number of samples N (used by SGD to compute epoch length; 0 if batching unsupported).
    virtual size_t sampleCount() const { return 0; }

    // Mini-batch size (0 when batching is unsupported, meaning full batch).
    virtual size_t batchSize() const { return 0; }

    // Whether mini-batch evaluation (SGD) is supported. Default false.
    virtual bool supportsBatching() const { return false; }

    // Hook called at the start of a new epoch (used by SGD to shuffle). Default empty.
    virtual void beginEpoch() {}

    // Return the sample indices of the next mini-batch; empty when batching unsupported.
    virtual std::vector<size_t> nextBatchIndices(size_t batchSize) {
        (void)batchSize;
        return {};
    }

    // Mini-batch evaluation: given parameters and sample indices, return the
    // batch's loss and gradient. Defaults to evaluate() (whole batch). Subclasses
    // should override to enable true SGD.
    virtual EvaluatorResult evaluateBatch(const std::vector<double>& params,
                                          const std::vector<size_t>& indices) const {
        (void)indices;
        return evaluate(params);
    }

    // Configuration snapshot (optional, for recording into history).
    virtual json config() const { return {}; }

    // Stable type tag for (de)serialization / factory dispatch.
    virtual std::string type() const { return "Evaluator"; }
};

// ----------------------------------------------------------------------------
// MVP reference implementation: multi-dimensional linear regression evaluator
// (minimizes mean squared error MSE)
//   loss(theta) = (1/N) sum_i (x_i . theta - y_i)^2
//   gradient    = (2/N) X^T (X theta - y)
// Supports full batch (GD) and mini-batch (SGD, with internal shuffle order).
// ----------------------------------------------------------------------------
class AUROX_API RegressionEvaluator : public Evaluator {
public:
    // X: row-major N x D matrix (N samples, D features; prepend a column of 1s
    //    to fit a bias term).
    // y: target values, length N.
    // stochastic: true => enable mini-batch (pair with an SGD optimizer).
    // miniBatchSize: mini-batch size (only used when stochastic=true).
    RegressionEvaluator(std::vector<std::vector<double>> X,
                        std::vector<double> y,
                        bool stochastic = false,
                        size_t miniBatchSize = 32,
                        unsigned int seed = 1u);

    EvaluatorResult evaluate(const std::vector<double>& params) const override;
    size_t dimension() const override { return D_; }
    size_t sampleCount() const override { return N_; }
    size_t batchSize() const override { return stochastic_ ? miniBatchSize_ : 0; }
    bool supportsBatching() const override { return stochastic_; }

    void beginEpoch() override;
    std::vector<size_t> nextBatchIndices(size_t batchSize) override;
    EvaluatorResult evaluateBatch(const std::vector<double>& params,
                                  const std::vector<size_t>& indices) const override;

    json config() const override;
    // Full serialization (includes the dataset X/y) for offline save / load and
    // back-end-free task reconstruction. For the lightweight live manifest, use
    // config() instead.
    std::string type() const override { return "RegressionEvaluator"; }
    json toJson() const;
    void fromJson(const json& j);

private:
    EvaluatorResult compute(const std::vector<double>& params,
                            const std::vector<size_t>& indices) const;

    std::vector<std::vector<double>> X_;
    std::vector<double> y_;
    size_t N_ = 0, D_ = 0;
    bool stochastic_ = false;
    size_t miniBatchSize_ = 32;
    unsigned int seed_ = 1;

    std::vector<size_t> order_;   // shuffle order for the current epoch
    size_t cursor_ = 0;           // position of the current mini-batch within order_
    size_t epochCounter_ = 0;     // number of epochs started (for reproducible shuffle)
};

// Factory: rebuild an evaluator from its serialized form (type + data + config).
// Enables the front end to reconstruct the evaluator (including its dataset)
// purely from the task manifest.
std::unique_ptr<Evaluator> createEvaluator(const json& j);

}  // namespace Evaluation
}  // namespace AuroX
