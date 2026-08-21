#pragma once

// ============================================================================
// AuroX - Optimizer  (optimization half)
// ----------------------------------------------------------------------------
// This is Part 2 of the "decouple reading data from optimizing" design: it is
// only responsible for updating parameters from the gradient, and reads no data
// at all. It is decoupled from Evaluator (it only receives the gradient vector)
// and cooperates with ParameterSpace (via vectorizeDouble / unvectorizeDouble
// to read and write parameters).
//
// Provides:
//   - Optimizer abstract base class: the custom interface; subclass and implement
//     update() to plug into the loop;
//   - GradientDescent: the default optimizer (full batch, theta <- theta - lr*grad);
//   - SGD: optional stochastic gradient descent (momentum + per-epoch learning-rate
//          decay); must be paired with an evaluator that supportsBatching()
//          (e.g. RegressionEvaluator(stochastic=true)).
//
// Default choice: GD is the most stable MVP starting point; switch to SGD for
// large datasets / online scenarios.
//
// Declarations only; method definitions live in Optimizer.cpp.
// ============================================================================

#include "json.hpp"
#include "AuroX/Space/ParameterSpace.hpp"

#include <string>
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
namespace Optimizer {

using json = nlohmann::json;
using Space::ParameterSpace;

// Optimizer abstract base class (the "optimization" half). A custom optimizer only
// needs to subclass and implement update().
class AUROX_API Optimizer {
public:
    virtual ~Optimizer() = default;

    // Compute (but do NOT apply) the parameter DELTA from the gradient. This is
    // the "judgment basis" the ActionSpace default action consumes. Subclasses
    // implement this; update() is provided by the base as compute + write-back.
    virtual std::vector<double> computeUpdate(ParameterSpace& space,
                                              const std::vector<double>& gradient) = 0;

    // Apply the update: compute the delta and write it back to the space.
    // Kept as a concrete method so existing callers (and the session) are
    // unaffected by the computeUpdate refactor.
    void update(ParameterSpace& space, const std::vector<double>& gradient);

    // Whether this is a stochastic optimizer (decides whether Session uses the
    // mini-batch evaluation path). Default false.
    virtual bool isStochastic() const;

    // Name (for recording / logging) and a stable type tag for serialization.
    virtual const char* name() const = 0;
    virtual std::string type() const;

    // Configuration snapshot (optional, recorded into history / manifest).
    virtual json config() const;

    // Hook called at the start of each epoch (e.g. learning-rate decay). Default empty.
    virtual void onEpochBegin();
};

// Gradient descent (default): full batch, delta = -lr * grad.
class AUROX_API GradientDescent : public Optimizer {
public:
    explicit GradientDescent(double learningRate = 0.01);

    std::vector<double> computeUpdate(ParameterSpace& space,
                                      const std::vector<double>& gradient) override;

    bool isStochastic() const override;
    const char* name() const override;
    json config() const override;

private:
    double lr_;
};

// Stochastic gradient descent (optional): momentum + per-epoch learning-rate decay.
// Must be paired with an evaluator that supportsBatching()
// (e.g. RegressionEvaluator(stochastic=true)).
// Update: v <- momentum*v - lr*grad; theta <- theta + v.
class AUROX_API SGD : public Optimizer {
public:
    SGD(double learningRate = 0.01,
        double momentum = 0.9,
        double decay = 0.0);          // learning rate is multiplied by (1 - decay) each epoch

    std::vector<double> computeUpdate(ParameterSpace& space,
                                      const std::vector<double>& gradient) override;

    bool isStochastic() const override;
    const char* name() const override;
    json config() const override;

    void onEpochBegin() override;

private:
    double lr_;
    double momentum_;
    double decay_;
    mutable std::vector<double> velocity_;   // mutable: updated inside const computeUpdate
};

// Factory: rebuild an optimizer from its (type, config) JSON. Enables the front
// end to reconstruct the optimizer purely from the task manifest.
std::unique_ptr<Optimizer> createOptimizer(const std::string& type,
                                           const json& config = json::object());

}  // namespace Optimizer
}  // namespace AuroX
