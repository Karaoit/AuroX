#include "AuroX/Optimizer/OptimizerSession.hpp"

#include <chrono>
#include <cmath>

namespace AuroX {
namespace Optimizer {

namespace {
    double gradientL2(const std::vector<double>& g) {
        double s = 0.0;
        for (double v : g) s += v * v;
        return std::sqrt(s);
    }
}  // namespace

OptimizerSession::OptimizerSession(Space::ParameterSpace& space,
                                   Evaluation::Evaluator& evaluator,
                                   Optimizer& optimizer,
                                   Memory::RunHistory& history,
                                   size_t maxIterations,
                                   double tolerance,
                                   double maxObjective)
    : space_(space),
      evaluator_(evaluator),
      optimizer_(optimizer),
      history_(history),
      maxIterations_(maxIterations),
      tolerance_(tolerance),
      maxObjective_(maxObjective) {}

OptimizerSession::Report OptimizerSession::run() {
    Report rep;

    // Dimension check: the evaluator dimension must match the parameter space's
    // vectorized dimension.
    const size_t dim = space_.vectorizeDouble().size();
    if (evaluator_.dimension() != dim) {
        rep.dimensionMismatch = true;
        return rep;
    }

    const bool stochastic = optimizer_.isStochastic();
    size_t epochLen = 0;
    if (stochastic) {
        const size_t N = evaluator_.sampleCount();
        const size_t b = evaluator_.batchSize();
        if (b > 0 && N > 0) epochLen = (N + b - 1) / b;   // ceil(N / b)
    }

    for (size_t it = 0; it < maxIterations_; ++it) {
        // At the start of each epoch: the evaluator shuffles and the optimizer
        // (e.g. SGD) decays its learning rate.
        if (stochastic && (it == 0 || (epochLen > 0 && it % epochLen == 0))) {
            evaluator_.beginEpoch();
            optimizer_.onEpochBegin();
        }

        auto t0 = std::chrono::high_resolution_clock::now();

        // Read-data half: take the current parameters from the space and ask the
        // evaluator for loss + gradient.
        std::vector<double> theta = space_.vectorizeDouble();

        Evaluation::EvaluatorResult res;
        if (stochastic) {
            std::vector<size_t> idx = evaluator_.nextBatchIndices(evaluator_.batchSize());
            if (idx.empty()) res = evaluator_.evaluate(theta);            // fall back to full batch
            else res = evaluator_.evaluateBatch(theta, idx);
        } else {
            res = evaluator_.evaluate(theta);
        }

        // Optimize half: update the parameter space using only the gradient.
        if (actionSpace_) {
            // Wire the optimizer's delta as the ActionSpace update source
            // ("basis from the optimizer"), then apply the full action list
            // (default parameter update + any serially-stacked custom actions).
            actionSpace_->setUpdateSource(
                [this](Space::ParameterSpace& s, const std::vector<double>& g) {
                    return optimizer_.computeUpdate(s, g);
                });
            actionSpace_->apply(space_, res.gradient);
        } else {
            optimizer_.update(space_, res.gradient);
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        const double elapsedMs =
            std::chrono::duration<double, std::milli>(t1 - t0).count();

        history_.record(it, res.value, &theta, gradientL2(res.gradient), 0, elapsedMs);

        // Early stopping: divergence (safety net for unbounded params) / convergence.
        if (history_.diverged(maxObjective_)) { rep.diverged = true; break; }
        if (history_.converged(tolerance_))   { rep.converged = true; break; }
    }

    rep.iterations = history_.size();
    if (history_.hasBest()) {
        rep.bestObjective = history_.bestObjective();
        rep.bestParams = history_.bestParams();
    }
    return rep;
}

}  // namespace Optimizer
}  // namespace AuroX
