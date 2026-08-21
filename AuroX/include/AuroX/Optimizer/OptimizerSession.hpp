#pragma once

// ============================================================================
// AuroX - OptimizerSession  (optimization loop orchestrator, part of Optimizer layer)
// ----------------------------------------------------------------------------
// Wires the "parameter space / evaluator / optimizer / history" into a training loop.
//
// Key responsibilities (scheduling only, no algorithm details):
//   - each round: take the parameter vector from ParameterSpace -> call Evaluator
//       to evaluate (read data) -> call Optimizer.update to update params
//       (optimize) -> write RunHistory;
//   - based on optimizer.isStochastic(), automatically choose the "full-batch
//       evaluation" or "mini-batch evaluation" path;
//   - use RunHistory's converged() / diverged() for early stopping (the safety
//       net for unbounded parameters).
//
// Thus "reading data" (Evaluator) and "optimizing" (Optimizer) cooperate here
// but remain decoupled from each other.
// ============================================================================

#include "AuroX/Space/ParameterSpace.hpp"
#include "AuroX/Space/ActionSpace.hpp"
#include "AuroX/Memory/RunHistory.hpp"
#include "AuroX/Evaluation/Evaluator.hpp"
#include "AuroX/Optimizer/Optimizer.hpp"

#include <vector>

namespace AuroX {
namespace Optimizer {

class OptimizerSession {
public:
    OptimizerSession(Space::ParameterSpace& space,
                     Evaluation::Evaluator& evaluator,
                     Optimizer& optimizer,
                     Memory::RunHistory& history,
                     size_t maxIterations = 1000,
                     double tolerance = 1e-6,
                     double maxObjective = 1e12);

    struct Report {
        size_t iterations = 0;
        bool converged = false;
        bool diverged = false;
        bool dimensionMismatch = false;   // evaluator dimension != parameter space dimension
        double bestObjective = 0.0;
        std::vector<double> bestParams;
    };

    // Optionally insert an ActionSpace between the optimizer's output and the
    // parameter update. The default (nullptr) keeps the original behavior: the
    // optimizer updates the parameters directly.
    void setActionSpace(Space::ActionSpace* actionSpace) { actionSpace_ = actionSpace; }
    Space::ActionSpace* actionSpace() const { return actionSpace_; }

    // Run the main loop and return the report.
    Report run();

private:
    Space::ParameterSpace& space_;
    Evaluation::Evaluator& evaluator_;
    Optimizer& optimizer_;
    Memory::RunHistory& history_;
    Space::ActionSpace* actionSpace_ = nullptr;
    size_t maxIterations_;
    double tolerance_;
    double maxObjective_;
};

}  // namespace Optimizer
}  // namespace AuroX
