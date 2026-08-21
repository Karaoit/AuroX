# Core layer: Optimizer (optimize, Layer 4)

## What this layer is

The `Optimizer` layer corresponds to **Layer 4: Optimizer / Runtime** in
`AuroX-General.md`. Its responsibility is to **update parameters from the
gradient**, and as the orchestrator of the closed loop it chains "parameter
space / evaluator / optimizer / history" together.

### Design core: reading data and optimizing are decoupled into two parts

| Part | Layer | Responsibility | Depends on |
|---|---|---|---|
| **Read data / evaluate** | `Evaluation` (`Evaluator`) | read data, compute objective + gradient | only receives the `std::vector<double>` parameter vector; no parameter update |
| **Optimize / update** | `Optimizer` (this layer) | update parameters from the gradient only | only receives the gradient vector; reads no data |

The two cooperate through `OptimizerSession` but are fully decoupled: swapping the
evaluator does not affect the optimizer, and swapping the optimizer does not
affect data reading. This is exactly the user's required "decouple reading data
from optimizing into two parts".

```
        ParameterSpace (Space layer)  ── vectorizeDouble() ──▶
                                                            │
   OptimizerSession  ── evaluate(theta) ──▶  Evaluator (Evaluation layer)
   (Optimizer layer, orchestrator)                │ reads data: loss + ∇
                                                 ▼
                                       Optimizer.update(space, ∇)  ◀── optimize part
                                                 │
                                                 ▼
                                          RunHistory (Memory layer)  ◀── records curve/best
```

## Optimizers (default GD, optional SGD)

| Optimizer | Default? | Notes |
|---|---|---|
| `GradientDescent` | ✅ default | Full batch, θ ← θ − lr·∇. Most stable MVP starting point. |
| `SGD` | optional | Momentum + per-epoch learning-rate decay; must pair with a `stochastic` evaluator. |

Both inherit the `Optimizer` abstract base — that is the **custom interface**:

```cpp
class MyOptimizer : public AuroX::Optimizer::Optimizer {
public:
    void update(ParameterSpace& space, const std::vector<double>& g) override {
        auto theta = space.vectorizeDouble();
        for (size_t i = 0; i < theta.size(); ++i) theta[i] -= lr_ * g[i]; // your update rule
        space.unvectorizeDouble(theta);
    }
    const char* name() const override { return "MyOptimizer"; }
    bool isStochastic() const override { return false; }  // true -> mini-batch evaluation path
};
```

## OptimizerSession (closed-loop orchestrator)

`OptimizerSession` chains the four into a training loop; it only schedules, no algorithm inside:

- each round: take params → `Evaluator` evaluate (read data) → `Optimizer.update` (optimize) → write `RunHistory`;
- switches the "full-batch" or "mini-batch" evaluation path automatically based on `optimizer.isStochastic()`;
- uses `RunHistory::converged()/diverged()` for early stopping (the safety net for unbounded parameters).

## How to use (MVP multi-dim regression, default GD)

```cpp
#include "AuroX/Core.hpp"
using namespace AuroX;

// 1) Parameter space (unbounded continuous params, MVP regression): 3 weights = bias + 2 feature weights
Space::ParameterSpace space;
space.declare("w0", 0.0);   // bias (intercept)
space.declare("w1", 0.0);   // feature-1 weight
space.declare("w2", 0.0);   // feature-2 weight

// 2) Evaluator (the data-reading part): 3 samples, 3 dims (bias col 1 + 2 features); dim must match the parameter space
Evaluation::RegressionEvaluator evaluator(
    /*X=*/{{1, 0.5, 0.2}, {1, 1.0, 0.3}, {1, 1.5, 0.4}},
    /*y=*/{0.9, 1.35, 1.8},
    /*stochastic=*/false);

// 3) Optimizer (default gradient descent)
Optimizer::GradientDescent optimizer(/*lr=*/0.1);

// 4) History (memory-first, recentCapacity=20 ring buffer)
Memory::RunHistory history(/*recentCapacity=*/20, /*minimize=*/true);

// 5) Closed-loop orchestration
Optimizer::OptimizerSession session(space, evaluator, optimizer, history,
                                    /*maxIterations=*/500, /*tolerance=*/1e-6);
auto rep = session.run();

// Results
json curve = history.toJson();          // for the Operator Console to plot the loss curve
auto best = history.bestParams();       // best parameter vector
```

## Switch to SGD (optional)

Just change two lines — enable `stochastic` on the evaluator and swap to `SGD`:

```cpp
Evaluation::RegressionEvaluator evaluator(X, y, /*stochastic=*/true, /*miniBatchSize=*/2);
Optimizer::SGD optimizer(/*lr=*/0.1, /*momentum=*/0.9, /*decay=*/0.0);
Optimizer::OptimizerSession session(space, evaluator, optimizer, history);
session.run();   // Session detects isStochastic()=true and auto-takes the mini-batch path
```

> Key point: SGD's "stochastic" comes from the evaluator's mini-batch sampling;
> the optimizer only owns the update rule. This matches frameworks like PyTorch
> (optimizer + DataLoader separation).

## Custom evaluator (custom interface)

Implement an `Evaluator` subclass to join the loop; no change to the optimizer or space:

```cpp
class MyEvaluator : public Evaluation::Evaluator {
public:
    size_t dimension() const override { return 3; }
    EvaluatorResult evaluate(const std::vector<double>& p) const override {
        EvaluatorResult r;
        r.gradient.assign(3, 0.0);
        r.value = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];   // e.g. minimize ||p||²
        r.gradient[0] = 2*p[0]; r.gradient[1] = 2*p[1]; r.gradient[2] = 2*p[2];
        return r;
    }
};
```

## File locations

- Declaration (optimizer): `include/AuroX/Optimizer/Optimizer.hpp`
- Implementation (optimizer): `src/Optimizer/Optimizer.cpp`
- Declaration (orchestrator): `include/AuroX/Optimizer/OptimizerSession.hpp`
- Implementation (orchestrator): `src/Optimizer/OptimizerSession.cpp`
- Evaluator: `include/AuroX/Evaluation/Evaluator.hpp` + `src/Evaluation/Evaluator.cpp`
- Aggregated automatically by the `__has_include` in `AuroX/Core.hpp`; you can
  directly `#include "AuroX/Core.hpp"` to use it.

## Constraints and boundaries

- The evaluator dimension (`Evaluator::dimension()`) must equal the length of
  `ParameterSpace::vectorizeDouble()`; otherwise `OptimizerSession::run()` returns
  `dimensionMismatch=true` and refuses to run.
- The parameter space should contain only vectorizable numeric parameters (the
  unbounded continuous regression scenario satisfies this naturally).
- The `Optimizer` layer does **no** persistence; history still lives in memory via
  `Memory/RunHistory` (see that layer's notes).
- Phase 3 surrogate models / experience libraries are not in this layer; this
  layer only handles single-task closed-loop optimization and orchestration.
