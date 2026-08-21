# Core layer: Evaluation (read data / evaluate, the data side of Layer 4)

## What this layer is

The `Evaluation` layer holds **Part 1 of "decoupling reading data from
optimizing": reading data / evaluating**. It pairs with the `Optimizer` layer
(Part 2: optimize / update) via `OptimizerSession`, but the two are decoupled.

- `Evaluator` only cares about: given a parameter vector `std::vector<double>`,
  compute objective value + gradient;
- it **never touches parameter updates**, and **does not depend on
  `ParameterSpace`** (it only receives the parameter vector), so the data-reading
  logic can be swapped freely.

> Boundary with the Space layer: Space describes "what parameters can be tuned";
> Evaluation is responsible for "given the parameters, what does the data say
> (loss + gradient)".

## Evaluator (abstract base = custom interface)

```cpp
struct EvaluatorResult { double value; std::vector<double> gradient; };

class Evaluator {
    virtual EvaluatorResult evaluate(const std::vector<double>& params) const = 0; // full batch (GD)
    virtual size_t dimension() const = 0;
    virtual size_t sampleCount() const;        // N, used by SGD
    virtual size_t batchSize() const;         // mini-batch size, 0 = full batch
    virtual bool supportsBatching() const;    // SGD
    virtual void beginEpoch();                // new-epoch shuffle hook
    virtual std::vector<size_t> nextBatchIndices(size_t b);  // next mini-batch indices
    virtual EvaluatorResult evaluateBatch(const std::vector<double>&, const std::vector<size_t>&) const; // mini-batch
};
```

A custom evaluator only needs to inherit and implement `evaluate()` (plus the
optional batch interface) to join the optimization loop.

## RegressionEvaluator (MVP reference implementation)

Multi-dimensional linear regression, minimizing mean-squared error MSE:

```
loss(θ) = (1/N) Σ_i (x_i·θ − y_i)^2
∇loss   = (2/N) Xᵀ (Xθ − y)
```

- Full batch (`stochastic=false`, with `GradientDescent`) → gradient over all N samples;
- Mini-batch (`stochastic=true`, with `SGD`) → maintains a shuffle order internally;
  each `nextBatchIndices` returns one mini-batch, `evaluateBatch` computes the
  empirical gradient only on that batch.

Construction:

```cpp
Evaluation::RegressionEvaluator evaluator(
    /*X=*/std::vector<std::vector<double>>{{1,0.5},{1,1.0},{1,1.5}},  // row-major N×D, first column may hold bias 1
    /*y=*/std::vector<double>{1.0, 2.0, 3.0},
    /*stochastic=*/false,
    /*miniBatchSize=*/32,
    /*seed=*/1u);
```

## File locations

- Declaration: `include/AuroX/Evaluation/Evaluator.hpp`
- Implementation: `src/Evaluation/Evaluator.cpp`
- Aggregated by the `__has_include` in `AuroX/Core.hpp`; you can directly
  `#include "AuroX/Core.hpp"` to use it.
