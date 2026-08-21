# AuroX - Dynamics layer

> Located in `include/AuroX/Dynamics/`, on the same level as `Space` and
> `Evaluation` (near framework layers L3/L5). It consumes the output of
> **ObservationSpace** below, provides state prediction to **Policy /
> Optimizer** above, and writes the standardized state vector `x̂` to
> **StateSpace** (in `Space/StateSpace.hpp`).

## Responsibilities

This layer keeps the "model-based closed loop" trio cohesive:

| Module | File | Responsibility |
|---|---|---|
| State estimator (Observer) | `StateEstimator.hpp` | filters/denoises/fuses the observation vector `y` into the best current state estimate `x̂` |
| State predictor / transition model | `TransitionModel.hpp` | `xₜ₊₁ = f(xₜ, uₜ)`; white-box physics / gray-box / black-box |
| System identifier | `SystemIdentifier.hpp` | corrects/refits `f` online or offline so it matches the true dynamics |

The orchestrator `Dynamics.hpp` wires the three together with
`ObservationSpace` / `StateSpace`:

```
ObservationSpace --(y)--> StateEstimator --> StateSpace (holds x̂)
                                       |
                                       v
                            TransitionModel (predict / rollout)
                                       ^
                            SystemIdentifier (fits LinearTransitionModel)
```

> Design note: this layer keeps all the "estimate / predict / identify"
> intelligence to itself; `StateSpace` is merely a **standardized state-vector
> holder** (no algorithm inside), matching the "thin recipient" separation.

## File list

All modules are now split into a header (declarations) plus a matching
implementation `.cpp` under `src/Dynamics/`:

- `LinAlg.hpp` + `LinAlg.cpp` — internal linear algebra (matrix multiply /
  transpose / inverse / linear-system solve, row-major, no external
  dependency). The helpers live in the `AuroX::Dynamics::LinAlg` namespace
  (the file name makes the purpose self-documenting; the old vague
  `internal` namespace was removed).
- `TransitionModel.hpp` + `TransitionModel.cpp` — `TransitionModel` abstract +
  `LinearTransitionModel` (the default identified model `x'=Ax+Bu`) +
  `WhiteBoxModel` (**user-supplied white-box model hook**).
- `StateEstimator.hpp` + `StateEstimator.cpp` — `StateEstimator` abstract +
  `PassThroughEstimator` (pass-through, minimal default) +
  `LowPassEstimator` (exponential-smoothing denoiser) + `KalmanEstimator`
  (linear Kalman observer).
- `SystemIdentifier.hpp` + `SystemIdentifier.cpp` — `SystemIdentifier` abstract
  + `LinearIdentifier` (least-squares fit of A, B via normal equations +
  Gaussian elimination).
- `Dynamics.hpp` + `Dynamics.cpp` — orchestrator that connects
  ObservationSpace and StateSpace.

> Because every module now has a `.cpp`, the build's `file(GLOB_RECURSE
> src/*.cpp)` picks them up automatically. **After adding or moving a `.cpp`,
> re-run `cmake -B build -S .` once** (otherwise the GLOB won't see it →
> LNK2019).

## Default capabilities

1. **Default linear-dynamics identification**: least-squares over the closed-loop
   history `(x, u, x')` to obtain `x' = A x + B u`.
2. **User white-box model**: `Dynamics::useWhiteBox(n, m, f)` swaps in a
   hand-written physics formula `f(x,u)->x'` at any time.
3. **State estimation**: pass-through by default (requires obs dim == state dim);
   Kalman / low-pass optional.
4. **Look-ahead prediction**: `simulate(u)` predicts the next state (for the
   Optimizer/Policy to evaluate an action), `rollout(us)` produces a
   multi-step trajectory (MPC).

## Minimal example

```cpp
#include "AuroX/Space/ObservationSpace.hpp"
#include "AuroX/Space/StateSpace.hpp"
#include "AuroX/Dynamics/Dynamics.hpp"
#include <memory>

using namespace AuroX;

int main() {
    const size_t n = 2, m = 1;

    // 1) Observation space: one "state" raw-vector channel
    Space::ObservationSpace obs;
    obs.addChannel(std::make_unique<Space::RawVectorObservationChannel>("x", "state", n, 1));

    // 2) State space (thin holder)
    Space::StateSpace state;

    // 3) Dynamics layer: pass-through estimate + default linear model + linear identifier
    auto dyn = std::make_unique<Dynamics::Dynamics>(
        &obs, &state,
        std::make_unique<Dynamics::PassThroughEstimator>(n),
        std::make_unique<Dynamics::LinearTransitionModel>(n, m),
        std::make_unique<Dynamics::LinearIdentifier>(n, m));

    // 4) Closed loop: apply control -> observe -> step
    std::vector<double> u = {0.0};
    for (int t = 0; t < N; ++t) {
        // ... advance the real system, obtain a new observation yo ...
        obs.ingest("x", yo, t);
        dyn->step(u, t);          // writes x̂ into state and feeds the identifier
        // ... use state.getState() / dyn->simulate(u_next) for decisions ...
    }

    // 5) Offline identification, adopt the fitted linear model
    dyn->identify();

    // 6) Or swap in a user white-box model at any time
    dyn->useWhiteBox(n, m, [](const std::vector<double>& x, const std::vector<double>& u) {
        return std::vector<double>{ x[0] + u[0], 2.0 * x[1] };  // hand-written physics
    });
}
```

## Wiring convention (must respect)

The contract of `Dynamics::step(u)` is: **apply control `u` first, THEN observe
the new state, THEN call `step(u)`**. In other words, the observation should
reflect "the state after control", so the identifier records `(xₜ, uₜ, xₜ₊₁)`.
If you feed in the "pre-transition" state, the control/state pairing is off by
one step and the identification will diverge.

## Future extensions

- `GrayBoxModel` / `BlackBoxModel` (NN, GP) plugged into the `TransitionModel` interface.
- Online RLS recursive identification (`LinearIdentifier` is currently batch `fit()`).
- Model uncertainty / confidence output for SafetyManager and robust MPC.
- Multi-sensor fusion entry point (merge multiple channel observations in the `StateEstimator` layer).
