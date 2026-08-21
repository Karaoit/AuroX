# Core layer: Memory (Data & Memory, Layer 2)

## What this layer is

The `Memory` layer corresponds to **Layer 2: Data & Memory** (episode / history /
dataset / experience library) in `AuroX-General.md`. It records each task run's
data and distills it into reusable experience.

In the **MVP stage**, this layer implements only the minimal run history `RunHistory`:

- **Memory-first**: all data stays in memory; nothing is written to disk;
- **Optional export**: the caller writes a snapshot to a JSON file at a chosen path on demand;
- **Not a persistent storage layer**: Phase 3's structured episode store (structured logs + artifacts + similarity index + cross-task transfer) is not implemented here; it comes with the experience-library and surrogate-model phase.

> Boundary with the Space layer: Space (Layer 3) describes "what parameters can
> be tuned"; Memory (Layer 2) records "what has been run". Different
> responsibilities, so `RunHistory` lives in `Memory/`, not `Space/`.

## RunHistory (MVP run history)

`RunHistory` is decoupled from `ParameterSpace`: it records the optimizer-friendly
parameter vector (`std::vector<double>`, i.e. `ParameterSpace::vectorizeDouble()`),
not the parameter objects themselves.

### What it records (per the MVP decision: no full-history storage)

- **Per-iteration scalars** (`iteration` / `objective` / `gradientNorm` / `status` / `elapsedMs`),
  for the front end to plot the loss curve, with minimal memory cost;
- **Only one copy of the best parameters** (`best()` / `bestParams()`);
- **Recent parameters use a capacity-bounded ring buffer** (`recent()`, controlled
  by the `recentCapacity` constructor argument), used only for convergence /
  divergence diagnostics, avoiding full storage of unbounded high-dimensional vectors;
- **No disk writes**: when needed, the caller triggers a one-shot export via `save()`.

### Safety net for unbounded parameters

When parameters are unbounded (no `parameter_bounds`), the safety layer cannot
reject via bounds. Then `diverged()` detects NaN/Inf or objective blow-up as a
runtime-guard substitute.

### External caller-driven interface

| Interface | Purpose |
|---|---|
| `toJson()` | export a UI-friendly snapshot (`curve` + `best` + `recent`) |
| `fromJson(j)` | rebuild in-memory state from a `toJson()` snapshot |
| `save(dir, name)` | write `toJson()` to `<dir>/<name>.json` (creates the dir if missing); data stays in memory |
| `load(dir, name)` | read back the snapshot written by `save()`, replacing current contents (enables cross-run warm-start) |

## How to use

```cpp
#include "AuroX/Memory/RunHistory.hpp"
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Memory;
using namespace AuroX::Space;

ParameterSpace space;
space.declare("w0", 0.0);   // unbounded continuous parameter
space.declare("w1", 0.0);

// recentCapacity=20: keep the last 20 full parameter vectors for diagnostics
RunHistory history(/*recentCapacity=*/20, /*minimize=*/true);

for (size_t it = 0; it < 100; ++it) {
    std::vector<double> theta = space.vectorizeDouble();
    double loss = evaluateRegression(theta);   // user's regression objective
    history.record(it, loss, &theta);
    // ... optimizer updates theta and space.unvectorizeDouble(theta) ...
    if (history.diverged(/*maxObjective=*/1e6)) break;   // unbounded divergence guard
}

auto best = history.bestParams();              // best parameter vector
json snapshot = history.toJson();              // for the Operator Console

// Optional: export to a chosen JSON folder (caller decides the path; module does not hardcode)
history.save("D:/runs/my_task", "history");    // -> D:/runs/my_task/history.json

// Optional: a later run reads it back as a warm-start point
RunHistory restored;
if (restored.load("D:/runs/my_task", "history")) {
    auto prevBest = restored.bestParams();
}
```

### Convergence / divergence checks

```cpp
if (history.converged(/*tolerance=*/1e-4, /*window=*/10)) { /* converged */ }
if (history.diverged(/*maxObjective=*/1e6, /*window=*/5))  { /* diverged, roll back to best */ }
```

## File locations

- Declaration (header): `include/AuroX/Memory/RunHistory.hpp`
- Implementation (source): `src/Memory/RunHistory.cpp`
- Aggregated automatically by the `__has_include` in `AuroX/Core.hpp`; you can
  directly `#include "AuroX/Core.hpp"` to use it.

## Evolution path (when to extend this layer further)

Extend to the full Data & Memory layer per `AuroX-General.md` §6 when any of these hold:

- need **cross-run warm-start** (same regression task, different dataset, reuse historical best);
- need a **failure-case library / parameter-sensitivity distillation** to support Phase 3 surrogate models;
- need **reproducible auditing** (external compliance requirement).

Then introduce structured episode logs, artifact storage, and a similarity index;
for unbounded parameters, store only a low-rank summary or the best config, never the full trajectory.
