# Core layer: Space

## ParameterSpace

**ParameterSpace** describes, stores, validates, and serializes a set of named
parameters.

- **Standard-type representation**: parameter values use C++ standard types
  directly — `bool` / `int` / `float` / `double` / `std::string`, plus a
  **named enum type**, with no custom wrapper needed.
- **Continuous / discrete / enum parameters**:
  - Continuous: with lower/upper bounds (`lower` / `upper`), e.g. learning rate, batch size.
  - Discrete: with a finite candidate set (`choices`), e.g. optimizer name `{adam, sgd, rmsprop}`.
  - Enum: a discrete variable with a **named label set**, e.g. color `{red, green, blue}`;
    stored and vectorized internally as "the index of the selected label" (see "Enum parameters" below).
- **Extensible base class**: `ParameterBase` is an abstract base class; callers can
  derive their own parameter types (e.g. struct parameters) and register them via
  `add(std::make_unique<MyParam>(...))` — zero framework changes.
- **Full serialization**: based on `nlohmann::json`, the whole parameter space can
  be serialized to / from config for persistence and network transport.
- **Vectorize / unvectorize** (for optimizer iteration):
  - `vectorize<T>()`: collect same-type parameters into `std::vector<T>`;
  - `vectorizeDouble()`: pack all numeric parameters into a unified `std::vector<double>` (most optimizer-friendly);
  - `unvectorize*()`: write the optimizer-updated contiguous vector back into the parameter objects;
  - `toJson()` / `valueToJson()`: expose structured parameters to the front end for editing.

Core types:
- `ParameterBase`: abstract parameter base class (pure-virtual interface: naming, kind, type, clone, bounds check, serialization, vectorization).
- `TypedParameter<T>`: built-in templated parameter class covering `bool/int/float/double/std::string`; users normally need not write their own.
- `ParameterSpace`: owns all parameters (`std::unique_ptr`), provides `add` / `get` / `declare` / serialization / vectorization.

> Run history `RunHistory` belongs to the **Memory** layer (Layer 2: Data &
> Memory), not the Space layer. See `include/AuroX/Memory/README.md`.

## How to use

### 1. Create a space and declare parameters

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Space;

ParameterSpace space;

// Continuous (with bounds)
auto* lr = space.declare("learning_rate", 0.01f, 0.001f, 0.1f);
auto* bs = space.declare("batch_size",    32,    1,      128);

// Discrete (candidate set)
auto* opt = space.declareDiscrete("optimizer", std::string("adam"),
                                  {std::string("adam"), std::string("sgd"), std::string("rmsprop")});

// Boolean
auto* drop = space.declare("use_dropout", true, false, true);

// Unbounded continuous (constraint delegated to the safety layer)
auto* temp = space.declare("temperature", 1.0);
```

### 2. Access and modify parameters

```cpp
// Directly through the raw pointer returned by declare
lr->value = 0.05f;

// Look up by name
ParameterBase* p = space.get("batch_size");

// Type-safe access (returns nullptr on failure)
if (auto* b = space.getAs<int>("batch_size")) {
    b->value = 64;
}

// Bounds / validity check (continuous -> bounds, discrete -> candidate set)
bool ok = p->isWithinBounds();
```

### 3. Serialize and deserialize

```cpp
// Whole space -> JSON
json config = space.toJson();

// Rebuild the whole space from JSON (missing params are created, duplicates overwritten)
ParameterSpace restored;
restored.fromJson(config);
```

### 4. Vectorize (hand to the optimizer for iteration)

```cpp
// Unified double vector
std::vector<double> vec = space.vectorizeDouble();

// Write back after the optimizer modifies vec
space.unvectorizeDouble(vec);

// Collect by same type
std::vector<float> floats = space.vectorize<float>();
space.unvectorize(floats);
```

### 5. Custom parameter types (optional)

Inherit `ParameterBase`, implement all virtuals, then register with `add()`:

```cpp
class MyParam : public ParameterBase {
    // Implement name/kind/typeId/typeName/clone/isWithinBounds/toJson/fromJson/
    //        valueToJson/setValueFromJson/packDouble/unpackDouble/componentCount
};
space.add(std::make_unique<MyParam>(/* ... */));
```

### 6. Enum (categorical) parameters

`declareEnum` declares a discrete parameter over **named labels** (distinct from
`declareDiscrete`'s "numeric candidate set"). Internally it stores "the index of
the selected label", its kind is `Discrete` and its type id is `Enum`; when
vectorized it packs as a single `double` index, so it slots seamlessly into
`vectorizeDouble()` / `unvectorizeDouble()`.

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Space;

ParameterSpace space;
space.declare("w0", 0.0);

// Enum: label set {red, green, blue}, default selection green (1st, 0-based index)
auto* color = space.declareEnum("color", std::string("green"),
                                {"red", "green", "blue"});

color->label();          // "green"
color->index();          // 1
color->setLabel("blue"); // switch to blue -> index()==2

// Unknown labels auto-extend the label set (stays representable)
color->setLabel("yellow"); // label set becomes 4, index()==3

// Vectorize: w0 (unbounded) + color index = double vector of length 2
auto v = space.vectorizeDouble();   // v[1] == 1.0 (green)
v[1] = 0.0;
space.unvectorizeDouble(v);          // write back -> color becomes red

// Type-safe lookup
if (auto* c = space.getEnum("color")) {
    std::cout << c->label() << "\n";
}

// Serialization round-trip: type field is "enum", with labels / value / label
json cfg = space.toJson();
ParameterSpace restored;
restored.fromJson(cfg);              // full enum rebuild (labels + selection)
```

> MVP limitation: a gradient optimizer treats the enum index as a numeric value.
> True categorical-aware optimization (e.g. one-hot + a categorical solver) is
> future work; for now the enum packs/unpacks as a plain index scalar.

## Minimal example

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
#include <iostream>

int main() {
    AuroX::Space::ParameterSpace space;
    space.declare("learning_rate", 0.01f, 0.001f, 0.1f);
    space.declare("batch_size",    32,    1,      128);

    if (auto* lr = space.getAs<float>("learning_rate")) {
        std::cout << "before: " << lr->value << "\n";
        lr->value = 0.05f;
        std::cout << "after : " << lr->value << "\n";
    }

    std::cout << space.toJson().dump(4) << "\n";

    auto vec = space.vectorizeDouble();
    vec[0] = 0.02;                 // optimizer changes learning rate to 0.02
    space.unvectorizeDouble(vec);
    std::cout << "updated lr: " << space.getAs<float>("learning_rate")->value << "\n";
    return 0;
}
```

## File locations

- Declaration (header): `include/AuroX/Space/ParameterSpace.hpp`
- Implementation (source): `src/Space/ParameterSpace.cpp`

---

## ActionSpace (in the Space layer)

### What problem it solves

`ActionSpace` is the decision surface placed **between the optimizer's output
(the gradient) and the parameter's actual update**. It demotes "the update the
optimizer produced" from "a step that is always applied" to "one default action
among several candidate actions", and lets you **serially stack** other actions
after it (e.g. a not-yet-implemented state transition, a camera move, a motor
command), all finally applied to the parameter vector together.

This layer maps exactly to the earlier design discussion:
- the optimizer only computes the parameter **delta** — i.e. "the judgment basis
  comes from the optimizer";
- the default action set contains **only the parameter update**;
- custom actions can be stacked serially;
- everything can `toJson()` / `fromJson()`, so the front end can reconstruct the
  whole task data-flow without a back end.

### Layering and decoupling

`ActionSpace` is in the Space layer and **depends only on `ParameterSpace` and
json**; it does not directly `#include` the optimizer header. The "judgment basis
from the optimizer" is injected at runtime through a **callback
(`ParameterUpdateFn`)**: the orchestrator (`OptimizerSession`) wires
`Optimizer::computeUpdate` as `ActionSpace`'s update source. This keeps the Space
layer free of a reverse dependency on the Optimizer layer (avoiding a circular
include) while the default action remains optimizer-driven.

### Core types

- `Action` (abstract base = the custom interface): implement `type()` / `name()`
  / `feasible()` / `apply()` / `clone()` / `config()` / `setConfig()` to plug in
  any action.
- `ParameterUpdateAction` (default action): on `apply`, asks the update source for
  the delta and writes it back; with no source wired, it degrades to "a gradient
  step at the default learning rate", remaining usable standalone.
- `CustomAction` (placeholder custom action): represents a not-yet-implemented
  action (e.g. state transition, camera move, motor command). It **does NOT
  execute real hardware** — it only carries data:
  - `params`: arbitrary action parameters (e.g. target pose, speed);
  - `state`: an attached (not-yet-implemented) state-space description;
  - `transition`: an attached state-transition equation / model description;
  - `paramDelta`: an optional demonstration delta applied to the parameter vector
    on `apply` (empty = pure placeholder, no side effect).

  This is exactly how "add a not-yet-implemented state space / transition /
  parameters" is realized today — as **data**, not executed code; the front end
  can render it directly from JSON.
- `ActionSpace`: owns an ordered action list. The default set contains **only the
  parameter update**; `addAction()` stacks after it; `apply()` runs in order;
  `clearCustom()` resets to the default set.

### Serial-stacking example

```cpp
#include "AuroX/Space/ActionSpace.hpp"
using namespace AuroX::Space;

ActionSpace actions;

// Already contains by default: the parameter update (judgment basis from the
// optimizer, wired by OptimizerSession)

// Serially stack a "move camera" custom action (state space / transition
// equation are currently just data placeholders)
actions.addAction(std::make_unique<CustomAction>(
    /*name=*/ "move_camera",
    /*params=*/ json{{"target", {0.0, 1.0, 0.0}}, {"speed", 0.5}},
    /*state=*/ json{{"space", "camera_pose"}, {"dim", 3}},
    /*transition=*/ json{{"equation", "x_{k+1} = f(x_k, u_k)"}, {"model", "unimplemented"}},
    /*paramDelta=*/ {}   // empty = pure placeholder; can hold a param-dimension vector for demos
));
```

During iteration (with `OptimizerSession` connected):

```cpp
AuroX::Optimizer::OptimizerSession session(space, evaluator, optimizer, history);
session.setActionSpace(&actions);   // wire the action space; nullptr reverts to original behavior
auto rep = session.run();
```

### Serialization and deserialization (the front/back-end decoupling key)

```cpp
// Whole action space -> JSON (front end can draw the action pipeline, rebuild the data-flow)
json a = actions.toJson();

// Rebuild from JSON (factory by type tag via createAction; default set never empty)
ActionSpace restored;
restored.fromJson(a);
```

Serialized shape:

```json
{
  "version": 1,
  "actions": [
    { "type": "parameter_update", "name": "parameter_update", "config": {} },
    { "type": "custom", "name": "move_camera",
      "config": {
        "params":     { "target": [0,1,0], "speed": 0.5 },
        "state":      { "space": "camera_pose", "dim": 3 },
        "transition": { "equation": "x_{k+1} = f(x_k, u_k)", "model": "unimplemented" }
      }
    }
  ]
}
```

### File locations

- Declaration (header): `include/AuroX/Space/ActionSpace.hpp`
- Implementation (source): `src/Space/ActionSpace.cpp`
- Wiring: `include/AuroX/Optimizer/OptimizerSession.hpp` (`setActionSpace`)

---

## ObservationSpace (in the Space layer)

### What problem it solves

The observation space **captures every quantifiable external input, vectorizes
it, and maintains it over time**, handing the current observation window to the
state space (StateSpace) for estimation / identification at any time. It is the
**input side** of the "system identifier / system modeler" from the design docs
— it unifies "the quantifiable information y the environment emits" into a
`vector<double>` language the optimizer/model can read, forming a dual with
`ParameterSpace` (the control u we inject).

### Layering and file split

| File | Responsibility |
|---|---|
| `ObservationEncoder.hpp` + `ObservationEncoder.cpp` | The three **default encoders** (continuous / discrete / categorical enum). Pure `vectorize`, **stateless, no serialization, no unvectorize, input-only**. |
| `ObservationSpace.hpp` + `ObservationSpace.cpp` | `ObservationChannel` base + concrete channels (continuous/discrete/categorical/RawVector) + the `ObservationSpace` container. Channels are stateful (hold a FIFO) and **channels are serializable**. |

> Both modules are now split into a header (declarations) plus a matching `.cpp`
> under `src/Space/`. Because the project's CMake uses `file(GLOB_RECURSE
> src/*.cpp)`, the new `.cpp` files are picked up automatically; **re-run
> `cmake -B build -S .`** if you add or move a source file, otherwise you get
> LNK2019.

### Default encoding (vectorize only)

An encoder is a **stateless pure function**: input → `vector<double>`; it holds
no members and does no serialization / unvectorize.

- **Continuous variable `ContinuousEncoder`**
  - Normalization modes: `minmax` ([min,max]→[0,1], out-of-range clamped) /
    `zscore` ((x-mean)/std, clamped to [clampLo,clampHi]) / `none` (pass-through).
  - `resolution r`: default 1 (a single normalized scalar); if `r>1` it outputs
    an `r`-length **soft histogram** (triangular-kernel binning), good for
    low-resolution observations.
- **Discrete variable `DiscreteEncoder`**
  - Modes: `scalar` (default, maps the level to a 1-length vector in [0,1]) /
    `thermometer` (ordered one-hot of length `r=levels`, preserving the ordinal
    relationship between levels).
  - `levels`: the ordered set of allowed values; empty degrades to identity (treated as a number).
- **Categorical (enum) variable `CategoricalEncoder`**
  - Outputs `r=K+1` **one-hot**: `K` known labels + 1 **unknown bit**; an unseen
    label lights the unknown bit instead of being treated as a valid category
    (avoiding the all-zero ambiguity).

### Channel semantics

The `ObservationChannel` base binds **one external input** and vectorizes it
directly. Key properties:

- **Vector length = resolution `r` × time width `T`** (each sample is `r` values, keep `T` time steps).
- **FIFO ring buffer**: each sample's `r` quantized values are first-in-first-out; drop the oldest beyond `T`.
- **Pull-on-demand**: `ObservationSpace::requestVector(id)` — **whenever the state
  space pulls, it returns the current flattened vector**; **unfilled slots are
  zero-filled**.
- **Serializable / deserializable**: channel metadata (id/name/kind/resolution/timeWidth/encoder config) + the current FIFO window, for disk persistence, warm-start, and logging. The encoder itself is **not serialized** (it is stateless config, described by the channel).
- **Optional feed-forward compensator `ObservationCompensator`**: default
  `NoOpCompensator`; users derive and implement `apply(vector<double>& window)`,
  which acts on the whole `r×T` window at the **post-encode / pre-output** stage
  (e.g. bias removal, detrending, canceling a known command effect). Note: it is
  called on every pull; stateful compensators must be idempotent or designed to
  tolerate re-application.

Concrete channels (same files): `ContinuousObservationChannel` /
`DiscreteObservationChannel` / `CategoricalObservationChannel` (consume `double` /
`double` / `string` respectively), plus `RawVectorObservationChannel` (binds an
input that **is already a vector**, e.g. an embedding / feature, with `resolution`
= its intrinsic length, pushed into the FIFO without re-encoding).

### The `ObservationSpace` container

- `addChannel(...)`: register a fully-constructed channel (the space takes ownership).
- `ingest(id, value, ts)`: three overloads (`double` / `string` / `vector<double>`), auto-routed to the right channel; a type mismatch is safely ignored.
- `requestVector(id)`: pull a single channel's current window (zero-filled).
- `collect()`: concatenate all channels into one combined observation `y`, with a
  **layout** (per-channel `offset` / `length`) for the Observer to slice.
- `serialize()` / `deserialize()` / `describe()` / `reset()`: whole-state persistence and description.

### Minimal example (illustrative, not wired into main)

```cpp
#include "AuroX/Space/ObservationSpace.hpp"
using namespace AuroX::Space;

ObservationSpace obs;

// Continuous: temperature, normalized to [0,10], time width 3 (keep last 3 steps)
ContinuousEncoder ce; ce.mode = ContinuousEncoder::Mode::MinMax;
ce.min = 0.0; ce.max = 10.0;
obs.addChannel(std::make_unique<ContinuousObservationChannel>("temp", "Temperature", 3, ce));

// Discrete: gear, levels {0,1,2}, time width 2
DiscreteEncoder de; de.levels = {0.0, 1.0, 2.0};
obs.addChannel(std::make_unique<DiscreteObservationChannel>("gear", "Gear", 2, de));

// Categorical: mode, labels {idle, run, err}, time width 2
obs.addChannel(std::make_unique<CategoricalObservationChannel>(
    "mode", "Mode", std::vector<std::string>{"idle", "run", "err"}, 2));

// An input that is already a vector (e.g. a feature), length 2, time width 1
obs.addChannel(std::make_unique<RawVectorObservationChannel>("feat", "Feature", 2, 1));

// Feed external data (multi-source, heterogeneous types; ingest auto-routes)
obs.ingest("temp", 5.0);                 // -> normalized 0.5
obs.ingest("gear", 1.0);
obs.ingest("mode", std::string("run"));
obs.ingest("feat", std::vector<double>{0.3, 0.7});

// The state space pulls at any time: unfilled slots are zero
std::vector<double> tempVec = obs.requestVector("temp");  // length 3 = [0.5, 0, 0]

// Combined observation + layout (for Observer / identifier consumption)
auto bundle = obs.collect();             // bundle.vector, bundle.layout
// bundle.layout[i] = { id, offset, length }

// Persist / restore
json snap = obs.serialize();
ObservationSpace restored;
restored.deserialize(snap);              // full rebuild of all channels + FIFO window

// Optional: user custom feed-forward compensator (subtract a known bias before
// returning to the state space)
struct BiasComp : ObservationCompensator {
    void apply(std::vector<double>& w) override { for (double& x : w) x -= 0.1; }
    std::string name() const override { return "bias_sub"; }
};
obs.get("temp")->setCompensator(std::make_unique<BiasComp>());
```

> MVP scope: this file **only implements observation capture / encoding / holding
> / pulling**; it is not wired into `main.cpp` and does not connect the
> identifier or state space. Multi-rate **alignment (global tick / hold / NaN
> policy)** across heterogeneous sources and the **missing-vs-true-zero semantic
> mask** are deferred enhancements; for now we provide minimal support via
> "zero-fill + expose fillCount/isFull".

### File locations

- Declaration + implementation (header + source): `include/AuroX/Space/ObservationEncoder.hpp` + `src/Space/ObservationEncoder.cpp`
- Declaration + implementation (header + source): `include/AuroX/Space/ObservationSpace.hpp` + `src/Space/ObservationSpace.cpp`
