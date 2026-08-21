#pragma once

// ============================================================================
// AuroX - ActionSpace  (part of the Space layer)
// ----------------------------------------------------------------------------
// Purpose:
//   An ActionSpace is the "what to do next" decision surface that sits BETWEEN
//   the optimizer's output (the gradient) and the actual parameter update. It
//   turns the optimizer's output from "a step that is always applied" into
//   "one candidate action among possibly several", and lets the caller stack
//   additional actions (e.g. a not-yet-implemented state transition, a camera
//   move, a motor command) in series after the default parameter update.
//
//   This is the concrete realization of the design discussed earlier:
//     - the optimizer computes a parameter DELTA (the "judgment basis");
//     - the default action is exactly that parameter update;
//     - custom actions can be added and applied serially;
//     - everything serializes to / from nlohmann::json so the whole task can be
//       saved offline and reconstructed by the front end without the back end.
//
// Layering:
//   ActionSpace lives in the Space layer and depends ONLY on ParameterSpace
//   (and json). It does NOT include the Optimizer header -- the "basis from
//   the optimizer" is injected at runtime through a callback (ParameterUpdateFn)
//   that the orchestrator (OptimizerSession) wires to the live optimizer. This
//   keeps the Space layer decoupled from the Optimizer layer (no circular
//   include) while still letting the default action be optimizer-driven.
//
// Serialization contract (front-end / back-end decoupling):
//   toJson()  -> { "version":1, "actions":[ {type, name, config}, ... ] }
//   fromJson()-> rebuilds the exact action list via a type-keyed factory.
//   The front end can therefore draw the action pipeline and reproduce the
//   data-flow graph purely from this JSON, with no C++ back end.
// ============================================================================

#include "json.hpp"
#include "AuroX/Space/ParameterSpace.hpp"

#include <functional>
#include <memory>
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
namespace Space {

using json = nlohmann::json;

// Update source: given the current parameter space and the optimizer gradient
// of this iteration, return the parameter DELTA the optimizer would apply.
// The orchestrator wires this to Optimizer::computeUpdate; ActionSpace itself
// never references the Optimizer type directly.
using ParameterUpdateFn =
    std::function<std::vector<double>(ParameterSpace& space,
                                      const std::vector<double>& gradient)>;

// ----------------------------------------------------------------------------
// Abstract action. A custom action only needs to subclass Action and implement
// apply() (and, optionally, config()/setConfig()/feasible()/clone()/type()).
// This is the reserved custom interface.
// ----------------------------------------------------------------------------
class AUROX_API Action {
public:
    virtual ~Action() = default;

    // Stable type tag used for (de)serialization. Must be unique per concrete
    // type (e.g. "parameter_update", "custom").
    virtual const char* type() const = 0;

    // Human-readable label (for UI display). Defaults to type().
    virtual std::string name() const { return type(); }

    // Whether this action may be applied in the current state. Used by the
    // safety layer. Default: always feasible.
    virtual bool feasible(const ParameterSpace& space) const {
        (void)space;
        return true;
    }

    // Apply the action to the parameter vector. `gradient` is this iteration's
    // optimizer gradient (may be ignored by actions that do not depend on it,
    // e.g. a camera move). `updateSource` is the optimizer-backed parameter
    // update function; the default ParameterUpdateAction uses it.
    virtual void apply(ParameterSpace& space,
                       const std::vector<double>& gradient,
                       const ParameterUpdateFn& updateSource) const = 0;

    // Serialization of this action's own configuration.
    virtual json config() const { return {}; }
    virtual void setConfig(const json& j) { (void)j; }

    // Deep copy so ActionSpace owns independent copies.
    virtual std::unique_ptr<Action> clone() const = 0;
};

// ----------------------------------------------------------------------------
// Default action: the parameter update. Its delta is produced by updateSource
// (wired to the optimizer by the orchestrator), so the "judgment basis comes
// from the optimizer" while ActionSpace stays decoupled from the Optimizer
// layer. If no update source is wired (standalone use), it falls back to a
// plain gradient step with a default learning rate so the action is meaningful.
// ----------------------------------------------------------------------------
class AUROX_API ParameterUpdateAction : public Action {
public:
    const char* type() const override { return "parameter_update"; }

    void apply(ParameterSpace& space,
               const std::vector<double>& gradient,
               const ParameterUpdateFn& updateSource) const override;

    std::unique_ptr<Action> clone() const override {
        return std::make_unique<ParameterUpdateAction>(*this);
    }
};

// ----------------------------------------------------------------------------
// Placeholder custom action. Represents a not-yet-implemented action such as a
// state-space transition, a camera move, or a motor command. It does NOT
// execute real hardware. It carries:
//   - name        : a label (e.g. "move_camera");
//   - params      : arbitrary action parameters (e.g. target pose, speed);
//   - state       : an attached (not-yet-implemented) state-space description;
//   - transition  : an attached state-transition equation / model descriptor;
//   - paramDelta  : an OPTIONAL demonstration delta applied to the parameter
//                   vector on apply (empty = pure placeholder, no effect).
// The front end can render all of the above directly from JSON, which is how
// "add a not-yet-implemented state space / transition / parameters" is supported
// today: as data, not as executed code.
// ----------------------------------------------------------------------------
class AUROX_API CustomAction : public Action {
public:
    CustomAction() = default;

    CustomAction(std::string actionName,
                 json params = {},
                 json state = {},
                 json transition = {},
                 std::vector<double> paramDelta = {})
        : name_(std::move(actionName)),
          params_(std::move(params)),
          state_(std::move(state)),
          transition_(std::move(transition)),
          paramDelta_(std::move(paramDelta)) {}

    const char* type() const override { return "custom"; }
    std::string name() const override { return name_.empty() ? std::string("custom") : name_; }

    bool feasible(const ParameterSpace& space) const override;
    void apply(ParameterSpace& space,
               const std::vector<double>& gradient,
               const ParameterUpdateFn& updateSource) const override;
    json config() const override;
    void setConfig(const json& j) override;
    std::unique_ptr<Action> clone() const override {
        return std::make_unique<CustomAction>(*this);
    }

    // Accessors for the attached (possibly unimplemented) state-space / transition data.
    const json& params() const { return params_; }
    const json& state() const { return state_; }
    const json& transition() const { return transition_; }
    void setState(const json& s) { state_ = s; }
    void setTransition(const json& t) { transition_ = t; }
    void setParamDelta(std::vector<double> d) { paramDelta_ = std::move(d); }

private:
    std::string name_;
    json params_;                 // arbitrary action parameters
    json state_;                  // attached (unimplemented) state-space description
    json transition_;             // attached state-transition equation / model descriptor
    std::vector<double> paramDelta_;  // optional demonstration delta applied to params
};

// ----------------------------------------------------------------------------
// ActionSpace: owns an ordered list of actions. The DEFAULT set contains ONLY
// the parameter update. Custom actions are added and applied serially AFTER it.
// ----------------------------------------------------------------------------
class AUROX_API ActionSpace {
public:
    // Construct with the default action set: only the parameter update.
    ActionSpace();

    // Set the optimizer-backed update source (the "basis from optimizer").
    // Called by the orchestrator before run().
    void setUpdateSource(ParameterUpdateFn fn) { updateSource_ = std::move(fn); }
    const ParameterUpdateFn& updateSource() const { return updateSource_; }

    // Add a custom action, serially stacked AFTER the existing actions.
    void addAction(std::unique_ptr<Action> a);

    // Number of actions (including the default parameter update).
    size_t size() const { return actions_.size(); }
    bool empty() const { return actions_.empty(); }

    // Apply all actions in series: default parameter update first, then any
    // custom actions in the order they were added.
    void apply(ParameterSpace& space, const std::vector<double>& gradient) const;

    // Read-only access to the action list (for UI / debugging).
    const std::vector<std::unique_ptr<Action>>& actions() const { return actions_; }

    // Remove all custom actions (reset to the default-only set).
    void clearCustom();

    // --- Serialization (front-end / back-end decoupling) ---
    json toJson() const;
    void fromJson(const json& j);

private:
    ParameterUpdateFn updateSource_;
    std::vector<std::unique_ptr<Action>> actions_;
};

// Factory: rebuild a single action from its serialized form.
// Returns nullptr only if the type tag is unknown AND empty.
std::unique_ptr<Action> createAction(const json& serialized);

}  // namespace Space
}  // namespace AuroX
