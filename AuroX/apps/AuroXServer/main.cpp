// ============================================================================
// AuroX - Complex Integrated Example
// ----------------------------------------------------------------------------
// Demonstrates the full closed-loop stack working together:
//   - ParameterSpace   : tunable parameters (Kp, Ki, Kd, + controller mode enum)
//   - ActionSpace      : default parameter-update action + a stacked custom action
//   - ObservationSpace : sensor readings (continuous) + mode (categorical) encoded
//   - StateSpace       : holds the estimated plant state each tick
//   - Dynamics         : white-box plant model + Kalman observer + system identifier
//   - Evaluator        : custom tracking-error cost (fallback when no dataset),
//                        or built-in RegressionEvaluator when data is supplied
//   - Optimizer         : GradientDescent (or SGD) via OptimizerSession
//   - RunHistory        : loss curve + best params + divergence/convergence guard
//
// Custom-interface fallback policy (per requirement):
//   If a built-in handler is unavailable, a user-supplied callable is used:
//     1. Dynamics transition : default = identified LinearTransitionModel;
//                              fallback = useWhiteBox() custom physics fn.
//     2. Evaluator           : default = RegressionEvaluator (needs X/y data);
//                              fallback = custom CostFunction when no data present.
//     3. ActionSpace action  : default = ParameterUpdateAction (optimizer-driven);
//                              fallback = CustomAction when a handler is missing.
// ============================================================================

#include "AuroX/Core.hpp"
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace AuroX;

// ----------------------------------------------------------------------------
// 0. Scenario constants
// ----------------------------------------------------------------------------
static const double kTs = 0.05;          // simulation timestep (s)
static const size_t kPlantOrder = 2;     // plant state dimension (x1, x2)
static const size_t kControlDim = 1;     // control dimension (u)
static const size_t kHorizon = 40;       // simulation steps per evaluation

// ----------------------------------------------------------------------------
// 1. Custom cost function (the Evaluator fallback interface)
//    Computes ISE of a 2nd-order plant driven by a PID controller whose gains
//    come from the ParameterSpace vector [Kp, Ki, Kd, _unused_].
// ----------------------------------------------------------------------------
struct CostFunction {
    std::vector<double> target;   // desired state (length kPlantOrder)

    // Run a closed-loop simulation and return (loss, gradient) w.r.t. gains.
    Evaluation::EvaluatorResult operator()(const std::vector<double>& gains) const {
        Evaluation::EvaluatorResult r;
        r.gradient.assign(gains.size(), 0.0);

        const double Kp = gains[0];
        const double Ki = gains[1];
        const double Kd = gains[2];

        // Finite-difference gradient (central difference) -- robust and simple.
        const double h = 1e-4;
        std::vector<double> base = {Kp, Ki, Kd};
        double baseLoss = simulate(base, Kp, Ki, Kd);

        for (size_t j = 0; j < 3; ++j) {
            std::vector<double> pp = base, pm = base;
            pp[j] += h; pm[j] -= h;
            double lp = simulate(pp, pp[0], pp[1], pp[2]);
            double lm = simulate(pm, pm[0], pm[1], pm[2]);
            r.gradient[j] = (lp - lm) / (2.0 * h);
        }
        r.value = baseLoss;
        return r;
    }

    // Closed-loop simulation: plant x' = A x + B u, PID on error e = r - y.
    double simulate(const std::vector<double>& /*gains*/,
                    double Kp, double Ki, double Kd) const {
        // Discrete plant: x_{k+1} = A x_k + B u_k, with A stable, B control.
        const double a11 = 0.90, a12 = kTs;
        const double a21 = -0.20, a22 = 0.95;
        const double b1 = 0.0, b2 = 0.30;

        double x1 = 0.0, x2 = 0.0;     // state
        double integ = 0.0;            // integral term
        double prevErr = 0.0;          // for derivative term
        double loss = 0.0;

        for (size_t k = 0; k < kHorizon; ++k) {
            double y = x1;             // measured output = first state
            double e = target[0] - y;  // tracking error
            integ += e * kTs;
            double deriv = (e - prevErr) / kTs;
            prevErr = e;

            // Anti-windup clip on the integral term.
            if (integ > 5.0) integ = 5.0;
            if (integ < -5.0) integ = -5.0;

            double u = Kp * e + Ki * integ + Kd * deriv;
            if (u > 10.0) u = 10.0;     // actuator saturation
            if (u < -10.0) u = -10.0;

            // Plant integration (explicit Euler).
            double nx1 = a11 * x1 + a12 * x2 + b1 * u;
            double nx2 = a21 * x1 + a22 * x2 + b2 * u;
            x1 = nx1; x2 = nx2;

            double err = target[0] - y;
            loss += err * err;          // integral of squared error
        }
        return loss;
    }
};

// ----------------------------------------------------------------------------
// 2. Custom Evaluator: wraps CostFunction when no dataset is available.
// ----------------------------------------------------------------------------
class PidCostEvaluator : public Evaluation::Evaluator {
public:
    explicit PidCostEvaluator(CostFunction f, size_t dim) : f_(std::move(f)), dim_(dim) {}

    size_t dimension() const override { return dim_; }

    Evaluation::EvaluatorResult evaluate(const std::vector<double>& params) const override {
        return f_(params);
    }

private:
    CostFunction f_;
    size_t dim_;
};

// ----------------------------------------------------------------------------
// 3. White-box plant model (the Dynamics transition fallback interface).
//    Supplied to Dynamics::useWhiteBox when we have an analytic plant.
// ----------------------------------------------------------------------------
std::vector<double> whiteBoxPlant(const std::vector<double>& x,
                                  const std::vector<double>& u) {
    const double a11 = 0.90, a12 = kTs;
    const double a21 = -0.20, a22 = 0.95;
    const double b1 = 0.0, b2 = 0.30;
    double u0 = (u.empty() ? 0.0 : u[0]);
    return {
        a11 * x[0] + a12 * x[1] + b1 * u0,
        a21 * x[0] + a22 * x[1] + b2 * u0
    };
}

// ----------------------------------------------------------------------------
// 4. A custom action that records the parameter delta it applies.
//    Demonstrates ActionSpace::CustomAction as the "missing handler" fallback:
//    if a named handler is not registered in the session, this custom action
//    is invoked to apply a demonstration delta + attach state/transition data.
// ----------------------------------------------------------------------------
class LoggingCustomAction : public Space::CustomAction {
public:
    LoggingCustomAction() : Space::CustomAction("log_gains") {
        // Attach a (not-yet-implemented) state-space + transition description so
        // the front end can render it from JSON. No real hardware runs here.
        setState(nlohmann::json::object({{"label", "controller_state"}, {"dim", kPlantOrder}}));
        setTransition(nlohmann::json::object({{"type", "linear"}, {"form", "x' = A x + B u"}}));
    }

    void apply(Space::ParameterSpace& space,
               const std::vector<double>& gradient,
               const Space::ParameterUpdateFn& updateSource) const override {
        // Default: still let the optimizer-driven parameter update happen.
        if (updateSource) {
            auto delta = updateSource(space, gradient);
            auto cur = space.vectorizeDouble();
            for (size_t i = 0; i < cur.size() && i < delta.size(); ++i) cur[i] += delta[i];
            space.unvectorizeDouble(cur);
        }
        // Log to a file for the Operator Console / offline analysis.
        std::ofstream log("custom_action_log.csv", std::ios::app);
        if (log) {
            auto p = space.vectorizeDouble();
            log << "log_gains";
            for (double v : p) log << "," << v;
            log << "\n";
        }
        // Throttle console noise: only print every 50th application (full
        // detail still goes to custom_action_log.csv every iteration).
        static long applyCount = 0;
        if (++applyCount % 50 == 0) {
            std::cout << "    [CustomAction] applied log_gains (call #" << applyCount
                      << "); state = " << nlohmann::json(state()).dump() << "\n";
        }
    }
};

// ----------------------------------------------------------------------------
// 5. Build the integrated pipeline.
// ----------------------------------------------------------------------------
int main() {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "=============================================================\n";
    std::cout << " AuroX integrated example: 4 spaces + Dynamics + Evaluator +\n";
    std::cout << " Optimizer, with custom-interface fallbacks\n";
    std::cout << "=============================================================\n\n";

    // ---- 5.1 ParameterSpace : PID gains + controller mode (enum) ----------
    // The 3 numeric gains are the optimized variables (vectorized first);
    // the enum is a categorical selector packed as its index, so the space
    // vectorizes to 4 components and the evaluator matches that dimension.
    Space::ParameterSpace space;
    space.declare("Kp", 1.0, 0.0, 20.0);
    space.declare("Ki", 0.1, 0.0, 20.0);
    space.declare("Kd", 0.05, 0.0, 20.0);
    // enum = "controller mode": the active compensation strategy.
    space.declareEnum("mode", size_t(0),
                      std::vector<std::string>{"PID", "PI", "PD", "Feedforward"});
    std::cout << "[ParameterSpace] declared " << space.size() << " parameters: ";
    for (auto& n : space.names()) std::cout << n << " ";
    std::cout << "\n";

    // ---- 5.2 ObservationSpace : sensor (continuous) + mode (categorical) --
    Space::ObservationSpace obs;
    obs.addChannel(std::make_unique<Space::ContinuousObservationChannel>(
        "sensor", "plant_output", /*timeWidth=*/5,
        Space::ContinuousEncoder{Space::ContinuousEncoder::Mode::MinMax, -10.0, 10.0}));
    obs.addChannel(std::make_unique<Space::CategoricalObservationChannel>(
        "mode", "controller_mode",
        std::vector<std::string>{"PID", "PI", "PD", "Feedforward"}, /*timeWidth=*/3));
    std::cout << "[ObservationSpace] channels = " << obs.size() << " (sensor + mode)\n";

    // ---- 5.3 StateSpace : holds the estimated 2-D plant state ------------
    Space::StateSpace state;
    state.setNames({"x1", "x2"});
    std::cout << "[StateSpace] dim = " << state.dim() << " (holds plant state)\n";

    // ---- 5.4 ActionSpace : default update + stacked custom action ---------
    Space::ActionSpace actions;
    actions.addAction(std::make_unique<LoggingCustomAction>());
    std::cout << "[ActionSpace] actions = " << actions.size()
              << " (parameter_update + LoggingCustomAction)\n";

    // ---- 5.5 Dynamics : Kalman observer + white-box plant (custom fallb.) -
    // Build the white-box transition model (custom-interface fallback for f).
    auto whiteModel = std::make_unique<AuroX::Dynamics::WhiteBoxModel>(
        kPlantOrder, kControlDim, whiteBoxPlant);
    // Kalman observer needs A, B (plant), C (obs map), Q, R.
    std::vector<double> A = {0.90, kTs, -0.20, 0.95};
    std::vector<double> B = {0.0, 0.30};
    std::vector<double> C = {1.0, 0.0};   // y = x1
    std::vector<double> Q = {1e-3, 0.0, 0.0, 1e-3};
    std::vector<double> R = {1e-2};
    auto estimator = std::make_unique<AuroX::Dynamics::KalmanEstimator>();
    estimator->configure(kPlantOrder, /*obsDim=*/1, kControlDim, A, B, C, Q, R);
    // System identifier runs in the background to re-fit a linear model.
    auto identifier = std::make_unique<AuroX::Dynamics::LinearIdentifier>(kPlantOrder, kControlDim);
    AuroX::Dynamics::Dynamics dyn(&obs, &state, std::move(estimator),
                           std::move(whiteModel), std::move(identifier));
    dyn.useWhiteBox(kPlantOrder, kControlDim, whiteBoxPlant);
    std::cout << "[Dynamics] Kalman observer + white-box plant model online\n";

    // ---- 5.6 Evaluator : default RegressionEvaluator needs data; fall back -
    // No dataset is available in this example, so we use the custom cost fn.
    // (If X/y were provided, we would instead do:
    //    Evaluation::RegressionEvaluator reg(X, y);  // built-in, data-driven
    // )
    CostFunction cost;
    cost.target = {1.0, 0.0};   // step reference of magnitude 1.0
    // dim = 4 matches the ParameterSpace vectorization: [Kp, Ki, Kd, mode].
    // The cost only tunes the first three gains; the enum index is ignored.
    PidCostEvaluator evaluator(cost, /*dim=*/4);
    std::cout << "[Evaluator] using custom cost function (no dataset); dim = "
              << evaluator.dimension() << "\n";

    // ---- 5.7 Optimizer : GradientDescent (full batch) ---------------------
    Optimizer::GradientDescent optimizer(0.05);
    std::cout << "[Optimizer] " << optimizer.name() << " (lr=0.05)\n";

    // ---- 5.8 RunHistory : diagnostics + divergence/convergence guard ------
    Memory::RunHistory history(/*recentCapacity=*/30, /*minimize=*/true);

    // ---- 5.9 OptimizerSession : wire action space between opt and params --
    Optimizer::OptimizerSession session(space, evaluator, optimizer, history,
                                        /*maxIterations=*/500, /*tolerance=*/1e-4);
    session.setActionSpace(&actions);

    // Wire the optimizer's update source into the ActionSpace so the default
    // ParameterUpdateAction is optimizer-driven (per the documented contract).
    actions.setUpdateSource([&optimizer, &space](Space::ParameterSpace&,
                                                 const std::vector<double>& g) {
        return optimizer.computeUpdate(space, g);
    });

    std::cout << "\n>>> Running integrated optimization (max 500 iters) <<<\n";
    auto report = session.run();

    // ---------------------------------------------------------------------
    // 6. Inspect results + exercise the persisted spaces.
    // ---------------------------------------------------------------------
    std::cout << "\n========== Optimization Report ==========\n";
    std::cout << "Iterations     : " << report.iterations << "\n";
    std::cout << "Converged      : " << (report.converged ? "yes" : "no") << "\n";
    std::cout << "Diverged       : " << (report.diverged ? "yes" : "no") << "\n";
    std::cout << "Best objective : " << report.bestObjective << "\n";
    if (report.dimensionMismatch)
        std::cout << "Dimension mismatch between evaluator and parameter space!\n";

    auto best = history.bestParams();
    std::cout << "Best params    : ";
    for (size_t i = 0; i < best.size(); ++i) std::cout << "p" << i << "=" << best[i] << " ";
    std::cout << "\n";

    // Push best params back into the ParameterSpace and read them out by name.
    space.unvectorizeDouble(best);
    if (auto* kp = space.getAs<double>("Kp"))
        std::cout << "  -> Kp = " << kp->value << "\n";
    if (auto* ki = space.getAs<double>("Ki"))
        std::cout << "  -> Ki = " << ki->value << "\n";
    if (auto* kd = space.getAs<double>("Kd"))
        std::cout << "  -> Kd = " << kd->value << "\n";
    if (auto* m = space.getEnum("mode"))
        std::cout << "  -> mode = " << m->label() << "\n";

    // ---------------------------------------------------------------------
    // 7. Drive the Dynamics layer with the tuned gains (one control tick).
    //    This shows the four spaces feeding the plant via the observer.
    // ---------------------------------------------------------------------
    std::cout << "\n>>> One closed-loop tick with tuned gains <<<\n";
    double u = 0.0;
    for (int t = 0; t < 10; ++t) {
        // Simulate a sensor reading from the (white-box) plant.
        auto x = state.hasState() ? state.getStateCopy()
                                  : std::vector<double>{0.0, 0.0};
        double y = x.empty() ? 0.0 : x[0];
        obs.ingest("sensor", y, /*ts=*/t * kTs);
        obs.ingest("mode", "PID", /*ts=*/t * kTs);
        bool ok = dyn.step({u}, /*ts=*/t * kTs);
        if (!ok) { std::cout << "  step failed (obs/state missing)\n"; break; }
        auto xhat = state.getStateCopy();
        // Simple proportional control on the estimated state (tuned Kp).
        double kpVal = space.getAs<double>("Kp") ? space.getAs<double>("Kp")->value : 1.0;
        u = kpVal * (cost.target[0] - (xhat.empty() ? 0.0 : xhat[0]));
        std::cout << "  t=" << t << " y=" << y << " xhat=("
                  << (xhat.empty() ? 0.0 : xhat[0]) << ","
                  << (xhat.size() > 1 ? xhat[1] : 0.0) << ") u=" << u << "\n";
    }

    // ---------------------------------------------------------------------
    // 8. Demonstrate the SystemIdentifier re-fitting a linear model from the
    //    collected (x, u, x') trajectory (background learning loop).
    // ---------------------------------------------------------------------
    std::cout << "\n>>> SystemIdentifier: fit linear model from trajectory <<<\n";
    Dynamics::LinearIdentifier sid(kPlantOrder, kControlDim);
    std::vector<double> x_prev = {0.0, 0.0};
    std::mt19937 rng(42);
    std::normal_distribution<double> noise(0.0, 0.01);
    for (int t = 0; t < 200; ++t) {
        double uu = 0.5 * std::sin(0.1 * t);
        auto xp = whiteBoxPlant(x_prev, {uu});
        xp[0] += noise(rng); xp[1] += noise(rng);   // measurement noise
        sid.addSample(x_prev, {uu}, xp);
        x_prev = xp;
    }
    bool fitOk = sid.fit();
    std::cout << "  identifier fit = " << (fitOk ? "success" : "failed") << "\n";
    if (fitOk && sid.model()) {
        std::cout << "  fitted A = [";
        for (double a : sid.model()->A()) std::cout << a << " ";
        std::cout << "]\n  fitted B = [";
        for (double b : sid.model()->B()) std::cout << b << " ";
        std::cout << "]\n";
        // Adopt the re-identified model into the live Dynamics pipeline.
        dyn.setModel(std::make_unique<Dynamics::LinearTransitionModel>(*sid.model()));
    }

    // ---------------------------------------------------------------------
    // 9. Demonstrate the custom-action / custom-interface fallback path.
    //    If a named handler is not registered, the LoggingCustomAction is the
    //    fallback that runs (here we invoke it explicitly via ActionSpace).
    // ---------------------------------------------------------------------
    std::cout << "\n>>> Custom-interface fallback: apply action stack <<<\n";
    actions.apply(space, /*gradient=*/std::vector<double>(space.vectorizeDouble().size(), 0.0));

    // ---------------------------------------------------------------------
    // 10. Serialize the full pipeline for the Operator Console (front-end).
    // ---------------------------------------------------------------------
    std::cout << "\n>>> Serialized snapshot (front-end friendly) <<<\n";
    nlohmann::json pipeline = nlohmann::json::object();
    pipeline["parameters"] = space.toJson();
    pipeline["observations"] = obs.serialize();
    pipeline["state"] = state.describe();
    pipeline["actions"] = actions.toJson();
    pipeline["history_curve_points"] = history.size();
    std::cout << "  parameters: " << pipeline["parameters"]["parameters"].size()
              << " entries\n";
    std::cout << "  observation channels: " << pipeline["observations"]["observations"].size()
              << "\n";
    std::cout << "  action pipeline types: ";
    for (auto& a : pipeline["actions"]["actions"]) std::cout << a["type"].get<std::string>() << " ";
    std::cout << "\n";

    std::cout << "\nDone. (Custom-action log written to custom_action_log.csv)\n";
    return 0;
}
