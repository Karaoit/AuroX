#pragma once

// ============================================================================
// AuroX - RunHistory  (Layer 2: Data & Memory)
// ----------------------------------------------------------------------------
// Purpose:
//   Lightweight, in-memory run history for the MVP closed-loop. It belongs to
//   the Memory layer (AuroX::Memory), distinct from the Space layer
//   (AuroX::Space = "what can be tuned"). It records the optimizer-friendly
//   parameter vector (std::vector<double>, i.e. ParameterSpace::vectorizeDouble()),
//   NOT the parameter objects themselves — so it stays decoupled from Space.
//
//   This is the minimal "history" the MVP actually needs. It is NOT a
//   persistent episode store (Layer 2 structured log / artifact / similarity
//   index in AuroX-General.md) — that belongs to Phase 3 (experience library
//   and surrogate models). For the MVP we deliberately avoid storing every
//   full unbounded parameter vector, because:
//     1. the optimization loop does not need past vectors to continue;
//     2. unbounded high-dimensional vectors make full-history storage costly;
//     3. the Operator Console only needs the loss curve + current best params.
//
//   What it records:
//     - a cheap scalar record per iteration (iteration / objective / grad norm
//       / status / elapsed) driving the loss curve;
//     - exactly ONE best parameter vector (best_);
//     - a small ring buffer of the last K full parameter vectors (recent_),
//       for convergence / divergence diagnostics.
//
//   External interface (caller-driven, optional persistence):
//     - toJson()  : export a UI-friendly snapshot (curve + best + recent).
//     - fromJson(): rebuild internal state from such a snapshot.
//     - save(dir, name) : write toJson() to <dir>/<name>.json (dir created if
//                         missing). Data stays in memory; this is a one-shot
//                         export the caller triggers, not a storage layer.
//     - load(dir, name) : read a snapshot written by save() back into memory
//                         (enables cross-run warm-start; Phase 3 territory,
//                         provided here for convenience).
//
//   Unbounded parameters & safety:
//     When parameters are unbounded (no parameter_bounds), the safety layer
//     cannot reject via bounds. RunHistory therefore provides diverged(),
//     which detects NaN/Inf or objective blow-up — a runtime guard substitute.
//
// Design notes:
//   - lower objective is treated as better by default (regression-loss
//     convention); pass minimize=false for maximization tasks.
//   - everything lives in RAM; no file/DB writes unless save() is called.
// ============================================================================

#include "json.hpp"

#include <cfloat>
#include <cmath>
#include <deque>
#include <fstream>
#include <filesystem>
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
namespace Memory {

using json = nlohmann::json;

// One recorded iteration. `params` is populated only for best / recent
// records (the full curve keeps it empty to stay cheap).
struct RunRecord {
    size_t iteration = 0;
    double objective = 0.0;      // loss / objective value (lower is better by default)
    double gradientNorm = -1.0;   // -1.0 => not available
    int    status = 0;           // 0 = ok; aligned with Runtime status codes
    double elapsedMs = 0.0;      // wall time spent on this iteration
    std::vector<double> params;  // parameter vector (unvectorized double form)
};

// In-memory run history with an optional JSON export/import interface.
class AUROX_API RunHistory {
public:
    // recentCapacity: max number of full parameter-vector snapshots kept in a
    //   ring buffer (0 = keep none beyond best). This bounds memory for
    //   unbounded / high-dimensional parameter vectors.
    // minimize: true => lower objective is better (regression loss convention).
    explicit RunHistory(size_t recentCapacity = 0, bool minimize = true)
        : recentCapacity_(recentCapacity), minimize_(minimize) {}

    // Record one iteration. `params` may be nullptr if the caller only wants
    // the scalar curve (recent/best will then hold empty param vectors).
    void record(size_t iteration,
                double objective,
                const std::vector<double>* params = nullptr,
                double gradientNorm = -1.0,
                int status = 0,
                double elapsedMs = 0.0);

    // --- Accessors ---
    size_t size() const { return records_.size(); }
    bool empty() const { return records_.empty(); }

    // Full scalar curve (params empty in each record). Drives the loss plot.
    const std::vector<RunRecord>& all() const { return records_; }

    // Single record by index, or nullptr if out of range.
    const RunRecord* at(size_t i) const;

    // Last K full records (with params), for diagnostics.
    const std::deque<RunRecord>& recent() const { return recent_; }

    // Best record so far (by objective). params populated if available.
    bool hasBest() const { return hasBest_; }
    const RunRecord& best() const { return bestRecord_; }
    double bestObjective() const { return hasBest_ ? bestRecord_.objective : (minimize_ ? DBL_MAX : -DBL_MAX); }
    const std::vector<double>& bestParams() const { return bestRecord_.params; }

    // --- Health checks (runtime-guard substitutes for unbounded params) ---
    // True if the best objective over the last `window` iterations improved by
    // less than `tolerance` (i.e. it has flattened out). Needs >= window records.
    bool converged(double tolerance, size_t window = 10) const;

    // True if any of the last `window` iterations has NaN/Inf objective or an
    // objective beyond `maxObjective` (blow-up). Unbounded params cannot be
    // rejected by bounds, so this is the divergence safety net.
    bool diverged(double maxObjective = DBL_MAX, size_t window = 5) const;

    void clear();

    // --- External (caller-driven) interface ---
    // Export a UI-friendly snapshot: curve (scalars) + best + recent.
    json toJson() const;

    // Rebuild internal state from a snapshot produced by toJson().
    void fromJson(const json& j);

    // Write toJson() to <dir>/<name>.json (".json" appended if missing).
    // <dir> is created if it does not exist. Returns true on success.
    // Data remains in memory; this is an optional, one-shot export.
    bool save(const std::string& dir, const std::string& name = "run_history") const;

    // Read a snapshot written by save() back into memory, replacing current
    // contents. Returns true on success.
    bool load(const std::string& dir, const std::string& name = "run_history");

private:
    size_t recentCapacity_;
    bool minimize_;

    std::vector<RunRecord> records_;   // all iterations, scalars only
    std::deque<RunRecord> recent_;     // ring buffer of last K full records
    RunRecord bestRecord_;
    bool hasBest_ = false;
};

}  // namespace Memory
}  // namespace AuroX
