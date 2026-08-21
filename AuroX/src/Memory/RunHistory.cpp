#include "AuroX/Memory/RunHistory.hpp"

#include "json.hpp"

#include <limits>

namespace AuroX {
namespace Memory {

void RunHistory::record(size_t iteration,
                        double objective,
                        const std::vector<double>* params,
                        double gradientNorm,
                        int status,
                        double elapsedMs) {
    RunRecord rec;
    rec.iteration = iteration;
    rec.objective = objective;
    rec.gradientNorm = gradientNorm;
    rec.status = status;
    rec.elapsedMs = elapsedMs;
    // Scalars-only record for the full curve (keeps memory bounded).
    records_.push_back(rec);

    // Ring buffer of recent full records (params kept).
    if (recentCapacity_ > 0) {
        RunRecord full = rec;
        if (params) full.params = *params;
        recent_.push_back(std::move(full));
        while (recent_.size() > recentCapacity_) recent_.pop_front();
    }

    // Best tracking. Lower objective is better unless minimize_ is false.
    const bool better = !hasBest_ || (minimize_ ? (objective < bestRecord_.objective)
                                                : (objective > bestRecord_.objective));
    if (better) {
        bestRecord_ = rec;
        if (params) bestRecord_.params = *params;
        hasBest_ = true;
    }
}

const RunRecord* RunHistory::at(size_t i) const {
    return i < records_.size() ? &records_[i] : nullptr;
}

bool RunHistory::converged(double tolerance, size_t window) const {
    if (records_.size() < window) return false;
    const size_t start = records_.size() - window;
    double lo = records_[start].objective;
    double hi = records_[start].objective;
    for (size_t i = start + 1; i < records_.size(); ++i) {
        lo = std::min(lo, records_[i].objective);
        hi = std::max(hi, records_[i].objective);
    }
    return (hi - lo) < tolerance;
}

bool RunHistory::diverged(double maxObjective, size_t window) const {
    if (records_.empty()) return false;
    const size_t start = (records_.size() > window) ? records_.size() - window : 0;
    for (size_t i = start; i < records_.size(); ++i) {
        const double o = records_[i].objective;
        if (!std::isfinite(o)) return true;            // NaN / Inf
        if (o > maxObjective) return true;             // objective blow-up
    }
    return false;
}

void RunHistory::clear() {
    records_.clear();
    recent_.clear();
    bestRecord_ = RunRecord{};
    hasBest_ = false;
}

json RunHistory::toJson() const {
    json curve = json::array();
    for (const auto& r : records_) {
        json e = json::object();
        e["iteration"] = r.iteration;
        e["objective"] = r.objective;
        e["gradient_norm"] = r.gradientNorm;
        e["status"] = r.status;
        e["elapsed_ms"] = r.elapsedMs;
        curve.push_back(std::move(e));
    }

    json best = json::object();
    best["iteration"] = bestRecord_.iteration;
    best["objective"] = bestRecord_.objective;
    best["gradient_norm"] = bestRecord_.gradientNorm;
    best["status"] = bestRecord_.status;
    best["params"] = bestRecord_.params;

    json recent = json::array();
    for (const auto& r : recent_) {
        json e = json::object();
        e["iteration"] = r.iteration;
        e["objective"] = r.objective;
        e["params"] = r.params;
        recent.push_back(std::move(e));
    }

    json out = json::object();
    out["size"] = records_.size();
    out["minimize"] = minimize_;
    out["recent_capacity"] = recentCapacity_;
    out["best"] = std::move(best);
    out["recent"] = std::move(recent);
    out["curve"] = std::move(curve);
    return out;
}

void RunHistory::fromJson(const json& j) {
    clear();
    minimize_ = j.value("minimize", true);
    recentCapacity_ = j.value("recent_capacity", static_cast<size_t>(0));

    if (j.contains("curve") && j["curve"].is_array()) {
        for (const auto& e : j["curve"]) {
            RunRecord r;
            r.iteration = e.value("iteration", static_cast<size_t>(0));
            r.objective = e.value("objective", 0.0);
            r.gradientNorm = e.value("gradient_norm", -1.0);
            r.status = e.value("status", 0);
            r.elapsedMs = e.value("elapsed_ms", 0.0);
            records_.push_back(std::move(r));
        }
    }

    if (j.contains("best") && j["best"].is_object()) {
        const auto& b = j["best"];
        bestRecord_.iteration = b.value("iteration", static_cast<size_t>(0));
        bestRecord_.objective = b.value("objective", 0.0);
        bestRecord_.gradientNorm = b.value("gradient_norm", -1.0);
        bestRecord_.status = b.value("status", 0);
        if (b.contains("params") && b["params"].is_array())
            bestRecord_.params = b["params"].get<std::vector<double>>();
        hasBest_ = true;
    }

    if (j.contains("recent") && j["recent"].is_array()) {
        for (const auto& e : j["recent"]) {
            RunRecord r;
            r.iteration = e.value("iteration", static_cast<size_t>(0));
            r.objective = e.value("objective", 0.0);
            if (e.contains("params") && e["params"].is_array())
                r.params = e["params"].get<std::vector<double>>();
            recent_.push_back(std::move(r));
        }
    }
}

bool RunHistory::save(const std::string& dir, const std::string& name) const {
    try {
        std::filesystem::path outDir(dir);
        std::error_code ec;
        std::filesystem::create_directories(outDir, ec);
        if (ec) return false;

        std::filesystem::path file = outDir / (name.empty() ? std::string("run_history") : name);
        if (file.extension().empty()) file += ".json";

        std::ofstream ofs(file, std::ios::binary);
        if (!ofs) return false;
        ofs << toJson().dump(2);
        ofs.flush();
        return static_cast<bool>(ofs);
    } catch (...) {
        return false;
    }
}

bool RunHistory::load(const std::string& dir, const std::string& name) {
    try {
        std::filesystem::path outDir(dir);
        std::filesystem::path file = outDir / (name.empty() ? std::string("run_history") : name);
        if (file.extension().empty()) file += ".json";

        std::ifstream ifs(file, std::ios::binary);
        if (!ifs) return false;
        json j;
        ifs >> j;
        if (ifs.fail() && !ifs.eof()) return false;
        if (!j.is_object()) return false;
        fromJson(j);
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace Memory
}  // namespace AuroX
