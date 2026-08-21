#include "AuroX/Space/ObservationEncoder.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace AuroX {
namespace Space {

// ----------------------------------------------------------------------------
// ContinuousEncoder
// ----------------------------------------------------------------------------
std::vector<double> ContinuousEncoder::encode(double x) const {
    if (resolution > 1) return encodeHistogram(x);
    double v = 0.0;
    switch (mode) {
        case Mode::MinMax: {
            const double span = max - min;
            const double t = (span > 0.0) ? (x - min) / span : 0.0;
            v = std::clamp(t, 0.0, 1.0);
            break;
        }
        case Mode::ZScore: {
            const double z = (std > 0.0) ? (x - mean) / std : 0.0;
            v = std::clamp(z, clampLo, clampHi);
            break;
        }
        case Mode::None:
        default:
            v = x;
            break;
    }
    return {v};
}

size_t ContinuousEncoder::outputDim() const { return resolution > 1 ? resolution : 1; }

std::vector<double> ContinuousEncoder::encodeHistogram(double x) const {
    std::vector<double> bins(resolution, 0.0);
    const double span = max - min;
    if (span <= 0.0) { bins[0] = 1.0; return bins; }
    const double t = std::clamp((x - min) / span, 0.0, 1.0);
    const double pos = t * static_cast<double>(resolution - 1);
    for (size_t b = 0; b < resolution; ++b) {
        const double d = std::abs(static_cast<double>(b) - pos);
        bins[b] = std::max(0.0, 1.0 - d);   // triangular (soft) binning
    }
    double sum = 0.0;
    for (double w : bins) sum += w;
    if (sum > 0.0) for (double& w : bins) w /= sum;
    return bins;
}

// ----------------------------------------------------------------------------
// DiscreteEncoder
// ----------------------------------------------------------------------------
std::vector<double> DiscreteEncoder::encode(double x) const {
    if (levels.empty()) return {x};   // identity

    long idx = -1;
    for (long i = 0; i < static_cast<long>(levels.size()); ++i) {
        if (std::abs(levels[i] - x) < 1e-9) { idx = i; break; }
    }
    if (idx < 0) {                      // not in set: nearest level
        double best = 1e18; long bi = 0;
        for (long i = 0; i < static_cast<long>(levels.size()); ++i) {
            const double d = std::abs(levels[i] - x);
            if (d < best) { best = d; bi = i; }
        }
        idx = bi;
    }

    if (mode == Mode::Thermometer) {
        std::vector<double> out(levels.size(), 0.0);
        for (long i = 0; i <= idx; ++i) out[i] = 1.0;
        return out;
    }
    const double v = (levels.size() > 1)
        ? static_cast<double>(idx) / static_cast<double>(levels.size() - 1)
        : 0.0;
    return {v};
}

size_t DiscreteEncoder::outputDim() const {
    if (mode == Mode::Thermometer) return levels.empty() ? 1 : levels.size();
    return 1;
}

// ----------------------------------------------------------------------------
// CategoricalEncoder
// ----------------------------------------------------------------------------
std::vector<double> CategoricalEncoder::encode(const std::string& label) const {
    const size_t K = labels.size();
    std::vector<double> out(K + 1, 0.0);   // K + 1 unknown slot
    long idx = -1;
    for (long i = 0; i < static_cast<long>(K); ++i) {
        if (labels[i] == label) { idx = i; break; }
    }
    if (idx < 0) out[K] = 1.0;             // unknown
    else         out[idx] = 1.0;
    return out;
}

size_t CategoricalEncoder::outputDim() const { return labels.empty() ? 1 : labels.size() + 1; }

}  // namespace Space
}  // namespace AuroX
