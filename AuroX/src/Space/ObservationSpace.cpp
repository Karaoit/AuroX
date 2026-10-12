// ============================================================================
// AuroX - ObservationSpace implementation
// ----------------------------------------------------------------------------
// Mirrors include/AuroX/Space/ObservationSpace.hpp one-to-one. The header holds
// declarations only; every implementation lives here (this is NOT header-only,
// and there is no .ipp because nothing here is a template).
//
// One instance owns one mutex; every public entry point locks it, so a single
// ObservationSpace is safe to share between producer(s) and a consumer. This is
// NOT hard real-time and does NOT synchronise physical sensor sampling.
//
// Dependencies: standard library only. No encoder, compensator, channel
// hierarchy, factory or serialization - by design.
// ============================================================================

#include "AuroX/Space/ObservationSpace.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace AuroX {
namespace Space {

namespace {

bool allFinite(const Vec& v) {
    for (double x : v)
        if (!std::isfinite(x)) return false;
    return true;
}

// sampleDim * historySize must fit in std::size_t (both factors are > 0).
bool overflows(std::size_t sampleDim, std::size_t historySize) {
    return sampleDim > std::numeric_limits<std::size_t>::max() / historySize;
}

} // namespace

// -- lookup ------------------------------------------------------------------
ObservationSpace::Channel* ObservationSpace::find(const Str& name) {
    for (Channel& c : channels_)
        if (c.name == name) return &c;
    return nullptr;
}

const ObservationSpace::Channel* ObservationSpace::find(const Str& name) const {
    for (const Channel& c : channels_)
        if (c.name == name) return &c;
    return nullptr;
}

// -- registration ------------------------------------------------------------
void ObservationSpace::add(Str name, std::size_t sampleDim, std::size_t historySize) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (frozen_)            throw std::invalid_argument("ObservationSpace: frozen, cannot add channel");
    if (name.empty())       throw std::invalid_argument("ObservationSpace: empty channel name");
    if (find(name))         throw std::invalid_argument("ObservationSpace: duplicate channel '" + name + "'");
    if (sampleDim == 0)     throw std::invalid_argument("ObservationSpace: sampleDim must be > 0");
    if (historySize == 0)   throw std::invalid_argument("ObservationSpace: historySize must be > 0");
    if (overflows(sampleDim, historySize))
        throw std::invalid_argument("ObservationSpace: sampleDim * historySize overflows");

    Channel ch;
    ch.name        = std::move(name);
    ch.sampleDim   = sampleDim;
    ch.historySize = historySize;
    channels_.push_back(std::move(ch));
}

void ObservationSpace::addCategory(Str name, std::vector<Str> labels, std::size_t historySize) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (frozen_)            throw std::invalid_argument("ObservationSpace: frozen, cannot add channel");
    if (name.empty())       throw std::invalid_argument("ObservationSpace: empty channel name");
    if (find(name))         throw std::invalid_argument("ObservationSpace: duplicate channel '" + name + "'");
    if (labels.empty())     throw std::invalid_argument("ObservationSpace: category needs at least one label");
    if (historySize == 0)   throw std::invalid_argument("ObservationSpace: historySize must be > 0");
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i].empty())
            throw std::invalid_argument("ObservationSpace: empty category label");
        for (std::size_t j = i + 1; j < labels.size(); ++j)
            if (labels[i] == labels[j])
                throw std::invalid_argument("ObservationSpace: duplicate category label '" + labels[i] + "'");
    }
    if (overflows(labels.size(), historySize))
        throw std::invalid_argument("ObservationSpace: sampleDim * historySize overflows");

    Channel ch;
    ch.name        = std::move(name);
    ch.labels      = std::move(labels);
    ch.sampleDim   = ch.labels.size();
    ch.historySize = historySize;
    channels_.push_back(std::move(ch));
}

// -- shared ingest tail ------------------------------------------------------
void ObservationSpace::pushEncoded(Channel& ch, Vec values, std::optional<double> ts) {
    if (values.size() != ch.sampleDim)
        throw std::invalid_argument(
            "ObservationSpace: '" + ch.name + "' expects " +
            std::to_string(ch.sampleDim) + " value(s)");
    if (!allFinite(values))
        throw std::invalid_argument("ObservationSpace: non-finite observation for '" + ch.name + "'");
    if (ts && !std::isfinite(*ts))
        throw std::invalid_argument("ObservationSpace: non-finite timestamp for '" + ch.name + "'");
    if (ts && !ch.samples.empty() && ch.samples.back().timestamp &&
        *ts < *ch.samples.back().timestamp)
        throw std::invalid_argument("ObservationSpace: non-decreasing timestamp required for '" + ch.name + "'");

    ch.samples.push_back(Sample{ std::move(values), ts });
    while (ch.samples.size() > ch.historySize)
        ch.samples.pop_front();                    // drop the OLDEST sample
}

// -- ingest ------------------------------------------------------------------
void ObservationSpace::ingest(const Str& name, double value, std::optional<double> ts) {
    std::lock_guard<std::mutex> lock(mtx_);
    Channel* ch = find(name);
    if (!ch)
        throw std::invalid_argument("ObservationSpace: unknown channel '" + name + "'");
    if (!ch->labels.empty())
        throw std::invalid_argument("ObservationSpace: '" + name + "' is a category channel, use a label");
    pushEncoded(*ch, Vec{ value }, ts);
}

void ObservationSpace::ingest(const Str& name, const Vec& value, std::optional<double> ts) {
    std::lock_guard<std::mutex> lock(mtx_);
    Channel* ch = find(name);
    if (!ch)
        throw std::invalid_argument("ObservationSpace: unknown channel '" + name + "'");
    if (!ch->labels.empty())
        throw std::invalid_argument("ObservationSpace: '" + name + "' is a category channel, use a label");
    pushEncoded(*ch, value, ts);
}

void ObservationSpace::ingest(const Str& name, const Str& label, std::optional<double> ts) {
    std::lock_guard<std::mutex> lock(mtx_);
    Channel* ch = find(name);
    if (!ch)
        throw std::invalid_argument("ObservationSpace: unknown channel '" + name + "'");
    if (ch->labels.empty())
        throw std::invalid_argument("ObservationSpace: '" + name + "' is a numeric channel, use a number or Vec");
    for (std::size_t i = 0; i < ch->labels.size(); ++i) {
        if (ch->labels[i] == label) {
            Vec onehot(ch->labels.size(), 0.0);
            onehot[i] = 1.0;
            pushEncoded(*ch, std::move(onehot), ts);
            return;
        }
    }
    throw std::invalid_argument("ObservationSpace: unknown label '" + label + "' for '" + name + "'");
}

// -- read --------------------------------------------------------------------
std::optional<Vec> ObservationSpace::latest(const Str& name) const {
    std::lock_guard<std::mutex> lock(mtx_);
    const Channel* ch = find(name);
    if (!ch)
        throw std::invalid_argument("ObservationSpace: unknown channel '" + name + "'");
    if (ch->samples.empty()) return std::nullopt;   // never fake a zero sample
    return ch->samples.back().values;
}

ObservationSpace::Bundle ObservationSpace::collect() const {
    std::lock_guard<std::mutex> lock(mtx_);
    Bundle out;

    for (const Channel& ch : channels_) {
        ChannelSlice slice;
        slice.name          = ch.name;
        slice.offset        = out.vector.size();
        slice.length        = ch.sampleDim * ch.historySize;
        slice.sampleDim     = ch.sampleDim;
        slice.validCount    = ch.samples.size();
        slice.lastTimestamp = ch.samples.empty() ? std::nullopt : ch.samples.back().timestamp;

        // Left zero-pad the unfilled slots; the newest sample stays right-most.
        const std::size_t missing = ch.historySize - ch.samples.size();
        out.vector.insert(out.vector.end(), missing * ch.sampleDim, 0.0);
        for (const Sample& s : ch.samples)
            out.vector.insert(out.vector.end(), s.values.begin(), s.values.end());

        out.layout.push_back(std::move(slice));
    }
    return out;
}

// -- lifecycle ---------------------------------------------------------------
void ObservationSpace::freeze() {
    std::lock_guard<std::mutex> lock(mtx_);
    frozen_ = true;                                 // structure locked, ingest still allowed
}

void ObservationSpace::reset() {
    std::lock_guard<std::mutex> lock(mtx_);
    for (Channel& ch : channels_)
        ch.samples.clear();                         // samples only; keep channels + frozen_
}

} // namespace Space
} // namespace AuroX
