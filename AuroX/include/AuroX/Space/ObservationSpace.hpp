#pragma once

// ============================================================================
// AuroX - ObservationSpace
// ----------------------------------------------------------------------------
// Design rule: add no entity unless it is necessary.
//
// ONE class only. There is no channel inheritance family, no encoder/
// compensator hierarchy, no factory, no plugin layer, no JSON serialization and
// no per-channel concurrency manager.
//
//   1 class    ObservationSpace
//   2 nested   ChannelSlice / Bundle   (result carriers, NOT system objects)
//   0 free fns, 0 macros, standard library only
//
// ObservationSpace is the FINITE-DIMENSIONAL EXTERNAL-OBSERVATION INLET: it
// receives, validates, keeps a short recent history and exposes a stable
// numeric view. It is NOT a sensor driver, NOT a controller and NOT a state
// estimator. It NEVER writes a StateSpace; external task code decides whether
// an observation (or an estimate) becomes the current state.
//
//   external data (CSV / sim / sensor / PLC / perception)
//        -> ObservationSpace : add/ingest -> recent N -> collect()
//        -> model training  |  external estimation -> StateSpace -> Planner
//
// File layout (declaration / implementation split; non-template -> no .ipp):
//   include/AuroX/Space/ObservationSpace.hpp  - declarations + comments (this)
//   src/Space/ObservationSpace.cpp            - all implementations
//
// The three usage tiers are the SAME API, only progressively exercised:
//   simple   : add + ingest + latest                          (no history/ts)
//   standard : addCategory + historySize + collect + layout
//   advanced : timestamped ingest + validCount + freshness checks
//
// Contract notes:
//   * historySize is a SAMPLE COUNT, never a time span. Windows are filled
//     left-to-right with zeros; the newest sample is always right-most.
//   * validCount counts REALLY received samples (a genuine value 0 counts too).
//     Never infer validity from "is the window all zeros".
//   * ts is std::optional<double>: absent != t == 0.0. Timestamps within a
//     channel must be non-decreasing; lastTimestamp only reflects the newest
//     sample (nullopt when the newest sample carries no timestamp).
//   * Every invalid input is rejected explicitly (std::invalid_argument):
//     unknown channel, wrong type/length, unknown label, NaN/Inf. There is NO
//     silent no-op path.
//   * The bundled mutex protects data operations and collect() snapshots on the
//     SAME instance. It does NOT promise hard real-time behaviour or physical
//     cross-sensor sample synchronisation.
// ============================================================================

#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

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

using Str = std::string;
using Vec = std::vector<double>;

class AUROX_API ObservationSpace {
public:
    // A result carrier describing one channel inside a Bundle. Not a system
    // object: it holds no data of its own.
    struct ChannelSlice {
        Str                   name;           // channel name
        std::size_t           offset = 0;     // start of this channel in Bundle::vector
        std::size_t           length = 0;     // sampleDim * historySize
        std::size_t           sampleDim = 0;  // numeric dimension of ONE encoded sample
        std::size_t           validCount = 0; // really received samples (<= historySize)
        std::optional<double> lastTimestamp;  // timestamp of the NEWEST sample, or nullopt
    };

    // A consistent snapshot: the vector and the layout come from the SAME read.
    struct Bundle {
        Vec                       vector;
        std::vector<ChannelSlice> layout;
    };

    ObservationSpace() = default;

    // A single instance owns a mutex -> it is neither copyable nor movable.
    ObservationSpace(const ObservationSpace&)            = delete;
    ObservationSpace& operator=(const ObservationSpace&) = delete;

    // -- registration (locked after freeze()) --------------------------------
    // Numeric channel. sampleDim > 0; historySize (samples) > 0.
    void add(Str name, std::size_t sampleDim = 1, std::size_t historySize = 1);

    // Category channel: values are one-hot over `labels`, in the given order.
    // Labels must be non-empty and unique; sampleDim == labels.size().
    void addCategory(Str name, std::vector<Str> labels, std::size_t historySize = 1);

    // -- ingest (strictly validated; never a silent no-op) --------------------
    // Scalar: a numeric 1-dim channel. Bool/switch readings use 0/1.
    void ingest(const Str& name, double value,
                std::optional<double> ts = std::nullopt);
    // Fixed-length vector: length must equal the declared sampleDim.
    void ingest(const Str& name, const Vec& value,
                std::optional<double> ts = std::nullopt);
    // Category label: must be one of the registered labels.
    void ingest(const Str& name, const Str& label,
                std::optional<double> ts = std::nullopt);

    // -- read ----------------------------------------------------------------
    // The newest encoded sample of a channel, or nullopt when nothing was
    // received. Unknown channel throws.
    std::optional<Vec> latest(const Str& name) const;

    // A consistent snapshot of every channel + its layout.
    Bundle collect() const;

    // -- lifecycle -----------------------------------------------------------
    void freeze();  // lock the channel STRUCTURE; ingest keeps working
    void reset();   // clear SAMPLES only: keep channels, keep the frozen state

private:
    struct Sample {
        Vec                   values;     // one already-encoded, finite sample
        std::optional<double> timestamp;  // unit/clock is a caller convention
    };
    struct Channel {
        Str                name;
        std::size_t        sampleDim = 1;
        std::size_t        historySize = 1;
        std::vector<Str>   labels;        // empty = numeric channel, else one-hot
        std::deque<Sample> samples;       // at most historySize, oldest -> newest
    };

    Channel*       find(const Str& name);
    const Channel* find(const Str& name) const;

    // Shared tail: validate values/ts, push, then trim to historySize.
    void pushEncoded(Channel& ch, Vec values, std::optional<double> ts);

    mutable std::mutex   mtx_;
    std::vector<Channel> channels_;
    bool                 frozen_ = false;
};

using OSpace = ObservationSpace;  // convenience alias

} // namespace Space
} // namespace AuroX
