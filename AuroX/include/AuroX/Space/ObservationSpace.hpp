#pragma once

// ============================================================================
// AuroX - ObservationSpace  (part of the Space layer)
// ----------------------------------------------------------------------------
// Purpose:
//   Capture EVERY quantifiable external input, vectorize it, hold the vectors in
//   a per-channel FIFO (with an observation time-width), and hand the current
//   window to the StateSpace on demand. This is the "sensory" half that feeds
//   the system identifier / estimator discussed in the design docs.
//
// This file is split from ObservationEncoder.hpp by responsibility:
//   - ObservationEncoder.hpp : stateless per-sample encoders (pure vectorize).
//   - ObservationSpace.hpp    : stateful channels (FIFO + serialize) + container.
//
// MVP behavior (per the agreed design):
//   - Each channel binds ONE external input and vectorizes it directly.
//   - Vector length = resolution (observation resolution) x timeWidth
//     (observation time width); stored first-in-first-out, oldest dropped.
//   - requestVector() returns the current window ANY time the StateSpace asks;
//     unfilled positions are ZERO-filled.
//   - Channels are serializable / deserializable (metadata + FIFO window).
//   - An optional feed-forward compensator (user-implemented) is applied
//     post-encode / pre-output on the whole window.
//
// Declarations only; method definitions live in ObservationSpace.cpp.
// ============================================================================

#include "AuroX/Space/ObservationEncoder.hpp"
#include "json.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace AuroX {
namespace Space {

using json = nlohmann::json;

// ----------------------------------------------------------------------------
// ObservationCompensator
//   Optional feed-forward compensation applied to the full r*T window just
//   before it is handed to the StateSpace. USER-IMPLEMENTED: derive from this
//   and override apply(). The default NoOpCompensator does nothing.
//   Typical uses: subtract a known bias, de-trend, cancel a known command's
//   effect (e.g. remove commanded velocity from visual-odometry drift).
//   NOTE: it is applied on EVERY pull; stateful compensators must be idempotent
//   or designed to tolerate re-application.
// ----------------------------------------------------------------------------
class ObservationCompensator {
public:
    virtual ~ObservationCompensator() = default;
    virtual void apply(std::vector<double>& window) = 0;
    virtual std::string name() const;
};

class NoOpCompensator : public ObservationCompensator {
public:
    void apply(std::vector<double>&) override {}
    std::string name() const override;
};

// ----------------------------------------------------------------------------
// ObservationChannel (base)
//   Holds id / name / resolution / timeWidth, a FIFO of per-sample vectors, and
//   an optional compensator. Concrete channels only implement the ingest entry
//   points relevant to their type plus (de)serialization / describe.
// ----------------------------------------------------------------------------
class ObservationChannel {
public:
    ObservationChannel(std::string id, std::string name, size_t resolution, size_t timeWidth);
    virtual ~ObservationChannel() = default;

    // Identity / geometry
    const std::string& id() const;
    std::string name() const;
    size_t resolution() const;
    size_t timeWidth() const;
    size_t vectorLength() const;

    // Ingest entry points: only the relevant one is overridden by a concrete
    // channel. Calling the wrong one is a safe no-op.
    virtual void ingestDouble(double /*value*/, double /*ts*/) {}
    virtual void ingestString(const std::string& /*label*/, double /*ts*/) {}
    virtual void ingestVector(const std::vector<double>& /*vec*/, double /*ts*/) {}

    // Current flattened FIFO window (front = oldest), zero-padded; compensator
    // applied before returning.
    std::vector<double> currentVector() const;

    size_t fillCount() const;
    bool isFull() const;

    void reset();

    void setCompensator(std::unique_ptr<ObservationCompensator> c);
    const ObservationCompensator* compensator() const;

    // (De)serialization + description (for persistence and front-end rendering).
    virtual json serialize() const = 0;
    virtual void deserialize(const json& j) = 0;
    virtual json describe() const = 0;

protected:
    void pushSample(std::vector<double> sample, double ts);
    std::vector<double> buildVector() const;
    std::vector<std::vector<double>> snapshotWindow() const;
    void restoreWindow(const std::vector<std::vector<double>>& w);

    std::string id_;
    std::string name_;
    size_t resolution_;
    size_t timeWidth_;
    std::deque<std::vector<double>> window_;   // FIFO of per-sample vectors
    mutable std::mutex mtx_;
    double lastTs_ = 0.0;
    std::unique_ptr<ObservationCompensator> compensator_ = std::make_unique<NoOpCompensator>();
};

// ----------------------------------------------------------------------------
// ContinuousObservationChannel
// ----------------------------------------------------------------------------
class ContinuousObservationChannel : public ObservationChannel {
public:
    ContinuousEncoder encoder;

    ContinuousObservationChannel(std::string id, std::string name, size_t timeWidth,
                                ContinuousEncoder enc = {});

    void ingestDouble(double value, double ts) override;

    json serialize() const override;
    void deserialize(const json& j) override;
    json describe() const override;

private:
    static std::string modeName(ContinuousEncoder::Mode m);
    void restoreMeta(const json& j);
};

// ----------------------------------------------------------------------------
// DiscreteObservationChannel
// ----------------------------------------------------------------------------
class DiscreteObservationChannel : public ObservationChannel {
public:
    DiscreteEncoder encoder;

    DiscreteObservationChannel(std::string id, std::string name, size_t timeWidth,
                               DiscreteEncoder enc = {});

    void ingestDouble(double value, double ts) override;

    json serialize() const override;
    void deserialize(const json& j) override;
    json describe() const override;

private:
    void restoreMeta(const json& j);
};

// ----------------------------------------------------------------------------
// CategoricalObservationChannel
// ----------------------------------------------------------------------------
class CategoricalObservationChannel : public ObservationChannel {
public:
    CategoricalEncoder encoder;

    CategoricalObservationChannel(std::string id, std::string name,
                                 std::vector<std::string> labels, size_t timeWidth);

    void ingestString(const std::string& label, double ts) override;

    json serialize() const override;
    void deserialize(const json& j) override;
    json describe() const override;

private:
    void restoreMeta(const json& j);
};

// ----------------------------------------------------------------------------
// RawVectorObservationChannel
//   Binds an input that is ALREADY a vector (e.g. an embedding, an encoder
//   feature). No re-encoding; the fixed `resolution` (expected vector length)
//   must be supplied at construction.
// ----------------------------------------------------------------------------
class RawVectorObservationChannel : public ObservationChannel {
public:
    RawVectorObservationChannel(std::string id, std::string name, size_t vectorLength, size_t timeWidth);

    void ingestVector(const std::vector<double>& vec, double ts) override;

    json serialize() const override;
    void deserialize(const json& j) override;
    json describe() const override;
};

// ----------------------------------------------------------------------------
// ObservationSpace (container)
//   Registers channels, routes typed ingest calls to the right channel, and
//   lets the StateSpace pull the current window of any channel at any time.
//   collect() concatenates every channel with a per-channel layout (offset /
//   length) so the Observer can slice the combined vector back apart.
// ----------------------------------------------------------------------------
class ObservationSpace {
public:
    struct ChannelSlice {
        std::string id;
        size_t offset = 0;
        size_t length = 0;
    };
    struct Bundle {
        std::vector<double> vector;
        std::vector<ChannelSlice> layout;
    };

    // Take ownership of a fully-constructed channel.
    void addChannel(std::unique_ptr<ObservationChannel> ch);

    // Typed ingest routing (safe no-op if the channel does not accept the type).
    void ingest(const std::string& id, double v, double ts = 0.0);
    void ingest(const std::string& id, const std::string& v, double ts = 0.0);
    void ingest(const std::string& id, const std::vector<double>& v, double ts = 0.0);

    // Pull the current window of one channel (zero-padded if not full).
    std::vector<double> requestVector(const std::string& id) const;

    // Concatenate all channels with layout (offset/length per channel).
    Bundle collect() const;

    ObservationChannel* get(const std::string& id);
    const ObservationChannel* get(const std::string& id) const;

    size_t size() const;
    bool empty() const;
    void clear();
    void reset();

    json describe() const;
    json serialize() const;
    void deserialize(const json& j);

private:
    std::vector<std::unique_ptr<ObservationChannel>> channels_;
    std::unordered_map<std::string, size_t> index_;

    ObservationChannel* find(const std::string& id);
    const ObservationChannel* find(const std::string& id) const;
};

}  // namespace Space
}  // namespace AuroX
