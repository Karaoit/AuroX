#include "AuroX/Space/ObservationSpace.hpp"

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace AuroX {
namespace Space {

// ----------------------------------------------------------------------------
// ObservationCompensator
// ----------------------------------------------------------------------------
std::string ObservationCompensator::name() const { return "compensator"; }

std::string NoOpCompensator::name() const { return "noop"; }

// ----------------------------------------------------------------------------
// ObservationChannel (base)
// ----------------------------------------------------------------------------
ObservationChannel::ObservationChannel(std::string id, std::string name,
                                       size_t resolution, size_t timeWidth)
    : id_(std::move(id)), name_(std::move(name)), resolution_(resolution), timeWidth_(timeWidth) {}

const std::string& ObservationChannel::id() const { return id_; }
std::string ObservationChannel::name() const { return name_; }
size_t ObservationChannel::resolution() const { return resolution_; }
size_t ObservationChannel::timeWidth() const { return timeWidth_; }
size_t ObservationChannel::vectorLength() const { return resolution_ * timeWidth_; }

std::vector<double> ObservationChannel::currentVector() const {
    std::vector<double> v = buildVector();
    if (compensator_) compensator_->apply(v);
    return v;
}

size_t ObservationChannel::fillCount() const { std::lock_guard<std::mutex> lk(mtx_); return window_.size(); }
bool ObservationChannel::isFull() const { std::lock_guard<std::mutex> lk(mtx_); return window_.size() >= timeWidth_; }

void ObservationChannel::reset() { std::lock_guard<std::mutex> lk(mtx_); window_.clear(); }

void ObservationChannel::setCompensator(std::unique_ptr<ObservationCompensator> c) {
    if (c) compensator_ = std::move(c);
}
const ObservationCompensator* ObservationChannel::compensator() const { return compensator_.get(); }

void ObservationChannel::pushSample(std::vector<double> sample, double ts) {
    std::lock_guard<std::mutex> lk(mtx_);
    window_.push_back(std::move(sample));
    while (window_.size() > timeWidth_) window_.pop_front();
    lastTs_ = ts;
}
std::vector<double> ObservationChannel::buildVector() const {
    std::lock_guard<std::mutex> lk(mtx_);
    std::vector<double> out(vectorLength(), 0.0);
    size_t off = 0;
    for (const auto& s : window_)
        for (double v : s) out[off++] = v;
    return out;
}
std::vector<std::vector<double>> ObservationChannel::snapshotWindow() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return {window_.begin(), window_.end()};
}
void ObservationChannel::restoreWindow(const std::vector<std::vector<double>>& w) {
    std::lock_guard<std::mutex> lk(mtx_);
    window_.assign(w.begin(), w.end());
}

// ----------------------------------------------------------------------------
// ContinuousObservationChannel
// ----------------------------------------------------------------------------
ContinuousObservationChannel::ContinuousObservationChannel(std::string id, std::string name,
                                                         size_t timeWidth, ContinuousEncoder enc)
    : ObservationChannel(std::move(id), std::move(name), enc.outputDim(), timeWidth),
      encoder(std::move(enc)) {}

void ContinuousObservationChannel::ingestDouble(double value, double ts) {
    pushSample(encoder.encode(value), ts);
}

json ContinuousObservationChannel::serialize() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "continuous";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount();
    j["encoder"] = {
        {"mode", modeName(encoder.mode)},
        {"min", encoder.min}, {"max", encoder.max},
        {"mean", encoder.mean}, {"std", encoder.std},
        {"resolution", encoder.resolution}
    };
    j["window"] = snapshotWindow();
    return j;
}
void ContinuousObservationChannel::deserialize(const json& j) {
    restoreMeta(j);
    if (j.contains("encoder")) {
        const auto& e = j["encoder"];
        const std::string m = e.value("mode", "minmax");
        encoder.mode = (m == "zscore") ? ContinuousEncoder::Mode::ZScore
                     : (m == "none")   ? ContinuousEncoder::Mode::None
                                       : ContinuousEncoder::Mode::MinMax;
        if (e.contains("min")) encoder.min = e["min"];
        if (e.contains("max")) encoder.max = e["max"];
        if (e.contains("mean")) encoder.mean = e["mean"];
        if (e.contains("std")) encoder.std = e["std"];
        if (e.contains("resolution")) encoder.resolution = e["resolution"];
    }
    if (j.contains("window") && j["window"].is_array()) {
        std::vector<std::vector<double>> w;
        for (const auto& s : j["window"]) w.push_back(s.get<std::vector<double>>());
        restoreWindow(w);
    }
}
json ContinuousObservationChannel::describe() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "continuous";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount(); j["isFull"] = isFull();
    j["compensator"] = compensator_ ? compensator_->name() : "none";
    return j;
}

std::string ContinuousObservationChannel::modeName(ContinuousEncoder::Mode m) {
    if (m == ContinuousEncoder::Mode::ZScore) return "zscore";
    if (m == ContinuousEncoder::Mode::None)    return "none";
    return "minmax";
}
void ContinuousObservationChannel::restoreMeta(const json& j) {
    if (j.contains("id"))   id_ = j["id"].get<std::string>();
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("resolution")) resolution_ = j["resolution"].get<size_t>();
    if (j.contains("timeWidth"))  timeWidth_ = j["timeWidth"].get<size_t>();
}

// ----------------------------------------------------------------------------
// DiscreteObservationChannel
// ----------------------------------------------------------------------------
DiscreteObservationChannel::DiscreteObservationChannel(std::string id, std::string name,
                                                     size_t timeWidth, DiscreteEncoder enc)
    : ObservationChannel(std::move(id), std::move(name), enc.outputDim(), timeWidth),
      encoder(std::move(enc)) {}

void DiscreteObservationChannel::ingestDouble(double value, double ts) {
    pushSample(encoder.encode(value), ts);
}

json DiscreteObservationChannel::serialize() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "discrete";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount();
    json elab = json::array();
    for (double v : encoder.levels) elab.push_back(v);
    j["encoder"] = {
        {"mode", encoder.mode == DiscreteEncoder::Mode::Thermometer ? "thermometer" : "scalar"},
        {"levels", elab}
    };
    j["window"] = snapshotWindow();
    return j;
}
void DiscreteObservationChannel::deserialize(const json& j) {
    restoreMeta(j);
    if (j.contains("encoder")) {
        const auto& e = j["encoder"];
        const std::string m = e.value("mode", "scalar");
        encoder.mode = (m == "thermometer") ? DiscreteEncoder::Mode::Thermometer
                                           : DiscreteEncoder::Mode::Scalar;
        if (e.contains("levels") && e["levels"].is_array())
            encoder.levels = e["levels"].get<std::vector<double>>();
    }
    if (j.contains("window") && j["window"].is_array()) {
        std::vector<std::vector<double>> w;
        for (const auto& s : j["window"]) w.push_back(s.get<std::vector<double>>());
        restoreWindow(w);
    }
}
json DiscreteObservationChannel::describe() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "discrete";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount(); j["isFull"] = isFull();
    j["compensator"] = compensator_ ? compensator_->name() : "none";
    return j;
}

void DiscreteObservationChannel::restoreMeta(const json& j) {
    if (j.contains("id"))   id_ = j["id"].get<std::string>();
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("resolution")) resolution_ = j["resolution"].get<size_t>();
    if (j.contains("timeWidth"))  timeWidth_ = j["timeWidth"].get<size_t>();
}

// ----------------------------------------------------------------------------
// CategoricalObservationChannel
// ----------------------------------------------------------------------------
CategoricalObservationChannel::CategoricalObservationChannel(std::string id, std::string name,
                                                           std::vector<std::string> labels,
                                                           size_t timeWidth)
    : ObservationChannel(std::move(id), std::move(name),
                         labels.empty() ? 1 : labels.size() + 1,
                         timeWidth) {
    encoder.labels = std::move(labels);
    resolution_ = encoder.outputDim();
}

void CategoricalObservationChannel::ingestString(const std::string& label, double ts) {
    pushSample(encoder.encode(label), ts);
}

json CategoricalObservationChannel::serialize() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "categorical";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount();
    j["encoder"] = {{"labels", encoder.labels}};
    j["window"] = snapshotWindow();
    return j;
}
void CategoricalObservationChannel::deserialize(const json& j) {
    restoreMeta(j);
    if (j.contains("encoder") && j["encoder"].contains("labels"))
        encoder.labels = j["encoder"]["labels"].get<std::vector<std::string>>();
    resolution_ = encoder.outputDim();
    if (j.contains("window") && j["window"].is_array()) {
        std::vector<std::vector<double>> w;
        for (const auto& s : j["window"]) w.push_back(s.get<std::vector<double>>());
        restoreWindow(w);
    }
}
json CategoricalObservationChannel::describe() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "categorical";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount(); j["isFull"] = isFull();
    j["numLabels"] = encoder.labels.size();
    j["compensator"] = compensator_ ? compensator_->name() : "none";
    return j;
}

void CategoricalObservationChannel::restoreMeta(const json& j) {
    if (j.contains("id"))   id_ = j["id"].get<std::string>();
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("timeWidth"))  timeWidth_ = j["timeWidth"].get<size_t>();
    // resolution is derived from label count on deserialize
}

// ----------------------------------------------------------------------------
// RawVectorObservationChannel
// ----------------------------------------------------------------------------
RawVectorObservationChannel::RawVectorObservationChannel(std::string id, std::string name,
                                                       size_t vectorLength, size_t timeWidth)
    : ObservationChannel(std::move(id), std::move(name), vectorLength, timeWidth) {}

void RawVectorObservationChannel::ingestVector(const std::vector<double>& vec, double ts) {
    if (vec.size() == resolution_) pushSample(vec, ts);
    // mismatched length is silently dropped (mismatch indicates a wiring bug)
}

json RawVectorObservationChannel::serialize() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "raw_vector";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount();
    j["window"] = snapshotWindow();
    return j;
}
void RawVectorObservationChannel::deserialize(const json& j) {
    if (j.contains("id"))   id_ = j["id"].get<std::string>();
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("resolution")) resolution_ = j["resolution"].get<size_t>();
    if (j.contains("timeWidth"))  timeWidth_ = j["timeWidth"].get<size_t>();
    if (j.contains("window") && j["window"].is_array()) {
        std::vector<std::vector<double>> w;
        for (const auto& s : j["window"]) w.push_back(s.get<std::vector<double>>());
        restoreWindow(w);
    }
}
json RawVectorObservationChannel::describe() const {
    json j = json::object();
    j["id"] = id_; j["name"] = name_; j["kind"] = "raw_vector";
    j["resolution"] = resolution_; j["timeWidth"] = timeWidth_;
    j["fillCount"] = fillCount(); j["isFull"] = isFull();
    j["compensator"] = compensator_ ? compensator_->name() : "none";
    return j;
}

// ----------------------------------------------------------------------------
// ObservationSpace (container)
// ----------------------------------------------------------------------------
void ObservationSpace::addChannel(std::unique_ptr<ObservationChannel> ch) {
    if (!ch) return;
    index_[ch->id()] = channels_.size();
    channels_.push_back(std::move(ch));
}

void ObservationSpace::ingest(const std::string& id, double v, double ts) {
    if (auto* c = find(id)) c->ingestDouble(v, ts);
}
void ObservationSpace::ingest(const std::string& id, const std::string& v, double ts) {
    if (auto* c = find(id)) c->ingestString(v, ts);
}
void ObservationSpace::ingest(const std::string& id, const std::vector<double>& v, double ts) {
    if (auto* c = find(id)) c->ingestVector(v, ts);
}

std::vector<double> ObservationSpace::requestVector(const std::string& id) const {
    if (auto* c = find(id)) return c->currentVector();
    return {};
}

ObservationSpace::Bundle ObservationSpace::collect() const {
    Bundle b;
    size_t off = 0;
    for (const auto& c : channels_) {
        std::vector<double> v = c->currentVector();
        b.layout.push_back({c->id(), off, v.size()});
        b.vector.insert(b.vector.end(), v.begin(), v.end());
        off += v.size();
    }
    return b;
}

ObservationChannel* ObservationSpace::get(const std::string& id) { return find(id); }
const ObservationChannel* ObservationSpace::get(const std::string& id) const { return find(id); }

size_t ObservationSpace::size() const { return channels_.size(); }
bool ObservationSpace::empty() const { return channels_.empty(); }
void ObservationSpace::clear() { channels_.clear(); index_.clear(); }
void ObservationSpace::reset() { for (auto& c : channels_) c->reset(); }

json ObservationSpace::describe() const {
    json arr = json::array();
    for (const auto& c : channels_) arr.push_back(c->describe());
    return json{{"channels", arr}};
}
json ObservationSpace::serialize() const {
    json arr = json::array();
    for (const auto& c : channels_) arr.push_back(c->serialize());
    return json{{"observations", arr}};
}
void ObservationSpace::deserialize(const json& j) {
    clear();
    if (!j.contains("observations") || !j["observations"].is_array()) return;
    for (const auto& cj : j["observations"]) {
        if (!cj.is_object()) continue;
        const std::string kind = cj.value("kind", "");
        std::unique_ptr<ObservationChannel> ch;
        if (kind == "continuous")       ch = std::make_unique<ContinuousObservationChannel>("", "", 1);
        else if (kind == "discrete")    ch = std::make_unique<DiscreteObservationChannel>("", "", 1);
        else if (kind == "categorical") ch = std::make_unique<CategoricalObservationChannel>("", "", std::vector<std::string>{}, 1);
        else if (kind == "raw_vector")  ch = std::make_unique<RawVectorObservationChannel>("", "", 1, 1);
        if (ch) { ch->deserialize(cj); addChannel(std::move(ch)); }
    }
}

ObservationChannel* ObservationSpace::find(const std::string& id) {
    auto it = index_.find(id);
    return it == index_.end() ? nullptr : channels_[it->second].get();
}
const ObservationChannel* ObservationSpace::find(const std::string& id) const {
    auto it = index_.find(id);
    return it == index_.end() ? nullptr : channels_[it->second].get();
}

}  // namespace Space
}  // namespace AuroX
