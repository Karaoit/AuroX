#include "AuroX/Space/ParameterSpace.hpp"
#include "json.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace AuroX {
namespace Space {

// ============================================================================
// Internal factory
// ============================================================================
static std::unique_ptr<ParameterBase> createParameter(const json& j) {
    // P1 (JSON structure validation): a well-formed entry must be an object
    // carrying an explicit, known "type" and a "name". Previously a missing
    // "type" silently defaulted to "float", so a malformed payload could create
    // a wrong-typed parameter instead of being rejected.
    if (!j.is_object()) return nullptr;
    if (!j.contains("type") || !j["type"].is_string()) return nullptr;
    if (!j.contains("name") || !j["name"].is_string()) return nullptr;
    const std::string type = j["type"].get<std::string>();
    const std::string name = j["name"].get<std::string>();
    auto build = [&](auto dummy) -> std::unique_ptr<ParameterBase> {
        using P = decltype(dummy);
        auto p = std::make_unique<P>();
        p->name_ = name;
        p->fromJson(j);
        return p;
    };
    if (type == "float") return build(TypedParameter<float>{});
    if (type == "double") return build(TypedParameter<double>{});
    if (type == "int") return build(TypedParameter<int>{});
    if (type == "bool") return build(TypedParameter<bool>{});
    if (type == "string") return build(TypedParameter<std::string>{});
    if (type == "enum") {
        auto p = std::make_unique<EnumParameter>();
        p->name_ = name;
        p->fromJson(j);
        return p;
    }
    return nullptr;
}

// ============================================================================
// ParameterBase - default virtual implementations
// ============================================================================
bool ParameterBase::isValid() const { return isWithinBounds(); }
bool ParameterBase::isBounded() const { return false; }
bool ParameterBase::packDouble(double& out) const { (void)out; return false; }
bool ParameterBase::unpackDouble(double v) { (void)v; return false; }
std::size_t ParameterBase::componentCount() const { return 0; }
// Lenient default: a custom parameter that packs doubles but does not override
// acceptsDouble() is still accepted. TypedParameter<T> and EnumParameter
// override this with strict, type-aware validation.
bool ParameterBase::acceptsDouble(double v) const { (void)v; return true; }

// ============================================================================
// EnumParameter
// ============================================================================
EnumParameter::EnumParameter(std::string name, std::size_t defIndex, std::vector<std::string> labels)
    : name_(std::move(name)), labels_(std::move(labels)), index_(defIndex < labels_.size() ? defIndex : 0) {}

const std::string& EnumParameter::name() const { return name_; }
ParameterKind EnumParameter::kind() const { return ParameterKind::Discrete; }
ParameterTypeId EnumParameter::typeId() const { return ParameterTypeId::Enum; }
const char* EnumParameter::typeName() const { return "enum"; }
std::unique_ptr<ParameterBase> EnumParameter::clone() const { return std::make_unique<EnumParameter>(*this); }
bool EnumParameter::isBounded() const { return false; }
bool EnumParameter::isWithinBounds() const { return index_ < labels_.size(); }
std::size_t EnumParameter::index() const { return index_; }

void EnumParameter::setIndex(std::size_t i) {
    if (labels_.empty()) { index_ = 0; return; }
    index_ = (i < labels_.size()) ? i : labels_.size() - 1;
}

const std::string& EnumParameter::label() const {
    static const std::string empty;
    return index_ < labels_.size() ? labels_[index_] : empty;
}

void EnumParameter::setLabel(const std::string& l) {
    for (std::size_t i = 0; i < labels_.size(); ++i) {
        if (labels_[i] == l) { index_ = i; return; }
    }
    labels_.push_back(l);
    index_ = labels_.size() - 1;
}

const std::vector<std::string>& EnumParameter::labels() const { return labels_; }

json EnumParameter::toJson() const {
    json j = json::object();
    j["name"] = name_; j["type"] = "enum"; j["kind"] = "discrete";
    j["labels"] = labels_; j["value"] = index_; j["label"] = label();
    return j;
}

void EnumParameter::fromJson(const json& j) {
    if (j.contains("labels") && j["labels"].is_array())
        labels_ = j["labels"].get<std::vector<std::string>>();
    if (j.contains("value")) {
        if (j["value"].is_number())
            setIndex(static_cast<std::size_t>(j["value"].get<long long>()));
        else if (j["value"].is_string())
            setLabel(j["value"].get<std::string>());
    } else if (j.contains("label")) {
        setLabel(j["label"].get<std::string>());
    }
}

json EnumParameter::valueToJson() const { return label(); }
void EnumParameter::setValueFromJson(const json& v) {
    if (v.is_string()) setLabel(v.get<std::string>());
    else if (v.is_number()) setIndex(static_cast<std::size_t>(v.get<long long>()));
}
bool EnumParameter::packDouble(double& out) const { out = static_cast<double>(index_); return true; }
bool EnumParameter::unpackDouble(double v) { setIndex(static_cast<std::size_t>(static_cast<long long>(std::llround(v)))); return true; }
std::size_t EnumParameter::componentCount() const { return 1; }
// P-04: strict check - the rounded index must be a real label index. Unlike
// setIndex(), this does NOT clamp, so an out-of-range candidate is rejected
// instead of silently snapping to the last label.
// P2: range-check BEFORE std::llround - rounding a huge double is undefined
// behaviour, so the candidate is bounded to [0, labels-1] first.
bool EnumParameter::acceptsDouble(double v) const {
    if (!std::isfinite(v) || labels_.empty()) return false;
    if (v < 0.0 || v > static_cast<double>(labels_.size() - 1)) return false;
    const long long idx = std::llround(v);
    return idx >= 0 && static_cast<std::size_t>(idx) < labels_.size();
}

// ============================================================================
// Registration
// ============================================================================
// P-02: registering a duplicate name is rejected. Replacement used to overwrite
// the unique_ptr in place, leaving any Param/EnumParam proxy that still pointed
// at the old object dangling. Re-targeting an existing proxy is done through
// ParameterSpace::rebind()/PStruct::rebind(), not by re-declaring.
void ParameterSpace::add(std::unique_ptr<ParameterBase> p) {
    if (!p) return;
    if (index_.find(p->name()) != index_.end())
        throw std::invalid_argument(
            "ParameterSpace: duplicate parameter name '" + p->name() + "'");
    index_[p->name()] = items_.size();
    items_.push_back(std::move(p));
}

// ============================================================================
// Lookup
// ============================================================================
ParameterBase* ParameterSpace::get(const std::string& name) {
    auto it = index_.find(name);
    return it == index_.end() ? nullptr : items_[it->second].get();
}
const ParameterBase* ParameterSpace::get(const std::string& name) const {
    auto it = index_.find(name);
    return it == index_.end() ? nullptr : items_[it->second].get();
}
bool ParameterSpace::contains(const std::string& name) const { return index_.find(name) != index_.end(); }
ParameterBase* ParameterSpace::at(std::size_t i) { return i < items_.size() ? items_[i].get() : nullptr; }
const ParameterBase* ParameterSpace::at(std::size_t i) const { return i < items_.size() ? items_[i].get() : nullptr; }

// ============================================================================
// Container
// ============================================================================
std::size_t ParameterSpace::size() const { return items_.size(); }
bool ParameterSpace::empty() const { return items_.empty(); }
void ParameterSpace::clear() { items_.clear(); index_.clear(); }
std::vector<std::string> ParameterSpace::names() const {
    std::vector<std::string> out; out.reserve(items_.size());
    for (const auto& p : items_) out.push_back(p->name());
    return out;
}

// ============================================================================
// Serialization
// ============================================================================
json ParameterSpace::toJson() const {
    json arr = json::array();
    for (const auto& p : items_) arr.push_back(p->toJson());
    return json{{"parameters", arr}};
}
// P-03: parse into a temporary space and commit only on full success. The old
// version clear()ed first, so a malformed payload destroyed the previous state
// and left a partial result behind. After a successful load the caller must
// re-bind any Param/EnumParam proxies (PStruct::fromJson does exactly that).
//
// P1 (JSON structure validation): the payload must be an object that carries a
// "parameters" array, and every entry must be a well-formed object. Previously
// the key was optional, so `tryFromJson(json::object())` returned true after
// silently WIPING the space - and, for a PStruct, leaving every Param proxy
// unbound. A structurally invalid payload now throws, which - via the
// temp-then-commit below - leaves the original space untouched.
void ParameterSpace::fromJson(const json& j) {
    if (!j.is_object())
        throw std::invalid_argument("ParameterSpace: JSON root must be an object");
    if (!j.contains("parameters") || !j["parameters"].is_array())
        throw std::invalid_argument("ParameterSpace: missing 'parameters' array");

    ParameterSpace tmp;
    for (const auto& pj : j["parameters"]) {
        if (!pj.is_object())
            throw std::invalid_argument("ParameterSpace: parameter entry must be an object");
        auto p = createParameter(pj);
        if (!p)
            throw std::invalid_argument("ParameterSpace: malformed or unknown parameter entry");
        tmp.add(std::move(p));   // throws on a duplicate name within the payload
    }
    // Commit (strong guarantee: the original is untouched if anything above threw).
    items_ = std::move(tmp.items_);
    index_ = std::move(tmp.index_);
}

bool ParameterSpace::tryFromJson(const json& j) noexcept {
    try {
        fromJson(j);
        return true;
    } catch (...) {
        return false;
    }
}

// ============================================================================
// Vectorization
// ============================================================================
std::vector<double> ParameterSpace::vectorizeDouble() const {
    std::vector<double> out; out.reserve(items_.size());
    for (const auto& p : items_) {
        double v = 0.0;
        if (p->packDouble(v)) out.push_back(v);
    }
    return out;
}
// P-04: validate the FULL length and every component before committing (all-or-
// nothing). The old version wrote short input partially and silently ignored
// extra entries, with no bound / candidate-set check.
void ParameterSpace::unvectorizeDouble(const std::vector<double>& v) {
    std::size_t expected = 0;
    for (const auto& p : items_) expected += p->componentCount();
    if (v.size() != expected)
        throw std::invalid_argument(
            "ParameterSpace: vector length mismatch (expected " +
            std::to_string(expected) + ", got " + std::to_string(v.size()) + ")");

    // Pass 1: validate every component (finite, in range / in candidate set).
    std::size_t i = 0;
    for (const auto& p : items_) {
        if (p->componentCount() == 0) continue;
        if (!p->acceptsDouble(v[i]))
            throw std::invalid_argument(
                "ParameterSpace: value out of range for parameter '" + p->name() + "'");
        ++i;
    }

    // Pass 2: commit - safe now, every value passed validation.
    i = 0;
    for (auto& p : items_)
        if (p->componentCount() != 0) p->unpackDouble(v[i++]);
}
std::vector<ParameterSpace::ComponentMeta> ParameterSpace::componentLayout() const {
    std::vector<ParameterSpace::ComponentMeta> out;
    std::size_t offset = 0;   // P-06: explicit start offset of each parameter in vec()
    for (const auto& p : items_) {
        const std::size_t c = p->componentCount();
        if (c == 0) continue;
        out.push_back({p->name(), p->typeId(), c, p->typeId() == ParameterTypeId::Enum, offset});
        offset += c;
    }
    return out;
}

// ============================================================================
// Continuous declaration - bounded
// ============================================================================
TypedParameter<float>* ParameterSpace::declare(const std::string& name, float def, float lo, float hi) {
    auto p = std::make_unique<TypedParameter<float>>(name, def, lo, hi);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<double>* ParameterSpace::declare(const std::string& name, double def, double lo, double hi) {
    auto p = std::make_unique<TypedParameter<double>>(name, def, lo, hi);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<int>* ParameterSpace::declare(const std::string& name, int def, int lo, int hi) {
    auto p = std::make_unique<TypedParameter<int>>(name, def, lo, hi);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<bool>* ParameterSpace::declare(const std::string& name, bool def, bool lo, bool hi) {
    auto p = std::make_unique<TypedParameter<bool>>(name, def, lo, hi);
    auto* raw = p.get(); add(std::move(p)); return raw;
}

// ============================================================================
// Continuous declaration - unbounded
// ============================================================================
TypedParameter<float>* ParameterSpace::declare(const std::string& name, float def) {
    auto p = std::make_unique<TypedParameter<float>>(name, def);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<double>* ParameterSpace::declare(const std::string& name, double def) {
    auto p = std::make_unique<TypedParameter<double>>(name, def);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<int>* ParameterSpace::declare(const std::string& name, int def) {
    auto p = std::make_unique<TypedParameter<int>>(name, def);
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<bool>* ParameterSpace::declare(const std::string& name, bool def) {
    auto p = std::make_unique<TypedParameter<bool>>(name, def);
    auto* raw = p.get(); add(std::move(p)); return raw;
}

// ============================================================================
// Discrete declaration
// ============================================================================
TypedParameter<int>* ParameterSpace::declareDiscrete(const std::string& name, int def, std::vector<int> choices) {
    auto p = std::make_unique<TypedParameter<int>>(name, def, std::move(choices));
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<float>* ParameterSpace::declareDiscrete(const std::string& name, float def, std::vector<float> choices) {
    auto p = std::make_unique<TypedParameter<float>>(name, def, std::move(choices));
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<double>* ParameterSpace::declareDiscrete(const std::string& name, double def, std::vector<double> choices) {
    auto p = std::make_unique<TypedParameter<double>>(name, def, std::move(choices));
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<bool>* ParameterSpace::declareDiscrete(const std::string& name, bool def, std::vector<bool> choices) {
    auto p = std::make_unique<TypedParameter<bool>>(name, def, std::move(choices));
    auto* raw = p.get(); add(std::move(p)); return raw;
}
TypedParameter<std::string>* ParameterSpace::declareDiscrete(const std::string& name, std::string def, std::vector<std::string> choices) {
    auto p = std::make_unique<TypedParameter<std::string>>(name, std::move(def), std::move(choices));
    auto* raw = p.get(); add(std::move(p)); return raw;
}

// ============================================================================
// Enum declaration
// ============================================================================
EnumParameter* ParameterSpace::declareEnum(const std::string& name, std::size_t defIndex, std::vector<std::string> labels) {
    auto p = std::make_unique<EnumParameter>(name, defIndex, std::move(labels));
    auto* raw = p.get(); add(std::move(p)); return raw;
}
EnumParameter* ParameterSpace::declareEnum(const std::string& name, const std::string& defLabel, std::vector<std::string> labels) {
    std::size_t idx = 0;
    for (std::size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == defLabel) { idx = i; break; }
    }
    return declareEnum(name, idx, std::move(labels));
}

// ============================================================================
// Enum lookup
// ============================================================================
EnumParameter* ParameterSpace::getEnum(const std::string& name) {
    auto* p = get(name);
    return (p && dynamic_cast<EnumParameter*>(p)) ? static_cast<EnumParameter*>(p) : nullptr;
}
const EnumParameter* ParameterSpace::getEnum(const std::string& name) const {
    auto* p = get(name);
    return (p && dynamic_cast<const EnumParameter*>(p)) ? static_cast<const EnumParameter*>(p) : nullptr;
}

// ============================================================================
// EnumParam
// ============================================================================
void EnumParam::bind(ParameterSpace& space, std::string_view name, const std::string& def, std::vector<std::string> labels) {
    name_ = std::string(name); value = def;
    raw_ = space.declareEnum(name_, def, std::move(labels));
    sync_to_raw();
}
bool EnumParam::rebind(ParameterSpace& space) {
    if (name_.empty()) { raw_ = nullptr; return false; }
    raw_ = space.getEnum(name_);
    return raw_ != nullptr;
}
void EnumParam::sync_to_raw() { if (raw_) raw_->setLabel(value); }
void EnumParam::sync_from_raw() { if (raw_) value = raw_->label(); }

EnumParam& EnumParam::operator=(const std::string& v) { value = v; sync_to_raw(); return *this; }
EnumParam& EnumParam::operator=(std::string&& v) { value = std::move(v); sync_to_raw(); return *this; }
EnumParam& EnumParam::operator=(const char* v) { value = v ? v : ""; sync_to_raw(); return *this; }
EnumParam::operator const std::string&() const { return value; }

// P-01: non-const std::string& get() removed - it let callers mutate the local
// mirror without going through sync_to_raw(). Use operator=() for writes.
const std::string& EnumParam::get() const { return value; }
const std::string& EnumParam::name() const { return name_; }
bool EnumParam::isBound() const { return raw_ != nullptr; }
bool EnumParam::isValid() const { return raw_ ? raw_->isValid() : false; }
// P1: only the const overload remains (see Param<T>::raw()) - no direct writes
// to the backing EnumParameter through an EnumParam proxy.
const EnumParameter* EnumParam::raw() const { return raw_; }

} // namespace Space
} // namespace AuroX
