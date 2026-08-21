#include "AuroX/Space/ParameterSpace.hpp"

#include "json.hpp"

namespace AuroX {
namespace Space {

static std::unique_ptr<ParameterBase> createParameter(const json& j) {
    const std::string type = j.value("type", "float");
    const std::string name = j.value("name", "");

    auto build = [&](auto dummy) -> std::unique_ptr<ParameterBase> {
        using P = decltype(dummy);
        auto p = std::make_unique<P>();
        p->name_ = name;
        p->fromJson(j);
        return p;
    };

    if (type == "float")   return build(TypedParameter<float>{});
    if (type == "double")  return build(TypedParameter<double>{});
    if (type == "int")     return build(TypedParameter<int>{});
    if (type == "bool")    return build(TypedParameter<bool>{});
    if (type == "string")  return build(TypedParameter<std::string>{});
    if (type == "enum") {
        auto p = std::make_unique<EnumParameter>();
        p->name_ = name;
        p->fromJson(j);
        return p;
    }

    return nullptr;
}

void ParameterSpace::add(std::unique_ptr<ParameterBase> p) {
    if (!p) return;
    auto it = index_.find(p->name());
    if (it != index_.end()) {
        items_[it->second] = std::move(p);
    } else {
        index_[p->name()] = items_.size();
        items_.push_back(std::move(p));
    }
}

ParameterBase* ParameterSpace::get(const std::string& name) {
    auto it = index_.find(name);
    return it == index_.end() ? nullptr : items_[it->second].get();
}

const ParameterBase* ParameterSpace::get(const std::string& name) const {
    auto it = index_.find(name);
    return it == index_.end() ? nullptr : items_[it->second].get();
}

bool ParameterSpace::contains(const std::string& name) const {
    return index_.find(name) != index_.end();
}

ParameterBase* ParameterSpace::at(size_t i) {
    return i < items_.size() ? items_[i].get() : nullptr;
}

const ParameterBase* ParameterSpace::at(size_t i) const {
    return i < items_.size() ? items_[i].get() : nullptr;
}

size_t ParameterSpace::size() const { return items_.size(); }

bool ParameterSpace::empty() const { return items_.empty(); }

void ParameterSpace::clear() {
    items_.clear();
    index_.clear();
}

std::vector<std::string> ParameterSpace::names() const {
    std::vector<std::string> out;
    out.reserve(items_.size());
    for (const auto& p : items_) out.push_back(p->name());
    return out;
}

json ParameterSpace::toJson() const {
    json arr = json::array();
    for (const auto& p : items_) arr.push_back(p->toJson());
    return json{{"parameters", arr}};
}

void ParameterSpace::fromJson(const json& j) {
    clear();
    if (!j.contains("parameters") || !j["parameters"].is_array()) return;
    for (const auto& pj : j["parameters"]) {
        if (!pj.is_object()) continue;
        auto p = createParameter(pj);
        if (p) add(std::move(p));
    }
}

std::vector<double> ParameterSpace::vectorizeDouble() const {
    std::vector<double> out;
    out.reserve(items_.size());
    for (const auto& p : items_) {
        double v = 0.0;
        if (p->packDouble(v)) out.push_back(v);
    }
    return out;
}

void ParameterSpace::unvectorizeDouble(const std::vector<double>& v) {
    size_t i = 0;
    for (auto& p : items_) {
        double probe = 0.0;
        if (p->packDouble(probe)) {
            if (i < v.size()) p->unpackDouble(v[i++]);
        }
    }
}

std::vector<ParameterSpace::ComponentMeta> ParameterSpace::componentLayout() const {
    std::vector<ComponentMeta> out;
    for (const auto& p : items_) {
        const size_t c = p->componentCount();
        if (c > 0) out.push_back({p->name(), p->typeId(), c});
    }
    return out;
}

TypedParameter<float>* ParameterSpace::declare(const std::string& name, float def, float lo, float hi) {
    auto p = std::make_unique<TypedParameter<float>>(name, def, lo, hi);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<double>* ParameterSpace::declare(const std::string& name, double def, double lo, double hi) {
    auto p = std::make_unique<TypedParameter<double>>(name, def, lo, hi);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<int>* ParameterSpace::declare(const std::string& name, int def, int lo, int hi) {
    auto p = std::make_unique<TypedParameter<int>>(name, def, lo, hi);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<bool>* ParameterSpace::declare(const std::string& name, bool def, bool lo, bool hi) {
    auto p = std::make_unique<TypedParameter<bool>>(name, def, lo, hi);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<float>* ParameterSpace::declare(const std::string& name, float def) {
    auto p = std::make_unique<TypedParameter<float>>(name, def);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<double>* ParameterSpace::declare(const std::string& name, double def) {
    auto p = std::make_unique<TypedParameter<double>>(name, def);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<int>* ParameterSpace::declare(const std::string& name, int def) {
    auto p = std::make_unique<TypedParameter<int>>(name, def);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<bool>* ParameterSpace::declare(const std::string& name, bool def) {
    auto p = std::make_unique<TypedParameter<bool>>(name, def);
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<int>* ParameterSpace::declareDiscrete(const std::string& name, int def, std::vector<int> choices) {
    auto p = std::make_unique<TypedParameter<int>>(name, def, std::move(choices));
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<float>* ParameterSpace::declareDiscrete(const std::string& name, float def, std::vector<float> choices) {
    auto p = std::make_unique<TypedParameter<float>>(name, def, std::move(choices));
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

TypedParameter<std::string>* ParameterSpace::declareDiscrete(const std::string& name, std::string def, std::vector<std::string> choices) {
    auto p = std::make_unique<TypedParameter<std::string>>(name, def, std::move(choices));
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

EnumParameter* ParameterSpace::declareEnum(const std::string& name, size_t defIndex, std::vector<std::string> labels) {
    auto p = std::make_unique<EnumParameter>(name, defIndex, std::move(labels));
    auto* raw = p.get();
    add(std::move(p));
    return raw;
}

EnumParameter* ParameterSpace::declareEnum(const std::string& name, const std::string& defLabel, std::vector<std::string> labels) {
    size_t idx = 0;
    for (size_t i = 0; i < labels.size(); ++i) {
        if (labels[i] == defLabel) { idx = i; break; }
    }
    return declareEnum(name, idx, std::move(labels));
}

EnumParameter* ParameterSpace::getEnum(const std::string& name) {
    auto* p = get(name);
    return (p && dynamic_cast<EnumParameter*>(p)) ? static_cast<EnumParameter*>(p) : nullptr;
}

const EnumParameter* ParameterSpace::getEnum(const std::string& name) const {
    auto* p = get(name);
    return (p && dynamic_cast<const EnumParameter*>(p)) ? static_cast<const EnumParameter*>(p) : nullptr;
}

}  // namespace Space
}  // namespace AuroX
