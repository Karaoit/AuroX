#include "AuroX/Space/ActionSpace.hpp"

#include <algorithm>
#include <cstddef>

namespace AuroX {
namespace Space {

// ============================================================================
// ActionResult
// ============================================================================
const char* toString(ActionResult r) noexcept {
    switch (r) {
        case ActionResult::Ok:       return "ok";
        case ActionResult::Skipped:  return "skipped";
        case ActionResult::Disabled: return "disabled";
        case ActionResult::Failed:   return "failed";
    }
    return "unknown";
}

ActionResult toActionResult(std::string_view s) noexcept {
    if (s == "skipped")  return ActionResult::Skipped;
    if (s == "disabled") return ActionResult::Disabled;
    if (s == "failed")   return ActionResult::Failed;
    return ActionResult::Ok;
}

// ============================================================================
// ParamUpdateAction - 默认动作
// ============================================================================
ActionResult ParamUpdateAction::apply(ParameterSpace& space,
                                      const std::vector<double>& gradient,
                                      const UpdateFn& updateSource) {
    if (!enabled_)             return ActionResult::Disabled;
    if (!shouldRun(space))     return ActionResult::Skipped;
    if (!feasible(space))      return ActionResult::Skipped;

    std::vector<double> delta;
    if (useSource && updateSource) {
        // 判断依据来自优化器：增量由接线进来的更新源给出。
        delta = updateSource(space, gradient);
    } else {
        // 独立使用（未接线优化器）：退化为普通梯度下降步。
        delta = gradient;
        for (auto& d : delta) d = -fallbackLr * d;
    }
    if (delta.empty()) return ActionResult::Ok;   // 空增量 = no-op

    auto theta = space.vectorizeDouble();
    const std::size_t n = std::min(theta.size(), delta.size());
    for (std::size_t i = 0; i < n; ++i) theta[i] += scale * delta[i];
    space.unvectorizeDouble(theta);
    return ActionResult::Ok;
}

json ParamUpdateAction::config() const {
    return json{{"scale", scale},
                {"fallbackLr", fallbackLr},
                {"useSource", useSource}};
}

void ParamUpdateAction::setConfig(const json& j) {
    if (j.contains("scale"))      scale      = j["scale"].get<double>();
    if (j.contains("fallbackLr")) fallbackLr = j["fallbackLr"].get<double>();
    if (j.contains("useSource"))  useSource  = j["useSource"].get<bool>();
}

// ============================================================================
// CustomAction - 数据型占位动作
// ============================================================================
bool CustomAction::feasible(const ParameterSpace& space) const {
    if (paramDelta_.empty()) return true;   // 纯占位永远可行
    return paramDelta_.size() == space.vectorizeDouble().size();
}

ActionResult CustomAction::apply(ParameterSpace& space,
                                 const std::vector<double>& gradient,
                                 const UpdateFn& updateSource) {
    (void)gradient;
    (void)updateSource;
    if (!enabled_)         return ActionResult::Disabled;
    if (!shouldRun(space)) return ActionResult::Skipped;
    if (!feasible(space))  return ActionResult::Skipped;

    // 不执行任何真实硬件；给了 paramDelta 就把它叠加到参数上，
    // 让「串行叠加」这条数据流端到端可观测。
    if (paramDelta_.empty()) return ActionResult::Ok;

    auto theta = space.vectorizeDouble();
    const std::size_t n = std::min(theta.size(), paramDelta_.size());
    for (std::size_t i = 0; i < n; ++i) theta[i] += paramDelta_[i];
    space.unvectorizeDouble(theta);
    return ActionResult::Ok;
}

json CustomAction::config() const {
    json c = json::object();
    c["name"] = name_;
    if (!params_.is_null())     c["params"] = params_;
    if (!state_.is_null())      c["state"] = state_;
    if (!transition_.is_null()) c["transition"] = transition_;
    if (!paramDelta_.empty())   c["paramDelta"] = paramDelta_;
    return c;
}

void CustomAction::setConfig(const json& j) {
    if (j.contains("name"))       name_       = j["name"].get<std::string>();
    if (j.contains("params"))     params_     = j["params"];
    if (j.contains("state"))      state_      = j["state"];
    if (j.contains("transition")) transition_ = j["transition"];
    if (j.contains("paramDelta"))
        paramDelta_ = j["paramDelta"].get<std::vector<double>>();
}

// ============================================================================
// FnAction - 函数 / lambda 动作
// ============================================================================
ActionResult FnAction::apply(ParameterSpace& space,
                             const std::vector<double>& gradient,
                             const UpdateFn& updateSource) {
    if (!enabled_)         return ActionResult::Disabled;
    if (!shouldRun(space)) return ActionResult::Skipped;
    if (!feasible(space))  return ActionResult::Skipped;
    if (!fn_)              return ActionResult::Failed;   // 反序列化后未回填函数体
    return fn_(space, gradient, updateSource);
}

json FnAction::config() const {
    json c = json::object();
    c["name"] = name_;
    c["cfg"] = cfg_;
    return c;
}

void FnAction::setConfig(const json& j) {
    if (j.contains("name")) name_ = j["name"].get<std::string>();
    if (j.contains("cfg"))  cfg_  = j["cfg"];
    // 函数体不可序列化：反序列化后需调用 setFn() 回填。
}

// ============================================================================
// 类型注册表（让自定义动作也能 JSON 往返）
// ============================================================================
namespace {
std::unordered_map<std::string, ActFactory>& actionRegistry() {
    // 函数内静态变量：避免跨编译单元的静态初始化顺序问题。
    static std::unordered_map<std::string, ActFactory> reg;
    return reg;
}
}  // namespace

void registerActionType(std::string typeTag, ActFactory factory) {
    if (typeTag.empty() || !factory) return;
    actionRegistry()[std::move(typeTag)] = std::move(factory);
}

std::unique_ptr<ActBase> createAction(const json& serialized) {
    const std::string t = serialized.value("type", "param_update");
    std::unique_ptr<ActBase> a;

    auto& reg = actionRegistry();
    auto it = reg.find(t);
    if (it != reg.end())          a = it->second();
    else if (t == "custom")       a = std::make_unique<CustomAction>();
    else if (t == "fn")           a = std::make_unique<FnAction>();
    else                          a = std::make_unique<ParamUpdateAction>();

    if (!a) return nullptr;
    if (serialized.contains("config")) a->setConfig(serialized["config"]);
    if (serialized.contains("id"))     a->setId(serialized["id"].get<ActionId>());
    return a;
}

// ============================================================================
// ActionSpace
// ============================================================================
ActionSpace::ActionSpace() { clear(); }

ActionId ActionSpace::add(std::unique_ptr<ActBase> a) {
    if (!a) return kInvalidActionId;

    ActionId id = a->id();
    if (id == kInvalidActionId) {
        id = nextId_++;
        a->setId(id);
    } else if (id >= nextId_) {
        nextId_ = id + 1;   // 用户指定了大 id，避免后续分配撞车
    }

    auto it = indexById_.find(id);
    if (it != indexById_.end()) {
        // 同 id 替换
        auto& old = items_[it->second];
        indexByName_.erase(old->name());
        old = std::move(a);
        indexByName_[old->name()] = it->second;
        return id;
    }

    indexById_[id] = items_.size();
    indexByName_[a->name()] = items_.size();
    items_.push_back(std::move(a));
    return id;
}

FnAction* ActionSpace::addFn(std::string name, ActFn fn, json cfg) {
    return emplace<FnAction>(std::move(name), std::move(fn), std::move(cfg));
}

ActBase* ActionSpace::get(std::string_view name) {
    auto it = indexByName_.find(std::string(name));
    return it == indexByName_.end() ? nullptr : items_[it->second].get();
}
const ActBase* ActionSpace::get(std::string_view name) const {
    return const_cast<ActionSpace*>(this)->get(name);
}

ActBase* ActionSpace::getById(ActionId id) {
    auto it = indexById_.find(id);
    return it == indexById_.end() ? nullptr : items_[it->second].get();
}
const ActBase* ActionSpace::getById(ActionId id) const {
    return const_cast<ActionSpace*>(this)->getById(id);
}

ActBase* ActionSpace::at(std::size_t i) {
    return i < items_.size() ? items_[i].get() : nullptr;
}
const ActBase* ActionSpace::at(std::size_t i) const {
    return i < items_.size() ? items_[i].get() : nullptr;
}

std::vector<std::string> ActionSpace::names() const {
    std::vector<std::string> out;
    out.reserve(items_.size());
    for (const auto& a : items_) out.push_back(a->name());
    return out;
}

bool ActionSpace::remove(std::string_view name) {
    auto* a = get(name);
    return a ? removeById(a->id()) : false;
}

bool ActionSpace::removeById(ActionId id) {
    auto it = indexById_.find(id);
    if (it == indexById_.end()) return false;
    items_.erase(items_.begin() + static_cast<std::ptrdiff_t>(it->second));
    if (id == defaultId_) defaultId_ = kInvalidActionId;
    reindex();
    return true;
}

void ActionSpace::clear() {
    // 复位为「仅默认动作」，保证默认集合永不为空。
    items_.clear();
    indexByName_.clear();
    indexById_.clear();

    auto def = std::make_unique<ParamUpdateAction>();
    def->setId(kDefaultActionId);
    indexByName_[def->name()] = 0;
    indexById_[def->id()] = 0;
    items_.push_back(std::move(def));

    defaultId_ = kDefaultActionId;
    nextId_ = kDefaultActionId + 1;
}

void ActionSpace::reindex() {
    indexByName_.clear();
    indexById_.clear();
    for (std::size_t i = 0; i < items_.size(); ++i) {
        indexByName_[items_[i]->name()] = i;
        indexById_[items_[i]->id()] = i;
    }
}

ActionResult ActionSpace::runOne(ActBase& a,
                                 ParameterSpace& space,
                                 const std::vector<double>& gradient,
                                 ActOutcome* out) {
    ActOutcome rec;
    rec.id = a.id();
    rec.name = a.name();

    if (!a.enabled())             rec.result = ActionResult::Disabled;
    else if (!a.shouldRun(space)) rec.result = ActionResult::Skipped;
    else if (!a.feasible(space))  rec.result = ActionResult::Skipped;
    else {
        try {
            rec.result = a.apply(space, gradient, updateSource_);
        } catch (...) {
            rec.result = ActionResult::Failed;   // 派生类抛异常统一降级
        }
    }

    if (out) *out = rec;
    return rec.result;
}

ActionResult ActionSpace::applyAll(ParameterSpace& space,
                                   const std::vector<double>& gradient,
                                   std::vector<ActOutcome>* out) {
    if (out) {
        out->clear();
        out->reserve(items_.size());
    }
    for (auto& a : items_) {
        ActOutcome rec;
        ActionResult r = runOne(*a, space, gradient, &rec);
        if (out) out->push_back(std::move(rec));
        if (r == ActionResult::Failed) return r;   // 遇失败立即停止
    }
    return ActionResult::Ok;
}

ActionResult ActionSpace::applyOne(std::string_view name,
                                   ParameterSpace& space,
                                   const std::vector<double>& gradient,
                                   ActOutcome* out) {
    auto* a = get(name);
    if (!a) return ActionResult::Failed;
    return runOne(*a, space, gradient, out);
}

ActionResult ActionSpace::applyOne(ActionId id,
                                   ParameterSpace& space,
                                   const std::vector<double>& gradient,
                                   ActOutcome* out) {
    auto* a = getById(id);
    if (!a) return ActionResult::Failed;
    return runOne(*a, space, gradient, out);
}

// ============================================================================
// 序列化
// ============================================================================
json ActionSpace::toJson() const {
    json arr = json::array();
    for (const auto& a : items_) {
        json aj = json::object();
        aj["type"] = a->type();
        aj["name"] = a->name();
        aj["id"] = a->id();
        aj["enabled"] = a->enabled();
        aj["config"] = a->config();
        arr.push_back(std::move(aj));
    }
    return json{{"version", 1}, {"actions", arr}};
}

void ActionSpace::fromJson(const json& j) {
    items_.clear();
    indexByName_.clear();
    indexById_.clear();
    defaultId_ = kInvalidActionId;
    nextId_ = kDefaultActionId + 1;

    if (j.contains("actions") && j["actions"].is_array()) {
        for (const auto& aj : j["actions"]) {
            auto a = createAction(aj);
            if (!a) continue;
            if (aj.contains("enabled")) a->setEnabled(aj["enabled"].get<bool>());
            add(std::move(a));
        }
    }

    if (items_.empty()) {
        clear();   // 保证默认集合非空
        return;
    }
    if (defaultId_ == kInvalidActionId) {
        for (const auto& a : items_) {
            if (std::string(a->type()) == "param_update") { defaultId_ = a->id(); break; }
        }
        if (defaultId_ == kInvalidActionId) defaultId_ = items_.front()->id();
    }
}

bool ActionSpace::tryFromJson(const json& j) noexcept {
    try {
        fromJson(j);
        return true;
    } catch (...) {
        return false;
    }
}

}  // namespace Space
}  // namespace AuroX
