#ifndef AUROX_API
#  if defined(_WIN32) && defined(AUROX_BUILD_DLL)
#    define AUROX_API __declspec(dllexport)
#  elif defined(_WIN32) && defined(AUROX_USE_DLL)
#    define AUROX_API __declspec(dllimport)
#  else
#    define AUROX_API
#  endif
#endif

#include <string>
#include <memory>

namespace AuroX {
namespace Space {

// 执行结果：只在动作层内部使用；任务流管理器将来可以自由翻译
enum class ActionResult {
    Ok,        // 成功 / 默认无事发生
    Skipped,   // shouldRun() 返回 false
    Disabled,  // enabled() == false
    Failed     // 派生类抛异常或契约违反
};

inline const char* toString(ActionResult r) noexcept {
    switch (r) {
        case ActionResult::Ok:       return "ok";
        case ActionResult::Skipped:  return "skipped";
        case ActionResult::Disabled: return "disabled";
        case ActionResult::Failed:   return "failed";
    }
    return "unknown";
}
using ActionId = std::uint64_t;
constexpr ActionId kInvalidActionId = 0;

} // namespace Space
} // namespace AuroX

class AUROX_API ActionBase {
public:
    virtual ~ActionBase() = default;

    // ==================== 元信息（仅三项） ====================
    virtual const std::string& name() const = 0;   // 实例名
    virtual ActionId             id()   const = 0;   // 唯一标识
    virtual const std::string& type() const = 0;   // 类型名（工厂/任务流用）

    // ==================== 条件 ====================
    // 默认：无参，永远返回 true。
    // 需要自定义判断时，子类覆盖本函数，或自行添加带参数的重载
    // （重载不参与多态，仅子类内部使用）。
    virtual bool shouldRun() const { return true; }

    // ==================== 执行 ====================
    // 默认：直接返回 Ok，不做任何事。
    // 契约：
    //   1) gradient 与 space.vectorizeDouble() 同布局；
    //   2) 派生类不应抛异常；抛了由 ActionSpace 捕获并转成 Failed；
    virtual ActionResult apply(ParameterSpace& /*space*/,
                               const std::vector<double>& /*gradient*/) {
        return ActionResult::Ok;
    }

    // ==================== 开关 ====================
    bool enabled() const noexcept { return enabled_; }
    void setEnabled(bool e) noexcept { enabled_ = e; }

protected:
    bool enabled_ = true;
};

class ParameterUpdateAction : public ActionBase {
public:
    ParameterUpdateAction(std::string name = "parameter_update",
                          ActionId id = kInvalidActionId)
        : name_(std::move(name)), id_(id) {}

    const std::string& name() const override { return name_; }
    ActionId            id()   const override { return id_; }

    const std::string& type() const override {
        static const std::string t = "parameter_update";
        return t;
    }

    ActionResult apply(ParameterSpace& space,
                       const std::vector<double>& g) override {
        if (!enabled_)    return ActionResult::Disabled;
        if (!shouldRun()) return ActionResult::Skipped;
        if (g.empty())    return ActionResult::Ok;   // 空梯度 = no-op

        auto x = space.vectorizeDouble();
        const std::size_t n = std::min(x.size(), g.size());
        for (std::size_t i = 0; i < n; ++i) {
            x[i] += g[i];
        }
        space.unvectorizeDouble(x);
        return ActionResult::Ok;
    }

private:
    std::string name_;
    ActionId    id_;
};


class AUROX_API ActionSpace {
public:
    // 默认构造：自带一个参数更新动作
    ActionSpace() {
        auto a = std::make_unique<ParameterUpdateAction>(
            "default_parameter_update", kDefaultActionId);
        defaultId_ = kDefaultActionId;
        nextId_    = kDefaultActionId + 1;
        indexByName_[a->name()] = 0;
        indexById_[a->id()]     = 0;
        items_.push_back(std::move(a));
    }

    // 不可拷贝（unique_ptr 成员），可移动
    ActionSpace(const ActionSpace&) = delete;
    ActionSpace& operator=(const ActionSpace&) = delete;
    ActionSpace(ActionSpace&&) noexcept = default;
    ActionSpace& operator=(ActionSpace&&) noexcept = default;

    // ==================== 注册 ====================
    // 返回分配到的 ID；id 为 kInvalidActionId 时由本空间自动分配
    ActionId add(std::unique_ptr<ActionBase> a) {
        if (!a) return kInvalidActionId;

        ActionId id = a->id();
        if (id == kInvalidActionId) {
            // 基类 id 只读，无法回写。约定：调用方在构造时必须给有效 id，
            // 或者用 ActionSpace::nextId() 预分配。
            return kInvalidActionId;
        }
        if (indexById_.count(id)) {
            // 同 ID 替换
            auto& old = items_[indexById_[id]];
            indexByName_.erase(old->name());
            old = std::move(a);
            indexByName_[old->name()] = indexById_[id];
            return id;
        }
        indexById_[id] = items_.size();
        indexByName_[a->name()] = items_.size();
        items_.push_back(std::move(a));
        return id;
    }

    // 预分配 ID（供用户构造 Action 时使用）
    ActionId nextId() noexcept { return nextId_++; }

    // ==================== 查找 ====================
    ActionBase*       get(std::string_view name) {
        auto it = indexByName_.find(std::string(name));
        return it == indexByName_.end() ? nullptr : items_[it->second].get();
    }
    ActionBase*       getById(ActionId id) {
        auto it = indexById_.find(id);
        return it == indexById_.end() ? nullptr : items_[it->second].get();
    }
    const ActionBase* get(std::string_view name) const {
        return const_cast<ActionSpace*>(this)->get(name);
    }
    const ActionBase* getById(ActionId id) const {
        return const_cast<ActionSpace*>(this)->getById(id);
    }

    // 默认自带的那个动作
    ActionBase*       defaultAction()       { return getById(defaultId_); }
    const ActionBase* defaultAction() const { return getById(defaultId_); }
    ActionId          defaultActionId() const noexcept { return defaultId_; }

    // ==================== 容器 ====================
    std::size_t size()  const noexcept { return items_.size(); }
    bool        empty() const noexcept { return items_.empty(); }

    void clear() {
        items_.clear();
        indexByName_.clear();
        indexById_.clear();
        defaultId_ = kInvalidActionId;
    }

    ActionBase*       at(std::size_t i)       { return i < items_.size() ? items_[i].get() : nullptr; }
    const ActionBase* at(std::size_t i) const { return i < items_.size() ? items_[i].get() : nullptr; }

    std::vector<std::string> names() const {
        std::vector<std::string> out; out.reserve(items_.size());
        for (auto& a : items_) out.push_back(a->name());
        return out;
    }

    // ==================== 执行 ====================
    // 顺序执行所有 enabled 的动作；遇到 Failed 立即停止
    ActionResult applyAll(ParameterSpace& space,
                          const std::vector<double>& gradient) {
        for (auto& a : items_) {
            if (!a->enabled()) continue;
            ActionResult r;
            try {
                r = a->apply(space, gradient);
            } catch (...) {
                r = ActionResult::Failed;
            }
            if (r == ActionResult::Failed) return r;
        }
        return ActionResult::Ok;
    }

    // 只执行指定名字的动作
    ActionResult applyOne(std::string_view name,
                          ParameterSpace& space,
                          const std::vector<double>& gradient) {
        auto* a = get(name);
        if (!a) return ActionResult::Failed;
        if (!a->enabled()) return ActionResult::Disabled;
        try {
            return a->apply(space, gradient);
        } catch (...) {
            return ActionResult::Failed;
        }
    }

private:
    static constexpr ActionId kDefaultActionId = 1;

    std::vector<std::unique_ptr<ActionBase>> items_;
    std::unordered_map<std::string, std::size_t> indexByName_;
    std::unordered_map<ActionId, std::size_t>    indexById_;
    ActionId defaultId_ = kInvalidActionId;
    ActionId nextId_    = kDefaultActionId + 1;
};