#pragma once

// ============================================================================
// AuroX - ActionSpace  (Space layer)
// ----------------------------------------------------------------------------
// ActionSpace 是插在「优化器输出」与「参数真正更新」之间的决策面：它把优化器
// 给出的更新从「必须执行的步」降级为「众多候选动作中的一个默认动作」，并允许
// 在其后串行叠加其它动作（状态转移、移动镜头、电机指令……）。
//
// 设计要点：
//   1) 优化器只负责算出增量 delta —— 通过 UpdateFn 回调注入，ActionSpace
//      永远不 include 优化器头文件（无循环依赖）。
//   2) 默认动作集合只含「参数更新」；自定义动作串行叠加。
//   3) 每个动作都有 ActionResult（Ok / Skipped / Disabled / Failed）与
//      ActionId，便于 UI 回显与逐动作追溯。
//   4) 全部可 toJson / fromJson，前端无需后端即可重建整条流水线。
//
// 命名约定（与参数空间对称，均为无歧义缩写）：
//   ActBase          - 动作抽象基类（对应用户子类化的入口）
//   Act              - ActBase 的短别名；Action 为向后兼容别名
//   ParamUpdateAction- 默认动作「参数更新」（别名 ParameterUpdateAction）
//   CustomAction     - 数据型占位动作（承载未实现的状态空间/转移方程）
//   FnAction         - 函数/lambda 动作，无需子类化即可定义动作
//   ASpace           - ActionSpace 的短别名
//
// 宏（对齐 PARAM_REG / PARAM_BIND）：
//   ACT_TYPE(tag)        - 在叶子类里一行生成 type() + clone()
//   ACT_REG(space, ...)  - 注册块，内部提供 actSpace 引用
//   ACT_ADD(Type, ...)   - 在 ACT_REG 块内添加一个动作
// ============================================================================

#include "json.hpp"
#include "AuroX/Space/ParameterSpace.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
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

using json = nlohmann::json;

// ============================================================================
// 执行结果 / 标识
// ============================================================================

enum class ActionResult : int {
    Ok = 0,        // 成功执行
    Skipped = 1,   // shouldRun()/feasible() 返回 false，本轮跳过
    Disabled = 2,  // enabled() == false
    Failed = 3     // 抛异常或契约违反
};

AUROX_API const char* toString(ActionResult r) noexcept;
AUROX_API ActionResult toActionResult(std::string_view s) noexcept;

using ActionId = std::uint64_t;
constexpr ActionId kInvalidActionId = 0;

// 单次执行的结果记录（供 UI / 日志回放）
struct ActOutcome {
    ActionId id = kInvalidActionId;
    std::string name;
    ActionResult result = ActionResult::Ok;

    json toJson() const {
        return json{{"id", id}, {"name", name}, {"result", toString(result)}};
    }
};

// ============================================================================
// 更新源：优化器解耦的关键
//   由编排器（OptimizerSession）把 Optimizer::computeUpdate 接进来；
//   ActionSpace 自身不认识 Optimizer 类型。
// ============================================================================

using UpdateFn =
    std::function<std::vector<double>(ParameterSpace& space,
                                      const std::vector<double>& gradient)>;

// 向后兼容旧名
using ParameterUpdateFn = UpdateFn;

// ============================================================================
// ActBase - 动作抽象基类
// ============================================================================

class AUROX_API ActBase {
public:
    virtual ~ActBase() = default;

    // ---- 元信息 ----
    // 稳定的类型标签，用于（反）序列化与工厂；每个具体类型唯一。
    virtual const char* type() const = 0;
    // 显示名，默认等于 type()。
    virtual std::string name() const { return type(); }
    // 唯一标识，一般由 ActionSpace 分配（构造时给 kInvalidActionId 即可）。
    ActionId id() const noexcept { return id_; }
    void setId(ActionId id) noexcept { id_ = id; }

    // ---- 开关 ----
    bool enabled() const noexcept { return enabled_; }
    void setEnabled(bool e) noexcept { enabled_ = e; }

    // ---- 条件钩子（都不传梯度，纯状态判断） ----
    // 用户自定义的运行时条件；false -> Skipped。默认 true。
    virtual bool shouldRun(const ParameterSpace& space) const {
        (void)space;
        return true;
    }
    // 安全层的准入判断；false -> Skipped。默认 true。
    virtual bool feasible(const ParameterSpace& space) const {
        (void)space;
        return true;
    }

    // ---- 执行 ----
    // 契约：
    //   1) gradient 与 space.vectorizeDouble() 同布局；
    //   2) 不应抛异常；抛了由 ActionSpace 捕获并转成 Failed；
    //   3) 非 const —— 真实动作（硬件句柄、状态机）往往有状态。
    virtual ActionResult apply(ParameterSpace& space,
                               const std::vector<double>& gradient,
                               const UpdateFn& updateSource) = 0;

    // ---- 配置序列化 ----
    virtual json config() const { return json::object(); }
    virtual void setConfig(const json& j) { (void)j; }

    // ---- 深拷贝 ----
    virtual std::unique_ptr<ActBase> clone() const = 0;

protected:
    bool enabled_ = true;

private:
    ActionId id_ = kInvalidActionId;
};

// 短别名 / 向后兼容别名
using Act = ActBase;
using Action = ActBase;

// ============================================================================
// ParamUpdateAction - 默认动作：参数更新
//   updateSource 已接线 -> delta = updateSource(space, gradient)；
//   未接线（独立使用）-> 退化为 -fallbackLr * gradient 的普通梯度步。
//   scale 是对 delta 的额外阻尼系数（默认 1.0 = 不缩放）。
// ============================================================================

class AUROX_API ParamUpdateAction : public ActBase {
public:
    double scale = 1.0;         // 步长缩放 / 阻尼
    double fallbackLr = 0.01;   // 仅在无更新源时生效
    bool useSource = true;      // false 则永远走 fallback 梯度步

    const char* type() const override { return "param_update"; }

    ActionResult apply(ParameterSpace& space,
                       const std::vector<double>& gradient,
                       const UpdateFn& updateSource) override;

    json config() const override;
    void setConfig(const json& j) override;
    std::unique_ptr<ActBase> clone() const override {
        return std::make_unique<ParamUpdateAction>(*this);
    }
};

using ParameterUpdateAction = ParamUpdateAction;

// ============================================================================
// CustomAction - 数据型占位动作
//   表示「尚未实现」的动作（状态转移、移动镜头、电机指令）。它不碰真实硬件，
//   只承载数据：params / state / transition，外加一个可选的演示增量
//   paramDelta（为空则是纯占位、无副作用）。前端可直接从 JSON 渲染。
// ============================================================================

class AUROX_API CustomAction : public ActBase {
public:
    CustomAction() = default;

    CustomAction(std::string actionName,
                 json params = {},
                 json state = {},
                 json transition = {},
                 std::vector<double> paramDelta = {})
        : name_(std::move(actionName)),
          params_(std::move(params)),
          state_(std::move(state)),
          transition_(std::move(transition)),
          paramDelta_(std::move(paramDelta)) {}

    const char* type() const override { return "custom"; }
    std::string name() const override {
        return name_.empty() ? std::string("custom") : name_;
    }

    // 给了 paramDelta 就要求它与参数向量同维，否则不可行。
    bool feasible(const ParameterSpace& space) const override;

    ActionResult apply(ParameterSpace& space,
                       const std::vector<double>& gradient,
                       const UpdateFn& updateSource) override;

    json config() const override;
    void setConfig(const json& j) override;
    std::unique_ptr<ActBase> clone() const override {
        return std::make_unique<CustomAction>(*this);
    }

    const json& params() const { return params_; }
    const json& state() const { return state_; }
    const json& transition() const { return transition_; }
    const std::vector<double>& paramDelta() const { return paramDelta_; }
    void setParams(const json& p) { params_ = p; }
    void setState(const json& s) { state_ = s; }
    void setTransition(const json& t) { transition_ = t; }
    void setParamDelta(std::vector<double> d) { paramDelta_ = std::move(d); }

private:
    std::string name_;
    json params_;
    json state_;
    json transition_;
    std::vector<double> paramDelta_;
};

// ============================================================================
// FnAction - 函数 / lambda 动作
//   不想子类化时，直接把一个可调用对象变成动作。lambda 本身无法序列化，
//   反序列化后需对重建出的 FnAction 调用 setFn() 回填函数体。
// ============================================================================

using ActFn = std::function<ActionResult(ParameterSpace& space,
                                         const std::vector<double>& gradient,
                                         const UpdateFn& updateSource)>;

class AUROX_API FnAction : public ActBase {
public:
    FnAction() = default;
    FnAction(std::string actionName, ActFn fn, json cfg = json::object())
        : name_(std::move(actionName)), fn_(std::move(fn)), cfg_(std::move(cfg)) {}

    const char* type() const override { return "fn"; }
    std::string name() const override {
        return name_.empty() ? std::string("fn") : name_;
    }

    ActionResult apply(ParameterSpace& space,
                       const std::vector<double>& gradient,
                       const UpdateFn& updateSource) override;

    // 只序列化 name + 用户附带的 cfg；函数体不落盘。
    json config() const override;
    void setConfig(const json& j) override;
    std::unique_ptr<ActBase> clone() const override {
        return std::make_unique<FnAction>(*this);
    }

    void setFn(ActFn fn) { fn_ = std::move(fn); }
    bool hasFn() const noexcept { return static_cast<bool>(fn_); }

private:
    std::string name_;
    ActFn fn_;
    json cfg_ = json::object();
};

// ============================================================================
// ActionSpace - 有序动作容器
// ============================================================================

// 用户自定义动作类型的工厂（让自定义动作也能 JSON 往返）
using ActFactory = std::function<std::unique_ptr<ActBase>()>;

// 注册一个类型标签 -> 工厂。反序列化时按 type 标签查表重建。
AUROX_API void registerActionType(std::string typeTag, ActFactory factory);

// 工厂：从序列化形态重建单个动作（未知类型回退为 ParamUpdateAction）。
AUROX_API std::unique_ptr<ActBase> createAction(const json& serialized);

class AUROX_API ActionSpace {
public:
    // 默认构造：自带一个「参数更新」动作（id = 1）。
    ActionSpace();

    // ---- 更新源（优化器解耦） ----
    void setUpdateSource(UpdateFn fn) { updateSource_ = std::move(fn); }
    const UpdateFn& updateSource() const { return updateSource_; }
    bool hasUpdateSource() const noexcept { return static_cast<bool>(updateSource_); }

    // ---- 注册 ----
    // 接管所有权；id 为 kInvalidActionId 时自动分配。返回最终 id。
    ActionId add(std::unique_ptr<ActBase> a);

    // 就地构造：emplace<MoveCamera>("cam", ...)。返回裸指针（空间仍持有所有权）。
    template <typename T, typename... Args>
    T* emplace(Args&&... args) {
        auto p = std::make_unique<T>(std::forward<Args>(args)...);
        T* raw = p.get();
        add(std::unique_ptr<ActBase>(std::move(p)));
        return raw;
    }

    // lambda 快捷方式：addFn("clamp", [](ParameterSpace& s, auto&, auto&){...})
    FnAction* addFn(std::string name, ActFn fn, json cfg = json::object());

    // 预分配 id（供用户在构造动作时指定）
    ActionId nextId() noexcept { return nextId_++; }

    // ---- 查找 ----
    ActBase* get(std::string_view name);
    const ActBase* get(std::string_view name) const;
    ActBase* getById(ActionId id);
    const ActBase* getById(ActionId id) const;
    ActBase* at(std::size_t i);
    const ActBase* at(std::size_t i) const;

    ActBase* defaultAction() { return getById(defaultId_); }
    const ActBase* defaultAction() const { return getById(defaultId_); }
    ActionId defaultActionId() const noexcept { return defaultId_; }

    // ---- 容器 ----
    std::size_t size() const noexcept { return items_.size(); }
    bool empty() const noexcept { return items_.empty(); }
    std::vector<std::string> names() const;

    bool remove(std::string_view name);
    bool removeById(ActionId id);
    // 复位为「仅默认动作」。
    void clear();

    // ---- 执行 ----
    // 串行执行所有动作；遇到 Failed 立即停止。out 可拿到逐动作记录。
    ActionResult applyAll(ParameterSpace& space,
                          const std::vector<double>& gradient,
                          std::vector<ActOutcome>* out = nullptr);

    // 只执行指定动作（按名字或 id）。
    ActionResult applyOne(std::string_view name,
                          ParameterSpace& space,
                          const std::vector<double>& gradient,
                          ActOutcome* out = nullptr);
    ActionResult applyOne(ActionId id,
                          ParameterSpace& space,
                          const std::vector<double>& gradient,
                          ActOutcome* out = nullptr);

    // ---- 序列化 ----
    json toJson() const;
    void fromJson(const json& j);
    bool tryFromJson(const json& j) noexcept;
    // tryLoadJson<ActionSpace>(path) 复用 ParameterSpace.ipp 里的通用模板。

private:
    static constexpr ActionId kDefaultActionId = 1;

    void reindex();
    ActionResult runOne(ActBase& a,
                        ParameterSpace& space,
                        const std::vector<double>& gradient,
                        ActOutcome* out);

    UpdateFn updateSource_;
    std::vector<std::unique_ptr<ActBase>> items_;
    std::unordered_map<std::string, std::size_t> indexByName_;
    std::unordered_map<ActionId, std::size_t> indexById_;
    ActionId defaultId_ = kInvalidActionId;
    ActionId nextId_ = kDefaultActionId + 1;
};

using ASpace = ActionSpace;

// ============================================================================
// 宏（对齐 PARAM_REG / PARAM_BIND 的使用手感）
// ============================================================================

// ACT_TYPE(tag)：在叶子类里一行生成 type() 与 clone()。
// 注意：必须在最终派生类里使用（clone 按当前静态类型拷贝，中间基类会切片）。
#define ACT_TYPE(tag)                                                        \
    const char* type() const override { return (tag); }                      \
    std::unique_ptr<::AuroX::Space::ActBase> clone() const override {        \
        using Self = std::remove_cv_t<std::remove_reference_t<decltype(*this)>>; \
        return std::make_unique<Self>(*this);                                \
    }

// ACT_REG(space, ...)：注册块，块内可用 ACT_ADD（引用名 actSpace）。
#define ACT_REG(space, ...)                     \
    do {                                        \
        auto& actSpace = (space);               \
        __VA_ARGS__                             \
    } while (0)

// ACT_ADD(Type, ...)：在 ACT_REG 块内构造并添加一个动作，返回裸指针。
#define ACT_ADD(Type, ...) actSpace.emplace<Type>(__VA_ARGS__)

}  // namespace Space
}  // namespace AuroX
// 说明：动作类型与参数类型同属 AuroX::Space，因此沿用参数空间的命名空间别名
// PSpace 即可（PSpace::ActBase / PSpace::Param<double>）。ASpace 是 ActionSpace
// 的类型短别名，方便写 `ASpace actions;`。
