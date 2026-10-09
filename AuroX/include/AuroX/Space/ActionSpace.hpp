#pragma once

// ============================================================================
// AuroX - ActionSpace
// ----------------------------------------------------------------------------
// 设计语言：如非必要，勿增实体
//
//   4 个类型    Condition / Action / Chain / ActionSpace
//   2 个自由函数 hashOf / dist
//   0 个宏
//   只依赖标准库
//
// 参数空间 / 状态空间 / 观测空间**不属于**动作层：需要它们时让闭包捕获即可
// （RunFn / NxtFn 都是 std::function）。动作层只认识一个中性状态向量 Vec。
//
// File layout (declaration / implementation split):
//   include/AuroX/Space/ActionSpace.hpp  - declarations only (this file)
//   src/Space/ActionSpace.cpp            - all implementations
//
// 用法三档是**包含关系**，不是并列：
//   档3 简易   Action{name,type,run} + reg + run    → 动作空间退化为动作集合
//   档1 标准   + conditions + nxt + plan            → 据当前/目标状态自动选链
//   档2 高级   + ban + replan                       → 状态异常时重规划
// ============================================================================

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
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

using Str = std::string;
using Vec = std::vector<double>;        // 中性状态向量：装什么由用户决定

// ── 自由函数 ─────────────────────────────────────────────────────────────
AUROX_API uint64_t hashOf(std::string_view s);          // FNV-1a 64，全局唯一键
AUROX_API double   dist  (const Vec& a, const Vec& b);  // 平方距离（不开根，只比大小）

// ── 1. Condition —— 一条判据 ────────────────────────────────────────────
struct AUROX_API Condition {
    Str                             desc;   // 给人看
    std::function<bool(const Vec&)> ok;     // 给机器看；留空 = 恒真

    bool test(const Vec& x) const;
};

// ── 2. Action —— 基础结构体（纯数据）───────────────────────────────────
using RunFn = std::function<void()>;               // 执行体：无参，外部函数由闭包包装
using NxtFn = std::function<Vec(const Vec&)>;      // 结束状态预测

struct AUROX_API Action {
    Str                    name;        // 动作名
    Str                    type;        // 动作族（与 name 一起决定 hash）
    uint64_t               hash = 0;    // 全局唯一键；留 0 → reg 时自动派生
    RunFn                  run;         // 执行函数
    std::vector<Condition> conditions;  // 条件向量：与系统状态结合判断能否执行
    NxtFn                  nxt;         // 结束状态预测；留空 = 状态不变

    bool can (const Vec& x) const;      // 条件是否全满足
    Vec  next(const Vec& x) const;      // 预测：x --本动作--> ?
    void exec() const;                  // 执行（run 为空则什么都不做）
};
using Act = Action;                     // 手感别名

// ── 3. Chain —— 选链产物 ────────────────────────────────────────────────
struct AUROX_API Chain {
    bool                             ok = false;
    Str                              why;    // 失败原因
    std::vector<uint64_t>            acts;   // 动作 hash 序列（链）
    std::vector<std::pair<int, int>> edges;  // 仅 asGraph=true 时填（图）
};

// ── 4. ActionSpace ───────────────────────────────────────────────────────
class AUROX_API ActionSpace {
public:
    ActionSpace() = default;
    explicit ActionSpace(Vec s);

    // 内部存的是指向「派生类自己成员」的 Action*，拷贝出来会指向另一个对象 → 禁用
    ActionSpace(const ActionSpace&)            = delete;
    ActionSpace& operator=(const ActionSpace&) = delete;

    // 登记：把派生类里的 Action 成员一次交给空间，不是逐个 add。
    // 唯一一处模板：只做一次转发，因此函数体必须留在头文件里。
    template <class... A> void reg(A&... as) { (reg_(as), ...); }

    // 状态（由用户写入；动作层不生产状态，只消费）
    const Vec& cur() const;
    void       cur(const Vec& s);

    // 查找
    Action*     get(uint64_t h);
    Action*     get(const Str& nm);     // 按名线性扫描
    std::size_t size() const;

    // 执行：条件全过才执行，随后状态按预测推进
    bool run(uint64_t h);
    bool run(const Str& nm);

    // 禁用集
    void ban   (uint64_t h);
    void unban (uint64_t h);
    bool banned(uint64_t h) const;

    // 选链（唯一可覆写点）
    virtual Chain plan(const Vec& goal, int maxDep = 8, double eps = 1e-6,
                       bool asGraph = false);

    // 重规划 == 换一组禁用，重跑同一次 plan（不改图）
    Chain replan(const Vec& goal, uint64_t culprit, int maxDep = 8, double eps = 1e-6);

    virtual ~ActionSpace() = default;

private:
    void reg_(Action& a);

    Vec                                 cur_;      // 当前状态
    std::unordered_map<uint64_t, Action*> byHash_; // hash → 动作（零拷贝）
    std::vector<uint64_t>               order_;    // 声明顺序（结果可复现）
    std::unordered_set<uint64_t>        ban_;      // 禁用集
};

using ASpace = ActionSpace;             // 手感别名

} // namespace Space
} // namespace AuroX
