// ============================================================================
// AuroX - ActionSpace 实现
// ----------------------------------------------------------------------------
// 与 include/AuroX/Space/ActionSpace.hpp 严格一一对应：
//   头文件只放声明，全部实现都在本文件（不做 header-only）。
//
// 一个必须知道的事实：空间内部存的是 Action*，指向**派生类自己的成员**。
// 因此拷贝出来的空间会指向另一个对象的成员，已在头文件里 delete 拷贝。
//
// 依赖：仅标准库。参数空间 / 状态空间 / 观测空间都不 include —— 需要时让闭包捕获。
// ============================================================================

#include "AuroX/Space/ActionSpace.hpp"

#include <algorithm>
#include <stdexcept>

namespace AuroX {
namespace Space {

// ── 自由函数 ─────────────────────────────────────────────────────────────
uint64_t hashOf(std::string_view s) {
    uint64_t h = 1469598103934665603ULL;                  // FNV-1a 64 offset basis
    for (char c : s) {
        h ^= static_cast<uint64_t>(static_cast<unsigned char>(c));
        h *= 1099511628211ULL;                            // FNV prime
    }
    return h;
}

double dist(const Vec& a, const Vec& b) {
    const std::size_t n = std::min(a.size(), b.size());
    double d = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double t = a[i] - b[i];
        d += t * t;
    }
    return d;                                             // 平方距离：只用于比较
}

// ── Condition ────────────────────────────────────────────────────────────
bool Condition::test(const Vec& x) const {
    return !ok || ok(x);                                  // 判据留空 = 恒真
}

// ── Action ───────────────────────────────────────────────────────────────
bool Action::can(const Vec& x) const {
    for (const Condition& c : conditions)
        if (!c.test(x)) return false;
    return true;
}

Vec Action::next(const Vec& x) const {
    return nxt ? nxt(x) : x;                              // 未写预测 = 状态不变
}

void Action::exec() const {
    if (run) run();
}

// ── ActionSpace ──────────────────────────────────────────────────────────
ActionSpace::ActionSpace(Vec s) : cur_(std::move(s)) {}

const Vec& ActionSpace::cur() const { return cur_; }

void ActionSpace::cur(const Vec& s) { cur_ = s; }

// 登记：自动补 hash；重复登记同一个对象直接忽略；hash 冲突直接抛，绝不静默覆盖
void ActionSpace::reg_(Action& a) {
    if (a.hash == 0) a.hash = hashOf(a.type + "/" + a.name);

    const auto it = byHash_.find(a.hash);
    if (it != byHash_.end()) {
        if (it->second == &a) return;
        throw std::runtime_error("ActionSpace: hash 冲突 -> " + a.type + "/" + a.name);
    }
    byHash_.emplace(a.hash, &a);
    order_.push_back(a.hash);                             // 保持声明顺序
}

Action* ActionSpace::get(uint64_t h) {
    const auto it = byHash_.find(h);
    return it == byHash_.end() ? nullptr : it->second;
}

Action* ActionSpace::get(const Str& nm) {
    for (uint64_t h : order_) {
        Action* a = get(h);
        if (a && a->name == nm) return a;
    }
    return nullptr;
}

std::size_t ActionSpace::size() const { return byHash_.size(); }

// 执行：条件门 → 执行体 → 状态按预测推进
bool ActionSpace::run(uint64_t h) {
    if (banned(h))          return false;                 // 已禁用
    Action* a = get(h);
    if (!a)                 return false;                 // 不存在
    if (!a->can(cur_))      return false;                 // 条件未全满足
    a->exec();                                            // 执行（闭包内捕获一切）
    cur_ = a->next(cur_);                                 // 状态按预测推进
    return true;
}

bool ActionSpace::run(const Str& nm) {
    Action* a = get(nm);
    return a ? run(a->hash) : false;
}

void ActionSpace::ban   (uint64_t h) { ban_.insert(h); }
void ActionSpace::unban (uint64_t h) { ban_.erase(h);  }
bool ActionSpace::banned(uint64_t h) const { return ban_.count(h) != 0; }

// 选链：前向 BFS。条件不满足就不成边，所以搜出来的每一条链都是物理上走得通的
Chain ActionSpace::plan(const Vec& goal, int maxDep, double eps, bool asGraph) {
    Chain out;

    struct Node { Vec x; int par; uint64_t h; };

    std::vector<Node> seen{ { cur_, -1, 0 } };            // 起点 = 用户给的当前状态
    if (dist(seen[0].x, goal) <= eps) { out.ok = true; return out; }   // 已在目标

    std::vector<int> layer{ 0 };
    for (int d = 0; d < maxDep && !layer.empty(); ++d) {
        std::vector<int> next;

        for (int i : layer)
        for (uint64_t h : order_) {
            if (banned(h)) continue;                      // ① 禁用集
            Action* a = get(h);
            if (!a) continue;
            if (!a->can(seen[i].x)) continue;             // ② 条件不满足 → 不成边
            const Vec nx = a->next(seen[i].x);            // ③ 预测下一步

            bool dup = false;                             // ④ 去重：同一状态不重复展开
            for (const Node& n : seen)
                if (dist(n.x, nx) <= eps) { dup = true; break; }
            if (dup) continue;

            seen.push_back({ nx, i, h });
            const int k = static_cast<int>(seen.size()) - 1;

            if (dist(nx, goal) <= eps) {                  // ⑤ 命中 → 回溯成链
                std::vector<uint64_t> r;
                for (int p = k; p > 0; p = seen[p].par) r.push_back(seen[p].h);
                out.acts.assign(r.rbegin(), r.rend());
                out.ok = true;
                return out;
            }
            next.push_back(k);
        }
        layer = std::move(next);
    }

    if (asGraph) {                                        // 没命中也要交付「图」
        for (std::size_t i = 0; i < seen.size(); ++i)
            if (seen[i].par >= 0)
                out.edges.push_back({ seen[i].par, static_cast<int>(i) });
    }
    out.why = "unreachable within depth " + std::to_string(maxDep);
    return out;
}

// 重规划 == 换一组禁用，重跑同一次 plan。链/图本身不可变，无需「改图」
Chain ActionSpace::replan(const Vec& goal, uint64_t culprit, int maxDep, double eps) {
    ban(culprit);
    return plan(goal, maxDep, eps);
}

} // namespace Space
} // namespace AuroX
