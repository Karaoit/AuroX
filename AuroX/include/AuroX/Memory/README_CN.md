# 核心层：Memory（数据 & 记忆，Layer 2）

## 这个层是什么

`Memory` 层对应 `AuroX-General.md` 的 **Layer 2：Data & Memory**（episode / history / dataset / 经验库）。
它负责记录每次任务运行的数据，并将其沉淀为可复用经验。

在 **MVP 阶段**，本层只实现最小可用的运行历史 `RunHistory`：

- **内存优先**：所有数据常驻内存，不落盘；
- **可选导出**：调用方按需把快照写到指定路径的 JSON 文件；
- **不是持久化储存层**：Phase 3 的结构化 episode 库（结构化日志 + artifact + 相似度索引 + 跨任务迁移）不在此实现，待经验库与代理模型阶段再做。

> 与 Space 层的边界：Space（Layer 3）描述「能调什么参数」；Memory（Layer 2）记录「跑过什么」。
> 两者职责不同，因此 `RunHistory` 放在 `Memory/`，而非 `Space/`。

## RunHistory（MVP 运行历史）

`RunHistory` 与 `ParameterSpace` 解耦：它记录的是优化器友好的参数向量
（`std::vector<double>`，即 `ParameterSpace::vectorizeDouble()`），而非参数对象本身。

### 记录什么（对应 MVP 决策：不做全量历史储存）

- **逐轮标量**（`iteration` / `objective` / `gradientNorm` / `status` / `elapsedMs`），
  用于前端画 loss 曲线，内存开销极小；
- **最优参数只留 1 份**（`best()` / `bestParams()`）；
- **近期参数用容量受限的环形缓冲**（`recent()`，由构造参数 `recentCapacity` 控制），
  仅用于收敛 / 发散诊断，避免对无界高维向量做全量存储；
- **不落盘**：需要时由调用方调用 `save()` 触发一次性导出。

### 无界参数的安全兜底

当参数无界（无 `parameter_bounds`）时，安全层无法用边界拒绝。此时用
`diverged()` 检测 NaN/Inf 或 objective 爆炸，作为 Runtime Guard 的替代。

### 外部调用接口（由调用方驱动）

| 接口 | 作用 |
|---|---|
| `toJson()` | 导出 UI 友好的快照（`curve` + `best` + `recent`） |
| `fromJson(j)` | 从 `toJson()` 的快照重建内存状态 |
| `save(dir, name)` | 把 `toJson()` 写到 `<dir>/<name>.json`（目录不存在则创建）；数据仍在内存 |
| `load(dir, name)` | 读回 `save()` 写的快照，替换当前内容（可用于跨运行 warm-start） |

## 如何使用

```cpp
#include "AuroX/Memory/RunHistory.hpp"
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Memory;
using namespace AuroX::Space;

ParameterSpace space;
space.declare("w0", 0.0);   // 无界连续参数
space.declare("w1", 0.0);

// recentCapacity=20：保留最近 20 份完整参数向量用于诊断
RunHistory history(/*recentCapacity=*/20, /*minimize=*/true);

for (size_t it = 0; it < 100; ++it) {
    std::vector<double> theta = space.vectorizeDouble();
    double loss = evaluateRegression(theta);   // 用户的回归目标函数
    history.record(it, loss, &theta);
    // ... 优化器更新 theta 并 space.unvectorizeDouble(theta) ...
    if (history.diverged(/*maxObjective=*/1e6)) break;   // 无界参数发散兜底
}

auto best = history.bestParams();              // 最优参数向量
json snapshot = history.toJson();              // 给 Operator Console

// 可选：导出到指定 JSON 文件夹（调用方决定路径，模块不硬编码）
history.save("D:/runs/my_task", "history");    // -> D:/runs/my_task/history.json

// 可选：后续运行读回，作为 warm-start 起点
RunHistory restored;
if (restored.load("D:/runs/my_task", "history")) {
    auto prevBest = restored.bestParams();
}
```

### 收敛 / 发散判断

```cpp
if (history.converged(/*tolerance=*/1e-4, /*window=*/10)) { /* 已收敛 */ }
if (history.diverged(/*maxObjective=*/1e6, /*window=*/5))  { /* 已发散，需回退 best */ }
```

## 文件位置

- 声明（头文件）：`include/AuroX/Memory/RunHistory.hpp`
- 实现（源文件）：`src/Memory/RunHistory.cpp`
- 通过 `AuroX/Core.hpp` 的 `__has_include` 自动聚合，可直接 `#include "AuroX/Core.hpp"` 使用。

## 演进路线（何时再扩展本层）

满足以下任一，再按 `AuroX-General.md` §6 升级为完整 Data & Memory 层：

- 需要**跨运行 warm-start**（同一回归任务换数据集，复用历史最优）；
- 需要**失败案例库 / 参数敏感度沉淀**支撑 Phase 3 代理模型；
- 需要**可复现审计**（外部合规要求）。

届时再引入结构化 episode 日志、artifact 存储与相似度索引；对无界参数应只存
低秩摘要或最优配置，不存全量轨迹。
