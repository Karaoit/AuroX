# 核心层：Optimizer（优化，Layer 4）

## 这一层是什么

`Optimizer` 层对应 `AuroX-General.md` 的 **Layer 4：Optimizer / Runtime**。它的职责是
**根据梯度更新参数**，并作为闭环的编排者，把「参数空间 / 评估器 / 优化器 / 历史」串起来。

### 设计核心：读取数据与优化解耦成两部分

| 部分 | 所在层 | 职责 | 依赖 |
|---|---|---|---|
| **读取数据 / 评估** | `Evaluation`（`Evaluator`） | 读数据、算目标值 + 梯度 | 只接收 `std::vector<double>` 参数向量，不碰参数更新 |
| **优化 / 更新** | `Optimizer`（本层） | 只根据梯度更新参数 | 只接收梯度向量，不读任何数据 |

两者通过 `OptimizerSession` 协作，但彼此完全解耦：换评估器不影响优化器，换优化器
不影响数据读取。这正是用户要求的「将读取数据和优化解耦成两个部分」。

```
        ParameterSpace (Space 层)  ── vectorizeDouble() ──▶
                                                            │
   OptimizerSession  ── evaluate(theta) ──▶  Evaluator (Evaluation 层)
   (Optimizer 层, 编排)                          │ 读取数据：loss + ∇
                                                 ▼
                                       Optimizer.update(space, ∇)  ◀── 优化部分
                                                 │
                                                 ▼
                                          RunHistory (Memory 层)  ◀── 记录曲线/最优
```

## 优化器（默认 GD，可选 SGD）

| 优化器 | 默认？ | 说明 |
|---|---|---|
| `GradientDescent` | ✅ 默认 | 全批量，θ ← θ − lr·∇。MVP 最稳起点 |
| `SGD` | 可选 | 动量 + 学习率按 epoch 衰减；需配合 `stochastic` 评估器 |

两者都继承 `Optimizer` 抽象基类——这就是**自定义接口**：

```cpp
class MyOptimizer : public AuroX::Optimizer::Optimizer {
public:
    void update(ParameterSpace& space, const std::vector<double>& g) override {
        auto theta = space.vectorizeDouble();
        for (size_t i = 0; i < theta.size(); ++i) theta[i] -= lr_ * g[i]; // 你的更新规则
        space.unvectorizeDouble(theta);
    }
    const char* name() const override { return "MyOptimizer"; }
    bool isStochastic() const override { return false; }  // true 则走小批量评估
};
```

## OptimizerSession（闭环编排器）

`OptimizerSession` 把四者串成训练循环，只调度、不含算法：

- 每轮：取参数 → `Evaluator` 评估（读取数据）→ `Optimizer.update`（优化）→ 写 `RunHistory`；
- 依据 `optimizer.isStochastic()` 自动切换「全量评估」或「小批量评估」路径；
- 用 `RunHistory::converged()/diverged()` 做早停（无界参数的安全兜底）。

## 如何使用（MVP 多维回归，默认 GD）

```cpp
#include "AuroX/Core.hpp"
using namespace AuroX;

// 1) 参数空间（无界连续参数，MVP 回归）：3 个权重 = 偏置 + 2 个特征权重
Space::ParameterSpace space;
space.declare("w0", 0.0);   // 偏置 (intercept)
space.declare("w1", 0.0);   // 特征1权重
space.declare("w2", 0.0);   // 特征2权重

// 2) 评估器（读取数据部分）：3 样本、3 维（偏置列1 + 2 个特征），维度须与参数空间一致
Evaluation::RegressionEvaluator evaluator(
    /*X=*/{{1, 0.5, 0.2}, {1, 1.0, 0.3}, {1, 1.5, 0.4}},
    /*y=*/{0.9, 1.35, 1.8},
    /*stochastic=*/false);

// 3) 优化器（默认梯度下降）
Optimizer::GradientDescent optimizer(/*lr=*/0.1);

// 4) 历史（内存优先，recentCapacity=20 环形缓冲）
Memory::RunHistory history(/*recentCapacity=*/20, /*minimize=*/true);

// 5) 闭环编排
Optimizer::OptimizerSession session(space, evaluator, optimizer, history,
                                    /*maxIterations=*/500, /*tolerance=*/1e-6);
auto rep = session.run();

// 结果
json curve = history.toJson();          // 给 Operator Console 画 loss 曲线
auto best = history.bestParams();       // 最优参数向量
```

## 切换到 SGD（可选）

只需换两行——评估器开启 `stochastic`，优化器换成 `SGD`：

```cpp
Evaluation::RegressionEvaluator evaluator(X, y, /*stochastic=*/true, /*miniBatchSize=*/2);
Optimizer::SGD optimizer(/*lr=*/0.1, /*momentum=*/0.9, /*decay=*/0.0);
Optimizer::OptimizerSession session(space, evaluator, optimizer, history);
session.run();   // Session 检测到 isStochastic()=true，自动走小批量评估路径
```

> 要点：SGD 的「随机」来自评估器的 mini-batch 采样，优化器只负责更新规则。
> 这与 PyTorch 等框架一致（优化器 + DataLoader 分离）。

## 自定义评估器（自定义接口）

实现一个 `Evaluator` 子类即可接入闭环，无需改动优化器或空间：

```cpp
class MyEvaluator : public Evaluation::Evaluator {
public:
    size_t dimension() const override { return 3; }
    EvaluatorResult evaluate(const std::vector<double>& p) const override {
        EvaluatorResult r;
        r.gradient.assign(3, 0.0);
        r.value = p[0]*p[0] + p[1]*p[1] + p[2]*p[2];   // 例：最小化 ||p||²
        r.gradient[0] = 2*p[0]; r.gradient[1] = 2*p[1]; r.gradient[2] = 2*p[2];
        return r;
    }
};
```

## 文件位置

- 声明（优化器）：`include/AuroX/Optimizer/Optimizer.hpp`（header-only，无 `.cpp`）
- 声明（编排器）：`include/AuroX/Optimizer/OptimizerSession.hpp`
- 实现（编排器）：`src/Optimizer/OptimizerSession.cpp`
- 评估器：`include/AuroX/Evaluation/Evaluator.hpp` + `src/Evaluation/Evaluator.cpp`
- 通过 `AuroX/Core.hpp` 的 `__has_include` 自动聚合，可直接 `#include "AuroX/Core.hpp"` 使用。

## 约束与边界

- 评估器维度（`Evaluator::dimension()`）必须与 `ParameterSpace::vectorizeDouble()`
  长度一致，否则 `OptimizerSession::run()` 返回 `dimensionMismatch=true` 并拒绝运行。
- 参数空间应全部为可向量化的数值参数（无界连续回归场景天然满足）。
- `Optimizer` 层**不**做持久化；历史仍由 `Memory/RunHistory` 内存持有（见该层说明）。
- Phase 3 的代理模型 / 经验库不在此层；本层只负责单任务闭环的优化与编排。
