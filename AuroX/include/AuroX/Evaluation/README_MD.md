# 核心层：Evaluation（评估 / 读取数据，Layer 4 的数据侧）

## 这一层是什么

`Evaluation` 层承载「读取数据与优化解耦」中的**第一部分：读取数据 / 评估**。
它与 `Optimizer` 层（第二部分：优化 / 更新）成对出现，二者通过 `OptimizerSession`
协作，但彼此解耦。

- `Evaluator` 只关心：给定参数向量 `std::vector<double>`，算出目标值 + 梯度；
- 它**完全不碰参数更新**，也**不依赖 `ParameterSpace`**（只接收参数向量），
  因此数据读取逻辑可以自由替换。

> 与 Space 层的边界：Space 描述「能调什么参数」；Evaluation 负责「给定参数，
> 数据说了什么（loss + 梯度）」。

## Evaluator（抽象基类 = 自定义接口）

```cpp
struct EvaluatorResult { double value; std::vector<double> gradient; };

class Evaluator {
    virtual EvaluatorResult evaluate(const std::vector<double>& params) const = 0; // 全量（GD）
    virtual size_t dimension() const = 0;
    virtual size_t sampleCount() const;        // N，SGD 用
    virtual size_t batchSize() const;         // mini-batch 大小，0=全量
    virtual bool supportsBatching() const;    // SGD 用
    virtual void beginEpoch();                // 新 epoch 洗牌钩子
    virtual std::vector<size_t> nextBatchIndices(size_t b);  // 下一个 mini-batch 下标
    virtual EvaluatorResult evaluateBatch(const std::vector<double>&, const std::vector<size_t>&) const; // 小批量
};
```

自定义评估器只需继承并实现 `evaluate()`（与可选的 batch 接口），即可接入优化闭环。

## RegressionEvaluator（MVP 参考实现）

多维线性回归，最小化均方误差 MSE：

```
loss(θ) = (1/N) Σ_i (x_i·θ − y_i)^2
∇loss   = (2/N) Xᵀ (Xθ − y)
```

- 全量（`stochastic=false`，配合 `GradientDescent`）→ 用全部 N 个样本算梯度；
- 小批量（`stochastic=true`，配合 `SGD`）→ 内部维护洗牌顺序，每次 `nextBatchIndices`
  返回一个 mini-batch，`evaluateBatch` 只在该批上算经验梯度。

构造：

```cpp
Evaluation::RegressionEvaluator evaluator(
    /*X=*/std::vector<std::vector<double>>{{1,0.5},{1,1.0},{1,1.5}},  // 行主序 N×D，第一列可放偏置 1
    /*y=*/std::vector<double>{1.0, 2.0, 3.0},
    /*stochastic=*/false,
    /*miniBatchSize=*/32,
    /*seed=*/1u);
```

## 文件位置

- 声明：`include/AuroX/Evaluation/Evaluator.hpp`
- 实现：`src/Evaluation/Evaluator.cpp`
- 由 `AuroX/Core.hpp` 的 `__has_include` 聚合，可直接 `#include "AuroX/Core.hpp"` 使用。
