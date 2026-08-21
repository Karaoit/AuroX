# AuroX 项目说明（当前状态）

> 本文档说明 AuroX 项目**截至 2026-08-21 的实际状态**：已落地了哪些模块、如何构建与验证、还有哪些缺口。
> 它是一份「状态快照 / 上手指南」，与 `AuroX-General.md`（技术路线与架构设计）互为补充：**技术路线文档讲「要建成什么样」，本文档讲「现在建成了什么样」**。请不要修改 `AuroX-General.md` 与 `AuroX-General_技术路线.md`。

---

## 1. 项目定位（一句话）

AuroX 是一个 **C++20 通用闭环优化框架**：把视觉检测、PID 整定、主动视觉、机器人操作等任务，统一抽象为「状态 / 观测 / 参数 / 动作 / 结果 / 反馈 / 损失 / 约束 / 优化策略」，并在安全边界内主动搜索更优参数或动作。

规划中的完整架构有 10 层（L0 系统接口 → L9 领域插件）。当前已实现的是内核中段的**空间层（L3）、动力学层（L3/L5）、评估层（L6）、优化层（L7）、记忆层（L2）**。

---

## 2. 目录结构

```
auro-x/
├── AuroX-General.md              # 技术路线文档（不要修改）
├── AuroX-General_技术路线.md     # 技术路线文档副本
├── AuroX-项目说明.md             # 本文档
├── cleanBuild.bat                # 一键清理+重新配置构建
├── Library/                      # 第三方头文件（nlohmann/json、cpp-httplib）
└── AuroX/
    ├── CMakeLists.txt            # 构建：静态库 AuroX + 可执行 AuroX_Exe
    ├── include/AuroX/
    │   ├── Core.hpp              # 聚合头（Space/Memory/Optimizer/Evaluation，不含 Dynamics）
    │   ├── Space/                # 四个空间 + 编码器
    │   ├── Dynamics/             # 动力学层（需单独 include）
    │   ├── Evaluation/           # 评估器
    │   ├── Optimizer/            # 优化器 + 会话编排
    │   └── Memory/               # 运行历史
    ├── src/                      # 各层实现（.cpp，与 .hpp 声明分离）
    ├── apps/AuroXServer/
    │   ├── main.cpp              # 集成示例（项目当前入口）
    │   └── ServerUI/             # 前端 Operator Console 原型（HTML/JS/CSS）
    └── build/                    # CMake 生成物（可删除后重配）
```

---

## 3. 已实现的模块（可编译、可运行）

### L3 空间层（Space）
| 类 | 能力 |
|---|---|
| `ParameterSpace` | 声明参数（`declare` 连续 / `declareEnum` 枚举）；`vectorizeDouble` / `unvectorizeDouble` 把参数打包成优化器可用的连续向量；`toJson` / `fromJson` 序列化。枚举按选中索引 pack（占 1 个分量）。 |
| `ObservationSpace` | `ContinuousObservationChannel` / `DiscreteObservationChannel` / `CategoricalObservationChannel` / `RawVectorObservationChannel`；FIFO 缓冲、零填充、`ingest` 写入、`requestVector` 拉特征向量、`serialize` / `deserialize`。 |
| `ObservationEncoder` | 连续（minmax / zscore / none + 直方图分辨率）、离散（scalar / thermometer）、分类（one-hot + unknown 位）。纯 vectorize、无状态。 |
| `StateSpace` | 薄持有者：持有估计状态 x̂ + 名称 + 时间戳，`setState` / `getStateCopy` / `describe` / `reset`。 |
| `ActionSpace` | 可堆叠的动作：`ParameterUpdateAction`（优化器驱动的参数更新，默认）+ `CustomAction`（缺处理器时的自定义回退）。`apply(space, gradient)` 执行整条动作栈。 |

### L3/L5 动力学层（Dynamics，header-only）
> 注意：本层**未**被 `Core.hpp` 包含，使用时需显式 `#include "AuroX/Dynamics/Dynamics.hpp"`，且类型要全限定为 `AuroX::Dynamics::...`（或单独 `using`）。

| 类 | 能力 |
|---|---|
| `Dynamics::LinAlg` | 内部线性代数助手（矩阵-向量乘、矩阵乘、转置、加/减、单位阵、求逆、求解线性方程组）。命名空间 `AuroX::Dynamics::LinAlg`（原混乱的 `internal` 已改名）。 |
| `Dynamics::TransitionModel` | 转移模型抽象；`LinearTransitionModel`（x' = A x + B u，默认由辨识得到，可 `A()` / `B()` 取矩阵）；`WhiteBoxModel`（用户提供的 `std::function` 白盒物理钩子，作为自定义回退）。 |
| `Dynamics::StateEstimator` | 状态观测器：`PassThroughEstimator`（直通默认）、`LowPassEstimator`（指数平滑）、`KalmanEstimator`（线性卡尔曼，需 `configure(stateDim, obsDim, controlDim, A, B, C, Q, R)`）。 |
| `Dynamics::SystemIdentifier` | 系统辨识抽象；`LinearIdentifier`（最小二乘 / 法方程拟合 A、B），`addSample(x, u, x_next)` + `fit()` + `model()`。 |
| `Dynamics` | 编排器：连接 `ObservationSpace` 与 `StateSpace`；`step(u, ts)` 单步推进、`simulate(u)` 预测、`rollout(x0, us)` 多步、`identify()` 辨识、`useWhiteBox(n, m, fn)` 接入白盒、`setModel(...)` 替换模型。 |

### L6 评估层（Evaluation）
- `Evaluator` 抽象（纯虚 `evaluate(params)`、`dimension()`）。
- `RegressionEvaluator`：给定数据集 (X, y) 做回归损失，支持随机 mini-batch。
- 当无数据集时，由调用方提供自定义 `Evaluator` 子类（见 §5 的回退策略）。

### L7 优化层（Optimizer）
- `Optimizer` 抽象；`GradientDescent(lr)`、`SGD(lr, momentum, decay)`，工厂 `createOptimizer(type, config)`。
- `OptimizerSession`：把「空间 + 评估器 + 优化器 + 运行历史」串成一次完整优化；`run()` 返回 `Report{iterations, converged, diverged, dimensionMismatch, bestObjective, bestParams}`。
  - 关键约束：若 `evaluator.dimension() != space.vectorizeDouble().size()`，`run()` 会提前以 `dimensionMismatch=true` 退出（不优化）。所以评估器维度必须等于参数向量长度（含枚举分量）。

### L2 记忆层（Memory）
- `RunHistory`：记录损失曲线、最优参数、最近 K 组（环形缓冲）；`converged(tol, window)` / `diverged(maxObj, window)` 做收敛 / 发散兜底；`toJson` / `save` / `load` 支持跨轮 warm-start。

---

## 4. 当前入口示例：`apps/AuroXServer/main.cpp`

`main.cpp` 是一个**集成示例**（覆盖式重写，非 MVP 早期版本），演示全栈协同：

- **场景**：二阶被控对象（plant）的 PID 增益（Kp/Ki/Kd）闭环整定；`mode` 为枚举型控制器模式选择。
- **覆盖的层**：四个空间 + `Dynamics`（白盒 + Kalman + 辨识）+ 自定义 `Evaluator` + `GradientDescent`（经 `OptimizerSession`）+ `RunHistory`。
- **验证结果**（MSVC 14.51 / `cl.exe` `/std:c++20 /utf-8` 手动编译链接运行）：
  - 编译 / 链接 / 运行退出码均为 0；
  - 优化 500 轮，`converged=no / diverged=no`，最优损失 ≈ 3.2588；
  - 最优参数 Kp≈7.01 / Ki≈4.55 / Kd≈1.12 / mode=PID；
  - 闭环 10 步收敛；`LinearIdentifier` 拟合成功（A≈[0.902, 0.0517, -0.210, 0.957]，B≈[-0.00244, 0.290]）；
  - 序列化快照：`parameters: 4` / `observation channels: 2` / `actions: [parameter_update, custom]`。

> 该示例运行时会向工作目录写入 `custom_action_log.csv`（逐轮记录自定义动作应用的参数），属临时产物，可随时删除。

---

## 5. 自定义接口回退策略（设计要点）

框架遵循「**内置处理器缺失时，调用方提供自定义接口**」的原则。示例 `main.cpp` 演示了三处回退：

1. **动力学转移**：默认 `LinearTransitionModel`（来自辨识）；无模型时回退 `Dynamics::useWhiteBox()` 接入用户白盒物理函数。
2. **评估器**：默认 `RegressionEvaluator`（需 X/y 数据）；无数据时回退自定义 `CostFunction`（闭环仿真 + 中心差分求梯度）。
3. **动作**：默认 `ParameterUpdateAction`（优化器驱动）；缺命名处理器时回退 `CustomAction` 子类。

---

## 6. 构建方式

### 依赖
- C++20 编译器（MSVC 14.5x / Clang / GCC 均支持）。
- 第三方头文件：`Library/nlohmann/json`（JSON 序列化，已包含）；`Library/cpp-httplib`（HTTP 服务，已 include 但未接线）。

### CMake（推荐）
```bash
cd AuroX
cmake -B build -S .          # 首次或新增/移动 .cpp 后必须重跑（GLOB 扫描）
cmake --build build --config Debug
# 运行示例：
./build/AuroX_Exe/Debug/AuroX_Exe.exe
```
- 产出：`AuroX`（静态库）+ `AuroX_Exe`（可执行，入口 = `apps/AuroXServer/main.cpp`）。
- 顶层 `cleanBuild.bat` 可一键清理并重新配置。

### 手动编译（cl.exe，排查/验证用）
由于本机 `MSBuild.exe` 环境偶发崩溃，可用 `cl.exe` 直接编译验证：
```bash
CLBIN=".../VC/Tools/MSVC/14.51.36231/bin/Hostx64/x64/cl.exe"
# 设置 INCLUDE / LIB 指向 VC 工具集与 Windows SDK 10.0.26100.0
"$CLBIN" /nologo /EHsc /std:c++20 /utf-8 /I AuroX/include /I Library/nlohmann/json \
         /c /Foobj/ $(find AuroX/src -name '*.cpp') AuroX/apps/AuroXServer/main.cpp
"$CLBIN" /nologo /Feaurox_test.exe obj/*.obj
```

> **CMake 重要约定**：`CMakeLists.txt` 用 `file(GLOB_RECURSE src/*.cpp)` 收集源文件。**新增或移动 `.cpp` 后务必重跑 `cmake -B build -S .`**，否则 GLOB 扫不到新文件，会触发 `LNK2019` 未解析外部符号。

---

## 7. 已知缺口 / 下一步（尚未实现）

| 层 | 状态 | 说明 |
|---|---|---|
| L1 Safety | 部分 | 仅 `RunHistory::diverged()` 做发散兜底；缺独立 `SafetyManager`（约束检查、安全拒绝）。 |
| L4 Runtime | 未实现 | 任务生命周期 / 外环编排（区分「单次优化会话」与「跨轮任务」）。 |
| L8 Policy / Controller | 未实现 | `Dynamics` 提供状态持有、预测、辨识，但「求控制量 u」仍是调用方职责，无内置控制器。 |
| L0 系统接口 | 未实现 | 外部触发 / 命令总线的接入点。 |
| 前端后端 | 未接线 | `apps/AuroXServer/ServerUI/` 是较完整的 Operator Console 前端原型（HTML/JS/CSS），但 C++ 后端未实现：`/api/command`、`Operator` 注册表均未接线（cpp-httplib 已 include）。 |
| 枚举感知优化 | 后续 | 梯度优化器当前把枚举索引当数值处理；one-hot + 分类求解器为后续工作。 |
| 观测空间增强 | 后续 | 多速率对齐、缺失掩码（当前提供 `fillCount` / `isFull` + 零填充）。 |

---

## 8. 文档导航

| 文档 | 内容 |
|---|---|
| `AuroX-General.md` / `AuroX-General_技术路线.md` | 技术路线与 10 层架构设计（**不要修改**）。 |
| `AuroX-项目说明.md`（本文档） | 当前实现状态、构建、验证、缺口。 |
| `AuroX/include/AuroX/<层>/README*.md` | 各层 API 说明与最小示例（中英文均有）。 |
| `AuroX/AUTONOMOUS_BRAIN_FUNCTION_SPEC.md` | 自主大脑功能规格。 |

---

*状态快照时间：2026-08-21。如与代码不一致，以代码与对应层 README 为准。*
