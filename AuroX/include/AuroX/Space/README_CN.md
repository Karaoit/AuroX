# 核心层：Space（参数空间）

## 这个层实现什么

`Space` 层提供统一的**参数空间（ParameterSpace）**抽象，用于描述、存储、校验和序列化一组命名参数。它同时满足 MVP 的要求：

- **标准类型表达**：参数值直接用 C++ 标准类型表达——`bool` / `int` / `float` / `double` / `std::string`，以及一个**命名枚举（enum）类型**，无需自定义包装。
- **连续 / 离散 / 枚举三类参数**：
  - 连续型：带上下界（`lower` / `upper`），例如学习率、批量大小；
  - 离散型：带有限候选集（`choices`），例如优化器名称 `{adam, sgd, rmsprop}`；
  - 枚举型（enum）：带**命名标签集**的离散变量，例如颜色 `{red, green, blue}`；内部以"选中标签的索引"存储与向量化（见下文「枚举类型参数」）。
- **可扩展基类**：`ParameterBase` 是抽象基类，调用方可以派生自己的参数类型（如结构体参数），通过 `add(std::make_unique<MyParam>(...))` 注册即可，框架零改动。
- **完整序列化**：基于 `nlohmann::json`，可以把整个参数空间序列化为配置、从配置反序列化，便于持久化与网络传输。
- **向量化 / 反向量化**（供优化器迭代）：
  - `vectorize<T>()`：把同种类型的参数收集到 `std::vector<T>`；
  - `vectorizeDouble()`：把所有数值型参数打包成统一的 `std::vector<double>`（对优化器最友好）；
  - `unvectorize*()`：把优化器更新后的连续向量写回参数对象；
  - `toJson()` / `valueToJson()`：把结构化参数暴露给前端修改。

核心类型：
- `ParameterBase`：参数抽象基类（纯虚接口：命名、类别、类型、克隆、边界校验、序列化、向量化）。
- `TypedParameter<T>`：内置模板参数类，覆盖 `bool/int/float/double/std::string`，用户通常无需自写。
- `ParameterSpace`：持有所有参数（`std::unique_ptr`），提供 `add` / `get` / `declare` / 序列化 / 向量化。

> 运行历史 `RunHistory` 属于 **Memory 层**（Layer 2: Data & Memory），不在 Space 层。
> 详见 `include/AuroX/Memory/README.md`。


## 如何使用

### 1. 创建空间并声明参数

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Space;

ParameterSpace space;

// 连续型（带上下界）
auto* lr = space.declare("learning_rate", 0.01f, 0.001f, 0.1f);
auto* bs = space.declare("batch_size",    32,    1,      128);

// 离散型（候选集）
auto* opt = space.declareDiscrete("optimizer", std::string("adam"),
                                  {std::string("adam"), std::string("sgd"), std::string("rmsprop")});

// 布尔型
auto* drop = space.declare("use_dropout", true, false, true);

// 无界连续型（约束交给后续安全层）
auto* temp = space.declare("temperature", 1.0);
```

### 2. 访问与修改参数

```cpp
// 直接通过声明返回的原始指针
lr->value = 0.05f;

// 通过名字查找
ParameterBase* p = space.get("batch_size");

// 类型安全访问（失败返回 nullptr）
if (auto* b = space.getAs<int>("batch_size")) {
    b->value = 64;
}

// 边界 / 合法性校验（连续查边界，离散查候选集）
bool ok = p->isWithinBounds();
```

### 3. 序列化与反序列化

```cpp
// 整个空间 -> JSON
json config = space.toJson();

// 从 JSON 重建整个空间（不存在的参数会被创建，重名会被覆盖）
ParameterSpace restored;
restored.fromJson(config);
```

### 4. 向量化（交给优化器迭代）

```cpp
// 统一 double 向量
std::vector<double> vec = space.vectorizeDouble();

// 优化器修改 vec 后写回
space.unvectorizeDouble(vec);

// 按同类型收集
std::vector<float> floats = space.vectorize<float>();
space.unvectorize(floats);
```

### 5. 自定义参数类型（可选）

继承 `ParameterBase`，实现全部虚函数，然后用 `add()` 注册：

```cpp
class MyParam : public ParameterBase {
    // 实现 name/kind/typeId/typeName/clone/isWithinBounds/toJson/fromJson/
    //        valueToJson/setValueFromJson/packDouble/unpackDouble/componentCount
};
space.add(std::make_unique<MyParam>(/* ... */));
```

### 6. 枚举（分类）类型参数

`declareEnum` 声明一个**命名标签**的离散参数（区别于 `declareDiscrete` 的"数值候选集"）。它内部保存"选中标签的索引"，类别为 `Discrete`，类型 id 为 `Enum`；向量化时按索引打包成单个 `double`，因此无缝接入 `vectorizeDouble()` / `unvectorizeDouble()`。

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Space;

ParameterSpace space;
space.declare("w0", 0.0);

// 枚举：标签集 {red, green, blue}，默认选中 green（第 1 个，索引从 0 起）
auto* color = space.declareEnum("color", std::string("green"),
                                {"red", "green", "blue"});

color->label();          // "green"
color->index();          // 1
color->setLabel("blue"); // 切换到 blue -> index()==2

// 未知标签会自动扩展标签集（保持可表示）
color->setLabel("yellow"); // 标签集变为 4 个，index()==3

// 向量化：w0(无界) + color 索引 = 长度 2 的 double 向量
auto v = space.vectorizeDouble();   // v[1] == 1.0 (green)
v[1] = 0.0;
space.unvectorizeDouble(v);          // 写回 -> color 变为 red

// 类型安全查找
if (auto* c = space.getEnum("color")) {
    std::cout << c->label() << "\n";
}

// 序列化往返：type 字段为 "enum"，附带 labels / value / label
json cfg = space.toJson();
ParameterSpace restored;
restored.fromJson(cfg);              // 完整重建枚举（含标签集与选中项）
```

> MVP 限制：梯度优化器会把枚举索引当作数值处理。真正的分类感知优化（如 one-hot + 分类求解器）是后续工作；当前枚举按"纯索引标量"打包/解包。

## 最小示例

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
#include <iostream>

int main() {
    AuroX::Space::ParameterSpace space;
    space.declare("learning_rate", 0.01f, 0.001f, 0.1f);
    space.declare("batch_size",    32,    1,      128);

    if (auto* lr = space.getAs<float>("learning_rate")) {
        std::cout << "before: " << lr->value << "\n";
        lr->value = 0.05f;
        std::cout << "after : " << lr->value << "\n";
    }

    std::cout << space.toJson().dump(4) << "\n";

    auto vec = space.vectorizeDouble();
    vec[0] = 0.02;                 // 优化器把学习率改成 0.02
    space.unvectorizeDouble(vec);
    std::cout << "updated lr: " << space.getAs<float>("learning_rate")->value << "\n";
    return 0;
}
```

## 文件位置与代码组织

`ParameterSpace` 采用"声明 / 实现分离"的三文件组织：

| 文件 | 内容 | 说明 |
|---|---|---|
| `include/AuroX/Space/ParameterSpace.hpp` | **仅声明** | 所有类的成员函数只有声明，不含函数体（`AUROX_PARAMS` / `PARAM_REG` / `PARAM_BIND` 宏除外——它们在用户派生类内部展开生成代码，必须留在头文件） |
| `include/AuroX/Space/ParameterSpace.ipp` | **模板实现** | `NumericTraits` 特化、`TypedParameter<T>`、`ParameterSpace::getAs/vectorize/unvectorize`、`Param<T>`、`PStruct<Derived>` 的实现。模板代码必须对每个编译单元可见，无法放进 cpp，故拆到此文件；由 hpp 末尾自动 `#include`，**不要直接包含** |
| `src/Space/ParameterSpace.cpp` | **非模板实现** | `ParameterBase` 默认虚函数、`EnumParameter`、`ParameterSpace`、`EnumParam` 的全部实现，随 DLL 编译 |

新增非模板实现一律写入 `ParameterSpace.cpp`；新增模板实现写入 `ParameterSpace.ipp`。

- 声明（头文件）：`include/AuroX/Space/ParameterSpace.hpp`
- 模板实现（随头文件分发）：`include/AuroX/Space/ParameterSpace.ipp`
- 非模板实现（源文件）：`src/Space/ParameterSpace.cpp`

---

## 动作空间 ActionSpace（位于 Space 层）

### 它解决什么问题

`ActionSpace` 是插在**「优化器输出（梯度）」与「参数真正更新」之间**的决策面。它把"优化器给出的更新"从"必须执行的步"降级为"众多候选动作中的一个默认动作"，并允许在其后**串行叠加**其它动作（例如尚未实现的状态转移、移动镜头、移动电机等），最终一起作用到参数向量上。

这一层完全对应之前的设计讨论：
- 优化器只负责算出参数的**增量（delta）**——即"判断依据来自优化器"；
- 默认动作集合**只含参数更新**；
- 自定义动作可串行叠加；
- 一切都能 `toJson` / `fromJson`，前端无需后端即可重建整条任务数据流。

### 分层与解耦

`ActionSpace` 属于 Space 层，**只依赖 `ParameterSpace` 与 json**，不直接 `#include` 优化器头文件。所谓"判断依据来自优化器"，是在运行时通过**回调（ParameterUpdateFn）**注入的：编排器（`OptimizerSession`）把 `Optimizer::computeUpdate` 接成 `ActionSpace` 的更新源。这样 Space 层与 Optimizer 层互不反向依赖（避免循环 include），默认动作却仍是优化器驱动的。

### 核心类型

- `Action`（抽象基类，即自定义接口）：实现 `type()` / `name()` / `feasible()` / `apply()` / `clone()` / `config()` / `setConfig()` 即可插入任意动作。
- `ParameterUpdateAction`（默认动作）：`apply` 时调用更新源拿到 delta 并写回参数；未接线时自动退化为"默认学习率的梯度步"，保证独立可用。
- `CustomAction`（占位自定义动作）：表示尚未实现的动作（如状态转移、移动镜头、电机指令）。它**不执行真实硬件**，只承载数据：
  - `params`：任意动作参数（如目标位姿、速度）；
  - `state`：附带（尚未实现的）状态空间描述；
  - `transition`：附带的状态转移方程 / 模型描述；
  - `paramDelta`：可选的演示增量，若提供则在 `apply` 时叠加到参数向量（为空则为纯占位、无副作用）。
  
  这正是"可添加还未实现的状态空间与状态转移方程或参数"的落地方式——以**数据**而非执行代码的形式存在，前端可直接从 JSON 渲染。
- `ActionSpace`：持有有序动作列表。默认集合**只含参数更新**；`addAction()` 在其后串行叠加；`apply()` 按序依次作用；`clearCustom()` 复位为默认集合。

### 串行叠加示例

```cpp
#include "AuroX/Space/ActionSpace.hpp"
using namespace AuroX::Space;

ActionSpace actions;

// 默认已含：参数更新（判断依据来自优化器，由 OptimizerSession 接线）

// 串行叠加一个"移动相机"自定义动作（状态空间/转移方程目前只是数据占位）
actions.addAction(std::make_unique<CustomAction>(
    /*name=*/ "move_camera",
    /*params=*/ json{{"target", {0.0, 1.0, 0.0}}, {"speed", 0.5}},
    /*state=*/ json{{"space", "camera_pose"}, {"dim", 3}},
    /*transition=*/ json{{"equation", "x_{k+1} = f(x_k, u_k)"}, {"model", "unimplemented"}},
    /*paramDelta=*/ {}   // 空 = 纯占位；可填与参数同维的向量做演示
));
```

迭代时（已接入 `OptimizerSession`）：

```cpp
AuroX::Optimizer::OptimizerSession session(space, evaluator, optimizer, history);
session.setActionSpace(&actions);   // 接入动作空间；传 nullptr 则退化为原行为
auto rep = session.run();
```

### 序列化与反序列化（前后端解耦关键）

```cpp
// 整个动作空间 -> JSON（前端可据此绘制动作流水线、重建数据流）
json a = actions.toJson();

// 从 JSON 重建（按 type 标签走工厂 createAction，默认动作集合永不为空）
ActionSpace restored;
restored.fromJson(a);
```

序列化形态：

```json
{
  "version": 1,
  "actions": [
    { "type": "parameter_update", "name": "parameter_update", "config": {} },
    { "type": "custom", "name": "move_camera",
      "config": {
        "params":     { "target": [0,1,0], "speed": 0.5 },
        "state":      { "space": "camera_pose", "dim": 3 },
        "transition": { "equation": "x_{k+1} = f(x_k, u_k)", "model": "unimplemented" }
      }
    }
  ]
}
```

### 文件位置

- 声明（头文件）：`include/AuroX/Space/ActionSpace.hpp`
- 实现（源文件）：`src/Space/ActionSpace.cpp`
- 接入编排：`include/AuroX/Optimizer/OptimizerSession.hpp`（`setActionSpace`）

---

## 观测空间 ObservationSpace（位于 Space 层）

### 它解决什么问题

观测空间负责**捕获外界输入的所有可量化信息、向量化、并按时间维持有**，随时把当前观测窗口交给状态空间（StateSpace）做估计/辨识。它是设计文档里"系统辨识器 / 系统建模器"的**输入侧**——把"环境吐出的可量化信息 y"统一成优化器/模型可读的 `vector<double>` 语言，与 `ParameterSpace`（我们注入的控制量 u）形成对偶。

### 分层与文件划分

| 文件 | 职责 |
|---|---|
| `ObservationEncoder.hpp` | 三种**默认编码器**（连续 / 离散 / 分类枚举）。纯 `vectorize`，**无状态、无序列化、无反向量化、只吃输入**。 |
| `ObservationSpace.hpp` | `ObservationChannel` 基类 + 具体信道（连续/离散/分类/RawVector）+ `ObservationSpace` 容器。信道有状态（持有 FIFO），**信道可序列化**。 |

> 两个文件均为 **header-only**（实现写在头文件内）。这是刻意的：本项目 CMake 用 `file(GLOB_RECURSE src/*.cpp)` 扫描源文件，**新增 `.cpp` 必须重新跑一次 `cmake` 配置**才会被纳入（否则会出现 LNK2019）。把观测空间做成头文件内联，就**完全绕开了这个坑**，现有构建保持绿色，无需重新配置。

### 默认编码方式（仅向量化）

编码器是**无状态的纯函数**：输入 → `vector<double>`，不持有任何成员、不做序列化/反向量化。

- **连续变量 `ContinuousEncoder`**
  - 归一化模式：`minmax`（[min,max]→[0,1]，越界 clamp）/ `zscore`（(x-mean)/std，clamp 到 [clampLo,clampHi]）/ `none`（透传）。
  - `resolution r`：默认 1（单个归一化标量）；若 `r>1` 则输出 `r` 长的**软直方图**（三角核分箱），适合低分辨率观测。
- **离散变量 `DiscreteEncoder`**
  - 模式：`scalar`（默认，把等级映射到 [0,1] 的 1 长向量）/ `thermometer`（`r=levels` 的有序 one-hot，保留离散间的序关系）。
  - `levels`：有序允许值集合；为空则退化为恒等（直接当数值）。
- **分类（枚举）变量 `CategoricalEncoder`**
  - 输出 `r=K+1` 的 **one-hot**：`K` 个已知标签 + 1 个 **unknown 位**，未见过标签点亮 unknown 位而非被当成合法类别（避免全零歧义）。

### 信道（Channel）语义

`ObservationChannel` 基类绑定**一个外部输入**并直接向量化，关键属性：

- **向量长度 = 分辨率 `r` × 时间宽度 `T`**（每样本 `r` 个数，保留 `T` 个时间步）。
- **FIFO 环形缓冲**：每样本 `r` 个数量化值，先进先出，超出 `T` 丢最旧。
- **拉模型请求**：`ObservationSpace::requestVector(id)` —— **不管何时，状态空间来拉就返回当前扁平向量**；**未填满的空位一律置零**。
- **可序列化 / 反序列化**：信道元数据（id/name/kind/resolution/timeWidth/编码器配置）+ 当前 FIFO 窗口，便于落盘、warm-start、日志。编码器本身**不序列化**（它是无状态配置，由信道描述）。
- **可选前馈补偿 `ObservationCompensator`**：默认 `NoOpCompensator`；用户派生实现 `apply(vector<double>& window)`，在 **post-encode / pre-output** 阶段作用于整段 `r×T` 窗口（如去偏置、去趋势、抵消已知指令效应）。注意：每次拉取都会调用，有状态补偿器需幂等或自行设计。

具体信道（同文件）：`ContinuousObservationChannel` / `DiscreteObservationChannel` / `CategoricalObservationChannel`（分别吃 `double` / `double` / `string`），以及 `RawVectorObservationChannel`（绑定**已经是向量**的输入，如嵌入/特征，`resolution` = 其固有长度，直接入 FIFO 不再编码）。

### 容器 `ObservationSpace`

- `addChannel(...)`：注册一个已构造的信道（空间取得所有权）。
- `ingest(id, value, ts)`：三个重载（`double` / `string` / `vector<double>`），自动路由到对应信道；类型不匹配则安全忽略。
- `requestVector(id)`：拉取单信道当前窗口（零填充）。
- `collect()`：把所有信道**拼接**成一个组合观测 `y`，并附带 **layout**（每信道 `offset` / `length`），供 Observer 切片。
- `serialize()` / `deserialize()` / `describe()` / `reset()`：整体持久化与描述。

### 最小示例（仅说明，未接入主程序）

```cpp
#include "AuroX/Space/ObservationSpace.hpp"
using namespace AuroX::Space;

ObservationSpace obs;

// 连续：温度，归一化到 [0,10]，时间宽度 3（保留最近 3 步）
ContinuousEncoder ce; ce.mode = ContinuousEncoder::Mode::MinMax;
ce.min = 0.0; ce.max = 10.0;
obs.addChannel(std::make_unique<ContinuousObservationChannel>("temp", "Temperature", 3, ce));

// 离散：挡位，等级 {0,1,2}，时间宽度 2
DiscreteEncoder de; de.levels = {0.0, 1.0, 2.0};
obs.addChannel(std::make_unique<DiscreteObservationChannel>("gear", "Gear", 2, de));

// 分类：模式，标签 {idle, run, err}，时间宽度 2
obs.addChannel(std::make_unique<CategoricalObservationChannel>(
    "mode", "Mode", std::vector<std::string>{"idle", "run", "err"}, 2));

// 已是向量的输入（如特征），长度 2，时间宽度 1
obs.addChannel(std::make_unique<RawVectorObservationChannel>("feat", "Feature", 2, 1));

// 摄入外部数据（多源、异构类型，ingest 自动路由）
obs.ingest("temp", 5.0);                 // -> 归一化 0.5
obs.ingest("gear", 1.0);
obs.ingest("mode", std::string("run"));
obs.ingest("feat", std::vector<double>{0.3, 0.7});

// 状态空间随时来拉：未填满的位置置零
std::vector<double> tempVec = obs.requestVector("temp");  // 长度 3 = [0.5, 0, 0]

// 组合观测 + 布局（供 Observer / 辨识器消费）
auto bundle = obs.collect();             // bundle.vector, bundle.layout
// bundle.layout[i] = { id, offset, length }

// 持久化 / 恢复
json snap = obs.serialize();
ObservationSpace restored;
restored.deserialize(snap);              // 完整重建全部信道与 FIFO 窗口

// 可选：用户自定义前馈补偿（在返回给状态空间前减去已知偏置）
struct BiasComp : ObservationCompensator {
    void apply(std::vector<double>& w) override { for (double& x : w) x -= 0.1; }
    std::string name() const override { return "bias_sub"; }
};
obs.get("temp")->setCompensator(std::make_unique<BiasComp>());
```

> MVP 范围：本文件**只实现观测空间的捕获/编码/持有/拉取**，不接入 `main.cpp`、不接线辨识器与状态空间。多源**不同采样率的对齐（全局 tick / hold / NaN 策略）** 与 **缺失 vs 真零的语义掩码** 作为后续增强，当前按"零填充 + 暴露 fillCount/isFull"提供最小支持。

### 文件位置

- 声明 + 实现（头文件，header-only）：`include/AuroX/Space/ObservationEncoder.hpp`
- 声明 + 实现（头文件，header-only）：`include/AuroX/Space/ObservationSpace.hpp`
- 设计讨论位于：项目记忆 `MEMORY.md`（观测空间设计思路与状态图）

