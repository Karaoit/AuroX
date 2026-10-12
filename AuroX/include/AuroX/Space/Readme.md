# AuroX · Space 层「四空间」说明

> 位置：`include/AuroX/Space/Readme.md`
> 设计依据：[`SSpace重构.md`](../../../SSpace重构.md)、[`ASpace重构设计.html`](../../../ASpace重构设计.html)
> 当前状态：**StateSpace / ParameterSpace / ActionSpace / ObservationSpace 四个已实现**（规划在 `AuroX::Planner`，不属于任何一个空间）。
> 交互演示：[`四空间演示.html`](../../../四空间演示.html)

---

## 0. 一句话定位

四个空间各管一件事，**互不越界**；「如非必要，勿增实体」。

| 空间 | 管什么 | 状态 | 头文件 | 外部依赖 |
|---|---|---|---|---|
| `StateSpace` | **当前状态**：带语义的值 + 稳定的数值视图 | 已实现 | `StateSpace.hpp` / `.ipp` | 仅标准库 |
| `ParameterSpace` | **可调参数**：配置 / 优化变量 + 边界 + 向量化 | 已实现 | `ParameterSpace.hpp` / `.ipp` | `nlohmann/json` |
| `ActionSpace` | **动作与执行**：注册 / 条件 / 执行 / 禁用（**不含搜索**） | 已实现 | `ActionSpace.hpp` | 仅标准库 |
| `ObservationSpace` | **外部观测**：通道 / 短历史 / 可选时标 / 稳定向量 | 已实现 | `ObservationSpace.hpp` | 仅标准库 |

> 规划（搜索）不属于任何一个「空间」：`AuroX::Planner`（`Planner/Planner.hpp` + `src/Planner/BFS.cpp`）**只读** `ActionSpace`，返回已注册动作的 hash 序列。依赖单向：`Planner → Space`，`ActionSpace` **不**依赖 `Planner`。

一句话区分容易混淆的三者：

- **State** = 外部程序确认或估计出的、当前供算法使用的状态值。
- **Observation** = 来自设备/用户的原始读数，可能有噪声与滞后。
- **Parameter** = 配置或优化变量，会改变「动作怎么执行」，但不等于系统当前状态。

> 关键原则：**「可信」由外部估计与校验保证**，不是写进 `StateSpace` 就自动可信。

---

## 1. 依赖方向与闭环

```text
外部观测源
    │
    ▼
ObservationSpace（外部观测：通道 + 短历史 + 可选时标）
    │  估计器 / 任务代码
    ▼
StateSpace（当前估计状态）        ParameterSpace（配置/可调参数）
    │ vec()                              │ 运行层按需取快照
    ▼                                    ▼
Planner.bfs(ActionSpace, cur, goal)   ← 只读 ActionSpace + Action::conditions / nxt
    │  返回 hash 序列（候选，不是实测）
    ▼
ActionSpace.run(id, actual) → Action::run   （只执行，不推演、不保存状态）
    ▼
外部设备 / 仿真
    │ 观测
    └──► 校正 StateSpace ──► 实测 vec() 即下一次规划/执行的起点（无需再同步副本）
```

**硬约束：**

- `StateSpace` 不保存 `ActionSpace&`；`ActionSpace` 不持有 `StateSpace*`。
- `ActionSpace` 只认识中性的 `Vec`——这是它最重要的优点，不要破坏。
- 四个空间之间**不互相包含头文件**（`StateSpace` / `ActionSpace` 连 json 都不依赖）。
- `Planner` 依赖 `ActionSpace`，反向**零依赖**；`ActionSpace` 不持有规划器，也不持有当前状态。

---

## 2. StateSpace —— 当前状态

### 2.1 契约

- **唯一数据源**：每个值只存在 `State::value` 一处。
- **两种访问，一份数据**：动态 `states.get<double>("v")` 与强类型 `robot.v.get()` 命中同一个值，**不需要任何 sync 函数**。
- **数值视图**：`vec()` 输出优化器 / 规划器消费的连续 `double` 向量；`layout()` 说明顺序与偏移。
- **冻结的是结构**：`freeze()` 锁定状态集合（名字/类型/形状），之后 `set()` 仍可写值。

支持类型与默认向量化：

| 类型 | 默认进 `vec()` | 约束 |
|---|---|---|
| `int` / `std::int64_t` | 是 | `int64` 向量化时要求 \|v\| ≤ 2^53（可精确转 double） |
| `float` / `double` | 是 | 拒绝 NaN / ±Inf |
| `Vec`（`std::vector<double>`） | 是 | 不能为空；更新时不能改长度 |
| `bool` | 否 | 显式开启后按 0/1 输出，**不等于**适合连续优化 |
| `Str`（`std::string`） | 否 | **不允许**隐式转成序号/哈希，避免伪距离 |

### 2.2 简易用法：动态命名状态

```cpp
#include "AuroX/Space/StateSpace.hpp"
using namespace AuroX::Space;

SSp states;
states.add("position_x", 1.5, "X coordinate");       // double，进 vec()
states.add("position_y", 2.0, "Y coordinate");
states.add("velocity",   0.8, "Speed");
states.add("mode", Str{"auto"}, "Control mode");     // 字符串，默认不进 vec()
states.freeze();                                     // 之后不能再 add

states.set("velocity", 1.2);
double v = states.get<double>("velocity");           // 1.2（返回副本）
const State& s = states.state("velocity");           // s.name / s.desc / s.value
Vec in = states.vec();                               // {1.5, 2.0, 1.2}
```

### 2.3 标准用法：强类型成员 + 动态访问共存

```cpp
struct RobotState : SSp {
    SField<double> x, y, speed;
    SField<Str>    mode;
    RobotState() {
        bind(x,     "x",     0.0, "X coordinate");
        bind(y,     "y",     0.0, "Y coordinate");
        bind(speed, "speed", 0.0, "Linear speed");
        bind(mode,  "mode",  Str{"idle"}, "Mode");
        freeze();
    }
};

RobotState robot;
robot.x = 1.5;                       // 强类型写
robot.set("speed", 1.2);             // 动态写
double a = robot.x.get();            // 强类型读
double b = robot.get<double>("x");   // 动态读 —— 与 a 始终一致，无需同步
double c = robot.speed;              // 隐式转换读（按值返回）
Vec    v = robot.vec();              // {1.5, 0.0, 1.2}
```

> `auto v = robot.speed;` **不可用**：`SField` 是代理而非值，拷贝被 `delete`。请用 `robot.speed.get()` 或 `double v = robot.speed;`。
> 不提供 `operator T&()` 或 `T& get()`，以免绕过写入校验。

### 2.4 高级用法：稳定布局 + 交给动作层

```cpp
RobotState robot;
auto meta = robot.layout();          // x: offset 0 / y: offset 1 / speed: offset 2

ActionSpace actions;
// 交给规划器的是 robot.vec() 这一份数值视图；ActionSpace 不再保存副本。
auto chain = AuroX::Planner::bfs(actions, robot.vec(), goal);
```

多维状态（一个 `Vec` 占多个分量）：

```cpp
SSp s;
s.add("pose", Vec{1.0, 2.0, 0.5}, "x, y, heading");  // offset 0, components 3
s.add("velocity", 0.8);                              // offset 3, components 1
s.freeze();
// layout() →  [{pose,0,3}, {velocity,3,1}]
```

### 2.5 边界与禁忌

| 操作 | 结果 |
|---|---|
| 空名 / 重名 `add` | 抛异常（不静默覆盖） |
| `freeze()` 后 `add` | 抛异常 |
| `freeze()` 后 `set` | **允许** |
| `get<T>` 类型不符 | 抛异常（不静默返回 0） |
| `set` 改类型 / 改 `Vec` 长度 | 抛异常 |
| NaN / ±Inf / 空 `Vec` / 字符串向量化 / 超范围 `int64` | 抛异常 |
| 未知名字 | 抛异常 |

**刻意不提供**：`unvec()`、`snapshot()`、`setBatch()`、`remove()`/`replace()`、线程安全承诺。

---

## 3. ParameterSpace —— 可调参数

### 3.1 契约

- 参数 = **配置 / 优化变量**，可改变动作执行方式，但**不是**当前状态。
- 写入**没有旁路**：非 const 的 `get()` 与 `operator T&()` 被移除，`raw()` 只返回 const 指针；写入必须走 `operator=`。
- 重名注册直接抛异常；同名替换会让已有代理悬空，所以禁止。
- JSON 导入**先解析、后整体提交**，失败不改动原空间。
- 反向向量化**全量校验后原子提交**。

### 3.2 简易用法：动态声明 + 向量化

```cpp
#include "AuroX/Space/ParameterSpace.hpp"
using namespace AuroX::Space;

ParameterSpace space;
space.declare("Kp", 1.0, 0.0, 20.0);                 // 有界 double
space.declare("Ki", 0.1, 0.0, 20.0);
space.declareDiscrete("mode", 1, {0, 1, 2});         // 离散 int
space.declareEnum("strategy", size_t(0), {"PID", "PI", "PD"});

std::vector<double> x = space.vectorizeDouble();     // 优化器输入
space.unvectorizeDouble({2.0, 0.3, 2.0, 1.0});       // 边界/候选集外 → 抛异常

if (auto* kp = space.getAs<double>("Kp"))
    kp->value;                                       // 读：与 vec() 同源
```

### 3.3 标准用法：`PStruct` + `Param` / `EnumParam`

```cpp
struct PIDParameters : public AuroX::Space::PStruct<PIDParameters> {
    AuroX::Space::Param<double>     Kp, Ki, Kd;
    AuroX::Space::EnumParam         mode;

    AUROX_PARAMS(Kp, Ki, Kd, mode)                   // 迭代清单（同步/序列化/向量化都用它）

    PIDParameters() {
        PARAM_REG(                                   // 注册骨架
            PARAM_BIND(Kp, 1.0, 0.0, 10.0);          // 名字由变量名自动派生，不再重复写字符串
            PARAM_BIND(Ki, 0.1, 0.0, 5.0);
            PARAM_BIND(Kd, 0.05);
            PARAM_BIND(mode, "auto", {"auto", "manual", "adaptive"});
        );
    }
};

PIDParameters pid;
pid.Kp = 1.5;                       // operator= ：同时更新本地镜像与底层对象
pid.mode = "manual";
std::vector<double> p = pid.vectorize();             // {1.5, 0.1, 0.05, <mode 下标>}
pid.unvectorize({2.0, 0.2, 0.1, 0.0});
```

别名：`PSpace::Param<T>`、`PSpace::PStruct<...>`、`PSpace::TParam<T>`、`PSpace::EParamRaw`、`PSpace::CompMeta`。

### 3.4 高级用法：持久化 + 布局偏移 + 非抛异常加载

```cpp
// 保存 / 加载（结构不合法会失败，且不修改原空间）
bool ok = tryLoadJson(pid, "pid.json");     // 非抛异常入口

// 失败即整体回滚（强异常保证）
if (!pid.tryFromJson(json::object()))       // 缺 "parameters" → false，原空间不变
    /* 保持旧配置 */;

// 数值化布局：给第 i 个参数在 vec() 中的起始下标
for (const auto& c : pid.layout())          // ComponentMeta{ name, type, components, is_enum, offset }
    use(c.name, c.offset, c.components);
```

### 3.5 边界

| 操作 | 结果 |
|---|---|
| 重名 `declare` | 抛异常 |
| `fromJson` 缺 `parameters` / 条目缺 `type` / 值类型不符 | `tryFromJson` 返回 `false`，**原空间不变** |
| `unvectorizeDouble` 长度不符 / 越界 / 非有限 / 不在候选集 | 抛异常，**全有或全无** |
| 直接写底层对象（`raw()->value = ...`） | **编译期禁止**（`raw()` 只返回 const 指针） |

---

## 4. ActionSpace —— 动作与执行（规划见 Planner）

### 4.1 契约

- 3 个类型（`Condition` / `Action` / `ActionSpace`）+ 1 个自由函数（`hashOf`）+ 0 个宏，**仅依赖标准库**。
- **不持有当前状态**：没有 `cur_` / `cur()`；真实状态只存在 `StateSpace` 一处（唯一数据源）。
- **不含搜索**：没有 `plan()` / `replan()` / `dist()` / BFS——规划全部在 `AuroX::Planner`。
- 动作层只认识中性的 `Vec`；参数/状态/观测用**闭包捕获**。
- `Action::nxt` 必须是**纯预测**：同输入同输出、无副作用、不碰真实状态；结果必须**同维且有限**。
- `Action::next()` 在 `nxt` 缺失时**抛 `std::logic_error`**：未知效果 ≠ 状态不变。
- `run()` **只执行、不推演、不推进状态**；`true` 只代表本地闭包正常返回，**不代表设备执行成功**。

### 4.2 简易用法：动作退化成一个集合

```cpp
#include "AuroX/Space/ActionSpace.hpp"
using namespace AuroX::Space;

ActionSpace as;
Action beep;
beep.name = "beep";
beep.type = "unit";
beep.run  = [] { /* 真实执行体：无参，需要什么就在闭包里捕获 */ };
as.reg(beep);

as.run(beep.hash);                   // 无参重载：仅当动作「无条件」时有效
```

### 4.3 标准用法：`conditions` + `nxt` + `Planner::bfs`

```cpp
#include "AuroX/Planner/Planner.hpp"

ActionSpace as;
Act right;
right.name = "right";
right.type = "move";
right.conditions.push_back({ "x <= 3", [](const Vec& s) { return s[0] <= 3.0; } });
right.nxt = [](const Vec& s) { Vec y = s; y[0] += 1.0; return y; };   // 纯预测
right.run = [&] { /* 驱动设备向右一步 */ };
as.reg(right);

const Vec start{0.0};
const Vec goal{3.0};
AuroX::Planner::Chain c = AuroX::Planner::bfs(as, start, goal);   // eps 默认 1e-6（平方距离）
if (c.ok) for (std::uint64_t h : c.acts) as.run(h, start);        // 执行必须带状态
else      /* c.why 说明失败原因 */;
```

> `bfs()` 的失败统一走 `Chain{ok=false, why=...}`，**不抛异常**：`eps<0`、`maxDepth<1`、`maxNodes<1`、维度不符、非有限数，都在入口拒绝。

### 4.4 高级用法：禁用 + 重规划 + 实测回写闭环

```cpp
Vec actual = states.vec();                          // 起点永远是「实测」
AuroX::Planner::Chain c = AuroX::Planner::bfs(as, actual, goal);

for (std::uint64_t h : c.acts) {
    Act* a = as.get(h);
    const Vec before = states.vec();                // 1) 每一步都取最新实测
    const Vec pred   = a->next(before);             // 2) 候选后继（纯预测）
    if (!as.run(h, before)) break;                  // 3) 只执行，不推进任何状态
    // 4) 等待设备完成确认 + 采集观测 → 更新 StateSpace
    const Vec measured = states.vec();
    // 5) 误差比较由任务侧自己做（Space 不再对外提供 dist()）
    bool diverged = false;
    for (std::size_t i = 0; i < measured.size() && i < pred.size(); ++i)
        if (std::fabs(measured[i] - pred[i]) > 1e-6) { diverged = true; break; }
    if (diverged) {
        as.ban(h);                                   // 6) 封锁失败动作
        c = AuroX::Planner::bfs(as, measured, goal); // 7) 从新实测状态重规划
        break;
    }
}
```

> `run(h, state)` 用传入状态检查 `conditions`；无参的 `run(h)` **不绕过**条件——只要动作有条件就返回 `false`。

同名动作消歧：`as.get(type, name)`（`type+name` → hash，O(1)）；`as.get(name)` 在同名时取声明序第一个。

### 4.5 类型速查

| 类型 | 关键成员 |
|---|---|
| `Condition` | `desc`、`ok`（空 = 恒真）、`test(x)` |
| `Action` | `name`、`type`、`hash`（0 = 由 `reg` 自动派生）、`run`、`conditions`、`nxt`；`can(x)` / `next(x)`（缺 `nxt` 抛异常） / `exec()` |
| `ActionSpace` | `reg(...)`、`get(...)`、`ids()`、`run(id, state)` / `run(id)`、`ban/unban/banned` |
| `Planner::Chain` | `ok`、`why`、`acts`（动作 hash 序列，**无 `edges`**） |

---

## 5. ObservationSpace —— 外部观测

### 5.1 契约

- **一个类**（`ObservationSpace`，别名 `OSpace`），**没有**通道继承族 / 编码器 / 补偿器 / 工厂 / JSON / 插件。仅依赖标准库。
- **有限维外部观测的入口**：接收 → 严格校验 → 最近 N 条短缓存 → 稳定拼接为 `Vec`。它**不是**传感器驱动、**不是**控制器、**不是**状态估计器。
- **不写 `StateSpace`**：外部任务代码自己决定是否把某条观测（或估计结果）写入 `StateSpace`。
- `historySize` 是**样本条数**，不是时间长度；窗口**左侧补零**，最新样本恒在**最右**。
- `validCount` 统计**真正收到**的样本（值为 `0` 也计入）；**不能**用「窗口是否全零」判断有效性。
- 时间戳是 `std::optional<double>`：**缺失 ≠ `t=0.0`**；同通道时间戳必须**非递减**；`lastTimestamp` 只反映**最新一条**（最新一条无时标则为 `nullopt`）。
- 输入非法**一律明确拒绝**（抛 `std::invalid_argument`）：未知通道、类型/长度不符、未知类别、NaN/Inf。**没有静默 no-op**。
- 一把 `mutex` 保护同一实例的写入与快照；**不承诺**硬实时或跨传感器物理同步。

### 5.2 简易用法：单标量（比赛 / 软件实验）

```cpp
#include "AuroX/Space/ObservationSpace.hpp"
using namespace AuroX::Space;

ObservationSpace obs;
obs.add("distance_m");          // 1 维，只保留最近 1 条
obs.freeze();

obs.ingest("distance_m", 0.48);
if (auto v = obs.latest("distance_m"))   // 没收到过 → nullopt，绝不伪造 0
    double d = v->at(0);
```

### 5.3 标准用法：历史窗 + 类别 + 稳定向量（训练 / 仿真）

```cpp
ObservationSpace obs;
obs.add("temperature", 1, 3);                 // 3 条标量
obs.add("joint", 3, 2);                       // 2 条、每条 3 维
obs.addCategory("mode", {"idle", "run", "fault"});
obs.freeze();

obs.ingest("temperature", 21.5);
obs.ingest("temperature", 22.0);
obs.ingest("joint", Vec{1, 2, 3});
obs.ingest("mode", Str{"run"});               // one-hot [0,1,0]

auto b = obs.collect();
// b.vector = [0, 21.5, 22.0, 0,0,0,1,2,3, 0,1,0]
// layout : temperature(off 0, len 3, dim 1, valid 2)
//          joint      (off 3, len 6, dim 3, valid 1)
//          mode       (off 9, len 3, dim 3, valid 1)
```

> 归一化 / 标准化 / Embedding 由**外部**训练或数据处理代码负责，不塞进 `ObservationSpace`。

### 5.4 高级用法：带时标 + 新鲜度检查（工业 / 具身）

```cpp
ObservationSpace obs;
obs.add("joint_position", 6, 3);
obs.freeze();

obs.ingest("joint_position", Vec{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}, 12.40);

const auto b = obs.collect();
const auto& m = b.layout.at(0);
const double now = 12.42, maxAgeSec = 0.05;    // 同一时钟基准 + 任务自定义阈值
if (m.validCount == 0 || !m.lastTimestamp ||
    now < *m.lastTimestamp || now - *m.lastTimestamp > maxAgeSec) {
    /* 未收到 / 无时标 / 过期：停机或报警由外部安全逻辑决定 */
} else {
    const auto begin = b.vector.begin() + m.offset + m.length - m.sampleDim;
    states.set("joint_position", Vec(begin, begin + m.sampleDim));  // 从同一快照取最新样本
}
```

### 5.5 边界与禁忌

| 操作 | 结果 |
|---|---|
| 重名 / 空名 `add` | 抛异常 |
| `sampleDim==0` / `historySize==0` / `sampleDim*historySize` 溢出 | 抛异常 |
| `freeze()` 后 `add` | 抛异常（`ingest` 仍可用） |
| 未知通道 / 类型不符 / 长度不符 / 未知类别 / NaN / Inf | 抛异常（**无静默 no-op**） |
| 时间戳倒退 | 抛异常 |
| `latest()` 无样本 | 返回 `nullopt`（不伪造零值） |
| `reset()` | 只清样本，保留通道与 `freeze` 状态 |

**刻意不提供**：编码器/归一化、滤波/融合、补偿器、动态向量长度、图像/点云缓存、JSON 序列化、跨传感器时钟同步、插值重采样、回调/TTL、设备驱动、无锁队列、`.ipp`、通道工厂。

---

## 6. 空间 + Planner：一个完整闭环

```cpp
#include "AuroX/Space/StateSpace.hpp"
#include "AuroX/Space/ParameterSpace.hpp"
#include "AuroX/Space/ActionSpace.hpp"
#include "AuroX/Planner/Planner.hpp"
using namespace AuroX::Space;

// ---- 1) 状态 ----
struct RobotState : SSp {
    SField<double> x, y, speed;
    RobotState() {
        bind(x, "x", 0.0, "X"); bind(y, "y", 0.0, "Y"); bind(speed, "speed", 0.0, "v");
        freeze();
    }
} robot;

// ---- 2) 参数（动作执行方式由它决定）----
ParameterSpace cfg;
cfg.declare("speed_scale", 1.0, 0.1, 3.0);

// ---- 3) 动作（闭包捕获参数与设备句柄）----
ActionSpace actions;
Act go;
go.name = "go_x"; go.type = "motion";
go.conditions.push_back({ "x <= 4", [](const Vec& s) { return s[0] <= 4.0; } });
go.nxt = [](const Vec& s) { Vec y = s; y[0] += 1.0; return y; };
go.run = [&] { /* device.step(cfg.getAs<double>("speed_scale")->value) */ };
actions.reg(go);

// ---- 4) 闭环：规划只读，执行带实测，重规划从新实测出发 ----
const Vec goal{3.0, 0.0, 0.0};
auto c = AuroX::Planner::bfs(actions, robot.vec(), goal);
for (std::uint64_t h : c.acts) {
    const Vec before = robot.vec();                       // 每步取最新实测
    const Vec pred   = actions.get(h)->next(before);      // 候选后继（纯预测）
    if (!actions.run(h, before)) break;                   // 只执行，不推进状态
    /* 采集观测 → 估计 → 更新 robot（实测回写） */ robot.x = robot.x.get() + 1.0;
    const Vec measured = robot.vec();
    // 任务侧自备平方误差比较（Space 不再对外提供 dist()）
    if (std::fabs(measured[0] - pred[0]) > 1e-6) {        // 与预测不符 → 重规划
        actions.ban(h);
        c = AuroX::Planner::bfs(actions, measured, goal);
    }
}
```

---

## 7. 常见错误速查

| 症状 | 原因 | 处理 |
|---|---|---|
| 改名/改类型后下游读错数据 | `StateSpace` 的 schema 是固定的 | 冻结前后都不要改类型或 `Vec` 长度 |
| `as.cur(...)` / `as.plan(...)` 编译不过 | V0.5 已移除：动作空间不再持有状态、不再搜索 | 改用 `AuroX::Planner::bfs(as, states.vec(), goal)` |
| `bfs()` 明明该失败却「成功」 | 把线性距离阈值当成了平方阈值 | `bfs()` 的 `eps` 用**平方误差单位**（Planner 内部 `sqDist`） |
| `run()` 返回 `true` 但设备没动 | `run()` 只代表本地闭包正常返回 | 外部确认执行 + 实测回写 `StateSpace` |
| 改完参数又被「变回去」 | 直接写了底层对象 | 写入统一走 `operator=` |
| 拓扑不变却规划出不同链 | `nxt` 内读了实时变化的外部数据 | `nxt` 必须是对假设状态的**纯函数** |
| 缺失 `nxt` 的动作规划不到 | `next()` 现在对缺失 `nxt` 明确失败 | 默认 `bfs` 跳过无 `nxt` 动作；或提供 `PredictFn` |
| 不同量纲分量被同一 `eps` 吞掉 | 未做归一化 | 归一化/加权由优化器或定制规划策略负责 |
| 观测窗口全零却当成「没数据」 | 真实测量 `0` 与「未收到」被混淆 | 用 `layout.validCount`（收到即计数），不要看是否全零 |
| 观测「有数据」却已过期 | `validCount>0` 只说明收到过 | 用 `lastTimestamp` + 任务自定义时间窗做新鲜度检查 |

---

## 8. 交互演示

打开 [`四空间演示.html`](../../../四空间演示.html)（浏览器直接双击即可，纯前端、无依赖）：
StateSpace / ParameterSpace / ActionSpace / ObservationSpace 四个标签页可实时操作（规划面板对应 `AuroX::Planner`）。

> 演示页里的「规划」面板对应 `AuroX::Planner`：前端用 JS 复刻了同一套最简 BFS，用于说明「规划只读、执行不推演状态」。
