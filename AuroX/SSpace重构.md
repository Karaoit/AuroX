# AuroX 状态空间重构文档

> **文档性质**：技术设计 / 缺陷审查 / 参考实现 / 迁移与验收方案  
> **设计版本**：1.0（提案，尚未合入项目）  
> **核查基线**：用户提供的 `AuroX-9.24.rar` 中的现有源码  
> **编译基线**：C++20，与当前 `AuroX/CMakeLists.txt` 的标准一致  
> **范围**：已重构的 `ParameterSpace`、`ActionSpace` 的接口问题及改进建议；待实现的 `StateSpace` 设计。**压缩包中不存在的旧模块，不视为当前实现。**

## 0. 执行摘要

### 0.1 最终决定

**StateSpace 采用“唯一数据所有者 + 命名状态 + 强类型字段代理 + 固定布局向量化”方案。**

- **唯一数据源**：每个状态值只由 `StateSpace` 内部的一个 `State` 对象持有。
- **两种访问体验**：`states.get<double>("velocity")` 和 `robot.velocity.get()` / `robot.velocity = 1.2`。
- **数值化出口**：`vec()` 返回优化器、动作规划器消费的连续 `std::vector<double>`；`layout()` 解释元素顺序及偏移。
- **结构约束**：状态名称、类型和向量分量数注册后固定；`freeze()` 阻止新增状态，但不妨碍更新现有状态值。
- **三档用法**：简易=动态容器；标准=强类型绑定；高级=固定布局、数值校验、外部闭环集成。三档共用相同底层，没有三套框架。
- **功能边界**：不内置 `predict()` / `transition()` / `plan()` / `estimate()` / `unvec()`。预测继续交给 `Action::nxt`，观测采集和状态估计由外部调度者组织。
- **主张的最小核心**：`State`、`SField<T>`、`StateSpace`（别名 `SSp`）；布局元数据作为 `StateSpace::ComponentMeta` 嵌套类型，不建立额外的多态层。

### 0.2 为什么不复制 ActionSpace 的功能复杂度

`ActionSpace` 负责动作条件、预测转移、BFS 搜索、禁用集、重规划，天然具有行为算法；`StateSpace` 的主要职责是维护可解释且一致的数据。两者应该在**易用性、命名风格、独立性、三档用法**上统一，而不是强行共享 `plan()` 等功能。做到结构稳定、读写清晰、向量契约可靠，比增加状态管理器、订阅事件或复杂继承更重要。

---

## 1. 审查依据和现有事实

### 1.1 本次核查的源文件

| 文件 | 作用 |
|---|---|
| `AuroX/include/AuroX/Space/ParameterSpace.hpp` | 参数类型、动态空间、`Param<T>`、`PStruct<Derived>` 声明 |
| `AuroX/include/AuroX/Space/ParameterSpace.ipp` | 参数的模板实现、绑定、同步和向量化 |
| `AuroX/src/Space/ParameterSpace.cpp` | 参数注册、JSON、向量化的非模板实现 |
| `AuroX/include/AuroX/Space/ActionSpace.hpp` | `Condition`、`Action`、`Chain`、`ActionSpace` 接口 |
| `AuroX/src/Space/ActionSpace.cpp` | 距离、执行、规划、重规划实现 |
| `AuroX/include/AuroX/Core.hpp` | 当前集成头的条件包含 |
| `AuroX/CMakeLists.txt` | 当前构建配置 |
| `AuroX/apps/AuroXServer/main.cpp` | 仍调用重构前接口的集成示例 |
| `AuroX/ASpace重构设计.html` | 动作空间的解耦与三档用法设计理念 |
| `AuroX-General_技术路线.md` | 状态/观测/参数/动作/状态方程的概念边界 |

### 1.2 已实现和未实现的界限

**已实现：**
- `ParameterSpace`：动态参数、`Param<T>` / `PStruct<Derived>`、JSON、数值向量化。
- `ActionSpace`：动作注册、条件、执行、`nxt` 预测、BFS `plan()`、`ban()`、`replan()`。

**当前压缩包中未实现：**
- `include/AuroX/Space/StateSpace.hpp` 及其 `.cpp`；
- `include/AuroX/Space/ObservationSpace.hpp` 及其 `.cpp`；
- 集成示例所依赖的部分旧 Dynamics / Optimizer / 动作子类等接口。

因此，下文 **“已确认缺陷”指当前有源码证据的行为**；“新 StateSpace 的风险”指必须在实现前规避的问题；“建议接口”及附录代码均为**新增提案**，不是现有公共 API。

---

## 2. 现存缺陷、风险与优化建议

### 2.1 分级方法

- **P0 / 阻断**：可以直接产生错误结果、悬空引用或导致关键调用无法链接/构建，应优先处理。
- **P1 / 高优先级**：影响数据一致性、异常安全或高频使用契约。
- **P2 / 建议改善**：语义不完整、性能与可维护性风险，按范围修正。
- 分类中，“确认”表示由当前代码直接成立或用最小程序复现；“风险/集成差距”不等于已发生的线上故障。

### 2.2 缺陷总表

| 编号 | 类别 | 严重度 | 证据位置 | 问题及后果 | 优化建议 |
|---|---|---|---|---|---|
| A-01 | 已确认 | **P0** | `ActionSpace.cpp:31-39` | `dist()` 只比较 `min(a.size(),b.size())`，例如 `{1,2}` 和 `{1,2,100}` 距离为 `0`，错误维度可能被认为达标或重复状态。 | 长度不相同直接拒绝；同时校验非有限值与 `eps` 语义。 |
| A-02 | 已确认 | **P1** | `ActionSpace.cpp:145-160` | `plan(goal,...,asGraph=true)` 成功时在生成 `edges` 之前就返回，成功结果的 `edges` 为空。 | 统一在返回前构建请求的图，或明确改变 API 为“失败时调试图”；推荐前者。 |
| A-03 | 契约风险 | **P1** | `ActionSpace.cpp:97-104` | `exec()` 返回 `void`，空 `run` 也是合法空操作；`ActionSpace::run()` 返回 `true` 并更新 `cur_` 仅代表内部流程走完，**不代表设备执行成功或实测状态已更新**。 | 注释明确“预测状态”；外部提供执行确认与实测回写，必要时未来加入执行结果对象。 |
| A-04 | 契约风险 | **P1** | `ActionSpace.cpp:53-55, 135-145` | `nxt` 返回的维度、NaN/Inf 和副作用未被检查；规划与运行都依赖其合法性。 | 约定 `nxt` 为纯预测；规划入口和每一步 `nxt` 后强制核查维度、有效值。 |
| A-05 | 性能风险 | **P2** | `ActionSpace.cpp:116-155` | BFS 去重对 `seen` 线性扫描，大搜索树可能出现二次代价，且没有节点预算。 | 加可配置节点上限；需要时采用量化状态键/哈希去重，不为 StateSpace 引入算法。 |
| A-06 | 使用风险 | **P2** | `ActionSpace.cpp:69-79, 86-92` | 动作注册按 `type/name` 计算 hash，但 `get(name)` 仅按 `name` 找首个匹配，跨类型同名时有歧义。 | 建议优先使用 hash 查找，或提供 `get(type,name)` 重载。 |
| A-07 | 兼容性问题 | **P2** | `ActionSpace.cpp:22-28` | `hashOf()` 注释写 FNV-1a 64，使用的 offset basis 为 `1469598103934665603`，而标准 64 位偏置是 `14695981039346656037`。 | 如需与标准 FNV-1a 一致应修正；**注意修正将改变已生成的动作 hash**，须先检查持久化/互操作契约。 |
| P-01 | 已确认 | **P1** | `ParameterSpace.hpp:288-327`; `ParameterSpace.ipp:203-255, 283-310` | `Param<T>` 成员 `value` 与 `TypedParameter<T>::value` 是两份数据。`Param<T>::get()` 非 const 返回 `T&`、隐式转换返回 `T&`，通过引用赋值可绕过 `sync_to_raw()`。 | StateSpace **不复制双份状态值**；使用只读值返回和受控写入的 `SField<T>`。参数空间可另行重构其同步入口。 |
| P-02 | 已确认 | **P1** | `ParameterSpace.cpp:112-117`；`ParameterSpace.hpp:317-329` | 同名 `add()` 直接替换 `unique_ptr`，原 `Param<T>::raw_` 若仍指向旧对象，会悬空。 | StateSpace 注册同名直接报错；不提供可使已有代理失效的替换/删除操作。参数空间已有绑定对象需显式 `rebind()` 才可安全重新指向。 |
| P-03 | 已确认 | **P1** | `ParameterSpace.cpp:154-170` | `fromJson()` 先 `clear()`，后逐项加载；加载中失败时，`tryFromJson()` 虽返回 `false`，旧状态已经被清掉或只剩部分新数据。 | 使用临时对象完整解析和校验，成功后整体提交，保证失败不更改原状态。 |
| P-04 | 已确认 | **P1** | `ParameterSpace.cpp:176-199`；`ParameterSpace.ipp:188-196` | `unvectorizeDouble()` 对短输入仅部分写入、长输入静默忽略；不对全量维度及参数边界作统一提交前检查。 | 先检查完整长度与全部分量合法性，再原子提交；**StateSpace V1 不提供无约束 `unvec()`**。 |
| P-05 | 已确认 | **P1** | `ParameterSpace.hpp:417`；`ParameterSpace.ipp:325-333` | `PStruct<Derived>::tryFromJson()` 有声明，但对应模板定义在 `.ipp` 中被注释掉；实例化调用时会出现缺失定义/链接问题。 | 补齐定义并增加编译/链接测试；新 StateSpace 的模板实现应保持可见且测试覆盖。 |
| P-06 | 改进建议 | **P2** | `ParameterSpace.hpp:228-233`; `ParameterSpace.cpp:193-199` | `ComponentMeta` 只有名称、类型、分量数，未携带显式向量偏移。 | 新状态空间 `layout()` 返回 `offset`、`components`，避免下游重复推导下标。 |
| I-01 | 集成差距 | **P0** | `AuroX/apps/AuroXServer/main.cpp:153-170, 219-255` | 仍引用当前包中不存在的 `CustomAction`、`ObservationSpace`、`StateSpace`、旧 Dynamics 等接口，不能作为新空间验收基线。 | 先建立独立的 StateSpace 单元测试与独立四空间演示，再逐步更新或替换服务器示例。 |
| I-02 | 构建建议 | **P2** | `AuroX/CMakeLists.txt:22-24` | `file(GLOB_RECURSE ...)` 未附 `CONFIGURE_DEPENDS`，新增 `.cpp` 后现有构建树不一定立即发现。 | 重新配置 CMake，或加 `CONFIGURE_DEPENDS` / 显式列出源文件。 |

### 2.3 已运行的最小复现

**动作空间维度问题：**

```cpp
dist(Vec{1, 2}, Vec{1, 2, 100}); // 当前实现结果：0
```

**成功规划缺失图边：**

```cpp
ASpace actions(Vec{0});
Act step;
step.name = "step";
step.type = "unit";
step.nxt = [](const Vec& x) {
    Vec y = x;
    y[0] += 1;
    return y;
};
actions.reg(step);

Chain result = actions.plan(Vec{1}, 8, 1e-6, true);
// 当前实测：result.ok == true, result.edges.size() == 0
```

这两个结果已使用包内 `ActionSpace.cpp` 和 C++20 编译器运行复现。

### 2.4 在新增 StateSpace 前建议立即修复什么

推荐按依赖顺序：

1. **A-01 维度判断**：这是状态向量跨空间协作的基础正确性要求。
2. **P-02/P-01 的设计教训**：新 `SField<T>` 不复制值、不保留可能被替换对象的裸指针。
3. **A-04 `nxt` 契约**：状态维度、有限性和纯预测要求需要写进 API 使用说明。
4. **I-01 隔离旧集成样例**：先让新空间可以独立编译/测试，不要依赖尚未迁移的 Server。
5. **A-02、P-03/P-04/P-05**：作为平行质量修复项推进，不阻断 StateSpace 文件创建。


### 2.5 建议修补的关键代码片段

**A-01：严格维度与有限性校验（适合优先处理）**

```cpp
#include <cmath>
#include <stdexcept>

double dist(const Vec& a, const Vec& b) {
    if (a.size() != b.size())
        throw std::invalid_argument("ActionSpace: state dimension mismatch");

    double d = 0.0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!std::isfinite(a[i]) || !std::isfinite(b[i]))
            throw std::invalid_argument("ActionSpace: non-finite state");
        const double delta = a[i] - b[i];
        d += delta * delta;
    }
    if (!std::isfinite(d))
        throw std::overflow_error("ActionSpace: squared distance overflow");
    return d;
}
```

同时在 `plan()` 入口拒绝 `eps < 0`、非有限的 `eps` 和错误维度；如果规划器向调用方提供非抛异常接口，也可以统一封装为 `Chain{ok=false, why=...}`。这属于**改变异常契约**，应保持全模块一致并补测试，不能只改一处而忽略调用方处理。

**A-02：确保成功返回时也附带请求的图。** 建议把目前失败分支内的 `edges` 构造提取为一个内部辅助操作，在“已命中目标”及“搜索耗尽”两种返回路径都执行。起点已经满足目标时，返回空 `edges` 是合理的，因为还没有生成边。修改前应明确“成功图”究竟是**完整已探索图**还是**最终路径图**；现有 `edges` 定义更接近前者。

**P-03：JSON 导入采用先解析、后提交。** 将全部参数解析进临时 `ParameterSpace`，成功且通过校验后再提交，失败不修改旧空间。注意当前 `Param<T>` 代理保存了 `raw_`，因此**不能仅交换容器就认为绑定仍然安全**；提交后必须重绑定，或者先重新设计稳定句柄。本条是设计修复方向，不是可直接粘贴的一行补丁。

**P-04：反向向量化先校验完整输入。** 对 `unvectorizeDouble()`，先核对长度与 `layout()` 的 `components` 总和，再把每个候选值转换到对应参数类型，逐项检查合法范围和离散候选集；全部成功才提交。当前直接循环 `unpackDouble()` 会产生部分更新。这个缺陷也说明新 StateSpace 不应在第一版默认暴露 `unvec()`。


---

## 3. StateSpace 的需求与非目标

### 3.1 功能需求

- R1：每个状态携带 `name`、可选 `desc`、具体 `value`。
- R2：按名称直接读取/修改，支持运行时动态构造状态集合。
- R3：通过 `SField<T>` 强类型成员读取/赋值，获得静态类型和 IDE 自动补全。
- R4：两种访问方式**绝不复制出独立状态副本**，始终命中相同数据。
- R5：`vec()` 输出优化器可用的纯 `double` 连续向量。
- R6：`layout()` 输出每个参与数值化的状态名称、类型、分量数、起始偏移。
- R7：向量顺序稳定，固定类型和长度，禁止静默重排或部分更新。
- R8：状态空间独立于参数空间、动作空间、观测空间和优化器，核心可只依赖标准库。
- R9：使用体验分为简易 / 标准 / 高级三档，但共用相同核心类型。

### 3.2 非目标（不放入首版）

| 非目标 | 应属模块/原因 |
|---|---|
| `StateSpace::predict()` / `transition()` | 当前已有 `Action::nxt`，通用动力学以后可在独立模块实现 |
| `StateSpace::plan()` / `replan()` | `ActionSpace` |
| `StateSpace::estimate()` / 传感器融合 | 任务相关估计器，消费 ObservationSpace |
| 观测通道、图像帧缓存、时间对齐 | `ObservationSpace` |
| 状态历史、长期持久化 | Memory / 运行记录层 |
| 梯度、目标函数、求解器 | Optimizer / Evaluation |
| 隐式 `unvec()` 覆盖真实状态 | 优化器产生的候选向量不等于当前可信状态 |
| 任意 `T&` 可写引用 | 会绕过类型、维度和有效性管理 |
| 状态类宏注册/CRTP/虚基类 | 首版没有必要，绑定代理已经实现两种访问需求 |

### 3.3 State / Observation / Prediction 的语义

- **Observation**：来自外界设备或用户的观测值，可能有噪声或滞后。
- **State**：由外部程序确认或估计得到、当前供算法使用的状态值；不是观察原始缓存。
- **Predicted State**：动作的 `nxt(x)` 在假设状态上推演的结果；不是实测结果。
- **Parameter**：配置或优化变量；可以改变动作执行方式，但不一定等于系统当前状态。

注意：**“可信”由外部估计与校验保证，不是只要写进 StateSpace 就自动可信**。首版不假装解决观测时间戳、状态估计误差或缺测判定。

---

## 4. 方案权衡和设计取舍

### 4.1 存储方案比较

| 方案 | 两种访问方式 | 单一数据源 | 复杂度 | 结论 |
|---|---|---|---|---|
| `Vec` + 名字数组 | 名字访问需自行映射 | 是 | 极低 | 丢失类型和具体状态语义，不推荐 |
| 复制 `Param<T>`/`PStruct` 同步机制 | 支持 | **否**，存在成员副本和原始副本 | 中高 | 不推荐用于高频状态 |
| 状态持有 `variant` + `SField<T>` 代理 | 支持 | **是** | 中低 | **首选** |
| `StateBase` + 多种虚派生类 | 支持 | 可做到 | 高 | 缺乏必要的扩展收益，暂不需要 |
| CRTP + 反射/宏生成 | 支持 | 取决于实现 | 高 | 可作后续语法糖，不作为底层前提 |

### 4.2 推荐数据模型

```text
StateSpace : owns
  items_  -> [State(name, desc, value, vectorized), ...]
  index_  -> name -> stable index
  dim_    -> cached vector layout dimension, NOT duplicated values
  frozen_ -> schema lock

Dynamic access:
  get<T>("velocity") ------+
  set<T>("velocity", v) ----+----> ONE State.value
                           |
Strong typed access:       |
  SField<T>.get() ---------+
  SField<T>::operator= ----+

Numeric view:
  vec()    -> Vec<double> (temporary copy)
  layout() -> [{name, type, offset, components}, ...]
```

**所有权**：`StateSpace` 唯一拥有 `State`；`SField<T>` 只保存 `StateSpace*` 与稳定下标，不另外保存 `T`。底层用 `std::vector<std::unique_ptr<State>>`，即使注册阶段内部向量扩容，已存在的 `State` 对象地址仍稳定。

**生命周期**：含 `SField<T>` 的结构体不能比它的基类 `StateSpace` 活得更久；首版禁用 `StateSpace` 的复制和移动，避免代理绑定到被移动的旧地址。不要在有活跃代理时动态删除、同名替换或交换状态对象。

### 4.3 名字、类型、维度与数值化策略

首版支持：`bool`、`int`、`int64_t`、`float`、`double`、`std::string`、`std::vector<double>`。

| 状态类型 | 默认参与 `vec()` | 特别约束 |
|---|---|---|
| `int` / `int64_t` | 是 | `int64_t` 转 `double` 时首版采用安全范围限制（绝对值不超过 2^53） |
| `float` / `double` | 是 | 拒绝 NaN 和 ±Inf |
| `Vec` | 是 | 向量化时不能为空；更新不允许改变原先长度 |
| `bool` | 否 | 仅用户明确要求时按 0/1 输出；不等同于适合连续优化 |
| `Str` | 否 | 不隐式转成序号或哈希值，避免引入伪距离 |

**关键约定：**
- 状态名称非空且全局唯一；同名注册抛错，不静默覆盖。
- `get<T>()` 必须与注册的实际类型相同，不做隐式换型。
- `set<T>()` 可以修改值，但不可以修改已声明类型、向量长度或数值化标记。
- `freeze()` 锁定状态集合；`set()` 仍可调用。
- 向量排序严格按注册顺序，不遍历 `unordered_map` 产生布局。
- `vec()` 返回一次性数值副本，不改变内部数据。
- `layout()` 包含 `offset`，多维状态会占据连续位置。
- 注册及更新应做数值校验。校验失败抛出异常，旧值保持不变（内存分配异常的异常安全另按标准容器规则处理）。

### 4.4 数据一致性与线程语义

首版是**单线程所有权模型**，不保证多个线程同时读写安全。`freeze()` 不代表可在不同线程中无锁并发修改/读取值。对于一个采集线程和一个控制线程，应由外部加锁或交付已完成的状态快照。

未来确有并发读取需求时，再加 `snapshot()`（返回所有状态的一致性副本）、`setBatch()`（全量校验后一次提交）或版本号。不要先把 mutex、事件监听和原子缓存全部放进核心。

---

## 5. 公共 API 草案

### 5.1 最小接口矩阵

| 方法 | 作用 | 异常/约束 |
|---|---|---|
| `add<T>(name, initial, desc, vectorized)` | 注册动态状态并返回稳定下标 | 空名、重名、冻结、非法类型和值均拒绝 |
| `bind<T>(field, name, initial, desc, vectorized)` | 注册并绑定强类型字段 | 一个 `SField` 不能重复绑定 |
| `get<T>(name)` | 读取指定名称的值（**返回副本**） | 未找到或类型不符报错 |
| `set<T>(name, value)` | 更新指定状态值 | 类型、维度、有限性校验 |
| `state(name)` | 只读访问完整状态记录（名称、描述、值） | 返回 `const State&`；有效期依附空间 |
| `contains(name)` | 是否存在状态 | 布尔结果 |
| `size()` | 注册的状态条目数 | 包括不参与数值化的字符串状态 |
| `dim()` | `vec()` 输出长度 | 已注册向量分量数之和 |
| `vec()` | 生成数值向量 | 只输出 `vectorized=true` 的状态 |
| `layout()` | 获取向量布局和偏移 | 保持注册顺序 |
| `freeze()` / `frozen()` | 锁定/检查结构 | 单向锁定，无 unfreeze |

### 5.2 访问方式的对照

```cpp
// Dynamic access
SSp states;
states.add("velocity", 0.8, "Speed");
states.set("velocity", 1.2);
double v1 = states.get<double>("velocity");

// Typed-member access
struct Robot : SSp {
    SField<double> velocity;
    Robot() {
        bind(velocity, "velocity", 0.8, "Speed");
        freeze();
    }
};
Robot robot;
robot.velocity = 1.2;
double v2 = robot.velocity.get();

// Both APIs always refer to ONE value in the SAME object:
robot.set("velocity", 2.0);
assert(robot.velocity.get() == robot.get<double>("velocity"));
```

`auto x = robot.velocity;` 不建议使用：`SField` 是代理而不是 `double`，且故意禁用了复制。请用 `auto x = robot.velocity.get()`，或者 `double x = robot.velocity`。不提供 `operator T&()` 或 `T& get()`，以免绕过写入校验。

---

## 6. 三档用法与完整示例

### 6.1 简易用法：动态命名状态

```cpp
#include "AuroX/Space/StateSpace.hpp"
using namespace AuroX::Space;

SSp states;
states.add("position_x", 1.5, "X coordinate");
states.add("position_y", 2.0, "Y coordinate");
states.add("velocity", 0.8, "Speed");
states.add("mode", Str{"auto"}, "Control mode"); // string excluded by default
states.freeze();

states.set("velocity", 1.2);
double v = states.get<double>("velocity");
const State& s = states.state("velocity"); // s.name, s.desc, s.value
Vec input = states.vec();                  // {1.5, 2.0, 1.2}
```

**说明**：动态注册不需要继承、不需要宏。`state()` 返回只读记录，业务层可以查看名称与描述；算法只拿到 `Vec`。`Str{"auto"}` 特意显式构造为字符串，避免泛型注册时字符数组类型推导歧义。

### 6.2 标准用法：强类型成员 + 动态访问共存

```cpp
struct RobotState : AuroX::Space::SSp {
    AuroX::Space::SField<double> x;
    AuroX::Space::SField<double> y;
    AuroX::Space::SField<double> speed;
    AuroX::Space::SField<AuroX::Space::Str> mode;

    RobotState() {
        bind(x, "x", 0.0, "X coordinate");
        bind(y, "y", 0.0, "Y coordinate");
        bind(speed, "speed", 0.0, "Linear speed");
        bind(mode, "mode", AuroX::Space::Str{"idle"}, "Mode");
        freeze();
    }
};

RobotState robot;
robot.x = 1.5;                             // typed field
robot.set("speed", 1.2);                     // dynamic name
double speed = robot.speed.get();           // 1.2
double x = robot.get<double>("x");          // 1.5
auto vec = robot.vec();                     // {1.5, 0.0, 1.2}
```

**说明**：`bind()` 是注册并绑定的一步操作；不需要 `SStruct<Derived>`、`AUROX_STATES` 或 `sync_from_space()`。成员本身只是索引代理，真实数值存储于基类空间中。

### 6.3 高级用法：稳定布局 + 动作空间协作

```cpp
RobotState robot;
auto meta = robot.layout();
// x: offset=0, components=1
// y: offset=1, components=1
// speed: offset=2, components=1
auto x0 = robot.vec();

AuroX::Space::ASpace actions;
actions.cur(x0);

// Plan with a goal of the SAME dimension as robot.vec()
auto chain = actions.plan(AuroX::Space::Vec{3.0, 0.0, 0.0});
```

**说明**：高级用法的重点是布局与一致性，并非给 `StateSpace` 添加自己的规划算法。只有**作用于当前规划向量中明确包含的分量**，`Action::conditions` 和 `Action::nxt` 才能在不同假设状态之间正确推演。像 `mode` 这样不进入 `vec()` 的信息，不能被规划器自动还原为“各个假设状态下的模式”。涉及模式转换时，应在规划模型中显式编码模式，或覆写任务专用规划器。

### 6.4 多维状态展开

```cpp
SSp states;
states.add("pose", Vec{1.0, 2.0, 0.5}, "x, y, heading");
states.add("velocity", 0.8);
states.add("mode", Str{"auto"});
states.freeze();

auto vec = states.vec();  // {1.0, 2.0, 0.5, 0.8}
auto layout = states.layout();
// pose:     offset=0, components=3
// velocity: offset=3, components=1
```

如果优化器需要多维状态的某一部分，不要从 `unordered_map` 的遍历顺序推测下标；从 `layout()` 获取 `offset`。首版不提供任意命名投影 `vec({"pose","velocity"})`，因为固定完整布局已能覆盖当前需要；确需投影时再引入明确版本化的 projection 描述。

---

## 7. 可复制的 C++20 最小参考实现

以下完整代码已经在独立目录中通过 `g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror` 编译，并与**当前** `ActionSpace.cpp` 进行联动测试。**这是参考实现，尚未修改源仓库，也不是现有项目的正式接口承诺。**

建议目标文件结构：

```text
AuroX/
  include/AuroX/Space/
    ParameterSpace.hpp
    ParameterSpace.ipp
    ActionSpace.hpp
    StateSpace.hpp          <- new
    StateSpace.ipp          <- new, templates only
  src/Space/
    ParameterSpace.cpp
    ActionSpace.cpp
    StateSpace.cpp          <- new
  tests/
    StateSpace_demo.cpp     <- new, optional independent test
```

为保证这份原型独立、易读，示例类没有显式加 `AUROX_API` 导出宏；当前项目以 `STATIC` 库构建时可以直接使用。若后续需要构建 Windows DLL，应按项目现有 `ActionSpace.hpp` 风格为公开非模板类型增加统一导出宏。生产环境还需按项目代码风格补充更多测试。

### 7.1 `StateSpace.hpp`：声明与访问代理

定义异构值类型、状态记录、字段代理声明及空间公开接口。模板声明位于头文件；实现通过末尾的 `.ipp` 可见。

```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace AuroX::Space {
using Str = std::string;
using Vec = std::vector<double>;
using StateValue = std::variant<bool, int, std::int64_t, float, double, Str, Vec>;

namespace detail {
template <class T>
inline constexpr bool supported_state_v =
    std::is_same_v<T, bool> || std::is_same_v<T, int> ||
    std::is_same_v<T, std::int64_t> || std::is_same_v<T, float> ||
    std::is_same_v<T, double> || std::is_same_v<T, Str> ||
    std::is_same_v<T, Vec>;
template <class T>
inline constexpr bool default_vectorized_v =
    std::is_same_v<T, int> || std::is_same_v<T, std::int64_t> ||
    std::is_same_v<T, float> || std::is_same_v<T, double> ||
    std::is_same_v<T, Vec>;
}

struct State {
    Str name;
    Str desc;
    StateValue value;
    bool vectorized = false;
};

template <class T> class SField;

class StateSpace {
public:
    struct ComponentMeta {
        Str name;
        Str type;
        std::size_t offset = 0;
        std::size_t components = 0;
    };
    StateSpace() = default;
    StateSpace(const StateSpace&) = delete;
    StateSpace& operator=(const StateSpace&) = delete;
    StateSpace(StateSpace&&) = delete;
    StateSpace& operator=(StateSpace&&) = delete;

    template <class T>
    std::size_t add(Str name, T initial, Str desc = {},
                    bool vectorized = detail::default_vectorized_v<T>);
    template <class T>
    void bind(SField<T>& field, Str name, T initial, Str desc = {},
              bool vectorized = detail::default_vectorized_v<T>);
    template <class T> T get(const Str& name) const;
    template <class T> void set(const Str& name, T value);

    const State& state(const Str& name) const;
    bool contains(const Str& name) const;
    std::size_t size() const noexcept;
    std::size_t dim() const noexcept;
    Vec vec() const;
    std::vector<ComponentMeta> layout() const;
    void freeze() noexcept;
    bool frozen() const noexcept;

private:
    template <class T> friend class SField;
    std::size_t addRaw(Str name, StateValue value, Str desc, bool vectorized);
    void setRaw(std::size_t index, StateValue value);
    std::size_t indexOf(const Str& name) const;
    template <class T> T getRaw(std::size_t index) const;
    std::vector<std::unique_ptr<State>> items_;
    std::unordered_map<Str, std::size_t> index_;
    std::size_t vectorDim_ = 0;
    bool frozen_ = false;
};
using SSp = StateSpace;

template <class T>
class SField {
public:
    SField() = default;
    SField(const SField&) = delete;
    SField& operator=(const SField&) = delete;
    SField(SField&&) = delete;
    SField& operator=(SField&&) = delete;
    T get() const;
    operator T() const { return get(); }
    SField& operator=(T value);
    bool bound() const noexcept { return space_ != nullptr; }
private:
    friend class StateSpace;
    StateSpace* space_ = nullptr;
    std::size_t index_ = 0;
};
}
#include "StateSpace.ipp"
```

### 7.2 `StateSpace.ipp`：模板实现

模板函数在使用端编译时必须可见，因此放入 `.ipp` 并由 `.hpp` 包含。无需使用宏注册或 CRTP。

```cpp
#pragma once
#include <stdexcept>

namespace AuroX::Space {
template <class T>
std::size_t StateSpace::add(Str name, T initial, Str desc, bool vectorized) {
    static_assert(detail::supported_state_v<T>, "Unsupported state type");
    return addRaw(std::move(name), StateValue{std::move(initial)}, std::move(desc), vectorized);
}

template <class T>
void StateSpace::bind(SField<T>& field, Str name, T initial, Str desc, bool vectorized) {
    static_assert(detail::supported_state_v<T>, "Unsupported state type");
    if (field.bound()) throw std::logic_error("SField already bound");
    const std::size_t id = add<T>(std::move(name), std::move(initial), std::move(desc), vectorized);
    field.space_ = this;
    field.index_ = id;
}

template <class T>
T StateSpace::get(const Str& name) const {
    static_assert(detail::supported_state_v<T>, "Unsupported state type");
    return getRaw<T>(indexOf(name));
}

template <class T>
void StateSpace::set(const Str& name, T value) {
    static_assert(detail::supported_state_v<T>, "Unsupported state type");
    setRaw(indexOf(name), StateValue{std::move(value)});
}

template <class T>
T StateSpace::getRaw(std::size_t index) const {
    static_assert(detail::supported_state_v<T>, "Unsupported state type");
    auto* p = std::get_if<T>(&items_.at(index)->value);
    if (!p) throw std::invalid_argument("StateSpace: type mismatch");
    return *p;
}

template <class T>
T SField<T>::get() const {
    if (!space_) throw std::logic_error("SField not bound");
    return space_->template getRaw<T>(index_);
}

template <class T>
SField<T>& SField<T>::operator=(T value) {
    if (!space_) throw std::logic_error("SField not bound");
    space_->setRaw(index_, StateValue{std::move(value)});
    return *this;
}
}
```

### 7.3 `StateSpace.cpp`：非模板实现

负责唯一值的创建、受控更新、数值校验、向量展开、布局计算。正常开发时应放入项目的 `src/Space/StateSpace.cpp`。

```cpp
#include "AuroX/Space/StateSpace.hpp"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace AuroX::Space {
namespace {
std::size_t components(const StateValue& value, bool vectorized) {
    if (!vectorized) return 0;
    if (std::holds_alternative<Str>(value)) {
        throw std::invalid_argument("StateSpace: string cannot enter Vec");
    }
    if (const auto* v = std::get_if<Vec>(&value)) {
        if (v->empty()) throw std::invalid_argument("StateSpace: empty numeric Vec");
        return v->size();
    }
    return 1;
}

void validate(const StateValue& value) {
    const auto finite = [](double x) { return std::isfinite(x); };
    if (const auto* p = std::get_if<float>(&value)) {
        if (!finite(*p)) throw std::invalid_argument("StateSpace: nonfinite float");
    } else if (const auto* p = std::get_if<double>(&value)) {
        if (!finite(*p)) throw std::invalid_argument("StateSpace: nonfinite double");
    } else if (const auto* p = std::get_if<Vec>(&value)) {
        for (double x : *p)
            if (!finite(x)) throw std::invalid_argument("StateSpace: nonfinite Vec");
    }
}

const char* typeName(const StateValue& value) {
    if (std::holds_alternative<bool>(value)) return "bool";
    if (std::holds_alternative<int>(value)) return "int";
    if (std::holds_alternative<std::int64_t>(value)) return "int64";
    if (std::holds_alternative<float>(value)) return "float";
    if (std::holds_alternative<double>(value)) return "double";
    if (std::holds_alternative<Str>(value)) return "string";
    return "vector<double>";
}

void pack(const StateValue& value, Vec& out) {
    if (const auto* v = std::get_if<Vec>(&value)) {
        out.insert(out.end(), v->begin(), v->end());
    } else if (const auto* p = std::get_if<bool>(&value)) {
        out.push_back(*p ? 1.0 : 0.0);
    } else if (const auto* p = std::get_if<int>(&value)) {
        out.push_back(static_cast<double>(*p));
    } else if (const auto* p = std::get_if<std::int64_t>(&value)) {
        constexpr auto limit = std::int64_t{9007199254740992LL};
        if (*p > limit || *p < -limit)
            throw std::invalid_argument("StateSpace: int64 not exactly representable in double");
        out.push_back(static_cast<double>(*p));
    } else if (const auto* p = std::get_if<float>(&value)) {
        out.push_back(static_cast<double>(*p));
    } else if (const auto* p = std::get_if<double>(&value)) {
        out.push_back(*p);
    } else {
        throw std::logic_error("StateSpace: string in numeric layout");
    }
}
} // anonymous namespace

std::size_t StateSpace::addRaw(Str name, StateValue value, Str desc, bool vectorized) {
    if (frozen_) throw std::logic_error("StateSpace: schema frozen");
    if (name.empty()) throw std::invalid_argument("StateSpace: empty name");
    if (contains(name)) throw std::invalid_argument("StateSpace: duplicate name");
    validate(value);
    const auto n = components(value, vectorized);
    if (vectorized) {
        if (const auto* x = std::get_if<std::int64_t>(&value)) {
            constexpr auto limit = std::int64_t{9007199254740992LL};
            if (*x > limit || *x < -limit)
                throw std::invalid_argument("StateSpace: int64 not exactly representable in double");
        }
    }
    if (n > std::numeric_limits<std::size_t>::max() - vectorDim_)
        throw std::length_error("StateSpace: dimension overflow");
    const auto i = items_.size();
    items_.push_back(std::make_unique<State>(State{std::move(name), std::move(desc), std::move(value), vectorized}));
    try {
        index_.emplace(items_[i]->name, i);
    } catch (...) {
        items_.pop_back();
        throw;
    }
    vectorDim_ += n;
    return i;
}

void StateSpace::setRaw(std::size_t i, StateValue value) {
    auto& old = items_.at(i)->value;
    if (old.index() != value.index()) throw std::invalid_argument("StateSpace: type changes are prohibited");
    if (const auto* v = std::get_if<Vec>(&old)) {
        if (std::get<Vec>(value).size() != v->size())
            throw std::invalid_argument("StateSpace: Vec dimension changes are prohibited");
    }
    validate(value);
    if (items_[i]->vectorized && std::holds_alternative<std::int64_t>(value)) {
        const auto x = std::get<std::int64_t>(value);
        constexpr auto limit = std::int64_t{9007199254740992LL};
        if (x > limit || x < -limit)
            throw std::invalid_argument("StateSpace: int64 not exactly representable in double");
    }
    old = std::move(value);
}

std::size_t StateSpace::indexOf(const Str& name) const {
    const auto it = index_.find(name);
    if (it == index_.end()) throw std::out_of_range("StateSpace: unknown state " + name);
    return it->second;
}
const State& StateSpace::state(const Str& name) const { return *items_.at(indexOf(name)); }
bool StateSpace::contains(const Str& name) const { return index_.find(name) != index_.end(); }
std::size_t StateSpace::size() const noexcept { return items_.size(); }
std::size_t StateSpace::dim() const noexcept { return vectorDim_; }
void StateSpace::freeze() noexcept { frozen_ = true; }
bool StateSpace::frozen() const noexcept { return frozen_; }

Vec StateSpace::vec() const {
    Vec out;
    out.reserve(vectorDim_);
    for (const auto& item : items_)
        if (item->vectorized) pack(item->value, out);
    return out;
}
std::vector<StateSpace::ComponentMeta> StateSpace::layout() const {
    std::vector<ComponentMeta> out;
    std::size_t offset = 0;
    for (const auto& item : items_) {
        if (!item->vectorized) continue;
        const auto n = components(item->value, true);
        out.push_back(ComponentMeta{item->name, typeName(item->value), offset, n});
        offset += n;
    }
    return out;
}
} // namespace AuroX::Space
```


### 7.4 参考实现设计解释

1. **单一值存储**：`State::value` 是唯一实际数据；`SField<T>` 只保存 `space_ + index_`。没有 `sync_to_raw()`、`sync_from_raw()`。
2. **代理生命周期**：禁止复制和移动 `StateSpace`；同时禁止复制和移动 `SField`。不提供 `erase()` / `clear()`，防止绑定索引失效。
3. **地址稳定性**：`std::vector<std::unique_ptr<State>>` 的扩容只移动智能指针，不移动 `State` 实体；`state(name)` 的 `const State&` 在空间对象存活且没有删除操作时保持有效。
4. **返回值而不是引用**：`get<T>()` / `SField<T>::get()` 返回值副本，防止外部通过引用越过校验机制写入。
5. **冻结的是布局，不是数据**：注册完成后可 `freeze()`；依旧允许 `set()` 和代理赋值。
6. **默认策略可解释**：数值类型默认参与向量，字符串和布尔默认不参与。布尔只有明确开启时转为 0/1；字符串不能直接开启数值化。
7. **整数精度限制**：使用 `std::int64_t` 的状态若进入 double 向量，采用保守的绝对值不超过 `2^53` 的约束，避免一般整数在连续映射中出现非精确表示。
8. **固定多维长度**：`Vec` 状态可以改变分量值，但不能改变初始长度；否则下游 `ActionSpace` 中固定下标的条件与 `nxt` 会误读数据。
9. **异常策略**：未知名、类型不符、非法维度、重复注册均抛出异常。避免用 `0`/`false` 混淆合法状态值与访问失败。
10. **开销**：`vec()` 每次生成一份 `std::vector<double>`；它换来稳定、无别名的优化器输入。只有测得高频分配成为瓶颈后，再考虑增加 `fill(Vec&)` 以复用缓存，避免在核心内部维护第二份状态值。

### 7.5 有意没有实现的接口

- `unvec()`：优化器或动作规划器输出的候选状态，不应无条件覆盖现实状态。若后续增加，应采用显式 `applyEstimate()` 一类具有来源和校验语义的 API。
- `snapshot()`：有用，但跨线程的一致快照需要明确同步模型，不能只把 `items_` 复制一下就宣称线程安全。
- `setBatch()`：有用，首版暂无必要。未来必须先全量校验，再做一次整体提交，而不是循环调用可能中途失败的 `set()`。
- `remove()` / `replace()`：会使既有绑定失效或改变向量布局，首版禁止。
- `SStruct<Derived>`/宏自动反射：当前 `bind()` 已足够简洁；若以后出现上百字段造成重复注册代码，再提供语法糖，不增加底层数据层次。

---

## 8. 与当前 ActionSpace 的边界与闭环

### 8.1 四空间之间的依赖规则

```text
External observation source
        |
        v
ObservationSpace (future: timestamped input)
        |
        | estimator / task code
        v
StateSpace (current estimated state)
        |                ParameterSpace (configuration)
        | vec()                | captured by runtime if needed
        v                      v
     ActionSpace.cur(Vec) + Action::nxt / conditions
        |
        | plan -> run (predicted internal cur)
        v
External device / simulation
        |
        +--> observation --> corrected StateSpace --> sync actions.cur(actual)
```

**不要**使 `StateSpace` 保存 `ActionSpace&`，也不要使 `ActionSpace` 直接持有 `StateSpace*`。当前 `ActionSpace` 仅知道中性的 `Vec`，应当保留这一优点。

### 8.2 “运行成功”与“实际状态已达到目标”不是一回事

当前动作空间实现：

```cpp
a->exec();
cur_ = a->next(cur_);
return true;
```

它推进的是**动作模型的预测向量**。即使返回 `true`，仍需要外部获取设备完成确认以及新的观测，再更新 `StateSpace` 并重新同步 `actions.cur(states.vec())`。

比较时要遵守以下约定：

1. 执行前保存预测 `pred = action.next(actions.cur())`。
2. 执行后等待真实执行确认（不能仅以 `ActionSpace::run()` 的布尔值代替）。
3. 外部估计器把观测转为状态，更新 `StateSpace`。
4. 比较 `pred` 与 `actual = states.vec()`：**先检查维度和有限性**，再比较。
5. 当前 `dist()` 返回平方距离，因此阈值需为平方误差单位；不要把线性距离阈值直接当作平方阈值。
6. 发现偏差时把实测状态作为新规划起点，再决定是否 `ban` 失败动作并 `replan`。
7. `Action::nxt` 应视为针对假设状态的纯函数；不得写入 `StateSpace`，不应读取执行期间变化的真实状态作为计算依据。

### 8.3 状态与参数互相影响时

若预测依赖 PID 参数等可调变量，应由运行层明确选择：
- **快照参数策略**：规划期间固定一份参数，所有 `nxt` 使用同一份配置；
- **扩展状态策略**：如果动作会改变影响后续预测的参数，则将这些规划相关参数显式纳入规划向量或更一般的规划状态。

不能让 BFS 在相同假设状态上因为闭包读取到变化的实时参数而生成不同的 `nxt` 结果。

### 8.4 数值误差与单位

`x (m)`、`velocity (m/s)`、`heading (rad)` 的数值尺度不同，直接使用欧氏平方距离可能偏向量纲较大的分量。StateSpace 首版只负责按固定布局输出**原始数值**，不偷偷标准化；归一化、权重或带物理语义的误差函数应由优化器或定制规划策略决定。若要引入缩放，必须公开明确的编码契约，使 `vec()` 与 `layout()` 的语义可追溯。

---

## 9. 参考编译与测试

### 9.1 独立编译（不依赖当前尚未迁移的 Server）

把第 7 章三个文件保存到建议位置，并将后面示例保存为 `AuroX/tests/StateSpace_demo.cpp`。进入 `AuroX/` 目录后可执行：

```bash
g++ -std=c++20 -Wall -Wextra -Wpedantic -Werror \
  -I include \
  src/Space/StateSpace.cpp \
  src/Space/ActionSpace.cpp \
  tests/StateSpace_demo.cpp \
  -o state_demo
./state_demo
```

该命令仅使用 StateSpace/ActionSpace 的标准库依赖，不依赖 `json.hpp`、服务器、旧动力学/优化器模块；方便先完成重构后再处理完整工程接入。

### 9.2 `StateSpace_demo.cpp`：可运行的断言与集成示例

以下样例既测试状态管理，又与现有 `ActionSpace.cpp` 联编；通过模拟执行机构给出“预测值”和“实际测量值”的边界。

```cpp
#include "AuroX/Space/StateSpace.hpp"
#include "AuroX/Space/ActionSpace.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace AuroX::Space;
struct RobotState : SSp {
    SField<double> x, y, velocity;
    SField<Str> mode;
    RobotState() {
        bind(x, "x", 0.0, "X position");
        bind(y, "y", 0.0, "Y position");
        bind(velocity, "velocity", 0.0, "Speed");
        bind(mode, "mode", Str{"idle"}, "Mode");
        freeze();
    }
};

template <class F> bool throws(F&& fn) {
    try { fn(); return false; } catch (const std::exception&) { return true; }
}
int main() {
    SSp dyn;
    dyn.add("distance", 3.0, "Distance");
    dyn.add("name", Str{"auto"}, "Label");
    dyn.add("pose", Vec{1,2,3}, "Pose");
    dyn.freeze();
    assert(dyn.dim() == 4);
    assert(dyn.vec() == Vec({3,1,2,3}));
    assert(dyn.layout().at(1).offset == 1);
    assert(throws([&] {dyn.add("fail", 0.); }));
    assert(throws([&] {dyn.set("pose", Vec{1,2}); }));
    assert(throws([&] {dyn.set("distance", 3); }));
    assert(throws([&] {dyn.set("distance", std::numeric_limits<double>::quiet_NaN()); }));
    assert(throws([&] {SSp local; local.add("huge", std::int64_t{9007199254740993LL}); }));
    assert(throws([&] {SSp local; local.add("words", Str{"hi"}, "", true); }));
    RobotState robot;
    robot.x = 1.5;
    assert(robot.get<double>("x") == 1.5);
    robot.set("x", 2.0);
    assert(robot.x.get() == 2.0);
    const State& state = robot.state("x");
    assert(state.name == "x" && state.desc == "X position");
    assert(robot.dim() == 3);
    assert(robot.vec() == Vec({2,0,0}));
    double physicalX=2.0;
    ASpace actions;
    Act move;
    move.name = "move_x";
    move.type = "motion";
    move.run = [&] { physicalX += 1.0; };
    move.nxt = [](const Vec& v) { Vec y = v; y[0] += 1; return y; };
    actions.reg(move);
    actions.cur(robot.vec());
    Chain chain = actions.plan(Vec{4.0,0.0,0.0});
    assert(chain.ok && chain.acts.size() == 2);
    for (auto h: chain.acts) {
        actions.cur(robot.vec());
        const Vec predicted=actions.get(h)->next(actions.cur());
        assert(actions.run(h));
        robot.x = physicalX;
        const Vec actual=robot.vec();
        assert(actual.size() == predicted.size());
        assert(dist(actual,predicted)<=1e-12);
        actions.cur(actual);
    }
    assert(robot.x.get() == 4);
    std::cout << "PASS: typed/dynamic access, layout, validation, ActionSpace integration\n";
}
```


### 9.3 测试结论与限制

本次已在独立的 Linux C++20 编译环境执行上述参考代码及断言测试，覆盖：

- 按名字添加、查询、更新；
- 强类型字段赋值与动态 `set()` 读取的一致性；
- 字符串状态默认排除、数值向量有序展开、`layout()` 偏移；
- 冻结后拒绝新增；
- 拒绝向量长度变化、类型变化、NaN、超出安全精度的 int64、强行向量化字符串；
- 与当前 `ActionSpace.cpp` 联动，规划两步并比较预测和模拟实测。

运行输出：

```text
PASS: typed/dynamic access, layout, validation, ActionSpace integration
```

这说明**参考原型可编译并通过指定单元测试**，不等于完整 AuroX 工程已经完成集成，也不表示并发、真实硬件、DLL 导出、极限性能已经验证。

### 9.4 需要补充的测试矩阵

| 测试场景 | 必须保证 |
|---|---|
| 同名、空名注册 | 拒绝且已有状态不受影响 |
| 重复绑定同一 `SField` | 报错且原绑定不变 |
| 名字不存在 / 类型错误 | 失败可诊断，不能悄悄返回 0 |
| `SField` 未绑定就读写 | 报错 |
| `freeze()` 后读写已有值 | 允许 |
| `freeze()` 后注册新值 | 拒绝 |
| 多维状态 `Vec` 更新 | 只能用相同维度 |
| `vec()` / `layout()` | 维度、顺序、偏移严格一致 |
| `bool` / `string` | 默认排除向量；bool 明确开启后按 0/1；string 不可隐式编码 |
| NaN、Inf、精度不安全的 int64 | 拒绝参与非法数值化 |
| 动作预测返回错误维度 | 集成层或修复后的 `ActionSpace` 拒绝 |
| 预测与实测不一致 | 以实测重新同步，必要时重规划 |
| 异常安全 | 值校验失败不修改原值；批量更新（未来）需全有或全无 |
| 生命周期 | 编译期禁止复制/移动绑定空间；相关代理不悬空 |
| 并发使用 | 未提供线程安全承诺，集成层负责同步；未来单独加测试 |

---

## 10. 工程接入与分阶段实施

### Phase 1：核心类型和基本契约

新增 `StateSpace.hpp`、`StateSpace.ipp`、`StateSpace.cpp`。实现：
- `State`、`SField<T>`、`StateSpace`；
- `add/bind/get/set/state`；
- `size/dim/vec/layout/freeze`；
- 类型、维度和有限性检查。

**验收**：独立编译；动态和强类型两套 API 可以交叉读写同一个值；没有任何 `sync_to_space()`。

### Phase 2：动作空间接入和正确性修补

- 保留 `ActionSpace::cur(Vec)` 的显式同步边界。
- 修正 A-01，避免错误维度被识别为零距离。
- 补齐 A-02 的成功规划图边行为。
- 在示例中用模拟设备 / 外部观测确认“预测与实测不同”的处理。

**验收**：正常规划成功；目标维度错误时失败明确；成功规划请求图数据时能取到边；模拟实测偏差触发重新同步。

### Phase 3：旧示例清理和构建管理

- 将过时的 `AuroXServer/main.cpp` 从**新空间验收链**中隔离，逐步重写为当前接口。
- 新 `.cpp` 加入工程并重新配置 CMake。若仍用 GLOB，建议：

```cmake
file(GLOB_RECURSE AUROX_SRC CONFIGURE_DEPENDS src/*.cpp)
```

- 当前 `Core.hpp` 使用 `__has_include`，新增头文件后可以被其条件包含；不要以其条件包含成功代替库链接和测试。
- 准备精简、可维护的 Space 独立测试目标。

### Phase 4：需求驱动的能力扩展（不先行落地）

按真正遇到的问题再考虑：
- 多线程状态快照；
- 带版本号的布局 / 按名称选择投影；
- 校验后一次提交的 `setBatch()`；
- 状态持久化与历史；
- 针对大量状态的向量缓冲复用；
- 跨空间更明确的数值编码描述与元数据。

---

## 11. 关键架构决策记录（ADR）

| 决策 | 选择 | 原因 |
|---|---|---|
| 唯一值存储 | `StateSpace` 独占 `State::value` | 避免 Param 式双份值同步问题 |
| 强类型访问 | `SField<T>` 非拥有代理 | 无需 CRTP 和成员复制 |
| 动态访问 | 按名称 `get<T>/set<T>` | 支持脚本/配置驱动调用 |
| 状态模型 | `variant` 固定常见类型 | 足以覆盖控制/视觉统计/小型向量，不引入虚基类 |
| 名称冲突 | 拒绝重复注册 | 不破坏代理与布局 |
| 布局 | 注册顺序 + `offset/components` | 数值算法可复现和可解释 |
| `freeze` | 只锁结构、不锁数值 | 允许控制循环持续更新 |
| 数值化 | 显式按类型与 `vectorized` 规则 | 避免把标签错误解释为可优化连续变量 |
| 写入策略 | `set`/代理赋值，拒绝裸 `T&` | 确保类型与维度校验不可绕过 |
| `unvec` | 首版不提供 | 候选解/预测解不应直接覆盖实测状态 |
| 预测 | 保留 `Action::nxt` | 空间职责不重叠 |
| 历史与观测 | 外部模块负责 | 维持 StateSpace 轻量 |
| 线程安全 | 不内置 | 不在无明确需求时引入互斥及性能成本 |
| 文件结构 | `.hpp` / `.ipp` / `.cpp` | 与当前 ParameterSpace 模板分离风格一致 |

---

## 12. 结论及建议的验收定义

**最终建议：统一调用体验，而不是统一功能复杂度。**

- `ParameterSpace`：管理可调配置与参数约束、向量化。
- `ActionSpace`：管理动作条件、预测、规划和重规划。
- `StateSpace`：管理带语义的当前状态与稳定的数值视图。
- `ObservationSpace`（后续）：管理外部观测的采集、时间和有效性。

StateSpace V1 的“完成”应以以下可测标准界定：

1. 用户能通过 `name + desc + value` 查看每个实际状态；
2. 用户既能 `states.get<T>("name")`，也能 `robot.member.get()` / `robot.member = value`；
3. 两种访问始终共享唯一值，不需要调用任何同步函数；
4. `vec()`、`layout()` 与注册顺序精确一致，并适配 `ActionSpace::cur(Vec)`；
5. 重名、错误类型、非有限值和错误维度不会静默损坏状态；
6. 初版只包含必要的数据抽象，不重新引入重型继承层、状态预测和复杂调度框架；
7. 完整 C++20 样例有编译测试，未实现模块与旧示例不影响其独立验收。

**特别提醒**：文档附带的 StateSpace 代码已通过局部独立测试，仍应在真实项目目标平台、编译选项和未来模块集成后再次验证。此文档提供的是可落地参考设计，不代表已经替换现有工程源码。

