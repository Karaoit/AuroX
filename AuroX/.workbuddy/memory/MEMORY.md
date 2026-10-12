# AuroX 项目长期记忆

## 项目结构 / 构建
- C++20，AuroX 为 STATIC 库（CMakeLists.txt），源文件 `src/**/*.cpp`，头文件 `include/AuroX/**`。
- 外部依赖：`../Library/nlohmann/json`（json.hpp）、`../Library/cpp-httplib`。
- `include/AuroX/Core.hpp` 用 `__has_include` 条件包含各 Space 头；**条件包含成功 ≠ 已链接/已测试**。
- `apps/AuroXServer/main.cpp` 已是可编译的文件快照 CLI（ParameterSpace+StateSpace+ActionSpace+Planner，10.12 迁移完成并通过「fast_copy 失败→ban→重规划」回归）。

## Space 层设计约定（SSpace重构.md）
- 设计语言：“如非必要勿增实体”。ActionSpace = 3 类型（Condition/Action/ActionSpace）+ 1 自由函数（hashOf）+ 0 宏，仅依赖标准库。
- ActionSpace 内部存 `Action*` → 指向派生类自己的成员 → 拷贝被 `delete`；`nxt` 必须是纯预测、同维、有限值。
- `ActionSpace` **不持有状态、不含搜索**（10.12 移除 cur_/cur()/plan()/replan()/dist()/BFS）：`run(id,state)` 只查 conditions 后执行，不推演/不推进状态；`run(id)` 仅对无条件动作有效。
- ParameterSpace：不允许写旁路（无 `operator T&`/非 const `get()`），写入走 `operator=`；重名注册抛异常；`fromJson` 先解析后整体提交；`unvectorizeDouble` 全量校验后原子提交；`ComponentMeta` 带 `offset`。
- 三档用法是**包含关系**：简易（reg+run）⊂ 标准（conditions+nxt+Planner::bfs）⊂ 高级（ban+Planner::bfs 重规划）。
- **StateSpace 已实现**（10.11，最小可用三档）：`include/AuroX/Space/StateSpace.hpp`（仅声明+注释+示例）/ `StateSpace.ipp`（模板实现）/ `src/Space/StateSpace.cpp`（非模板实现）。
  - 唯一数据源：`State::value`（`variant<bool,int,int64,float,double,Str,Vec>`）；`SField<T>` 只存 `StateSpace* + index`，无副本、无 sync 函数。
  - `vec()` 只输出 `vectorized=true` 的状态（计数类型默认 true，bool/Str 默认 false）；`layout()` 给 `offset/components`。
  - `add/bind/get/set` 为模板；`addRaw/setRaw/vec/layout/...` 为非模板（`.cpp`）。
  - 拷贝/移动全 delete；无 `unvec/snapshot/setBatch/remove`；`int64` 向量化时限 |v|≤2^53；拒绝 NaN/Inf、空 Vec、改变已声明类型或 Vec 长度。
  - `Core.hpp` 已用 `__has_include` 自动接入，无需改动。
- **Space 层文档/演示**：`include/AuroX/Space/Readme.md`（四空间中文说明，四个空间均已实现）；`AuroX/四空间演示.html`（根目录，纯前端单文件交互演示，四个标签页）。

## Planner 模块（10.12，V0.5 解耦）
- 目录：`include/AuroX/Planner/Planner.hpp`（仅声明）+ `src/Planner/BFS.cpp`（实现）。依赖**单向** Planner→Space；`ActionSpace.cpp` 已不含任何搜索。
- 实体最小：`Planner::Chain{ok,why,acts}`（**无 edges**）+ 别名 `PlanFn`/`PredictFn`（std::function，非类）+ 函数 `bfs()`。明确**不引入** IPlanner/工厂/LLM 类/WorldModelBase。
- `bfs(actions,start,goal,predict={},maxDepth=8,eps=1e-6,maxNodes=10000)`：前向 BFS，按 `ids()` 注册序枚举；跳过 banned/无 run/条件不满足；提供 `predict` 时**只用它**（返回 nullopt 不回退 `nxt`），否则只用有 `nxt` 的动作；预测须同维有限否则 `ok=false`；不输出搜索图；`eps` 为平方误差单位。
- `ActionSpace` 新增只读面：`const Action* get(hash) const`、`const std::vector<uint64_t>& ids() const`。
- 断链语义：`Action::next()` 缺 `nxt` 抛 `std::logic_error`（≠状态不变）；`run` 空闭包返回 false；闭包异常向外传播不吞。
- `Space::dist()` 已移除；误差比较由任务侧自做（`main.cpp` 用本地 `sqDist`）。
- `Core.hpp` 已加 `Planner/Planner.hpp` 条件包含。CMake `GLOB_RECURSE src/*.cpp` 自动收 `src/Planner/BFS.cpp`，但**已存在的构建目录需重新 configure**。
- 验收：MSVC `/W4` 零警告；`AuroX_Exe` 默认流程（注入 fast_copy 失败→检测偏差→ban→bfs 重规划→SUCCESS）通过；18 项契约测试全过。

## ObservationSpace 模块（10.12，V0.5）
- 目录：`include/AuroX/Space/ObservationSpace.hpp`（仅声明）+ `src/Space/ObservationSpace.cpp`（实现，**无 .ipp**）。仅依赖标准库；`Str`/`Vec` 与其它空间同源（重复同类型别名合法）。
- 单类，**无**通道继承族 / 编码器 / 补偿器 / 工厂 / JSON / 插件。内嵌 `ChannelSlice`/`Bundle`（结果载体，非系统对象）；私有 `Sample{Vec values, optional<double> timestamp}` + `Channel{name,sampleDim,historySize,labels,deque<Sample>}`。别名 `OSpace`。
- API：`add(name, sampleDim=1, historySize=1)`、`addCategory(name, labels, historySize=1)`、`ingest(name, double|Vec|Str, ts=nullopt)`、`latest(name)->optional<Vec>`、`collect()->Bundle`、`freeze()`、`reset()`。
- 语义契约：`historySize` = **条数**非秒；窗口**左补零、最新在最右**；`validCount` 统计真正收到（**真实 0 也计入**，禁止用全零判断有效性）；`ts` 用 `optional`（缺失≠`t=0`），同通道**非递减**，`lastTimestamp` 只反映最新样本；非法输入一律抛 `std::invalid_argument`（未知通道/类型/长度/类别/NaN/Inf，**无静默 no-op**）；`freeze` 后禁 `add` 但 `ingest` 可用；`reset` 只清样本。一把 `mutable mutex` 保护写入与快照。
- `Core.hpp` 早已 `__has_include` 该头，无需改；CMake `GLOB_RECURSE` 自动收新 cpp（旧构建目录需重配）。
- 验收：MSVC `/W4` 零警告；`observation_test.cpp` 覆盖设计 §6C 全条目（含并发 ingest/collect、简易/标准/高级三例）**全过**。
- 勘误：设计文档 §6A 要求删除的旧通道继承族/`ObservationEncoder`/`ObservationCompensator` 在本快照中**并不存在**，仅文档提及。

## 本机 MSVC 编译姿势（无 cmd 时）
PowerShell 工具禁 `cmd.exe`；CMake 识别不了 VS18 preview；`Launch-VsDevShell.ps1` 非交互失败。手工拼环境直调 cl：
- VC：`C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.51.36231`
- SDK：`C:\Program Files (x86)\Windows Kits\10`，版本 `10.0.26100.0`
- `INCLUDE`=VC\include; SDK\Include\<v>\{ucrt,um,shared,cppwinrt}；`LIB`=VC\lib\x64; SDK\Lib\<v>\{ucrt,um}\x64；`PATH`+=VC\bin\Hostx64\x64; SDK\bin\<v>\x64
- 输出重定向到文件再 Read（工具不回显 stdout）。

## 用户偏好
- 中文回复；喜欢表格对比 + 代码片段 + 直接给结论；习惯连续追问深入细节。
- 期望改动**范围可控**：明确“只改 X、不新增 Y”时严格遵守。
