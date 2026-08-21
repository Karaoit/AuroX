# AuroX-General 技术路线文档

> 版本：v0.1  
> 定位：从 AuroX-Lite 轻量化规则优化器，扩展为面向视觉、控制、主动视觉与具身智能任务的通用闭环优化器。  
> 文档目标：定义 AuroX-General 的统一抽象层、各层职责、实现方式、数据闭环、优化机制与阶段性落地路线。  
> 说明：本文档为技术路线说明，不包含代码实现。

---

## 1. 背景与目标

### 1.1 AuroX-Lite 的现有基础

AuroX-Lite 当前是一个面向晶圆显微图像的轻量化自动优化与几何推理算法。它的核心能力包括：

1. 基于图像处理提取候选晶粒；
2. 基于鲁棒统计估计当前图像尺度；
3. 基于近邻关系推理二维网格；
4. 基于图像边缘证据和四邻域拓扑补偿缺失点；
5. 通过运行时动态参数适应不同图像尺度与局部质量变化。

其本质不是神经网络训练模型，而是：

```text
图像处理 + 几何约束 + 鲁棒统计 + 规则网格推理 + 动态参数更新
```

AuroX-Lite 的优势在于可解释、轻量、部署成本低、安全拒绝机制明确。  
但它的能力边界也较清晰：主要适用于规则晶粒阵列、边界可见、网格近似正交、目标尺寸相对稳定的场景。

### 1.2 AuroX-General 的目标

AuroX-General 的目标不是简单扩大 AuroX-Lite 的参数范围，而是将其升级为一个通用闭环优化框架：

```text
各类任务
    ↓
统一抽象为状态、动作、参数、反馈、损失、优化目标
    ↓
通过运行数据学习任务代理模型或近似状态方程
    ↓
通过优化器选择下一组参数或动作
    ↓
再次运行任务并收集反馈
    ↓
形成可持续改进的闭环系统
```

最终希望支持：

| 任务类型 | 示例 |
|---|---|
| 视觉任务 | 晶粒检测、目标识别、缺陷定位、ROI 初始化 |
| 控制任务 | PID 自整定、运动控制、温控、伺服控制 |
| 混合任务 | 主动视觉、机器人看-动闭环、在线检测与调整 |
| 规划任务 | 参数搜索、路径选择、工艺调参、实验设计 |
| 人机反馈任务 | 人工验收、偏好反馈、少样本调参 |

---

## 2. 核心设计原则

### 2.1 统一抽象，不统一算法

AuroX-General 不应该用一个固定算法解决所有任务。  
它应该统一任务接口，但允许不同任务选择不同优化器。

```text
统一的是：
状态、动作、参数、反馈、损失、历史记录、优化接口

不统一的是：
具体模型、具体优化器、具体任务插件、具体评价指标
```

例如：

| 场景 | 合适优化器 |
|---|---|
| 小规模黑箱参数优化 | Bayesian Optimization / TPE |
| 中等维度不可微参数优化 | CMA-ES / Evolution Strategy |
| 高噪声在线调参 | SPSA / Bandit |
| 有明确动力学模型 | MPC / System Identification |
| 长序列决策 | Reinforcement Learning |
| 有人工偏好反馈 | Preference Learning / Dueling Bandit |

### 2.2 以闭环为核心

AuroX-General 的基本运行闭环是：

```text
观察状态
    ↓
选择动作或参数
    ↓
执行任务
    ↓
得到结果
    ↓
接收反馈
    ↓
计算损失
    ↓
更新模型 / 优化器 / 策略
    ↓
进入下一轮
```

这使它从“单次运行自适应”升级为“跨运行学习与优化”。

### 2.3 安全优先

通用优化器必须有安全边界。  
特别是在控制和具身智能场景中，优化器不能为了探索而让系统进入危险状态。

安全机制包括：

1. 参数边界；
2. 动作边界；
3. 运行状态监控；
4. 约束惩罚；
5. 失败早停；
6. 不确定性过高时拒绝执行；
7. 人工确认机制；
8. 只在仿真环境中允许高风险探索。

### 2.4 可解释优先

AuroX-Lite 的核心优势之一是可解释。  
AuroX-General 应该继承这一点。

每一次优化建议都应该能回答：

1. 当前状态是什么？
2. 优化器为什么选择这组参数？
3. 预计改善哪个指标？
4. 风险在哪里？
5. 违反了哪些约束？
6. 如果失败，失败原因属于哪一层？

---

## 3. AuroX-General 总体架构

### 3.1 分层架构总览

```text
┌────────────────────────────────────────────┐
│ Layer 9：Application & Domain Plugins       │
│ 视觉 / 控制 / 主动视觉 / 机器人 / 工艺任务   │
├────────────────────────────────────────────┤
│ Layer 8：Policy & Agent Layer               │
│ 策略、动作选择、任务级决策                   │
├────────────────────────────────────────────┤
│ Layer 7：Optimizer Orchestration Layer      │
│ 优化器调度、搜索策略、超参数选择             │
├────────────────────────────────────────────┤
│ Layer 6：Loss & Feedback Layer              │
│ 多目标损失、奖励、约束、人工反馈             │
├────────────────────────────────────────────┤
│ Layer 5：Surrogate & Dynamics Model Layer   │
│ 代理模型、状态方程近似、结果预测             │
├────────────────────────────────────────────┤
│ Layer 4：Execution Runtime Layer            │
│ reset / step / evaluate / rollback          │
├────────────────────────────────────────────┤
│ Layer 3：Task Abstraction Layer             │
│ 状态空间、动作空间、参数空间、观测空间       │
├────────────────────────────────────────────┤
│ Layer 2：Data & Memory Layer                │
│ episode、history、dataset、经验库            │
├────────────────────────────────────────────┤
│ Layer 1：Safety & Constraint Layer          │
│ 安全边界、约束校验、拒绝机制、权限控制       │
├────────────────────────────────────────────┤
│ Layer 0：System Interface Layer             │
│ 传感器、执行器、图像、控制器、外部系统接口   │
└────────────────────────────────────────────┘
```

### 3.2 每一层的核心问题

| 层级 | 名称 | 解决的问题 |
|---|---|---|
| Layer 0 | System Interface | 如何连接真实系统或仿真系统 |
| Layer 1 | Safety & Constraint | 什么不能做，什么情况下必须拒绝 |
| Layer 2 | Data & Memory | 如何记录运行历史并形成经验 |
| Layer 3 | Task Abstraction | 如何把不同任务统一成标准形式 |
| Layer 4 | Execution Runtime | 如何执行一次任务并返回标准结果 |
| Layer 5 | Surrogate & Dynamics | 如何从历史中学习近似模型 |
| Layer 6 | Loss & Feedback | 如何把结果转成可优化目标 |
| Layer 7 | Optimizer Orchestration | 如何选择优化器并生成下一步建议 |
| Layer 8 | Policy & Agent | 如何在多步任务中选择动作 |
| Layer 9 | Domain Plugins | 如何接入具体业务场景 |

---

# 4. Layer 0：System Interface Layer

## 4.1 定义

System Interface Layer 是 AuroX-General 与外部世界连接的接口层。  
它负责把不同来源的数据、设备和执行系统接入统一框架。

外部系统包括：

1. 图像输入；
2. 相机；
3. 机械臂；
4. 运动平台；
5. PID 控制器；
6. 仿真环境；
7. 产线检测系统；
8. 人工标注或验收系统；
9. 数据库和日志系统。

## 4.2 输入输出

| 类型 | 示例 |
|---|---|
| 输入 | 图像、传感器值、当前控制误差、机器人位姿、人工反馈 |
| 输出 | 参数配置、控制量、动作命令、相机调整、任务执行请求 |

## 4.3 实现方式

该层不关心优化逻辑，只负责标准化输入输出。

建议实现为适配器模式：

| 适配器 | 说明 |
|---|---|
| ImageAdapter | 接入图片、视频流、显微图像 |
| SensorAdapter | 接入温度、压力、位置、电流等传感器 |
| ControllerAdapter | 接入 PID、PLC、运动控制器 |
| RobotAdapter | 接入机械臂、移动平台、末端执行器 |
| SimulationAdapter | 接入仿真环境 |
| HumanFeedbackAdapter | 接入人工评分、验收、标注结果 |

## 4.4 标准化原则

所有外部输入最终都要转成统一观测：

```text
Observation = {
  raw_input,
  timestamp,
  source,
  metadata,
  quality,
  confidence
}
```

例如图像任务：

```text
Observation = {
  raw_input: image,
  source: microscope_camera,
  metadata: {
    width,
    height,
    magnification,
    exposure
  },
  quality: {
    brightness,
    contrast,
    blur_score
  }
}
```

PID 任务：

```text
Observation = {
  raw_input: sensor_value,
  metadata: {
    target_value,
    sampling_rate
  },
  quality: {
    noise_level,
    missing_rate
  }
}
```

---

# 5. Layer 1：Safety & Constraint Layer

## 5.1 定义

Safety & Constraint Layer 负责定义优化器不能突破的边界。  
它是所有层的前置保护机制。

任何参数、动作、策略、模型建议，在执行前都必须经过该层检查。

## 5.2 约束类型

| 约束类型 | 示例 |
|---|---|
| 参数边界 | `Kp_min <= Kp <= Kp_max` |
| 动作边界 | 机械臂不能进入禁区 |
| 状态边界 | 温度不能超过上限 |
| 资源边界 | 单次运行时间不能超过阈值 |
| 质量边界 | 图像质量过低时拒绝推理 |
| 置信度边界 | 模型不确定性过高时要求人工确认 |
| 任务边界 | 非规则网格任务不能强行使用网格推理 |
| 权限边界 | 高风险动作必须人工授权 |

## 5.3 实现方式

该层由三部分组成：

### 5.3.1 Static Constraints

静态约束在任务开始前定义：

| 字段 | 说明 |
|---|---|
| parameter_bounds | 参数上下限 |
| action_bounds | 动作范围 |
| forbidden_states | 禁止状态 |
| max_runtime | 最大运行时间 |
| max_cost | 最大成本 |
| required_approval | 是否需要人工确认 |

### 5.3.2 Runtime Guards

运行时守卫用于监控系统状态：

| 守卫 | 作用 |
|---|---|
| StateGuard | 检查状态是否异常 |
| ActionGuard | 检查动作是否越界 |
| ParameterGuard | 检查参数是否安全 |
| ResultGuard | 检查结果是否可信 |
| UncertaintyGuard | 检查预测不确定性 |
| EmergencyStop | 触发停止或回滚 |

### 5.3.3 Constraint Penalty

软约束可以进入损失函数：

```text
L_safety = Σ μ_j * max(0, violation_j)^2
```

硬约束不能只做惩罚，必须直接拒绝执行。

## 5.4 输出

该层输出：

```text
SafetyDecision = {
  allowed,
  rejected_reason,
  violated_constraints,
  adjusted_action,
  adjusted_parameters,
  require_human_approval
}
```

## 5.5 设计要点

1. 安全层优先级高于优化层；
2. 优化器不能绕过安全层；
3. 拒绝执行也是有效输出；
4. 安全拒绝应被记录为训练数据；
5. 不确定性高时应保守处理。

---

# 6. Layer 2：Data & Memory Layer

## 6.1 定义

Data & Memory Layer 负责记录每一次任务运行的数据，并将其转化为可复用经验。

AuroX-Lite 当前主要是单次运行自适应。  
AuroX-General 需要将每次运行都沉淀为经验：

```text
参数 → 执行 → 结果 → 反馈 → 损失 → 下一轮优化
```

## 6.2 核心数据单元：Episode

一次完整运行称为一个 Episode。

```text
Episode = {
  task_id,
  context,
  initial_state,
  observation,
  parameters,
  action,
  result,
  feedback,
  loss,
  next_state,
  safety_decision,
  optimizer_decision,
  timestamp
}
```

## 6.3 数据类型

| 数据 | 说明 |
|---|---|
| Context | 任务背景，例如图像类型、设备、环境 |
| State | 当前任务状态 |
| Parameters | 本轮使用的可调参数 |
| Action | 本轮采取的动作 |
| Result | 系统输出结果 |
| Feedback | 人工或自动反馈 |
| Loss | 计算得到的损失 |
| NextState | 执行后的新状态 |
| SafetyLog | 安全检查结果 |
| OptimizerLog | 优化器建议原因 |
| Artifacts | 图像、日志、曲线、可视化结果 |

## 6.4 记忆类型

| 记忆类型 | 作用 |
|---|---|
| Short-term Memory | 当前任务内的运行历史 |
| Long-term Memory | 跨任务、跨图像、跨设备经验 |
| Failure Memory | 失败案例库 |
| Best Config Memory | 最优参数库 |
| Transfer Memory | 相似任务迁移经验 |
| Human Preference Memory | 人工偏好和验收标准 |

## 6.5 实现方式

建议采用三层存储：

| 存储层 | 内容 |
|---|---|
| Structured Log | JSON / 表格形式的结构化 episode |
| Artifact Store | 图像、可视化、运行报告、模型文件 |
| Vector / Similarity Index | 用于查找相似任务和历史经验 |

## 6.6 相似任务检索

当新任务开始时，系统先检索历史中相似任务：

```text
new_task_context
    ↓
similarity_search
    ↓
retrieve previous best configs
    ↓
warm-start optimizer
```

相似度可以来自：

1. 图像尺寸；
2. 目标尺度；
3. 设备类型；
4. 光照条件；
5. 任务类别；
6. 状态统计量；
7. 历史损失曲线；
8. 人工标签。

---

# 7. Layer 3：Task Abstraction Layer

## 7.1 定义

Task Abstraction Layer 是 AuroX-General 的核心统一抽象层。  
它把不同任务统一表示为：

```text
Task = {
  StateSpace,
  ObservationSpace,
  ParameterSpace,
  ActionSpace,
  StateEquation,
  ResultSpace,
  FeedbackSpace,
  LossFunction,
  Constraints,
  TerminationCondition
}
```

## 7.2 状态空间 StateSpace

状态空间描述任务当前处于什么情况。

```text
s_t ∈ S
```

状态不一定等于原始输入。  
状态是从输入中提炼出来、对优化有用的变量集合。

### 7.2.1 视觉任务状态

| 状态变量 | 说明 |
|---|---|
| image_quality | 图像质量 |
| brightness | 亮度 |
| contrast | 对比度 |
| blur_score | 模糊程度 |
| candidate_count | 候选目标数量 |
| median_object_size | 目标中位尺寸 |
| edge_strength | 边缘强度 |
| grid_pitch | 网格间距 |
| residual_mad | 网格残差统计 |
| detector_confidence | 检测置信度 |

### 7.2.2 控制任务状态

| 状态变量 | 说明 |
|---|---|
| target_value | 目标值 |
| current_value | 当前值 |
| error | 当前误差 |
| integral_error | 积分误差 |
| derivative_error | 误差变化率 |
| actuator_output | 执行器输出 |
| saturation_state | 是否饱和 |
| disturbance_estimate | 扰动估计 |
| stability_score | 稳定性评分 |

### 7.2.3 主动视觉任务状态

| 状态变量 | 说明 |
|---|---|
| robot_pose | 机器人位姿 |
| camera_pose | 相机位姿 |
| visual_observation | 当前视觉观测 |
| target_belief | 目标存在概率 |
| uncertainty_map | 不确定性地图 |
| occlusion_map | 遮挡信息 |
| view_history | 已观察视角 |
| energy_used | 已消耗能量 |
| time_used | 已用时间 |

## 7.3 观测空间 ObservationSpace

观测是系统能直接看到的数据：

```text
o_t = H(s_t)
```

状态是真实或估计的内部变量，观测是可获得的外部数据。  
在很多任务中，状态不可完全观测，只能通过观测估计。

| 任务 | 观测 |
|---|---|
| 视觉 | 图像、视频帧、ROI |
| 控制 | 传感器读数、目标值、误差 |
| 主动视觉 | 图像 + 位姿 + 历史视角 |
| 工艺调参 | 参数配置 + 产品检测结果 |
| 人工反馈 | 人工评分、接受/拒绝、备注 |

## 7.4 参数空间 ParameterSpace

参数是任务执行过程中可调节但不一定每一步都直接动作的变量。

```text
θ ∈ Θ
```

### 7.4.1 参数分类

| 参数类型 | 说明 | 示例 |
|---|---|---|
| Continuous | 连续参数 | 阈值、增益、权重 |
| Discrete | 离散参数 | 核大小、迭代次数 |
| Boolean | 开关参数 | 是否启用 fallback |
| Categorical | 类别参数 | 使用哪种检测器 |
| Structured | 结构化参数 | 一组规则组合 |

### 7.4.2 参数元信息

每个参数都应该定义：

| 元信息 | 说明 |
|---|---|
| name | 参数名 |
| type | 参数类型 |
| default | 默认值 |
| lower_bound | 下界 |
| upper_bound | 上界 |
| step | 离散步长 |
| safety_level | 安全等级 |
| sensitivity | 历史敏感度 |
| dependency | 与其他参数的依赖关系 |
| description | 物理或业务含义 |

## 7.5 动作空间 ActionSpace

动作是系统在当前状态下主动采取的操作。

```text
a_t ∈ A
```

动作和参数的区别：

| 项目 | 参数 | 动作 |
|---|---|---|
| 含义 | 改变任务执行方式 | 改变环境或流程 |
| 示例 | 阈值、Kp、置信度下限 | 移动相机、重新拍照、改变目标点 |
| 时间性 | 通常在一次运行前设定 | 通常在运行过程中发生 |
| 影响 | 改变算法行为 | 改变系统状态 |

### 7.5.1 视觉任务动作

| 动作 | 说明 |
|---|---|
| adjust_threshold | 调整检测阈值 |
| crop_roi | 裁剪局部区域 |
| switch_detector | 切换检测模块 |
| increase_exposure | 增加曝光 |
| request_human_check | 请求人工确认 |

### 7.5.2 控制任务动作

| 动作 | 说明 |
|---|---|
| adjust_pid | 调整 PID 参数 |
| apply_control | 输出控制量 |
| reduce_gain | 降低增益 |
| emergency_stop | 紧急停止 |
| switch_mode | 切换控制模式 |

### 7.5.3 主动视觉任务动作

| 动作 | 说明 |
|---|---|
| move_camera | 移动相机 |
| rotate_view | 改变视角 |
| zoom_in | 放大观察 |
| change_focus | 改变焦距 |
| choose_next_view | 选择下一观察位姿 |

## 7.6 状态方程 StateEquation

通用状态方程为：

```text
s_{t+1} = F(s_t, a_t, θ_t, ε_t)
```

其中：

| 符号 | 说明 |
|---|---|
| s_t | 当前状态 |
| a_t | 当前动作 |
| θ_t | 当前参数 |
| ε_t | 噪声、扰动、不确定性 |
| s_{t+1} | 下一状态 |

真实的 F 往往未知。  
AuroX-General 中使用近似模型：

```text
s_hat_{t+1} = F_hat(s_t, a_t, θ_t)
```

如果无法获得下一状态，也可以学习结果模型：

```text
y_hat = G_hat(s_t, a_t, θ_t)
```

## 7.7 结果空间 ResultSpace

结果是一次执行后的直接输出。

| 任务 | 结果 |
|---|---|
| 视觉 | 检测框、掩膜、中心点、置信度、残差 |
| 控制 | 响应曲线、超调量、稳定时间、误差积分 |
| 主动视觉 | 目标位置估计、视角质量、不确定性下降 |
| 工艺调参 | 产品质量指标、良率、缺陷率 |
| 人工任务 | 验收结果、评分、修改意见 |

## 7.8 反馈空间 FeedbackSpace

反馈是对结果好坏的评价。  
反馈可以来自自动指标，也可以来自人工。

| 反馈类型 | 示例 |
|---|---|
| Numeric | 评分、误差、准确率 |
| Boolean | 成功 / 失败，接受 / 拒绝 |
| Ranking | A 比 B 好 |
| Textual | 人工备注 |
| Delayed | 多步之后才知道结果 |
| Sparse | 只有最终成功失败 |

## 7.9 终止条件 TerminationCondition

任务什么时候停止，需要明确规则。

| 条件 | 示例 |
|---|---|
| 达到目标 | loss 小于阈值 |
| 达到预算 | 迭代次数用完 |
| 安全停止 | 触发硬约束 |
| 收敛停止 | 多轮没有改善 |
| 人工停止 | 人工验收通过 |
| 不确定性停止 | 模型不确定性过高 |

---

# 8. Layer 4：Execution Runtime Layer

## 8.1 定义

Execution Runtime Layer 负责把一次任务执行标准化。

所有任务插件都必须支持类似如下生命周期：

```text
initialize
    ↓
reset
    ↓
observe
    ↓
execute
    ↓
evaluate
    ↓
record
    ↓
finish / continue
```

## 8.2 标准运行单元

一次运行包括：

```text
Run = {
  input_context,
  state_before,
  parameters,
  action,
  execution_result,
  feedback,
  loss,
  state_after,
  logs
}
```

## 8.3 核心职责

| 职责 | 说明 |
|---|---|
| 参数注入 | 将优化器建议的参数注入任务 |
| 动作执行 | 执行动作或调用外部系统 |
| 结果收集 | 收集检测结果、控制响应或任务输出 |
| 反馈接收 | 接收自动评价或人工评价 |
| 异常处理 | 捕获失败、超时、崩溃 |
| 回滚机制 | 对危险动作进行撤销或恢复 |
| 标准化输出 | 统一返回 result / feedback / loss |

## 8.4 实现方式

建议每个任务插件提供一个 Runtime Wrapper。

| Wrapper | 说明 |
|---|---|
| VisionRuntime | 执行视觉算法并返回检测指标 |
| ControlRuntime | 执行控制试验并返回响应指标 |
| ActiveVisionRuntime | 执行看-动闭环并返回视角收益 |
| SimulationRuntime | 在仿真中执行动作 |
| HumanFeedbackRuntime | 管理人工反馈与验收 |

## 8.5 状态码机制

所有 Runtime 都应返回标准状态码：

| 状态码 | 含义 |
|---|---|
| ok | 成功 |
| failed | 执行失败 |
| unsafe_action | 动作不安全 |
| invalid_parameters | 参数非法 |
| timeout | 超时 |
| low_confidence | 置信度过低 |
| insufficient_observation | 观测不足 |
| model_untrusted | 模型不可信 |
| human_required | 需要人工确认 |
| no_improvement | 无明显改善 |

AuroX-Lite 原有的状态码机制可以迁移到该层，并扩展为跨任务通用状态码。

---

# 9. Layer 5：Surrogate & Dynamics Model Layer

## 9.1 定义

该层负责从历史运行数据中学习近似模型。

它不要求反推出真实物理方程，而是学习足够有用的预测模型。

## 9.2 两类模型

### 9.2.1 结果代理模型

结果代理模型预测某组参数或动作会产生什么结果：

```text
y_hat = G_hat(s, a, θ)
```

适用于：

1. 图像参数优化；
2. 工艺调参；
3. 黑箱函数优化；
4. 人工反馈优化；
5. 无法获得完整下一状态的场景。

### 9.2.2 状态动力学模型

状态动力学模型预测下一状态：

```text
s_hat_{t+1} = F_hat(s_t, a_t, θ_t)
```

适用于：

1. 控制系统；
2. 主动视觉；
3. 机器人操作；
4. 多步决策；
5. 可连续观测状态变化的任务。

## 9.3 模型选择

| 数据条件 | 推荐模型 |
|---|---|
| 样本极少，参数维度低 | Gaussian Process |
| 参数混合类型，多噪声 | Random Forest / TPE |
| 中维黑箱优化 | Gradient Boosting / CMA-ES surrogate |
| 高维连续控制 | Neural Dynamics / Koopman / NARX |
| 稀疏符号结构 | SINDy / Sparse Regression |
| 长序列决策 | World Model / Recurrent Model |
| 不需要显式模型 | Model-free Bandit / Evolution Strategy |

## 9.4 不确定性估计

通用优化器必须知道自己“不知道什么”。

不确定性来源包括：

1. 数据少；
2. 参数区域未探索；
3. 噪声大；
4. 反馈不一致；
5. 任务分布变化；
6. 模型外推。

不确定性可用于：

| 用途 | 说明 |
|---|---|
| 探索 | 选择有信息价值的试验 |
| 安全 | 不确定性过高时拒绝 |
| 人工确认 | 高风险建议需要人审 |
| 损失惩罚 | 将不确定性加入损失 |
| 模型切换 | 当前模型不可靠时换优化器 |

## 9.5 模型输出

该层输出：

```text
ModelPrediction = {
  predicted_result,
  predicted_next_state,
  uncertainty,
  expected_loss,
  confidence,
  explanation
}
```

---

# 10. Layer 6：Loss & Feedback Layer

## 10.1 定义

该层负责把结果和反馈转化为可优化目标。

通用损失函数可以写成：

```text
L = L_task + L_cost + L_safety + L_uncertainty
```

其中：

| 部分 | 说明 |
|---|---|
| L_task | 任务本身误差 |
| L_cost | 时间、能耗、计算量、动作代价 |
| L_safety | 约束违反惩罚 |
| L_uncertainty | 模型不确定性惩罚 |

## 10.2 视觉任务损失

```text
L_vision =
  w1 * false_negative
+ w2 * false_positive
+ w3 * localization_error
+ w4 * residual_error
+ w5 * runtime
+ penalty(grid_failed)
```

对应 AuroX-Lite / AuroX-General 视觉插件：

| 指标 | 说明 |
|---|---|
| false_negative | 漏检 |
| false_positive | 误检 |
| localization_error | 定位误差 |
| residual_error | 网格残差 |
| grid_failed | 网格建立失败 |
| runtime | 运行时间 |
| human_acceptance | 人工是否接受 |

## 10.3 控制任务损失

```text
L_control =
  w1 * integral_absolute_error
+ w2 * overshoot
+ w3 * settling_time
+ w4 * oscillation
+ w5 * energy
+ penalty(saturation)
```

| 指标 | 说明 |
|---|---|
| integral_absolute_error | 误差绝对值积分 |
| overshoot | 超调 |
| settling_time | 稳定时间 |
| oscillation | 振荡 |
| energy | 能耗 |
| saturation | 执行器饱和 |

## 10.4 主动视觉损失

```text
L_active_vision =
  w1 * localization_error
+ w2 * uncertainty
+ w3 * motion_cost
+ w4 * time_cost
+ w5 * view_failure
+ penalty(collision_risk)
```

| 指标 | 说明 |
|---|---|
| localization_error | 目标定位误差 |
| uncertainty | 目标不确定性 |
| motion_cost | 运动代价 |
| time_cost | 时间代价 |
| view_failure | 视角无效 |
| collision_risk | 碰撞风险 |

## 10.5 多目标优化

很多任务不是单目标问题。  
例如视觉任务既要准确，也要快，还要稳定。

可以采用三种方式：

| 方法 | 说明 |
|---|---|
| 加权和 | 将多个指标加权成一个 loss |
| 分层目标 | 先满足安全和质量，再优化速度 |
| Pareto 优化 | 输出多组非支配解供选择 |

推荐默认策略：

```text
硬约束优先
    ↓
任务质量优先
    ↓
稳定性第二
    ↓
成本和速度第三
```

## 10.6 人工反馈转化

人工反馈可转为损失：

| 人工反馈 | 转换方式 |
|---|---|
| 接受 / 拒绝 | 接受 loss 低，拒绝 loss 高 |
| 1-5 分评分 | 归一化为 reward |
| A 比 B 好 | 偏好学习 |
| 文本意见 | 转为标签或错误类别 |
| 修改后结果 | 作为目标参考 |

---

# 11. Layer 7：Optimizer Orchestration Layer

## 11.1 定义

该层负责选择和管理优化器。

AuroX-General 不绑定单一优化算法，而是根据任务特征选择合适优化器。

## 11.2 输入

```text
OptimizerInput = {
  task_type,
  state,
  parameter_space,
  action_space,
  history,
  loss_definition,
  constraints,
  model_prediction,
  budget
}
```

## 11.3 输出

```text
OptimizerDecision = {
  next_parameters,
  next_action,
  expected_improvement,
  uncertainty,
  risk_level,
  reason,
  fallback_plan
}
```

## 11.4 优化器选择规则

| 任务特征 | 推荐优化器 |
|---|---|
| 参数少、单次运行昂贵 | Bayesian Optimization |
| 参数多、不可微 | CMA-ES |
| 参数包含离散和类别 | TPE / SMAC |
| 在线噪声大 | SPSA / Bandit |
| 需要安全探索 | Safe Bayesian Optimization |
| 可建立状态模型 | MPC |
| 多步动作策略 | Reinforcement Learning |
| 人工偏好为主 | Preference Optimization |
| 初期无数据 | Random Search / Latin Hypercube |

## 11.5 搜索策略

| 策略 | 说明 |
|---|---|
| Exploitation | 利用当前最优区域 |
| Exploration | 探索未知区域 |
| Warm Start | 使用历史最优配置初始化 |
| Trust Region | 只在安全邻域内搜索 |
| Multi-fidelity | 先低成本粗试，再高精度验证 |
| Early Stopping | 明显失败时提前停止 |
| Fallback | 优化失败时回退到保守参数 |

## 11.6 参数重要性分析

优化器应持续估计参数敏感度：

| 输出 | 作用 |
|---|---|
| important_parameters | 哪些参数最影响结果 |
| insensitive_parameters | 哪些参数可固定 |
| interaction_terms | 哪些参数存在耦合 |
| safe_ranges | 哪些参数范围稳定 |
| risk_ranges | 哪些参数范围容易失败 |

这对工程调试非常重要。

---

# 12. Layer 8：Policy & Agent Layer

## 12.1 定义

Policy & Agent Layer 负责多步决策。  
如果任务只是单次参数优化，该层可以很薄。  
如果任务涉及连续动作，例如主动视觉或机器人控制，该层就是核心。

策略定义为：

```text
a_t = π(s_t)
```

即在当前状态下选择动作。

## 12.2 与优化器的关系

| 优化器 | 策略层 |
|---|---|
| 优化当前参数 | 决定当前动作 |
| 面向单次运行 | 面向多步任务 |
| 关注下一组 θ | 关注下一步 a |
| 常用于黑箱调参 | 常用于具身智能 |

二者可以结合：

```text
π(s_t) 选择动作
Optimizer 选择该动作下的参数
Runtime 执行任务
Feedback 更新策略和优化器
```

## 12.3 策略类型

| 策略类型 | 示例 |
|---|---|
| Rule-based Policy | 基于规则选择下一视角 |
| Greedy Policy | 选择预计收益最高动作 |
| Uncertainty-driven Policy | 选择最能降低不确定性的动作 |
| MPC Policy | 在短期预测窗口内优化动作序列 |
| RL Policy | 通过奖励学习长期策略 |
| Human-in-the-loop Policy | 高风险动作交给人工确认 |

## 12.4 主动视觉策略示例

主动视觉中的策略目标是：

```text
在最小运动代价下，最大化目标识别置信度并降低不确定性。
```

策略输入：

| 输入 | 说明 |
|---|---|
| 当前相机位姿 | 当前从哪里看 |
| 当前图像 | 当前看到什么 |
| 检测置信度 | 是否已经看清目标 |
| 不确定性地图 | 哪些区域不确定 |
| 遮挡地图 | 哪些区域被遮挡 |
| 运动成本 | 下一步移动代价 |

策略输出：

| 输出 | 说明 |
|---|---|
| 下一视角 | 去哪里看 |
| 相机参数 | 曝光、焦距、放大倍率 |
| 检测参数 | 置信度阈值、ROI 大小 |
| 是否停止 | 是否已经足够确定 |

---

# 13. Layer 9：Application & Domain Plugins

## 13.1 定义

Domain Plugins 是具体任务实现层。  
每个任务插件只需要遵守统一接口，就可以接入 AuroX-General。

## 13.2 插件组成

每个插件包含：

```text
Plugin = {
  task_schema,
  state_extractor,
  parameter_space,
  action_space,
  runtime,
  evaluator,
  loss_builder,
  safety_rules,
  visualization,
  report_generator
}
```

## 13.3 视觉插件

### 目标

从图像中检测目标、定位目标、提取结构或判断缺陷。

### 示例任务

| 任务 | 说明 |
|---|---|
| WaferGridDetection | 晶圆晶粒检测与网格推理 |
| ObjectDetection | 通用目标识别 |
| DefectLocalization | 缺陷定位 |
| ROIInitialization | 自动 ROI 初始化 |

### 可复用 AuroX-Lite 能力

| AuroX-Lite 能力 | 在 AuroX-General 中的位置 |
|---|---|
| 图像预处理 | Vision State Extractor |
| 几何筛选 | Vision Runtime |
| 鲁棒统计 | State & Parameter Adapter |
| 网格推理 | Domain Model |
| 状态码 | Runtime Status |
| 动态参数 | Parameter Optimization Target |

## 13.4 控制插件

### 目标

优化控制系统参数或动作，使系统响应更稳定、更快、更安全。

### 示例任务

| 任务 | 说明 |
|---|---|
| PIDTuning | PID 自整定 |
| ServoControl | 伺服控制 |
| TemperatureControl | 温度控制 |
| MotionStabilization | 运动稳定 |

### 核心状态

| 状态 | 说明 |
|---|---|
| error | 当前误差 |
| error_history | 误差历史 |
| response_curve | 响应曲线 |
| actuator_state | 执行器状态 |
| disturbance | 外部扰动 |

## 13.5 主动视觉插件

### 目标

通过动作主动改善视觉观测。

### 示例任务

| 任务 | 说明 |
|---|---|
| NextBestView | 下一最佳视角 |
| VisualServoing | 视觉伺服 |
| ActiveObjectSearch | 主动目标搜索 |
| InspectionPlanning | 主动检测路径规划 |

### 核心闭环

```text
当前视角
    ↓
视觉识别
    ↓
不确定性估计
    ↓
选择下一视角
    ↓
执行移动
    ↓
重新观察
```

## 13.6 人工反馈插件

### 目标

接入人工评价，使优化器能够学习人类偏好或验收标准。

### 示例反馈

| 反馈 | 用途 |
|---|---|
| 接受 / 拒绝 | 构造二值 reward |
| 评分 | 构造连续 reward |
| 排序 | 构造偏好模型 |
| 标注修正 | 构造监督信号 |
| 文字意见 | 分类错误原因 |

---

# 14. 统一数据流

## 14.1 单轮优化数据流

```text
Input / Observation
    ↓
State Extractor
    ↓
Safety Check
    ↓
Optimizer Decision
    ↓
Runtime Execute
    ↓
Result
    ↓
Feedback
    ↓
Loss Builder
    ↓
Memory Update
    ↓
Model / Optimizer Update
```

## 14.2 多轮闭环数据流

```text
Episode 1
    ↓
Episode 2
    ↓
Episode 3
    ↓
...
    ↓
历史经验库
    ↓
代理模型更新
    ↓
优化器 warm-start
    ↓
新任务更快收敛
```

---

# 15. 三类任务统一映射

## 15.1 视觉识别任务

| 抽象项 | 视觉任务含义 |
|---|---|
| State | 图像质量、候选区域、尺度、边缘、置信度 |
| Parameter | 阈值、核大小、置信度下限、ROI 尺寸 |
| Action | 调整参数、裁剪 ROI、切换检测器、重拍 |
| Result | 检测框、掩膜、中心点、类别、置信度 |
| Feedback | IoU、人工接受、误检漏检、下游任务成功 |
| Loss | 检测误差 + 误检 + 漏检 + 时间成本 |
| Optimizer | BO / TPE / CMA-ES |
| Policy | 选择是否重拍、是否放大、是否请求人工 |

## 15.2 PID 控制任务

| 抽象项 | PID 任务含义 |
|---|---|
| State | 目标值、当前值、误差、误差积分、误差微分 |
| Parameter | Kp、Ki、Kd、滤波系数、限幅参数 |
| Action | 调整 PID、输出控制量、切换模式 |
| Result | 响应曲线、超调、稳定时间、稳态误差 |
| Feedback | 是否稳定、是否超调、人工验收 |
| Loss | 误差积分 + 超调 + 稳定时间 + 能耗 |
| Optimizer | BO / SPSA / Safe BO / MPC |
| Policy | 在线选择控制量或调整参数 |

## 15.3 主动视觉任务

| 抽象项 | 主动视觉含义 |
|---|---|
| State | 相机位姿、机器人位姿、图像、不确定性、遮挡 |
| Parameter | 检测阈值、视角评分权重、曝光、焦距 |
| Action | 移动相机、调整焦距、选择下一视角 |
| Result | 目标定位、置信度、不确定性下降 |
| Feedback | 是否找到目标、定位误差、任务成功 |
| Loss | 定位误差 + 不确定性 + 运动代价 + 时间 |
| Optimizer | MPC / RL / Bandit / BO |
| Policy | 下一最佳视角策略 |

---

# 16. 从 AuroX-Lite 到 AuroX-General 的演进路线

## 16.1 阶段一：AuroX-Lite 参数闭环优化

### 目标

将 AuroX-Lite 从单次自适应扩展为跨运行参数优化。

### 工作内容

1. 标准化 AuroX-Lite 输入输出；
2. 将关键参数注册到 ParameterSpace；
3. 将运行结果转为 ResultSpace；
4. 将状态码、残差、检测数量、人工反馈转为 Loss；
5. 引入黑箱优化器；
6. 建立 episode 记录；
7. 对不同图像自动 warm-start。

### 优先优化参数

| 参数类别 | 示例 |
|---|---|
| 尺度参数 | adaptiveMinAreaRatio、adaptiveMinRectSideRatio |
| 形态学参数 | morphRatio、openIterations、closeIterations |
| 几何筛选参数 | minFillRatio、minHullCompactness |
| 网格参数 | maxSnapResidualRatio、minGridInlierRatio |
| 证据过滤参数 | frameScale、minMeanEdgeScore |
| 补偿参数 | compensationMin4NeighborSupport |

### 阶段验收标准

| 指标 | 目标 |
|---|---|
| 人工调参次数 | 明显减少 |
| 多图像平均 loss | 下降 |
| 失败状态码占比 | 下降 |
| 最优参数复用率 | 提升 |
| 运行时间 | 可控 |

---

## 16.2 阶段二：统一任务抽象框架

### 目标

使视觉、控制、主动视觉任务都可以接入统一接口。

### 工作内容

1. 定义 Task Schema；
2. 定义 StateSpace / ParameterSpace / ActionSpace；
3. 定义 Runtime Wrapper；
4. 定义 Loss Builder；
5. 定义 Safety Constraint；
6. 定义 Memory 格式；
7. 定义 Optimizer 接口；
8. 实现任务插件注册机制。

### 阶段验收标准

| 指标 | 目标 |
|---|---|
| 新任务接入成本 | 降低 |
| 数据格式 | 统一 |
| 优化器复用 | 可跨任务 |
| 损失函数组合 | 可配置 |
| 安全约束 | 可统一检查 |

---

## 16.3 阶段三：代理模型与经验迁移

### 目标

让系统能够从历史任务中学习经验，并迁移到新任务。

### 工作内容

1. 训练结果代理模型；
2. 估计参数重要性；
3. 建立相似任务检索；
4. 支持 warm-start；
5. 支持不确定性估计；
6. 建立失败案例库；
7. 建立最佳配置库。

### 阶段验收标准

| 指标 | 目标 |
|---|---|
| 新任务收敛轮数 | 减少 |
| 初始参数质量 | 提升 |
| 不确定性校准 | 更准确 |
| 失败案例复用 | 可解释 |
| 跨图像迁移 | 有效果 |

---

## 16.4 阶段四：控制任务接入

### 目标

支持 PID 自整定和简单控制系统优化。

### 工作内容

1. 定义控制状态变量；
2. 定义 PID 参数空间；
3. 定义响应指标；
4. 定义控制损失；
5. 引入安全探索；
6. 支持仿真先行；
7. 支持真实系统小步更新。

### 阶段验收标准

| 指标 | 目标 |
|---|---|
| 超调量 | 降低 |
| 稳定时间 | 缩短 |
| 稳态误差 | 降低 |
| 振荡 | 减少 |
| 安全违规 | 为零或可控 |

---

## 16.5 阶段五：主动视觉闭环

### 目标

支持“看见—判断—移动—再看”的具身智能闭环。

### 工作内容

1. 定义相机和机器人状态；
2. 定义下一视角动作空间；
3. 定义视角收益函数；
4. 定义不确定性地图；
5. 接入视觉插件；
6. 接入运动控制插件；
7. 引入策略层；
8. 支持仿真训练与真实验证。

### 阶段验收标准

| 指标 | 目标 |
|---|---|
| 目标发现率 | 提升 |
| 定位误差 | 降低 |
| 视角数量 | 减少 |
| 运动成本 | 降低 |
| 碰撞风险 | 受控 |
| 任务成功率 | 提升 |

---

# 17. 推荐模块划分

## 17.1 核心模块

| 模块 | 职责 |
|---|---|
| TaskRegistry | 注册任务插件 |
| StateExtractor | 从观测中提取状态 |
| ParameterManager | 管理参数空间 |
| ActionManager | 管理动作空间 |
| SafetyManager | 约束检查 |
| RuntimeManager | 执行任务 |
| FeedbackManager | 接收反馈 |
| LossBuilder | 构造损失 |
| MemoryManager | 记录经验 |
| ModelManager | 训练代理模型 |
| OptimizerManager | 调度优化器 |
| PolicyManager | 管理策略 |
| ReportManager | 生成报告 |

## 17.2 模块交互

```text
TaskRegistry
    ↓
StateExtractor
    ↓
SafetyManager
    ↓
OptimizerManager / PolicyManager
    ↓
RuntimeManager
    ↓
FeedbackManager
    ↓
LossBuilder
    ↓
MemoryManager
    ↓
ModelManager
    ↓
OptimizerManager 更新
```

---

# 18. 技术难点与风险

## 18.1 状态变量设计难

如果状态变量缺失关键因素，优化器无法学到真实规律。

### 应对策略

1. 尽量记录原始观测；
2. 同时记录统计特征；
3. 保留人工标注和失败截图；
4. 定期分析参数重要性；
5. 对不同任务设计专用 state extractor。

## 18.2 反馈稀疏或不稳定

只有成功 / 失败反馈时，优化效率较低。

### 应对策略

1. 增加中间指标；
2. 引入人工评分；
3. 引入弱监督信号；
4. 使用偏好学习；
5. 使用多轮累计 reward。

## 18.3 多参数耦合

多个参数共同影响结果，单独调参容易误判。

### 应对策略

1. 使用联合参数搜索；
2. 做参数敏感度分析；
3. 记录参数交互项；
4. 对强耦合参数进行分组优化；
5. 采用 trust region 防止大幅跳变。

## 18.4 优化器误探索

优化器可能尝试危险或无意义参数。

### 应对策略

1. 安全层前置；
2. 低风险仿真优先；
3. 参数边界保守；
4. 高不确定性时人工确认；
5. 支持回滚和早停。

## 18.5 迁移失败

历史任务与新任务看似相似，但实际不同，导致 warm-start 失败。

### 应对策略

1. 相似度加入任务内容特征；
2. 不直接信任历史最优参数；
3. warm-start 后仍保留探索；
4. 记录迁移成功率；
5. 检测分布漂移。

---

# 19. 与 AuroX-Lite 的关系

## 19.1 保留能力

AuroX-General 不应抛弃 AuroX-Lite 的优势，而应将其插件化。

| AuroX-Lite 能力 | AuroX-General 处理方式 |
|---|---|
| 图像处理流程 | 作为 Vision Plugin |
| 鲁棒统计 | 作为 State Extractor |
| 动态参数 | 纳入 ParameterSpace |
| 网格推理 | 作为 Domain Model |
| 状态码 | 纳入 Runtime Status |
| 安全拒绝 | 纳入 Safety Layer |
| 可解释调试 | 纳入 Report Layer |

## 19.2 升级方向

| AuroX-Lite | AuroX-General |
|---|---|
| 单任务 | 多任务 |
| 单次运行自适应 | 跨运行闭环学习 |
| 规则动态参数 | 优化器主动搜索参数 |
| 图像输入 | 多模态观测 |
| 几何反馈 | 通用反馈 |
| 网格任务 | 视觉、控制、主动视觉 |
| 手工调试 | 自动报告和参数建议 |
| 无长期记忆 | 有经验库和迁移能力 |

---

# 20. 推荐最小可行产品 MVP

## 20.1 MVP 目标

先不要直接做完整具身智能系统。  
建议第一个 MVP 聚焦：

```text
AuroX-Lite 视觉参数自动优化闭环
```

## 20.2 MVP 范围

### 输入

1. 图像；
2. 初始参数；
3. 可选人工标注；
4. 可选人工验收。

### 输出

1. 最优参数；
2. 检测结果；
3. loss 曲线；
4. 参数敏感度；
5. 失败原因；
6. 推荐调参说明。

### 闭环

```text
图像
    ↓
AuroX-Lite 运行
    ↓
结果评价
    ↓
计算 loss
    ↓
优化器生成新参数
    ↓
再次运行
    ↓
得到最优配置
```

## 20.3 MVP 优先指标

| 指标 | 说明 |
|---|---|
| grid_success_rate | 网格成功率 |
| detection_accuracy | 检测准确率 |
| false_positive_rate | 误检率 |
| false_negative_rate | 漏检率 |
| mean_residual | 平均网格残差 |
| runtime | 运行时间 |
| human_acceptance | 人工接受率 |

---

# 21. 结论

AuroX-General 的本质应定义为：

```text
一个面向多任务的通用闭环优化框架。
它将视觉、控制、主动视觉等任务统一抽象为：
状态、观测、参数、动作、结果、反馈、损失、约束和优化策略。
它不追求从有限数据中唯一反推出真实状态方程，
而是通过历史运行数据学习可用的代理模型或近似动力学模型，
在安全边界内主动搜索更优参数或动作，
并通过持续反馈形成跨运行、跨任务、可迁移的优化能力。
```

从工程路线看，建议按以下顺序推进：

```text
1. AuroX-Lite 参数闭环优化
2. 统一任务抽象接口
3. 经验库与代理模型
4. 控制任务接入
5. 主动视觉闭环
6. 多任务策略与具身智能扩展
```

AuroX-General 不应该是一个“万能算法”，而应该是一个“统一问题表达 + 多优化器调度 + 安全闭环反馈”的系统框架。  
只要统一抽象层设计稳定，后续视觉、控制、主动视觉、机器人操作和工艺调参都可以以插件形式逐步接入。
