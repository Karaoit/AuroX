// ============================================================================
// ActionSpace 最小全功能示例
// ----------------------------------------------------------------------------
// 目的：给「后续优化器 / 编排器 / 前端」提供一份可执行的接口契约参考。
// 用法：本文件仅作示例，CMake 只 glob src/*.cpp，因此不会被编进 AuroX 库。
//       想跑起来：把本文件复制成 apps/ 下的一个 main.cpp 并链接 AuroX 即可。
//
// 覆盖清单：
//   1. 与已重构的参数空间对接（PStruct + PARAM_REG/PARAM_BIND）
//   2. 默认动作（参数更新）+ 更新源（优化器解耦）
//   3. 子类化自定义动作（ACT_TYPE 一行生成 type()/clone()）
//   4. ACT_REG / ACT_ADD 注册块
//   5. lambda 动作（addFn，无需子类化）
//   6. 数据型占位动作（CustomAction：未实现的状态空间/转移方程）
//   7. 条件钩子 shouldRun / 开关 enabled -> Skipped / Disabled
//   8. applyAll 全量执行 + ActOutcome 逐动作回执
//   9. applyOne 按名字 / 按 id 单独执行
//  10. 自定义类型的 JSON 往返（registerActionType）
//  11. 序列化 / 反序列化（含 lambda 函数体回填）
//  12. 删除与复位
// ============================================================================

#include "AuroX/Space/ActionSpace.hpp"

#include <algorithm>
#include <iostream>

using namespace AuroX::Space;   // 也可写 PSpace:: 前缀

// ----------------------------------------------------------------------------
// 1) 参数空间：复用已重构的 PStruct
// ----------------------------------------------------------------------------
struct PIDParams : public PStruct<PIDParams> {
    Param<double> Kp;
    Param<double> Ki;
    EnumParam mode;

    AUROX_PARAMS(Kp, Ki, mode)

    PIDParams() {
        PARAM_REG(
            PARAM_BIND(Kp, 1.0, 0.0, 10.0);
            PARAM_BIND(Ki, 0.1, 0.0,  5.0);
            PARAM_BIND(mode, "auto", {"auto", "manual"});
        );
    }
};

// ----------------------------------------------------------------------------
// 2) 自定义动作：子类化 + ACT_TYPE
//    ACT_TYPE("clamp") 一行生成 type() 与 clone()（必须在最终派生类里使用）。
//    注意 apply() 是**非 const** 的 —— 真实动作往往要维护状态（这里是 round）。
// ----------------------------------------------------------------------------
class ClampAction : public ActBase {
public:
    ACT_TYPE("clamp")

    int maxRounds = 3;
    int round = 0;

    // 只在前 maxRounds 轮参与；此后自动 -> Skipped
    bool shouldRun(const ParameterSpace&) const override { return round < maxRounds; }

    ActionResult apply(ParameterSpace& space,
                       const std::vector<double>& /*gradient*/,
                       const UpdateFn& /*updateSource*/) override {
        auto v = space.vec();                       // vec() 是 vectorizeDouble() 的别名
        for (auto& x : v) x = std::clamp(x, 0.0, 5.0);
        space.unvec(v);                             // unvec() 是 unvectorizeDouble() 的别名
        ++round;
        return ActionResult::Ok;
    }

    json config() const override { return json{{"maxRounds", maxRounds}}; }
    void setConfig(const json& j) override {
        if (j.contains("maxRounds")) maxRounds = j["maxRounds"].get<int>();
    }
};

int main() {
    PIDParams params;
    ASpace actions;                       // 默认集合：只含 param_update（id = 1）

    // ------------------------------------------------------------------
    // 3) 接线优化器：更新源
    //    真实场景由 OptimizerSession 把 Optimizer::computeUpdate 接进来；
    //    ActionSpace 永远不认识 Optimizer 类型。此处用 lambda 模拟。
    // ------------------------------------------------------------------
    actions.setUpdateSource([](ParameterSpace& /*space*/,
                               const std::vector<double>& g) {
        std::vector<double> delta(g.size());
        for (std::size_t i = 0; i < g.size(); ++i) delta[i] = -0.05 * g[i];
        return delta;                     // 返回的是「增量」，不是梯度
    });

    // ------------------------------------------------------------------
    // 4) ACT_REG / ACT_ADD 注册块（对齐 PARAM_REG / PARAM_BIND 的手感）
    // ------------------------------------------------------------------
    ACT_REG(actions,
        auto* clamp = ACT_ADD(ClampAction);
        clamp->maxRounds = 2;
    );

    // ------------------------------------------------------------------
    // 5) lambda 动作：不想子类化时的最短路径
    // ------------------------------------------------------------------
    actions.addFn("snapshot",
                  [](ParameterSpace& space, const std::vector<double>&, const UpdateFn&) {
                      std::cout << "  [snapshot] dim = " << space.vec().size() << "\n";
                      return ActionResult::Ok;
                  });

    // ------------------------------------------------------------------
    // 6) 数据型占位动作：承载「尚未实现」的状态空间 / 转移方程
    //    paramDelta 留空 = 纯占位、无副作用；前端可直接渲染其 JSON。
    // ------------------------------------------------------------------
    actions.add(std::make_unique<CustomAction>(
        "move_camera",
        /*params=*/     json{{"target", {0.0, 1.0, 0.0}}, {"speed", 0.5}},
        /*state=*/      json{{"space", "camera_pose"}, {"dim", 3}},
        /*transition=*/ json{{"equation", "x_{k+1} = f(x_k, u_k)"}}
    ));

    // ------------------------------------------------------------------
    // 7) 注册自定义类型 -> 让它也能 JSON 往返
    // ------------------------------------------------------------------
    registerActionType("clamp", [] { return std::make_unique<ClampAction>(); });

    // ------------------------------------------------------------------
    // 8) 跑几轮：applyAll + ActOutcome 回执
    // ------------------------------------------------------------------
    const std::vector<double> grad = {0.1, -0.2, 0.0};
    for (int iter = 0; iter < 3; ++iter) {
        std::cout << "iter " << iter << ":\n";
        std::vector<ActOutcome> log;
        ActionResult r = actions.applyAll(params.space(), grad, &log);
        for (const auto& o : log)
            std::cout << "  " << o.name << " -> " << toString(o.result) << "\n";
        std::cout << "  total = " << toString(r) << "\n";
    }
    // 第 3 轮起 clamp 的 shouldRun() 为 false -> Skipped

    // ------------------------------------------------------------------
    // 9) 单独执行：按名字 / 按 id
    // ------------------------------------------------------------------
    actions.applyOne("clamp", params.space(), grad);
    actions.applyOne(actions.defaultActionId(), params.space(), grad);

    // ------------------------------------------------------------------
    // 10) 开关：禁用后该动作恒为 Disabled，且 applyAll 跳过它
    // ------------------------------------------------------------------
    if (auto* cam = actions.get("move_camera")) cam->setEnabled(false);

    // ------------------------------------------------------------------
    // 11) 序列化往返
    // ------------------------------------------------------------------
    json j = actions.toJson();
    ASpace restored;
    if (!restored.tryFromJson(j)) std::cout << "restore failed\n";

    // lambda 的函数体不可序列化，反序列化后需回填
    if (auto* fn = dynamic_cast<FnAction*>(restored.get("snapshot"))) {
        fn->setFn([](ParameterSpace&, const std::vector<double>&, const UpdateFn&) {
            return ActionResult::Ok;
        });
    }

    // ------------------------------------------------------------------
    // 12) 删除与复位
    // ------------------------------------------------------------------
    actions.remove("move_camera");   // 按名字删
    actions.removeById(2);           // 按 id 删
    actions.clear();                 // 复位为「仅默认参数更新」

    std::cout << "final size = " << actions.size() << "\n";
    return 0;
}
