vo5L Modularization Pack (drop into +algorithms)
===============================================

本包提供“避碰 Step5L（模块化版）”代码。

设计目标
--------
- ✅ 主函数接口保持不变：
      newVelocity = computeVO_Step5L(agent, neighbors, params)
- ✅ 保持原有主干逻辑不变：
      硬安全 > (FSM/COLREG 硬规则) > 缓冲软代价 > Past&Clear > Seamanship > Fallback
- ✅ 只做按主干功能模块化（高内聚、低耦合），不做过度稀碎化拆分。

推荐放置位置（与你现有工程一致）
--------------------------------
你现有避碰版本都放在：
    +algorithms/

本包也按这个落盘：
    +algorithms/computeVO_Step5L.m
    +algorithms/+vo5L/...
    +algorithms/vo5L_modular/...

其中 +vo5L 是子 package：外部全名为 algorithms.vo5L.*

如何集成
--------
1) 把本包里的这三项复制到你工程的 +algorithms/ 目录下（与 computeVO_Step5A/B/C... 平级）：
   - computeVO_Step5L.m
   - +vo5L/
   - vo5L_modular/   (可选：只放文档/回归工具，不影响运行)

2) 确保 MATLAB path 加的是 +algorithms 的“父目录”，而不是 +algorithms 目录本身。
   （你原工程能跑，说明这一步已经满足。）

3) 在 +algorithms 内部调用时，你可以继续像以前一样直接写：
      v = computeVO_Step5L(agent, neighbors, params);
   在包外部调用时，需要明确包前缀：
      v = algorithms.computeVO_Step5L(agent, neighbors, params);

为什么没有 Config.m
--------------------
你工程里已有 SimConfigV1_2 等配置脚本。
为了避免“Config.m”这种泛名造成混淆，本包把原先的 Config 模块改名为：
    +algorithms/+vo5L/VOContext.m
它只负责把 params/agent/neighbors 解析成 ctx（上下文），不与仿真配置文件抢名字。

模块组成（与原 S0~S4 对齐）
--------------------------
- computeVO_Step5L.m
    主入口：保留 debug 节流与 Action-5 的细节调试门控；其余逻辑下放模块。

- +vo5L/VOContext.m
    S0：参数解析/派生量/邻船排序/基线信息/各种开关与阈值。

- +vo5L/State.m
    Sx：持久状态读写（原 S5STATE 迁移）。支持：
      algorithms.vo5L.State('get', agentId)
      algorithms.vo5L.State('set', agentId, st)
      algorithms.vo5L.State('reset')

- +vo5L/Frontend.m
    S1：CCO/BCO 扫描 + GCCO/GBCO 扩展 + 静止目标标记。

- +vo5L/RiskScene.m
    S2：CR 计算 + Past&Clear + sr-lock（episode-lock）+ COLREGS mask。

- +vo5L/FSM.m
    S3：FSM 更新 + override mask 覆盖（调试输出保持）。

- +vo5L/Planner.m
    S4：联合优化(C+s) + Past&Clear 回航向 + Seamanship 平滑 + fallback。

- +vo5L/Utils.m
    通用几何/数学/3DoF 扩展等工具，集中放在一个文件里避免拆得太碎。

回归验证（推荐）
----------------
本包附带：vo5L_compare_step.m（在 vo5L_modular/ 里）。
使用方式：
1) 复制你旧版的 computeVO_Step5K.m 为 computeVO_Step5L_ref.m（仅用于对照，不改你原文件）
2) 在仿真循环里调用：
      [v_new, v_ref, dv] = vo5L_compare_step(agent, neighbors, params);
   其中 dv = norm(v_new - v_ref)

注意：vo5L_modular/ 目录默认不在 MATLAB path 上；你做回归时可以临时 addpath。
