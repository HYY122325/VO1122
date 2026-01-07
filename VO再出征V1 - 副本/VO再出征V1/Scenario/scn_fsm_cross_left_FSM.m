function [agents, params] = scn_fsm_cross_left_FSM(params)
% FSM 场景 4：左交叉（CROSS_LEFT），OS 为 stand-on 船
% 目标：测试直航船在 CROSS_LEFT 下的 C-3/C-4 逻辑以及 TC-2。

agents = [];

%% Agent 1 (VO) - OS，向北
a = Agent(1,[-6 -14],[-6 14],0.3,1.4,'VO',[0 0]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 3 (CONST) - TS，从左侧横穿
% 先水平右行一段，进入 dp 区后向上偏一点，模拟“给路但又不太规矩”的动作。
a = Agent(3,[-18 0],[18 0],0.3,1.0,'CONST',[1.2 0.0]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];

a.setWaypoints( ...
    [ -18.0   0.0;  % 起点：左侧
       -4.0   0.0;  % 接近会遇区，航向未变（dCTS 小）
        4.0   2.5;  % 进入 dp 附近后抬头偏转（触发 dCTS>deg5）
       18.0   2.5], ...
    'none', 1.2, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.05;
params.sim.maxSteps    = 3000;

end
