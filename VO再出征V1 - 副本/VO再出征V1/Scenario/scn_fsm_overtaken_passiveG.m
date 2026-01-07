function [agents, params] = scn_fsm_overtaken_passiveG(params)
% FSM 场景 3：被追越（OVERTAKEN），TS 追越不避让
% 目标：测试 OS 被追越时的 TC-2 逻辑及 Action-4 (不得向追越船方向转向)。

agents = [];

%% Agent 1 (VO) - OS，慢速向北
% 速度 0.4
a = Agent(1,[0 -5],[0 15],0.3,0.4,'VO',[0 0.4]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 2 (CONST) - TS，从正后方高速追上来
% 速度 0.8 (比 OS 快)，从右后方切入，有碰撞风险
a = Agent(2,[1 -15],[0 15],0.3,0.8,'CONST',[0 0.8]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];
a.setWaypoints( ...
    [ 1.0 -15.0;
      0.0   0.0;   % 撞击点：瞄准 OS 的船身
      0.0  15.0], ...
    'none', 0.8, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.1;
params.sim.maxSteps    = 500;
end