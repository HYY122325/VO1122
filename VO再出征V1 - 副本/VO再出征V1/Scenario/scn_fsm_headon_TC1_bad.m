function [agents, params] = scn_fsm_headon_TC1_bad(params)
% FSM 场景 2：正对会遇，TS 向其左舷（东向）转向
% 目标：同样触发 give-way 船的 TC-1，但 TS 的改变“不利”，
%      对应 C-2 / action-2（即 OS 需要采取更强的右转行动）。

agents = [];

%% Agent 1 (VO) - OS
a = Agent(1,[0 -14],[-1 14],0.3,1.0,'VO',[0 0]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 3 (CONST) - TS：先正对 OS，随后向东偏转（对 OS 来说更危险）
a = Agent(3,[0 18],[ 3 -4 ],0.3,1.0,'CONST',[0 -1.5]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];

a.setWaypoints( ...
    [ -3   18.0;   % 起点：正对
      0   10.0;   % 继续冲近
      3    2.0;   % 向东偏（对 OS 变成左转，不利改变）
      4  -4.0], ...
    'none', 1.5, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.05;
params.sim.maxSteps    = 800;

end
