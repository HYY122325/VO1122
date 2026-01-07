function [agents, params] = scn_fsm_headon_TC1_good(params)
% FSM 场景 1：正对会遇，TS 主动协同右转
% 目标：触发 give-way 船的 TC-1（Observation / Prepare for action），
%      且 TS 的改变是“有利”的，对应 C-1 / action-1。

agents = [];

%% Agent 1 (VO) - 本船 OS，向北行驶
a = Agent(1,[0 -14],[0 14],0.3,1.0,'VO',[0 0]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 3 (CONST) - 目标船 TS，先正对而来，之后向其右舷（西向）转向
% 初始大致正对 OS（HEAD_ON），之后在靠近时向西偏转，模拟“积极避让”
a = Agent(3,[0 18],[ -3 -4 ],0.3,1.5,'CONST',[0 -1.5]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];

% 路径：直冲 -> 略靠近 -> 向西侧大转（右转，朝其右舷）
a.setWaypoints( ...
    [ 0   18.0;   % 起点
      0    6.0;   % 正对接近
     -1   2.0;   % 向西偏转
    -3   -4.0], ... % 远离
    'none', 1.5, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.05;
params.sim.maxSteps    = 3000;

end
