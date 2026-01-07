function [agents, params] = scn_fsm_cross_left_insufficientG(params)
% FSM 场景 4：左交叉，TS 避让幅度不足
% 目标：TS 采取了行动（转了），但没转够。测试 TC-3 (Assistance Action)。

agents = [];

%% Agent 1 (VO) - OS，向北
% Stand-on
a = Agent(1,[0 -10],[0 10],0.3,0.5,'VO',[0 0.5]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 2 (CONST) - TS，从左侧过来，微弱避让
% 它的路径稍微向右修正了一点点，试图从 OS 船尾过，但计算上仍然会撞（或者 DCPA 很小）
a = Agent(2,[-10 2],[10 -2],0.3,0.6,'CONST',[0.6 0]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];
a.setWaypoints( ...
    [-10.0  2.0;
     -4.0   2.0;   % 接近
     -1.0   1.5;   % 【关键动作】微弱右转（向南），试图避让
      10.0 -3.0], ...
    'none', 0.6, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.1;
params.sim.maxSteps    = 600;
end