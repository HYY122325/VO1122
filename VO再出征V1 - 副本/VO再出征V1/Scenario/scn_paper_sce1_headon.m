function [agents, params] = scn_paper_sce1_headon(params)
% Scenario 1 in paper: HEAD_ON, OS takes action
% OS: Agent 1 (VO)，自南向北
% TS: Agent 3 (CONST)，自北向南

agents = [];

% ---- OS: VO 船 ----
a = Agent(1, [-6.0 -14.0], [-6.0 14.0], 0.3, 1.5, 'VO', [0.0 0.0]);
a.safetyRadius = 1.0;
a.color = [0.050 0.490 0.193];   % 绿色
agents = [agents, a];

% ---- TS: CONST 船（对遇）----
a = Agent(3, [-6.0 14.0], [-6.0 -14.0], 0.3, 1.5, 'CONST', [0.0 -1.5]);
a.safetyRadius = 1.0;
a.color = [0.55 0.55 0.55];
a.setWaypoints([-6.0 14.0; -6.0 -14.0], 'none', 1.5, 0.3);
agents = [agents, a];

% ---- 仿真参数 ----
params.world.fieldSize = 20.0;
params.sim.dt         = 0.05;
params.sim.maxSteps   = 1000;    % 距离约 28m，1.5 m/s，给足时间

end
