function [agents, params] = scn_paper_sce3_overtaking(params)
% Scenario 3 in paper: OVERTAKING, OS takes action
% OS: Agent 1 (VO)，在后方稍快
% TS: Agent 3 (CONST)，在前方同航向较慢

agents = [];

% ---- OS: VO 船（后方较快）----
a = Agent(1, [-6.0 -14.0], [-6.0 14.0], 0.3, 1.6, 'VO', [0.0 0.0]);
a.safetyRadius = 1.0;
a.color = [0.050 0.490 0.193];
agents = [agents, a];

% ---- TS: CONST 船（前方较慢，被追越）----
% 前方同航向，从 y = -4 向北
a = Agent(3, [-6.0 -7.0], [-6.0 8.0], 0.3, 0.8, 'CONST', [0.0 0.8]);
a.safetyRadius = 1.0;
a.color = [0.55 0.55 0.55];
a.setWaypoints([-6.0 -7.0; -6.0 8.0], 'none', 0.8, 0.3);
agents = [agents, a];

% ---- 仿真参数 ----
params.world.fieldSize = 20.0;
params.sim.dt         = 0.05;
params.sim.maxSteps   = 1400;

end
