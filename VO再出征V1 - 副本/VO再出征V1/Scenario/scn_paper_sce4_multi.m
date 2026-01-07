function [agents, params] = scn_paper_sce4_multi(params)
% Scenario 4 in paper: Multi-ship encounter
% OS: Agent 1 (VO)
% TS1: 右交叉   （从东向西）
% TS2: 左交叉   （从西向东）
% TS3: 正对遇   （自北向南）

agents = [];

% ---- OS: VO 船 ----
a = Agent(1, [-6.0 -14.0], [-6.0 14.0], 0.3, 1.2, 'VO', [0.0 0.0]);
a.safetyRadius = 1.0;
a.color = [0.050 0.490 0.193];
agents = [agents, a];

% ---- TS1：右交叉（东→西，略在 OS 前方）----
a = Agent(3, [18.0 4.0], [-18.0 4.0], 0.3, 1.0, 'CONST', [-1.0 0.0]);
a.safetyRadius = 1.0;
a.color = [0.55 0.55 0.55];
a.setWaypoints([18.0 4.0; -18.0 4.0], 'none', 1.0, 0.3);
agents = [agents, a];

% ---- TS2：左交叉（西→东，略在 OS 后方）----
a = Agent(4, [-18.0 -3.0], [18.0 -3.0], 0.3, 1.0, 'CONST', [1.0 0.0]);
a.safetyRadius = 1.0;
a.color = [0.55 0.55 0.55];
a.setWaypoints([-18.0 -3.0; 18.0 -3.0], 'none', 1.0, 0.3);
agents = [agents, a];

% ---- TS3：正对遇（北→南）----
a = Agent(5, [-2.0 18.0], [-2.0 -18.0], 0.3, 1.2, 'CONST', [0.0 -1.2]);
a.safetyRadius = 1.0;
a.color = [0.55 0.55 0.55];
a.setWaypoints([-2.0 18.0; -2.0 -18.0], 'none', 1.2, 0.3);
agents = [agents, a];

% ---- 仿真参数 ----
params.world.fieldSize = 20.0;
params.sim.dt         = 0.05;
params.sim.maxSteps   = 1600;

end
