function [agents, params] = scn_case_beovertaken(params)
% 被追越场景：
%   Agent 1: VO，本船，被后方快船追越（stand-on）
%   Agent 2: CONST，后方更快的追越船

agents = [];

% Agent 1 (VO) - 被追越直航船
a = Agent(1, [-13.000000 0.000000], [8.000000 0.000000], ...
          0.300000, 1.200000, 'VO', [0.000000 0.000000]);
a.timeHorizon = 20.000000;
a.neighborDist = 30.000000;
a.safetyRadius = 1.000000;
a.color = [0.050 0.490 0.193];
agents = [agents, a];

% Agent 2 (CONST) - 后方更快的追越船
a = Agent(2, [-20.000000 -1.000000], [20.000000 -1.000000], ...
          0.300000, 1.800000, 'CONST', [1.800000 0.000000]);
a.safetyRadius = 1.000000;
a.color = [0.550 0.550 0.550];
a.setWaypoints([-20.000000 -1.000000; 20.000000 -1.000000], ...
               'none', 1.800000, 0.300000);
agents = [agents, a];

% 仿真区域 / 时间步长
params.world.fieldSize = 20.000000;
params.sim.dt          = 0.050000;
params.sim.maxSteps    = 1200;

% 其他避碰静态参数仍建议在 SimConfigV1_2.default() 中统一配置

end
