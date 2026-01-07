function [agents, params] = scn_fsm_cross_right_FSM(params)
% FSM 场景 3：右交叉（CROSS_RIGHT），OS 为 give-way 船
% 目标：测试在右交叉局面下，TS 中途改向时 FSM 的 TC-1 分支。

agents = [];

%% Agent 1 (VO) - OS，沿 y 轴向北
a = Agent(1,[-6 -14],[-6 14],0.3,0.7,'VO',[0 0]);
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100];
agents = [agents, a];

%% Agent 3 (CONST) - TS，从右侧横穿过来，并在相遇附近向下偏转
% 起点在 OS 右前方，初始基本水平左行，接近交汇点后向南转一点。
a = Agent(3,[18 3],[-12 3],0.3,1.2,'CONST',[-1.2 0.0]);
a.safetyRadius = 1.0;
a.color = [0.550 0.550 0.550];

a.setWaypoints( ...
    [ 18.0   3.0;   % 起点：右侧
       6.0   3.0;   % 进入会遇区
       0.0  0.0;   % 向下偏转（改变航向，触发 dCTS>deg5）
     -12.0  0.0], ...
    'none', 1.2, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.05;
params.sim.maxSteps    = 3000;

end
