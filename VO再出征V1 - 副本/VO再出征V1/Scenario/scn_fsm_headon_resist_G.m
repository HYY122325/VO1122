function [agents, params] = scn_fsm_headon_resist_G(params)
% FSM 场景 1：对遇（HEAD_ON），TS 采取阻碍行动
% 目标：OS 右转避让时，TS 突然左转（抢占 OS 的避让空间），触发 TC-1。

agents = [];

%% Agent 1 (VO) - OS，从下往上 (0, -10) -> (0, 10)
% 初始向北
a = Agent(1,[0 -10],[0 10],0.3,0.7,'VO',[0 0.7]); 
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100]; % 绿色 OS
agents = [agents, a];

%% Agent 2 (CONST) - TS，从上往下，中途向它的左侧（即画面右侧）偏转
% 正常对遇应各自右转（画面左侧）。
% 这里让 TS 向画面右侧（x正向）转，阻碍 OS 的右转路径。
a = Agent(2,[0 10],[0 -10],0.3,0.7,'CONST',[0 -0.7]);
a.safetyRadius = 1.0;
a.color = [0.850 0.325 0.098]; % 红色 TS
a.setWaypoints( ...
    [ 0.0  10.0;    % 起点
      0.0   4.0;    % 接近 OS
      7.5  2.0;    % 【关键动作】突然向画面右侧偏转（TS 的左转），阻碍 OS
      7.0  -10.0], ...
    'none', 0.7, 0.3);
agents = [agents, a];

%% 仿真参数
params.world.fieldSize = 20.0;
params.sim.dt          = 0.1;
params.sim.maxSteps    = 400;
end