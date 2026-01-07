function [agents, params] = scn_crossing_hell_8(params)
% FSM 场景：死亡十字路口 (The Intersection)
% 场地：25x25
% 船数：8
% 剧情：八卦阵布局，所有船向中心/对角线冲锋。测试多船协同与避让优先级。

agents = [];
rng(202); 

%% 1. 全局参数
params.world.fieldSize = 25; 
params.sim.dt          = 0.1;
params.sim.maxSteps    = 2000; 

%% 2. Agent 1 (OS) - 本船
% 从左下往右上 (西南 -> 东北)
start_os = [-18, -18];
goal_os  = [18, 18];
os_speed = 0.8; 

a = Agent(1, start_os, goal_os, os_speed, 1.0, 'VO', [0.5, 0.5]); 
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100]; 
agents = [agents, a];

%% 3. 关键交互船只

% --- Agent 2: 右舷交叉 (Give-way 考验) ---
% 从右下往左上，位于 OS 右侧，OS 必须让路（通常右转从船尾过）
start_2 = [18, -18];
goal_2  = [-18, 18];
spd_2   = 0.7;
vel_2   = (goal_2 - start_2) / norm(goal_2 - start_2) * spd_2;
a = Agent(2, start_2, goal_2, spd_2, 1.0, 'CONST', vel_2);
a.setWaypoints([start_2; goal_2], 'none', 1.0, spd_2);
a.color = [0.9 0.1 0.1]; % 红色警报
agents = [agents, a];

% --- Agent 3: 正面对遇 (Head-on) ---
% 从右上往左下，直冲 OS
start_3 = [18, 18];
goal_3  = [-18, -18];
spd_3   = 0.7;
vel_3   = (goal_3 - start_3) / norm(goal_3 - start_3) * spd_3;
a = Agent(3, start_3, goal_3, spd_3, 1.0, 'CONST', vel_3);
a.setWaypoints([start_3; goal_3], 'none', 1.0, spd_3);
a.color = [0.8 0.4 0.0]; % 橙色
agents = [agents, a];

% --- Agent 4: 左舷交叉 (Stand-on 考验) ---
% 从左上往右下，OS 是直航船，看 Agent 4 是否让路（仿真里 CONST 不会让，迫使 OS 触发 Action-5）
start_4 = [-18, 18];
goal_4  = [18, -18];
spd_4   = 0.6;
vel_4   = (goal_4 - start_4) / norm(goal_4 - start_4) * spd_4;
a = Agent(4, start_4, goal_4, spd_4, 1.0, 'CONST', vel_4);
a.setWaypoints([start_4; goal_4], 'none', 1.0, spd_4);
a.color = [0.2 0.2 0.8]; % 蓝色 (理论上它该让我)
agents = [agents, a];

%% 4. 外围包围网 (增加计算压力)
% 在东西南北四个正方向生成船只，往圆心走
directions = [0, 1; 0, -1; 1, 0; -1, 0]; % 北南东西
starts     = [0, -20; 0, 20; -20, 0; 20, 0];
goals      = [0, 20; 0, -20; 20, 0; -20, 0];

for i = 1:4
    id = 4 + i;
    sp = starts(i,:);
    gp = goals(i,:);
    spd = 0.5;
    vel = (gp - sp)/norm(gp-sp)*spd;
    
    a = Agent(id, sp, gp, spd, 0.8, 'CONST', vel);
    a.setWaypoints([sp; gp], 'none', 1.0, spd);
    a.color = [0.5 0.5 0.5];
    agents = [agents, a];
end

end