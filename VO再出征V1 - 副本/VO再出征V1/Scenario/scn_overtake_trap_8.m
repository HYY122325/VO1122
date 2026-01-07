function [agents, params] = scn_overtake_trap_8(params)
% FSM 场景：连环陷阱 (The Overtake Trap)
% 场地：25x25
% 船数：8
% 剧情：OS 试图超车，但变道后立即遭遇右侧突袭和正面封锁。

agents = [];
rng(303); 

%% 1. 全局参数
params.world.fieldSize = 25; 
params.sim.maxSteps    = 1200; 

%% 2. Agent 1 (OS) - 本船
% 从西向东
start_os = [-22, 0];
goal_os  = [22, 0];
os_speed = 0.9; % 速度较快，有超车欲望

a = Agent(1, start_os, goal_os, os_speed, 1.0, 'VO', [0.9, 0]); 
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100]; 
agents = [agents, a];

%% 3. 陷阱设计

% --- 陷阱 1: 诱饵慢船 (Overtaking) ---
% 在 OS 正前方，速度极慢，诱导 OS 变道
start_2 = [-10, 0];
goal_2  = [20, 0];
spd_2   = 0.3;
a = Agent(2, start_2, goal_2, spd_2, 0.5, 'CONST', [0.3, 0]);
a.setWaypoints([start_2; goal_2], 'none', 1.0, spd_2);
a.color = [0.9 0.8 0.2]; % 黄色诱饵
agents = [agents, a];

% --- 陷阱 2: 右侧伏击 (Crossing Give-way) ---
% 在 OS 变道路径（通常往右变道）上等待
% 从南向北切入，位于 OS 右舷
start_3 = [0, -15];
goal_3  = [0, 15];
spd_3   = 0.7;
a = Agent(3, start_3, goal_3, spd_3, 0.8, 'CONST', [0, 0.7]);
a.setWaypoints([start_3; goal_3], 'none', 1.0, spd_3);
a.color = [0.8 0.1 0.1]; % 红色伏击
agents = [agents, a];

% --- 陷阱 3: 左道封锁 (Head-on) ---
% 如果 OS 选择向左变道超车，会迎面撞上 Agent 4
start_4 = [15, 2]; % 稍微偏北一点（左道）
goal_4  = [-15, 2];
spd_4   = 0.6;
a = Agent(4, start_4, goal_4, spd_4, 0.8, 'CONST', [-0.6, 0]);
a.setWaypoints([start_4; goal_4], 'none', 1.0, spd_4);
a.color = [0.8 0.1 0.1]; % 红色封锁
agents = [agents, a];

%% 4. 混乱制造者
% 在后半程放置一些斜向运动的船，增加计算复杂度
for i = 1:4
    id = 4 + i;
    % 随机生成在右半场
    sp = [10 + rand*10, (rand-0.5)*30];
    gp = [-10, (rand-0.5)*30];
    spd = 0.4 + rand*0.3;
    vel = (gp - sp)/norm(gp-sp)*spd;
    
    a = Agent(id, sp, gp, spd, 0.8, 'CONST', vel);
    a.setWaypoints([sp; gp], 'none', 1.0, spd);
    a.color = [0.6 0.6 0.7]; 
    agents = [agents, a];
end

end