function [agents, params] = scn_busy_channel_8(params)
% FSM 场景：拥挤航道 (The Busy Channel)
% 场地：25x25 (50m x 50m)
% 船数：8 (1 OS + 7 TS)
% 剧情：
%   OS 在正中央向北航行。
%   前方有一艘慢船 (TS-2) 挡路 -> 触发追越。
%   左右两侧各有一列船队 (TS-3, TS-4, TS-5) 南下 -> 形成“双向单车道”的压迫感。
%   外围有干扰船。

agents = [];
rng(101); % 固定种子

%% 1. 全局参数
params.world.fieldSize = 25; 
params.sim.maxSteps    = 1000; 

%% 2. Agent 1 (OS) - 本船
% 从南向北
start_os = [0, -20];
goal_os  = [0, 20];
os_speed = 0.8; 

a = Agent(1, start_os, goal_os, os_speed, 1.0, 'VO', [0, 0.8]); 
a.safetyRadius = 1.0;
a.color = [0.000 0.600 0.100]; % 绿色
agents = [agents, a];

%% 3. 核心剧本船

% --- [前车] Agent 2: 挡路慢船 (追越对象) ---
start_2 = [0, -5]; % 在 OS 前方 15米
goal_2  = [0, 26];
spd_2   = 0.3;     % 龟速
a = Agent(2, start_2, goal_2, spd_2, 0.5, 'CONST', [0, 0.3]);
a.setWaypoints([start_2; goal_2], 'none', 1.0, spd_2);
a.color = [0.9 0.7 0.1]; % 黄色
agents = [agents, a];

% --- [左侧对向] Agent 3 & 4: 逼迫 OS 向右的对遇船 ---
% 它们在 x=-4 的轨道上南下，限制 OS 左转空间
for i = 1:2
    id = 2 + i;
    start_pos = [-4, 20 + (i-1)*10]; % 分散开
    goal_pos  = [-4, -20];
    spd       = 0.6;
    a = Agent(id, start_pos, goal_pos, spd, 0.8, 'CONST', [0, -0.6]);
    a.setWaypoints([start_pos; goal_pos], 'none', 1.0, spd);
    a.color = [0.8 0.2 0.2]; % 红色 (危险)
    agents = [agents, a];
end

% --- [右侧对向] Agent 5: 封锁右路的对遇船 ---
% 在 x=4 的轨道上南下，迫使 OS 必须精确穿插
id = 5;
start_pos = [4, 18];
goal_pos  = [4, -20];
spd       = 0.6;
a = Agent(id, start_pos, goal_pos, spd, 0.8, 'CONST', [0, -0.6]);
a.setWaypoints([start_pos; goal_pos], 'none', 1.0, spd);
a.color = [0.8 0.2 0.2]; 
agents = [agents, a];

%% 4. 干扰船 (Crossing)
% --- Agent 6 & 7: 横穿航道的捣乱者 ---
start_6 = [-20, 5]; goal_6 = [20, 5]; % 左到右
a = Agent(6, start_6, goal_6, 0.5, 0.8, 'CONST', [0.5, 0]);
a.setWaypoints([start_6; goal_6], 'none', 1.0, 0.5);
agents = [agents, a];

start_7 = [20, -10]; goal_7 = [-20, -10]; % 右到左
a = Agent(7, start_7, goal_7, 0.5, 0.8, 'CONST', [-0.5, 0]);
a.setWaypoints([start_7; goal_7], 'none', 1.0, 0.5);
agents = [agents, a];

% --- Agent 8: 凑数背景 ---
a = Agent(8, [-15, -15], [-5, -5], 0.2, 0.5, 'CONST', [0.1, 0.1]);
agents = [agents, a];

end