function [agents, params] = scn_mixed_colregs_8(params)
% FSM 场景：COLREGs 综合大考 (Mixed Rules)
% 场地：25x25
% 目标：一次性测试 VO/RVO 处理 追越、对遇、交叉 的能力。
% 布局：
%   - Agent 2 (前): 慢速同向，制造追越需求。
%   - Agent 3 (对): 逆向而来，制造对遇局面。
%   - Agent 4 (右): 右侧横切，制造让路 (Give-way) 局面。
%   - Agent 5 (左): 左侧横切，制造直航 (Stand-on) 局面。

agents = [];
rng(888); % 固定种子

%% 1. 全局参数
params.world.fieldSize = 12.5; % 半径12.5 -> 25x25
params.sim.maxSteps    = 1200; 

%% 2. Agent 1 (VO) - OS 本船
% 路线：左上 -> 右下 (SE方向)
start_os = [-15, 15];
goal_os  = [20, -20];
os_speed = 0.6; 

a = Agent(1, start_os, goal_os, os_speed, 0.8, 'VO', [0.4, -0.4]); 
a.safetyRadius = 0.8;
a.color = [0.000 0.600 0.100]; % 绿色 OS
agents = [agents, a];

%% 3. 剧本船只 (Scripted Agents)

% --- Scenario A: 追越 (Overtaking) ---
% Agent 2: 在 OS 前方 6米处，同向，但速度只有 0.3 (OS是0.6)
% OS 必须决定从左侧还是右侧超车
start_2 = [-8, 8]; % 位于 OS 路径前方
goal_2  = [12, -12]; % 终点相同
spd_2   = 0.3;     % 慢速
a = Agent(2, start_2, goal_2, spd_2, 0.8, 'CONST', [0.2, -0.2]);
a.setWaypoints([start_2; goal_2], 'none', 1.0, spd_2);
a.color = [0.929 0.694 0.125]; % 橙色 (慢船)
agents = [agents, a];

% --- Scenario B: 对遇 (Head-on) ---
% Agent 3: 从右下角出发，逆向开往左上角
% 它会在 OS 超越慢船的过程中或之后，与 OS 正面相遇
start_3 = [11, -11];
goal_3  = [-11, 11];
spd_3   = 0.6;
a = Agent(3, start_3, goal_3, spd_3, 0.8, 'CONST', [-0.4, 0.4]);
a.setWaypoints([start_3; goal_3], 'none', 1.0, spd_3);
a.color = [0.850 0.325 0.098]; % 红色 (对向威胁)
agents = [agents, a];

% --- Scenario C: 交叉-让路 (Crossing Give-way) ---
% Agent 4: 从左下(-10, -10) 开往 右上(10, 10)
% 它的路径与 OS 垂直交叉。
% 相对于 OS (向东南开)，Agent 4 位于 OS 的 右舷 (Starboard)。
% 根据 COLREGs，OS 应该让路 (通常是右转，从 Agent 4 船尾通过)。
start_4 = [-10, -10];
goal_4  = [10, 10];
spd_4   = 0.55;
a = Agent(4, start_4, goal_4, spd_4, 0.8, 'CONST', [0.4, 0.4]);
a.setWaypoints([start_4; goal_4], 'none', 1.0, spd_4);
a.color = [0.635 0.078 0.184]; % 深红 (右侧交叉-高危)
agents = [agents, a];

% --- Scenario D: 交叉-直航 (Crossing Stand-on) ---
% Agent 5: 从右上(10, 10) 开往 左下(-10, -10)
% 相对于 OS，它位于 左舷 (Port)。OS 有优先权，但在拥挤环境下仍需小心。
start_5 = [10, 10];
goal_5  = [-10, -10];
spd_5   = 0.5;
a = Agent(5, start_5, goal_5, spd_5, 0.8, 'CONST', [-0.4, -0.4]);
a.setWaypoints([start_5; goal_5], 'none', 1.0, spd_5);
a.color = [0.494 0.184 0.556]; % 紫色 (左侧交叉)
agents = [agents, a];

%% 4. 填充船只 (Fillers)
% 生成 3 个额外的随机船，增加背景噪音，限制 OS 的大范围机动
existing_pos = [start_os; start_2; start_3; start_4; start_5];

for i = 1:3
    id = 5 + i;
    valid = false;
    while ~valid
        % 在边缘生成
        pos = (rand(1,2)-0.5)*20;
        if norm(pos - start_os) > 5 && min(vecnorm(existing_pos - pos, 2, 2)) > 3.0
            valid = true;
            start_pos = pos;
            % 目标为对面
            goal_pos = -start_pos + (rand(1,2)-0.5)*5;
            spd = 0.3 + rand*0.3;
            
            vel = (goal_pos - start_pos)/norm(goal_pos-start_pos)*spd;
            a = Agent(id, start_pos, goal_pos, spd, 0.8, 'CONST', vel);
            a.setWaypoints([start_pos; goal_pos], 'none', 1.0, spd);
            a.color = [0.5 0.5 0.5]; % 灰色 (背景船)
            agents = [agents, a];
            existing_pos = [existing_pos; start_pos];
        end
    end
end

end