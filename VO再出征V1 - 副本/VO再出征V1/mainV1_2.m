%% ========================================
% 多智能体避碰仿真主程序 V1.1 (final) - 增强版
%
% - 从 SimConfigV1_2 统一读取参数
% - 场景来源：UI 场景构建器 / 已保存 .m 场景
% - 新增：仿真结束后的轨迹总览可视化
% - 关键点：在仿真前，用配置类中的避碰静态参数
%          统一下发到所有 Agent 实例：
%          neighborDist / timeHorizon / safetyRadius
%          确保和 LVO / Step5 等算法数值一致
%% ========================================

clear; close all; clc;
addpath(genpath('.'));


%% ------- 1. 参数设置：从配置类读取 -------

if exist('+config/SimConfigV1_2.m','file')n
    % 包形式：+config/SimConfigV1_2.m
    params = config.SimConfigV1_2.default();
elseif exist('SimConfigV1_2.m','file')
    % 普通 classdef 文件
    params = SimConfigV1_2.default();
else
    error('找不到 SimConfigV1_2.m 配置文件');
end

% 新增：轨迹图像保存选项
params.viz.saveTrajectoryImage = false;  % 设为true可自动保存轨迹图像


%% ------- 2. 场景选择（UI / 场景脚本） -------

fprintf('\n=== 场景加载选择 ===\n');
fprintf('请选择场景来源：\n');
fprintf('1. 使用 UI 场景构建器（交互式创建）\n');
fprintf('2. 加载已保存的场景文件（.m 文件）\n');

choice = input('请输入选项 (1 或 2): ', 's');

if strcmp(choice, '2')
    % === 2.1 选择场景脚本 ===
    [file, path] = uigetfile('*.m', '选择场景文件', 'Scenario/');
    if isequal(file, 0)
        fprintf('未选择文件，退出。\n');
        return;
    end

    fullPath = fullfile(path, file);
    fprintf('加载场景文件: %s\n', fullPath);

    % 从文件名获取函数名
    [~, funcName, ~] = fileparts(file);

    % 添加路径并调用场景函数：约定 [agents, params] = funcName(params)
    addpath(path);
    try
        [agents, params] = feval(funcName, params);
        fprintf('成功加载场景，共 %d 个 agents\n', numel(agents));
    catch ME
        fprintf('加载场景文件失败: %s\n', ME.message);
        return;
    end

else
    % === 2.2 使用 UI 场景构建器 ===
    params.sim.source = 'ui';

    fprintf('\n=== 启动场景构建器 V1.1 版本 ===\n');
    [agents, params] = ScenarioBuilderV1_1(params);

    if isempty(agents)
        fprintf('用户取消或无 agents，退出。\n');
        return;
    end
end


%% ------- 3. 从配置类下发避碰静态参数到 Agent -------
% 统一调用 SimConfigV1_2.applyAgentCollisionConfig
% 避碰相关静态参数唯一真值源：params.agent

if exist('+config/SimConfigV1_2.m','file') ...
        && exist('config.SimConfigV1_2','class')
    agents = config.SimConfigV1_2.applyAgentCollisionConfig(params, agents);
elseif exist('SimConfigV1_2.m','file') ...
        && exist('SimConfigV1_2','class')
    agents = SimConfigV1_2.applyAgentCollisionConfig(params, agents);
else
    warning('未找到 SimConfigV1_2.applyAgentCollisionConfig，使用场景脚本中的 Agent 参数。');
end


%% ------- 4. 打印场景信息（此时已是"下发后"的参数） -------

fprintf('\n=== 场景信息 ===\n');
fprintf('共有 %d 个 agents\n', numel(agents));

for i = 1:numel(agents)
    fprintf('Agent %d: Mode=%s', agents(i).id, agents(i).mode);

    if strcmpi(agents(i).mode, 'CONST')
        if ~isempty(agents(i).waypoints)
            fprintf(', Waypoints=%d 个, Mode=%s', ...
                size(agents(i).waypoints,1), agents(i).wpMode);
        end
    end

    % 打印避碰参数，方便对齐检查
    if isprop(agents(i),'neighborDist') && isprop(agents(i),'timeHorizon') && isprop(agents(i),'safetyRadius')
        fprintf(', neighborDist=%.2f, timeHorizon=%.2f, safetyRadius=%.2f', ...
            agents(i).neighborDist, agents(i).timeHorizon, agents(i).safetyRadius);
    end

    fprintf('\n');
end


%% ------- 5. 可视化初始化 -------

figure('Position', [100 100 1000 900]);
hold on; grid on; axis equal;

xlim(params.world.xlim);
ylim(params.world.ylim);

title(sprintf('%s - 多智能体避碰仿真 (V1.1 final)', params.algorithm.name));
xlabel('X'); ylabel('Y');

hold off;


%% ------- 6. 轨迹记录初始化 -------

trajectories = cell(1, numel(agents));
for i = 1:numel(agents)
    trajectories{i} = agents(i).position;
end


%% ------- 7. 仿真循环 -------

fprintf('\n>>> 开始仿真...\n');

actualSteps = 0;  % 记录实际运行的步数

for step = 1:params.sim.maxSteps

    actualSteps = step;  % 更新实际步数
    % ---- 把当前步号 / 仿真时间写入 params，供避碰算法调试使用 ----
    params.sim.step = step;                                 %调试
    params.sim.time = step * params.sim.dt;                 %调试
    % --- 7.1 检查 VO 智能体是否全部到达 ---
    allReached = true;
    for i = 1:numel(agents)
        if strcmpi(agents(i).mode, 'VO')
            if ~agents(i).hasReachedGoal()
                allReached = false;
                break;
            end
        end
    end

    if allReached
        fprintf('所有 VO 智能体已到达目标！步数: %d, 时间: %.2fs\n', ...
            step, step * params.sim.dt);
        break;
    end


    % --- 7.2 邻居搜索（基于 agent.neighborDist） ---
    for i = 1:numel(agents)
        agents(i).findNeighbors(agents);
    end


    % --- 7.3 速度规划 ---
    for i = 1:numel(agents)

        % 已到达：直接置 0 速
        if agents(i).hasReachedGoal()
            agents(i).velocity = [0, 0];
            continue;
        end

        % 先算期望速度：
        %   - CONST：按 waypoints / constVel
        %   - VO   ：朝向 goal 的最大速度方向
        agents(i).computePreferredVelocity();

        if strcmpi(agents(i).mode,'CONST')
            % CONST 模式：直接用期望速度（已考虑 waypoint），再做一次限幅
            v = agents(i).prefVelocity;
            s = norm(v);
            vmax = agents(i).maxSpeed;

            if s > vmax && s > 1e-6
                v = v * (vmax / s);
            end

            agents(i).velocity = v;

        else
            % VO 模式：统一算法入口（名称大小写/下划线/短横线都不敏感）
            % computeVelocity 内部根据 params.algorithm.name 分发到
            % VO / VO_Step3B / VO_Step5B 等版本
            agents(i).velocity = algorithms.computeVelocity( ...
                agents(i), agents(i).neighbors, params);
        end
    end


    % --- 7.4 位置更新 + 轨迹记录 ---
    for i = 1:numel(agents)
        agents(i).updatePosition(params.sim.dt);
        trajectories{i} = [trajectories{i}; agents(i).position];
    end


    % --- 7.5 可视化更新 ---
    if mod(step, params.viz.updateInterval) == 0
        cla; hold on;

        plotAgentsV1_1(agents, trajectories, ...
            params.viz.showVelocity, ...
            params.viz.showGoals, ...
            params.viz.showSafetyCircle, ...
            params.viz.showSpeedValue, ...
            params.agent.safetyRadius, ...
            params);

        xlim(params.world.xlim);
        ylim(params.world.ylim);
        grid on; axis equal;

        title(sprintf('%s | step=%d | t=%.2fs', ...
            params.algorithm.name, step, step * params.sim.dt));
        xlabel('X'); ylabel('Y');

        hold off;
        drawnow;
        pause(params.viz.pauseTime);
    end


    % --- 7.6 进度输出 ---
    if mod(step, 100) == 0
        fprintf(' 进度: %d / %d (t=%.2fs)\n', ...
            step, params.sim.maxSteps, step * params.sim.dt);
    end
end

fprintf('>>> 仿真完成。\n\n');


%% ------- 8. 统计信息 -------

fprintf('=== 仿真统计 ===\n');
for i = 1:numel(agents)
    fprintf('Agent %d (%s): ', agents(i).id, agents(i).mode);

    if strcmpi(agents(i).mode,'CONST') && ~isempty(agents(i).waypoints)
        fprintf('完成了 waypoint 路径巡航');
        if strcmpi(agents(i).wpMode,'loop')
            fprintf('（循环模式）');
        elseif strcmpi(agents(i).wpMode,'pingpong')
            fprintf('（往返模式）');
        end
    else
        dist = norm(agents(i).position - agents(i).goal);
        fprintf('距目标 %.2f 米', dist);
    end

    fprintf('\n');
end


%% ------- 9. 轨迹后处理可视化（新增） -------

fprintf('\n=== 生成轨迹总览图 ===\n');

% 检查函数是否存在
if ~exist('plotFinalTrajectories', 'file')
    warning('未找到 plotFinalTrajectories 函数，请确保该文件在路径中。');
    fprintf('跳过轨迹后处理可视化。\n');
else
    % 调用新的轨迹可视化函数
    plotFinalTrajectories(agents, trajectories, params, actualSteps);
    fprintf('轨迹可视化完成。\n');
end

% 询问用户是否保存轨迹数据
saveChoice = input('\n是否保存轨迹数据到文件？(y/n): ', 's');
if strcmpi(saveChoice, 'y')
    filename = sprintf('trajectory_data_%s_%s.mat', ...
        params.algorithm.name, datestr(now, 'yyyymmdd_HHMMSS'));
    save(filename, 'agents', 'trajectories', 'params', 'actualSteps');
    fprintf('轨迹数据已保存到: %s\n', filename);
end

fprintf('\n>>> 全部任务完成。\n');