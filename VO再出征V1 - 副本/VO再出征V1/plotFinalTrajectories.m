function plotFinalTrajectories(agents, trajectories, params, totalSteps)
% plotFinalTrajectories - 仿真结束后的轨迹总览可视化（支持 FSM 历史）
% 功能：
%   - 显示所有智能体的完整运行轨迹
%   - 在轨迹上标记定时间隔的位置点
%   - 显示起始点、终点和路径方向
%   - 按 FSM 状态着色：Normal / Observe / Action
%
% 输入：
%   agents       - 智能体数组
%   trajectories - 轨迹数据：
%                  1) 新版：结构体数组，字段：
%                       .position       [maxSteps x 2]
%                       .fsm_state_code [maxSteps x 1] 0/1/2
%                  2) 旧版：cell 数组，每个 cell 是 [N x 2]（无 FSM）
%   params       - 仿真参数
%   totalSteps   - 实际运行的总步数（main 里传 actualSteps 进来）

% ========== 参数设置 ==========
% 确保 params.viz 存在
if ~isfield(params, 'viz') || ~isstruct(params.viz)
    params.viz = struct();
end

% 从配置中读取参数，如果没有则使用默认值
if isfield(params.viz, 'trajectoryTimeInterval') && ~isempty(params.viz.trajectoryTimeInterval)
    timeMarkInterval = params.viz.trajectoryTimeInterval;
else
    timeMarkInterval = 1.0;  % 默认时间标记间隔（秒）
end

if isfield(params.viz, 'trajectoryMarkerSizes') && ~isempty(params.viz.trajectoryMarkerSizes)
    markerSizes = params.viz.trajectoryMarkerSizes;
else
    markerSizes = [8, 6, 4];  % 默认不同时间段的标记大小
end

dt = params.sim.dt;

% 若 totalSteps 没传或为 0，则从轨迹里估一个
if nargin < 4 || isempty(totalSteps) || totalSteps <= 0
    totalSteps = 0;
    if isstruct(trajectories)
        for i = 1:numel(trajectories)
            if ~isempty(trajectories(i).position)
                totalSteps = max(totalSteps, size(trajectories(i).position, 1));
            end
        end
    elseif iscell(trajectories)
        for i = 1:numel(trajectories)
            if i <= numel(trajectories) && ~isempty(trajectories{i})
                totalSteps = max(totalSteps, size(trajectories{i}, 1));
            end
        end
    end
end

stepMarkInterval = max(1, round(timeMarkInterval / dt));  % 对应的步数间隔

% FSM 可视化颜色定义
colorObserve = [1, 0.8, 0];   % 黄色 (Observe)
colorAction  = [1, 0.2, 0.2]; % 红色 (Action)

% ========== 创建新图窗 ==========
figure('Name', '仿真轨迹总览', 'Position', [150 150 1200 900]);
hold on; grid on; axis equal;

% 设置坐标范围
xlim(params.world.xlim);
ylim(params.world.ylim);

% ========== 绘制网格背景 ==========
ax = gca;
ax.GridLineStyle = '--';
ax.GridAlpha = 0.3;

try
    ax.MinorGrid = 'on';
    ax.MinorGridLineStyle = ':';
    ax.MinorGridAlpha = 0.15;
catch
    grid minor;
end

% ========== 绘制每个智能体的轨迹 ==========
for i = 1:numel(agents)
    % ---------- 1. 从 trajectories 里取出轨迹 & FSM 状态 ----------
    if isstruct(trajectories)
        % 新版：结构体数组
        if i > numel(trajectories) || isempty(trajectories(i).position)
            continue;
        end
        
        posAll = trajectories(i).position;
        maxN = min(size(posAll, 1), totalSteps);
        if maxN < 2
            continue;
        end
        posAll = posAll(1:maxN, :);
        
        % 去掉前后 NaN（预分配用的）
        validRows = ~any(isnan(posAll), 2);
        if ~any(validRows)
            continue;
        end
        firstIdx = find(validRows, 1, 'first');
        lastIdx  = find(validRows, 1, 'last');
        idxRange = firstIdx:lastIdx;
        
        traj = posAll(idxRange, :);
        nPoints = size(traj, 1);
        if nPoints < 2
            continue;
        end
        
        % FSM 状态（0=Normal, 1=Observe, 2=Action）
        if isfield(trajectories(i), 'fsm_state_code') && ~isempty(trajectories(i).fsm_state_code)
            fsmAll = trajectories(i).fsm_state_code;
            if numel(fsmAll) < lastIdx
                fsmAll(end+1:lastIdx, 1) = 0; %#ok<AGROW>
            end
            fsm_states = fsmAll(idxRange);
        else
            fsm_states = zeros(nPoints, 1, 'uint8');
        end
        
        globalStepIdx = idxRange(:);  % 每个局部点对应的全局 step 索引
        
    else
        % 旧版：cell 数组
        if i > numel(trajectories) || isempty(trajectories{i})
            continue;
        end
        
        posAll = trajectories{i};
        maxN = min(size(posAll, 1), totalSteps);
        if maxN < 2
            continue;
        end
        
        traj = posAll(1:maxN, :);
        nPoints = size(traj, 1);
        fsm_states = zeros(nPoints, 1, 'uint8');  % 没有 FSM 信息，全 0
        globalStepIdx = (1:nPoints).';           % 全局 step 就是局部索引
    end
    
    % ---------- 2. 绘制轨迹线（集成 FSM 颜色 + 渐变） ----------
    agentColor = agents(i).color;
    
    for k = 1:nPoints-1
        statusCode = fsm_states(k);
        
        if statusCode == 1
            plotColor = colorObserve;
            lineWidth = 3.0; % Observe 阶段加粗
        elseif statusCode == 2
            plotColor = colorAction;
            lineWidth = 3.0; % Action 阶段加粗
        else
            plotColor = agentColor;
            lineWidth = 2.0;
        end
        
        % 渐变效果：往白色过渡，模拟“前淡后深”
        alpha = 0.3 + 0.7 * (k / nPoints); % 0.3 ~ 1.0
        segColor = (1 - alpha) * [1 1 1] + alpha * plotColor;
        
        plot([traj(k,1), traj(k+1,1)], [traj(k,2), traj(k+1,2)], '-', ...
            'Color', segColor, 'LineWidth', lineWidth);
    end
    
    % ---------- 3. 标记起始位置（大圆圈） ----------
    plot(traj(1,1), traj(1,2), 'o', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', agentColor, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 2);
    
    text(traj(1,1) + 0.5, traj(1,2) + 0.5, ...
        sprintf('A%d Start', agents(i).id), ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', agentColor * 0.8);
    
    % ---------- 4. 标记终止位置（大方块） ----------
    plot(traj(end,1), traj(end,2), 's', ...
        'MarkerSize', 12, ...
        'MarkerFaceColor', agentColor * 0.8, ...
        'MarkerEdgeColor', 'k', ...
        'LineWidth', 2);
    
    text(traj(end,1) + 0.5, traj(end,2) - 0.5, ...
        sprintf('A%d End', agents(i).id), ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'Color', agentColor * 0.8);
    
    % ---------- 5. 标记时间间隔位置 ----------
    for idxLocal = 1:stepMarkInterval:nPoints
        gStep = globalStepIdx(idxLocal);         % 对应的全局 step
        currentTime = (gStep - 1) * dt;          % 仿真时间
        
        % 标记大小按时间段
        totalTime = totalSteps * dt;
        if currentTime < totalTime * 0.33
            markerSize = markerSizes(1);
        elseif currentTime < totalTime * 0.67
            markerSize = markerSizes(2);
        else
            markerSize = markerSizes(3);
        end
        
        plot(traj(idxLocal,1), traj(idxLocal,2), 'o', ...
            'MarkerSize', markerSize, ...
            'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', agentColor, ...
            'LineWidth', 1.5);
        
        % 每隔几个时间点添加时间标签（近似判断）
        if currentTime > 0 && mod(round(currentTime, 3), 3*timeMarkInterval) == 0
            text(traj(idxLocal,1), traj(idxLocal,2) - 0.3, ...
                sprintf('t=%.1fs', currentTime), ...
                'FontSize', 10, 'Color', [0.3 0.3 0.3], ...
                'HorizontalAlignment', 'center');
        end
    end
    
    % ---------- 6. 绘制运动方向箭头 ----------
    arrowSteps = round(linspace(round(nPoints*0.2), round(nPoints*0.8), 3));
    for k = arrowSteps
        if k < nPoints && k > 1
            dx = traj(k+1,1) - traj(k,1);
            dy = traj(k+1,2) - traj(k,2);
            if norm([dx, dy]) > 1e-6
                quiver(traj(k,1), traj(k,2), dx, dy, 0.5, ...
                    'Color', agentColor, 'LineWidth', 2, ...
                    'MaxHeadSize', 0.8, 'AutoScale', 'off');
            end
        end
    end
    
    % ---------- 7. CONST 模式：绘制 waypoints 路径 ----------
    if isfield(agents(i), 'mode') && strcmpi(agents(i).mode, 'CONST') && ...
            isfield(agents(i), 'waypoints') && ~isempty(agents(i).waypoints)
        wp = agents(i).waypoints;
        pathColor = agentColor * 0.5 + [0.5 0.5 0.5];
        
        if isfield(agents(i), 'wpMode') && strcmpi(agents(i).wpMode, 'loop')
            wpLoop = [wp; wp(1,:)];
            plot(wpLoop(:,1), wpLoop(:,2), ':', ...
                'Color', pathColor, 'LineWidth', 1.5);
        else
            plot(wp(:,1), wp(:,2), ':', ...
                'Color', pathColor, 'LineWidth', 1.5);
        end
        
        plot(wp(:,1), wp(:,2), 'd', ...
            'MarkerSize', 5, ...
            'MarkerFaceColor', pathColor, ...
            'MarkerEdgeColor', 'k');
    end
    
    % ---------- 8. VO 模式：绘制目标位置 + 航路基线 ----------
    if isfield(agents(i), 'mode') && strcmpi(agents(i).mode, 'VO')
        % 目标点
        if isfield(agents(i), 'goal') && numel(agents(i).goal) >= 2
            plot(agents(i).goal(1), agents(i).goal(2), 'x', ...
                'MarkerSize', 10, 'LineWidth', 2, ...
                'Color', agentColor);
            text(agents(i).goal(1) + 0.3, agents(i).goal(2) + 0.3, ...
                sprintf('Goal %d', agents(i).id), ...
                'FontSize', 8, 'Color', agentColor * 0.8);
        end
        
        % 航路保持基线：从 pathOrigin 到 goal（如果有）
        if isprop(agents(i), 'pathOrigin') && ~isempty(agents(i).pathOrigin) && ...
                isfield(agents(i), 'goal') && numel(agents(i).goal) >= 2
            origin = agents(i).pathOrigin(:).';
            goal   = agents(i).goal(:).';
            if all(isfinite(origin)) && all(isfinite(goal)) && norm(goal - origin) > 1.0
                plot([origin(1), goal(1)], [origin(2), goal(2)], ':', ...
                    'Color', [0.5 0.5 0.5 0.7], ...
                    'LineWidth', 1.5, ...
                    'HandleVisibility', 'off');
            end
        end
    end
end

% ========== 添加图例 ==========
legendHandles = gobjects(0);
legendLabels  = strings(0);

for i = 1:min(10, numel(agents))  % 最多显示 10 个智能体的图例
    h = plot(NaN, NaN, '-', 'LineWidth', 2.5, 'Color', agents(i).color);
    legendHandles(end+1) = h;
    
    if isfield(agents(i), 'mode') && strcmpi(agents(i).mode, 'CONST')
        if isfield(agents(i), 'waypoints') && ~isempty(agents(i).waypoints)
            labelText = sprintf('Agent %d (CONST-%s)', agents(i).id, upper(agents(i).wpMode));
        else
            labelText = sprintf('Agent %d (CONST)', agents(i).id);
        end
    else
        labelText = sprintf('Agent %d (%s)', agents(i).id, agents(i).mode);
    end
    legendLabels(end+1) = string(labelText);
end

if ~isempty(legendHandles)
    legend(legendHandles, cellstr(legendLabels), ...
        'Location', 'northeastoutside', ...
        'FontSize', 9, 'Box', 'on');
end

% ========== 添加标题和坐标轴标签 ==========
totalTime = totalSteps * dt;
title(sprintf('多智能体轨迹总览 - %s 算法\n总仿真时间: %.2f 秒 | 步数: %d | 时间标记间隔: %.1f 秒', ...
    params.algorithm.name, totalTime, totalSteps, timeMarkInterval), ...
    'FontSize', 12, 'FontWeight', 'bold');

xlabel('X (m)', 'FontSize', 11);
ylabel('Y (m)', 'FontSize', 11);

% ========== 添加统计信息文本框 ==========
statsText = sprintf('仿真统计:\n智能体数: %d\n', numel(agents));
for i = 1:min(5, numel(agents))
    trajLen = 0;
    
    if isstruct(trajectories)
        if i <= numel(trajectories) && ~isempty(trajectories(i).position)
            posAll = trajectories(i).position;
            maxN = min(size(posAll, 1), totalSteps);
            posAll = posAll(1:maxN, :);
            validRows = ~any(isnan(posAll), 2);
            pos = posAll(validRows, :);
        else
            pos = [];
        end
    else
        if i <= numel(trajectories) && ~isempty(trajectories{i})
            pos = trajectories{i};
        else
            pos = [];
        end
    end
    
    if ~isempty(pos) && size(pos, 1) > 1
        diffSeg = diff(pos, 1, 1);
        trajLen = sum(sqrt(sum(diffSeg.^2, 2)));
    end
    
    statsText = [statsText, sprintf('A%d: 路径长度=%.2fm\n', agents(i).id, trajLen)];
end

annotation('textbox', [0.02, 0.75, 0.12, 0.2], ...
    'String', statsText, ...
    'FontSize', 9, ...
    'BackgroundColor', [1 1 1 0.9], ...
    'EdgeColor', 'k', ...
    'LineWidth', 1);

% ========== FSM 状态说明框（替代原来的时间标记说明） ==========
annotation('textbox', [0.02, 0.55, 0.15, 0.1], ...
    'String', sprintf(['FSM 状态可视化:\n', ...
                       '粗黄线: Observe 阶段 (观测期 μ)\n', ...
                       '粗红线: Action 阶段 (A1-A5)\n', ...
                       '细线: Normal 正常航行']), ...
    'FontSize', 9, ...
    'BackgroundColor', [1 0.9 0.9 0.9], ...
    'EdgeColor', [0.5 0 0], ...
    'LineWidth', 1);

hold off;

% ========== 保存图像选项 ==========
if isfield(params, 'viz') && isfield(params.viz, 'saveTrajectoryImage') ...
        && params.viz.saveTrajectoryImage
    filename = sprintf('trajectory_%s_%s.png', ...
        params.algorithm.name, datestr(now, 'yyyymmdd_HHMMSS'));
    saveas(gcf, filename);
    fprintf('轨迹图像已保存: %s\n', filename);
end

end
