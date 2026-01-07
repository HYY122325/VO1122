function plotAgentsV1_1(agents, trajectories, showVelocity, showGoals, showSafetyCircle, showSpeedValue, safetyRadius, params)
% plotAgentsV1_1 - V1.1版本绘图函数
% 修改功能 (Q2/Q4)：
% - Q2新增：FSM状态可视化（Observe/Action阶段高亮显示）
% - Q4新增：动力学模式下显示实际航向指示器

% -------- 参数默认值 --------
if nargin < 2 || isempty(trajectories), trajectories = {}; end
if nargin < 3 || isempty(showVelocity), showVelocity = true; end
if nargin < 4 || isempty(showGoals), showGoals = true; end
if nargin < 5 || isempty(showSafetyCircle), showSafetyCircle = true; end
if nargin < 6 || isempty(showSpeedValue), showSpeedValue = true; end
if nargin < 7 || isempty(safetyRadius), safetyRadius = 0.5; end
if nargin < 8, params = struct(); end
if ~isfield(params,'viz') || ~isfield(params.viz,'showVOCones'), params.viz.showVOCones = false; end
if ~isfield(params,'lvo'), params.lvo = struct('ds_base',2.5,'dp_factor',2.2,'useGoodwin',true); end
if ~isfield(params,'agent') || ~isfield(params.agent,'timeHorizon'), params.agent.timeHorizon = 10; end

% 确保 perceptionDist 存在
if isfield(params, 'agent') && isfield(params.agent, 'neighborDist')
    if ~isfield(params.agent,'perceptionDist')
        params.agent.perceptionDist = params.agent.neighborDist;
    end
end


% Q2可视化：新增FSM状态显示开关（建议在SimConfig中添加此参数控制）
if ~isfield(params,'viz') || ~isfield(params.viz,'showFSMStatus')
    params.viz.showFSMStatus = true; 
end

ax = gca;

% ========== 1) 轨迹 ==========
keepTrajTags = strings(0);
for i = 1:numel(agents)
    tag = sprintf('Trajectory_%d', agents(i).id);
    keepTrajTags(end+1) = string(tag); %#ok<AGROW>

    % --- [修复开始] 兼容 Struct 和 Cell 两种数据格式 ---
    posData = [];
    if isstruct(trajectories)
        % 新版 main1_3 传入的是结构体数组
        if i <= numel(trajectories)
            posData = trajectories(i).position;
        end
    elseif iscell(trajectories)
        % 旧版兼容：如果是 cell 数组
        if i <= numel(trajectories)
            posData = trajectories{i};
        end
    end
    % ------------------------------------------------

    if ~isempty(posData) && size(posData,1) > 1
        h = findobj(ax, 'Type','line', 'Tag', tag);
        if isempty(h) || ~isgraphics(h)
            % 使用提取出来的 posData 绘图
            plot(posData(:,1), posData(:,2), '-', ...
                'Color', agents(i).color, 'LineWidth', 1.2, 'Tag', tag);
        else
            % 更新数据
            set(h,'XData',posData(:,1),'YData',posData(:,2),'Color',agents(i).color);
        end
    end
end

% 清理不存在的轨迹
allTraj = findobj(ax, '-regexp', 'Tag', '^Trajectory_');
for k = 1:numel(allTraj)
    if ~any(strcmp(string(get(allTraj(k),'Tag')), keepTrajTags))
        delete(allTraj(k));
    end
end

% ========== 2) 删除非轨迹元素（本帧重画） ==========
allObjs = allchild(ax);
trajObjs = findobj(ax, '-regexp', 'Tag', '^Trajectory_');
delete(setdiff(allObjs, trajObjs));

hold(ax, 'on'); axis(ax, 'equal'); grid(ax, 'on'); box(ax, 'on');

% ========== 3) （可选）先画 LVO CCO/BCO 扇区 ==========
if params.viz.showVOCones
    drawVOCones_runtime(ax, agents, params);
end

% ========== 4) 绘制CONST agents的waypoints路径 ==========
for i = 1:numel(agents)
    if strcmpi(agents(i).mode,'CONST') && ~isempty(agents(i).waypoints) && size(agents(i).waypoints,1) >= 2
        wp = agents(i).waypoints;
        pathColor = agents(i).color * 0.6 + [0.4 0.4 0.4];
        % 根据模式画路径
        if strcmpi(agents(i).wpMode, 'loop')
            % 循环：连接最后一点到第一点
            wpLoop = [wp; wp(1,:)];
            plot(wpLoop(:,1), wpLoop(:,2), '--', 'Color', pathColor, ...
                'LineWidth', 2.5, 'Tag', 'Dyn_WPPath');
            % 标注为循环
            midIdx = floor(size(wp,1)/2);
            text(wp(midIdx,1), wp(midIdx,2), ' ↻LOOP', ...
                'FontSize', 9, 'Color', pathColor, 'FontWeight', 'bold', 'Tag', 'Dyn_WPLabel');
        elseif strcmpi(agents(i).wpMode, 'pingpong')
            % Ping-pong：来回
            plot(wp(:,1), wp(:,2), '--', 'Color', pathColor, ...
                'LineWidth', 2.5, 'Tag', 'Dyn_WPPath');
            % 标注为往返
            midIdx = floor(size(wp,1)/2);
            text(wp(midIdx,1), wp(midIdx,2), ' ↔PP', ...
                'FontSize', 9, 'Color', pathColor, 'FontWeight', 'bold', 'Tag', 'Dyn_WPLabel');
        else
            % none：单向
            plot(wp(:,1), wp(:,2), '--', 'Color', pathColor, ...
                'LineWidth', 2, 'Tag', 'Dyn_WPPath');
        end
        % 画waypoint点（小方块）
        plot(wp(:,1), wp(:,2), 's', 'MarkerSize', 6, ...
            'MarkerFaceColor', pathColor, 'MarkerEdgeColor', 'k', ...
            'LineWidth', 1, 'Tag', 'Dyn_WPPoints');
        % 高亮当前目标waypoint
        currentWP = agents(i).currentWaypoint();
        plot(currentWP(1), currentWP(2), 'o', 'MarkerSize', 10, ...
            'MarkerFaceColor', [1 0.8 0], 'MarkerEdgeColor', 'k', ...
            'LineWidth', 2, 'Tag', 'Dyn_CurrentWP');
    end
end


% ========== 5) 绘制当前帧智能体本体（修改） ==========
for i = 1:numel(agents)
    agentColor = agents(i).color;
    edgeColor = 'k';
    lineWidth = 1.5;
    
    % Q2可视化：根据FSM状态改变外观
    fsm_mode = 0;
    isObserving = false;
    observe_ids = [];
    
    if params.viz.showFSMStatus && isprop(agents(i), 'debugInfo') && ~isempty(agents(i).debugInfo)
        if isfield(agents(i).debugInfo, 'fsm_mode')
            fsm_mode = agents(i).debugInfo.fsm_mode;
        end
        if isfield(agents(i).debugInfo, 'fsm_observe_ids') && ~isempty(agents(i).debugInfo.fsm_observe_ids)
            isObserving = true;
            observe_ids = agents(i).debugInfo.fsm_observe_ids;
        end
        
        % 外观规则：Observe > Action > 正常
        if isObserving
            % Observe阶段：黄色加粗边框
            edgeColor = [1, 0.8, 0]; 
            lineWidth = 3.0;
        elseif fsm_mode > 0
            % Action阶段：红色加粗边框
            edgeColor = [1, 0.2, 0.2];
            lineWidth = 3.0;
        end
    end
    
    % 绘制Agent本体
    rectangle('Position', [agents(i).position(1)-agents(i).radius, ...
        agents(i).position(2)-agents(i).radius, ...
        2*agents(i).radius, 2*agents(i).radius], ...
        'Curvature', [1,1], 'FaceColor', agentColor, ...
        'EdgeColor', edgeColor, 'LineWidth', lineWidth, 'Tag', 'Dyn_Agent');
    
    % 绘制Agent ID文本
    text(agents(i).position(1), agents(i).position(2), sprintf('%d', agents(i).id), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', 'w', 'Tag', 'Dyn_AgentText');

    % Q2可视化：添加FSM状态文本标签
    if params.viz.showFSMStatus
        statusText = '';
        textColor = edgeColor;
        if isObserving
            statusText = sprintf('Obs(%s)', num2str(observe_ids));
        elseif fsm_mode > 0
            statusText = sprintf('Act(M%d)', fsm_mode);
        end
        
        if ~isempty(statusText)
            % 将文本放在Agent下方
            text(agents(i).position(1), agents(i).position(2) - agents(i).radius*1.8, statusText, ...
                'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
                'FontSize', 8, 'FontWeight', 'bold', 'Color', textColor, 'Tag', 'Dyn_FSMText');
        end
    end
end

% ========== 6) 安全冗余圈 ==========
if showSafetyCircle
    for i = 1:numel(agents)
        rectangle('Position', [agents(i).position(1)-safetyRadius, ...
            agents(i).position(2)-safetyRadius, ...
            2*safetyRadius, 2*safetyRadius], ...
            'Curvature', [1,1], 'FaceColor', 'none', ...
            'EdgeColor', [agents(i).color, 0.7], ...
            'LineWidth', 1.2, 'Tag', 'Dyn_Safety');
    end
end

% ========== 7) 目标点 ==========
if showGoals
    for i = 1:numel(agents)
        plot(agents(i).goal(1), agents(i).goal(2), 'x', ...
            'MarkerSize', 12, 'LineWidth', 2.5, ...
            'Color', agents(i).color, 'Tag', 'Dyn_Goal');
    end
end

% ========== 8) 速度矢量与速度标注 ==========
if showVelocity
    for i = 1:numel(agents)
        v = agents(i).velocity;
        if norm(v) > 0.01
            
            % Q4集成：如果使用动力学，显示航向指示器（三角形）
            % 这样做是因为动力学模式下速度矢量（实际轨迹方向）可能与船首向（psi）不同
            if isprop(agents(i), 'useDynamics') && agents(i).useDynamics && isprop(agents(i), 'dynState')
                psi = agents(i).dynState.psi;
                R = agents(i).radius;
                % 绘制一个指向航向的三角形（船首指示）
                verts = [cos(psi)*R*1.2, sin(psi)*R*1.2; 
                         cos(psi+pi*0.8)*R*0.5, sin(psi+pi*0.8)*R*0.5;
                         cos(psi-pi*0.8)*R*0.5, sin(psi-pi*0.8)*R*0.5];
                patch(agents(i).position(1) + verts(:,1), agents(i).position(2) + verts(:,2), ...
                      'w', 'EdgeColor', 'k', 'FaceAlpha', 0.8, 'Tag', 'Dyn_HeadingIndicator');
            end
            
            % 绘制速度矢量（实际运动方向）
            quiver(agents(i).position(1), agents(i).position(2), ...
                v(1), v(2), 1.2, 'Color', agents(i).color, ...
                'LineWidth', 1.5, 'MaxHeadSize', 0.7, 'Tag', 'Dyn_Vel');
            
            if showSpeedValue
                speed = norm(v);
                textOffset = agents(i).radius * 1.2;
                text(agents(i).position(1)+textOffset, ...
                    agents(i).position(2)+textOffset, ...
                    sprintf('v=%.2f', speed), ...
                    'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
                    'FontSize', 7, 'Color', 'k', 'Tag', 'Dyn_SpeedText');
            end
        end
    end
end

% ========== 9) 图例 ==========
legendHandles = gobjects(0);
legendLabels = strings(0);
for i = 1:min(20, numel(agents))
    h = plot(NaN, NaN, 'o', 'MarkerSize', 10, ...
        'MarkerFaceColor', agents(i).color, ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1, 'Tag', 'Dyn_LegendProxy');
    legendHandles(end+1) = h; %#ok<AGROW>
    % 图例文本：显示模式和waypoint模式
    if strcmpi(agents(i).mode,'CONST') && ~isempty(agents(i).waypoints)
        labelText = sprintf('%d(CONST-%s)', agents(i).id, upper(agents(i).wpMode));
    else
        labelText = sprintf('%d(%s)', agents(i).id, agents(i).mode);
    end
    % Q4: 图例中显示动力学状态
    if isprop(agents(i), 'useDynamics') && agents(i).useDynamics
        labelText = strcat(labelText, '[D]');
    end
    legendLabels(end+1) = string(labelText); %#ok<AGROW>
end
if ~isempty(legendHandles)
    legend(legendHandles, cellstr(legendLabels), ...
        'Location', 'northeast', 'FontSize', 9, ...
        'Box', 'on', 'NumColumns', 1);
end

hold(ax, 'off');
end

% ================== 运行时 LVO 扇区绘制 ==================
% (以下辅助函数保持不变，仅为了文件完整性提供)
function drawVOCones_runtime(ax, agents, params)

if ~isfield(params,'lvo'), return; end
if ~isfield(params,'agent'), params.agent = struct(); end

% 方位采样数量，尽量和 LVO hdgGridN 对齐
if isfield(params.lvo,'hdgGridN')
    Ntheta = params.lvo.hdgGridN;
else
    Ntheta = 180;
end

xl = xlim(ax); yl = ylim(ax);
sceneScale = max(diff(xl), diff(yl));

if isfield(params.agent,'safetyRadius')
    safR = params.agent.safetyRadius;
else
    safR = 1.0;
end
rBroadVis = max(0.08*sceneScale, 3*safR);
rCoreVis  = max(0.06*sceneScale, 2*safR);

if isfield(params.agent,'perceptionDist')
    perceptionDist = params.agent.perceptionDist;
else
    perceptionDist = inf;
end

if isfield(params.agent,'timeHorizon')
    T_default = params.agent.timeHorizon;
else
    T_default = 10;
end

for i = 1:numel(agents)
    os = agents(i);
    if ~strcmpi(os.mode,'VO'), continue; end

    % 邻船列表：优先用 Agent.neighbors，没有就退回到所有其他船
    nbs = os.neighbors;
    if isempty(nbs)
        nbs = agents;
        nbs(i) = [];
    end
    if isempty(nbs), continue; end

    % 海速模长：与 Step5E 一致，优先用 prefVelocity
    Vsea = norm(os.prefVelocity);
    if Vsea < 1e-3
        if isprop(os,'maxSpeed') && os.maxSpeed > 1e-3
            Vsea = os.maxSpeed;
        else
            % 静止 VO 就不画圆锥了
            continue;
        end
    end

    % 时间视野：优先用 Agent.timeHorizon
    if isprop(os,'timeHorizon') && ~isempty(os.timeHorizon) ...
            && isfinite(os.timeHorizon) && os.timeHorizon > 0
        T = os.timeHorizon;
    else
        T = T_default;
    end

    thetas = linspace(-pi, pi, Ntheta);
    inCore  = false(1,Ntheta);
    inBroad = false(1,Ntheta);

    for it = 1:Ntheta
        C = thetas(it);
        v_os = Vsea * [cos(C), sin(C)];  % 候选 OS 速度

        for j = 1:numel(nbs)
            nb = nbs(j);
            RP = nb.position - os.position;   % TS - OS
            RV = nb.velocity - v_os;          % V_TS - V_OS(cand)

            [ds, dp] = compute_ds_dp_runtime(RP, C, params);

            [hitCore, ~]  = dcpaTcpaHit_runtime(RP, RV, ds, T, perceptionDist);
            [hitBroad, ~] = dcpaTcpaHit_runtime(RP, RV, dp, T, perceptionDist);

            inCore(it)  = inCore(it)  || hitCore;
            inBroad(it) = inBroad(it) || (~hitCore && hitBroad);
        end
    end

    % BCO 区域：浅色
    drawSectors_runtime(ax, os.position, inBroad, thetas, [1 0.85 0.5], rBroadVis);
    % CCO 区域：深色覆盖
    drawSectors_runtime(ax, os.position, inCore,  thetas, [1 0.40 0.30], rCoreVis);
end
end

function drawSectors_runtime(ax, center, mask, thetas, colorRGB, radius)
if ~any(mask), return; end
idx = find(mask);
cuts = find(diff(idx) > 1);
starts = [idx(1), idx(cuts+1)];
ends = [idx(cuts), idx(end)];
for k = 1:numel(starts)
    a1 = thetas(starts(k)); a2 = thetas(ends(k));
    if a2 < a1, t=a1; a1=a2; a2=t; end
    ang = linspace(a1, a2, max(4, round((a2-a1)/(2*pi)*100)));
    x = center(1) + radius * cos(ang);
    y = center(2) + radius * sin(ang);
    patch(ax, [center(1) x center(1)], [center(2) y center(2)], colorRGB, ...
        'FaceAlpha', 0.25, 'EdgeColor', colorRGB*0.85, 'LineStyle','-', 'Tag','Dyn_VO');
end
end

%% ====== 与 Step5E 对齐的 ds/dp 与 DCPA/TCPA 判定 ======
function [ds, dp] = compute_ds_dp_runtime(RP, refHdg, params)
% 与 computeVO_Step5E 里的 compute_ds_dp_paper 逻辑一致：

ds0       = params.lvo.ds_base;
dp_factor = params.lvo.dp_factor;
useGoodwin = true;
if isfield(params.lvo,'useGoodwin')
    useGoodwin = params.lvo.useGoodwin;
end

if useGoodwin
    bearing = atan2(RP(2), RP(1));
    B = abs(atan2(sin(bearing - refHdg), cos(bearing - refHdg)));
    ds = ds_from_goodwin_piecewise_runtime(B, ds0);
else
    ds = ds0;
end
dp = max(1e-6, dp_factor * ds);
end

function s = ds_from_goodwin_piecewise_runtime(B, ds0)
Bb = mod(B, 2*pi);
if Bb < 5*pi/8
    f = 1.1 - (Bb/pi)*0.2;
elseif Bb < pi
    f = 1.0 - (Bb/pi)*0.4;
elseif Bb < 11*pi/8
    f = 1.0 - ((2*pi - Bb)/pi)*0.4;
else
    f = 1.1 - ((2*pi - Bb)/pi)*0.4;
end
s = max(1e-3, ds0 * f);
end

function [hit, ttc] = dcpaTcpaHit_runtime(RP, RV, radius, timeHorizon, perceptionDist)
% 与 computeVO_Step5E 文件中的 dcpaTcpaHit 一致（只是改名）
a  = dot(RV, RV);
r0 = norm(RP);

if r0 > perceptionDist
    hit = false; ttc = inf; return;
end

if r0 <= radius
    hit = true;  ttc = 0;   return;
end

if a < 1e-8
    hit = (r0 <= 1.2*radius);
    ttc = inf;
    return;
end

dotrv      = dot(RP, RV);
approaching= (dotrv < 0);
tcpa0      = - dotrv / a;
if approaching, tcpa0 = max(tcpa0, 0); end
tcpa = min(max(tcpa0, 0), timeHorizon);
dcpa = norm(RP + RV * tcpa);

hit = approaching && (dcpa <= radius);

if ~approaching
    ttc = inf;
else
    if tcpa0 <= timeHorizon
        ttc = tcpa0;
    else
        if dcpa <= radius
            ttc = timeHorizon;
        else
            ttc = inf;
        end
    end
end
end