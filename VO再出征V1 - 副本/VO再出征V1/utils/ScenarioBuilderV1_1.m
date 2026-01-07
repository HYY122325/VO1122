function [agentsOut, paramsOut] = ScenarioBuilderV1_1(paramsIn)
% UI 场景构建器 V1.1（含修复 & 增强 & 和配置类对齐版）
% 变更要点：
% 1) 移除非法 LineSpec（如 '->','o->','<->'），修复 loop/pingpong 预览异常
% 2) Waypoints 预览对象 HitTest 关闭，保证坐标轴点击可用
% 3) 新增：CONST 船路径“虚线显示”开关
% 4) ★修改：VO 的 NeighborDist / TimeHorizon / PerceptionDist 都以配置类为真值源
% 5) ★修改：导出场景脚本不再写死 timeHorizon / neighborDist（由配置类统一控制）

% ---------- 参数准备（每次都用默认值） ----------
if nargin < 1 || isempty(paramsIn)
    if exist('+config/SimConfigV1_1.m','file')
        params = config.SimConfigV1_1.default();
    else
        params = SimConfigV1_1.default();
    end
else
    params = paramsIn;
end

params.sim.scenario = 'ui_custom';
agentsOut = [];
paramsOut = params;

% LVO 相关默认值（仅在配置里缺失时兜底，不覆盖配置）
if ~isfield(params,'lvo'), params.lvo = struct(); end
if ~isfield(params.lvo,'ds_base'),    params.lvo.ds_base = 2.5;   end
if ~isfield(params.lvo,'dp_factor'),  params.lvo.dp_factor = 2.2; end

% ★修改：这几个只在缺失时才给默认，不和配置的数值打架
if ~isfield(params,'agent'), params.agent = struct(); end
if ~isfield(params.agent,'timeHorizon'),    params.agent.timeHorizon    = 10.0; end
if ~isfield(params.agent,'perceptionDist'), params.agent.perceptionDist = 6.0;  end
if ~isfield(params.agent,'neighborDist')
    % 如果没配 neighborDist，就先用 perceptionDist 兜底
    if isfield(params.agent,'perceptionDist')
        params.agent.neighborDist = params.agent.perceptionDist;
    else
        params.agent.neighborDist = 10.0;
    end
end

% ---------- UI 布局 ----------
f = uifigure('Name','Scenario Builder V1.1 - Click to Add Waypoints',...
    'Position',[50 50 1600 950]);
gl = uigridlayout(f,[9,2]);
gl.RowHeight = {30,30,'1x','1x',40,40,40,40,40};
gl.ColumnWidth = {'1.2x','0.8x'};

% ===== Row1-2: 顶部参数面板 =====
pTop = uipanel(gl,'BorderType','none');
pTop.Layout.Row = [1 2];
pTop.Layout.Column = [1 2];
gTop = uigridlayout(pTop,[2,5]);
gTop.RowHeight = {30,30};
gTop.ColumnWidth = {220,220,280,280,200};

% Row1 Col1: 场地大小
p0 = uipanel(gTop,'Title','Field Size');
p0.Layout.Row = 1; p0.Layout.Column = 1;
g0 = uigridlayout(p0,[1,2]);
g0.ColumnWidth = {110,90};
uilabel(g0,'Text','Half-length:');
eField = uieditfield(g0,'numeric','Value',params.world.fieldSize);

% Row1 Col2: 时间参数
p1 = uipanel(gTop,'Title','Time Parameters');
p1.Layout.Row = 1; p1.Layout.Column = 2;
g1 = uigridlayout(p1,[1,4]);
g1.ColumnWidth = {35,70,50,65};
uilabel(g1,'Text','dt:');
eDt = uieditfield(g1,'numeric','Value',params.sim.dt);
uilabel(g1,'Text','steps:');
eSteps = uieditfield(g1,'numeric','Value',params.sim.maxSteps);

% Row1 Col3: LVO参数
p3 = uipanel(gTop,'Title','LVO Parameters (ds/dpf/TH)');
p3.Layout.Row = 1; p3.Layout.Column = 3;
g3 = uigridlayout(p3,[1,6]);
g3.ColumnWidth = {30,60,35,60,30,60};
uilabel(g3,'Text','ds:');
eDs = uieditfield(g3,'numeric','Value',params.lvo.ds_base);
uilabel(g3,'Text','dpf:');
eDpf = uieditfield(g3,'numeric','Value',params.lvo.dp_factor);
uilabel(g3,'Text','TH:');
eTH = uieditfield(g3,'numeric','Value',params.agent.timeHorizon);

% Row1 Col4: 默认值
p2 = uipanel(gTop,'Title','Default Values');
p2.Layout.Row = 1; p2.Layout.Column = 4;
g2 = uigridlayout(p2,[1,6]);
g2.ColumnWidth = {25,50,45,50,30,50};
uilabel(g2,'Text','r:');
eR = uieditfield(g2,'numeric','Value',params.agent.radius);
uilabel(g2,'Text','vmax:');
eVmax = uieditfield(g2,'numeric','Value',params.agent.maxSpeed);
uilabel(g2,'Text','safe:');
eSafe = uieditfield(g2,'numeric','Value',params.agent.safetyRadius);

% Row1 Col5: 感知距离 + 邻居距离
p4 = uipanel(gTop,'Title','Perception / Neighbor Dist');
p4.Layout.Row = 1; p4.Layout.Column = 5;
% ★修改：改成同时显示感知距离 & 邻居距离
g4 = uigridlayout(p4,[1,4]);
g4.ColumnWidth = {80,70,80,70};
uilabel(g4,'Text','Perception:');
ePerceptionDist = uieditfield(g4,'numeric','Value',params.agent.perceptionDist);
uilabel(g4,'Text','Neighbor:');
eNeighborDist   = uieditfield(g4,'numeric','Value',params.agent.neighborDist);

% Row2 Col1: CCO/BCO开关
cbShowVO = uicheckbox(gTop,'Text','Show CCO/BCO','Value',true);
cbShowVO.Layout.Row = 2;
cbShowVO.Layout.Column = 1;

% Row2 Col2-5: 网格生成
pg = uipanel(gTop,'Title','Add Grid Fleet','BorderType','line');
pg.Layout.Row = 2;
pg.Layout.Column = [2 5];
gg = uigridlayout(pg,[1,19]);
gg.ColumnWidth = {22,38,22,38,28,38,28,38,28,38,28,38,45,70,28,38,28,38,65};
uilabel(gg,'Text','R:');
eRows = uieditfield(gg,'numeric','Value',2);
uilabel(gg,'Text','C:');
eCols = uieditfield(gg,'numeric','Value',3);
uilabel(gg,'Text','dx:');
edx = uieditfield(gg,'numeric','Value',3.0);
uilabel(gg,'Text','dy:');
edy = uieditfield(gg,'numeric','Value',3.0);
uilabel(gg,'Text','Ox:');
eOx = uieditfield(gg,'numeric','Value',-6.0);
uilabel(gg,'Text','Oy:');
eOy = uieditfield(gg,'numeric','Value',-6.0);
uilabel(gg,'Text','Mode:');
ddMode = uidropdown(gg,'Items',{'VO','CONST'},'Value','CONST');
uilabel(gg,'Text','Vx:');
eGVx = uieditfield(gg,'numeric','Value',1.0);
uilabel(gg,'Text','Vy:');
eGVy = uieditfield(gg,'numeric','Value',0.0);
bGrid = uibutton(gg,'Text','Add Grid','ButtonPushedFcn',@(s,e)addGrid());

% ===== Row3-4 Col1: 左侧Tab页 =====
tabGroup = uitabgroup(gl);
tabGroup.Layout.Row = [3 4];
tabGroup.Layout.Column = 1;

% Tab1: VO Agents表格
tab1 = uitab(tabGroup,'Title','VO Agents');
gl1 = uigridlayout(tab1,[2,1]);
gl1.RowHeight = {'1x',40};

colNamesVO = {'ID','StartX','StartY','GoalX','GoalY','MaxSpeed','Radius',...
    'SafetyRadius','NeighborDist','TimeHorizon','PerceptionDist'};
colEditableVO = true(1,11);
colFormatsVO = cell(1,11);

tblVO = uitable(gl1,'ColumnName',colNamesVO,'ColumnEditable',colEditableVO);
tblVO.ColumnFormat = colFormatsVO;
tblVO.Data = defaultTableVO();
tblVO.CellSelectionCallback = @(s,e)onTableVOSelect(e);

gBtns1 = uigridlayout(gl1,[1,4]);
gBtns1.ColumnWidth = {'1x','1x','1x','1x'};
btnAddVO = uibutton(gBtns1,'Text','+ Add VO','ButtonPushedFcn',@(s,e)addRowVO());
btnDelVO = uibutton(gBtns1,'Text','- Delete','ButtonPushedFcn',@(s,e)delRowVO());
btnClearVO = uibutton(gBtns1,'Text','Clear All','ButtonPushedFcn',@(s,e)clearAllVO());
btnPreviewVO = uibutton(gBtns1,'Text','Preview','ButtonPushedFcn',@(s,e)doPreview());

% Tab2: CONST Agents表格
tab2 = uitab(tabGroup,'Title','CONST Agents');
gl2 = uigridlayout(tab2,[2,1]);
gl2.RowHeight = {'1x',40};

colNamesCONST = {'ID','StartX','StartY','GoalX','GoalY','MaxSpeed','Radius',...
    'SafetyRadius','Vx','Vy'};
colEditableCONST = true(1,10);
colFormatsCONST = cell(1,10);

tblCONST = uitable(gl2,'ColumnName',colNamesCONST,'ColumnEditable',colEditableCONST);
tblCONST.ColumnFormat = colFormatsCONST;
tblCONST.Data = defaultTableCONST();
tblCONST.CellSelectionCallback = @(s,e)onTableCONSTSelect(e);

gBtns2 = uigridlayout(gl2,[1,4]);
gBtns2.ColumnWidth = {'1x','1x','1x','1x'};
btnAddCONST = uibutton(gBtns2,'Text','+ Add CONST','ButtonPushedFcn',@(s,e)addRowCONST());
btnDelCONST = uibutton(gBtns2,'Text','- Delete','ButtonPushedFcn',@(s,e)delRowCONST());
btnClearCONST = uibutton(gBtns2,'Text','Clear All','ButtonPushedFcn',@(s,e)clearAllCONST());
btnPreviewCONST = uibutton(gBtns2,'Text','Preview','ButtonPushedFcn',@(s,e)doPreview());

% Tab3: Waypoints编辑器 - 支持鼠标点击
tab3 = uitab(tabGroup,'Title','Waypoints Editor');
gl3 = uigridlayout(tab3,[12,1]); % +1 行给虚线开关
gl3.RowHeight = {35,'1x',35,35,35,30,30,30,40,40,40,40};

lblSelected = uilabel(gl3,'Text','No CONST agent selected - Please select a CONST agent first',...
    'FontWeight','bold','FontColor',[0.6 0.6 0.6]);

tblWP = uitable(gl3,'ColumnName',{'X','Y'},'ColumnEditable',true(1,2));
tblWP.Data = table([],[],VariableNames={'X','Y'});

gMode = uigridlayout(gl3,[1,2]);
gMode.ColumnWidth = {110,'1x'};
uilabel(gMode,'Text','Path Mode:');
ddWPMode = uidropdown(gMode,'Items',{'none','loop','pingpong'},'Value','none');

gSpeed = uigridlayout(gl3,[1,2]);
gSpeed.ColumnWidth = {110,'1x'};
uilabel(gSpeed,'Text','Const Speed:');
eWPSpeed = uieditfield(gSpeed,'numeric','Value',1.0);

gTol = uigridlayout(gl3,[1,2]);
gTol.ColumnWidth = {110,'1x'};
uilabel(gTol,'Text','WP Tolerance:');
eWPTol = uieditfield(gTol,'numeric','Value',0.3);

lblInfo = uilabel(gl3,'Text','Add waypoints, then click "Apply" to save.',...
    'FontSize',10,'FontColor',[0.4 0.4 0.4],'FontAngle','italic');

% 新增：鼠标点击提示
lblClickInfo = uilabel(gl3,'Text','✓ Click on the preview to add waypoints!',...
    'FontSize',11,'FontColor',[0 0.6 0],'FontWeight','bold','FontAngle','italic');

% 新增：虚线开关
cbDashedConst = uicheckbox(gl3, ...
    'Text','Use dashed path for CONST', ...
    'Value', false);

gBtnsWP1 = uigridlayout(gl3,[1,2]);
gBtnsWP1.ColumnWidth = {'1x','1x'};
btnAddWP = uibutton(gBtnsWP1,'Text','+ Add Waypoint','ButtonPushedFcn',@(s,e)addWaypoint());
btnDelWP = uibutton(gBtnsWP1,'Text','- Delete Last','ButtonPushedFcn',@(s,e)delWaypoint());

gBtnsWP2 = uigridlayout(gl3,[1,1]);
gBtnsWP2.ColumnWidth = {'1x'};
btnClearWP = uibutton(gBtnsWP2,'Text','Clear All Waypoints',...
    'ButtonPushedFcn',@(s,e)clearWaypoints(),...
    'BackgroundColor',[0.9 0.6 0.3],'FontWeight','normal');

gBtnsWP3 = uigridlayout(gl3,[1,1]);
gBtnsWP3.ColumnWidth = {'1x'};
btnApplyWP = uibutton(gBtnsWP3,'Text','Apply Waypoints to CONST Agent',...
    'ButtonPushedFcn',@(s,e)applyWaypoints(),...
    'BackgroundColor',[0.3 0.7 0.3],'FontWeight','bold');

% ===== Row3-4 Col2: 右侧预览区（支持鼠标点击） =====
ax = uiaxes(gl);
axis(ax,'equal');
grid(ax,'on');
box(ax,'on');
title(ax,'Preview - Click to add waypoints (when editing CONST agent)');
xlabel(ax,'X');
ylabel(ax,'Y');
ax.Layout.Row = [3 4];
ax.Layout.Column = 2;

% 绑定鼠标点击事件
ax.ButtonDownFcn = @(src,event)onAxesClick(event);

% ===== Row5-9: 底部按钮区域 =====
lblInfo2 = uilabel(gl,'Text','Click "Run & Close" to start simulation with current scenario',...
    'FontSize',11,'FontWeight','bold','HorizontalAlignment','center');
lblInfo2.Layout.Row = 5;
lblInfo2.Layout.Column = [1 2];

btnExport = uibutton(gl,'Text','Export Scenario (m-file)',...
    'ButtonPushedFcn',@(s,e)doExport(),'FontSize',12);
btnExport.Layout.Row = 6;
btnExport.Layout.Column = [1 2];

btnRefresh = uibutton(gl,'Text','Refresh Preview',...
    'ButtonPushedFcn',@(s,e)doPreview(),'FontSize',12,...
    'BackgroundColor',[0.3 0.6 0.9],'FontWeight','bold');
btnRefresh.Layout.Row = 7;
btnRefresh.Layout.Column = [1 2];

btnRun = uibutton(gl,'Text','Run & Close',...
    'ButtonPushedFcn',@(s,e)doRun(),'FontSize',14,...
    'BackgroundColor',[0.2 0.7 0.3],'FontWeight','bold');
btnRun.Layout.Row = 8;
btnRun.Layout.Column = [1 2];

btnClose = uibutton(gl,'Text','Close',...
    'ButtonPushedFcn',@(s,e)close(f),'FontSize',12);
btnClose.Layout.Row = 9;
btnClose.Layout.Column = [1 2];

% 存储waypoints数据和选中状态
waypointsData = struct();
selectedAgentID = [];
clickAddMode = false; % 鼠标点击添加模式

% 表格编辑回调
tblVO.CellEditCallback = @(s,e)doPreview();
tblCONST.CellEditCallback = @(s,e)doPreview();

% 初次预览
doPreview();

% 等待 UI 关闭
uiwait(f);

% ==================== 嵌套工具函数 ====================
    function T = defaultTableVO()
        % 每次都返回默认值，不记忆上次设置
        fs = params.world.fieldSize;
        sr = params.agent.safetyRadius;
        nd = params.agent.neighborDist;      % ★修改：邻居距离用配置
        th = params.agent.timeHorizon;
        pd = params.agent.perceptionDist;    % ★修改：感知距离单独一列
        T = table(...
            uint16([1;2]), ...
            [-6; 6], ...
            [-0.7*fs; -0.7*fs], ...
            [-6; 6], ...
            [0.7*fs; 0.7*fs], ...
            [params.agent.maxSpeed; params.agent.maxSpeed], ...
            [params.agent.radius; params.agent.radius], ...
            [sr; sr], ...
            [nd; nd], ...
            [th; th], ...
            [pd; pd], ...
            'VariableNames',colNamesVO);
    end

    function T = defaultTableCONST()
        % 每次都返回默认值，不记忆上次设置
        fs = params.world.fieldSize;
        sr = params.agent.safetyRadius;
        T = table(...
            uint16([3;4]), ...
            [-0.9*fs; 0.9*fs], ...
            [0; 5], ...
            [0.9*fs; -0.9*fs], ...
            [0; 5], ...
            [1.2; 1.0], ...
            [params.agent.radius; params.agent.radius], ...
            [sr; sr], ...
            [1.2; -1.0], ...
            [0.0; 0.0], ...
            'VariableNames',colNamesCONST);
    end

    function onAxesClick(event)
        % 鼠标点击预览图添加waypoint
        if isempty(selectedAgentID)
            % 没有选中CONST agent，忽略点击
            return;
        end

        % 获取点击位置
        clickX = event.IntersectionPoint(1);
        clickY = event.IntersectionPoint(2);

        % 添加到waypoints表格
        D = tblWP.Data;
        newRow = table(clickX, clickY, 'VariableNames', {'X', 'Y'});
        D = [D; newRow];
        tblWP.Data = D;

        % 立即更新预览
        doPreview();

        % 提示用户
        title(ax, sprintf('Waypoint added at (%.2f, %.2f) - Click to add more or press Apply', clickX, clickY));
    end

    function addRowVO()
        D = tblVO.Data;
        maxIDVO = 0;
        maxIDCONST = 0;
        if ~isempty(D), maxIDVO = max([D.ID; 0]); end
        D2 = tblCONST.Data;
        if ~isempty(D2), maxIDCONST = max([D2.ID; 0]); end
        nextID = uint16(max(maxIDVO, maxIDCONST) + 1);
        sr = eSafe.Value;
        nd = eNeighborDist.Value;      % ★修改：邻居距离从独立输入控件取
        th = eTH.Value;
        pd = ePerceptionDist.Value;    % ★修改：感知距离单独一列
        newRow = {nextID,0,0,5,0,eVmax.Value,eR.Value,sr,nd,th,pd};
        D = [D; newRow];
        tblVO.Data = D;
        doPreview();
    end

    function delRowVO()
        if isempty(tblVO.Selection) || all(tblVO.Selection==0), return; end
        rows = unique(tblVO.Selection(:,1));
        D = tblVO.Data;
        D(rows,:) = [];
        tblVO.Data = D;
        doPreview();
    end

    function clearAllVO()
        answer = uiconfirm(f,'Clear all VO agents?','Clear VO','Icon','warning');
        if strcmp(answer,'OK')
            tblVO.Data = table();
            doPreview();
        end
    end

    function addRowCONST()
        D = tblCONST.Data;
        maxIDVO = 0;
        maxIDCONST = 0;
        D2 = tblVO.Data;
        if ~isempty(D2), maxIDVO = max([D2.ID; 0]); end
        if ~isempty(D), maxIDCONST = max([D.ID; 0]); end
        nextID = uint16(max(maxIDVO, maxIDCONST) + 1);
        sr = eSafe.Value;
        newRow = {nextID,0,0,5,0,eVmax.Value,eR.Value,sr,1.0,0.0};
        D = [D; newRow];
        tblCONST.Data = D;
        doPreview();
    end

    function delRowCONST()
        if isempty(tblCONST.Selection) || all(tblCONST.Selection==0), return; end
        rows = unique(tblCONST.Selection(:,1));
        D = tblCONST.Data;
        for r = rows'
            if r <= height(D)
                aid = double(D.ID(r));
                fn = sprintf('agent_%d',aid);
                if isfield(waypointsData, fn)
                    waypointsData = rmfield(waypointsData, fn);
                end
            end
        end
        D(rows,:) = [];
        tblCONST.Data = D;
        doPreview();
    end

    function clearAllCONST()
        answer = uiconfirm(f,'Clear all CONST agents?','Clear CONST','Icon','warning');
        if strcmp(answer,'OK')
            tblCONST.Data = table();
            waypointsData = struct();
            selectedAgentID = [];
            doPreview();
        end
    end

    function addGrid()
        mode = char(ddMode.Value);
        rows = max(1, round(eRows.Value));
        cols = max(1, round(eCols.Value));
        dx = edx.Value;
        dy = edy.Value;
        ox = eOx.Value;
        oy = eOy.Value;
        gv = [eGVx.Value, eGVy.Value];
        sr = eSafe.Value;
        nd = eNeighborDist.Value;       % ★修改
        th = eTH.Value;
        pd = ePerceptionDist.Value;     % ★修改

        maxIDVO = 0;
        maxIDCONST = 0;
        DVO = tblVO.Data;
        DCONST = tblCONST.Data;
        if ~isempty(DVO), maxIDVO = max([DVO.ID; 0]); end
        if ~isempty(DCONST), maxIDCONST = max([DCONST.ID; 0]); end
        nextID = uint16(max(maxIDVO, maxIDCONST) + 1);

        if strcmpi(mode,'VO')
            D = tblVO.Data;
            for r = 0:rows-1
                for c = 0:cols-1
                    sx = ox + c*dx;
                    sy = oy + r*dy;
                    gx = sx + 10;
                    gy = sy;
                    vmax = eVmax.Value;
                    rad = eR.Value;
                    newRow = {nextID, sx, sy, gx, gy, vmax, rad, sr, nd, th, pd};
                    D = [D; newRow];
                    nextID = nextID + 1;
                end
            end
            tblVO.Data = D;
        else
            D = tblCONST.Data;
            for r = 0:rows-1
                for c = 0:cols-1
                    sx = ox + c*dx;
                    sy = oy + r*dy;
                    gx = sx + 10;
                    gy = sy;
                    vmax = eVmax.Value;
                    rad = eR.Value;
                    newRow = {nextID, sx, sy, gx, gy, vmax, rad, sr, gv(1), gv(2)};
                    D = [D; newRow];
                    nextID = nextID + 1;
                end
            end
            tblCONST.Data = D;
        end
        doPreview();
    end

    function onTableVOSelect(evt)
        if isempty(evt.Indices), return; end
        lblSelected.Text = 'VO agents do not use waypoints - Select a CONST agent instead';
        lblSelected.FontColor = [0.8 0.4 0.4];
        selectedAgentID = []; % 清除选中
        clickAddMode = false;
        title(ax,'Preview - VO ships (colored) & CONST ships (gray with waypoints)');
    end

    function onTableCONSTSelect(evt)
        if isempty(evt.Indices), return; end
        row = evt.Indices(1);
        D = tblCONST.Data;
        if row > height(D), return; end
        aid = double(D.ID(row));

        tabGroup.SelectedTab = tab3;
        selectedAgentID = aid;
        clickAddMode = true;
        lblSelected.Text = sprintf('Editing waypoints for CONST Agent %d - Click on preview to add!', aid);
        lblSelected.FontColor = [0 0.6 0];

        % 加载已有waypoints
        fn = sprintf('agent_%d',aid);
        if isfield(waypointsData, fn)
            wp = waypointsData.(fn);
            if ~isempty(wp.points)
                tblWP.Data = array2table(wp.points,'VariableNames',{'X','Y'});
            else
                tblWP.Data = table([],[],VariableNames={'X','Y'});
            end
            ddWPMode.Value = wp.mode;
            eWPSpeed.Value = wp.speed;
            eWPTol.Value = wp.tolerance;
        else
            % 默认使用起点和终点
            sx = D.StartX(row); sy = D.StartY(row);
            gx = D.GoalX(row); gy = D.GoalY(row);
            tblWP.Data = array2table([sx sy; gx gy],'VariableNames',{'X','Y'});
            ddWPMode.Value = 'none';
            eWPSpeed.Value = D.MaxSpeed(row);
            eWPTol.Value = 0.3;
        end

        % 更新预览标题
        title(ax,sprintf('Click to add waypoints for Agent %d', aid));
        doPreview();
    end

    function addWaypoint()
        if isempty(selectedAgentID)
            uialert(f,'Please select a CONST agent first','No Selection');
            return;
        end
        D = tblWP.Data;
        if height(D) == 0
            D = table(0, 0, 'VariableNames', {'X', 'Y'});
        else
            lastRow = D(end,:);
            D = [D; lastRow];
        end
        tblWP.Data = D;
        doPreview();
    end

    function delWaypoint()
        if isempty(selectedAgentID), return; end
        D = tblWP.Data;
        if height(D) <= 0, return; end
        D(end,:) = [];
        tblWP.Data = D;
        doPreview();
    end

    function clearWaypoints()
        if isempty(selectedAgentID)
            uialert(f,'Please select a CONST agent first','No Selection');
            return;
        end
        answer = uiconfirm(f,sprintf('Clear all waypoints for Agent %d?', selectedAgentID),...
            'Clear Waypoints','Icon','warning');
        if strcmp(answer,'OK')
            tblWP.Data = table([],[],VariableNames={'X','Y'});
            doPreview();
        end
    end

    function applyWaypoints()
        if isempty(selectedAgentID)
            uialert(f,'No CONST agent selected','Error');
            return;
        end
        D = tblWP.Data;
        pts = table2array(D);
        mode = char(ddWPMode.Value);
        speed = eWPSpeed.Value;
        tol = eWPTol.Value;
        fn = sprintf('agent_%d', selectedAgentID);
        waypointsData.(fn) = struct('points', pts, 'mode', mode, 'speed', speed, 'tolerance', tol);

        tabGroup.SelectedTab = tab2;
        doPreview();
        uialert(f, sprintf('Waypoints applied to CONST Agent %d', selectedAgentID),...
            'Success','Icon','success');
    end

    function [agents, paramsNow] = buildAgentsFromTable()
        paramsNow = params;
        paramsNow.world.fieldSize = eField.Value;
        fs = paramsNow.world.fieldSize;
        paramsNow.world.xlim = [-fs fs];
        paramsNow.world.ylim = [-fs fs];
        paramsNow.sim.dt = eDt.Value;
        paramsNow.sim.maxSteps = round(eSteps.Value);
        paramsNow.agent.radius        = eR.Value;
        paramsNow.agent.maxSpeed      = eVmax.Value;
        paramsNow.agent.safetyRadius  = eSafe.Value;
        paramsNow.agent.perceptionDist= ePerceptionDist.Value;  % ★修改
        paramsNow.agent.neighborDist  = eNeighborDist.Value;    % ★修改
        paramsNow.lvo.ds_base         = eDs.Value;
        paramsNow.lvo.dp_factor       = eDpf.Value;
        paramsNow.agent.timeHorizon   = eTH.Value;

        DVO = tblVO.Data;
        DCONST = tblCONST.Data;
        nVO = height(DVO);
        nCONST = height(DCONST);
        ntotal = nVO + nCONST;

        if ntotal == 0
            agents = Agent.empty(1,0);
            return;
        end

        agents = repmat(Agent(0,[0 0],[0 0],paramsNow.agent.radius, paramsNow.agent.maxSpeed,'VO',[0,0]), 1, ntotal);

        % 处理VO agents
        for i = 1:nVO
            id = double(DVO.ID(i));
            sx = double(DVO.StartX(i)); sy = double(DVO.StartY(i));
            gx = double(DVO.GoalX(i)); gy = double(DVO.GoalY(i));
            vmax = double(DVO.MaxSpeed(i));
            rad = double(DVO.Radius(i));
            sr  = double(DVO.SafetyRadius(i));
            nd  = double(DVO.NeighborDist(i));
            th  = double(DVO.TimeHorizon(i));
            pd  = double(DVO.PerceptionDist(i));

            agents(i) = Agent(id, [sx,sy], [gx,gy], rad, vmax, 'VO', [0,0]);
            agents(i).timeHorizon = th;
            agents(i).neighborDist = nd;
            agents(i).safetyRadius = sr;
            if isfield(agents(i).debugInfo,'perceptionDist')
                agents(i).debugInfo.perceptionDist = pd;
            end
            agents(i).color = rand(1,3);
        end

        % 处理CONST agents
        for i = 1:nCONST
            id = double(DCONST.ID(i));
            sx = double(DCONST.StartX(i)); sy = double(DCONST.StartY(i));
            gx = double(DCONST.GoalX(i)); gy = double(DCONST.GoalY(i));
            vmax = double(DCONST.MaxSpeed(i));
            rad = double(DCONST.Radius(i));
            sr  = double(DCONST.SafetyRadius(i));
            vx  = double(DCONST.Vx(i));
            vy  = double(DCONST.Vy(i));

            constVel = [0,0];
            if ~isnan(vx) && ~isnan(vy)
                constVel = [vx, vy];
            else
                d = [gx - sx, gy - sy];
                nd_temp = norm(d);
                if nd_temp < 1e-6
                    constVel = [vmax, 0];
                else
                    constVel = d / nd_temp * vmax;
                end
            end

            idx = nVO + i;
            agents(idx) = Agent(id, [sx,sy], [gx,gy], rad, vmax, 'CONST', constVel);
            agents(idx).safetyRadius = sr;
            agents(idx).color = [0.55 0.55 0.55];

            % 应用waypoints
            fn = sprintf('agent_%d', id);
            if isfield(waypointsData, fn)
                wp = waypointsData.(fn);
                if ~isempty(wp.points) && size(wp.points,1) > 0
                    agents(idx).setWaypoints(wp.points, wp.mode, wp.speed, wp.tolerance);
                end
            end
        end
    end

    function doPreview()
        [agentsPrev, paramsPrev] = buildAgentsFromTable();
        cla(ax);
        hold(ax,'on');
        axis(ax,'equal');
        grid(ax,'on');
        box(ax,'on');
        xlim(ax, paramsPrev.world.xlim);
        ylim(ax, paramsPrev.world.ylim);

        if clickAddMode && ~isempty(selectedAgentID)
            title(ax,sprintf('Click to add waypoints for Agent %d (or edit table)', selectedAgentID));
        else
            title(ax,'Preview - VO ships (colored) & CONST ships (gray with waypoints)');
        end

        % 路径线型（虚线开关）
        lineStyleConst = '-';
        if cbDashedConst.Value
            lineStyleConst = '--';
        end

        % 画 CONST waypoints 路径（优先于单体绘制）
        for k = 1:numel(agentsPrev)
            a = agentsPrev(k);
            if strcmpi(a.mode,'CONST') && ~isempty(a.waypoints) && size(a.waypoints,1) >= 2
                wp = a.waypoints;
                pathColor = a.color * 0.7 + [0.3 0.3 0.3];

                % loop 模式闭合
                if strcmpi(a.wpMode, 'loop')
                    wpUse = [wp; wp(1,:)];
                else
                    wpUse = wp;
                end

                % 主路径线
                plot(ax, wpUse(:,1), wpUse(:,2), lineStyleConst, ...
                    'Color', pathColor, 'LineWidth', 2.5, ...
                    'HitTest','off','PickableParts','none');

                % 节点与编号
                plot(ax, wp(:,1), wp(:,2), 's', 'MarkerSize', 9, ...
                    'MarkerFaceColor', pathColor, 'MarkerEdgeColor', 'k', 'LineWidth', 1.2, ...
                    'LineStyle','none', 'HitTest','off','PickableParts','none');
                for wi = 1:size(wp,1)
                    text(ax, wp(wi,1), wp(wi,2), sprintf(' WP%d', wi), ...
                        'FontSize', 9, 'Color', pathColor*0.7, 'FontWeight', 'bold', ...
                        'HitTest','off');
                end

                % 方向提示：pingpong 两端三角；非 pingpong 末段 quiver 箭头
                if strcmpi(a.wpMode, 'pingpong')
                    plot(ax, wp(1,1),  wp(1,2),  '<', 'MarkerSize', 10, ...
                        'MarkerFaceColor', pathColor, 'MarkerEdgeColor','k', ...
                        'LineStyle','none','HitTest','off','PickableParts','none');
                    plot(ax, wp(end,1), wp(end,2), '>', 'MarkerSize', 10, ...
                        'MarkerFaceColor', pathColor, 'MarkerEdgeColor','k', ...
                        'LineStyle','none','HitTest','off','PickableParts','none');
                else
                    if size(wp,1) >= 2
                        v = wp(end,:) - wp(end-1,:);
                        nv = norm(v);
                        if nv > 0
                            u = v / nv;
                            L = min(0.6, max(0.2, nv*0.25)); % 箭头长度
                            quiver(ax, wp(end-1,1), wp(end-1,2), u(1)*L, u(2)*L, 0, ...
                                'Color', pathColor, 'LineWidth', 1.6, 'MaxHeadSize', 1.6, ...
                                'HitTest','off','PickableParts','none');
                        end
                    end
                end
            end
        end

        % 画当前编辑的waypoints（编辑态高亮）
        if clickAddMode && ~isempty(selectedAgentID)
            D = tblWP.Data;
            if height(D) > 0
                pts = table2array(D);
                plot(ax, pts(:,1), pts(:,2), 'o-', ...
                    'LineWidth', 3, 'MarkerSize', 8, ...
                    'HitTest','off','PickableParts','none');
                for i = 1:size(pts,1)
                    text(ax, pts(i,1), pts(i,2), sprintf('  %d', i), ...
                        'FontSize', 12, 'FontWeight', 'bold', 'HitTest','off');
                end
            end
        end

        % 画起点/本体/目标
        for k = 1:numel(agentsPrev)
            a = agentsPrev(k);
            plot(ax, a.position(1), a.position(2), 'o', ...
                'MarkerFaceColor', a.color, 'MarkerEdgeColor','k', 'MarkerSize', 12, 'LineWidth', 1.5, ...
                'HitTest','off','PickableParts','none');
            plot(ax, a.goal(1), a.goal(2), 'x', 'Color', a.color, ...
                'LineWidth', 3, 'MarkerSize', 14, 'HitTest','off','PickableParts','none');
            rectangle(ax, 'Position', [a.position(1)-a.radius, a.position(2)-a.radius, ...
                2*a.radius, 2*a.radius], 'Curvature', [1,1], ...
                'EdgeColor', a.color, 'LineStyle', '--', 'LineWidth', 1.2, ...
                'HitTest','off');
            rectangle(ax, 'Position', [a.position(1)-a.safetyRadius, a.position(2)-a.safetyRadius, ...
                2*a.safetyRadius, 2*a.safetyRadius], 'Curvature', [1,1], ...
                'EdgeColor', [a.color 0.5], 'LineStyle', ':', 'LineWidth', 1, ...
                'HitTest','off');
            text(ax, a.position(1), a.position(2)-a.radius-0.5, ...
                sprintf('ID:%d(%s)', a.id, a.mode), ...
                'Color', 'k', 'FontSize', 10, 'FontWeight', 'bold', ...
                'HorizontalAlignment', 'center', 'HitTest','off');

            if strcmpi(a.mode,'CONST') && (isempty(a.waypoints) || size(a.waypoints,1) == 0)
                if norm(a.constVel) > 1e-6
                    quiver(ax, a.position(1), a.position(2), ...
                        a.constVel(1), a.constVel(2), 1.0, ...
                        'Color', a.color, 'LineWidth', 2, 'MaxHeadSize', 0.8, ...
                        'HitTest','off','PickableParts','none');
                end
            end
        end

        if cbShowVO.Value
            drawVOCones(ax, agentsPrev, paramsPrev);
        end

        hold(ax,'off');
    end

    function doExport()
        [agentsExp, paramsExp] = buildAgentsFromTable();
        if ~exist('Scenario','dir')
            mkdir('Scenario');
        end
        dst = fullfile('Scenario','scn_ui_custom.m');
        writeScenarioFile(dst, agentsExp, paramsExp);
        uialert(f, sprintf('Scenario exported to:\n%s', dst), 'Export Successful','Icon','success');
    end

    function writeScenarioFile(dst, agentsExp, paramsExp)
        fid = fopen(dst,'w');
        assert(fid>0,'Cannot open file for writing.');
        fprintf(fid,'function [agents, params] = scn_ui_custom(params)\n');
        fprintf(fid,'%% Auto-generated by ScenarioBuilderV1_1 on %s\n', datestr(now));
        fprintf(fid,'%% This scenario contains %d agents\n\n', numel(agentsExp));
        fprintf(fid,'agents = [];\n\n');

        for k = 1:numel(agentsExp)
            a = agentsExp(k);
            fprintf(fid,'%% Agent %d (%s)\n', a.id, a.mode);
            fprintf(fid,'a = Agent(%d,[%.6f %.6f],[%.6f %.6f],%.6f,%.6f,''%s'',[%.6f %.6f]);\n', ...
                a.id, a.position(1), a.position(2), a.goal(1), a.goal(2), ...
                a.radius, a.maxSpeed, a.mode, a.constVel(1), a.constVel(2));

            % ★修改：timeHorizon / neighborDist / perceptionDist 由配置类统一控制，
            % 导出的场景不再写死这些值，避免和 SimConfig 冲突。

            fprintf(fid,'a.safetyRadius = %.6f;\n', a.safetyRadius);
            fprintf(fid,'a.color = [%.3f %.3f %.3f];\n', a.color(1), a.color(2), a.color(3));

            if ~isempty(a.waypoints) && size(a.waypoints,1) > 0
                fprintf(fid,'a.setWaypoints([');
                for wi = 1:size(a.waypoints,1)
                    fprintf(fid,'%.6f %.6f', a.waypoints(wi,1), a.waypoints(wi,2));
                    if wi < size(a.waypoints,1)
                        fprintf(fid,'; ');
                    end
                end
                fprintf(fid,'], ''%s'', %.6f, %.6f);\n', a.wpMode, a.constSpeed, a.wpTol);
            end
            fprintf(fid,'agents = [agents, a];\n\n');
        end

        fprintf(fid,'%% Update parameters if needed (几何 / 仿真相关，避碰静态参数仍由配置类控制)\n');
        fprintf(fid,'params.world.fieldSize = %.6f;\n', paramsExp.world.fieldSize);
        fprintf(fid,'params.sim.dt = %.6f;\n', paramsExp.sim.dt);
        fprintf(fid,'params.sim.maxSteps = %d;\n', paramsExp.sim.maxSteps);
        fprintf(fid,'%% ds_base / dp_factor / timeHorizon / perceptionDist / neighborDist\n');
        fprintf(fid,'%% 请在 SimConfigV1_1.default() 中统一配置。\n');
        fprintf(fid,'\nend\n');
        fclose(fid);
    end

    function doRun()
        [agentsExp, paramsExp] = buildAgentsFromTable();
        if isempty(agentsExp)
            uialert(f,'No agents defined. Please add agents before running.','No Agents','Icon','warning');
            return;
        end
        agentsOut = agentsExp;
        paramsOut = paramsExp;
        try
            uiresume(f);
        catch
        end
        delete(f);
    end

    function drawVOCones(axIn, agentsIn, paramsIn)
        Ntheta = 180;
        for i = 1:numel(agentsIn)
            os = agentsIn(i);
            if ~strcmpi(os.mode,'VO'), continue; end

            nbs = [];
            for j = 1:numel(agentsIn)
                if j == i, continue; end
                if norm(agentsIn(j).position - os.position) < os.neighborDist*1.5
                    nbs = [nbs, agentsIn(j)];
                end
            end
            if isempty(nbs), continue; end

            thetas = linspace(-pi, pi, Ntheta);
            inCore = false(1,Ntheta);
            inBroad = false(1,Ntheta);

            for it = 1:Ntheta
                hdg = thetas(it);
                v_os = os.maxSpeed * [cos(hdg), sin(hdg)];
                for idx = 1:numel(nbs)
                    nb = nbs(idx);
                    RP = nb.position - os.position;
                    RV = v_os - nb.velocity;
                    [ds, dp] = compute_ds_dp_preview(RP, headingFromVelocity_preview(os), paramsIn);
                    Rgeom = os.radius + nb.radius + paramsIn.agent.safetyRadius;
                    if ds < Rgeom, ds = Rgeom; end
                    if dp < ds*paramsIn.lvo.dp_factor
                        dp = ds*paramsIn.lvo.dp_factor;
                    end
                    hitCore = rayCircleHit_preview(RP, RV, ds, paramsIn.agent.timeHorizon);
                    hitBroad = rayCircleHit_preview(RP, RV, dp, paramsIn.agent.timeHorizon);
                    inCore(it) = inCore(it) || hitCore;
                    inBroad(it) = inBroad(it) || (~hitCore && hitBroad);
                end
            end

            xl = xlim(axIn);
            yl = ylim(axIn);
            sceneScale = max(diff(xl), diff(yl));
            rBroadVis = max(0.10*sceneScale, 4*paramsIn.agent.safetyRadius);
            rCoreVis = max(0.15*sceneScale, 5*paramsIn.agent.safetyRadius);
            drawSectors(axIn, os.position, inBroad, thetas, [1 0.85 0.5], rBroadVis);
            drawSectors(axIn, os.position, inCore, thetas, [1 0.40 0.30], rCoreVis);
        end
    end

    function drawSectors(axIn, center, mask, thetas, colorRGB, radius)
        if ~any(mask), return; end
        idx = find(mask);
        breaks = find(diff(idx) > 1);
        starts = [idx(1), idx(breaks+1)];
        ends = [idx(breaks), idx(end)];
        for k = 1:numel(starts)
            a1 = thetas(starts(k));
            a2 = thetas(ends(k));
            if a2 < a1
                temp = a1; a1 = a2; a2 = temp;
            end
            ang = linspace(a1, a2, max(4, round((a2-a1)/(2*pi)*100)));
            x = center(1) + radius * cos(ang);
            y = center(2) + radius * sin(ang);
            patch(axIn, [center(1) x center(1)], [center(2) y center(2)], colorRGB, ...
                'FaceAlpha', 0.25, 'EdgeColor', colorRGB*0.8, 'LineStyle','-', 'LineWidth', 1, ...
                'HitTest','off','PickableParts','none');
        end
    end

    function hit = rayCircleHit_preview(RP, RV, radius, timeHorizon)
        a = dot(RV,RV);
        b = 2*dot(RP,RV);
        c = dot(RP,RP) - radius^2;
        if a < 1e-8
            hit = false;
            return;
        end
        disc = b^2 - 4*a*c;
        if disc < 0
            hit = false;
            return;
        end
        t1 = (-b - sqrt(disc))/(2*a);
        t2 = (-b + sqrt(disc))/(2*a);
        t = inf;
        if t1 >= 0
            t = t1;
        elseif t2 >= 0
            t = t2;
        end
        hit = isfinite(t) && (t <= timeHorizon);
    end

    function [ds, dp] = compute_ds_dp_preview(RP, osHeading, paramsIn)
        if isfield(paramsIn,'lvo') && isfield(paramsIn.lvo,'useGoodwin') && paramsIn.lvo.useGoodwin
            B = abs(angleDiff_preview(atan2(RP(2), RP(1)), osHeading));
            Bdeg = B * 180/pi;
            if Bdeg <= 22.5
                ds = 1.5 * paramsIn.lvo.ds_base;
            elseif Bdeg <= 112.5
                ds = 1.0 * paramsIn.lvo.ds_base;
            else
                ds = 1.2 * paramsIn.lvo.ds_base;
            end
        else
            ds = paramsIn.lvo.ds_base;
        end
        dp = paramsIn.lvo.dp_factor * ds;
    end

    function hdg = headingFromVelocity_preview(agent)
        v = agent.constVel;
        if norm(v) < 1e-6
            toGoal = agent.goal - agent.position;
            if norm(toGoal) < 1e-6
                hdg = 0;
            else
                hdg = atan2(toGoal(2), toGoal(1));
            end
        else
            hdg = atan2(v(2), v(1));
        end
    end

    function d = angleDiff_preview(a, b)
        d = atan2(sin(a-b), cos(a-b));
    end
end
