function newVelocity = computeVO_Step5J(agent, neighbors, params)
% computeVO_Step5I — Debug 增强版
%
% [Debug 目标]
% 1. 每 1.0 s 打印一帧「决策快照」，按论文流程图结构：
%    S0 本船信息 → S1 前端扫描 → S2 风险+COLREG → S3 FSM/协同 → S4 后端速度选择。
% 2. 若进入 FSM，重点打印各 TS 的 Phase / colregs_mode / overrides。
% 3. 若未进入 FSM，打印 COLREG 场景 + 角色 + Seamanship 使用情况。
% 4. 解释最终速度选择的「来源逻辑」。

%% =========== 参数与默认 ===========

s5       = getfieldwithdef(params, 'step5', struct());
epsC1    = getfieldwithdef(s5, 'c1_eps', 1e-2);

useGW    = getfieldwithdef(params.lvo, 'useGoodwin', true);
ds_base  = params.lvo.ds_base;
dp_factor= params.lvo.dp_factor;

hdgGridN = getfieldwithdef(params.lvo,'hdgGridN',360);
hdgGrid  = linspace(-pi, pi, hdgGridN);

% ------ 仿真时间（用于 debug 前缀） ------
simStep = -1;
simDt   = 0.1;
if isfield(params, 'sim')
    if isfield(params.sim,'step'), simStep = params.sim.step; end
    if isfield(params.sim,'dt'),   simDt   = params.sim.dt;   end
end
if simStep >= 0
    t_now = simStep * simDt;
else
    t_now = NaN;
end
isFirstStep = (simStep <= 1); % [新增] 判断是否需要重置历史记录 (第一帧)
% --- DEBUG 节流控制（1 Hz） ---
persistent S5STATE ASSIST_last_print_t DEBUG_last_print_t
if isfield(params, 'sim') && isfield(params.sim, 'step') && params.sim.step <= 1
    S5STATE            = [];
    ASSIST_last_print_t= -inf;
    DEBUG_last_print_t = -inf;
end
if isempty(S5STATE),             S5STATE = struct('ids',[], 'data',[]); end
if isempty(ASSIST_last_print_t), ASSIST_last_print_t = -inf;           end
if isempty(DEBUG_last_print_t),  DEBUG_last_print_t  = -inf;           end

% 是否本步启用详细打印
doDebug = false;
if isfinite(t_now) && (t_now - DEBUG_last_print_t >= 1.0 - 1e-9)
    DEBUG_last_print_t = t_now;
    doDebug = true;
    fprintf('\n================ [DEBUG t=%.2f s | Agent %d] ================\n', ...
        t_now, agent.id);
end

% --- 动态权重增强 ---
orig_lambda = getfieldwithdef(params.lvo,'lambda_pts',1.0);
if orig_lambda < 5.0
    lambda_pts = 10.0;
else
    lambda_pts = orig_lambda;
end

% 航线保持权重
omega      = getfieldwithdef(s5, 'route_w', 0.5);
omega_ct   = getfieldwithdef(s5,'route_ct_w',omega);
gamma      = getfieldwithdef(params.lvo,'gamma',2.5);

useCOLREGS = getfieldwithdef(s5,'useCOLREGS',true);
useSeaman  = getfieldwithdef(s5,'useSeamanship',true);
useFSM     = getfieldwithdef(s5,'useFSM',true);

% FSM 相关参数
fsm_sigma  = getfieldwithdef(s5,'fsm_sigma',10.0);
fsm_dt     = getfieldwithdef(s5,'fsm_dt',0.05);

% 横向偏移预测
route_ct_T     = getfieldwithdef(s5,'route_ct_horizon',5.0);
route_ct_scale = getfieldwithdef(s5,'route_ct_scale',2.0*ds_base*dp_factor);

% 3-DoF/GCCO 扩展参数
scanStep  = deg2rad(getfieldwithdef(s5,'scanStepDeg',1.0));
scanMax   = deg2rad(getfieldwithdef(s5,'scanMaxDeg',45.0));
predTime  = getfieldwithdef(s5,'predTime',20.0);
dt_chk    = getfieldwithdef(s5,'dtCheck',0.1);

Tr        = getfieldwithdef(s5,'yaw_T',  5.0);
Kdel      = getfieldwithdef(s5,'yaw_K',  1.0);
Tu        = getfieldwithdef(s5,'u_T',   2.0);
Dv        = getfieldwithdef(s5,'v_D',    0.8);
Alpha     = getfieldwithdef(s5,'uv_alpha',0.2);
Kp        = getfieldwithdef(s5,'Kp',     1.6);
Kd        = getfieldwithdef(s5,'Kd',     0.4);
deltaMax  = deg2rad(getfieldwithdef(s5,'delta_max_deg',35));
deltaRate = deg2rad(getfieldwithdef(s5,'delta_rate_deg',10));
aMax      = getfieldwithdef(s5,'a_max',  1);

perceptionDist = params.agent.perceptionDist;
T              = agent.timeHorizon;

% 速度（海速）与参考航向
Vsea   = max(1e-3, norm(agent.prefVelocity));
Cprev  = headingFromVelocity(agent);
toGoal = agent.goal - agent.position;
CLOS   = atan2(toGoal(2), toGoal(1));

% S0：本船信息打印
if doDebug
    fprintf(' [S0-OS] Pos=(%.1f,%.1f), Vel=(%.2f,%.2f), V=%.2f m/s\n', ...
        agent.position(1), agent.position(2), ...
        agent.velocity(1), agent.velocity(2), Vsea);
    fprintf(' [S0-OS] Cprev=%.1f deg, CLOS=%.1f deg\n', ...
        rad2deg(Cprev), rad2deg(CLOS));
    fprintf(' [S0-OS] Flags: useCOLREGS=%d, useFSM=%d, useSeamanship=%d\n', ...
        useCOLREGS, useFSM, useSeaman);
end

% 读取/初始化 FSM 状态
st = get_agent_state();

% === 计算当前对“起点→终点基线”的横向偏移 ===
if isprop(agent, 'pathOrigin')
    currentPathOrigin = agent.pathOrigin;
else
    currentPathOrigin = agent.position;
end

haveBase = all(isfinite(currentPathOrigin)) && ...
           (norm(agent.goal - currentPathOrigin) > 1e-6);
if haveBase
    base_vec = agent.goal - currentPathOrigin;
    base_dir = base_vec / norm(base_vec);
    n_base   = [-base_dir(2), base_dir(1)];   % 基线法向
    rel_now  = agent.position - currentPathOrigin;
    d_path_now = abs(dot(rel_now, n_base));
else
    n_base      = [0,0];
    d_path_now  = 0;
end

%% =========== S1 前端：扫描 CCO/BCO 与 3-DoF 扩展 ===========

obs = repmat(struct('Al',NaN,'Ar',NaN,'Bl',NaN,'Br',NaN, ...
                    'gAl',NaN,'gAr',NaN,'gBl',NaN,'gBr',NaN, ...
                    'scene',"OTHERS",'role','standon', ...
                    'colregsMask',@(C)true), ...
             numel(neighbors),1);

for j = 1:numel(neighbors)
    nb = neighbors(j);

    in_core = false(1, hdgGridN);
    in_bco  = false(1, hdgGridN);

    % 几何 CCO/BCO
    for k = 1:hdgGridN
        C = hdgGrid(k);
        v_scan = Vsea * [cos(C), sin(C)];
        RP     = nb.position - agent.position;
        RV     = nb.velocity - v_scan;

        [ds, dp] = compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
        [hitCore, ~]  = dcpaTcpaHit(RP, RV, ds, T, perceptionDist);
        [hitBroad, ~] = dcpaTcpaHit(RP, RV, dp, T, perceptionDist);

        in_core(k) = hitCore;
        in_bco(k)  = (~hitCore) && hitBroad;
    end

    [Al,Ar,Bl,Br] = local_find_intervals(hdgGrid, in_core, in_bco);
    obs(j).Al=Al; obs(j).Ar=Ar; obs(j).Bl=Bl; obs(j).Br=Br;

    % ---- 3-DoF 扩展：GCCO/GBCO ----
    RP0 = nb.position - agent.position;
    Vts = nb.velocity;

    [ds_ref, dp_ref] = compute_ds_dp_paper(RP0, Cprev, ds_base, dp_factor, useGW);

    X0_os = [];
    if isprop(agent, 'useDynamics') && agent.useDynamics && isprop(agent, 'dynState')
        s = agent.dynState;
        if isfield(s, 'u') && isfield(s, 'v') && isfield(s, 'r') && ...
           isfield(s, 'psi') && isfield(s, 'delta')
            X0_os = [s.u, s.v, s.r, s.psi, s.delta];
        end
    end

    if all(isfinite([Al,Bl]))
        obs(j).gBl = expand_one_side_3dof(Bl, Al, -1, dp_ref, ...
            agent, RP0, Cprev, Vsea, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAl = expand_one_side_3dof(Al, Bl, -1, ds_ref, ...
            agent, RP0, Cprev, Vsea, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end

    if all(isfinite([Ar,Br]))
        obs(j).gBr = expand_one_side_3dof(Br, Ar, +1, dp_ref, ...
            agent, RP0, Cprev, Vsea, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAr = expand_one_side_3dof(Ar, Br, +1, ds_ref, ...
            agent, RP0, Cprev, Vsea, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end
end

if doDebug && ~isempty(neighbors)
    fprintf(' [S1-FRONT] 完成 CCO/BCO + GCCO/GBCO 扫描, 邻船数 = %d\n', numel(neighbors));
end

%% =========== S2 前端：CR + 场景 + COLREGS ===========

[CR, riskMask] = compute_frontend_CR_paper(agent, neighbors, params, Cprev);

Nnb   = numel(neighbors);
useTS = true(1, Nnb);
thrCR = params.lvo.cr_threshold;

for j = 1:Nnb
    nb = neighbors(j);
    RP = nb.position - agent.position;
    [~, dpj] = compute_ds_dp_paper(RP, Cprev, ds_base, dp_factor, useGW);
    D = norm(RP);
    if (CR(j) < thrCR) && (D > 2*dpj)
        useTS(j) = false;  % Past & Clear
    end
end

if doDebug && ~isempty(neighbors)
    fprintf(' [S2-RISK] CR:');
    for j = 1:Nnb
        fprintf(' TS%d=%.2f', neighbors(j).id, CR(j));
    end
    fprintf('\n');
end

% 场景 + 角色 + COLREGS mask
for j=1:numel(neighbors)
    shouldReset = isFirstStep && (j == 1);
    [scene, role] = classify_scene_role_paper(agent, neighbors(j), Cprev, doDebug, shouldReset);
    obs(j).scene = scene;
    obs(j).role  = role;

    obs(j).colregsMask = @(C) true;
    if useCOLREGS
        obs(j).colregsMask = build_colregs_mask_paper(agent, neighbors(j), scene, role, Vsea);
    end
end

%% =========== S3 FSM 覆盖 ===========

fsmActive = false;

if useFSM && ~isempty(neighbors)
    % [DEBUG] 传入 doDebug，FSM 内部按流程图打印状态流转原因
    st = fsm_update(st, agent, neighbors, obs, CR, ...
                    fsm_sigma, deg2rad(5.0), Cprev, CLOS, ...
                    ds_base, dp_factor, useGW, fsm_dt, t_now, doDebug);

    % 先全局解除原始 COLREGS，再按 overrides 重新施加
    if isfield(st,'colregs_mode') && st.colregs_mode ~= 0
        for j2 = 1:numel(obs)
            obs(j2).colregsMask = @(C) true;
        end
    end

    if isfield(st,'overrides') && ~isempty(st.overrides)
        for k = 1:numel(st.overrides)
            idx = st.overrides(k).j;
            if idx>=1 && idx<=numel(obs)
                obs(idx).colregsMask = st.overrides(k).mask;
            end
        end
    end

    if isfield(st,'tsPhase')
        fsmActive = (st.colregs_mode ~= 0) || any(st.tsPhase > 0);
    end
end

% 打印 FSM / COLREGS 总览
if doDebug && ~isempty(neighbors)
    fprintf('---------------------------------------------------\n');
    if useFSM
        if fsmActive
            fprintf(' [S3-FSM] ACTIVE. colregs_mode=%d, tsPhase=[', st.colregs_mode);
            fprintf('%d ', st.tsPhase);
            fprintf(']\n');
        else
            fprintf(' [S3-FSM] ENABLED but NOT triggered (tsPhase 全 0, colregs_mode=0).\n');
        end
    else
        fprintf(' [S3-FSM] DISABLED. 使用纯 COLREGS + Seamanship.\n');
    end

    % COLREGS 场景+角色概览（无论是否进入 FSM 都打印）
    for jdbg = 1:numel(neighbors)
        fprintf('   TS %d: scene=%s, role=%s\n', ...
            neighbors(jdbg).id, char(obs(jdbg).scene), obs(jdbg).role);
    end
    fprintf('---------------------------------------------------\n');
end

%% =========== S4 后端：Usum = Umulti + Uroute ===========

% 检查是否处于观察期 (Phase 1)
isObserving = false;
if isfield(st,'tsPhase')
    for j = 1:numel(neighbors)
        if st.tsPhase(j) == 1
            isObserving = true;
            break;
        end
    end
end

% 动作锁定逻辑：如果处于观察期，且有上一帧决策，强制保持（跳过优化）
skipOptimization = false;
bestHdg   = Cprev;
bestCost  = inf;
bestU_multi      = NaN;
bestU_route_ang  = NaN;
bestU_route_path = NaN;
decisionSource   = 'OPT';   % 默认：来自优化器

if isObserving && isfield(st, 'prevObjHdg') && isfinite(st.prevObjHdg)
    isSafe = true;
    lockedHdg = st.prevObjHdg;

    for j = 1:numel(neighbors)
        if ~useTS(j), continue; end
        if any_is_in_gcco(lockedHdg, obs(j))
            isSafe = false;
            break;
        end
    end

    if isSafe
        bestHdg = lockedHdg;
        skipOptimization = true;
        decisionSource   = 'FSM_LOCK';
        if doDebug
            fprintf(' [S4-LOCK] Phase-1 观察锁定航向: %.1f deg (跳过优化)\n', ...
                rad2deg(bestHdg));
        end
    else
        if doDebug
            fprintf(' [S4-LOCK] Phase-1 观察锁定航向不安全，强制重新优化。\n');
        end
    end
end

% 如果没有锁定，或者锁定后发现不安全，则执行常规优化
if ~skipOptimization
    for k = 1:hdgGridN
        C = hdgGrid(k);

        % 1) 安全性过滤：GCCO + COLREGS
        infeasible = false;
        for j = 1:numel(neighbors)
            if ~useTS(j), continue; end
            if any_is_in_gcco(C, obs(j))
                infeasible = true; break;
            end
            if ~obs(j).colregsMask(C)
                infeasible = true; break;
            end
        end
        if infeasible, continue; end

        % 2) 多船碰撞代价 U_multi
        U_multi = 0;
        for j = 1:numel(neighbors)
            if ~useTS(j), continue; end

            if getfieldwithdef(s5,'useGBCOinPTS',true)
                Al = obs(j).gAl; Ar = obs(j).gAr;
                Bl = obs(j).gBl; Br = obs(j).gBr;
            else
                Al = obs(j).Al;  Ar = obs(j).Ar;
                Bl = obs(j).Bl;  Br = obs(j).Br;
            end

            Pts = local_pts_cost(C, Al, Ar, Bl, Br, gamma);
            if Pts > 0
                if riskMask(j)
                    U_multi = U_multi + lambda_pts * CR(j) * Pts;
                else
                    U_multi = U_multi + lambda_pts * Pts;
                end
            end
        end

        % 3) 航路保持代价 U_route = U_route_ang + U_route_path
        theta_d = abs(atan2(sin(C - CLOS), cos(C - CLOS)));
        U_route_ang  = omega * (theta_d / pi);

        if haveBase
            v_dir    = [cos(C), sin(C)];
            P_pred   = agent.position + Vsea * route_ct_T * v_dir;
            rel_pred = P_pred - currentPathOrigin;
            d_path_pred = abs(dot(rel_pred, n_base));

            delta_d  = max(0, d_path_pred - d_path_now);
            e_path_norm = min(delta_d / max(1e-3, route_ct_scale), 1.0);
        else
            e_path_norm = 0;
        end
        U_route_path = omega_ct * e_path_norm;

        U_route = U_route_ang + U_route_path;

        U_sum = U_multi + U_route;

        if U_sum < bestCost
            bestCost         = U_sum;
            bestHdg          = C;
            bestU_multi      = U_multi;
            bestU_route_ang  = U_route_ang;
            bestU_route_path = U_route_path;
            decisionSource   = 'OPT';
        end
    end
end

%% =========== Seamanship & 其他策略 ===========

% 若所有 CR 都很小，直接朝 LOS 走（覆盖前面选择）
if ~isempty(neighbors)
    if all(CR < thrCR) && isfinite(CLOS)
        if doDebug
            fprintf(' [S4-CR] 所有 CR < 阈值 (%.2f)，直接朝 LOS 航向。\n', thrCR);
        end
        bestHdg        = CLOS;
        decisionSource = 'CR_LOW_ROUTE';
    end
end

% Seamanship 平滑（仅在非锁定状态下运行）
if ~skipOptimization && useSeaman && isfield(st,'prevObjHdg') && isfinite(st.prevObjHdg)
    dC0 = angdiff_signed(st.prevObjHdg, st.prevStartHdg);  % 上一帧偏离
    dC1 = angdiff_signed(bestHdg,      st.prevStartHdg);   % 本次候选偏离

    if sign(dC0)*sign(dC1) > 0 && abs(dC1) < abs(dC0)
        if doDebug
            fprintf(' [S4-SEAMANSHIP] 平滑生效：保持上一帧航向 %.1f deg (候选=%.1f deg)\n', ...
                rad2deg(st.prevObjHdg), rad2deg(bestHdg));
        end
        bestHdg        = st.prevObjHdg;
        decisionSource = 'SEAMAN_KEEP';
    else
        if doDebug && useSeaman
            fprintf(' [S4-SEAMANSHIP] 已开启，但本步未触发平滑限制。\n');
        end
    end
elseif doDebug && useSeaman && skipOptimization
    fprintf(' [S4-SEAMANSHIP] 已开启，但由于 FSM 锁定，本步未使用平滑。\n');
elseif doDebug && ~useSeaman
    fprintf(' [S4-SEAMANSHIP] 已关闭 (useSeamanship=false)。\n');
end

%% =========== 最终决策打印 ===========

if doDebug
    % 决策来源说明
    switch decisionSource
        case 'FSM_LOCK'
            srcStr = 'FSM Phase-1 观察锁定航向';
        case 'CR_LOW_ROUTE'
            srcStr = '全部 CR 较小，直接指向 LOS 航向';
        case 'SEAMAN_KEEP'
            srcStr = 'Seamanship 平滑保持上一帧航向';
        otherwise
            srcStr = '优化器最小化 U_sum = U_multi + U_route';
    end

    fprintf(' [S4-DECISION] Source: %s\n', srcStr);

    if strcmp(decisionSource,'OPT') && isfinite(bestCost)
        fprintf(' [S4-DECISION] U_multi=%.3f, U_route_ang=%.3f, U_route_path=%.3f, U_sum=%.3f\n', ...
            bestU_multi, bestU_route_ang, bestU_route_path, bestCost);
    end

    fprintf(' [S4-DECISION] BestHdg: %.1f deg (Target CLOS: %.1f deg)\n', ...
        rad2deg(bestHdg), rad2deg(CLOS));
    fprintf('=================================================================\n');
end

%% =========== Action-5 Assist Rule 细节调试（0.5 s） ===========

if isfield(st,'colregs_mode') && st.colregs_mode == 3 && isfinite(t_now)
    if t_now - ASSIST_last_print_t >= 0.5 - 1e-9
        ASSIST_last_print_t = t_now;
        for j_dbg = 1:numel(neighbors)
            if isfield(st,'tsRVTS') && size(st.tsRVTS,1) >= j_dbg ...
                    && all(isfinite(st.tsRVTS(j_dbg,:))) ...
                    && size(st.tsRVo,1)   >= j_dbg ...
                    && all(isfinite(st.tsRVo(j_dbg,:)))

                ts_dbg   = neighbors(j_dbg);
                RVo_dbg  = st.tsRVo(j_dbg,:);
                RVTS_dbg = st.tsRVTS(j_dbg,:);

                v_c_dbg  = Vsea*[cos(bestHdg), sin(bestHdg)];
                RVOS_dbg = v_c_dbg - ts_dbg.velocity;

                nRVo  = norm(RVo_dbg);
                nRVTS = norm(RVTS_dbg);
                nRVOS = norm(RVOS_dbg);
                if nRVo > 1e-8 && nRVTS > 1e-8 && nRVOS > 1e-8
                    cosOS   = dot(RVo_dbg, RVOS_dbg)/(nRVo*nRVOS);
                    cosTS   = dot(RVo_dbg, RVTS_dbg)/(nRVo*nRVTS);
                    lhs_dbg = cosOS - cosTS;

                    ok_dbg  = assist_rule(agent, ts_dbg, bestHdg, RVo_dbg, RVTS_dbg);

                    hdg_RVo = atan2(RVo_dbg(2), RVo_dbg(1));
                    dPhi    = angdiff_signed(bestHdg, hdg_RVo);
                    if dPhi > deg2rad(1.0)
                        sideStr = 'LEFT';
                    elseif dPhi < -deg2rad(1.0)
                        sideStr = 'RIGHT';
                    else
                        sideStr = 'ALIGNED';
                    end

                    RP_dbg   = ts_dbg.position - agent.position;
                    Vrel_dbg = agent.velocity - ts_dbg.velocity;
                    [dcpa_dbg, ~, tstar_dbg] = compute_dcpa_tcpa(RP_dbg, Vrel_dbg);

                    fprintf(['[t=%.2f s] DBG-ASSIST: OS=%d, TS=%d, bestHdg=%.2f deg (%s of RVo), ', ...
                             'dPhi=%.2f deg, cosOS=%.3f, cosTS=%.3f, lhs=%.3f, ok=%d, ', ...
                             'D=%.2f, DCPA_now=%.2f, tstar_now=%.2f\n'], ...
                             t_now, agent.id, ts_dbg.id, rad2deg(bestHdg), sideStr, ...
                             rad2deg(dPhi), cosOS, cosTS, lhs_dbg, ok_dbg, ...
                             norm(RP_dbg), dcpa_dbg, tstar_dbg);
                end
            end
        end
    end
end

%% =========== debugInfo 记录 FSM 状态 ===========

if isprop(agent, 'debugInfo')
    if isfield(st, 'colregs_mode')
        agent.debugInfo.fsm_mode = st.colregs_mode;
    end
    if isfield(st, 'tsPhase')
        observe_indices = find(st.tsPhase == 1);
        observe_ids = [];
        for k_obs = 1:numel(observe_indices)
            idx = observe_indices(k_obs);
            if idx <= numel(neighbors) && idx <= numel(st.tsIds) && ...
                    st.tsIds(idx) == neighbors(idx).id
                observe_ids(end+1) = neighbors(idx).id; %#ok<AGROW>
            end
        end
        agent.debugInfo.fsm_observe_ids = observe_ids;
    end
end

%% =========== 输出新速度 & 写回 FSM 状态 ===========

newVelocity = Vsea * [cos(bestHdg), sin(bestHdg)];

st.prevObjHdg   = bestHdg;
st.prevStartHdg = Cprev;
set_agent_state(st);

%% ==================== 内部函数 ====================

    function st2 = get_agent_state()
        st2 = struct('prevObjHdg', NaN, 'prevStartHdg', NaN, ...
                     'tObsEnd', -inf, 'phase', "IDLE", 'overrides', [], ...
                     'tsIds', [], ...
                     'tsHeadings', [], 'tsHead0', [], ...
                     'tsPhase', [], 'tsObsLeft', [], ...
                     'tsRVo', zeros(0,2), 'tsRVopt', zeros(0,2), ...
                     'tsRVTS', zeros(0,2), ...
                     'colregs_mode', 0);
        k = find(S5STATE.ids==agent.id,1);
        if isempty(k)
            S5STATE.ids(end+1)   = agent.id;
            S5STATE.data{end+1}  = st2;
        else
            st2 = S5STATE.data{k};
            if ~isfield(st2,'tsIds'),         st2.tsIds = [];              end
            if ~isfield(st2,'tsHeadings'),    st2.tsHeadings = [];         end
            if ~isfield(st2,'tsHead0'),       st2.tsHead0 = [];            end
            if ~isfield(st2,'tsPhase'),       st2.tsPhase = [];            end
            if ~isfield(st2,'tsObsLeft'),     st2.tsObsLeft = [];          end
            if ~isfield(st2,'tsRVo'),         st2.tsRVo = zeros(0,2);      end
            if ~isfield(st2,'tsRVopt'),       st2.tsRVopt = zeros(0,2);    end
            if ~isfield(st2,'tsRVTS'),        st2.tsRVTS = zeros(0,2);     end
            if ~isfield(st2,'colregs_mode'),  st2.colregs_mode = 0;        end
        end
    end

    function set_agent_state(st2)
        k = find(S5STATE.ids==agent.id,1);
        if ~isempty(k)
            S5STATE.data{k} = st2;
        end
    end

    function [ds, dp] = compute_ds_dp_paper(RP, refHdg, ds0, dp_fac, useGoodwin_)
        if useGoodwin_
            B = abs(angleDiff(atan2(RP(2), RP(1)), refHdg));
            ds = ds_from_goodwin_piecewise(B, ds0);
        else
            ds = ds0;
        end
        dp = max(1e-6, dp_fac * ds);
    end

    function s = ds_from_goodwin_piecewise(B, ds0)
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

    function [CRval, riskMask] = compute_frontend_CR_paper(os, nbs, prm, osHdg)
        N = numel(nbs);
        CRval   = zeros(1,N);
        riskMask= false(1,N);
        w   = getfieldwithdef(prm.lvo, 'cr_w', ...
                [0.40, 0.367, 0.167, 0.033, 0.033]);
        thr = getfieldwithdef(prm.lvo, 'cr_threshold',0.5);

        for jj=1:N
            ts = nbs(jj);
            RP = ts.position - os.position;
            Vrel_vec = ts.velocity - os.velocity;
            [dcpa, ~, tstar] = compute_dcpa_tcpa(RP, Vrel_vec);
            [dsj, dpj] = compute_ds_dp_paper(RP, osHdg, ...
                                             prm.lvo.ds_base, prm.lvo.dp_factor, ...
                                             prm.lvo.useGoodwin);
            D = norm(RP);
            B = abs(angleDiff(atan2(RP(2),RP(1)), osHdg));
            K = max(1e-6, norm(os.velocity)) / max(1e-6, norm(ts.velocity));

            U1 = U_DCPA_paper(dcpa, dsj, dpj);
            U2 = U_TCPA_paper(tstar, dcpa, dsj, dpj, norm(Vrel_vec));
            U3 = U_D_paper(D, dsj, dpj);
            U4 = U_B_paper(B);
            U5 = U_K_sym(K);

            CRval(jj)    = w(1)*U1 + w(2)*U2 + w(3)*U3 + w(4)*U4 + w(5)*U5;
            riskMask(jj) = (CRval(jj) >= thr);
        end
    end

    function u = U_DCPA_paper(dcpa, ds, dp)
        if dcpa <= ds
            u = 1;
        elseif dcpa <= dp
            x = (dcpa - ds)/max(1e-6,(dp-ds));
            u = 0.5 - 0.5*sin(pi*(x - 0.5));
        else
            u = 0;
        end
    end

    function u = U_TCPA_paper(tcpa_signed, dcpa, ds, dp, Vrel)
        if Vrel < 1e-8, u = 0; return; end
        if tcpa_signed < 0, u = 0; return; end
        tcpa = tcpa_signed;

        if dcpa <= ds
            t1 = sqrt(max(ds^2 - dcpa^2,0))/Vrel;
        else
            t1 = (ds - dcpa)/Vrel; t1 = max(0,t1);
        end
        t2 = sqrt(max(dp^2 - min(dcpa,dp)^2,0))/Vrel;
        t2 = max(t2, t1 + 1e-6);

        if tcpa <= t1
            u = 1;
        elseif tcpa <= t2
            x = (tcpa - t1)/(t2 - t1); u = (1 - x)^2;
        else
            u = 0;
        end
    end

    function u = U_D_paper(D, ds, dp)
        if D <= ds
            u = 1;
        elseif D <= dp
            x = (D - ds)/max(1e-6,(dp-ds)); u = (1 - x);
        else
            u = 0;
        end
    end

    function u = U_B_paper(B)
        b = B - 19*pi/180;
        term_cos = cos(b);
        u = 0.5 * (term_cos + sqrt(440/289 + term_cos^2)) - 5/17;
        u = max(0, min(1, u));
    end

    function u = U_K_sym(K)
        K_safe = max(K, 1e-6);
        tau = log(K_safe);
        u = 1 - 1/(1 + exp(-tau));
        u = max(0, min(1, u));
    end

    %% ===== 新版：带历史记忆 + 滞回 + 角度滤波的场景判定 =====
    function [scene, role] = classify_scene_role_paper(os, ts, osHdg, doDbg, isReset)
        % classify_scene_role_paper — 带历史记忆 + 滞回 + 角度低通
        %
        % 输入:
        %   os    - 本船
        %   ts    - 他船
        %   osHdg - 本船参考航向（一般取当前 Cprev）
        %   doDbg - 是否打印 debug 信息
        %
        % 输出:
        %   scene ∈ {"HEAD_ON","CROSS_LEFT","CROSS_RIGHT","OVERTAKING","OVERTAKEN","OTHERS"}
        %   role  ∈ {'giveway','standon'}

        if nargin < 4
            doDbg = false;
        end
        if nargin < 5
            isReset = false; 
        end % 默认不重置

        %% ---------- 0. 历史状态 + 角度滤波器 ----------
        % 对每一对 (os.id, ts.id) 维护历史 scene/role + 滤波后的角度
        persistent HIST
        if isReset
        HIST = []; 
        end
        if isempty(HIST)
            HIST.osId       = [];
            HIST.tsId       = [];
            HIST.scene      = strings(0,1);  % string 类型
            HIST.role       = cell(0,1);     % char
            HIST.Q_filt     = [];
            HIST.dC_filt    = [];
            HIST.Qview_filt = [];
        end

        osId    = [];
        tsId    = [];
        idxHist = [];
        if isprop(os,'id') && isprop(ts,'id')
            osId    = os.id;
            tsId    = ts.id;
            idxHist = find(HIST.osId == osId & HIST.tsId == tsId, 1);
        end

        if isempty(idxHist)
            prevScene = "NONE";
            prevRole  = 'standon';
            Q_prev    = NaN;
            dC_prev   = NaN;
            Qv_prev   = NaN;
        else
            prevScene = HIST.scene(idxHist);
            prevRole  = HIST.role{idxHist};
            Q_prev    = HIST.Q_filt(idxHist);
            dC_prev   = HIST.dC_filt(idxHist);
            Qv_prev   = HIST.Qview_filt(idxHist);
        end

        %% ---------- 1. 几何量 + CPA ----------
        P_diff   = ts.position - os.position;   % OS -> TS
        V_os     = os.velocity;
        V_ts     = ts.velocity;
        V_ts_val = norm(V_ts);

        V_rel = V_os - V_ts;
        [dcpa0, tcpa0, tstar0] = compute_dcpa_tcpa(P_diff, V_rel);

        % Closing 判定：基于 CPA 时间窗
        T_close_max = 60;   % 默认 60 s 内认为“潜在会遇”
        if isprop(os,'timeHorizon') && isfinite(os.timeHorizon) && os.timeHorizon > 0
            T_close_max = max(10, os.timeHorizon);
        end
        is_closing = (tstar0 > 0) && (tstar0 < T_close_max);

        %% ---------- 2. 航向 / 方位（原始角） ----------
        if V_ts_val > 1e-6
            C_ts = atan2(V_ts(2), V_ts(1));
        else
            % 低速/几乎静止时，用本船航向当作“意图方向”
            C_ts = osHdg;
        end

        B_ts_from_os = atan2(P_diff(2), P_diff(1));          % TS 真方位
        Q_raw        = angdiff_signed(B_ts_from_os, osHdg);  % 舷角（>0 左舷, <0 右舷）
        dC_raw       = angdiff_signed(C_ts, osHdg);          % 航向差

        B_os_from_ts = atan2(-P_diff(2), -P_diff(1));
        Qview_raw    = angdiff_signed(B_os_from_ts, C_ts);   % OS 在 TS 视角的舷角

        %% ---------- 3. 角度低通滤波 ----------
        % 使用 α=0.7：当前 70% + 历史 30%，响应更快，抖动更小
        alpha = 0.7;

        if ~isfinite(Q_prev),   Q_prev   = Q_raw;     end
        if ~isfinite(dC_prev),  dC_prev  = dC_raw;    end
        if ~isfinite(Qv_prev),  Qv_prev  = Qview_raw; end

        Q   = wrapToPi_loc(Q_prev   + alpha * angdiff_signed(Q_raw,     Q_prev));
        dC  = wrapToPi_loc(dC_prev  + alpha * angdiff_signed(dC_raw,    dC_prev));
        Qvw = wrapToPi_loc(Qv_prev  + alpha * angdiff_signed(Qview_raw, Qv_prev));

        dC_abs = abs(dC);

        %% ---------- 4. 阈值常量 ----------
        DEG_5     = deg2rad(5);
        DEG_15    = deg2rad(15);
        DEG_20    = deg2rad(20);
        DEG_105   = deg2rad(105);
        DEG_112_5 = deg2rad(112.5);
        DEG_135   = deg2rad(135);
        DEG_140   = deg2rad(140);
        DEG_150   = deg2rad(150);

        scene_raw = "OTHERS";
        role_raw  = 'standon';

        %% ---------- 5. 静止 / 低速目标：作为 OTHERS 交给 VO/CR ----------
        % 静止/低速目标不参与 COLREGS/FSM，只走 VO+CR
        V_ts_min = 0.2;
        if V_ts_val < V_ts_min
            scene = "OTHERS";
            role  = 'standon';

            if doDbg
                fprintf('   > TS %d: [STATIC] V_ts=%.2f m/s -> treat as OTHERS (no COLREGS/FSM)\n', ...
                    ts.id, V_ts_val);
            end

            % 写回历史
            if ~isempty(osId)
                if isempty(idxHist)
                    HIST.osId(end+1)       = osId;
                    HIST.tsId(end+1)       = tsId;
                    HIST.scene(end+1)      = scene;
                    HIST.role{end+1}       = role;
                    HIST.Q_filt(end+1)     = Q;
                    HIST.dC_filt(end+1)    = dC;
                    HIST.Qview_filt(end+1) = Qvw;
                else
                    HIST.scene(idxHist)      = scene;
                    HIST.role{idxHist}       = role;
                    HIST.Q_filt(idxHist)     = Q;
                    HIST.dC_filt(idxHist)    = dC;
                    HIST.Qview_filt(idxHist) = Qvw;
                end
            end
            return;
        end

        %% ---------- 6. 原始场景分类 (使用滤波后的 Q, dC, Qvw) ----------
        % 6.1 被追越：TS 在本船艉后扇区
        if is_closing && (abs(Q) > DEG_112_5)
            scene_raw = "OVERTAKEN";
            role_raw  = 'standon';
            if doDbg
                fprintf('   > TS %d: [OVERTAKEN-raw] Q_filt=%.1f°\n', ts.id, rad2deg(Q));
            end

        % 6.2 追越：本船在 TS 艉后扇区
        elseif is_closing && (abs(Qvw) > DEG_112_5)
            scene_raw = "OVERTAKING";
            role_raw  = 'giveway';
            if doDbg
                fprintf('   > TS %d: [OVERTAKING-raw] Qview_filt=%.1f°\n', ts.id, rad2deg(Qvw));
            end

        % 6.3 对遇：Q ≈ 0 且 航向差接近 180°
        elseif is_closing && (abs(Q) <= DEG_15) && (dC_abs >= DEG_150)
            scene_raw = "HEAD_ON";
            role_raw  = 'giveway';
            if doDbg
                fprintf('   > TS %d: [HEAD_ON-raw] dC_filt=%.1f°, Q_filt=%.1f°\n', ...
                    ts.id, rad2deg(dC_abs), rad2deg(Q));
            end

        else
            % 6.4 交叉：几何上不平行，也不到完全对遇
            if is_closing && (dC_abs >= DEG_15) && (dC_abs <= DEG_150) ...
                          && (abs(Q) <= DEG_112_5)

                % 用 CPA + Goodwin ds 再过滤一层
                [ds_ref, ~] = compute_ds_dp_paper(P_diff, osHdg, ds_base, dp_factor, useGW);
                T_cross_max = T_close_max;

                if (tcpa0 > 0) && (tcpa0 < T_cross_max) && (dcpa0 < 1.2 * ds_ref)
                    if Q < 0
                        scene_raw = "CROSS_RIGHT";
                        role_raw  = 'giveway';
                    else
                        scene_raw = "CROSS_LEFT";
                        role_raw  = 'standon';
                    end
                    if doDbg
                        fprintf(['   > TS %d: [%s-raw] dC_filt=%.1f°, Q_filt=%.1f°, ', ...
                                 'DCPA=%.1f, TCPA=%.1f\n'], ...
                                 ts.id, char(scene_raw), ...
                                 rad2deg(dC), rad2deg(Q), dcpa0, tcpa0);
                    end
                else
                    if doDbg
                        fprintf(['   > TS %d: CROSS geom but CPA safe: ', ...
                                 'dC_filt=%.1f°, Q_filt=%.1f°, DCPA=%.1f, TCPA=%.1f\n'], ...
                                 ts.id, rad2deg(dC), rad2deg(Q), dcpa0, tcpa0);
                    end
                end
            else
                if doDbg
                    fprintf('   > TS %d: [OTHERS-raw] closing=%d, dC_filt=%.1f°, Q_filt=%.1f°\n', ...
                        ts.id, is_closing, rad2deg(dC_abs), rad2deg(Q));
                end
            end
        end

        %% ---------- 7. 应用滞回 ----------
        [scene, role] = apply_hysteresis(prevScene, prevRole, ...
                                         scene_raw, role_raw, ...
                                         P_diff, V_os, V_ts, osHdg, ...
                                         dcpa0, tcpa0, tstar0, is_closing);

        %% ---------- 8. 写回历史状态 ----------
        if ~isempty(osId)
            if isempty(idxHist)
                HIST.osId(end+1)       = osId;
                HIST.tsId(end+1)       = tsId;
                HIST.scene(end+1)      = scene;
                HIST.role{end+1}       = role;
                HIST.Q_filt(end+1)     = Q;
                HIST.dC_filt(end+1)    = dC;
                HIST.Qview_filt(end+1) = Qvw;
            else
                HIST.scene(idxHist)      = scene;
                HIST.role{idxHist}       = role;
                HIST.Q_filt(idxHist)     = Q;
                HIST.dC_filt(idxHist)    = dC;
                HIST.Qview_filt(idxHist) = Qvw;
            end
        end

        if doDbg
            fprintf('   > TS %d: [FINAL] scene=%s, role=%s\n', ...
                ts.id, char(scene), role);
        end

        %% ---------- 内部：滞回策略函数 ----------
        function [scene_out, role_out] = apply_hysteresis(prevScene_, prevRole_, ...
                                                          scene_now, role_now, ...
                                                          P_diff_, V_os_, V_ts_, osHdg_, ...
                                                          dcpa_, tcpa_, tstar_, is_closing_)
            scene_out = scene_now;
            role_out  = role_now;

            if prevScene_ == "NONE"
                return;
            end

            C_ts_ = atan2(V_ts_(2), V_ts_(1));
            B_ts_from_os_ = atan2(P_diff_(2), P_diff_(1));
            Qh      = angdiff_signed(B_ts_from_os_, osHdg_);
            dCh     = angdiff_signed(C_ts_,        osHdg_);
            dCh_abs = abs(dCh);

            B_os_from_ts_ = atan2(-P_diff_(2), -P_diff_(1));
            Qview_h       = angdiff_signed(B_os_from_ts_, C_ts_);

            % 对应当前 ds，用于 DCPA 相关的“危险程度”判断
            [ds_h, ~] = compute_ds_dp_paper(P_diff_, osHdg_, ds_base, dp_factor, useGW);

            % ---- HEAD_ON 滞回（简化版）----
            if prevScene_ == "HEAD_ON" && scene_now ~= "HEAD_ON"
                if is_closing_ && (abs(Qh) <= DEG_20) && (dCh_abs >= DEG_140)
                    scene_out = "HEAD_ON";
                    role_out  = 'giveway';
                    return;
                end
            end

            % ---- OVERTAKEN 滞回 ----
            if prevScene_ == "OVERTAKEN" && scene_now ~= "OVERTAKEN"
                if is_closing_ && (abs(Qh) > DEG_105)
                    scene_out = "OVERTAKEN";
                    role_out  = 'standon';
                    return;
                end
            end

            % ---- OVERTAKING 滞回 ----
            if prevScene_ == "OVERTAKING" && scene_now ~= "OVERTAKING"
                if is_closing_ && (abs(Qview_h) > DEG_105)
                    scene_out = "OVERTAKING";
                    role_out  = 'giveway';
                    return;
                end
            end

            % ---- CROSSING 滞回：收紧条件 ----
            if (prevScene_ == "CROSS_LEFT" || prevScene_ == "CROSS_RIGHT") ...
                    && ~(scene_now == "CROSS_LEFT" || scene_now == "CROSS_RIGHT")

                % “仍然像交叉”的条件：
                % - 还在 closing；
                % - 航向差在 [20°, 135°]；
                % - 舷角在 [5°, 120°]；
                % - DCPA 不大于 ~1.5 ds；
                still_cross_like = is_closing_ ...
                    && (dCh_abs > deg2rad(20)) && (dCh_abs < deg2rad(135)) ...
                    && (abs(Qh)   > deg2rad(5)) && (abs(Qh)   < deg2rad(120)) ...
                    && (dcpa_     < 1.5 * ds_h);

                if still_cross_like
                    scene_out = prevScene_;
                    role_out  = prevRole_;
                    return;
                end
            end
        end

    end

    function mask = build_colregs_mask_paper(os, ts, scene, role, Vsea_loc2)
        mask = @(C) true;
        if ~ismember(scene, ["HEAD_ON","CROSS_RIGHT"]) || ~strcmpi(role,'giveway')
            return;
        end
        mask = @(C) colregs_ok(os, ts, C);

        function ok = colregs_ok(os_, ts_, Ccand)
            RP = ts_.position - os_.position;
            v_cand = Vsea_loc2*[cos(Ccand), sin(Ccand)];
            RV = v_cand - ts_.velocity;
            z = RP(1)*RV(2) - RP(2)*RV(1);
            ok = (z <= -1e-9);  % 右让左行
        end
    end

    function stx = fsm_update(stx, os, nbs, obss, CR_list, ...
                               sigma, deg5_, osHdg, CLOS_loc, ...
                               ds0, dp_fac, useGW_, fsm_dt_, t_now_, doDbg)
        N = numel(nbs);
        new_overrides = [];
        new_mode      = 0;

        % 结构体长度初始化
        if ~isfield(stx,'tsIds') || numel(stx.tsIds) < N
            tmp = zeros(1,N);
            if isfield(stx,'tsIds'), tmp(1:numel(stx.tsIds))=stx.tsIds; end
            stx.tsIds=tmp;
        end
        if ~isfield(stx,'tsHeadings') || numel(stx.tsHeadings) < N
            tmp = nan(1,N);
            if isfield(stx,'tsHeadings'), tmp(1:numel(stx.tsHeadings))=stx.tsHeadings; end
            stx.tsHeadings=tmp;
        end
        if ~isfield(stx,'tsHead0') || numel(stx.tsHead0) < N
            tmp = nan(1,N);
            if isfield(stx,'tsHead0'), tmp(1:numel(stx.tsHead0))=stx.tsHead0; end
            stx.tsHead0=tmp;
        end
        if ~isfield(stx,'tsPhase') || numel(stx.tsPhase) < N
            tmp = zeros(1,N);
            if isfield(stx,'tsPhase'), tmp(1:numel(stx.tsPhase))=stx.tsPhase; end
            stx.tsPhase=tmp;
        end
        if ~isfield(stx,'tsObsLeft') || numel(stx.tsObsLeft) < N
            tmp = zeros(1,N);
            if isfield(stx,'tsObsLeft'), tmp(1:numel(stx.tsObsLeft))=stx.tsObsLeft; end
            stx.tsObsLeft=tmp;
        end
        if ~isfield(stx,'tsRVo') || size(stx.tsRVo,1) < N
            newR = nan(N,2);
            if isfield(stx,'tsRVo'), newR(1:size(stx.tsRVo,1),:)=stx.tsRVo; end
            stx.tsRVo=newR;
        end
        if ~isfield(stx,'tsRVopt') || size(stx.tsRVopt,1) < N
            newR = nan(N,2);
            if isfield(stx,'tsRVopt'), newR(1:size(stx.tsRVopt,1),:)=stx.tsRVopt; end
            stx.tsRVopt=newR;
        end
        if ~isfield(stx,'tsRVTS') || size(stx.tsRVTS,1) < N
            newR = nan(N,2);
            if isfield(stx,'tsRVTS'), newR(1:size(stx.tsRVTS,1),:)=stx.tsRVTS; end
            stx.tsRVTS=newR;
        end
        if ~isfield(stx,'colregs_mode'), stx.colregs_mode = 0; end

        Vsea_loc = Vsea; %#ok<NASGU>

        persistent lastPrintPhase
        if isempty(lastPrintPhase) || numel(lastPrintPhase) ~= N
            lastPrintPhase = -ones(1,N);
        end

        if doDbg
            fprintf(' [S3-FSM] 进入 FSM 更新 (N=%d)\n', N);
        end

        for jj = 1:N
            ts = nbs(jj);
            tsSpeed = norm(ts.velocity);

            if stx.tsIds(jj) ~= ts.id
                stx.tsIds(jj)     = ts.id;
                stx.tsPhase(jj)   = 0;
                stx.tsHead0(jj)   = NaN;
                stx.tsObsLeft(jj) = 0;
                stx.tsRVo(jj,:)   = [NaN, NaN];
                stx.tsRVopt(jj,:) = [NaN, NaN];
                stx.tsRVTS(jj,:)  = [NaN, NaN];
                stx.tsHeadings(jj)= NaN;
                lastPrintPhase(jj)= -1;
            end

            RP = ts.position - os.position;
            [~, dpj] = compute_ds_dp_paper(RP, osHdg, ds0, dp_fac, useGW_);
            D = norm(RP);

            Cts_now  = atan2(ts.velocity(2), ts.velocity(1));

            if isfinite(stx.tsHeadings(jj))
                Cts_prev = stx.tsHeadings(jj);
            else
                Cts_prev = Cts_now;
            end

            % 若已在 Phase1/2，则强制视作 giveway
            currentPhase = stx.tsPhase(jj);
            effective_role = obss(jj).role;
            if currentPhase >= 1
                effective_role = 'giveway';
            end

            RV_osTs = os.velocity - ts.velocity;
            RV_tsOs = ts.velocity - os.velocity;

            if ~all(isfinite(stx.tsRVo(jj,:)))
                stx.tsRVo(jj,:) = RV_osTs;
            end

            [~, tcpa_val, tstar] = compute_dcpa_tcpa(RP, RV_tsOs);

            isAvoidingNow = false;
            if isfield(stx,'prevObjHdg') && isfinite(stx.prevObjHdg) && isfinite(CLOS_loc)
                isAvoidingNow = abs(angdiff_signed(stx.prevObjHdg, CLOS_loc)) > deg2rad(1.0);
            end

            if tsSpeed > 0.1
                if isAvoidingNow && ~isfinite(stx.tsHead0(jj))
                    stx.tsHead0(jj) = Cts_prev;
                end
                if ~isfinite(stx.tsHead0(jj)) && (D <= dpj)
                    stx.tsHead0(jj) = Cts_prev;
                end
            end

            if isfinite(stx.tsHead0(jj))
                dCTS_total = abs(angdiff_signed(Cts_now, stx.tsHead0(jj)));
            else
                dCTS_total = 0;
            end

            scene = obss(jj).scene; %#ok<NASGU>

            % ========= Give-way 流程 =========
            if strcmpi(effective_role,'giveway')
                if currentPhase == 0
                    if dCTS_total > deg5_
                        RVo = RV_osTs;
                        RVopt = RV_osTs;
                        if isfield(stx,'prevObjHdg') && isfinite(stx.prevObjHdg)
                            vopt = Vsea * [cos(stx.prevObjHdg), sin(stx.prevObjHdg)];
                            RVopt = vopt - ts.velocity;
                        end
                        c1 = dot(RVo, RVopt);
                        c2 = dot(RVo, RVo);

                        if (c1 - c2) < 0
                            stx.tsPhase(jj)   = 1;
                            stx.tsObsLeft(jj) = min(5.0, max(0, tstar)/max(1e-6, sigma));
                            stx.tsRVo(jj,:)   = RVo;
                            stx.tsRVopt(jj,:) = RVopt;
                            if doDbg
                                fprintf('   > FSM [TS %d]: P0->P1 (OBSERVE). Trigger: Resistance (c1-c2=%.3f < 0)\n', ...
                                    ts.id, c1-c2);
                            end
                        else
                            if doDbg
                                fprintf('   > FSM [TS %d]: 保持 P0. Resistance OK (c1-c2=%.3f > 0)\n', ...
                                    ts.id, c1-c2);
                            end
                        end
                    else
                        if doDbg
                            fprintf('   > FSM [TS %d]: 保持 P0. Target hdg 稳定 (dH=%.2f deg < 5.0)\n', ...
                                ts.id, rad2deg(dCTS_total));
                        end
                    end

                elseif currentPhase == 1
                    stx.tsObsLeft(jj) = stx.tsObsLeft(jj) - fsm_dt_;
                    if doDbg
                        fprintf('   > FSM [TS %d]: P1 (OBSERVE). Time Left: %.2fs. ', ...
                            ts.id, stx.tsObsLeft(jj));
                    end

                    force_exit = (tcpa_val < 0) || (D > dpj * 1.5);
                    if stx.tsObsLeft(jj) <= 0 || force_exit
                        if force_exit
                            stx.tsPhase(jj) = 0;
                            stx.tsObsLeft(jj) = 0;
                            if doDbg
                                fprintf('EXIT -> P0 (Safe/Timeout).\n');
                            end
                        else
                            RVo = stx.tsRVo(jj,:);
                            RVopt = stx.tsRVopt(jj,:);
                            RVpre = RV_osTs;

                            z1 = RVo(1)*RVpre(2) - RVo(2)*RVpre(1);
                            z2 = RVo(1)*RVopt(2) - RVo(2)*RVopt(1);
                            isC1 = (z1*z2 < 0) || (abs(z1*z2) < 1e-2);

                            if isC1
                                stx.tsPhase(jj) = 2;
                                lastPrintPhase(jj)=1;
                                new_mode = max(new_mode,1);
                                new_overrides(end+1).j = jj;
                                new_overrides(end).mask = @(C) flip_mask(os, ts, C, 1);
                                if doDbg
                                    fprintf('TRANSITION -> P2 (ACTION). Reason: Z-Check (z1*z2=%.4f <= 0)\n', ...
                                        z1*z2);
                                end
                            else
                                stx.tsPhase(jj) = 0;
                                if doDbg
                                    fprintf('TRANSITION -> P0. Reason: Z-Check Passed (z1*z2=%.4f > 0)\n', ...
                                        z1*z2);
                                end
                            end
                        end
                    else
                        if doDbg
                            fprintf('Continuing Observation.\n');
                        end
                    end

                elseif currentPhase == 2
                    is_safe = (tcpa_val <= 0) || (CR_list(jj) < 0.2);
                    if is_safe
                        stx.tsPhase(jj) = 0;
                        if doDbg
                            fprintf('   > FSM [TS %d]: P2->P0. Reason: Safe (TCPA<0 or CR<0.2)\n', ts.id);
                        end
                    else
                        new_mode = max(new_mode,1);
                        new_overrides(end+1).j = jj;
                        new_overrides(end).mask = @(C) flip_mask(os, ts, C, 1);
                        if doDbg
                            fprintf('   > FSM [TS %d]: P2 (ACTION). Mask: Flip Right. CR=%.2f\n', ...
                                ts.id, CR_list(jj));
                        end
                    end
                end
            end

            % ========= Stand-on 流程 =========
            if strcmpi(effective_role,'standon') && currentPhase == 0 && (D <= dpj)
                in_gcco = any_is_in_gcco(osHdg, obss(jj));

                if dCTS_total < deg5_
                    if strcmpi(scene,'CROSS_LEFT')
                        new_mode = max(new_mode, 2);
                        new_overrides(end+1).j = jj;
                        new_overrides(end).mask = @(C) flip_mask(os, ts, C, -1);
                        if doDbg
                            fprintf('   > FSM [TS %d]: Stand-on Action. Rule 17(a)(ii) -> Flip Left.\n', ...
                                ts.id);
                        end
                    elseif strcmpi(scene,'OVERTAKEN')
                        new_mode = max(new_mode, 2);
                        new_overrides(end+1).j = jj;
                        new_overrides(end).mask = @(C) mask_overtaken(os, ts, C, osHdg);
                        if doDbg
                            fprintf('   > FSM [TS %d]: Stand-on Action. Maintaining Course.\n', ts.id);
                        end
                    end
                elseif dCTS_total > deg5_ && in_gcco
                    % Action 5: Assist Rule
                    new_mode = max(new_mode, 3);
                    RVoA = stx.tsRVo(jj,:);
                    if ~all(isfinite(RVoA)), RVoA = RV_osTs; end
                    RVTSA = RV_osTs;
                    new_overrides(end+1).j = jj;
                    new_overrides(end).mask = @(C) assist_rule(os, ts, C, RVoA, RVTSA);
                    if doDbg
                        fprintf('   > FSM [TS %d]: Stand-on Assist. Rule 17(c) + Math Check.\n', ts.id);
                    end
                end
            end

            if tsSpeed > 0.1
                stx.tsHeadings(jj) = Cts_now;
            end
        end

        stx.colregs_mode = new_mode;
        stx.overrides    = new_overrides;

        if doDbg
            fprintf(' [S3-FSM] 退出 FSM 更新: colregs_mode=%d, overrides=%d\n', ...
                stx.colregs_mode, numel(stx.overrides));
        end
    end

    function ok = flip_mask(os, ts, C, sgn)
        RP = ts.position - os.position;
        v_c = Vsea*[cos(C), sin(C)];
        RV  = v_c - ts.velocity;
        z   = RP(1)*RV(2) - RP(2)*RV(1);
        if sgn>0
            ok = (z >= 0);
        else
            ok = (z <= 0);
        end
    end

    function ok = mask_overtaken(os, ts, C, osHdg_)
        Q  = atan2(ts.position(2)-os.position(2), ...
                   ts.position(1)-os.position(1)) - osHdg_;
        RP = ts.position - os.position;
        v_c = Vsea*[cos(C), sin(C)];
        RV  = v_c - ts.velocity;
        z   = RP(1)*RV(2) - RP(2)*RV(1);
        if Q > 0
            ok = (z <= 0);
        else
            ok = (z >= 0);
        end
    end

    function ok = assist_rule(os, ts, C, RVo, RVTS)
        % Action 5 核心判定：数学 + Rule 17(c)
        Vsea_val = norm(os.velocity);
        if Vsea_val < 0.1, Vsea_val = os.maxSpeed; end

        v_c  = Vsea_val*[cos(C), sin(C)];
        RVOS = v_c - ts.velocity;

        lhs = dot(RVo, RVOS)/max(1e-9,norm(RVo)*norm(RVOS)) ...
            - dot(RVo, RVTS)/max(1e-9,norm(RVo)*norm(RVTS));
        math_ok = (lhs < 0);

        % Rule 17(c)：不向左避让本船左舷的船
        RP = ts.position - os.position;
        osHdg_curr = atan2(os.velocity(2), os.velocity(1));
        B = atan2(RP(2), RP(1)) - osHdg_curr;
        B = atan2(sin(B), cos(B));
        is_on_port = (B > 0);

        dHdg = C - osHdg_curr;
        dHdg = atan2(sin(dHdg), cos(dHdg));
        is_turning_left = (dHdg > deg2rad(5.0));

        if is_on_port && is_turning_left
            ok = false;  % 违反 Rule 17(c)
        else
            ok = math_ok;
        end
    end

end  % ====== computeVO_Step5I 结束 ======


%% ==================== 外部小工具函数 ====================

function val = getfieldwithdef(s, name, def)
if isfield(s,name), val=s.(name); else, val=def; end
end

function [dcpa, tcpa, tstar] = compute_dcpa_tcpa(RP, RV)
a = dot(RV,RV);
if a < 1e-10
    tstar = inf; tcpa = inf; dcpa = norm(RP); return;
end
tstar = -dot(RP,RV)/a;
tcpa  = max(0, tstar);
dcpa  = norm(RP + RV*tcpa);
end

function [hit, ttc] = dcpaTcpaHit(RP, RV, radius, timeHorizon, perceptionDist)
a = dot(RV, RV);
r0 = norm(RP);
if r0 > perceptionDist, hit = false; ttc = inf; return; end
if a < 1e-8
    if r0 <= 1.2*radius, hit = true; ttc = 0; else, hit = false; ttc = inf; end
    return;
end
dotrv = dot(RP, RV);
approaching = (dotrv < 0);
if ~approaching, hit = false; ttc = inf; return; end
tcpa0 = - dotrv / a;
if tcpa0 > timeHorizon, hit = false; ttc = inf; return; end
tcpa = max(0, tcpa0);
dcpa = norm(RP + RV * tcpa);
hit = (dcpa <= radius);
if hit, ttc = tcpa; else, ttc = inf; end
end

function hdg = headingFromVelocity(agent)
if isprop(agent, 'useDynamics') && agent.useDynamics && isprop(agent, 'dynState')
    if isfield(agent.dynState, 'psi') && isfinite(agent.dynState.psi)
        hdg = agent.dynState.psi; return;
    end
end
v = agent.velocity;
if norm(v) < 1e-6
    g = agent.goal - agent.position;
    hdg = atan2(g(2), g(1)); if ~isfinite(hdg), hdg=0; end
else
    hdg = atan2(v(2), v(1)); if ~isfinite(hdg), hdg=0; end
end
end

function d = angleDiff(a,b),    d = atan2(sin(a-b), cos(a-b)); end
function x = wrapToPi_loc(a),   x = atan2(sin(a), cos(a));     end
function d = angdiff_signed(a,b), d = atan2(sin(a-b), cos(a-b)); end

function [Al,Ar,Bl,Br] = local_find_intervals(hdgGrid, in_core, in_bco)
Al=NaN; Ar=NaN; Bl=NaN; Br=NaN;
N = numel(hdgGrid);
idxCore = find(in_core);
if isempty(idxCore)
    [L1,R1] = longest_run_circ(in_bco);
    mask12  = idx_range_mask_circ(N,L1,R1);
    [L2,R2] = longest_run_circ(in_bco & ~mask12);
    if ~isempty(L1), Al=hdgGrid(1+mod(L1-1,N)); Bl=hdgGrid(1+mod(R1-1,N)); end
    if ~isempty(L2), Ar=hdgGrid(1+mod(L2-1,N)); Br=hdgGrid(1+mod(R2-1,N)); end
    return
end
[Lc, Rc] = longest_run_circ(in_core);
[L1,R1] = nearest_run_to_circ(Lc, in_bco, 'left');
[L2,R2] = nearest_run_to_circ(Rc, in_bco, 'right');
if ~isempty(L1), Al=hdgGrid(1+mod(R1-1,N)); Bl=hdgGrid(1+mod(L1-1,N)); end
if ~isempty(L2), Ar=hdgGrid(1+mod(L2-1,N)); Br=hdgGrid(1+mod(R2-1,N)); end
end

function [L,R] = longest_run_circ(bvec)
N=numel(bvec);
bb=[bvec(:).', bvec(:).'];
d=diff([false, bb, false]);
starts=find(d==1);
ends=find(d==-1)-1;
if isempty(starts), L=[]; R=[]; return; end
[~,ix]=max(ends-starts+1);
Lc=starts(ix); Rc=ends(ix);
L=mod(Lc-1,N)+1;
R=mod(Rc-1,N)+1;
if R<L, R=R+N; end
end

function mask = idx_range_mask_circ(N,L,R)
mask=false(1,N);
if isempty(L) || isempty(R), return; end
if R<=N
    mask(L:R)=true;
else
    mask(L:N)=true;
    mask(1:mod(R-1,N)+1)=true;
end
end

function [L,R] = nearest_run_to_circ(edgeIndex, bvec, side)
N=numel(bvec);
bb=[bvec(:).', bvec(:).'];
d=diff([false, bb, false]);
starts=find(d==1);
ends=find(d==-1)-1;
L=[]; R=[];
if isempty(starts), return; end
if strcmp(side,'left')
    cand = find(ends < edgeIndex | ends > edgeIndex+N);
    if isempty(cand), return; end
    [~,ix] = min(mod(edgeIndex - ends(cand), N));
else
    cand = find(starts > edgeIndex & starts <= edgeIndex+N);
    if isempty(cand), return; end
    [~,ix] = min(mod(starts(cand) - edgeIndex, N));
end
if isempty(cand), return; end
ix = min(ix, numel(cand));
Lc=starts(cand(ix)); Rc=ends(cand(ix));
L=mod(Lc-1,N)+1;
R=mod(Rc-1,N)+1;
if R<L, R=R+N; end
end

function P = local_pts_cost(C, Al, Ar, Bl, Br, gamma_)
P = 0;
if all(isfinite([Al,Bl])) && is_between_circ(C, Al, Bl)
    denom = max(1e-6, wrap_to_pi_mag(Bl - Al));
    P = ((wrap_to_pi_mag(Bl - C))/denom)^gamma_;
elseif all(isfinite([Ar,Br])) && is_between_circ(C, Ar, Br)
    denom = max(1e-6, wrap_to_pi_mag(Br - Ar));
    P = ((wrap_to_pi_mag(Br - C))/denom)^gamma_;
end
P = max(0,P);
end

function x = wrap_to_pi_mag(a)
x = atan2(sin(a), cos(a));
if x<0, x = x + 2*pi*(x<-pi); end
x = abs(x);
end

function tf = is_between_circ(C, L, R)
dLR = atan2(sin(R-L), cos(R-L));
dLC = atan2(sin(C-L), cos(C-L));
dCR = atan2(sin(R-C), cos(R-C));
dLR = mod(dLR + 2*pi, 2*pi);
dLC = mod(dLC + 2*pi, 2*pi);
dCR = mod(dCR + 2*pi, 2*pi);
tf = all( (dLC > 0) & (dCR > 0) & ((dLC + dCR) < (dLR + 1e-12)) );
end

function tf = any_is_in_gcco(C, ob)
tf = false;
if all(isfinite([ob.gAl, ob.gBl])) && is_between_circ(C, ob.gAl, ob.gBl), tf = true; return; end
if all(isfinite([ob.gAr, ob.gBr])) && is_between_circ(C, ob.gAr, ob.gBr), tf = true; return; end
end

function gNear = expand_one_side_3dof(nearB, farB, dirSign, radius, ...
    agent, RP0, hdg0, u_ref, Vts, X0_os, ...
    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
    t_pred, dt, scanStep, scanMax)
gNear = nearB;
stepMax = ceil(scanMax / scanStep);
if ~isempty(X0_os) && numel(X0_os) == 5
    X0 = X0_os;
else
    u0 = max(1e-3, norm(agent.velocity));
    psi0 = hdg0; v0 = 0; r0 = 0; delta0 = 0;
    X0 = [u0, v0, r0, psi0, delta0];
end
for m = 0:stepMax
    C = wrapToPi_loc( nearB + dirSign*m*scanStep );
    if dirSign < 0
        if ang_passed(C, farB, nearB, -1), break; end
    else
        if ang_passed(C, nearB, farB, +1), break; end
    end
    if forward_safe_3dof(RP0, [0,0], X0, u_ref, Vts, C, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            t_pred, dt, radius)
        gNear = C; break;
    end
end
end

function yes = forward_safe_3dof(RP0, P0_os, X0_os, u_ref, Vts, C, ...
    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, t_pred, dt, radius)
if numel(X0_os) ~= 5, yes = false; return; end
u = X0_os(1);
v = X0_os(2);
r = X0_os(3);
psi = X0_os(4);
delta = X0_os(5);
Pos = P0_os(:).';
Pnb0 = RP0(:).';
Vts  = Vts(:).';
t = 0;
yes = true;
while t <= t_pred
    Pnb = Pnb0 + Vts * t;
    d = norm(Pnb - Pos);
    if t > 0 && d < radius - 1e-6, yes = false; return; end
    epsi = atan2(sin(C - psi), cos(C - psi));
    delta_cmd = Kp*epsi + Kd*(-r);
    delta_cmd = max(-deltaMax, min(deltaMax, delta_cmd));
    step_delta = max(-deltaRate*dt, min(deltaRate*dt, delta_cmd - delta));
    delta = delta + step_delta;
    r_dot = (Kdel*delta - r)/max(1e-6,Tr);
    u_dot = (u_ref - u)/max(1e-6,Tu);  u_dot = max(-aMax, min(aMax, u_dot));
    v_dot = -Dv*v + Alpha*u*r;
    r = r + r_dot*dt;
    u = u + u_dot*dt;
    v = v + v_dot*dt;
    psi = wrapToPi_loc(psi + r*dt);
    Pos = Pos + [u*cos(psi) - v*sin(psi), u*sin(psi) + v*cos(psi)] * dt;
    t = t + dt;
end
end

function tf = ang_passed(C, L, R, dirSign)
if dirSign < 0
    tf = angleLess(C, L) | angleGreater(C, R);
else
    tf = angleGreater(C, R) | angleLess(C, L);
end
end

function tf = angleLess(a,b),    tf = atan2(sin(a-b),cos(a-b)) < 0; end
function tf = angleGreater(a,b), tf = atan2(sin(a-b),cos(a-b)) > 0; end
