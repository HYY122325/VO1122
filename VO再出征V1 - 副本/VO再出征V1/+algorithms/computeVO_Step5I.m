function newVelocity = computeVO_Step5I(agent, neighbors, params)
% computeVO_Step5I — Engineering-Robust Version + COLREG Episode Lock (No Jumping)
%
% ========================= 本版核心修复（对应你的 1~6 条） =========================
% (1) 静止目标不走 COLREG/FSM：视为障碍物（仍参与硬安全 + 缓冲软惩罚）。
% (2) Give-way FSM 触发增加风险门槛 gate：无 risk/无 CR/远离 dp 时不提前触发 TC-1。
% (3) tsPhase 退出条件对齐 Past&Clear/sr_lock：优先采用 (t*<0 & separating) 立即退出。
% (4) tsPhase>0 时 FSM 使用“FSM锁定的 role/scene”，不吃实时分类（杜绝僵尸 phase）。
% (5) Past&Clear 不再被 fsmActive 一票否决：只要 LOS 候选不违反当前 override 就允许切回（但 Phase-1 锁航向时不抢控）。
% (6) Seamanship Fixed Gate：仅当上一帧航向与 CLOS 夹角 < gate(默认60°) 且不更偏离目标，才允许平滑覆盖。
%
% 其余结构保持：硬安全 > (FSM/COLREG硬规则) > 缓冲软代价 > Past&Clear > Seamanship > Fallback
% ==============================================================================

%% =========== 边界情况 ===========
if isempty(neighbors)
    newVelocity = agent.prefVelocity;
    return;
end

%% =========== 参数与默认 ===========
s5       = getfieldwithdef(params, 'step5', struct());
epsC1    = getfieldwithdef(s5, 'c1_eps', 1e-2);

useGW    = getfieldwithdef(params.lvo, 'useGoodwin', true);
ds_base  = params.lvo.ds_base;
dp_factor= params.lvo.dp_factor;

cr_enter = getfieldwithdef(params.lvo, 'cr_threshold', 0.4);

hdgGridN = getfieldwithdef(params.lvo,'hdgGridN',360);
hdgGrid  = linspace(-pi, pi, hdgGridN);

% --- 静止目标阈值（新增） ---
static_v_min = getfieldwithdef(s5,'static_v_min',0.10);

% --- Seamanship Fixed Gate（新增） ---
seam_gate_deg = getfieldwithdef(s5,'seamanship_gate_deg',60.0);
seam_gate_rad = deg2rad(seam_gate_deg);

% --- 时间（debug） ---
simStep = -1;
simDt   = 0.1;
if isfield(params, 'sim')
    if isfield(params.sim,'step'), simStep = params.sim.step; end
    if isfield(params.sim,'dt'),   simDt   = params.sim.dt;   end
end
if simStep >= 0, t_now = simStep * simDt; else, t_now = NaN; end

% --- DEBUG 节流（1 Hz） ---
persistent S5STATE ASSIST_last_print_t DEBUG_last_print_t
if isfield(params, 'sim') && isfield(params.sim, 'step') && params.sim.step <= 1
    S5STATE             = [];
    ASSIST_last_print_t = -inf;
    DEBUG_last_print_t  = -inf;
end
if isempty(S5STATE),             S5STATE = struct('ids',[], 'data',[]); end
if isempty(ASSIST_last_print_t), ASSIST_last_print_t = -inf;           end
if isempty(DEBUG_last_print_t),  DEBUG_last_print_t = -inf;            end

doDebug = false;
if isfinite(t_now) && (t_now - DEBUG_last_print_t >= 1.0 - 1e-9)
    DEBUG_last_print_t = t_now;
    doDebug = true;
    fprintf('\n================ [DEBUG t=%.2f s | Agent %d] ================\n', t_now, agent.id);
end

% ---------- 代价权重 ----------
lambda_pts = getfieldwithdef(params.lvo, 'lambda_pts', 2.0);
gamma      = getfieldwithdef(params.lvo,'gamma',2.5);

omega      = getfieldwithdef(s5, 'route_w', 0.5);
omega_ct   = getfieldwithdef(s5,'route_ct_w',omega);

% ---------- 工程新增参数 ----------
speed_scales = getfieldwithdef(s5,'speed_scales',[1.0, 0.8]);
w_speed      = getfieldwithdef(s5,'speed_w',0.8);

CR_floor     = getfieldwithdef(s5,'cr_floor',0.20);
w_colregs    = getfieldwithdef(s5,'colregs_soft_w',5.0);

useGCCO_hard = getfieldwithdef(s5,'use_gcco_hard',true);
gcco_gate_s  = getfieldwithdef(s5,'gcco_speed_gate',0.85);

allow_stop   = getfieldwithdef(s5,'allow_stop',true);
stop_scale   = getfieldwithdef(s5,'stop_scale',0.0);

useCOLREGS = getfieldwithdef(s5,'useCOLREGS',true);
useSeaman  = getfieldwithdef(s5,'useSeamanship',true);
useFSM     = getfieldwithdef(s5,'useFSM',true);

% FSM 参数
fsm_sigma  = getfieldwithdef(s5,'fsm_sigma',25.0);
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

% 速度与参考航向
Vpref  = max(1e-3, norm(agent.prefVelocity));   % 期望海速（用于分类/规则/采样尺度）
Cprev  = headingFromVelocity(agent);
toGoal = agent.goal - agent.position;
CLOS   = atan2(toGoal(2), toGoal(1));
if ~isfinite(CLOS), CLOS = Cprev; end

%% ===== neighbors 按 id 排序，稳定 FSM 索引 =====
if ~isempty(neighbors)
    [~, ord] = sort([neighbors.id]);
    neighbors = neighbors(ord);
end

% S0 debug
if doDebug
    fprintf(' [S0-OS] Pos=(%.1f,%.1f), Vel=(%.2f,%.2f), Vpref=%.2f m/s\n', ...
        agent.position(1), agent.position(2), agent.velocity(1), agent.velocity(2), Vpref);
    fprintf(' [S0-OS] Cprev=%.1f deg, CLOS=%.1f deg\n', rad2deg(Cprev), rad2deg(CLOS));
    fprintf(' [S0-OS] Flags: useCOLREGS=%d, useFSM=%d, useSeamanship=%d\n', useCOLREGS, useFSM, useSeaman);
    fprintf(' [S0-OS] speed_scales = ['); fprintf('%.2f ', speed_scales); fprintf(']\n');
    fprintf(' [S0-OS] seamanship_gate = %.1f deg\n', seam_gate_deg);
    if ~isempty(neighbors)
        fprintf(' [S0-OS] Neighbor IDs(sorted): '); fprintf('%d ', neighbors.id); fprintf('\n');
    end
end

% 读取状态（含 episode-lock 记忆）
st = get_agent_state();

% ====== episode-lock 配置（新增）======
sr_enable          = getfieldwithdef(s5,'sr_lock_enable',true);

sr_confirm_steps   = getfieldwithdef(s5,'sr_confirm_steps',2);  % 连续一致几帧后锁（风险高时可立即锁）
sr_lock_on_risk    = getfieldwithdef(s5,'sr_lock_on_risk',true);
sr_lock_on_dp      = getfieldwithdef(s5,'sr_lock_on_dp',true);

sr_cr_exit_factor  = getfieldwithdef(s5,'sr_cr_exit_factor',0.50); % cr_exit = factor*cr_enter
sr_cr_exit         = getfieldwithdef(s5,'sr_cr_exit',sr_cr_exit_factor*cr_enter);

sr_dp_clear_factor = getfieldwithdef(s5,'sr_dp_clear_factor',1.20); % D>1.2*dp 也允许解锁（配合分离）
sr_sep_eps         = getfieldwithdef(s5,'sr_sep_eps',0.00);         % separating 判据 dot(RP,RV)>eps

sr_mem_ttl         = getfieldwithdef(s5,'sr_mem_ttl',60.0);         % 记忆TTL（秒），防止长期堆积
if sr_enable
    st = srmem_prune(st, t_now, sr_mem_ttl);
end

%% === 基线横向偏移 ===
if isprop(agent, 'pathOrigin')
    currentPathOrigin = agent.pathOrigin;
else
    currentPathOrigin = agent.position;
end
haveBase = all(isfinite(currentPathOrigin)) && (norm(agent.goal - currentPathOrigin) > 1e-6);
if haveBase
    base_vec = agent.goal - currentPathOrigin;
    base_dir = base_vec / norm(base_vec);
    n_base   = [-base_dir(2), base_dir(1)];
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
                    'colregsMask',@(C)true, ...
                    'RP',[NaN,NaN],'dp',NaN, ...
                    'isStatic',false,'skipFSM',false,'skipCOLREGS',false), ...
             numel(neighbors),1);

for j = 1:numel(neighbors)
    nb = neighbors(j);

    in_core = false(1, hdgGridN);
    in_bco  = false(1, hdgGridN);

    % 几何 CCO/BCO（Vpref 扫描）
    for k = 1:hdgGridN
        C = hdgGrid(k);
        v_scan = Vpref * [cos(C), sin(C)];
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
    obs(j).RP = RP0;

    [ds_ref, dp_ref] = compute_ds_dp_paper(RP0, Cprev, ds_base, dp_factor, useGW);
    obs(j).dp = dp_ref;

    % 标记静止目标（新增）
    obs(j).isStatic      = (norm(Vts) < static_v_min);
    obs(j).skipFSM       = obs(j).isStatic;
    obs(j).skipCOLREGS   = obs(j).isStatic;

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
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAl = expand_one_side_3dof(Al, Bl, -1, ds_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end

    if all(isfinite([Ar,Br]))
        obs(j).gBr = expand_one_side_3dof(Br, Ar, +1, dp_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAr = expand_one_side_3dof(Ar, Br, +1, ds_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end
end

if doDebug && ~isempty(neighbors)
    fprintf(' [S1-FRONT] 完成 CCO/BCO + GCCO/GBCO 扫描, 邻船数 = %d\n', numel(neighbors));
    for jdbg = 1:numel(neighbors)
        fprintf('   TS%3d | CCO:[%6.1f,%6.1f]  BCO:[%6.1f,%6.1f] | GCCO:[%6.1f,%6.1f] GBCO:[%6.1f,%6.1f] | static=%d\n', ...
            neighbors(jdbg).id, ...
            rad2deg(obs(jdbg).Al), rad2deg(obs(jdbg).Bl), ...
            rad2deg(obs(jdbg).Ar), rad2deg(obs(jdbg).Br), ...
            rad2deg(obs(jdbg).gAl), rad2deg(obs(jdbg).gBl), ...
            rad2deg(obs(jdbg).gAr), rad2deg(obs(jdbg).gBr), ...
            obs(jdbg).isStatic);
    end
end

%% =========== S2：CR + 局面/角色（带 episode-lock） + COLREGS ===========
[CR, riskMask] = compute_frontend_CR_paper(os_for_cr(agent), neighbors, params, Cprev, cr_enter);

Nnb   = numel(neighbors);
useTS = true(1, Nnb);

if doDebug && ~isempty(neighbors)
    fprintf(' [S2-RISK] CR:');
    for j = 1:Nnb, fprintf(' TS%d=%.2f', neighbors(j).id, CR(j)); end
    fprintf('\n');
end

% Past & Clear：风险集合记忆清理（只对 riskMask=true 的 TS 才会被移除）
for j = 1:Nnb
    if ~riskMask(j), continue; end
    nb = neighbors(j);
    RP = nb.position - agent.position;
    Vrel_pair = nb.velocity - agent.velocity;           % TS-OS
    [~, ~, tstar_pair] = compute_dcpa_tcpa(RP, Vrel_pair);
    is_sep = dot(RP, Vrel_pair) > 0;
    if tstar_pair < 0 && is_sep
        useTS(j) = false;
        if doDebug
            fprintf('   [S2-RISK] Past&Clear: TS %d t* = %.2f < 0 & Sep=1, 从风险集合移除.\n', nb.id, tstar_pair);
        end
    end
end
riskTS_active   = riskMask & useTS;
allRiskCleared  = ~any(riskTS_active);

% --- 局面/角色 + COLREGS mask（关键：先 episode-lock，再建 mask） ---
for j=1:numel(neighbors)
    nb = neighbors(j);

    % (1) 静止目标：直接障碍物化（不分类、不锁定、不出规则、不进FSM）
    if obs(j).isStatic
        obs(j).scene = "OTHERS";
        obs(j).role  = 'standon';
        obs(j).colregsMask = @(C) true;
        if doDebug
            fprintf('   > TS %d: [STATIC_OBS] v=%.2f < %.2f -> 障碍物模式(跳过COLREG/FSM)\n', ...
                nb.id, norm(nb.velocity), static_v_min);
        end
        continue;
    end

    % 1) raw 分类（closing 判据修正+更稳）
    [scene_raw, role_raw] = classify_scene_role_paper_stable(agent, nb, Cprev, Vpref, doDebug);

    % 2) episode-lock 更新并得到最终 scene/role（不跳变）
    if sr_enable
        [st, scene_use, role_use] = sr_lock_update(st, agent, nb, obs(j).dp, ...
        scene_raw, role_raw, CR(j), riskMask(j), useTS(j), ...
        cr_enter, sr_cr_exit, sr_confirm_steps, sr_lock_on_risk, sr_lock_on_dp, ...
        sr_dp_clear_factor, sr_sep_eps, t_now, doDebug);
    else
        scene_use = scene_raw;
        role_use  = role_raw;
    end

    obs(j).scene = scene_use;
    obs(j).role  = role_use;

    % 3) COLREGS mask（只吃“锁定后的 scene/role”）
    obs(j).colregsMask = @(C) true;
    if useCOLREGS && ~obs(j).skipCOLREGS
        obs(j).colregsMask = build_colregs_mask_paper(agent, nb, obs(j).scene, obs(j).role, Vpref);
    end
end

%% =========== S3：FSM 覆盖（吃稳定的 scene/role + FSM内部锁定） ===========
fsmActive = false;

if useFSM && ~isempty(neighbors)
    st = fsm_update_paper_strict(st, agent, neighbors, obs, ...
                                 fsm_sigma, deg2rad(5.0), Cprev, CLOS, ...
                                 ds_base, dp_factor, useGW, fsm_dt, t_now, doDebug, epsC1, ...
                                 CR, riskMask, useTS, cr_enter, sr_cr_exit);

    % 先全局解除原始 COLREGS，再按 overrides 施加
    if isfield(st,'colregs_mode') && st.colregs_mode ~= 0
        for j2 = 1:numel(obs), obs(j2).colregsMask = @(C) true; end
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

if doDebug && ~isempty(neighbors)
    fprintf('---------------------------------------------------\n');
    if useFSM
        if fsmActive
            fprintf(' [S3-FSM] ACTIVE. colregs_mode=%d, tsPhase=[', st.colregs_mode);
            fprintf('%d ', st.tsPhase);
            fprintf(']\n');
        else
            fprintf(' [S3-FSM] ENABLED but NOT triggered.\n');
        end
    else
        fprintf(' [S3-FSM] DISABLED. 使用纯 COLREGS + Seamanship.\n');
    end

    for jdbg = 1:numel(neighbors)
        ph = 0; if isfield(st,'tsPhase') && numel(st.tsPhase)>=jdbg, ph = st.tsPhase(jdbg); end
        fprintf('   TS%3d | CR=%.2f risk=%d useTS=%d | scene=%s role=%s | P=%d | static=%d\n', ...
            neighbors(jdbg).id, CR(jdbg), riskMask(jdbg), useTS(jdbg), ...
            char(obs(jdbg).scene), obs(jdbg).role, ph, obs(jdbg).isStatic);
    end
    fprintf('---------------------------------------------------\n');
end

%% =========== S4：联合优化（C + s） + 不矛盾覆盖 ===========
isObserving = false;
if isfield(st,'tsPhase'), isObserving = any(st.tsPhase == 1); end

bestHdg    = Cprev;
bestScale  = 1.0;
bestCost   = inf;
bestBreak  = struct('U_multi',NaN,'U_route',NaN,'U_speed',NaN,'U_colregs',NaN);
decisionSource = 'OPT';

lockApplied = false; optApplied = false; pcApplied = false; seaApplied = false; fbApplied = false;

% --------- 观察期锁定（锁航向 + 锁速度倍率）---------
if isObserving && isfield(st,'prevObjHdg') && isfinite(st.prevObjHdg)
    lockScale = 1.0;
    if isfield(st,'prevObjScale') && isfinite(st.prevObjScale)
        lockScale = st.prevObjScale;
    end
    [okLock, ~] = candidate_feasible_hard(st.prevObjHdg, lockScale, false);
    if okLock
        bestHdg = st.prevObjHdg;
        bestScale = lockScale;
        bestCost = NaN;
        decisionSource = 'FSM_LOCK';
        lockApplied = true;
        if doDebug
            fprintf(' [S4-LOCK] Phase-1 锁定 (C=%.1fdeg, s=%.2f) 通过硬安全+规则，跳过优化\n', ...
                rad2deg(bestHdg), bestScale);
        end
    else
        if doDebug, fprintf(' [S4-LOCK] 锁定候选不安全/不合规，转入优化。\n'); end
    end
end

% --------- 主优化：硬安全 + 硬规则（默认）---------
if ~lockApplied
    [bestHdg, bestScale, bestCost, bestBreak, found] = solve_search(false);
    optApplied = true;

    if ~found
        fbApplied = true;
        if doDebug, fprintf(' [S4-FB] 无可行解：进入 Fallback-1（COLREGS 软惩罚）...\n'); end
        [bestHdg, bestScale, bestCost, bestBreak, found2] = solve_search(true);

        if ~found2
            if doDebug, fprintf(' [S4-FB] Fallback-1 仍无解：进入 Fallback-2（最不坏）...\n'); end
            [bestHdg, bestScale, bestCost, bestBreak] = solve_least_bad();
        end
    end
end

% --------- Past&Clear：切回 LOS（不再被 fsmActive 一票否决；但不抢 Phase-1 锁控）---------
if ~lockApplied && ~isempty(neighbors) && allRiskCleared && isfinite(CLOS)
    [okPC, ~] = candidate_feasible_hard(CLOS, 1.0, false);
    if okPC
        bestHdg = CLOS;
        bestScale = 1.0;
        decisionSource = 'PAST_CLEAR_ROUTE';
        pcApplied = true;
        if doDebug, fprintf(' [S4-PAST&CLEAR] LOS 可行 -> 切回 LOS（C=%.1fdeg,s=1.0）\n', rad2deg(bestHdg)); end
    else
        if doDebug, fprintf(' [S4-PAST&CLEAR] LOS 不可行(硬安全/override不通过) -> 不覆盖。\n'); end
    end
end

% --------- Seamanship 平滑：Fixed Gate + 不更偏离目标 + 硬安全/规则可行 ---------
if useSeaman && isfield(st,'prevObjHdg') && isfinite(st.prevObjHdg) && ~strcmp(decisionSource,'FSM_LOCK') && isfinite(CLOS)
    devPrev = abs(angdiff_signed(st.prevObjHdg, CLOS));
    devNow  = abs(angdiff_signed(bestHdg,      CLOS));

    % Fixed Gate：上一帧若已背道而驰（偏离目标超过门限），禁止平滑覆盖
    if devPrev <= seam_gate_rad + 1e-12

        % 不允许“更偏离目标”：上一帧必须不比当前优化解更差（否则平滑没有价值）
        if devPrev <= devNow + 1e-12

            % 你原来的“同向转舵且幅度收敛”条件（保留）
            dC0 = angdiff_signed(st.prevObjHdg, st.prevStartHdg);
            dC1 = angdiff_signed(bestHdg,      st.prevStartHdg);

            if sign(dC0)*sign(dC1) > 0 && abs(dC1) < abs(dC0)
                candH = st.prevObjHdg;
                candS = bestScale;
                if isfield(st,'prevObjScale') && isfinite(st.prevObjScale)
                    candS = st.prevObjScale;
                end

                [okSea, ~] = candidate_feasible_hard(candH, candS, false);
                if okSea
                    bestHdg = candH;
                    bestScale = candS;
                    decisionSource = 'SEAMAN_KEEP';
                    seaApplied = true;
                    if doDebug
                        fprintf(' [S4-SEAMANSHIP] FixedGate通过(devPrev=%.1fdeg<=%.1fdeg) 且不更偏离 -> 采用上一帧 (C=%.1fdeg,s=%.2f)\n', ...
                            rad2deg(devPrev), seam_gate_deg, rad2deg(bestHdg), bestScale);
                    end
                end
            end
        else
            if doDebug
                fprintf(' [S4-SEAMANSHIP] 禁止：上一帧更偏离目标(devPrev=%.1fdeg > devNow=%.1fdeg)\n', rad2deg(devPrev), rad2deg(devNow));
            end
        end
    else
        if doDebug
            fprintf(' [S4-SEAMANSHIP] FixedGate阻断：上一帧偏离目标%.1fdeg > gate=%.1fdeg -> 服从当前解\n', ...
                rad2deg(devPrev), seam_gate_deg);
        end
    end
end

%% =========== 决策打印 ===========
if doDebug
    fprintf(' [S4-DECISION] Source=%s | C=%.1fdeg | s=%.2f | V=%.2f m/s\n', ...
        decisionSource, rad2deg(bestHdg), bestScale, bestScale*Vpref);

    if isfinite(bestCost)
        fprintf(' [S4-COST] U_multi=%.3f, U_route=%.3f, U_speed=%.3f, U_colregs=%.3f, U_sum=%.3f\n', ...
            bestBreak.U_multi, bestBreak.U_route, bestBreak.U_speed, bestBreak.U_colregs, bestCost);
    end

    fprintf(' [S4-CHAIN] LOCK=%d OPT=%d PAST_CLEAR=%d SEAMAN=%d FB=%d -> FINAL=%s\n', ...
        lockApplied, optApplied, pcApplied, seaApplied, fbApplied, decisionSource);
    fprintf('=================================================================\n');
end

%% =========== Action-5 Assist Rule 细节调试（保留） ===========
if isfield(st,'colregs_mode') && st.colregs_mode == 5 && isfinite(t_now)
    if t_now - ASSIST_last_print_t >= 0.5 - 1e-9
        ASSIST_last_print_t = t_now;
        for j_dbg = 1:numel(neighbors)
            if isfield(st,'tsRVo') && size(st.tsRVo,1) >= j_dbg && all(isfinite(st.tsRVo(j_dbg,:)))
                ts_dbg   = neighbors(j_dbg);
                RVo_dbg  = st.tsRVo(j_dbg,:);
                RVm_dbg  = agent.velocity - ts_dbg.velocity;

                v_c_dbg  = (bestScale*Vpref)*[cos(bestHdg), sin(bestHdg)];
                RVOS_dbg = v_c_dbg - ts_dbg.velocity;

                cosOS   = cos_between(RVo_dbg, RVOS_dbg);
                cosM    = cos_between(RVo_dbg, RVm_dbg);
                lhs_dbg = cosOS - cosM;
                ok_dbg  = assist_rule_mask(bestHdg, ts_dbg.velocity, (bestScale*Vpref), RVo_dbg, RVm_dbg);

                RP_dbg   = ts_dbg.position - agent.position;
                Vrel_dbg = ts_dbg.velocity - agent.velocity;
                [dcpa_dbg, ~, tstar_dbg] = compute_dcpa_tcpa(RP_dbg, Vrel_dbg);

                fprintf('[t=%.2f] DBG-A5: TS=%d lhs=%.3f ok=%d | D=%.2f DCPA=%.2f t*=%.2f\n', ...
                        t_now, ts_dbg.id, lhs_dbg, ok_dbg, norm(RP_dbg), dcpa_dbg, tstar_dbg);
            end
        end
    end
end

%% =========== 输出 & 写回状态 ===========
newVelocity = (bestScale*Vpref) * [cos(bestHdg), sin(bestHdg)];
st.prevObjHdg    = bestHdg;
st.prevStartHdg  = Cprev;
st.prevObjScale  = bestScale;
set_agent_state(st);

%% ==================== 内部函数 ====================
    function [stx] = os_for_cr(os_in)
        % CR 计算用一个“稳定的 OS 速度尺度”（避免起步/低速导致 K 抖动）
        stx = os_in;
        % compute_frontend_CR_paper 内部会优先读 prefVelocity
    end

    function stx = fsm_update_paper_strict(stx, os, nbs, obss, ...
                                           sigma, deg5_, COS, CLOS_loc, ...
                                           ds0, dp_fac, useGW_, fsm_dt_, t_now_, doDbg, epsC1_loc, ...
                                           CR_in, riskMask_in, useTS_in, cr_enter_in, cr_exit_in)
        N = numel(nbs);
        stx = ensure_len_local(stx, N);

        persistent lastPhasePrinted
        if isempty(lastPhasePrinted) || numel(lastPhasePrinted) ~= N
            lastPhasePrinted = -ones(1,N);
        end

        overrides = struct('j',{},'mask',{},'type',{});
        mode = 0;

        Vsea = max(1e-3, norm(os.prefVelocity));

        for jj = 1:N
            ts = nbs(jj);

            % id变化：清空该槽位状态
            if stx.tsIds(jj) ~= ts.id
                stx.tsIds(jj)      = ts.id;
                stx.tsPhase(jj)    = 0;
                stx.tsObsLeft(jj)  = 0;
                stx.tsHead0(jj)    = NaN;
                stx.tsHeadings(jj) = NaN;
                stx.tsRVo(jj,:)    = [NaN,NaN];
                stx.tsRVopt(jj,:)  = [NaN,NaN];
                stx.fsmRole(jj)    = "";
                stx.fsmScene(jj)   = "NONE";
                lastPhasePrinted(jj) = -1;
            end

            % 静止目标/跳过FSM：直接强制退回P0（避免僵尸phase）
            if isfield(obss(jj),'skipFSM') && obss(jj).skipFSM
                if stx.tsPhase(jj) ~= 0
                    stx.tsPhase(jj) = 0;
                    stx.tsObsLeft(jj) = 0;
                    stx.fsmRole(jj)  = "";
                    stx.fsmScene(jj) = "NONE";
                end
                continue;
            end

            RP = obss(jj).RP;
            if ~all(isfinite(RP)), RP = ts.position - os.position; end

            [~, dpj] = compute_ds_dp_paper(RP, COS, ds0, dp_fac, useGW_);
            D = norm(RP);

            Cts_now = atan2(ts.velocity(2), ts.velocity(1));
            if isfinite(stx.tsHeadings(jj)), Cts_prev = stx.tsHeadings(jj); else, Cts_prev = Cts_now; end

            if D <= dpj && ~isfinite(stx.tsHead0(jj))
                stx.tsHead0(jj) = Cts_prev;
                stx.tsRVo(jj,:) = os.velocity - ts.velocity;
            end

            if isfinite(stx.tsHead0(jj))
                dCTS = abs(angdiff_signed(Cts_now, stx.tsHead0(jj)));
            else
                dCTS = 0;
            end

            inGCCO = any_is_in_gcco(COS, obss(jj));

            Vrel_ts = ts.velocity - os.velocity;
            [~, ~, tstar] = compute_dcpa_tcpa(RP, Vrel_ts);
            mu = max(0, tstar) / max(1e-6, sigma);

            % 读取风险信息（用于 gate 与退出一致化）
            CRjj   = 0; if numel(CR_in) >= jj && isfinite(CR_in(jj)), CRjj = CR_in(jj); end
            rskjj  = false; if numel(riskMask_in) >= jj, rskjj = riskMask_in(jj); end
            usejj  = true;  if numel(useTS_in) >= jj,   usejj = useTS_in(jj);   end

            % ====== FSM内部 role/scene 锁定（tsPhase>0 时强制使用） ======
            if stx.tsPhase(jj) > 0 && strlength(stx.fsmRole(jj)) > 0
                base_role = char(stx.fsmRole(jj));
                scene     = stx.fsmScene(jj);
            else
                base_role = obss(jj).role;
                scene     = obss(jj).scene;
            end

            % ---------------- GIVE-WAY ----------------
            if strcmpi(base_role,'giveway')

                % (2) 风险门槛 gate：无 risk/无 CR/远离 dp/不在GCCO 时不触发 TC-1
                gate_ok = ( (rskjj && usejj) || (CRjj >= cr_enter_in) || (D <= dpj) || inGCCO );

                if stx.tsPhase(jj) == 0
                    if gate_ok
                        if dCTS > deg5_
                            RVopt = os.velocity - ts.velocity;
                            if isfield(stx,'prevObjHdg') && isfinite(stx.prevObjHdg)
                                sc = 1.0;
                                if isfield(stx,'prevObjScale') && isfinite(stx.prevObjScale)
                                    sc = stx.prevObjScale;
                                end
                                vopt = (sc*Vsea) * [cos(stx.prevObjHdg), sin(stx.prevObjHdg)];
                                RVopt = vopt - ts.velocity;
                            end

                            RVo = stx.tsRVo(jj,:);
                            if ~all(isfinite(RVo)), RVo = os.velocity - ts.velocity; end
                            RVm = os.velocity - ts.velocity;

                            lhs = cos_between(RVo, RVopt) - cos_between(RVo, RVm);
                            if lhs < -1e-6
                                stx.tsPhase(jj)   = 1;
                                stx.tsObsLeft(jj) = min(5.0, mu);
                                stx.tsRVopt(jj,:) = RVopt;

                                % 进入FSM：锁定本次 episode 的 role/scene
                                stx.fsmRole(jj)  = string(obss(jj).role);
                                stx.fsmScene(jj) = string(obss(jj).scene);

                                if doDbg
                                    fprintf('   [FSM] TS%d TC-1 triggered (Give-way). gate=1 dCTS=%.1fdeg, mu=%.2f, lhs=%.3f<0 -> P1 | lock(%s,%s)\n', ...
                                            ts.id, rad2deg(dCTS), stx.tsObsLeft(jj), lhs, stx.fsmRole(jj), stx.fsmScene(jj));
                                end
                            end
                        end
                    else
                        if doDbg && (dCTS > deg5_)
                            fprintf('   [FSM] TS%d Give-way gate=0 (noRisk/noCR/far/noGCCO) -> 不触发TC-1\n', ts.id);
                        end
                    end

                elseif stx.tsPhase(jj) == 1
                    stx.tsObsLeft(jj) = stx.tsObsLeft(jj) - fsm_dt_;
                    force_exit = (tstar < 0) || (D > 1.5*dpj);

                    if stx.tsObsLeft(jj) <= 0 || force_exit
                        if force_exit
                            stx.tsPhase(jj)   = 0;
                            stx.tsObsLeft(jj) = 0;
                            stx.fsmRole(jj)   = "";
                            stx.fsmScene(jj)  = "NONE";
                            if doDbg
                                fprintf('   [FSM] TS%d P1 exit (t*<0 or far). back to P0\n', ts.id);
                            end
                        else
                            RVo   = stx.tsRVo(jj,:);
                            RVopt = stx.tsRVopt(jj,:);
                            RVpre = os.velocity - ts.velocity;

                            z1 = cross2z_local(RVo, RVpre);
                            z2 = cross2z_local(RVo, RVopt);
                            isC1 = (z1*z2 < 0) || (abs(z1*z2) < epsC1_loc);

                            if isC1
                                stx.tsPhase(jj) = 2;
                                overrides(end+1).j    = jj; %#ok<AGROW>
                                overrides(end).mask   = @(C) mask_action1_z_le0_local(os, ts, C, Vsea);
                                overrides(end).type   = "ACTION1_Z<=0";
                                mode = max(mode, 1);

                                if doDbg
                                    fprintf('   [FSM] TS%d C-1 satisfied -> Action-1 (P2) z1*z2=%.4f\n', ts.id, z1*z2);
                                end
                            else
                                stx.tsPhase(jj) = 0;
                                stx.fsmRole(jj)   = "";
                                stx.fsmScene(jj)  = "NONE";
                                if doDbg
                                    fprintf('   [FSM] TS%d C-1 NOT satisfied -> back to P0 z1*z2=%.4f\n', ts.id, z1*z2);
                                end
                            end
                        end
                    end

                elseif stx.tsPhase(jj) == 2
                    % (3) 退出条件对齐：优先 (t*<0 & sep) 立即退出，不必等 D>dp
                    sep = is_separating_local(os, ts);
                    if (tstar < 0 && sep) || (D > dpj && sep) || (CRjj <= cr_exit_in && sep && tstar < 0)
                        stx.tsPhase(jj) = 0;
                        stx.fsmRole(jj)   = "";
                        stx.fsmScene(jj)  = "NONE";
                        if doDbg
                            fprintf('   [FSM] TS%d Action-1 exit (t*<0&sep OR (D>dp&sep)) -> P0 | t*=%.2f D=%.2f dp=%.2f\n', ...
                                ts.id, tstar, D, dpj);
                        end
                    else
                        overrides(end+1).j    = jj; %#ok<AGROW>
                        overrides(end).mask   = @(C) mask_action1_z_le0_local(os, ts, C, Vsea);
                        overrides(end).type   = "ACTION1_Z<=0";
                        mode = max(mode, 1);
                    end
                end
            end

            % ---------------- STAND-ON ----------------
            if strcmpi(base_role,'standon')
                if stx.tsPhase(jj) == 3
                    sep = is_separating_local(os, ts);
                    if (tstar < 0 && sep) || (D > dpj && sep) || (CRjj <= cr_exit_in && sep && tstar < 0)
                        stx.tsPhase(jj) = 0;
                        stx.fsmRole(jj)   = "";
                        stx.fsmScene(jj)  = "NONE";
                        if doDbg
                            fprintf('   [FSM] TS%d Stand-on action exit (t*<0&sep OR (D>dp&sep)) -> P0\n', ts.id);
                        end
                    else
                        if dCTS < deg5_
                            if strcmpi(scene,'CROSS_LEFT')
                                overrides(end+1).j  = jj; %#ok<AGROW>
                                overrides(end).mask = @(C) mask_action3_z_ge0_local(os, ts, C, Vsea);
                                overrides(end).type = "ACTION3_Z>=0";
                                mode = max(mode, 3);
                            elseif strcmpi(scene,'OVERTAKEN')
                                overrides(end+1).j  = jj; %#ok<AGROW>
                                overrides(end).mask = @(C) mask_overtaken_local(os, ts, C, COS, Vsea);
                                overrides(end).type = "ACTION4_OVERTAKEN";
                                mode = max(mode, 4);
                            end
                        else
                            RVo = stx.tsRVo(jj,:);
                            if ~all(isfinite(RVo)), RVo = os.velocity - ts.velocity; end
                            RVm = os.velocity - ts.velocity;
                            overrides(end+1).j  = jj; %#ok<AGROW>
                            overrides(end).mask = @(C) assist_rule_mask(C, ts.velocity, Vsea, RVo, RVm);
                            overrides(end).type = "ACTION5_ASSIST";
                            mode = max(mode, 5);
                        end
                    end
                end

                if stx.tsPhase(jj) == 0
                    if D <= dpj
                        if dCTS < deg5_
                            if strcmpi(scene,'CROSS_LEFT')
                                stx.tsPhase(jj) = 3;
                                stx.fsmRole(jj)  = string(obss(jj).role);
                                stx.fsmScene(jj) = string(obss(jj).scene);

                                overrides(end+1).j  = jj; %#ok<AGROW>
                                overrides(end).mask = @(C) mask_action3_z_ge0_local(os, ts, C, Vsea);
                                overrides(end).type = "TC2->ACTION3";
                                mode = max(mode, 3);
                                if doDbg
                                    fprintf('   [FSM] TS%d TC-2 -> Action-3 (CROSS_LEFT). D<=dp, dCTS=%.1fdeg\n', ...
                                            ts.id, rad2deg(dCTS));
                                end
                            elseif strcmpi(scene,'OVERTAKEN')
                                stx.tsPhase(jj) = 3;
                                stx.fsmRole(jj)  = string(obss(jj).role);
                                stx.fsmScene(jj) = string(obss(jj).scene);

                                overrides(end+1).j  = jj; %#ok<AGROW>
                                overrides(end).mask = @(C) mask_overtaken_local(os, ts, C, COS, Vsea);
                                overrides(end).type = "TC2->ACTION4";
                                mode = max(mode, 4);
                                if doDbg
                                    fprintf('   [FSM] TS%d TC-2 -> Action-4 (OVERTAKEN). D<=dp, dCTS=%.1fdeg\n', ...
                                            ts.id, rad2deg(dCTS));
                                end
                            end

                        elseif (dCTS > deg5_) && inGCCO
                            stx.tsPhase(jj) = 3;
                            stx.fsmRole(jj)  = string(obss(jj).role);
                            stx.fsmScene(jj) = string(obss(jj).scene);

                            RVo = stx.tsRVo(jj,:);
                            if ~all(isfinite(RVo)), RVo = os.velocity - ts.velocity; end
                            RVm = os.velocity - ts.velocity;

                            overrides(end+1).j  = jj; %#ok<AGROW>
                            overrides(end).mask = @(C) assist_rule_mask(C, ts.velocity, Vsea, RVo, RVm);
                            overrides(end).type = "TC3->ACTION5";
                            mode = max(mode, 5);

                            if doDbg
                                fprintf('   [FSM] TS%d TC-3 -> Action-5 (Assist). D<=dp, dCTS=%.1fdeg, COS∈GCCO=1\n', ...
                                        ts.id, rad2deg(dCTS));
                            end
                        end
                    end
                end
            end

            if norm(ts.velocity) > static_v_min
                stx.tsHeadings(jj) = Cts_now;
            end

            if doDbg && lastPhasePrinted(jj) ~= stx.tsPhase(jj)
                lastPhasePrinted(jj) = stx.tsPhase(jj);
            end
        end

        stx.overrides = overrides;
        stx.colregs_mode = mode;

        function stx2 = ensure_len_local(stx2, Nloc)
            if ~isfield(stx2,'tsIds') || numel(stx2.tsIds) < Nloc
                tmp = zeros(1,Nloc); if isfield(stx2,'tsIds'), tmp(1:numel(stx2.tsIds))=stx2.tsIds; end
                stx2.tsIds = tmp;
            end
            if ~isfield(stx2,'tsHeadings') || numel(stx2.tsHeadings) < Nloc
                tmp = nan(1,Nloc); if isfield(stx2,'tsHeadings'), tmp(1:numel(stx2.tsHeadings))=stx2.tsHeadings; end
                stx2.tsHeadings = tmp;
            end
            if ~isfield(stx2,'tsHead0') || numel(stx2.tsHead0) < Nloc
                tmp = nan(1,Nloc); if isfield(stx2,'tsHead0'), tmp(1:numel(stx2.tsHead0))=stx2.tsHead0; end
                stx2.tsHead0 = tmp;
            end
            if ~isfield(stx2,'tsPhase') || numel(stx2.tsPhase) < Nloc
                tmp = zeros(1,Nloc); if isfield(stx2,'tsPhase'), tmp(1:numel(stx2.tsPhase))=stx2.tsPhase; end
                stx2.tsPhase = tmp;
            end
            if ~isfield(stx2,'tsObsLeft') || numel(stx2.tsObsLeft) < Nloc
                tmp = zeros(1,Nloc); if isfield(stx2,'tsObsLeft'), tmp(1:numel(stx2.tsObsLeft))=stx2.tsObsLeft; end
                stx2.tsObsLeft = tmp;
            end
            if ~isfield(stx2,'tsRVo') || size(stx2.tsRVo,1) < Nloc
                newR = nan(Nloc,2); if isfield(stx2,'tsRVo'), newR(1:size(stx2.tsRVo,1),:)=stx2.tsRVo; end
                stx2.tsRVo = newR;
            end
            if ~isfield(stx2,'tsRVopt') || size(stx2.tsRVopt,1) < Nloc
                newR = nan(Nloc,2); if isfield(stx2,'tsRVopt'), newR(1:size(stx2.tsRVopt,1),:)=stx2.tsRVopt; end
                stx2.tsRVopt = newR;
            end

            % FSM内部锁定字段（新增）
            if ~isfield(stx2,'fsmRole') || numel(stx2.fsmRole) < Nloc
                tmp = repmat("",1,Nloc);
                if isfield(stx2,'fsmRole')
                    tmp(1:numel(stx2.fsmRole)) = stx2.fsmRole;
                end
                stx2.fsmRole = tmp;
            end
            if ~isfield(stx2,'fsmScene') || numel(stx2.fsmScene) < Nloc
                tmp = repmat("NONE",1,Nloc);
                if isfield(stx2,'fsmScene')
                    tmp(1:numel(stx2.fsmScene)) = stx2.fsmScene;
                end
                stx2.fsmScene = tmp;
            end

            if ~isfield(stx2,'overrides'), stx2.overrides = []; end
            if ~isfield(stx2,'colregs_mode'), stx2.colregs_mode = 0; end
            if ~isfield(stx2,'prevObjScale'), stx2.prevObjScale = NaN; end
        end

        function ok = mask_action1_z_le0_local(os_, ts_, Ccand, Vsea_loc)
            RP_ = ts_.position - os_.position;
            v_c = Vsea_loc*[cos(Ccand), sin(Ccand)];
            RC  = v_c - ts_.velocity;
            z   = RP_(1)*RC(2) - RP_(2)*RC(1);
            ok = (z <= 1e-12);
        end

        function ok = mask_action3_z_ge0_local(os_, ts_, Ccand, Vsea_loc)
            RP_ = ts_.position - os_.position;
            v_c = Vsea_loc*[cos(Ccand), sin(Ccand)];
            RC  = v_c - ts_.velocity;
            z   = RP_(1)*RC(2) - RP_(2)*RC(1);
            ok = (z >= -1e-12);
        end

        function ok = mask_overtaken_local(os_, ts_, Ccand, osHdg_, Vsea_loc)
            Q  = atan2(ts_.position(2)-os_.position(2), ts_.position(1)-os_.position(1)) - osHdg_;
            Q  = atan2(sin(Q), cos(Q));
            RP_ = ts_.position - os_.position;
            v_c = Vsea_loc*[cos(Ccand), sin(Ccand)];
            RC  = v_c - ts_.velocity;
            z   = RP_(1)*RC(2) - RP_(2)*RC(1);
            if Q > 0
                ok = (z <= 1e-12);
            else
                ok = (z >= -1e-12);
            end
        end

        function tf = is_separating_local(os_, ts_)
            RP_   = ts_.position - os_.position;
            Vrel_ = ts_.velocity - os_.velocity;
            tf = dot(RP_, Vrel_) > 0;
        end

        function z = cross2z_local(a,b)
            z = a(1)*b(2) - a(2)*b(1);
        end
    end

    function st2 = get_agent_state()
        st2 = struct('prevObjHdg', NaN, 'prevStartHdg', NaN, 'prevObjScale', NaN, ...
                     'overrides', [], ...
                     'tsIds', [], ...
                     'tsHeadings', [], 'tsHead0', [], ...
                     'tsPhase', [], 'tsObsLeft', [], ...
                     'tsRVo', zeros(0,2), 'tsRVopt', zeros(0,2), ...
                     'colregs_mode', 0, ...
                     'fsmRole', string([]), 'fsmScene', string([]), ...
                     'srMem', struct('id',{},'locked',{},'scene',{},'role',{}, ...
                                     'pendScene',{},'pendRole',{},'pendCount',{}, ...
                                     'lastSeen',{},'lastCR',{}));
        k = find(S5STATE.ids==agent.id,1);
        if isempty(k)
            S5STATE.ids(end+1)   = agent.id;
            S5STATE.data{end+1}  = st2;
        else
            st2 = S5STATE.data{k};
            if ~isfield(st2,'prevObjScale'), st2.prevObjScale = NaN; end
            if ~isfield(st2,'srMem')
                st2.srMem = struct('id',{},'locked',{},'scene',{},'role',{}, ...
                                   'pendScene',{},'pendRole',{},'pendCount',{}, ...
                                   'lastSeen',{},'lastCR',{});
            end
            if ~isfield(st2,'fsmRole'),  st2.fsmRole = string([]); end
            if ~isfield(st2,'fsmScene'), st2.fsmScene = string([]); end
        end
    end

    function set_agent_state(st2)
        k = find(S5STATE.ids==agent.id,1);
        if ~isempty(k)
            S5STATE.data{k} = st2;
        end
    end

    % --------- 主搜索：softCOLREGS=false 为硬规则；true 为软惩罚 ---------
    function [Cbest, Sbest, Jbest, breakdown, found] = solve_search(softCOLREGS)
        Cbest = Cprev; Sbest = 1.0; Jbest = inf;
        breakdown = struct('U_multi',NaN,'U_route',NaN,'U_speed',NaN,'U_colregs',NaN);
        found = false;

        for si = 1:numel(speed_scales)
            s = speed_scales(si);
            s = max(0, min(1.0, s));
            for k = 1:hdgGridN
                C = hdgGrid(k);
                [ok, bd] = candidate_feasible_hard(C, s, softCOLREGS);
                if ~ok, continue; end

                theta_d = abs(atan2(sin(C - CLOS), cos(C - CLOS)));
                U_route_ang  = omega * (theta_d / pi);

                if haveBase
                    v_dir    = [cos(C), sin(C)];
                    P_pred   = agent.position + (s*Vpref) * route_ct_T * v_dir;
                    rel_pred = P_pred - currentPathOrigin;
                    d_path_pred = abs(dot(rel_pred, n_base));
                    delta_d  = max(0, d_path_pred - d_path_now);
                    e_path_norm = min(delta_d / max(1e-3, route_ct_scale), 1.0);
                else
                    e_path_norm = 0;
                end
                U_route_path = omega_ct * e_path_norm;
                U_route = U_route_ang + U_route_path;

                U_speed = w_speed * (1 - s)^2;

                J = bd.U_multi + U_route + U_speed + bd.U_colregs;

                if J < Jbest
                    Jbest = J;
                    Cbest = C;
                    Sbest = s;
                    breakdown.U_multi   = bd.U_multi;
                    breakdown.U_route   = U_route;
                    breakdown.U_speed   = U_speed;
                    breakdown.U_colregs = bd.U_colregs;
                    found = true;
                end
            end
        end
    end

    % --------- 候选 (C,s) 的硬安全+规则检查，并返回软碰撞代价 ---------
    function [ok, bd] = candidate_feasible_hard(C, s, softCOLREGS)
        ok = true;
        bd = struct('U_multi',0,'U_colregs',0);

        v_cand = (s*Vpref) * [cos(C), sin(C)];

        for j = 1:numel(neighbors)
            if ~useTS(j), continue; end
            nb = neighbors(j);
            RP = nb.position - agent.position;
            RV = nb.velocity - v_cand;

            [ds, dp] = compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
            [hitCore, ~] = dcpaTcpaHit(RP, RV, ds, T, perceptionDist);
            if hitCore
                ok = false; return;
            end

            if useGCCO_hard && (s >= gcco_gate_s)
                if any_is_in_gcco(C, obs(j))
                    ok = false; return;
                end
            end

            if ~obs(j).colregsMask(C)
                if softCOLREGS
                    bd.U_colregs = bd.U_colregs + w_colregs;
                else
                    ok = false; return;
                end
            end

            Pbuf = buffer_penalty(RP, RV, ds, dp, gamma, T);
            if Pbuf > 0
                wcr = max(CR(j), CR_floor);
                bd.U_multi = bd.U_multi + lambda_pts * wcr * Pbuf;
            end
        end
    end

    % --------- Emergency Level-2：最不坏 ---------
    function [Cbest, Sbest, Jbest, breakdown] = solve_least_bad()
        Cbest = Cprev; Sbest = 0.4; Jbest = inf;
        breakdown = struct('U_multi',NaN,'U_route',NaN,'U_speed',NaN,'U_colregs',NaN);

        Sset = speed_scales;
        if allow_stop
            Sset = unique([Sset(:).', stop_scale]);
        end

        eps_t = 0.5;
        alpha_pen = 2.0;
        beta_col  = 0.5;

        for si = 1:numel(Sset)
            s = max(0, min(1.0, Sset(si)));
            v_cand_base = (s*Vpref);
            for k = 1:hdgGridN
                C = hdgGrid(k);
                v_cand = v_cand_base * [cos(C), sin(C)];

                worstSev = 0;
                colPen   = 0;

                for j = 1:numel(neighbors)
                    if ~useTS(j), continue; end
                    nb = neighbors(j);
                    RP = nb.position - agent.position;
                    RV = nb.velocity - v_cand;

                    [ds, dp] = compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
                    [dcpa, ~, tstar] = compute_dcpa_tcpa(RP, RV);

                    if ~isfinite(tstar), t_use = T; else, t_use = max(0, min(tstar, T)); end
                    d_cpa = norm(RP + RV * t_use);

                    pen = max(0, (ds - d_cpa) / max(1e-6, ds));
                    sev = 1/(t_use + eps_t) + alpha_pen * pen^2;

                    if d_cpa < dp
                        x = (dp - d_cpa)/max(1e-6, (dp - ds));
                        sev = sev + 0.5 * max(0, min(1, x))^gamma;
                    end

                    worstSev = max(worstSev, sev);

                    if ~obs(j).colregsMask(C)
                        colPen = colPen + beta_col;
                    end
                end

                theta_d = abs(atan2(sin(C - CLOS), cos(C - CLOS)));
                U_route_ang  = omega * (theta_d / pi);
                if haveBase
                    v_dir    = [cos(C), sin(C)];
                    P_pred   = agent.position + (s*Vpref) * route_ct_T * v_dir;
                    rel_pred = P_pred - currentPathOrigin;
                    d_path_pred = abs(dot(rel_pred, n_base));
                    delta_d  = max(0, d_path_pred - d_path_now);
                    e_path_norm = min(delta_d / max(1e-3, route_ct_scale), 1.0);
                else
                    e_path_norm = 0;
                end
                U_route = U_route_ang + omega_ct * e_path_norm;
                U_speed = w_speed * (1 - s)^2;

                J = worstSev + 0.2*U_route + 0.2*U_speed + colPen;

                if J < Jbest
                    Jbest = J;
                    Cbest = C;
                    Sbest = s;
                    breakdown.U_multi   = worstSev;
                    breakdown.U_route   = U_route;
                    breakdown.U_speed   = U_speed;
                    breakdown.U_colregs = colPen;
                end
            end
        end
    end

    % --------- 缓冲区软惩罚 ---------
    function P = buffer_penalty(RP, RV, ds, dp, gamma_, Th)
        P = 0;
        a = dot(RV,RV);
        if a < 1e-10, return; end

        tstar = -dot(RP,RV)/a;
        if ~isfinite(tstar) || tstar < 0, return; end
        t = min(tstar, Th);

        dcpa = norm(RP + RV*t);
        if dcpa >= dp
            P = 0;
        elseif dcpa <= ds
            P = 1.0;
        else
            x = (dp - dcpa)/max(1e-6, (dp - ds));
            x = max(0, min(1, x));
            P = x^gamma_;
        end
    end

end % ===== computeVO_Step5I end =====

%% ==================== 外部工具函数 ====================

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

function d = angleDiff(a,b),      d = atan2(sin(a-b), cos(a-b)); end
function d = angdiff_signed(a,b), d = atan2(sin(a-b), cos(a-b)); end

function c = cos_between(a,b)
na = norm(a); nb = norm(b);
if na < 1e-9 || nb < 1e-9, c = 1; return; end
c = dot(a,b)/(na*nb);
c = max(-1,min(1,c));
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

function [CRval, riskMask2] = compute_frontend_CR_paper(os, nbs, prm, osHdg, cr_thr_default)
N = numel(nbs);
CRval    = zeros(1,N);
riskMask2= false(1,N);
w   = getfieldwithdef(prm.lvo, 'cr_w', [0.40, 0.367, 0.167, 0.033, 0.033]);
thr = getfieldwithdef(prm.lvo, 'cr_threshold', cr_thr_default);

Vref_forK = 0;
try
    Vref_forK = norm(os.prefVelocity);
catch
    Vref_forK = norm(os.velocity);
end
Vref_forK = max(1e-6, Vref_forK);

for jj=1:N
    ts = nbs(jj);
    RP = ts.position - os.position;
    Vrel_vec = ts.velocity - os.velocity; % TS-OS
    [dcpa, ~, tstar] = compute_dcpa_tcpa(RP, Vrel_vec);
    [dsj, dpj] = compute_ds_dp_paper(RP, osHdg, prm.lvo.ds_base, prm.lvo.dp_factor, prm.lvo.useGoodwin);
    D = norm(RP);
    B = abs(angleDiff(atan2(RP(2),RP(1)), osHdg));
    K = Vref_forK / max(1e-6, norm(ts.velocity));        % 用 Vpref 稳定 K

    U1 = U_DCPA_paper(dcpa, dsj, dpj);
    U2 = U_TCPA_paper(tstar, dcpa, dsj, dpj, norm(Vrel_vec));
    U3 = U_D_paper(D, dsj, dpj);
    U4 = U_B_paper(B);
    U5 = U_K_sym(K);

    CRval(jj)     = w(1)*U1 + w(2)*U2 + w(3)*U3 + w(4)*U4 + w(5)*U5;
    riskMask2(jj) = (CRval(jj) >= thr);
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

% ======== 分类：closing 判据一致化 + 低速用期望速度稳住 ========
function [scene, role, info] = classify_scene_role_paper_stable(os, ts, osHdg, params, doDbg)
% classify_scene_role_paper_stable — CPA-first + Course-referenced (robust)
% 输入: (os, ts, osHdg, params, doDbg)
% 输出: scene(string) + role(char) + info(struct)
%
% 目标：避免右交叉长期掉进 OTHERS（不再用 RC_sin 门槛误杀）

if nargin < 5, doDbg = false; end

wrapPi = @(a) atan2(sin(a), cos(a));

% ---------- 基础量 ----------
P = ts.position - os.position;
D = norm(P);

% Vpref：用于低速时稳定 OS course
Vpref = 0;
try
    Vpref = norm(os.prefVelocity);
catch
    Vpref = norm(os.velocity);
end
Vpref = max(1e-3, Vpref);

% OS用于参考的速度：低速时用“期望速度方向”
V_os_use = os.velocity;
if norm(V_os_use) < 0.2 * Vpref
    V_os_use = Vpref * [cos(osHdg), sin(osHdg)];
end
C_os_ref = atan2(V_os_use(2), V_os_use(1));

% TS course
V_ts = ts.velocity;
V_ts_val = norm(V_ts);
if V_ts_val > 1e-6
    C_ts = atan2(V_ts(2), V_ts(1));
else
    C_ts = C_os_ref;
end

% ---------- 静止TS：交给“障碍物模式”，分类直接 OTHERS ----------
col = struct();
if isfield(params,'step5') && isfield(params.step5,'colregs')
    col = params.step5.colregs;
elseif isfield(params,'colregs')
    col = params.colregs;
end
V_ts_min = getfieldwithdef(col,'V_ts_min',0.10);

scene = "OTHERS";
role  = 'standon';
info  = struct();

if V_ts_val < V_ts_min
    if doDbg
        fprintf('   > TS %d: [STATIC] v=%.2f<%.2f -> scene=OTHERS (handled as obstacle)\n', ts.id, V_ts_val, V_ts_min);
    end
    info.is_static = true;
    return;
end

% ---------- CPA/TCPA（用 TS-OS 的相对速度，OS 用 V_os_use 稳定） ----------
Vrel = V_ts - V_os_use;
vrel2 = dot(Vrel, Vrel);

if vrel2 < 1e-8
    tcpa  = inf;
    dcpa  = D;
    tstar = inf;
else
    tstar = - dot(P, Vrel) / vrel2;
    tcpa  = max(0, tstar);
    P_cpa = P + tcpa * Vrel;
    dcpa  = norm(P_cpa);
end

% 接近判据（连续更稳）
is_closing = (dot(P, Vrel) < -0.05);

% 会遇门槛：默认用 dp（随Goodwin变化），更贴合你前端 dp 的尺度
try
    ds0 = params.lvo.ds_base;
    dp_fac = params.lvo.dp_factor;
    useGW  = params.lvo.useGoodwin;
    [~, dpj] = compute_ds_dp_paper(P, C_os_ref, ds0, dp_fac, useGW);
catch
    dpj = max(5.0, 1.5*D); % 兜底
end

T_enc = getfieldwithdef(col,'T_enc', 60.0);
D_enc = getfieldwithdef(col,'D_enc', dpj);    % 不设就用 dpj
D_enc = max(1e-6, D_enc);

is_encounter = (tcpa > 0) && (tcpa < T_enc) && (dcpa < D_enc);

% 相对方位 beta（相对 OS course；正=左舷，负=右舷）
beta = wrapPi(atan2(P(2), P(1)) - C_os_ref);
dC   = abs(wrapPi(C_ts - C_os_ref));

% 阈值
beta_headon_max   = deg2rad(getfieldwithdef(col,'beta_headon_max_deg',15.0));
dC_headon_min     = deg2rad(getfieldwithdef(col,'dC_headon_min_deg',150.0));
beta_overtake_min = deg2rad(getfieldwithdef(col,'beta_overtake_min_deg',112.5));
beta_cross_max    = deg2rad(getfieldwithdef(col,'beta_cross_max_deg',112.5));

% ---------- 分类（顺序：追越/被追越 → 对遇 → 交叉） ----------
if ~is_encounter
    scene = "OTHERS";
    role  = 'standon';
else
    % OVERTAKEN：TS 在本船后扇区（本船被追越，通常 stand-on）
    if is_closing && (abs(beta) > beta_overtake_min)
        scene = "OVERTAKEN";
        role  = 'standon';

    else
        % OVERTAKING：从 TS 视角看 OS 在其后扇区（本船追越，让路）
        B_os_from_ts = atan2(-P(2), -P(1));
        Q_os_in_ts_view = wrapPi(B_os_from_ts - C_ts);
        if is_closing && (abs(Q_os_in_ts_view) > beta_overtake_min)
            scene = "OVERTAKING";
            role  = 'giveway';

        % HEAD_ON：正前方 + 航向差接近对头
        elseif is_closing && (abs(beta) <= beta_headon_max) && (dC >= dC_headon_min)
            scene = "HEAD_ON";
            role  = 'giveway';

        % CROSSING：只要在前112.5°扇区并构成会遇，就别掉回 OTHERS
        elseif is_closing && (abs(beta) <= beta_cross_max)
            if beta < 0
                scene = "CROSS_RIGHT";  % TS 在右舷 -> OS give-way（你要的 TS4 情况）
                role  = 'giveway';
            else
                scene = "CROSS_LEFT";   % TS 在左舷 -> OS stand-on
                role  = 'standon';
            end
        else
            scene = "OTHERS";
            role  = 'standon';
        end
    end
end

% ---------- info/debug ----------
info.D = D; info.tcpa = tcpa; info.dcpa = dcpa; info.tstar = tstar;
info.beta_deg = rad2deg(beta); info.dC_deg = rad2deg(dC);
info.is_closing = is_closing; info.is_encounter = is_encounter;
info.C_os_ref_deg = rad2deg(C_os_ref); info.C_ts_deg = rad2deg(C_ts);
info.D_enc = D_enc; info.T_enc = T_enc;

if doDbg
    fprintf('   > TS %d: scene=%s role=%s | beta=%.1fdeg dC=%.1fdeg | tcpa=%.2f dcpa=%.2f (enc=%d)\n', ...
        ts.id, char(scene), role, info.beta_deg, info.dC_deg, tcpa, dcpa, is_encounter);
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
        ok = (z <= -1e-9);
    end
end

% ================= episode-lock 记忆与更新（新增核心） =================
function st = srmem_prune(st, t_now, ttl)
if ~isfinite(t_now) || ttl <= 0 || ~isfield(st,'srMem') || isempty(st.srMem)
    return;
end
keep = true(1,numel(st.srMem));
for i=1:numel(st.srMem)
    if ~isfield(st.srMem(i),'lastSeen') || ~isfinite(st.srMem(i).lastSeen)
        continue;
    end
    if (t_now - st.srMem(i).lastSeen) > ttl
        keep(i) = false;
    end
end
st.srMem = st.srMem(keep);
end

function [st, scene_use, role_use] = sr_lock_update(st, os, ts, dp, ...
    scene_raw, role_raw, CR, riskMask, useTS, ...
    cr_enter, cr_exit, confirmSteps, lockOnRisk, lockOnDp, ...
    dp_clear_factor, sep_eps, t_now, doDbg)

if ~isfield(st,'srMem') || isempty(st.srMem)
    st.srMem = struct('id',{},'locked',{},'scene',{},'role',{}, ...
                      'pendScene',{},'pendRole',{},'pendCount',{}, ...
                      'lastSeen',{},'lastCR',{});
end

idx = find([st.srMem.id] == ts.id, 1);
if isempty(idx)
    rec = struct('id',ts.id,'locked',false,'scene',"OTHERS",'role','standon', ...
                 'pendScene',"",'pendRole','', 'pendCount',0, ...
                 'lastSeen',t_now,'lastCR',CR);
    st.srMem(end+1) = rec;
    idx = numel(st.srMem);
end
rec = st.srMem(idx);

rec.lastSeen = t_now;
rec.lastCR   = CR;

RP   = ts.position - os.position;
Vrel = ts.velocity - os.velocity;
D    = norm(RP);

[~, ~, tstar] = compute_dcpa_tcpa(RP, Vrel);
separating = dot(RP, Vrel) > sep_eps;

dp_eff = dp;
if ~isfinite(dp_eff) || dp_eff <= 0
    dp_eff = 1e-3;
end
farEnough = (D > dp_clear_factor * dp_eff);

clear_ok = (CR <= cr_exit) && separating && (tstar < 0 || farEnough);

if rec.locked
    scene_use = rec.scene;
    role_use  = rec.role;

    if clear_ok
        if doDbg
            fprintf('   [SR-LOCK] TS%d UNLOCK (CR=%.2f<=%.2f, sep=1, t*=%.2f, D=%.2f, dp=%.2f)\n', ...
                ts.id, CR, cr_exit, tstar, D, dp_eff);
        end
        rec.locked = false;
        rec.scene  = "OTHERS";
        rec.role   = 'standon';
        rec.pendScene = "";
        rec.pendRole  = "";
        rec.pendCount = 0;

        scene_use = scene_raw;
        role_use  = role_raw;
    end

    st.srMem(idx) = rec;
    return;
end

meaningful = ~strcmp(scene_raw,"OTHERS");

if meaningful
    if strcmp(rec.pendScene, scene_raw) && strcmp(rec.pendRole, role_raw)
        rec.pendCount = rec.pendCount + 1;
    else
        rec.pendScene = scene_raw;
        rec.pendRole  = role_raw;
        rec.pendCount = 1;
    end
else
    rec.pendCount = max(0, rec.pendCount - 1);
end

lock_now = false;
reason = '';

if meaningful && rec.pendCount >= max(1,confirmSteps)
    lock_now = true; reason = sprintf('confirm=%d', rec.pendCount);
end
if ~lock_now && meaningful && lockOnRisk && riskMask && useTS && (CR >= cr_enter)
    lock_now = true; reason = sprintf('risk&CR>=enter(%.2f)', cr_enter);
end
if ~lock_now && meaningful && lockOnDp && (D <= dp_eff)
    lock_now = true; reason = 'D<=dp';
end

if lock_now
    rec.locked = true;
    rec.scene  = scene_raw;
    rec.role   = role_raw;

    scene_use  = rec.scene;
    role_use   = rec.role;

    if doDbg
        fprintf('   [SR-LOCK] TS%d LOCK scene=%s role=%s (%s). CR=%.2f D=%.2f dp=%.2f\n', ...
            ts.id, char(rec.scene), rec.role, reason, CR, D, dp_eff);
    end
else
    scene_use = scene_raw;
    role_use  = role_raw;
end

st.srMem(idx) = rec;
end

% ================== 其余工具函数（保持） ==================
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
ix = min(ix, numel(cand));
Lc=starts(cand(ix)); Rc=ends(cand(ix));
L=mod(Lc-1,N)+1;
R=mod(Rc-1,N)+1;
if R<L, R=R+N; end
end

function tf = any_is_in_gcco(C, ob)
tf = false;
if all(isfinite([ob.gAl, ob.gBl])) && is_between_circ(C, ob.gAl, ob.gBl), tf = true; return; end
if all(isfinite([ob.gAr, ob.gBr])) && is_between_circ(C, ob.gAr, ob.gBr), tf = true; return; end
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

% ===== 3DoF 扩展（沿用你的版本） =====
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
    C = atan2(sin( nearB + dirSign*m*scanStep ), cos( nearB + dirSign*m*scanStep ));
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
    psi = atan2(sin(psi + r*dt), cos(psi + r*dt));
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

% ===== Assist Rule mask（沿用你的版本）=====
function ok = assist_rule_mask(Ccand, v_ts, Vsea_loc, RVo, RVm)
v_c  = Vsea_loc*[cos(Ccand), sin(Ccand)];
RVOS = v_c - v_ts;
lhs = cos_between(RVo, RVOS) - cos_between(RVo, RVm);
ok = (lhs > 1e-6);
end
