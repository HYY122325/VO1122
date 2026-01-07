function newVelocity = computeVO_Step5H(agent, neighbors, params)  % 离散
%从“只要在圈内就全部危险” → “只屏蔽真正会再次逼近的速度”
% computeVO_Step5G — 论文对齐 + 工程修正版
% 修正版要点：
%   1) 3-DoF 前向仿真使用【目标船绝对速度 Vts = nb.velocity】和【初始相对位矢 RP0】。
%   2) expand_one_side_3dof 与 forward_safe_3dof 接口新增 RP0、Vts。
%   3) DCPA/TCPA 统一用 compute_dcpa_tcpa 返回的 tstar（有符号 CPA 时间）。
%   4) Past & Clear：用 CR<thr 判“驶过让清”，并加入 CR+距离 的“安全忘记”机制。
%   5) 航路保持代价：
%      U_route(C) = ω·(角度差/π) + ω_ct·Δd_path_norm(C)
%      其中 Δd_path_norm(C) 是“沿 C 走一小段时间后，对起点→终点基线的横向偏移
%      比现在多出来的那一部分”归一化到 [0,1]。
%   6) FSM：按论文 TC-1/TC-2/TC-3 + Action-1/3/4/5 思想重写，
%      - ΔCTS 使用对参考航向的累计变化（参考在 OS 开始避让/进入近域时冻结）
%      - Action-1/3/4/5 在本帧中全局移除所有 TS 的原始 COLREG 约束，
%        再对问题 TS 单独施加新的几何约束。
%      - 加入观测期 μ + C-1/C-2 + Action-2（不改 COLREGS，只重新优化）。

%% =========== 参数与默认 ===========
s5       = getfieldwithdef(params, 'step5', struct());
useGW    = getfieldwithdef(params.lvo, 'useGoodwin', true);
ds_base  = params.lvo.ds_base;
dp_factor= params.lvo.dp_factor;
hdgGridN = getfieldwithdef(params.lvo,'hdgGridN',360);
hdgGrid  = linspace(-pi, pi, hdgGridN);

% 后端代价权重
gamma      = getfieldwithdef(params.lvo,'gamma',2.5);
lambda_pts = getfieldwithdef(params.lvo,'lambda_pts',1.0);
omega      = getfieldwithdef(s5,'route_w',1.0);              % 航向角度偏离权重
omega_ct   = getfieldwithdef(s5,'route_ct_w',omega);         % 横向偏移权重（默认同 omega）
useCOLREGS = getfieldwithdef(s5,'useCOLREGS',true);
useSeaman  = getfieldwithdef(s5,'useSeamanship',true);
useFSM     = getfieldwithdef(s5,'useFSM',true);

% FSM 相关参数（观测期 μ = TCPA_TS / sigma，离散步长 fsm_dt 视为决策周期）
fsm_sigma  = getfieldwithdef(s5,'fsm_sigma',25.0);
fsm_dt     = getfieldwithdef(s5,'fsm_dt',1.0);   % 默认 1s

% 横向偏移预测时间 + 归一化尺度
route_ct_T     = getfieldwithdef(s5,'route_ct_horizon',5.0);           % 看 5s 后偏移变化
route_ct_scale = getfieldwithdef(s5,'route_ct_scale',2.0*ds_base*dp_factor);

% 3-DoF/GCCO 扩展参数
scanStep  = deg2rad(getfieldwithdef(s5,'scanStepDeg',1.0));
scanMax   = deg2rad(getfieldwithdef(s5,'scanMaxDeg',45.0));
predTime  = getfieldwithdef(s5,'predTime',20.0);
dt_chk    = getfieldwithdef(s5,'dtCheck',0.1);
Tr        = getfieldwithdef(s5,'yaw_T',  5.0);
Kdel      = getfieldwithdef(s5,'yaw_K',  1.0);
Tu        = getfieldwithdef(s5,'u_T',   15.0);
Dv        = getfieldwithdef(s5,'v_D',    0.8);
Alpha     = getfieldwithdef(s5,'uv_alpha',0.2);
Kp        = getfieldwithdef(s5,'Kp',     1.6);
Kd        = getfieldwithdef(s5,'Kd',     0.4);
deltaMax  = deg2rad(getfieldwithdef(s5,'delta_max_deg',35));
deltaRate = deg2rad(getfieldwithdef(s5,'delta_rate_deg',10));
aMax      = getfieldwithdef(s5,'a_max',  1);

perceptionDist = params.agent.perceptionDist;
T              = agent.timeHorizon;

% 速度（海速）：保持模长不变
Vsea   = max(1e-3, norm(agent.prefVelocity));
Cprev  = headingFromVelocity(agent);
toGoal = agent.goal - agent.position;
distToGoal = norm(toGoal); %#ok<NASGU>   % 目前未用，将来可加速度规划
CLOS   = atan2(toGoal(2), toGoal(1));

% 持久化 seamanship/FSM 状态（这里也顺便存“路径起点”）
persistent S5STATE
if isempty(S5STATE), S5STATE = struct('ids',[], 'data',[]); end
st = get_agent_state();

% 若还没记录路径起点，则第一帧记录为当前坐标
if ~isfield(st,'pathOrigin') || any(~isfinite(st.pathOrigin))
    st.pathOrigin = agent.position;
end

% === 计算当前对“起点→终点基线”的横向偏移（d_path_now） ===
haveBase = all(isfinite(st.pathOrigin)) && (norm(agent.goal - st.pathOrigin) > 1e-6);
if haveBase
    base_vec = agent.goal - st.pathOrigin;
    base_dir = base_vec / norm(base_vec);     % 起点→终点的单位向量
    n_base   = [-base_dir(2), base_dir(1)];   % 左法向
    rel_now  = agent.position - st.pathOrigin;
    d_path_now = abs(dot(rel_now, n_base));   % 当前横向偏移（标量）
else
    n_base      = [0,0];
    d_path_now  = 0;
end

%% =========== 前端：扫描 CCO/BCO 与 3-DoF 扩展 GCCO/GBCO ===========
obs = repmat(struct('Al',NaN,'Ar',NaN,'Bl',NaN,'Br',NaN, ...
                    'gAl',NaN,'gAr',NaN,'gBl',NaN,'gBr',NaN, ...
                    'scene','OTHERS','role','standon','colregsMask',@(C)true), ...
             numel(neighbors),1);

for j = 1:numel(neighbors)
    nb = neighbors(j);

    in_core = false(1, hdgGridN);
    in_bco  = false(1, hdgGridN);

    for k = 1:hdgGridN
        C = hdgGrid(k);
        v_scan = Vsea * [cos(C), sin(C)];        % OS 候选速度
        RP     = nb.position - agent.position;   % TS-OS

        % 相对速度：RP = P_TS - P_OS，则 RV = V_TS - V_OS
        RV     = nb.velocity - v_scan;           % [RV 统一约定]

        [ds, dp] = compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
        [hitCore, ~]  = dcpaTcpaHit(RP, RV, ds, T, perceptionDist);
        [hitBroad, ~] = dcpaTcpaHit(RP, RV, dp, T, perceptionDist);

        in_core(k) = hitCore;
        in_bco(k)  = (~hitCore) && hitBroad;
    end

    [Al,Ar,Bl,Br] = local_find_intervals(hdgGrid, in_core, in_bco);
    obs(j).Al=Al; obs(j).Ar=Ar; obs(j).Bl=Bl; obs(j).Br=Br;

    % ---- 3-DoF 扩展：GCCO/GBCO ----
    RP0 = nb.position - agent.position;   % TS 初始相对位矢
    Vts = nb.velocity;                    % TS 绝对速度
    [ds_ref, dp_ref] = compute_ds_dp_paper(RP0, Cprev, ds_base, dp_factor, useGW);

    if all(isfinite([Al,Bl])) % 左侧
        obs(j).gBl = expand_one_side_3dof(Bl, Al, -1, dp_ref, ...
            agent, RP0, Cprev, Vsea, Vts, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
        obs(j).gAl = expand_one_side_3dof(Al, Bl, -1, ds_ref, ...
            agent, RP0, Cprev, Vsea, Vts, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end
    if all(isfinite([Ar,Br])) % 右侧
        obs(j).gBr = expand_one_side_3dof(Br, Ar, +1, dp_ref, ...
            agent, RP0, Cprev, Vsea, Vts, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
        obs(j).gAr = expand_one_side_3dof(Ar, Br, +1, ds_ref, ...
            agent, RP0, Cprev, Vsea, Vts, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end
end

%% =========== 前端：CR + 场景/角色 + COLREGS 掩膜 ===========
[CR, riskMask] = compute_frontend_CR_paper(agent, neighbors, params, Cprev);

% 基于 CR + 距离 的“安全忘记”掩膜：useTS(j)=false 表示这艘船已驶过且足够远
Nnb   = numel(neighbors);
useTS = true(1, Nnb);
thrCR = params.lvo.cr_threshold;
for j = 1:Nnb
    nb = neighbors(j);
    RP = nb.position - agent.position;
    [~, dpj] = compute_ds_dp_paper(RP, Cprev, ds_base, dp_factor, useGW);
    D = norm(RP);
    if (CR(j) < thrCR) && (D > 2*dpj)
        useTS(j) = false;   % 认为已驶过让清，不再参与 GCCO / Umulti
    end
end

for j=1:numel(neighbors)
    [scene, role] = classify_scene_role_paper(agent, neighbors(j), Cprev);
    obs(j).scene = scene; obs(j).role = role;

    obs(j).colregsMask = @(C) true;
    if useCOLREGS
        obs(j).colregsMask = build_colregs_mask_paper(agent, neighbors(j), scene, role);
    end
end

% FSM 覆盖（仍对所有邻船运行，但后端会用 useTS 掩膜筛选）
if useFSM && ~isempty(neighbors)
    st = fsm_update(st, agent, neighbors, obs, ...
                    fsm_sigma, deg2rad(5.0), Cprev, CLOS, ...
                    ds_base, dp_factor, useGW, fsm_dt);

    % 1) 若处于 Action-1/3/4/5，先“全局移除 COLREG 约束”
    if isfield(st,'colregs_mode') && st.colregs_mode ~= 0
        for j2 = 1:numel(obs)
            obs(j2).colregsMask = @(C) true;
        end
    end

    % 2) 再对“问题 TS”单独加新的几何限制
    if isfield(st,'overrides') && ~isempty(st.overrides)
        for k = 1:numel(st.overrides)
            idx = st.overrides(k).j;
            if idx>=1 && idx<=numel(obs)
                obs(idx).colregsMask = st.overrides(k).mask;
            end
        end
    end
end

%% =========== 后端：Usum = Umulti + Uroute 搜索最优航向 ===========
bestHdg  = Cprev;
bestCost = inf;

for k = 1:hdgGridN
    C = hdgGrid(k);

    % 可行性：不得落在任一 GCCO；不得违反 COLREGS（可能被 FSM 修改）
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

    % Umulti(C)：仅对“尚未驶过”的 TS 计算
    U_multi = 0;
    for j = 1:numel(neighbors)
        if ~useTS(j), continue; end
        if getfieldwithdef(s5,'useGBCOinPTS',true)
            Al = obs(j).gAl; Ar = obs(j).gAr; Bl = obs(j).gBl; Br = obs(j).gBr;
        else
            Al = obs(j).Al;  Ar = obs(j).Ar;  Bl = obs(j).Bl;  Br = obs(j).Br;
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

    % ========== Uroute(C)：角度 + “比现在更偏航路多少” ==========
    % 角度部分：与 LOS 航向的偏差
    theta_d = abs(atan2(sin(C - CLOS), cos(C - CLOS)));
    U_route_ang  = omega * (theta_d / pi);

    % 横向偏移部分
    if haveBase
        v_dir    = [cos(C), sin(C)];
        P_pred   = agent.position + Vsea * route_ct_T * v_dir;
        rel_pred = P_pred - st.pathOrigin;
        d_path_pred = abs(dot(rel_pred, n_base));       % 预测横向偏移
        delta_d  = max(0, d_path_pred - d_path_now);    % 比现在更偏多少
        e_path_norm = min(delta_d / max(1e-3, route_ct_scale), 1.0);
    else
        e_path_norm = 0;
    end
    U_route_path = omega_ct * e_path_norm;

    U_route = U_route_ang + U_route_path;

    % 总代价
    U_sum = U_multi + U_route;
    if U_sum < bestCost
        bestCost = U_sum; bestHdg = C;
    end
end

%% =========== Seamanship：Past & Clear + 保持规则 ===========
% Past & Clear：所有邻船 CR<thr → 驶过让清，直接回到 LOS 航向
if ~isempty(neighbors)
    if all(CR < thrCR) && isfinite(CLOS)
        bestHdg = CLOS;
    end
end

% 保持规则：若两次同向且新角度更小，则保持上一目标航向
if useSeaman && isfield(st,'prevObjHdg') && isfinite(st.prevObjHdg)
    dC0 = angdiff_signed(st.prevObjHdg, st.prevStartHdg);
    dC1 = angdiff_signed(bestHdg,      st.prevStartHdg);
    if sign(dC0)*sign(dC1) > 0 && abs(dC1) < abs(dC0)
        bestHdg = st.prevObjHdg;
    end
end

% 输出速度向量：海速 + 最优航向
newVelocity = Vsea * [cos(bestHdg), sin(bestHdg)];

% 更新状态（包含 pathOrigin）
st.prevObjHdg   = bestHdg;
st.prevStartHdg = Cprev;
set_agent_state(st);

%% ==================== 内部函数（嵌套） ====================

    function st2 = get_agent_state()
        % 扩展状态：加入 tsHeadings / tsHead0 / colregs_mode / FSM 缓存
        st2 = struct('prevObjHdg', NaN, 'prevStartHdg', NaN, ...
                     'tObsEnd', -inf, 'phase', "IDLE", 'overrides', [], ...
                     'pathOrigin', [NaN,NaN], ...
                     'tsHeadings', [], 'tsHead0', [], ...
                     'tsPhase', [], 'tsObsLeft', [], ...
                     'tsRVo', zeros(0,2), 'tsRVopt', zeros(0,2), ...
                     'colregs_mode', 0);
        k = find(S5STATE.ids==agent.id,1);
        if isempty(k)
            S5STATE.ids(end+1)   = agent.id;
            S5STATE.data{end+1}  = st2;
        else
            st2 = S5STATE.data{k};
            if ~isfield(st2,'pathOrigin'),    st2.pathOrigin = [NaN,NaN]; end
            if ~isfield(st2,'tsHeadings'),    st2.tsHeadings = [];        end
            if ~isfield(st2,'tsHead0'),       st2.tsHead0 = [];           end
            if ~isfield(st2,'tsPhase'),       st2.tsPhase = [];           end
            if ~isfield(st2,'tsObsLeft'),     st2.tsObsLeft = [];         end
            if ~isfield(st2,'tsRVo'),         st2.tsRVo = zeros(0,2);     end
            if ~isfield(st2,'tsRVopt'),       st2.tsRVopt = zeros(0,2);   end
            if ~isfield(st2,'colregs_mode'),  st2.colregs_mode = 0;       end
        end
    end

    function set_agent_state(st2)
        k = find(S5STATE.ids==agent.id,1);
        S5STATE.data{k} = st2;
    end

    % ---- ds(B) 与 dp ----
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

    % ---- CR 前端 ----
    function [CRval, riskMask] = compute_frontend_CR_paper(os, nbs, prm, osHdg)
        N = numel(nbs);
        CRval   = zeros(1,N);
        riskMask= false(1,N);

        w   = getfieldwithdef(prm.lvo, 'cr_w', [0.40, 0.367, 0.167, 0.033, 0.033]);
        thr = getfieldwithdef(prm.lvo, 'cr_threshold',0.5);

        for jj=1:N
            ts = nbs(jj);
            RP = ts.position - os.position;

            % Vrel = V_TS - V_OS（与 compute_dcpa_tcpa 约定一致）
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
        if dcpa <= ds, u = 1;
        elseif dcpa <= dp
            x = (dcpa - ds)/max(1e-6,(dp-ds));
            u = 0.5 - 0.5*sin(pi*(x - 0.5));
        else, u = 0;
        end
    end

    function u = U_TCPA_paper(tcpa_signed, dcpa, ds, dp, Vrel)
        if Vrel < 1e-8, u = 0; return; end
        if tcpa_signed < 0
            u = 0; return;
        end
        tcpa = tcpa_signed;

        if dcpa <= ds
            t1 = sqrt(max(ds^2 - dcpa^2,0))/Vrel;
        else
            t1 = (ds - dcpa)/Vrel; t1 = max(0,t1);
        end
        t2 = sqrt(max(dp^2 - min(dcpa,dp)^2,0))/Vrel;
        t2 = max(t2, t1 + 1e-6);

        if tcpa <= t1, u = 1;
        elseif tcpa <= t2
            x = (tcpa - t1)/(t2 - t1); u = (1 - x)^2;
        else, u = 0;
        end
    end

    function u = U_D_paper(D, ds, dp)
        if D <= ds, u = 1;
        elseif D <= dp
            x = (D - ds)/max(1e-6,(dp-ds)); u = (1 - x);
        else, u = 0;
        end
    end

    function u = U_B_paper(B)
        b = B - 19*pi/180;
        u = 0.5*( cos(b) + sqrt( 440 / (289 + cos(b)^2) ) ) - 5/17;
        u = max(0, min(1, u));
    end

    function u = U_K_sym(K)
        K_safe = max(K, 1e-6);
        tau = log(K_safe);
        u = 1 - 1/(1 + exp(-tau));
        u = max(0, min(1, u));
    end

    % ---- 场景/角色（保持你原来的“交集”版本）----
    function [scene, role] = classify_scene_role_paper(os, ts, osHdg)
        VOS = norm(os.velocity); VTS = norm(ts.velocity);
        Cts = atan2(ts.velocity(2), ts.velocity(1));
        B   = atan2(ts.position(2)-os.position(2), ts.position(1)-os.position(1));
        Q   = wrapToPi_loc(B - osHdg);
        RC  = wrapToPi_loc( atan2( VTS*sin(Cts) - VOS*sin(osHdg), ...
                                   VTS*cos(Cts) - VOS*cos(osHdg) ) );
        scene = "OTHERS"; role = 'standon';

        if (abs(Q) <= pi/8) && (abs(RC) >= 3*pi/4)
            scene = "HEAD_ON";  role='giveway';
        elseif abs(Q) >= 5*pi/8
            scene = "OVERTAKEN"; role='standon';
        elseif (Q >  pi/8) && (Q <= 5*pi/8) && (abs(RC) >= pi/8) && (abs(RC) <= 5*pi/8)
            scene = "CROSS_RIGHT"; role='giveway';
        elseif (Q < -pi/8) && (Q >= -5*pi/8) && (abs(RC) >= pi/8) && (abs(RC) <= 5*pi/8)
            scene = "CROSS_LEFT"; role='standon';
        end

        if (abs(Q) <= 3*pi/8) && (VOS > VTS)
            scene = "OVERTAKING"; role='giveway';
        end
    end

    % ---- COLREGS 掩膜 ----
    function mask = build_colregs_mask_paper(os, ts, scene, role)
        mask = @(C) true;
        if ~ismember(scene, ["HEAD_ON","CROSS_RIGHT"]) || ~strcmpi(role,'giveway')
            return;
        end
        mask = @(C) colregs_ok(os, ts, C);
        function ok = colregs_ok(os_, ts_, Ccand)
            RP = ts_.position - os_.position;
            v_cand = Vsea*[cos(Ccand), sin(Ccand)];
            RV = v_cand - ts_.velocity;
            z = RP(1)*RV(2) - RP(2)*RV(1);
            ok = (z <= -1e-9);   % 强制右转
        end
    end

    % ---- FSM：按论文思想重写 + 观测期 μ + C-1/C-2 + ΔCTS 参考修正 ----
    function stx = fsm_update(stx, os, nbs, obss, sigma, deg5_, osHdg, CLOS_loc, ds0, dp_fac, useGW_, fsm_dt_)
        N = numel(nbs);
        stx.overrides    = [];
        stx.colregs_mode = 0;   % 0=正常;1=Action-1;2=Action-3/4;3=Action-5

        % ------- 保证缓存数组长度足够（支持多 TS） -------
        if ~isfield(stx,'tsHeadings') || numel(stx.tsHeadings) < N
            tmp = nan(1,N);
            if isfield(stx,'tsHeadings')
                tmp(1:numel(stx.tsHeadings)) = stx.tsHeadings;
            end
            stx.tsHeadings = tmp;
        end
        if ~isfield(stx,'tsHead0') || numel(stx.tsHead0) < N
            tmp = nan(1,N);
            if isfield(stx,'tsHead0')
                tmp(1:numel(stx.tsHead0)) = stx.tsHead0;
            end
            stx.tsHead0 = tmp;
        end
        if ~isfield(stx,'tsPhase') || numel(stx.tsPhase) < N
            tmp = zeros(1,N);
            if isfield(stx,'tsPhase')
                tmp(1:numel(stx.tsPhase)) = stx.tsPhase;
            end
            stx.tsPhase = tmp;   % 0=IDLE,1=OBSERVE,2=ACTION1_DONE
        end
        if ~isfield(stx,'tsObsLeft') || numel(stx.tsObsLeft) < N
            tmp = zeros(1,N);
            if isfield(stx,'tsObsLeft')
                tmp(1:numel(stx.tsObsLeft)) = stx.tsObsLeft;
            end
            stx.tsObsLeft = tmp;
        end
        if ~isfield(stx,'tsRVo') || size(stx.tsRVo,1) < N
            new = nan(N,2);
            if isfield(stx,'tsRVo')
                new(1:size(stx.tsRVo,1),:) = stx.tsRVo;
            end
            stx.tsRVo = new;
        end
        if ~isfield(stx,'tsRVopt') || size(stx.tsRVopt,1) < N
            new = nan(N,2);
            if isfield(stx,'tsRVopt')
                new(1:size(stx.tsRVopt,1),:) = stx.tsRVopt;
            end
            stx.tsRVopt = new;
        end

        Vsea_loc = Vsea;  % 使用入口定义的海速模长

        for jj = 1:N
            ts = nbs(jj);

            % --- 距离 / ds,dp ---
            RP = ts.position - os.position;
            [dsj, dpj] = compute_ds_dp_paper(RP, osHdg, ds0, dp_fac, useGW_);
            D = norm(RP);

            % --- TS 航向缓存 ---
            Cts_now  = atan2(ts.velocity(2), ts.velocity(1));
            Cts_prev = Cts_now;
            if isfinite(stx.tsHeadings(jj))
                Cts_prev = stx.tsHeadings(jj);
            end

            % --- ΔCTS 的参考：尽量贴近“OS 开始避让时刻”的 TS 航向 ---
            isAvoidingNow = false;
            if isfield(stx,'prevObjHdg') && isfinite(stx.prevObjHdg) && isfinite(CLOS_loc)
                isAvoidingNow = abs(angdiff_signed(stx.prevObjHdg, CLOS_loc)) > deg2rad(1.0);
            end

            % 1) 如果 OS 已经进入避碰（相对 LOS 偏离>1°），且尚未记录参考航向 → 锁定
            if isAvoidingNow && ~isfinite(stx.tsHead0(jj))
                stx.tsHead0(jj) = Cts_prev;
            end

            % 2) 若仍未锁定，但距离已进入 dp 范围 → 用第一次观测作为参考
            if ~isfinite(stx.tsHead0(jj)) && (D <= dpj)
                stx.tsHead0(jj) = Cts_prev;
            end

            if isfinite(stx.tsHead0(jj))
                dCTS_total = abs(angdiff_signed(Cts_now, stx.tsHead0(jj)));
            else
                dCTS_total = 0;
            end

            scene = obss(jj).scene;
            role  = obss(jj).role;

            % 相对速度（OS-TS / TS-OS）
            RV_osTs = os.velocity - ts.velocity;  % OS - TS
            RV_tsOs = ts.velocity - os.velocity;  % TS - OS (给 DCPA/TCPA)

            % 若还未有 RVo 快照，则用当前 OS-TS 作为初始 RVo 近似
            if ~all(isfinite(stx.tsRVo(jj,:)))
                stx.tsRVo(jj,:) = RV_osTs;
            end

            [~, ~, tstar] = compute_dcpa_tcpa(RP, RV_tsOs);  % tstar ≈ TCPA_TS（有符号）

            % ------------------- OS 为 give-way 的分支（TC-1 / C-1 / C-2） -------------------
            if strcmpi(role,'giveway')
                ph = stx.tsPhase(jj);

                % --- phase 0：正常状态，检查是否满足 TC-1 进入“观察期 μ” ---
                if ph == 0
                    if dCTS_total > deg5_
                        RVo   = RV_osTs;    % 初始相对速度（近似）
                        RVopt = RV_osTs;
                        if isfield(stx,'prevObjHdg') && isfinite(stx.prevObjHdg)
                            vopt  = Vsea_loc * [cos(stx.prevObjHdg), sin(stx.prevObjHdg)];
                            RVopt = vopt - ts.velocity;   % OS 期望避让航向对应相对速度
                        end
                        RVm = RV_osTs;       % 当前相对速度视作 TS 已动作后的 RVm

                        c1 = dot(RVo, RVopt) / max(1e-9, norm(RVo)*norm(RVopt));
                        c2 = dot(RVo, RVm  ) / max(1e-9, norm(RVo)*norm(RVm));

                        isRm = (c1 - c2) < 0;

                        if isRm
                            mu = max(0, tstar) / max(1e-6, sigma);   % μ = TCPA_TS / σ
                            stx.tsPhase(jj)   = 1;                   % 1 = OBSERVE
                            stx.tsObsLeft(jj) = mu;
                            stx.tsRVo(jj,:)   = RVo;                 % 更新 RVo 快照
                            stx.tsRVopt(jj,:) = RVopt;
                        end
                    end

                % --- phase 1：OBSERVE 状态，按 μ 倒计时，结束时判 C-1 / C-2 ---
                elseif ph == 1
                    if ~isfinite(stx.tsObsLeft(jj))
                        stx.tsObsLeft(jj) = 0;
                    end
                    stx.tsObsLeft(jj) = stx.tsObsLeft(jj) - fsm_dt_;
                    if stx.tsObsLeft(jj) <= 0
                        RVo   = stx.tsRVo(jj,:);
                        RVopt = stx.tsRVopt(jj,:);
                        RVpre = RV_osTs;   % 观测期结束时的相对速度

                        % 计算 (RVo × RVpre)_z 与 (RVo × RVopt)_z
                        z1 = RVo(1)*RVpre(2) - RVo(2)*RVpre(1);
                        z2 = RVo(1)*RVopt(2) - RVo(2)*RVopt(1);

                        sameRV = (norm(RVo-RVpre) < 1e-3);
                        isC1   = sameRV || (z1 * z2 < 0);

                        if isC1
                            % ========== Action-1：强制 TS 从“正确侧”避让 ==========
                            stx.colregs_mode = 1;
                            stx.overrides(end+1).j  = jj; %#ok<AGROW>
                            stx.overrides(end).mask = @(C) flip_mask(os, ts, C, -1);
                            stx.tsPhase(jj)   = 2;    % ACTION-1 已执行
                            stx.tsObsLeft(jj) = 0;
                        else
                            % ========== C-2：协同但不充分 → Action-2 ==========
                            % Action-2：只重新跑多阶段优化，不额外改 COLREGS
                            stx.tsPhase(jj)   = 0;    % 回到 IDLE
                            stx.tsObsLeft(jj) = 0;
                        end
                    end
                end
            end  % give-way 分支结束

            % ------------------- OS 为 stand-on 的分支（TC-2 / TC-3） -------------------
            if strcmpi(role,'standon') && (D <= dpj)
                in_gcco = any_is_in_gcco(osHdg, obss(jj));

                % TC-2：D ≤ dp & ΔCTS_total < 5° → OS 视为接管避让
                if dCTS_total < deg5_
                    if strcmpi(scene,'CROSS_LEFT')
                        stx.colregs_mode = max(stx.colregs_mode, 2);
                        stx.overrides(end+1).j  = jj; %#ok<AGROW>
                        stx.overrides(end).mask = @(C) flip_mask(os, ts, C, -1);
                    elseif strcmpi(scene,'OVERTAKEN')
                        stx.colregs_mode = max(stx.colregs_mode, 2);
                        stx.overrides(end+1).j  = jj; %#ok<AGROW>
                        stx.overrides(end).mask = @(C) mask_overtaken(os, ts, C, osHdg);
                    end

                % TC-3：D ≤ dp & ΔCTS_total > 5° & COS ∈ GCCO → Action-5 协同避碰
                elseif dCTS_total > deg5_ && in_gcco
                    stx.colregs_mode = max(stx.colregs_mode, 3);

                    % RVo：用之前冻结的 OS-TS 相对速度快照，若还没则退回当前
                    RVo = stx.tsRVo(jj,:);
                    if ~all(isfinite(RVo))
                        RVo = RV_osTs;
                    end
                    % RVTS：触发 TC-3 时刻的 OS-TS 相对速度
                    RVTS = RV_osTs;

                    stx.overrides(end+1).j  = jj; %#ok<AGROW>
                    stx.overrides(end).mask = @(C) assist_rule(os, ts, C, RVo, RVTS);
                end
            end

            % ---- 更新当前帧 TS 航向缓存 ----
            stx.tsHeadings(jj) = Cts_now;
        end
    end

    % ---- FSM 用到的几何掩膜 ----
    function ok = flip_mask(os, ts, C, sgn)
        RP = ts.position - os.position;
        v_c = Vsea*[cos(C), sin(C)];
        RV  = v_c - ts.velocity;
        z   = RP(1)*RV(2) - RP(2)*RV(1);
        if sgn>0, ok = (z >= 0); else, ok = (z <= 0); end
    end

    function ok = mask_overtaken(os, ts, C, osHdg_)
        Q  = atan2(ts.position(2)-os.position(2), ts.position(1)-os.position(1)) - osHdg_;
        RP = ts.position - os.position;
        v_c = Vsea*[cos(C), sin(C)];
        RV  = v_c - ts.velocity;
        z   = RP(1)*RV(2) - RP(2)*RV(1);
        if Q > 0   % TS 在右舷
            ok = (z <= 0);
        else       % TS 在左舷
            ok = (z >= 0);
        end
    end

    function ok = assist_rule(os, ts, C, RVo, RVTS)
        % 协同避碰 Am 的候选集合条件（Eq.(41) 的离散近似）
        v_c  = Vsea*[cos(C), sin(C)];
        RVOS = v_c - ts.velocity;   % OS(cand) - TS，仍然是 OS-TS 方向

        lhs = dot(RVo, RVOS)/max(1e-9,norm(RVo)*norm(RVOS)) ...
            -  dot(RVo, RVTS)/max(1e-9,norm(RVo)*norm(RVTS));
        ok = (lhs < 0);
    end

end  % === computeVO_Step5G 主体结束 ===

%% ==================== 通用子函数 ====================
function val = getfieldwithdef(s, name, def)
if isfield(s,name), val=s.(name); else, val=def; end
end

function [dcpa, tcpa, tstar] = compute_dcpa_tcpa(RP, RV)
a = dot(RV,RV);
if a < 1e-10
    tstar = inf;
    tcpa  = inf;
    dcpa  = norm(RP);
    return;
end
tstar = -dot(RP,RV)/a;
tcpa  = max(0, tstar);
dcpa  = norm(RP + RV*tcpa);
end

% =================== [MOD] 改进的 dcpaTcpaHit =====================
function [hit, ttc] = dcpaTcpaHit(RP, RV, radius, timeHorizon, perceptionDist)
a = dot(RV, RV);
r0 = norm(RP);

% 超出感知距离，直接视为无碰撞机会
if r0 > perceptionDist
    hit = false; ttc = inf; return;
end

% 相对速度几乎为 0：静止/同步航行
if a < 1e-8
    % 距离非常近就视为“当前已经在碰撞域内”
    if r0 <= 1.2*radius
        hit = true;  ttc = 0;
    else
        hit = false; ttc = inf;
    end
    return;
end

dotrv = dot(RP, RV);
approaching = (dotrv < 0);

% 已经在远离（包括会遇之后、擦肩而过之后） → 不再认为有未来碰撞
if ~approaching
    hit = false; ttc = inf;
    return;
end

% 逼近方向：计算 TCPA，并限制在 [0, timeHorizon]
tcpa0 = - dotrv / a;
if tcpa0 > timeHorizon
    hit = false; ttc = inf;
    return;
end

tcpa = max(0, tcpa0);
dcpa = norm(RP + RV * tcpa);

hit = (dcpa <= radius);
if hit
    ttc = tcpa;
else
    ttc = inf;
end
end
% =================== [MOD] 结束 =====================

function hdg = headingFromVelocity(agent)
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

% === 航向区间工具/3-DoF 工具 ===
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

% === 修改点 ===
% 用 longest_run_circ 只取 in_core 里“最长的一段连续区间”作为核心扇区，
% 避免多个 core 碎片被一股脑包起来。
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
L=mod(Lc-1,N)+1; R=mod(Rc-1,N)+1;
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
Lc=starts(cand(ix)); Rc=ends(cand(ix));
L=mod(Lc-1,N)+1; R=mod(Rc-1,N)+1;
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
    agent, RP0, hdg0, u_ref, Vts, ...
    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
    t_pred, dt, scanStep, scanMax)

gNear = nearB;
stepMax = ceil(scanMax / scanStep);
u0 = max(1e-3, norm(agent.velocity));
psi0 = hdg0; v0 = 0; r0 = 0; delta0 = 0;

for m = 0:stepMax
    C = wrapToPi_loc( nearB + dirSign*m*scanStep );
    if dirSign < 0
        if ang_passed(C, farB, nearB, -1), break; end
    else
        if ang_passed(C, nearB, farB, +1), break; end
    end

    if forward_safe_3dof(RP0, [0,0], [u0,v0,r0,psi0,delta0], u_ref, Vts, C, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            t_pred, dt, radius)
        gNear = C; break;
    end
end
end

% =================== [MOD] 改进的 3DoF 判安全 =====================
function yes = forward_safe_3dof(RP0, P0_os, X0_os, u_ref, Vts, C, ...
    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, t_pred, dt, radius)

u = X0_os(1); v = X0_os(2); r = X0_os(3); psi = X0_os(4); delta = X0_os(5);
Pos = P0_os(:).';
Pnb0 = RP0(:).'; Vts  = Vts(:).';
t = 0; yes = true;
while t <= t_pred
    Pnb = Pnb0 + Vts * t;
    d = norm(Pnb - Pos);

    % 关键修改：只对 t>0 检查是否进入安全圈
    % 允许“当前已经略微侵入，但下一瞬间开始往外逃”的候选速度被视作可行
    if t > 0 && d < radius - 1e-6
        yes = false; return;
    end

    epsi = atan2(sin(C - psi), cos(C - psi));
    delta_cmd = Kp*epsi + Kd*(-r);
    delta_cmd = max(-deltaMax, min(deltaMax, delta_cmd));
    step_delta = max(-deltaRate*dt, min(deltaRate*dt, delta_cmd - delta));
    delta = delta + step_delta;

    r_dot = (Kdel*delta - r)/max(1e-6,Tr);
    u_dot = (u_ref - u)/max(1e-6,Tu);  u_dot = max(-aMax, min(aMax, u_dot));
    v_dot = -Dv*v + Alpha*u*r;
    r = r + r_dot*dt; u = u + u_dot*dt; v = v + v_dot*dt; psi = wrapToPi_loc(psi + r*dt);
    Pos = Pos + [u*cos(psi) - v*sin(psi), u*sin(psi) + v*cos(psi)] * dt;
    t = t + dt;
end
end
% =================== [MOD] 结束 =====================

function tf = ang_passed(C, L, R, dirSign)
if dirSign < 0, tf = angleLess(C, L) | angleGreater(C, R);
else,            tf = angleGreater(C, R) | angleLess(C, L);
end
end
function tf = angleLess(a,b),    tf = atan2(sin(a-b),cos(a-b)) < 0; end
function tf = angleGreater(a,b), tf = atan2(sin(a-b),cos(a-b)) > 0; end
