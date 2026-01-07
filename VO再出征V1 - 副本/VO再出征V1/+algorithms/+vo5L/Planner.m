function sol = Planner(ctx, obs, st, risk)
% vo5L.Planner — S4：联合优化(C+s) + Past&Clear + Seamanship + Fallback
%
% 输出 sol 字段：
%   bestHdg / bestScale / bestCost / bestBreak / decisionSource
%   lockApplied / optApplied / pcApplied / seaApplied / fbApplied
import algorithms.vo5L.Utils; 
agent     = ctx.agent;
neighbors = ctx.neighbors;

Vpref = ctx.Vpref;
Cprev = ctx.Cprev;
CLOS  = ctx.CLOS;

% ---------- 配置 ----------
hdgGridN = ctx.hdgGridN;
hdgGrid  = ctx.hdgGrid;

speed_scales = ctx.speed_scales;
w_speed      = ctx.w_speed;

lambda_pts = ctx.lambda_pts;
gamma      = ctx.gamma;

omega    = ctx.omega;
omega_ct = ctx.omega_ct;

CR_floor  = ctx.CR_floor;
w_colregs = ctx.w_colregs;

useGCCO_hard = ctx.useGCCO_hard;
gcco_gate_s  = ctx.gcco_gate_s;

allow_stop = ctx.allow_stop;
stop_scale = ctx.stop_scale;

useSeaman = ctx.useSeaman;

T              = ctx.T;
perceptionDist = ctx.perceptionDist;
ds_base        = ctx.ds_base;
dp_factor      = ctx.dp_factor;
useGW          = ctx.useGW;

route_ct_T     = ctx.route_ct_T;
route_ct_scale = ctx.route_ct_scale;

haveBase          = ctx.haveBase;
currentPathOrigin = ctx.currentPathOrigin;
n_base            = ctx.n_base;
d_path_now        = ctx.d_path_now;

CR    = risk.CR;
useTS = risk.useTS;

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
        if ctx.doDebug
            fprintf(' [S4-LOCK] Phase-1 锁定 (C=%.1fdeg, s=%.2f) 通过硬安全+规则，跳过优化\n', ...
                rad2deg(bestHdg), bestScale);
        end
    else
        if ctx.doDebug, fprintf(' [S4-LOCK] 锁定候选不安全/不合规，转入优化。\n'); end
    end
end

% --------- 主优化：硬安全 + 硬规则（默认）---------
if ~lockApplied
    [bestHdg, bestScale, bestCost, bestBreak, found] = solve_search(false);
    optApplied = true;

    if ~found
        fbApplied = true;
        if ctx.doDebug, fprintf(' [S4-FB] 无可行解：进入 Fallback-1（COLREGS 软惩罚）...\n'); end
        [bestHdg, bestScale, bestCost, bestBreak, found2] = solve_search(true);

        if ~found2
            if ctx.doDebug, fprintf(' [S4-FB] Fallback-1 仍无解：进入 Fallback-2（最不坏）...\n'); end
            [bestHdg, bestScale, bestCost, bestBreak] = solve_least_bad();
        end
    end
end

% --------- Past&Clear：切回 LOS（不抢 Phase-1 锁控）---------
if ~lockApplied && ~isempty(neighbors) && risk.allRiskCleared && isfinite(CLOS)
    [okPC, ~] = candidate_feasible_hard(CLOS, 1.0, false);
    if okPC
        bestHdg = CLOS;
        bestScale = 1.0;
        decisionSource = 'PAST_CLEAR_ROUTE';
        pcApplied = true;
        if ctx.doDebug, fprintf(' [S4-PAST&CLEAR] LOS 可行 -> 切回 LOS（C=%.1fdeg,s=1.0）\n', rad2deg(bestHdg)); end
    else
        if ctx.doDebug, fprintf(' [S4-PAST&CLEAR] LOS 不可行(硬安全/override不通过) -> 不覆盖。\n'); end
    end
end

% --------- Seamanship 平滑：Fixed Gate + 不更偏离目标 + 硬安全/规则可行 ---------
if useSeaman && isfield(st,'prevObjHdg') && isfinite(st.prevObjHdg) && ~strcmp(decisionSource,'FSM_LOCK') && isfinite(CLOS)
    devPrev = abs(Utils.angdiff_signed(st.prevObjHdg, CLOS));
    devNow  = abs(Utils.angdiff_signed(bestHdg,      CLOS));

    % Fixed Gate：上一帧若已背道而驰（偏离目标超过门限），禁止平滑覆盖
    if devPrev <= ctx.seam_gate_rad + 1e-12

        % 不允许“更偏离目标”：上一帧必须不比当前优化解更差（否则平滑没有价值）
        if devPrev <= devNow + 1e-12

            % 原条件：同向转舵且幅度收敛
            dC0 = Utils.angdiff_signed(st.prevObjHdg, st.prevStartHdg);
            dC1 = Utils.angdiff_signed(bestHdg,      st.prevStartHdg);

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
                    if ctx.doDebug
                        fprintf(' [S4-SEAMANSHIP] FixedGate通过(devPrev=%.1fdeg<=%.1fdeg) 且不更偏离 -> 采用上一帧 (C=%.1fdeg,s=%.2f)\n', ...
                            rad2deg(devPrev), ctx.seam_gate_deg, rad2deg(bestHdg), bestScale);
                    end
                end
            end
        else
            if ctx.doDebug
                fprintf(' [S4-SEAMANSHIP] 禁止：上一帧更偏离目标(devPrev=%.1fdeg > devNow=%.1fdeg)\n', rad2deg(devPrev), rad2deg(devNow));
            end
        end
    else
        if ctx.doDebug
            fprintf(' [S4-SEAMANSHIP] FixedGate阻断：上一帧偏离目标%.1fdeg > gate=%.1fdeg -> 服从当前解\n', ...
                rad2deg(devPrev), ctx.seam_gate_deg);
        end
    end
end

%% =========== 决策打印 ===========
if ctx.doDebug
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

%% =========== 输出 ===========
sol = struct();
sol.bestHdg         = bestHdg;
sol.bestScale       = bestScale;
sol.bestCost        = bestCost;
sol.bestBreak       = bestBreak;
sol.decisionSource  = decisionSource;

sol.lockApplied = lockApplied;
sol.optApplied  = optApplied;
sol.pcApplied   = pcApplied;
sol.seaApplied  = seaApplied;
sol.fbApplied   = fbApplied;

%% ==================== 内部函数（保持原结构） ====================
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

    function [ok, bd] = candidate_feasible_hard(C, s, softCOLREGS)
        import algorithms.vo5L.Utils;
        ok = true;
        bd = struct('U_multi',0,'U_colregs',0);

        v_cand = (s*Vpref) * [cos(C), sin(C)];

        for j = 1:numel(neighbors)
            if ~useTS(j), continue; end
            nb = neighbors(j);
            RP = nb.position - agent.position;
            RV = nb.velocity - v_cand;

            [ds, dp] = Utils.compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
            [hitCore, ~] = Utils.dcpaTcpaHit(RP, RV, ds, T, perceptionDist);
            if hitCore
                ok = false; return;
            end

            if useGCCO_hard && (s >= gcco_gate_s)
                if Utils.any_is_in_gcco(C, obs(j))
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

    function [Cbest, Sbest, Jbest, breakdown] = solve_least_bad()
        import algorithms.vo5L.Utils;
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

                    [ds, dp] = Utils.compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
                    [dcpa, ~, tstar] = Utils.compute_dcpa_tcpa(RP, RV);

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

end
