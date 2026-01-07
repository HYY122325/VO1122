function [obs, st, fsm] = FSM(ctx, obs, st, risk)
% vo5L.FSM — S3：FSM 更新 + overrides 覆盖（吃稳定的 scene/role + FSM内部锁定）
%
% 输入：
%   ctx: vo5L.VOContext 输出
%   obs: vo5L.Frontend / vo5L.RiskScene 输出（含 scene/role/colregsMask）
%   st : vo5L.State('get',...) 输出
%   risk: struct(CR,riskMask,useTS,...)
%
% 输出：
%   obs: colregsMask 可能被 overrides 覆盖
%   st : tsPhase/overrides/colregs_mode 等被更新
%   fsm: struct(fsmActive=...)
import algorithms.vo5L.Utils;
neighbors = ctx.neighbors;
agent     = ctx.agent;

fsmActive = false;

if ctx.useFSM && ~isempty(neighbors)
    st = fsm_update_paper_strict(st, agent, neighbors, obs, ...
        ctx.fsm_sigma, deg2rad(5.0), ctx.Cprev, ctx.CLOS, ...
        ctx.ds_base, ctx.dp_factor, ctx.useGW, ctx.fsm_dt, ctx.t_now, ctx.doDebug, ctx.epsC1, ...
        risk.CR, risk.riskMask, risk.useTS, ctx.cr_enter, ctx.sr_cr_exit, ctx.static_v_min);

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

if ctx.doDebug && ~isempty(neighbors)
    fprintf('---------------------------------------------------\n');
    if ctx.useFSM
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
            neighbors(jdbg).id, risk.CR(jdbg), risk.riskMask(jdbg), risk.useTS(jdbg), ...
            char(obs(jdbg).scene), obs(jdbg).role, ph, obs(jdbg).isStatic);
    end
    fprintf('---------------------------------------------------\n');
end

fsm = struct('fsmActive', fsmActive);

end

%% ==================== FSM 内核（保持原实现结构） ====================
function stx = fsm_update_paper_strict(stx, os, nbs, obss, ...
                                       sigma, deg5_, COS, CLOS_loc, ... %#ok<INUSD>
                                       ds0, dp_fac, useGW_, fsm_dt_, t_now_, doDbg, epsC1_loc, ...
                                       CR_in, riskMask_in, useTS_in, cr_enter_in, cr_exit_in, static_v_min)
import algorithms.vo5L.Utils;
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

    [~, dpj] = Utils.compute_ds_dp_paper(RP, COS, ds0, dp_fac, useGW_);
    D = norm(RP);

    Cts_now = atan2(ts.velocity(2), ts.velocity(1));
    if isfinite(stx.tsHeadings(jj)), Cts_prev = stx.tsHeadings(jj); else, Cts_prev = Cts_now; end

    if D <= dpj && ~isfinite(stx.tsHead0(jj))
        stx.tsHead0(jj) = Cts_prev;
        stx.tsRVo(jj,:) = os.velocity - ts.velocity;
    end

    if isfinite(stx.tsHead0(jj))
        dCTS = abs(Utils.angdiff_signed(Cts_now, stx.tsHead0(jj)));
    else
        dCTS = 0;
    end

    inGCCO = Utils.any_is_in_gcco(COS, obss(jj));

    Vrel_ts = ts.velocity - os.velocity;
    [~, ~, tstar] = Utils.compute_dcpa_tcpa(RP, Vrel_ts);
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

                    lhs = Utils.cos_between(RVo, RVopt) - Utils.cos_between(RVo, RVm);
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
                    overrides(end).mask = @(C) Utils.assist_rule_mask(C, ts.velocity, Vsea, RVo, RVm);
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
                    overrides(end).mask = @(C) Utils.assist_rule_mask(C, ts.velocity, Vsea, RVo, RVm);
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

%% ---------- local helpers ----------
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

    % FSM内部锁定字段
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
