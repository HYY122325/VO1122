function [obs, st, risk] = RiskScene(ctx, obs, st)
% vo5L.RiskScene — S2：CR + Past&Clear + (episode-lock) + COLREGS mask
%
% 输出：
%   obs(j).scene / obs(j).role / obs(j).colregsMask 更新
%   st.srMem 可能更新
%   risk: struct('CR',...,'riskMask',...,'useTS',...,'allRiskCleared',...)
import algorithms.vo5L.Utils;
agent     = ctx.agent;
neighbors = ctx.neighbors;
params    = ctx.params;

Cprev    = ctx.Cprev;
Vpref    = ctx.Vpref;

cr_enter = ctx.cr_enter;

%% ===== episode-lock prune（与原位置等价：在使用 srMem 之前）=====
if ctx.sr_enable
    st = srmem_prune(st, ctx.t_now, ctx.sr_mem_ttl);
end

%% =========== S2：CR ===========
[CR, riskMask] = compute_frontend_CR_paper(os_for_cr(agent), neighbors, params, Cprev, cr_enter);

Nnb   = numel(neighbors);
useTS = true(1, Nnb);

if ctx.doDebug && ~isempty(neighbors)
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
    [~, ~, tstar_pair] = Utils.compute_dcpa_tcpa(RP, Vrel_pair);
    is_sep = dot(RP, Vrel_pair) > 0;
    if tstar_pair < 0 && is_sep
        useTS(j) = false;
        if ctx.doDebug
            fprintf('   [S2-RISK] Past&Clear: TS %d t* = %.2f < 0 & Sep=1, 从风险集合移除.\n', nb.id, tstar_pair);
        end
    end
end
riskTS_active   = riskMask & useTS;
allRiskCleared  = ~any(riskTS_active);

%% --- 局面/角色 + COLREGS mask（关键：先 episode-lock，再建 mask） ---
for j=1:numel(neighbors)
    nb = neighbors(j);

    % (1) 静止目标：直接障碍物化（不分类、不锁定、不出规则、不进FSM）
    if obs(j).isStatic
        obs(j).scene = "OTHERS";
        obs(j).role  = 'standon';
        obs(j).colregsMask = @(C) true;
        if ctx.doDebug
            fprintf('   > TS %d: [STATIC_OBS] v=%.2f < %.2f -> 障碍物模式(跳过COLREG/FSM)\n', ...
                nb.id, norm(nb.velocity), ctx.static_v_min);
        end
        continue;
    end

    % 1) raw 分类
    [scene_raw, role_raw] = classify_scene_role_paper_stable(agent, nb, Cprev, Vpref, ctx.doDebug);

    % 2) episode-lock 更新并得到最终 scene/role（不跳变）
    if ctx.sr_enable
        [st, scene_use, role_use] = sr_lock_update(st, agent, nb, obs(j).dp, ...
            scene_raw, role_raw, CR(j), riskMask(j), useTS(j), ...
            cr_enter, ctx.sr_cr_exit, ctx.sr_confirm_steps, ctx.sr_lock_on_risk, ctx.sr_lock_on_dp, ...
            ctx.sr_dp_clear_factor, ctx.sr_sep_eps, ctx.t_now, ctx.doDebug);
    else
        scene_use = scene_raw;
        role_use  = role_raw;
    end

    obs(j).scene = scene_use;
    obs(j).role  = role_use;

    % 3) COLREGS mask（只吃“锁定后的 scene/role”）
    obs(j).colregsMask = @(C) true;
    if ctx.useCOLREGS && ~obs(j).skipCOLREGS
        obs(j).colregsMask = build_colregs_mask_paper(agent, nb, obs(j).scene, obs(j).role, Vpref);
    end
end

risk = struct();
risk.CR            = CR;
risk.riskMask      = riskMask;
risk.useTS         = useTS;
risk.allRiskCleared= allRiskCleared;

end

%% ==================== 内部函数（与原实现保持一致） ====================
function stx = os_for_cr(os_in)
% CR 计算用一个“稳定的 OS 速度尺度”（避免起步/低速导致 K 抖动）
stx = os_in;
end

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
import algorithms.vo5L.Utils; 
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

[~, ~, tstar] = Utils.compute_dcpa_tcpa(RP, Vrel);
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

%% ======== CR 计算（保持原函数簇）========
function [CRval, riskMask2] = compute_frontend_CR_paper(os, nbs, prm, osHdg, cr_thr_default)
import algorithms.vo5L.Utils;
N = numel(nbs);
CRval    = zeros(1,N);
riskMask2= false(1,N);
w   = Utils.getfieldwithdef(prm.lvo, 'cr_w', [0.40, 0.367, 0.167, 0.033, 0.033]);
thr = Utils.getfieldwithdef(prm.lvo, 'cr_threshold', cr_thr_default);

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
    [dcpa, ~, tstar] = Utils.compute_dcpa_tcpa(RP, Vrel_vec);
    [dsj, dpj] = Utils.compute_ds_dp_paper(RP, osHdg, prm.lvo.ds_base, prm.lvo.dp_factor, prm.lvo.useGoodwin);
    D = norm(RP);
    B = abs(Utils.angleDiff(atan2(RP(2),RP(1)), osHdg));
    K = Vref_forK / max(1e-6, norm(ts.velocity));

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

%% ======== 分类：closing 判据一致化 + 低速用期望速度稳住 ========
function [scene, role] = classify_scene_role_paper_stable(os, ts, osHdg, Vpref, doDbg)
P_diff = ts.position - os.position;

V_ts_val = norm(ts.velocity);

% OS用于判别closing的速度：低速时用“期望速度方向”
V_os_use = os.velocity;
if norm(V_os_use) < 0.2*Vpref
    V_os_use = Vpref * [cos(osHdg), sin(osHdg)];
end

RV = ts.velocity - V_os_use;              % TS-OS
is_closing = dot(P_diff, RV) < -0.05;     % approaching 判据一致

if V_ts_val > 1e-6
    C_ts = atan2(ts.velocity(2), ts.velocity(1));
else
    C_ts = osHdg;
end

B_ts_from_os = atan2(P_diff(2), P_diff(1));
Q  = atan2(sin(B_ts_from_os - osHdg), cos(B_ts_from_os - osHdg));
dC = abs(atan2(sin(C_ts - osHdg), cos(C_ts - osHdg)));

DEG_PARALLEL   = 15.0 * pi / 180;
DEG_HEADON_BRG = 15.0 * pi / 180;
DEG_112_5      = 112.5 * pi / 180;
DEG_HEADON_HDG = 150.0 * pi / 180;

scene = "OTHERS";
role  = 'standon';

% (1) 静止目标直接 OTHERS：由主流程障碍物化处理，不走 COLREG/FSM
V_ts_min = 0.10;
if V_ts_val < V_ts_min
    if doDbg && is_closing
        fprintf('   > TS %d: [STATIC] closing but treated as obstacle only (scene=OTHERS)\n', ts.id);
    end
    return;
end

% OVERTAKEN：TS在前方，本船在其艉后视角
if is_closing && (abs(Q) > DEG_112_5)
    scene = "OVERTAKEN"; role  = 'standon';
    if doDbg
        fprintf('   > TS %d: [OVERTAKEN] Role=STAND-ON. |Q|=%.1f>112.5\n', ts.id, rad2deg(abs(Q)));
    end
    return;
end

B_os_from_ts = atan2(-P_diff(2), -P_diff(1));
Q_os_in_ts_view = atan2(sin(B_os_from_ts - C_ts), cos(B_os_from_ts - C_ts));
if is_closing && (abs(Q_os_in_ts_view) > DEG_112_5)
    scene = "OVERTAKING"; role  = 'giveway';
    if doDbg
        fprintf('   > TS %d: [OVERTAKING] Role=GIVE-WAY. OS 在 TS 艉后 (Q_view=%.1f)\n', ts.id, rad2deg(Q_os_in_ts_view));
    end
    return;
end

if is_closing && (abs(Q) <= DEG_HEADON_BRG) && (dC >= DEG_HEADON_HDG)
    scene = "HEAD_ON"; role  = 'giveway';
    if doDbg
        fprintf('   > TS %d: [HEAD_ON] Role=GIVE-WAY. |Q|<15 & dC>150\n', ts.id);
    end
    return;
end

is_crossing_geom = (dC > DEG_PARALLEL) && (dC < DEG_HEADON_HDG);
if is_closing && is_crossing_geom
    RC_sin = sin(C_ts - osHdg);
    if (Q > 0) && (RC_sin > 0.05)
        scene = "CROSS_LEFT"; role  = 'standon';
        return;
    end
    if (Q < 0) && (RC_sin < -0.05)
        scene = "CROSS_RIGHT"; role  = 'giveway';
        return;
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
        ok = (z <= -1e-9);
    end
end
