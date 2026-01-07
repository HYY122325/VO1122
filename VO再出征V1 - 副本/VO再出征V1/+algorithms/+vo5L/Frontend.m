function obs = Frontend(ctx)
import algorithms.vo5L.Utils;
% vo5L.Frontend — S1 前端扫描：CCO/BCO + GCCO/GBCO + 静止目标标记
%
% 输入：
%   ctx.agent / ctx.neighbors / ctx.Vpref / ctx.Cprev / ...（由 vo5L.VOContext 构建）
% 输出：
%   obs(j) 结构体数组（与原 monolithic 版本字段对齐）

agent    = ctx.agent;
neighbors= ctx.neighbors;

hdgGridN = ctx.hdgGridN;
hdgGrid  = ctx.hdgGrid;

Vpref    = ctx.Vpref;
Cprev    = ctx.Cprev;

useGW     = ctx.useGW;
ds_base   = ctx.ds_base;
dp_factor = ctx.dp_factor;

T              = ctx.T;
perceptionDist = ctx.perceptionDist;

static_v_min = ctx.static_v_min;

scanStep  = ctx.scanStep;
scanMax   = ctx.scanMax;
predTime  = ctx.predTime;
dt_chk    = ctx.dt_chk;

Tr        = ctx.Tr;
Kdel      = ctx.Kdel;
Tu        = ctx.Tu;
Dv        = ctx.Dv;
Alpha     = ctx.Alpha;
Kp        = ctx.Kp;
Kd        = ctx.Kd;
deltaMax  = ctx.deltaMax;
deltaRate = ctx.deltaRate;
aMax      = ctx.aMax;

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

        [ds, dp] = Utils.compute_ds_dp_paper(RP, C, ds_base, dp_factor, useGW);
        [hitCore, ~]  = Utils.dcpaTcpaHit(RP, RV, ds, T, perceptionDist);
        [hitBroad, ~] = Utils.dcpaTcpaHit(RP, RV, dp, T, perceptionDist);

        in_core(k) = hitCore;
        in_bco(k)  = (~hitCore) && hitBroad;
    end

    [Al,Ar,Bl,Br] = Utils.local_find_intervals(hdgGrid, in_core, in_bco);
    obs(j).Al=Al; obs(j).Ar=Ar; obs(j).Bl=Bl; obs(j).Br=Br;

    % ---- 3-DoF 扩展：GCCO/GBCO ----
    RP0 = nb.position - agent.position;
    Vts = nb.velocity;
    obs(j).RP = RP0;

    [ds_ref, dp_ref] = Utils.compute_ds_dp_paper(RP0, Cprev, ds_base, dp_factor, useGW);
    obs(j).dp = dp_ref;

    % 标记静止目标
    obs(j).isStatic      = (norm(Vts) < static_v_min);
    obs(j).skipFSM       = obs(j).isStatic;
    obs(j).skipCOLREGS   = obs(j).isStatic;

    % 初始 3DoF 状态
    X0_os = [];
    if isprop(agent, 'useDynamics') && agent.useDynamics && isprop(agent, 'dynState')
        s = agent.dynState;
        if isfield(s, 'u') && isfield(s, 'v') && isfield(s, 'r') && ...
           isfield(s, 'psi') && isfield(s, 'delta')
            X0_os = [s.u, s.v, s.r, s.psi, s.delta];
        end
    end

    if all(isfinite([Al,Bl]))
        obs(j).gBl = Utils.expand_one_side_3dof(Bl, Al, -1, dp_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAl = Utils.expand_one_side_3dof(Al, Bl, -1, ds_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end

    if all(isfinite([Ar,Br]))
        obs(j).gBr = Utils.expand_one_side_3dof(Br, Ar, +1, dp_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);

        obs(j).gAr = Utils.expand_one_side_3dof(Ar, Br, +1, ds_ref, ...
            agent, RP0, Cprev, Vpref, Vts, X0_os, ...
            Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
            predTime, dt_chk, scanStep, scanMax);
    end
end

if ctx.doDebug && ~isempty(neighbors)
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

end
