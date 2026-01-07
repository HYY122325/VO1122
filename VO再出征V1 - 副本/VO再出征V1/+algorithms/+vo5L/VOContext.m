function ctx = VOContext(agent, neighbors, params, t_now, doDebug)
% vo5L.VOContext — 参数解析 & 派生上下文（保持原 computeVO_Step5L 的 S0 逻辑）
%
% ctx 字段尽量保持与原变量同名，便于对照与回归验证。
import algorithms.vo5L.Utils;
gfd = @algorithms.vo5L.Utils.getfieldwithdef;

%% =========== 参数与默认 ===========
s5       = gfd(params, 'step5', struct());
epsC1    = gfd(s5, 'c1_eps', 1e-2);

useGW    = gfd(params.lvo, 'useGoodwin', true);
ds_base  = params.lvo.ds_base;
dp_factor= params.lvo.dp_factor;

cr_enter = gfd(params.lvo, 'cr_threshold', 0.4);

hdgGridN = gfd(params.lvo,'hdgGridN',360);
hdgGrid  = linspace(-pi, pi, hdgGridN);

% --- 静止目标阈值 ---
static_v_min = gfd(s5,'static_v_min',0.10);

% --- Seamanship Fixed Gate ---
seam_gate_deg = gfd(s5,'seamanship_gate_deg',60.0);
seam_gate_rad = deg2rad(seam_gate_deg);

% ---------- 代价权重 ----------
lambda_pts = gfd(params.lvo, 'lambda_pts', 2.0);
gamma      = gfd(params.lvo,'gamma',2.5);

omega      = gfd(s5, 'route_w', 0.5);
omega_ct   = gfd(s5,'route_ct_w',omega);

% ---------- 工程新增参数 ----------
speed_scales = gfd(s5,'speed_scales',[1.0, 0.8]);
w_speed      = gfd(s5,'speed_w',0.8);

CR_floor     = gfd(s5,'cr_floor',0.20);
w_colregs    = gfd(s5,'colregs_soft_w',5.0);

useGCCO_hard = gfd(s5,'use_gcco_hard',true);
gcco_gate_s  = gfd(s5,'gcco_speed_gate',0.85);

allow_stop   = gfd(s5,'allow_stop',true);
stop_scale   = gfd(s5,'stop_scale',0.0);

useCOLREGS = gfd(s5,'useCOLREGS',true);
useSeaman  = gfd(s5,'useSeamanship',true);
useFSM     = gfd(s5,'useFSM',true);

% FSM 参数
fsm_sigma  = gfd(s5,'fsm_sigma',25.0);
fsm_dt     = gfd(s5,'fsm_dt',0.05);

% 横向偏移预测
route_ct_T     = gfd(s5,'route_ct_horizon',5.0);
route_ct_scale = gfd(s5,'route_ct_scale',2.0*ds_base*dp_factor);

% 3-DoF/GCCO 扩展参数
scanStep  = deg2rad(gfd(s5,'scanStepDeg',1.0));
scanMax   = deg2rad(gfd(s5,'scanMaxDeg',45.0));
predTime  = gfd(s5,'predTime',20.0);
dt_chk    = gfd(s5,'dtCheck',0.1);

Tr        = gfd(s5,'yaw_T',  5.0);
Kdel      = gfd(s5,'yaw_K',  1.0);
Tu        = gfd(s5,'u_T',   2.0);
Dv        = gfd(s5,'v_D',    0.8);
Alpha     = gfd(s5,'uv_alpha',0.2);
Kp        = gfd(s5,'Kp',     1.6);
Kd        = gfd(s5,'Kd',     0.4);
deltaMax  = deg2rad(gfd(s5,'delta_max_deg',35));
deltaRate = deg2rad(gfd(s5,'delta_rate_deg',10));
aMax      = gfd(s5,'a_max',  1);

perceptionDist = params.agent.perceptionDist;
T              = agent.timeHorizon;

% 速度与参考航向
Vpref  = max(1e-3, norm(agent.prefVelocity));
Cprev  = Utils.headingFromVelocity(agent);
toGoal = agent.goal - agent.position;
CLOS   = atan2(toGoal(2), toGoal(1));
if ~isfinite(CLOS), CLOS = Cprev; end

%% ===== neighbors 按 id 排序，稳定 FSM 索引 =====
if ~isempty(neighbors)
    [~, ord] = sort([neighbors.id]);
    neighbors = neighbors(ord);
end

%% ====== episode-lock 配置（与原字段一致）======
sr_enable          = gfd(s5,'sr_lock_enable',true);

sr_confirm_steps   = gfd(s5,'sr_confirm_steps',2);
sr_lock_on_risk    = gfd(s5,'sr_lock_on_risk',true);
sr_lock_on_dp      = gfd(s5,'sr_lock_on_dp',true);

sr_cr_exit_factor  = gfd(s5,'sr_cr_exit_factor',0.50);
sr_cr_exit         = gfd(s5,'sr_cr_exit',sr_cr_exit_factor*cr_enter);

sr_dp_clear_factor = gfd(s5,'sr_dp_clear_factor',1.20);
sr_sep_eps         = gfd(s5,'sr_sep_eps',0.00);

sr_mem_ttl         = gfd(s5,'sr_mem_ttl',60.0);

%% === 基线横向偏移（保持原逻辑） ===
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
    base_dir    = [NaN,NaN];
    n_base      = [0,0];
    d_path_now  = 0;
end

%% =========== 打包 ctx ===========
ctx = struct();

ctx.agent    = agent;
ctx.neighbors= neighbors;
ctx.params   = params;

ctx.s5   = s5;

ctx.epsC1 = epsC1;

ctx.useGW     = useGW;
ctx.ds_base   = ds_base;
ctx.dp_factor = dp_factor;

ctx.cr_enter = cr_enter;

ctx.hdgGridN = hdgGridN;
ctx.hdgGrid  = hdgGrid;

ctx.static_v_min  = static_v_min;

ctx.seam_gate_deg = seam_gate_deg;
ctx.seam_gate_rad = seam_gate_rad;

ctx.lambda_pts = lambda_pts;
ctx.gamma      = gamma;

ctx.omega      = omega;
ctx.omega_ct   = omega_ct;

ctx.speed_scales = speed_scales;
ctx.w_speed      = w_speed;

ctx.CR_floor   = CR_floor;
ctx.w_colregs  = w_colregs;

ctx.useGCCO_hard = useGCCO_hard;
ctx.gcco_gate_s  = gcco_gate_s;

ctx.allow_stop = allow_stop;
ctx.stop_scale = stop_scale;

ctx.useCOLREGS = useCOLREGS;
ctx.useSeaman  = useSeaman;
ctx.useFSM     = useFSM;

ctx.fsm_sigma  = fsm_sigma;
ctx.fsm_dt     = fsm_dt;

ctx.route_ct_T     = route_ct_T;
ctx.route_ct_scale = route_ct_scale;

ctx.scanStep  = scanStep;
ctx.scanMax   = scanMax;
ctx.predTime  = predTime;
ctx.dt_chk    = dt_chk;

ctx.Tr        = Tr;
ctx.Kdel      = Kdel;
ctx.Tu        = Tu;
ctx.Dv        = Dv;
ctx.Alpha     = Alpha;
ctx.Kp        = Kp;
ctx.Kd        = Kd;
ctx.deltaMax  = deltaMax;
ctx.deltaRate = deltaRate;
ctx.aMax      = aMax;

ctx.perceptionDist = perceptionDist;
ctx.T              = T;

ctx.Vpref = Vpref;
ctx.Cprev = Cprev;
ctx.CLOS  = CLOS;

ctx.haveBase          = haveBase;
ctx.currentPathOrigin = currentPathOrigin;
ctx.base_dir          = base_dir;
ctx.n_base            = n_base;
ctx.d_path_now        = d_path_now;

ctx.t_now   = t_now;
ctx.doDebug = doDebug;

% sr-lock 配置
ctx.sr_enable          = sr_enable;
ctx.sr_confirm_steps   = sr_confirm_steps;
ctx.sr_lock_on_risk    = sr_lock_on_risk;
ctx.sr_lock_on_dp      = sr_lock_on_dp;
ctx.sr_cr_exit_factor  = sr_cr_exit_factor;
ctx.sr_cr_exit         = sr_cr_exit;
ctx.sr_dp_clear_factor = sr_dp_clear_factor;
ctx.sr_sep_eps         = sr_sep_eps;
ctx.sr_mem_ttl         = sr_mem_ttl;

end
