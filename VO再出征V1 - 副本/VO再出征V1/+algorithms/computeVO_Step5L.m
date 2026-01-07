function newVelocity = computeVO_Step5L(agent, neighbors, params)
% computeVO_Step5L — Modularized Version (Namespace: +vo5L/)
%
% ✅ 保持接口不变：newVelocity = computeVO_Step5L(agent, neighbors, params)
% ✅ 保持原有主干逻辑不变：硬安全 > (FSM/COLREG硬规则) > 缓冲软代价 > Past&Clear > Seamanship > Fallback
% ✅ 仅做“按主干功能模块化”拆分，避免过度稀碎
%
% 模块划分（与原 S0~S4 对齐）：
%   S0: algorithms.vo5L.VOContext      —— 参数解析/派生量/邻船排序/基线信息
%   Sx: algorithms.vo5L.State       —— 持久状态读写（S5STATE 迁移）
%   S1: algorithms.vo5L.Frontend    —— CCO/BCO + GCCO/GBCO 扫描（含静止目标标记）
%   S2: algorithms.vo5L.RiskScene   —— CR + Past&Clear + (sr_lock) + COLREGS mask
%   S3: algorithms.vo5L.FSM         —— FSM 更新 + override mask 覆盖（含调试输出）
%   S4: algorithms.vo5L.Planner     —— 联合优化(C+s) + Past&Clear + Seamanship + Fallback（含调试输出）
%   Utils: algorithms.vo5L.Utils    —— 通用几何/数学/3DoF 扩展等
%
% 注意：本文件仅保留“节流 debug”与 Action-5 的细节调试门控，其余逻辑已下放到模块。

%% =========== 边界情况 ===========
if isempty(neighbors)
    newVelocity = agent.prefVelocity;
    return;
end

%% =========== 时间（debug）===========
simStep = -1;
simDt   = 0.1;
if isfield(params, 'sim')
    if isfield(params.sim,'step'), simStep = params.sim.step; end
    if isfield(params.sim,'dt'),   simDt   = params.sim.dt;   end
end
if simStep >= 0, t_now = simStep * simDt; else, t_now = NaN; end

%% =========== DEBUG 节流（1 Hz）===========
persistent ASSIST_last_print_t DEBUG_last_print_t
if isfield(params, 'sim') && isfield(params.sim, 'step') && params.sim.step <= 1
    % 重置模块状态（对应原先 S5STATE 清空）
    algorithms.vo5L.State('reset');
    ASSIST_last_print_t = -inf;
    DEBUG_last_print_t  = -inf;
end
if isempty(ASSIST_last_print_t), ASSIST_last_print_t = -inf; end
if isempty(DEBUG_last_print_t),  DEBUG_last_print_t  = -inf;  end

doDebug = false;
if isfinite(t_now) && (t_now - DEBUG_last_print_t >= 1.0 - 1e-9)
    DEBUG_last_print_t = t_now;
    doDebug = true;
    fprintf('\n================ [DEBUG t=%.2f s | Agent %d] ================\n', t_now, agent.id);
end

%% =========== S0：构建上下文（参数/派生量/邻船排序/基线）===========
ctx = algorithms.vo5L.VOContext(agent, neighbors, params, t_now, doDebug);

% S0 debug（保持原格式）
if ctx.doDebug
    fprintf(' [S0-OS] Pos=(%.1f,%.1f), Vel=(%.2f,%.2f), Vpref=%.2f m/s\n', ...
        ctx.agent.position(1), ctx.agent.position(2), ctx.agent.velocity(1), ctx.agent.velocity(2), ctx.Vpref);
    fprintf(' [S0-OS] Cprev=%.1f deg, CLOS=%.1f deg\n', rad2deg(ctx.Cprev), rad2deg(ctx.CLOS));
    fprintf(' [S0-OS] Flags: useCOLREGS=%d, useFSM=%d, useSeamanship=%d\n', ctx.useCOLREGS, ctx.useFSM, ctx.useSeaman);
    fprintf(' [S0-OS] speed_scales = ['); fprintf('%.2f ', ctx.speed_scales); fprintf(']\n');
    fprintf(' [S0-OS] seamanship_gate = %.1f deg\n', ctx.seam_gate_deg);
    if ~isempty(ctx.neighbors)
        fprintf(' [S0-OS] Neighbor IDs(sorted): '); fprintf('%d ', ctx.neighbors.id); fprintf('\n');
    end
end

%% =========== 读取状态（含 episode-lock 记忆）===========
st = algorithms.vo5L.State('get', ctx.agent.id);

%% =========== S1：前端扫描 ===========
obs = algorithms.vo5L.Frontend(ctx);

%% =========== S2：风险/局面/锁定 + COLREGS ===========
[obs, st, risk] = algorithms.vo5L.RiskScene(ctx, obs, st);

%% =========== S3：FSM 覆盖 ===========
[obs, st, fsm] = algorithms.vo5L.FSM(ctx, obs, st, risk); %#ok<NASGU>

%% =========== S4：联合优化 + 覆盖链 ===========
sol = algorithms.vo5L.Planner(ctx, obs, st, risk);

%% =========== Action-5 Assist Rule 细节调试（保留） ===========
if isfield(st,'colregs_mode') && st.colregs_mode == 5 && isfinite(ctx.t_now)
    if ctx.t_now - ASSIST_last_print_t >= 0.5 - 1e-9
        ASSIST_last_print_t = ctx.t_now;
        for j_dbg = 1:numel(ctx.neighbors)
            if isfield(st,'tsRVo') && size(st.tsRVo,1) >= j_dbg && all(isfinite(st.tsRVo(j_dbg,:)))
                ts_dbg   = ctx.neighbors(j_dbg);
                RVo_dbg  = st.tsRVo(j_dbg,:);
                RVm_dbg  = ctx.agent.velocity - ts_dbg.velocity;

                v_c_dbg  = (sol.bestScale*ctx.Vpref)*[cos(sol.bestHdg), sin(sol.bestHdg)];
                RVOS_dbg = v_c_dbg - ts_dbg.velocity;

                cosOS   = algorithms.vo5L.Utils.cos_between(RVo_dbg, RVOS_dbg);
                cosM    = algorithms.vo5L.Utils.cos_between(RVo_dbg, RVm_dbg);
                lhs_dbg = cosOS - cosM;
                ok_dbg  = algorithms.vo5L.Utils.assist_rule_mask(sol.bestHdg, ts_dbg.velocity, (sol.bestScale*ctx.Vpref), RVo_dbg, RVm_dbg);

                RP_dbg   = ts_dbg.position - ctx.agent.position;
                Vrel_dbg = ts_dbg.velocity - ctx.agent.velocity;
                [dcpa_dbg, ~, tstar_dbg] = algorithms.vo5L.Utils.compute_dcpa_tcpa(RP_dbg, Vrel_dbg);

                fprintf('[t=%.2f] DBG-A5: TS=%d lhs=%.3f ok=%d | D=%.2f DCPA=%.2f t*=%.2f\n', ...
                        ctx.t_now, ts_dbg.id, lhs_dbg, ok_dbg, norm(RP_dbg), dcpa_dbg, tstar_dbg);
            end
        end
    end
end

%% =========== 输出 & 写回状态（保持原顺序）===========
newVelocity = (sol.bestScale*ctx.Vpref) * [cos(sol.bestHdg), sin(sol.bestHdg)];

st.prevObjHdg    = sol.bestHdg;
st.prevStartHdg  = ctx.Cprev;
st.prevObjScale  = sol.bestScale;

algorithms.vo5L.State('set', ctx.agent.id, st);

end
