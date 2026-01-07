classdef SimConfigV1_2
    % 统一配置类 V1.2 - FSM 滞回增强版 + Step5I
    %
    % 说明：
    %   - params.lvo.cr_threshold = 0.4
    %     作为：
    %       1) CR 风险阈值（compute_frontend_CR_paper）
    %       2) FSM 的 CR_ENTER（避碰/防御动作进入门槛）
    %     二者统一，避免“风险判定”和“FSM 触发”门槛不一致。
    %
    %   - fsm_dt = sim.dt，保证 FSM 更新频率 = 仿真步长
    %   - 本配置默认使用 computeVO_Step5I 作为算法入口

    methods (Static)

        %% ---------- 默认配置 ----------
        function params = default()

            %% ---------- 世界与仿真 ----------
            params.world.fieldSize = 25;
            fs = params.world.fieldSize;
            params.world.xlim = [-fs, fs];
            params.world.ylim = [-fs, fs];

            params.sim.dt       = 0.2;
            params.sim.maxSteps = 3000;
            params.sim.scenario = 'headon_2'; 

            %% ---------- Agent ----------
            params.agent.radius        = 0.5;
            params.agent.maxSpeed      = 1.00;

            params.agent.neighborDist  = 30.00;  
            params.agent.timeHorizon   = 15.0;   
            params.agent.safetyRadius  = 1.0;    

            params.agent.perceptionDist = 30.00;

            %% ---------- 算法入口 ----------
            params.algorithm.name       = 'VO_CompareStep';
            params.algorithm.numSamples = 180;   % 航向离散数（与 hdgGridN 对应）

            %% ---------- LVO / CR / 基础参数 ----------
            params.lvo.useGoodwin   = true;
            params.lvo.ds_base      = 3;      % 基础安全距离 ds0
            params.lvo.dp_factor    = 2;      % dp = dp_factor * ds

            params.lvo.hdgGridN     = 360;
            params.lvo.gamma        = 2.5;
            params.lvo.lambda_pts   = 3;      % PTS 权重基数
            params.lvo.engScale     = 1.0;

            % CR 权重（论文式(9) 的 w1~w5）
            params.lvo.cr_w         = [0.40, 0.367, 0.167, 0.033, 0.033];

            % [关键] CR 风险阈值：同时用于
            %   1) compute_frontend_CR_paper 的 thr
            %   2) FSM 的 CR_ENTER（进入 Action-3/4/5）
            params.lvo.cr_threshold = 0.35;

            %% ---------- Step5 参数 ----------
            params.step5.scanStepDeg        = 1.0;
            params.step5.scanMaxDeg         = 45.0;
            params.step5.useGCCOinFeasible  = true;
            params.step5.useGBCOinPTS       = true;

            % 动力学参数（3-DoF 船模）
            params.step5.yaw_T     = 0.5;    % 航向响应时间常数
            params.step5.yaw_K     = 1.0;    % 舵角→转速增益
            params.step5.u_T       = 2.0;    % 纵向速度响应时间常数
            
            params.step5.v_D       = 0.8;    % 横摇阻尼系数
            params.step5.uv_alpha  = 0.2;    % 交叉项系数

            % [新增] 全局动力学开关：true = 开启物理运动模型; false = 质点模型
            params.sim.enableDynamics = true;

            % PD 控制器调优
            params.step5.Kp = 1.5;
            params.step5.Kd = 3.0;

            % 物理约束
            params.step5.delta_max_deg  = 35;  % 最大舵角
            params.step5.delta_rate_deg = 15;  % 最大舵角变化率 (deg/s)
            params.step5.a_max          = 1.0; % 最大加速度

            % 前向模拟（3-DoF 扩展 GCCO/GBCO）
            params.step5.predTime = 6.0;
            params.step5.dtCheck  = 0.1;

            % COLREGS / FSM / 航路保持
            params.step5.useCOLREGS    = 1;
            params.step5.route_w       = 1;   
            params.step5.route_ct_w    = 0;   
            params.step5.route_ct_horizon = 0.05;  % 航路横向偏移预测时间
            params.step5.route_ct_scale   = 2.0 * params.lvo.ds_base * params.lvo.dp_factor;

            params.step5.useSeamanship = 1;
            params.step5.useFSM        = 1;
            
            % FSM 关键参数
            params.step5.fsm_sigma     = 10.0;              % μ = TCPA / sigma
            params.step5.fsm_dt        = params.sim.dt;     % [重要] FSM 时间步长 = 仿真步长
            params.step5.c1_eps        = 1e-2;              % TC-1 判据容差

            %% ---------- 工程代价（旧版权重接口，保留占位） ----------
            params.w.deviation  = 0.0;
            params.w.zero       = 0.0;
            params.w.direction  = 0.0;
            params.w.smoothness = 0.0;
            params.w.ttc        = 0.0;

            %% ---------- 可视化 ----------
            params.viz.showVelocity     = true;
            params.viz.showGoals        = true;
            params.viz.showSafetyCircle = true;
            params.viz.showSpeedValue   = true;
            params.viz.updateInterval   = 15;
            params.viz.pauseTime        = 0.05;

            params.viz.saveVideo   = false;        
            params.viz.videoName   = 'sim_Step5.mp4';
            params.viz.videoFPS    = 30;
            params.viz.showVOCones = true;
            
            params.viz.showFSMStatus = true; 

            params.viz.saveTrajectoryImage    = false;   
            params.viz.trajectoryTimeInterval = 1.0;     
            params.viz.trajectoryMarkerSizes  = [8, 6, 4]; 

            %% ---------- 调试 ----------
            params.debug.dumpEvery      = 0;
            params.debug.printBestCost  = false;

            params.step5.debug.enable  = true;   % 开启 Step5I debug 打印
            params.step5.debug.agentId = 1;      % 只打印某个 agent
            params.step5.debug.tStart  = 0.0;    
            params.step5.debug.tEnd    = inf;    
        end

        %% ---------- 把配置写回 Agent 的统一接口 ----------
        function agents = applyAgentCollisionConfig(params, agents)
            if isempty(agents), return; end
            if ~isfield(params, 'agent')
                warning('SimConfigV1_2: params.agent 不存在');
                return;
            end

            cfg = params.agent;

            for i = 1:numel(agents)
                a = agents(i);

                if isprop(a, 'neighborDist') && isfield(cfg, 'neighborDist')
                    a.neighborDist = cfg.neighborDist;
                end
                if isprop(a, 'timeHorizon') && isfield(cfg, 'timeHorizon')
                    a.timeHorizon = cfg.timeHorizon;
                end
                if isprop(a, 'safetyRadius') && isfield(cfg, 'safetyRadius')
                    a.safetyRadius = cfg.safetyRadius;
                end
                if isprop(a, 'radius') && isfield(cfg, 'radius')
                    if isempty(a.radius) || (isscalar(a.radius) && a.radius <= 0)
                        a.radius = cfg.radius;
                    end
                end
                if isprop(a, 'maxSpeed') && isfield(cfg, 'maxSpeed')
                    if isempty(a.maxSpeed) || (isscalar(a.maxSpeed) && a.maxSpeed <= 0)
                        a.maxSpeed = cfg.maxSpeed;
                    end
                end
            end
        end
    end
end
