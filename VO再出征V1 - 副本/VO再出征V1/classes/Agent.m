classdef Agent < handle
    % 通用智能体（支持 VO / CONST / STOP）
    % [MOD] 修复了动力学跟随不稳、终点绕圈的问题

    properties
        id
        position
        velocity
        goal
        radius
        maxSpeed
        prefVelocity
        neighbors
        neighborDist      % 由配置类写入
        timeHorizon       % 由配置类写入
        safetyRadius      % 由配置类写入
        color
        debugInfo

        % ===== Q1新增：路径起点（用于U_route计算） =====
        pathOrigin double = [NaN, NaN]

        % ===== Q4新增：动力学模型集成 =====
        useDynamics logical = false
        % 动力学状态 [u, v, r, psi, delta]
        dynState struct = struct('u', 0, 'v', 0, 'r', 0, 'psi', 0, 'delta', 0)
        % 动力学状态向量 (State Vector):
        % u: 纵向速度 (前进速度)
        % v: 横向速度 (漂移速度)
        % r: 角速度 (转弯快慢)
        % psi: 航向角 (船头朝向)
        % delta: 舵角 (方向盘角度)
        % 动力学参数（由主脚本从Config注入）
        dynParams struct = struct()

        % 模式与恒速参数
        mode char = 'VO'          % 'VO' | 'CONST' | 'STOP'
        constVel   double = [0,0] % CONST: 恒定世界速度
        constSpeed double = 0     % CONST: 沿 waypoint 的恒速 (<=0 时用 maxSpeed)

        % 到达策略
        arrivalTol  double   = 0.0   % 到达容差（<=0 时自动取 radius）
        stopAtGoal  logical  = true  % 到达后自动停

        % Waypoints（仅 CONST 用）
        waypoints double = []    % N×2 路径点
        wpIndex   (1,1) double = 1
        wpMode    char = 'none'  % 'none' | 'loop' | 'pingpong'
        wpDir     (1,1) double = 1
        wpTol     (1,1) double = 0
    end

    methods
        function obj = Agent(id, position, goal, radius, maxSpeed, mode, constVel)
            if nargin < 6, mode     = 'VO';       end
            if nargin < 7, constVel = [0, 0];     end

            obj.id       = id;
            obj.position = position;
            obj.goal     = goal;

            obj.radius   = radius;
            obj.maxSpeed = maxSpeed;

            obj.velocity     = [0,0];
            obj.prefVelocity = [0,0];

            obj.neighbors    = [];

            obj.neighborDist = NaN;
            obj.timeHorizon  = NaN;
            obj.safetyRadius = NaN;

            obj.color    = rand(1,3);
            
            % Q2: 初始化debugInfo以包含FSM字段
            obj.debugInfo = struct('devCost',0,'collCost',0,'zeroPenalty',0,...
                                   'dirCost',0,'smoothCost',0,'totalCost',0, ...
                                   'fsm_mode', 0, 'fsm_observe_ids', []);

            obj.mode      = mode;
            obj.constVel  = constVel;

            % 默认到达阈值：为了防止动力学过冲，稍微放大一点容差
            obj.arrivalTol = radius;
            obj.wpTol      = obj.arrivalTol;
            obj.stopAtGoal = true;
            
            % Q1: 初始化 pathOrigin
            obj.pathOrigin = position;
            
            % Q4: 初始化动力学状态（航向psi）
            toGoal = goal - position;
            if norm(toGoal) > 1e-6
                obj.dynState.psi = atan2(toGoal(2), toGoal(1));
            else
                obj.dynState.psi = 0;
            end
        end

        %% ===== Waypoint API =====
        function setWaypoints(obj, W, mode, speed, tol)
            if nargin < 3 || isempty(mode),  mode  = 'none'; end
            if nargin < 4,                  speed = 0;      end
            if nargin < 5,                  tol   = 0;      end

            obj.waypoints = double(W);
            obj.wpIndex   = 1;
            obj.wpMode    = char(mode);
            obj.wpDir     = 1;
            obj.constSpeed= speed;

            if tol > 0
                obj.wpTol = tol;
            end

            if ~isempty(W)
                obj.goal = W(end,:); % 兼容“goal”显示
            end
        end        %设置路径点

        function wp = currentWaypoint(obj)
            if isempty(obj.waypoints)
                wp = obj.goal;
            else
                k  = max(1, min(size(obj.waypoints,1), obj.wpIndex));
                wp = obj.waypoints(k,:);
            end
        end

        function advanceWaypointIfReached(obj)
            if isempty(obj.waypoints), return; end

            tol = obj.wpTol;
            if tol <= 0
                tol = obj.arrivalTol;
            end

            if norm(obj.position - obj.currentWaypoint()) <= tol
                n = size(obj.waypoints,1);

                if strcmpi(obj.wpMode,'loop')
                    obj.wpIndex = obj.wpIndex + 1;
                    if obj.wpIndex > n, obj.wpIndex = 1; end

                elseif strcmpi(obj.wpMode,'pingpong')
                    obj.wpIndex = obj.wpIndex + obj.wpDir;
                    if obj.wpIndex > n
                        obj.wpIndex = n-1;
                        obj.wpDir   = -1;
                    end
                    if obj.wpIndex < 1
                        obj.wpIndex = 2;
                        obj.wpDir   = +1;
                    end

                else % 'none'
                    if obj.wpIndex < n
                        obj.wpIndex = obj.wpIndex + 1;
                    else
                        if obj.stopAtGoal
                            obj.velocity = [0,0];
                        end
                    end
                end
            end
        end

        %% ===== 期望速度计算 =====
        function computePreferredVelocity(obj)
            % - CONST：沿 waypoint/constVel
            % - VO/其他：朝向 goal，模长为 maxSpeed

            if strcmpi(obj.mode, 'CONST')
                if ~isempty(obj.waypoints)  % waypoint 模式
                    wp  = obj.currentWaypoint();
                    d   = wp - obj.position;
                    nd  = norm(d);
                    spd = obj.constSpeed;
                    if spd <= 0, spd = obj.maxSpeed; end

                    if nd > 1e-6
                        obj.prefVelocity = (d / nd) * spd;
                    else
                        obj.prefVelocity = [0,0];
                    end
                else
                    % 固定世界速度
                    obj.prefVelocity = obj.constVel;
                end
            else
                % VO/其它：朝向 goal
                d    = obj.goal - obj.position;
                dist = norm(d);
                if dist > 0.2
                    obj.prefVelocity = (d / dist) * obj.maxSpeed;
                else
                    obj.prefVelocity = [0,0];
                end
            end
        end

        %% ===== 状态更新 =====
        function updatePosition(obj, dt, ~)
            if obj.useDynamics
                obj.updateDynamics(dt);
            else
                % 经典欧拉积分
                obj.position = obj.position + obj.velocity * dt;
            end

            if strcmpi(obj.mode,'CONST') && ~isempty(obj.waypoints)
                obj.advanceWaypointIfReached();
            end
        end
        
        % [MOD] 重写后的动力学更新：增加减速逻辑和航向防抖
        function updateDynamics(obj, dt)
            p = obj.dynParams;
            % 1. 获取动力学参数结构体
            if isempty(fieldnames(p)), return; end 
            % 2.防御性编程
            s = obj.dynState;
            % 3.获取当前的动力学状态（上一步的状态）
            % s 包含: u(纵向速), v(横向速), r(角速度), psi(航向), delta(舵角)
            v_cmd = obj.velocity;  
            distToGoal = norm(obj.goal - obj.position);
            % 4. 获取规划层给出的期望速度矢量 (由 VO 计算得出)
            slowDownDist = obj.maxSpeed * 3.0;  
            stopDist     = obj.arrivalTol;      
            % 5. 智能到达逻辑：防止过冲和绕圈
            % 定义减速区间：例如最大速度的 3 倍距离内开始减速
            targetSpeed = norm(v_cmd);
            
            % 如果进入减速区，强制衰减期望速度
            if distToGoal < slowDownDist
                decayFactor = max(0, (distToGoal - stopDist * 0.5) / (slowDownDist - stopDist * 0.5));
                targetSpeed = min(targetSpeed, obj.maxSpeed * decayFactor);
                % “零速目标点”设在了容差圈的圆心和边缘的中间，船舶试图在该点停下来
                % 极近距离且低速时，强制刹死
                if distToGoal < stopDist && s.u < 0.05
                    targetSpeed = 0;
                    s.u = 0; 
                end
            end
            % 逻辑全景图
            % 3米外：不管，听 VO 算法的，全速跑。
            % 3米 ~ 0.5米：执行 线性减速。距离每近一点，最高限速就低一点。
            % 进入 1米圈 且 速度很慢：判定为“停稳了”，直接清零速度，防止微小的漂移震荡。
            
            % 6. 目标航向计算与防抖
            if targetSpeed > 0.01
                % 只有速度足够大时才更新目标航向
                targetHeading = atan2(v_cmd(2), v_cmd(1));
            else
                % 速度极低时保持当前航向，防止 atan2(0,0) 随机跳变
                targetHeading = s.psi;
            end
            
            % 4. 控制器 (PD)
            epsi = atan2(sin(targetHeading - s.psi), cos(targetHeading - s.psi));
            
            % [新增] 死区控制：如果角度误差很小(<1度)，不动作，防止蛇形摆动
            if abs(epsi) < deg2rad(1.0)
                epsi = 0;
            end
            
            delta_cmd = p.Kp * epsi + p.Kd * (-s.r);
            delta_cmd = max(-p.deltaMax, min(p.deltaMax, delta_cmd));
            
            % 5. 舵机模型
            step_delta = max(-p.deltaRate*dt, min(p.deltaRate*dt, delta_cmd - s.delta));
            s.delta = s.delta + step_delta;

            % 6. 动力学方程积分
            % 旋转
            r_dot = (p.Kdel * s.delta - s.r) / max(1e-6, p.Tr);
            
            % 纵向：加减速不对称处理
            % 如果需要减速（当前速度 > 目标速度），使用更小的时间常数模拟主动刹车
            if s.u > targetSpeed
                effective_Tu = p.Tu * 0.2; % 刹车比加速快
            else
                effective_Tu = p.Tu;
            end
            
            u_dot = (targetSpeed - s.u) / max(1e-6, effective_Tu);
            u_dot = max(-p.aMax, min(p.aMax, u_dot));
            
            % 横漂
            v_dot = -p.Dv * s.v + p.Alpha * s.u * s.r;
            
            % 状态积分
            s.r = s.r + r_dot * dt;
            s.u = s.u + u_dot * dt;
            s.v = s.v + v_dot * dt;
            s.psi = wrapToPi(s.psi + s.r * dt); % 确保 [-pi, pi]

            % 位置更新
            dx = (s.u * cos(s.psi) - s.v * sin(s.psi)) * dt;
            dy = (s.u * sin(s.psi) + s.v * cos(s.psi)) * dt;
            
            obj.position = obj.position + [dx, dy];
            
            % [重要] 将实际物理速度写回 velocity，供下一帧 VO 算法使用
            obj.velocity = [s.u * cos(s.psi) - s.v * sin(s.psi), ...
                            s.u * sin(s.psi) + s.v * cos(s.psi)];
            
            obj.dynState = s;
        end

        function at = isAtGoal(obj)
            tol = obj.arrivalTol;
            if tol <= 0
                tol = obj.radius;
            end
            at = norm(obj.position - obj.goal) <= tol;
        end

        function reached = hasReachedGoal(obj)
            reached = obj.isAtGoal();
            if reached && obj.stopAtGoal
                % 这里不清零 velocity，交由动力学模型的减速逻辑处理
            end
        end

        %% ===== 邻居搜索 =====
        function findNeighbors(obj, allAgents)
            obj.neighbors = [];
            for k = 1:numel(allAgents)
                if allAgents(k).id == obj.id
                    continue;
                end
                if norm(obj.position - allAgents(k).position) < obj.neighborDist
                    obj.neighbors = [obj.neighbors, allAgents(k)]; %#ok<AGROW>
                end
            end
        end
    end
end