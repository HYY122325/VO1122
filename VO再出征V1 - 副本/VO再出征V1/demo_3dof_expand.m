function demo_3dof_expand()
    % VISUALIZE_GCCO_CLEAN
    % V4极简版：
    % 1. 移除了所有碍眼的红圈/绿圈
    % 2. 仅保留轨迹线，视觉中心更集中
    % 3. 在“最近会遇点”打标记：红叉(X)=碰撞，绿圆点(O)=安全
    
    clc; close all;

    %% 1. 场景与参数
    os.pos = [0, 0];
    os.v   = 15 * 0.5144; 
    os.hdg = deg2rad(90); 
    
    ts.pos = [200, 250];
    ts.v   = 15 * 0.5144;
    ts.hdg = deg2rad(225); 
    ts.vel = ts.v * [cos(ts.hdg), sin(ts.hdg)];
    
    radius = 60;   
    T_pred = 40;   
    dt     = 0.1;
    
    % 动力学参数 (保持平滑)
    p.u_ref=os.v; p.Tr=1.5; p.Kdel=1.0; p.Tu=10.0; p.Dv=0.5; p.Alpha=0.1;
    p.Kp=0.8; p.Kd=3.0; p.deltaMax=deg2rad(35); p.deltaRate=deg2rad(10); p.aMax=1.0;

    %% 2. 计算
    [geo_min, geo_max] = scan_boundary(@(c) check_linear_safe(c, os, ts, radius, T_pred), os, ts, radius, T_pred);
    [dyn_min, dyn_max] = scan_boundary(@(c) check_dynamic_safe(c, os, ts, radius, T_pred, dt, p), os, ts, radius, T_pred);

    %% 3. 绘图
    figure('Color','w', 'Position', [100, 100, 1300, 600]);
    
    % === 左图: 航向空间 ===
    subplot(1, 2, 1); hold on; axis equal;
    title('航向决策空间', 'FontSize', 14, 'FontWeight', 'bold');
    draw_compass_grid();
    
    hGeo = draw_sector(geo_min, geo_max, 1.0, [1 0.6 0.6]); 
    if dyn_min < geo_min
        hDyn = draw_sector(dyn_min, geo_min, 1.0, [1 0.8 0.2]);
    end
    quiver(0,0, cos(os.hdg), sin(os.hdg), 0.8, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    lgd = legend([hGeo, hDyn], ...
        '几何禁区 (理论计算)', '扩展禁区 (操纵性限制)', ...
        'Location', 'southoutside');
    set(lgd, 'FontSize', 11);
    
    % === 右图: 物理轨迹 (清爽版) ===
    subplot(1, 2, 2); hold on; grid on; axis equal;
    title('轨迹推演 ', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('东 (m)'); ylabel('北 (m)');
    
    % 1. 绘制 TS 轨迹
    ts_end = ts.pos + ts.vel * T_pred;
    plot([ts.pos(1), ts_end(1)], [ts.pos(2), ts_end(2)], 'r:', 'LineWidth', 1.5);
    plot(ts.pos(1), ts.pos(2), 'r^', 'MarkerSize', 8, 'MarkerFaceColor','r');
    text(ts.pos(1), ts.pos(2)+10, 'TS', 'Color', 'r', 'FontWeight','bold');
    
    % 2. 绘制 OS 起点
    plot(os.pos(1), os.pos(2), 'b^', 'MarkerSize', 8, 'MarkerFaceColor','b');
    text(os.pos(1), os.pos(2)-10, 'OS', 'Color', 'b', 'FontWeight','bold');
    
    % 3. 方案 A (碰撞)
    target_A = geo_min - deg2rad(2);
    [trajA, ~] = simulate_3dof(os, ts, target_A, radius, T_pred, dt, p);
    hTrajA = plot(trajA.x, trajA.y, 'k--', 'LineWidth', 1.5);
    
    % 4. 方案 B (安全)
    target_B = dyn_min - deg2rad(2);
    [trajB, ~] = simulate_3dof(os, ts, target_B, radius, T_pred, dt, p);
    hTrajB = plot(trajB.x, trajB.y, 'b-', 'LineWidth', 2.5);
    
    % 5. 计算并标记 CPA (最近会遇点)
    distsA = vecnorm([trajA.x; trajA.y] - [trajA.tsx; trajA.tsy]);
    [minDistA, idxA] = min(distsA);
    
    distsB = vecnorm([trajB.x; trajB.y] - [trajB.tsx; trajB.tsy]);
    [minDistB, idxB] = min(distsB);
    
    % 标记A (红叉)
    posA = [trajA.x(idxA), trajA.y(idxA)];
    plot(posA(1), posA(2), 'rx', 'MarkerSize', 12, 'LineWidth', 2);
    text(posA(1)+10, posA(2), sprintf('最近 %.1fm (危险)小于船舶安全半径60', minDistA), 'Color', 'r', 'FontSize', 10);
    
    % 标记B (绿点)
    posB = [trajB.x(idxB), trajB.y(idxB)];
    plot(posB(1), posB(2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    text(posB(1)+10, posB(2), sprintf('最近 %.1fm (安全)', minDistB), 'Color', [0 0.5 0], 'FontSize', 10);

    % 图例
    lgd2 = legend([hTrajA, hTrajB], ...
        '方案A: 按几何边界 ', ...
        '方案B: 按扩展边界 ', ...
        'Location', 'southoutside');
    set(lgd2, 'FontSize', 11);
end

%% === 核心计算函数 (不变) ===
function [min_hdg, max_hdg] = scan_boundary(check_func, os, ts, radius, T)
    angles = linspace(-pi, pi, 360);
    unsafe = false(size(angles));
    for i = 1:length(angles), unsafe(i) = ~check_func(angles(i)); end
    [L, R] = find_longest_true_run(unsafe);
    min_hdg = angles(L); max_hdg = angles(R);
end

function safe = check_linear_safe(target_hdg, os, ts, radius, T)
    v_os_new = os.v * [cos(target_hdg), sin(target_hdg)];
    RV = ts.vel - v_os_new; RP = ts.pos - os.pos;
    a = dot(RV, RV); b = 2 * dot(RP, RV); c = dot(RP, RP) - radius^2;
    delta = b^2 - 4*a*c;
    if delta < 0, safe = true; return; end
    t1 = (-b - sqrt(delta)) / (2*a); t2 = (-b + sqrt(delta)) / (2*a);
    if (t1 > 0 && t1 < T) || (t2 > 0 && t2 < T) || (t1 < 0 && t2 > 0), safe = false; else, safe = true; end
end

function safe = check_dynamic_safe(target_hdg, os, ts, radius, T, dt, p)
    X0 = [os.v, 0, 0, os.hdg, 0]; 
    RP0 = ts.pos - os.pos;
    safe = forward_safe_3dof(RP0, [0,0], X0, p.u_ref, ts.vel, target_hdg, ...
        p.Kp, p.Kd, p.Kdel, p.Tr, p.Tu, p.Dv, p.Alpha, ...
        p.deltaMax, p.deltaRate, p.aMax, T, dt, radius);
end

function [traj, safe] = simulate_3dof(os, ts, C, radius, T, dt, p)
    u = os.v; v=0; r=0; psi=os.hdg; delta=0;
    Pos = os.pos; TS_Pos = ts.pos;
    N = floor(T/dt);
    traj.x = zeros(1,N); traj.y = zeros(1,N); traj.tsx = zeros(1,N); traj.tsy = zeros(1,N);
    safe = true;
    for i = 1:N
        TS_Pos = TS_Pos + ts.vel * dt;
        epsi = atan2(sin(C - psi), cos(C - psi));
        delta_cmd = p.Kp*epsi + p.Kd*(-r);
        delta_cmd = max(-p.deltaMax, min(p.deltaMax, delta_cmd));
        step = max(-p.deltaRate*dt, min(p.deltaRate*dt, delta_cmd - delta));
        delta = delta + step;
        r_dot = (p.Kdel*delta - r)/max(1e-6,p.Tr);
        u_dot = (p.u_ref - u)/max(1e-6,p.Tu); 
        v_dot = -p.Dv*v + p.Alpha*u*r;
        r=r+r_dot*dt; u=u+u_dot*dt; v=v+v_dot*dt; psi=psi+r*dt;
        Pos = Pos + [u*cos(psi) - v*sin(psi), u*sin(psi) + v*cos(psi)]*dt;
        traj.x(i) = Pos(1); traj.y(i) = Pos(2); traj.tsx(i) = TS_Pos(1); traj.tsy(i) = TS_Pos(2);
    end
end

function yes = forward_safe_3dof(RP0, P0_os, X0_os, u_ref, Vts, C, ...
    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, t_pred, dt, radius)
    u = X0_os(1); v = X0_os(2); r = X0_os(3); psi = X0_os(4); delta = X0_os(5);
    Pos = P0_os(:).'; Pnb0 = RP0(:).'; Vts  = Vts(:).';
    t = 0; yes = true;
    while t <= t_pred
        Pnb = Pnb0 + Vts * t;
        d = norm(Pnb - Pos);
        if t > 0 && d < radius - 1e-6, yes = false; return; end 
        epsi = atan2(sin(C - psi), cos(C - psi));
        delta_cmd = Kp*epsi + Kd*(-r);
        delta_cmd = max(-deltaMax, min(deltaMax, delta_cmd));
        step = max(-deltaRate*dt, min(deltaRate*dt, delta_cmd - delta));
        delta = delta + step;
        r_dot = (Kdel*delta - r)/max(1e-6,Tr);
        u_dot = (u_ref - u)/max(1e-6,Tu);
        v_dot = -Dv*v + Alpha*u*r;
        r=r+r_dot*dt; u=u+u_dot*dt; v=v+v_dot*dt; psi=psi+r*dt;
        Pos = Pos + [u*cos(psi) - v*sin(psi), u*sin(psi) + v*cos(psi)]*dt;
        t = t + dt;
    end
end

function [L,R] = find_longest_true_run(bvec)
    n = length(bvec); bb = [bvec, bvec];
    d = diff([false, bb, false]);
    starts = find(d==1); ends = find(d==-1)-1;
    [~, idx] = max(ends-starts);
    if isempty(idx), L=1; R=1; return; end
    L = mod(starts(idx)-1, n) + 1; R = mod(ends(idx)-1, n) + 1;
end

function draw_compass_grid()
    theta = linspace(0, 2*pi, 100);
    plot(cos(theta), sin(theta), '-', 'Color', [0.9 0.9 0.9]);
    line([-1 1], [0 0], 'Color', [0.9 0.9 0.9]);
    line([0 0], [-1 1], 'Color', [0.9 0.9 0.9]);
end

function h = draw_sector(angle1, angle2, r, color)
    if angle2 < angle1, angle2 = angle2 + 2*pi; end
    th = linspace(angle1, angle2, 50);
    x = [0, r*cos(th), 0]; y = [0, r*sin(th), 0];
    h = patch(x, y, color, 'FaceAlpha', 0.9, 'EdgeColor', 'none');
end
