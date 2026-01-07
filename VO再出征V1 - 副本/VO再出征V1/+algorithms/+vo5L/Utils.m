classdef Utils
% Utils — 通用工具集（几何/数学/区间/3DoF 扩展）
%
% 为避免模块拆分过碎，本文件集中放置“跨模块共享”的工具函数。
% （对应原 monolithic 版本底部的“外部工具函数”与若干共享 helpers）
methods(Static)

    %% ===== struct helper =====
    function val = getfieldwithdef(s, name, def)
        if isfield(s,name), val=s.(name); else, val=def; end
    end

    %% ===== CPA / Hit =====
    function [dcpa, tcpa, tstar] = compute_dcpa_tcpa(RP, RV)
        a = dot(RV,RV);
        if a < 1e-10
            tstar = inf; tcpa = inf; dcpa = norm(RP); return;
        end
        tstar = -dot(RP,RV)/a;
        tcpa  = max(0, tstar);
        dcpa  = norm(RP + RV*tcpa);
    end

    function [hit, ttc] = dcpaTcpaHit(RP, RV, radius, timeHorizon, perceptionDist)
        a = dot(RV, RV);
        r0 = norm(RP);
        if r0 > perceptionDist, hit = false; ttc = inf; return; end
        if a < 1e-8
            if r0 <= 1.2*radius, hit = true; ttc = 0; else, hit = false; ttc = inf; end
            return;
        end
        dotrv = dot(RP, RV);
        approaching = (dotrv < 0);
        if ~approaching, hit = false; ttc = inf; return; end
        tcpa0 = - dotrv / a;
        if tcpa0 > timeHorizon, hit = false; ttc = inf; return; end
        tcpa = max(0, tcpa0);
        dcpa = norm(RP + RV * tcpa);
        hit = (dcpa <= radius);
        if hit, ttc = tcpa; else, ttc = inf; end
    end

    %% ===== Heading =====
    function hdg = headingFromVelocity(agent)
        if isprop(agent, 'useDynamics') && agent.useDynamics && isprop(agent, 'dynState')
            if isfield(agent.dynState, 'psi') && isfinite(agent.dynState.psi)
                hdg = agent.dynState.psi; return;
            end
        end
        v = agent.velocity;
        if norm(v) < 1e-6
            g = agent.goal - agent.position;
            hdg = atan2(g(2), g(1)); if ~isfinite(hdg), hdg=0; end
        else
            hdg = atan2(v(2), v(1)); if ~isfinite(hdg), hdg=0; end
        end
    end

    %% ===== Angle / Vector =====
    function d = angleDiff(a,b)
        d = atan2(sin(a-b), cos(a-b));
    end

    function d = angdiff_signed(a,b)
        d = atan2(sin(a-b), cos(a-b));
    end

    function c = cos_between(a,b)
        na = norm(a); nb = norm(b);
        if na < 1e-9 || nb < 1e-9, c = 1; return; end
        c = dot(a,b)/(na*nb);
        c = max(-1,min(1,c));
    end

    %% ===== ds/dp（Goodwin）=====
    function [ds, dp] = compute_ds_dp_paper(RP, refHdg, ds0, dp_fac, useGoodwin_)
        if useGoodwin_
            B = abs(algorithms.vo5L.Utils.angleDiff(atan2(RP(2), RP(1)), refHdg));
            ds = algorithms.vo5L.Utils.ds_from_goodwin_piecewise(B, ds0);
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

    %% ===== Interval extraction (CCO/BCO) =====
    function [Al,Ar,Bl,Br] = local_find_intervals(hdgGrid, in_core, in_bco)
        Al=NaN; Ar=NaN; Bl=NaN; Br=NaN;
        N = numel(hdgGrid);
        idxCore = find(in_core);
        if isempty(idxCore)
            [L1,R1] = algorithms.vo5L.Utils.longest_run_circ(in_bco);
            mask12  = algorithms.vo5L.Utils.idx_range_mask_circ(N,L1,R1);
            [L2,R2] = algorithms.vo5L.Utils.longest_run_circ(in_bco & ~mask12);
            if ~isempty(L1), Al=hdgGrid(1+mod(L1-1,N)); Bl=hdgGrid(1+mod(R1-1,N)); end
            if ~isempty(L2), Ar=hdgGrid(1+mod(L2-1,N)); Br=hdgGrid(1+mod(R2-1,N)); end
            return
        end
        [Lc, Rc] = algorithms.vo5L.Utils.longest_run_circ(in_core);
        [L1,R1] = algorithms.vo5L.Utils.nearest_run_to_circ(Lc, in_bco, 'left');
        [L2,R2] = algorithms.vo5L.Utils.nearest_run_to_circ(Rc, in_bco, 'right');
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
        L=mod(Lc-1,N)+1;
        R=mod(Rc-1,N)+1;
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
        ix = min(ix, numel(cand));
        Lc=starts(cand(ix)); Rc=ends(cand(ix));
        L=mod(Lc-1,N)+1;
        R=mod(Rc-1,N)+1;
        if R<L, R=R+N; end
    end

    %% ===== GCCO helpers =====
    function tf = any_is_in_gcco(C, ob)
        tf = false;
        if all(isfinite([ob.gAl, ob.gBl])) && algorithms.vo5L.Utils.is_between_circ(C, ob.gAl, ob.gBl), tf = true; return; end
        if all(isfinite([ob.gAr, ob.gBr])) && algorithms.vo5L.Utils.is_between_circ(C, ob.gAr, ob.gBr), tf = true; return; end
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

    %% ===== 3DoF 扩展（沿用原实现）=====
    function gNear = expand_one_side_3dof(nearB, farB, dirSign, radius, ...
        agent, RP0, hdg0, u_ref, Vts, X0_os, ...
        Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
        t_pred, dt, scanStep, scanMax)

        gNear = nearB;
        stepMax = ceil(scanMax / scanStep);
        if ~isempty(X0_os) && numel(X0_os) == 5
            X0 = X0_os;
        else
            u0 = max(1e-3, norm(agent.velocity));
            psi0 = hdg0; v0 = 0; r0 = 0; delta0 = 0;
            X0 = [u0, v0, r0, psi0, delta0];
        end
        for m = 0:stepMax
            C = atan2(sin( nearB + dirSign*m*scanStep ), cos( nearB + dirSign*m*scanStep ));
            if dirSign < 0
                if algorithms.vo5L.Utils.ang_passed(C, farB, nearB, -1), break; end
            else
                if algorithms.vo5L.Utils.ang_passed(C, nearB, farB, +1), break; end
            end
            if algorithms.vo5L.Utils.forward_safe_3dof(RP0, [0,0], X0, u_ref, Vts, C, ...
                    Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, ...
                    t_pred, dt, radius)
                gNear = C; break;
            end
        end
    end

    function yes = forward_safe_3dof(RP0, P0_os, X0_os, u_ref, Vts, C, ...
        Kp,Kd,Kdel,Tr,Tu,Dv,Alpha,deltaMax,deltaRate,aMax, t_pred, dt, radius)

        if numel(X0_os) ~= 5, yes = false; return; end
        u = X0_os(1);
        v = X0_os(2);
        r = X0_os(3);
        psi = X0_os(4);
        delta = X0_os(5);
        Pos = P0_os(:).';
        Pnb0 = RP0(:).';
        Vts  = Vts(:).';
        t = 0;
        yes = true;
        while t <= t_pred
            Pnb = Pnb0 + Vts * t;
            d = norm(Pnb - Pos);
            if t > 0 && d < radius - 1e-6, yes = false; return; end
            epsi = atan2(sin(C - psi), cos(C - psi));
            delta_cmd = Kp*epsi + Kd*(-r);
            delta_cmd = max(-deltaMax, min(deltaMax, delta_cmd));
            step_delta = max(-deltaRate*dt, min(deltaRate*dt, delta_cmd - delta));
            delta = delta + step_delta;
            r_dot = (Kdel*delta - r)/max(1e-6,Tr);
            u_dot = (u_ref - u)/max(1e-6,Tu);  u_dot = max(-aMax, min(aMax, u_dot));
            v_dot = -Dv*v + Alpha*u*r;
            r = r + r_dot*dt;
            u = u + u_dot*dt;
            v = v + v_dot*dt;
            psi = atan2(sin(psi + r*dt), cos(psi + r*dt));
            Pos = Pos + [u*cos(psi) - v*sin(psi), u*sin(psi) + v*cos(psi)] * dt;
            t = t + dt;
        end
    end

    function tf = ang_passed(C, L, R, dirSign)
        if dirSign < 0
            tf = algorithms.vo5L.Utils.angleLess(C, L) | algorithms.vo5L.Utils.angleGreater(C, R);
        else
            tf = algorithms.vo5L.Utils.angleGreater(C, R) | algorithms.vo5L.Utils.angleLess(C, L);
        end
    end

    function tf = angleLess(a,b)
        tf = atan2(sin(a-b),cos(a-b)) < 0;
    end

    function tf = angleGreater(a,b)
        tf = atan2(sin(a-b),cos(a-b)) > 0;
    end

    %% ===== Assist Rule mask =====
    function ok = assist_rule_mask(Ccand, v_ts, Vsea_loc, RVo, RVm)
        v_c  = Vsea_loc*[cos(Ccand), sin(Ccand)];
        RVOS = v_c - v_ts;
        lhs = algorithms.vo5L.Utils.cos_between(RVo, RVOS) - algorithms.vo5L.Utils.cos_between(RVo, RVm);
        ok = (lhs > 1e-6);
    end

end

end
