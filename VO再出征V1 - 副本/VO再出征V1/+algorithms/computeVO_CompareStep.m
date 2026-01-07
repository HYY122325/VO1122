function [v_new, v_ref, dv] = computeVO_CompareStep(agent, neighbors, params)
% computeVO_CompareStep — 对比测试工具 (节流版)
% 
% 特性：
% 1. 红色报错：随时打印，绝不错过。
% 2. 绿色通过：每隔 1.0 秒打印一次，防止刷屏。

% 1. 跑新代码
v_new = algorithms.computeVO_Step5L(agent, neighbors, params);

% 2. 检查旧代码是否存在
check1 = exist('algorithms.computeVO_Step5L_ref', 'file');
check2 = exist('+algorithms/computeVO_Step5L_ref.m', 'file');
check3 = exist('computeVO_Step5L_ref', 'file');

if check1 == 2 || check2 == 2 || check3 == 2
    
    % 3. 跑旧代码
    v_ref = algorithms.computeVO_Step5L_ref(agent, neighbors, params);
    
    % 4. 计算误差
    dv = norm(v_new - v_ref);
    
    % === 验证逻辑 ===
    if dv > 1e-10
        % 【ERROR】报错必须立即、醒目地打印出来，不受时间限制
        fprintf(2, '[ERROR] Agent %d 结果不一致! dv = %.15f\n', agent.id, dv);
        fprintf(2, '    New: [%.4f, %.4f]\n', v_new(1), v_new(2));
        fprintf(2, '    Ref: [%.4f, %.4f]\n', v_ref(1), v_ref(2));
    else
        % 【OK】成功信息进行“节流”：每1秒打印一次
        
        % 获取当前时间
        t_now = 0;
        if isfield(params, 'sim') && isfield(params.sim, 'time')
            t_now = params.sim.time;
        end
        
        % 核心技巧：判断时间是否接近整数秒 (例如 1.0, 2.0)
        % mod(t_now, 1.0) 会得到小数部分，如果小数部分接近0或1，说明是整秒
        if abs(mod(t_now, 1.0)) < 1e-4 || abs(mod(t_now, 1.0) - 1.0) < 1e-4
            fprintf('[OK] Agent %d Pass. dv = %.2e (t=%.1fs)\n', agent.id, dv, t_now);
        end
    end
    
else
    % 如果找不到参考文件
    v_ref = [NaN, NaN];
    dv = NaN;
    
    % 只在第1个智能体、第1帧时打印一次警告
    if agent.id == 1 && isfield(params,'sim') && params.sim.step == 1
        warning('vo5L:RefNotFound', '未找到 algorithms.computeVO_Step5L_ref，无法对比。');
    end
end

end