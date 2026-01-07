function newVelocity = computeVelocity(agent, neighbors, params)
% 统一算法选择的入口（便于后续 step2-5 无缝切换）
switch params.algorithm.name
    case 'VO'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO(agent, neighbors, params);
    case 'VO_step5T'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5T(agent, neighbors, params);
    case 'VO_Step5I'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5I(agent, neighbors, params);
    case 'VO_Step3A'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step3A(agent, neighbors, params);
    case 'VO_Step5'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5(agent, neighbors, params);
    case 'VO_Step5F'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5F(agent, neighbors, params);
    case 'VO_Step5A'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5A(agent, neighbors, params);
    case 'VO_Step5E'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5E(agent, neighbors, params);
    case 'VO_Step5C'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5C(agent, neighbors, params);
    case 'VO_Step5D'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5D(agent, neighbors, params);
    case 'VO_Step5G'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5G(agent, neighbors, params);
    case 'VO_Step5H'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5H(agent, neighbors, params);
    case 'VO_Step5J'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5J(agent, neighbors, params);
    case 'VO_Step5K'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5K(agent, neighbors, params);
    case 'VO_Step5L'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
        newVelocity = algorithms.computeVO_Step5L(agent, neighbors, params);
    case 'VO_CompareStep'   % 本次提交：LVO-CCO/BCO（从TTC过渡）
       [v_new, v_ref, dv] = algorithms.computeVO_CompareStep(agent, neighbors, params);
       newVelocity = v_new;
    otherwise
        % 兜底：仍然可以挂回你原来的 VO/RVO/ORCA（后续可继续并存）
        newVelocity = algorithms.computeVO(agent, neighbors, params);
end
end
