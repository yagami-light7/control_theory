function [K, poles_cl] = calc_LQR_K(A, B, Q, R)
% CALC_LQR_K 计算 LQR 反馈增益矩阵 K
% 
% 输入参数:
%   A - 系统状态矩阵 (n x n)
%   B - 输入矩阵 (n m)
%   Q - 状态权重矩阵 (n x n)
%   R - 控制权重矩阵 (m x m)
%
% 输出参数:
%   K        - 最优反馈增益矩阵 (u = -Kx)
%   poles_cl - 闭环系统的极点 (用于检查稳定性)

    %% 1. 安全检查：系统维度验证
    [n_states, ~] = size(A);
    [n_inputs] = size(B, 2);
    
    % 检查 Q 和 R 的维度是否匹配
    if size(Q, 1) ~= n_states || size(Q, 2) ~= n_states
        error('错误：Q 矩阵的维度必须是 %dx%d', n_states, n_states);
    end
    
    if size(R, 1) ~= n_inputs || size(R, 2) ~= n_inputs
        error('错误：R 矩阵的维度必须是 %dx%d', n_inputs, n_inputs);
    end

    %% 2. 核心检查：可控性 (Controllability)
    % 如果系统不可控，LQR 是算不出来的，或者算出来也没用
    Co = ctrb(A, B);
    if rank(Co) < n_states
        error('❌ 严重错误：系统不可控 (Uncontrollable)！无法设计 LQR 控制器。请检查 A, B 矩阵。');
    end

    %% 3. 计算 LQR 增益
    try
        [K, S, poles_cl] = lqr(A, B, Q, R);
    catch ME
        error('LQR 计算失败，请检查 Q 是否半正定，R 是否正定。\n错误信息: %s', ME.message);
    end

    %% 4. 输出调试信息 (可选)
    fprintf('\n----------------------------------------\n');
    fprintf('✅ LQR 计算成功\n');
    fprintf('反馈增益 K = \n');
    disp(K);
    fprintf('闭环极点 (Closed-loop Poles) = \n');
    disp(poles_cl);
    fprintf('----------------------------------------\n');

end