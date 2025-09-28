%% 领导者动力学与控制模块（三阶机械臂：0–20s 从 0 到 1，随后静止）
% 状态: x = [x1; x2; x3] = [q; q_dot; q_ddot]
% 动力学:
%   x1dot = x2
%   x2dot = x3
%   x3dot = -a1*x1 - a2*x2 - a3*x3 + b*u + eps*(1 - (x3/rho)^2)*x3 + d(t)
% 目标:
%   0–T 内: q 从 a=0 过渡到 b=1；t>=T 后: q=1, qdot=0, qddot=0 保持静止

function [dx_leader, u_leader, V_leader] = leader_module(x_leader, t, leader_params, trajectory_params, control_params)
    % 倒立摆领导者动力学模型（论文中的领导者模型）
    % 状态: x = [p_0; v_0] = [角度; 角速度]
    % 动力学: p_0 = v_0, v_0 = a_0
    % 其中 a_0 = I_0^(-1)(-m_0*g*l_0*sin(p_0) - C_0*v_0)
    
    % 将当前时间写入参数
    leader_params.t = t;

    % 期望轨迹（五次多项式）
    [qd, qd_dot] = generate_reference(t, trajectory_params);

    % 虚拟稳定化项：V = -(k1 e1 + k2 e2)
    V_leader = compute_leader_control(x_leader, [qd, qd_dot], control_params.leader_gains);

    % 反馈线性化输入：取消名义项并注入期望加速度 + V
    u_leader = compute_leader_input(x_leader, V_leader, qd_dot, leader_params);

    % 真实动力学（含扰动）
    dx_leader = compute_leader_dynamics(x_leader, u_leader, leader_params);
end

% ========================= 参考轨迹（五次多项式） =========================
% 0–T: qd = a + (b-a)*(10 s^3 - 15 s^4 + 6 s^5),  s=t/T
% T 后: qd=b, qd_dot=0
function [qd, qd_dot] = generate_reference(t, p)
    % 使用传入的参数，如果没有则使用默认值
    if isfield(p, 'a')
        a = p.a;
    else
        a = 0;
    end
    
    if isfield(p, 'b')
        b = p.b;
    else
        b = 2*pi;
    end
    
    if isfield(p, 'T')
        T = p.T;
    else
        T = 20;
    end
    if t <= 0
        s = 0;
    elseif t >= T
        s = 1;
    else
        s = t / T;
    end
    Delta = b - a;

    if s <= 0
        qd = a; qd_dot = 0;
    elseif s >= 1
        qd = b; qd_dot = 0;
    else
        qd      = a + Delta*(10*s^3 - 15*s^4 + 6*s^5);
        qd_dot  = Delta*(30*s^2 - 60*s^3 + 30*s^4)/T;
    end
end

% 兼容保留：若外部仍调用 generate_trajectory，仅返回位置参考
function u_input = generate_trajectory(t, trajectory_params)
    [qd, ~] = generate_reference(t, trajectory_params);
    u_input = qd;
end

% ========================= 虚拟控制（误差整形） =========================
% e1=p_0-qd, e2=v_0-qd_dot
% V = -(k1 e1 + k2 e2)  —— 作为"期望加速度"的稳定化项
function V_leader = compute_leader_control(x_leader, qd_pack, gains)
    p_0 = x_leader(1); v_0 = x_leader(2);
    qd = qd_pack(1);  qd_dot = qd_pack(2);

    e1 = p_0 - qd;
    e2 = v_0 - qd_dot;

    K1 = gains(1); K2 = gains(2);
    V_leader = -(K1*e1 + K2*e2);
end

% ========================= 反馈线性化输入 =========================
% u = I_0 * (期望加速度 + V) - (-m_0*g*l_0*sin(p_0) - C_0*v_0)
function u_leader = compute_leader_input(x_leader, V_leader, qd_dot_dot, params)
    p_0 = x_leader(1); v_0 = x_leader(2);

    % 倒立摆系统参数
    I_0 = params.I_0;
    m_0 = params.m_0;
    g = params.g;
    l_0 = params.l_0;
    C_0 = params.C_0;

    % 名义动力学项
    nominal_term = -m_0 * g * l_0 * sin(p_0) - C_0 * v_0;

    % 反馈线性化输入
    u_leader = I_0 * (qd_dot_dot + V_leader) - nominal_term;
end

% ========================= 真实动力学（含扰动） =========================
function dx_leader = compute_leader_dynamics(x_leader, u_leader, params)
    dx_leader = zeros(2,1);
    p_0 = x_leader(1); v_0 = x_leader(2);

    % 倒立摆系统参数
    I_0 = params.I_0;
    m_0 = params.m_0;
    g = params.g;
    l_0 = params.l_0;
    C_0 = params.C_0;

    % 动力学方程：p_0 = v_0, v_0 = a_0
    % 其中 a_0 = I_0^(-1)(-m_0*g*l_0*sin(p_0) - C_0*v_0 + u_leader)
    dx_leader(1) = v_0;
    dx_leader(2) = I_0^(-1) * (-m_0 * g * l_0 * sin(p_0) - C_0 * v_0 + u_leader);
end

% ========================= 缺省参数 =========================
function leader_params = create_default_leader_params()
    % 倒立摆系统参数（论文中的数值）
    leader_params = struct();
    leader_params.I_0 = 7;      % 转动惯量
    leader_params.m_0 = 4;       % 质量
    leader_params.g = 9.8;       % 重力加速度
    leader_params.l_0 = 1;       % 摆长
    leader_params.C_0 = 10;      % 阻尼系数

    % 运行时写入
    leader_params.t = 0.0;
end

function control_params = create_default_leader_control_params()
    % 将误差特征多项式定为 (s + ω)^2 = s^2 + 2ω s + ω^2
    % T=20s 时，建议 ω ∈ [0.5, 0.8]；默认取 ω=0.6
    omega = 0.6;
    k2 = 2*omega;
    k1 = omega^2;

    control_params = struct();
    control_params.leader_gains = [k1, k2];
end

function trajectory_params = create_default_trajectory_params()
    % 0–20s: 从 a=0 到 b=1；20s 后静止
    trajectory_params = struct();
    trajectory_params.a = 0.0;
    trajectory_params.b = 1.0;
    trajectory_params.T = 20.0;
end
