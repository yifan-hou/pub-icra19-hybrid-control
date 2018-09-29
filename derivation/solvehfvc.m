% Solve for hybrid force-velocity control actions
%
% Input arguments
%   required:
%       Omega
%       Jac_phi_q_all
%       G
%       b_G
%       F
%       Aeq
%       beq
%       A
%       b_A
%       dims
%           dims.Actualized
%           dims.UnActualized
%           dims.SlidingFriction
%           dims.Lambda
%
%   optional:
%       VelocitySampleSize

function [] = solvehfvc(Omega, Jac_phi_q_all, G, b_G, F, ...
        Aeq, beq, A, b_A, dims, varargin)

persistent para
if isempty(para)
    para = inputParser;
    validMatrix = @(x) isnumeric(x);
    validVector = @(x) isnumeric(x) && (size(x, 2) == 1);
    validStruct = @(x) isstruct(x);
    addRequired(para, 'Omega', validMatrix);
    addRequired(para, 'Jac_phi_q_all', validMatrix);
    addRequired(para, 'G', validMatrix);
    addRequired(para, 'b_G', validVector);
    addRequired(para, 'F', validVector);
    addRequired(para, 'Aeq', validMatrix);
    addRequired(para, 'beq', validVector);
    addRequired(para, 'A', validMatrix);
    addRequired(para, 'b_A', validVector);
    addRequired(para, 'dims', validStruct);
    addParameter(p, 'VelocitySampleSize', 100);
end

parse(para, Omega, Jac_phi_q_all, G, b_G, F, Aeq, beq, A, b_A, dims, varargin);

kVelocitySampleSize = para.Results.VelocitySampleSize;

% constants
kDimActualized      = dims.Actualized;
kDimUnActualized    = dims.UnActualized;
kDimSlidingFriction = dims.SlidingFriction;
kDimLambda          = dims.Lambda;

Jac_phi_q = Jac_phi_q_all(1 : end - kDimSlidingFriction, :);
kDimGeneralized = kDimActualized + kDimUnActualized;
assert(kDimLambda == size(Jac_phi_q_all, 1));

disp('============================================================');
disp('          Begin solving for velocity commands               ');
disp('============================================================');
disp('-------    Determine Possible Dimension of Control   -------');

N = Jac_phi_q*Omega;
NG = [N; G];

rank_N = rank(N);
rank_NG = rank(NG);

n_av_min = rank_NG - rank_N;
% n_av_max = kDimGeneralized - rank_N;
n_av = n_av_min;
n_af = kDimActualized - n_av;
% b_NG = [zeros(size(N, 1), 1); b_G];
basis_NG = null(NG);
basis_c = null([basis_NG';
        eye(kDimUnActualized), zeros(kDimUnActualized,kDimActualized)]);

disp(['r_N + n_a: ', num2str(rank_N + kDimActualized)]);
disp(['n_v: ', num2str(kDimGeneralized)]);
assert(rank_N + kDimActualized > kDimGeneralized);

disp('-------  Solve for Directions  -------')

% sample C
n_c = rank_NG - kDimUnActualized;
cost_all_samples = zeros(kVelocitySampleSize, 1);
rank_C_all_samples = zeros(kVelocitySampleSize, 1);
rank_NC_all_samples = zeros(kVelocitySampleSize, 1);
C_all_samples = zeros(n_av, kDimGeneralized, kVelocitySampleSize);
basis_N = null(N);
for i = 1:kVelocitySampleSize
    k = rand(n_c, n_av)-0.5;
    C = (basis_c*k)';
    C = normalizeByRow(C);
    C_all_samples(:, :, i) = C;
    % compute costs
    CC = abs(C*(C'));
    cost_c_independent = sum(sum(CC - diag(diag(CC))));
    cost_N_independent = sum(sum(abs(C*basis_N)));
    cost_all_samples(i) = cost_c_independent - cost_N_independent;
    % check independency
    rank_C_all_samples(i) = rank(C);
    rank_NC_all_samples(i) = rank([N;C]);
end

cost_all_samples(rank_C_all_samples~=n_av) = inf;
cost_all_samples(rank_NC_all_samples~=rank_NG) = inf;

[~, best_id] = min(cost_all_samples);

C_best = C_all_samples(:, :, best_id);

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);

b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;
w_v = C_best*v_star;
% disp('R_a:');
% disp(R_a);
% disp('v:');
% disp(R_a^-1*[zeros(n_af, 1); w_v]);
disp('============================================================');
disp('          Begin solving for force commands');
disp('============================================================');
% unactuated dimensions
H = [eye(kDimUnActualized), zeros(kDimUnActualized, kDimActualized)];
% Newton's laws
T_inv = T^-1;
M_newton = [zeros(kDimUnActualized, kDimLambda) H*T_inv; ...
            T*(Omega')*(Jac_phi_q_all') eye(kDimGeneralized); ...
            Aeq];
b_newton = [zeros(size(H,1), 1); -T*F; beq];

M_free = M_newton(:, [1:kDimLambda+kDimUnActualized, kDimLambda+kDimUnActualized+n_af+1:end]);
M_eta_af = M_newton(:, [kDimLambda+kDimUnActualized+1:kDimLambda+kDimUnActualized+n_af]);

% prepare the QP
%   variables: [free_force, dual_free_force, eta_af]
n_free = kDimLambda + kDimUnActualized + n_av;
n_dual_free = size(M_newton, 1);
% 0.5 x'Qx + f'x
qp.Q = diag([zeros(1, n_free + n_dual_free), ones(1, n_af)]);
qp.f = zeros(n_free + n_dual_free + n_af, 1);
% Ax<b
A_temp = [A(:, 1:kDimLambda), A(:, kDimLambda+1:end)*T_inv];
A_lambda_eta_u = A_temp(:, 1:kDimLambda+kDimUnActualized);
A_eta_af = A_temp(:, kDimLambda+kDimUnActualized+1:kDimLambda+kDimUnActualized+n_af);
A_eta_av = A_temp(:, kDimLambda+kDimUnActualized+n_af+1:end);

qp.A = [A_lambda_eta_u A_eta_av zeros(size(A, 1), n_dual_free) A_eta_af];
qp.b = b_A;
% Aeq = beq
qp.Aeq = [2*eye(n_free), M_free', zeros(n_free, n_af);
          M_free, zeros(size(M_free, 1)), M_eta_af];
qp.beq = [zeros(n_free, 1); b_newton];

options = optimoptions('quadprog', 'Display', 'final-detailed');
x = quadprog(qp.Q, qp.f, qp.A, qp.b, qp.Aeq, qp.beq, [], [], [],options);

disp('============================================================');
disp('                  Done. Examing results');
disp('============================================================');

lambda = x(1:kDimLambda);
force_force = x(n_free + n_dual_free + 1:end);

% hand_contact_force = lambda(1:2);
% table_contact_force = lambda([3 5]);
% binwall_contact_force = lambda([4 6]);


disp('World frame velocity:');
disp(R_a^-1*[zeros(n_af, 1); w_v]);

disp('World frame force:');
disp(R_a^-1*[force_force; zeros(n_av, 1)]);


% disp('hand contact force: ');
% disp(hand_contact_force');
% disp('table contact force: ');
% disp(table_contact_force');
% disp('binwall contact force: ');
% disp(binwall_contact_force');

disp('Equality constraints:');
disp(qp.beq - qp.Aeq*x);
disp('Inequality constraints b - Ax > 0:');
disp(qp.b - qp.A*x);