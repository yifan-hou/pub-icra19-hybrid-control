% state
%   [y_o, z_0, theta_o, y_h, z_h]
% choice of coordinate frames
%   see figure
% Goal is specified as the velocity of hand contact in z direction
function [n_av, n_af, R_a, w_v, force_force] = example_flip_against_corner(inputs)
% clear;clc;
addpath ../derivation/generated

% how to cope with sliding friciton
% 1. separate jacobian for velocity and force. force has more entries for friction
% 2. Guard condition:
%       equality constraint on friction and normal force
%       normal force upper limit
% 3. for sliding friciton, friciton coefficient should use upper bound

% weight
kObjectMass = 1.5;
kHandMass = 0.0;
kGravityConstant = 9.8;

% friction
kFrictionCoefficientHand = 0.8; % lower bound to ensure sticking
kFrictionCoefficientTable = 0.6; % upper bound to ensure sliding
kFrictionCoefficientBin = 0.3; % upper bound to ensure sliding
kMinNormalForce = 5; % Newton
kMinNormalForceSliding = 0.5; % Newton
kMaxNormalForceSliding = 100; % Newton


kDimGeneralized = 5;
kDimUnActualized = 3;
kDimActualized = 2;
kDimLambda = 6;


% inputs
if nargin == 0
    kObjectLength = 0.1;
    kObjectThickness = 0.02;
    kGoalRotationVelocity = 0.5; % rad
    % feedback
    p_WH = [kObjectLength, kObjectThickness/2]';

    % initial poses
    p_WH0 = [kObjectLength, kObjectThickness/2]';
    p_WO0 = [kObjectLength/2, kObjectThickness/2]';
else
    kObjectLength = inputs.kObjectLength;
    kObjectThickness = inputs.kObjectThickness;
    kGoalRotationVelocity = inputs.kGoalRotationVelocity; % rad
    % feedback
    p_WH = inputs.p_WH;

    % initial poses
    p_WH0 = inputs.p_WH0;
    p_WO0 = inputs.p_WO0;
end

% compute object pose
%   Ideally this should be done by perception; here we hack this by assuming the
%   contact between the hand and the object is sticking, and solve the object
%   pose from hand pose

% 1. Solve for theta
%   This is a a*sin(theta)+bsin(theta)=c problem
a = p_WH0(2) - p_WO0(2) + kObjectThickness/2;
b = kObjectLength;
c = p_WH(2) - p_WO0(2) + kObjectThickness/2;
phi = atan2(a, b);
theta_plus_phi = asin(c/norm([a, b]));
theta = theta_plus_phi - phi;
% 2. solve for y and z
p_WO = [kObjectThickness*sin(theta), kObjectThickness*cos(theta)]';

R_WO = aa2mat(theta, [1 0 0]');
R_WO = R_WO(2:3, 2:3);

% Generalized velocity = q time derivative
% thus Omega = I
% object pose
Omega = eye(5);

% contact point with Table, Bin and Hand
p_OHC = p_WH0 - p_WO0;
p_OTC = [-kObjectLength/2; -kObjectThickness/2];
p_OBC = [-kObjectLength/2; kObjectThickness/2];
% goal
goal_velocity_z = kGoalRotationVelocity*kObjectLength*cos(theta);
G = [0 0 1 0 0];
b_G = goal_velocity_z;

% Holonomic constraints
Jac_phi_q_all = jac_phi_q_flip_against_corner(p_WO, theta, p_WH, p_OHC, p_OTC, p_OBC);
Jac_phi_q = Jac_phi_q_all(1:end-2, :);

% external force
F_WGO = [0 -kObjectMass*kGravityConstant]';
F_WGH = [0 -kHandMass*kGravityConstant]';
F = [F_WGO; 0; F_WGH];

% Guard Conditions
%   Inequality A*lambda<b_A
%       hand contact is sticking; (force is in world frame)
%       hand contact normal force lower bound
%       table contact normal force lower bound
%       table contact normal force upper bound
%       binwall contact normal force lower bound
%       binwall contact normal force upper bound

%   Equality Aeq*lambda = beq
%       table contact is sliding;
%       bin wall contact is sliding;
% lambda: f_why, f_whx, f_table_normal, f_binwall_normal,
%         f_table_friction, f_binwall_friction
A = zeros(2 + 1 + 4, kDimLambda + kDimGeneralized);
y = [1 0]';
z = [0 1]';
A(1, 1:2) = (z' - kFrictionCoefficientHand*y')*(R_WO');
A(2, 1:2) = (-z' - kFrictionCoefficientHand*y')*(R_WO');
A(3, 1:2) = -y'*(R_WO');
A(4, 3) = -1;
A(5, 3) = 1;
A(6, 4) = -1;
A(7, 4) = 1;
b_A = [0 0 -kMinNormalForce -kMinNormalForceSliding kMaxNormalForceSliding ...
        -kMinNormalForceSliding kMaxNormalForceSliding]';

Aeq = zeros(2, kDimLambda + kDimGeneralized);
% % % -y'*ftable = mu*z'*ftable
Aeq(1, [5, 3]) = kFrictionCoefficientTable*z'+y';
% % z'*f = mu*y'*f
Aeq(2, [4, 6]) = kFrictionCoefficientBin*y'-z';
beq = [0; 0];
disp('============================================================');
disp('          Begin solving for velocity commands               ');
disp('============================================================');
disp('-------    Determine Possible Dimension of Control   -------');
kVelocitySampleSize = 500;

N = Jac_phi_q*Omega;
NG = [N; G];

rank_N = rank(N)
rank_NG = rank(NG)

n_av_min = rank_NG - rank_N
n_av_max = kDimGeneralized - rank_N
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

% disp('cost/rank_C/rank_NC/filtered cost:');
% disp(cost_all_samples);
% disp(rank_C_all_samples);
% disp(rank_NC_all_samples);
cost_all_samples(rank_C_all_samples~=n_av) = inf;
cost_all_samples(rank_NC_all_samples~=rank_NG) = inf;
% disp(cost_all_samples);

[~, best_id] = min(cost_all_samples);

C_best = C_all_samples(:, :, best_id);

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);

b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;
w_v = C_best*v_star;
disp('R_a:');
disp(R_a);
disp('w_v:');
disp(w_v);
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

hand_contact_force = lambda(1:2);
table_contact_force = lambda([3 5]);
binwall_contact_force = lambda([4 6]);
disp('hand contact force: ');
disp(hand_contact_force');
disp('table contact force: ');
disp(table_contact_force');
disp('binwall contact force: ');
disp(binwall_contact_force');

disp('Equality constraints:');
disp(qp.beq - qp.Aeq*x);
disp('Inequality constraints b - Ax > 0:');
disp(qp.b - qp.A*x);
