% object frame is set on the line contact between object and the table
function [n_av, n_af, R_a, w_v, force_force] = example_2D_block_tilting(inputs)
% clear;clc;

addpath ../derivation/generated


% weight
kObjectMass = 0.5;
kHandMass = 0.3;
kGravityConstant = 9.8;


% friction
kFrictionCoefficientTable = 0.8;
kFrictionCoefficientHand = 0.8;
kFrictionConeSides = 6;  % polyhedron approximation of friction cone
v_friction_directions = zeros(3, kFrictionConeSides);
kMinNormalForce = 10; % Newton

% inputs
if nargin == 0
    kObjectEdgeLength = 0.075;
    p_WH = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    kGoalVelocity = 0.5; % rad
    kTiltDirection = [1 0 0]';
    m_project = diag([1 0 1]);

    % initial poses
    p_WH0 = [0.4*kObjectEdgeLength, 0, kObjectEdgeLength]';
    p_WO0 = [kObjectEdgeLength/2, 0, 0]';

else
    p_WH = inputs.p_WH;

    kObjectEdgeLength = inputs.ObjectEdgeLength;
    kGoalVelocity     = inputs.GoalVelocity;
    kTiltDirection    = inputs.TiltDirection;
    m_project         = inputs.m_project;
    p_WH0             = inputs.p_WH0;
    p_WO0             = inputs.p_WO0;
end
kRotateAxis = cross([0 0 1]', kTiltDirection); % to the left
p_LineContact = p_WO0;
p_WO = p_WO0;
v_C2C0 = p_WH0 - p_LineContact;
q_WO = quatBTVec(m_project*v_C2C0, m_project*(p_WH - p_WO));


q = [p_WO; q_WO; p_WH];
disp('Input q: ');
disp(q);

% ------------------------------------------------------------------
% Generalized velocity: object body twist, hand linear velocity in world frame
kDimGeneralized = 9;
kDimUnActualized = 6;
kDimActualized = 3;
kDimLambda = 3*(1+2);

for i = 1:kFrictionConeSides
    v_friction_directions(1, i) = sin(2*pi*i/kFrictionConeSides);
    v_friction_directions(2, i) = cos(2*pi*i/kFrictionConeSides);
end

p_OHC = p_WH0 - p_WO0;
% p_HHC = [0, 0, 0]';

% object pose
R_WO = quat2m(q_WO);
E_qO = 0.5*[-q_WO(2) -q_WO(3) -q_WO(4);
            q_WO(1) -q_WO(4) q_WO(3);
            q_WO(4) q_WO(1) -q_WO(2);
            -q_WO(3) q_WO(2) q_WO(1)];
Omega = blkdiag(R_WO, E_qO, eye(3));

% contact point with table
p_OTC_all = [kRotateAxis*kObjectEdgeLength/2, -kRotateAxis*kObjectEdgeLength/2];
p_WTC_all = R_WO*p_OTC_all + p_WO*[1, 1];

% goal twist
t_WG = [-cross(kRotateAxis, p_LineContact); kRotateAxis]*kGoalVelocity;
Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];

t_OG = Adj_g_WO_inv*t_WG;

G = [eye(6) zeros(6, 3)];
b_G = t_OG;

% Holonomic constraints
Jac_phi_q = jac_phi_q_2D_block_tilting(p_WO, q_WO, p_WH, p_OHC, p_WTC_all, p_OTC_all);

% external force
F_WGO = [0 0 -kObjectMass*kGravityConstant]';
F_WGH = [0 0 -kHandMass*kGravityConstant]';
F = [R_WO'*F_WGO; zeros(3,1); F_WGH];

% newton's third law
H = [zeros(6, 3*(1+2)), eye(6), zeros(6, 3)];

% Artificial constraints
%   hand contact is sticking; (force is in world frame)
%   table contacts are sticking;
%   hand contact normal force lower bound
A = zeros(kFrictionConeSides*(1+2)+1, ...
        3*(1+2)+9);
z = [0 0 1]';
for i=1:kFrictionConeSides
    A(i, 1:3) = (v_friction_directions(:, i)' - kFrictionCoefficientHand*z')*(R_WO');
    A(kFrictionConeSides+i, 4:6) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
    A(2*kFrictionConeSides+i, 7:9) = ...
            v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
end
A(3*kFrictionConeSides + 1, 1:3) = -z'*(R_WO');
b_A = [zeros(kFrictionConeSides*(1+2), 1); -kMinNormalForce];

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

disp('cost/rank_C/rank_NC/filtered cost:');
disp(cost_all_samples);
disp(rank_C_all_samples);
disp(rank_NC_all_samples);
cost_all_samples(rank_C_all_samples~=n_av) = inf;
cost_all_samples(rank_NC_all_samples~=rank_NG) = inf;
disp(cost_all_samples);

[~, best_id] = min(cost_all_samples);

C_best = C_all_samples(:, :, best_id);

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);

b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;
w_v = C_best*v_star;

disp('============================================================');
disp('          Begin solving for force commands');
disp('============================================================');

% Newton's laws
T_inv = T^-1;
H_lambda = H(:, 1:kDimLambda);
H_f = H(:, kDimLambda+1:end);
M_newton = [H_lambda H_f*T_inv; T*(Omega')*(Jac_phi_q') eye(kDimGeneralized)];
b_newton = [zeros(size(H,1), 1); -T*F];

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

hc_force = x(1:3);
tc_force = x(4:kDimLambda);
un_force = x(kDimLambda+1:kDimLambda+kDimUnActualized);
vel_force = x(kDimLambda+kDimUnActualized+1:n_free);
dual_force = x(n_free+1:n_free+n_dual_free);
force_force = x(n_free + n_dual_free + 1:end);

disp('tc_force: ');
disp(tc_force');
disp('hc_force: ');
disp(hc_force');
disp('un_force: ');
disp(un_force');
disp('vel_force: ');
disp(vel_force');
% disp('dual_force: ');
% disp(dual_force');
disp('force_force: ');
disp(force_force');




