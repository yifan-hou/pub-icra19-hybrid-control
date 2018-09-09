clear;clc;
addpath generated

% weight
kObjectMass = 1.0;
kHandMass = 2.0;
kGravityConstant = 9.8;

% geometry
kObjectRadius = 0.04;
kObjectLength = 0.2;
kTableZ = 0.360;  % project rotation axis to this height to avoid ossilation

% friction
kFrictionCoefficientTable = 0.8;
kFrictionCoefficientHand = 1.2;
kPointsPerFaceContact = 4;  % contact points between object and hand
kFrictionConeSides = 8;  % polyhedron approximation of friction cone
v_friction_directions = zeros(3, kFrictionConeSides);

% inputs
p_WH = [0 0 0.5]';
q_WH = aa2quat(0.2, [1 0 0]');
kGoalVelocity = 0.5; % rad


% ------------------------------------------------------------------
kDimGeneralized = 12;
kDimUnActualized = 6;
kDimActualized = 6;
kDimLambda = 3*(1+kPointsPerFaceContact);

for i = 1:kFrictionConeSides
    v_friction_directions(1, i) = sin(2*pi*i/kFrictionConeSides);
    v_friction_directions(2, i) = cos(2*pi*i/kFrictionConeSides);
end
% origin of object frame is at the center of its bottom
% origin of gripper frame is at the center of palm flange surface
p_OHC_all = zeros(3, kPointsPerFaceContact);
for i = 1:kPointsPerFaceContact
    p_OHC_all(1, i) = kObjectRadius*sin(2*pi*i/kPointsPerFaceContact);
    p_OHC_all(2, i) = kObjectRadius*cos(2*pi*i/kPointsPerFaceContact);
    p_OHC_all(3, i) = kObjectLength;
end
p_HHC_all = p_OHC_all;
p_HHC_all(3, :) = 0;

% object pose
p_WO = p_WH - quatOnVec([0 0 1]', q_WH)*kObjectLength;
q_WO = q_WH;
q = [p_WO; q_WO; p_WH; q_WH];
disp('Input q: ');
disp(q);

R_WO = quat2m(q_WO);
R_WH = quat2m(q_WH);
E_qO = 0.5*[-q_WO(2) -q_WO(3) -q_WO(4);
			q_WO(1) -q_WO(4) q_WO(3);
			q_WO(4) q_WO(1) -q_WO(2);
			-q_WO(3) q_WO(2) q_WO(1)];
E_qH = 0.5*[-q_WH(2) -q_WH(3) -q_WH(4);
			q_WH(1) -q_WH(4) q_WH(3);
			q_WH(4) q_WH(1) -q_WH(2);
			-q_WH(3) q_WH(2) q_WH(1)];
Omega = blkdiag(R_WO, E_qO, R_WH, E_qH);

% contact point with table
p_Wtemp = quatOnVec([0 0 1]', q_WO);
p_Wtemp(3) = 0;
p_Otemp = R_WO'*p_Wtemp;
p_Otemp(3) = 0;
p_OTC = p_Otemp/norm(p_Otemp)*kObjectRadius; % table contact
p_WTC = R_WO*p_OTC + p_WO;
% p_WTC(3) = kTableZ;

% rotation axis on the table
v_WRotAxis = R_WO*p_Otemp;
v_WRotAxis(3) = 0;
v_WRotAxis = v_WRotAxis/norm(v_WRotAxis);

% goal twist
t_WG = [-cross(v_WRotAxis, p_WTC); v_WRotAxis]*kGoalVelocity;
Adj_g_WO_inv = [R_WO', -R_WO'*wedge(p_WO); zeros(3), R_WO'];

t_OG = Adj_g_WO_inv*t_WG;

G = [eye(6) zeros(6)];
b_G = t_OG;

% Holonomic constraints
Jac_phi_q = jac_phi_q(p_WO, q_WO, p_WH, q_WH, p_OTC, p_WTC, ...
        p_OHC_all, p_HHC_all);

% external force
F_WGO = [0 0 -kObjectMass*kGravityConstant]';
F_WGH = [0 0 -kHandMass*kGravityConstant]';
F = [R_WO'*F_WGO; zeros(3,1); R_WH'*F_WGH];

% newton's third law
H = [zeros(6, 3*(1+kPointsPerFaceContact)), eye(6), zeros(6)];

% Artificial constraints
A = zeros(kFrictionConeSides*(1+kPointsPerFaceContact), ...
        3*(1+kPointsPerFaceContact))+12;
z = [0 0 1]';
for i=1:kFrictionConeSides
    A(i, 1:3) = v_friction_directions(:, i)' - kFrictionCoefficientTable*z';
    for j=1:kPointsPerFaceContact
        A(j*kFrictionConeSides+i, j*3+1:j*3+3) = ...
                v_friction_directions(:, i)' - kFrictionCoefficientHand*z';
    end
end
b_A = zeros(size(A, 1), 1);


disp('============================================================');
disp('          Begin solving for velocity commands');
disp('============================================================');
disp('-------  Determine Possible Dimension of Control -------');
kVelocitySampleSize = 50;

N = Jac_phi_q*Omega;
NG = [N; G];

rank_N = rank(N)
rank_NG = rank(NG)

n_av_min = rank_NG - rank_N
n_av_max = kDimGeneralized - rank_N
n_av = n_av_min;

% b_NG = [zeros(size(N, 1), 1); b_G];
basis_NG = null(NG);
basis_c = null([basis_NG';
        eye(kDimActualized), zeros(kDimUnActualized,kDimActualized)]);

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
    k = rand(n_c, n_av);
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
T = [zeros(kDimUnActualized,kDimGeneralized);
        zeros(kDimActualized, kDimUnActualized), R_a];


disp('============================================================');
disp('          Begin solving for force commands');
disp('============================================================');

% Newton's laws
T_inv = T^-1;
H_lambda = H(:, 1:kDimLambda);
H_f = H(:, kDimLambda+1:end);
M_newton = [H_lambda H_f*T_inv; T*(Omega')*(Jac_phi_q') eye()]


Z_a = diag([0 0 0 1 1]);

Newton_A = [H_lambda H_f*T_inv; T*Omega'*J_Phi' Z_a];
Newton_b = [zeros(5,1); -T*F];

lamEta0 = Newton_A\Newton_b;
C_lambEta = null(Newton_A);

etaf0 = lamEta0(8);
NN = C_lambEta(8);
NN_inv = NN^-1;

artificial_A = [A_lambda A_f*T_inv]*C_lambEta*NN_inv;
artificial_b = b_A - [A_lambda A_f*T_inv]*(lamEta0 - C_lambEta*NN_inv*etaf0);