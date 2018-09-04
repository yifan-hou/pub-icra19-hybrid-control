clear;clc;
% constants
mg          = 10;
thetad_goal = 0.3;
mu          = 0.5;
F_normal    = 4.5;
L1          = 0.2;
L2          = 0.3;

% current q
q.theta = 45*pi/180;

% matrices
Omega = eye(5);

F = [0 -mg 0 0 0]';
G = [0 0 1 0 0];
b_G = thetad_goal;

A_f      = zeros(3,5);
A_lambda = [0 0 -1 0; 0 0 -mu -1; 0 0 -mu 1];

b_A = [-F_normal; 0; 0];

J_Phi = [1 0 -L2*sin(q.theta) -1  0;
		 0 1  L2*cos(q.theta)  0 -1;
		 0 1 -L1*cos(q.theta)  0  0;
		 1 0  L1*sin(q.theta)  0  0];

H_f = eye(5);
H_lambda = [ 0  0 0 0;
	  0  0 0 0;
	  0  0 0 0;
	 -1  0 0 0;
	  0 -1 0 0];


disp('Begin solving for velocity commands.');

N = J_Phi*Omega;
NG = [N; G];
b_NG = [0 0 0 0 b_G']';

rank_N = rank(N);
rank_NG = rank(NG);

v1 = NG\b_NG;

rwv = null([v1(end-1) v1(end) -1]);

gamma_basis = rwv(1:end-1, :);

Nsample = 10;
k = rand(2, Nsample)-0.5;

gamma_sampled = gamma_basis*k;
gamma_sampled = normalizeByCol(gamma_sampled);

nullN = null(N);
alpha_sampled = [zeros(3, Nsample); gamma_sampled];
beta_sampled = v1'*alpha_sampled;
cost_sampled = -normByCol(nullN'*alpha_sampled);

[~, best_id] = min(cost_sampled);

best_constraint = alpha_sampled(:, best_id);

best_beta = best_constraint'*v1;

R_a = [null(best_constraint(4:5)')'; best_constraint(4:5)'];
T = [eye(3) zeros(3,2);zeros(2, 3) R_a];


Z_a = diag([0 0 0 1 1]);

T_inv = T^-1;
Newton_A = [H_lambda H_f*T_inv; T*Omega'*J_Phi' Z_a];
Newton_b = [zeros(5,1); -T*F];

lamEta0 = Newton_A\Newton_b;
C_lambEta = null(Newton_A);

etaf0 = lamEta0(8);
NN = C_lambEta(8);
NN_inv = NN^-1;

artificial_A = [A_lambda A_f*T_inv]*C_lambEta*NN_inv;
artificial_b = b_A - [A_lambda A_f*T_inv]*(lamEta0 - C_lambEta*NN_inv*etaf0);
