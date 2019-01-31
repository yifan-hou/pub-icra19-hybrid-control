% Solve for hybrid force-velocity control actions
%
% Input arguments
%   required:
%       Omega: matrix for mapping from generalized velocity to configuration
%               time derivatives: q_dot = Omega*v
%       Jac_phi_q_all: Jacobian of holonomic constraints w.r.t. configuration
%       G, b_G: Goal description, affine constraints on generalized velocity
%               G*v = b_G
%       F: External force vector. Same size as generalized force
%       Aeq, beq: Guard condition, Aeq*v = beq
%       A, b_A: Guard condition, A*v <= b_A
%       dims:
%           dims.Actualized: number of actualized dimensions
%           dims.UnActualized: number of unactualized dimensions
%           dims.SlidingFriction: number of sliding friction dimensions
%           dims.Lambda: number of reaction forces
%
%   optional:
%       num_seeds: number of random initializations to try when solving for
%               the velocity control
% Outputs
%   n_av: number, dimensionality of velocity controlled actions
%   n_af: number, dimensionality of force controlled actions
%   R_a: (n_av+n_af)x(n_av+n_af) matrix, the transformation that describes the
%           direction of velocity&force control actions
%   w_av: n_av x 1 vector, magnitudes of velocity controls
%   eta_af: n_af x 1 vector,  magnitudes of force controls


function [n_av, n_af, R_a, w_av, eta_af] = solvehfvc(Omega, ...
        Jac_phi_q_all, G, b_G, F, Aeq, beq, A, b_A, dims, varargin)

persistent para
if isempty(para)
    para = inputParser;
    validMatrix = @(x) isnumeric(x);
    validVector = @(x) isempty(x) || (isnumeric(x) && (size(x, 2) == 1));
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
    addParameter(para, 'num_seeds', 1);
end

parse(para, Omega, Jac_phi_q_all, G, b_G, F, Aeq, beq, A, b_A, dims, varargin{:});

kNumSeeds = para.Results.num_seeds;

% constants
kDimActualized      = dims.Actualized;
kDimUnActualized    = dims.UnActualized;
kDimSlidingFriction = dims.SlidingFriction;
kDimLambda          = dims.Lambda;
kDimContactForce    = kDimLambda + kDimSlidingFriction;

Jac_phi_q = Jac_phi_q_all(1 : end - kDimSlidingFriction, :);
kDimGeneralized = kDimActualized + kDimUnActualized;
assert(kDimLambda == size(Jac_phi_q, 1));

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
n_c = rank_NG - kDimUnActualized;

% Projected gradient descent
NIter   = 50;
basis_N = null(N);
BB      = basis_c'*basis_c;
NN      = basis_N*(basis_N');

cost_all = zeros(1, kNumSeeds);
k_all = rand([n_c, n_av, kNumSeeds]);
for seed = 1:kNumSeeds
    k  = k_all(:,:, seed);
    kn = normByCol(basis_c*k);
    k  = bsxfun(@rdivide, k, kn);

    for iter = 1:NIter
        % compute gradient
        g = zeros(n_c, n_av);
        costs = 0;
        for i = 1:n_av
            ki = k(:,i);
            for j = 1:n_av
                if i == j
                    continue;
                end
                kj = k(:,j);
                costs = costs + (ki'*BB*kj)^2;
                g(:, i) = g(:, i) + 2*(ki'*BB*kj)*BB*kj;
            end
            g(:, i) = g(:, i) - 2*(basis_c')*NN*basis_c*ki;
            costs   = costs - ki'*(basis_c')*NN*basis_c*ki;
        end
        % descent
        delta = 10;
        k     = k - delta*g;
        % project
        kn = normByCol(basis_c*k);
        k  = bsxfun(@rdivide, k, kn);
    end
    cost_all(seed) = costs;
    k_all(:,:,seed) = k;
    disp(['cost: ' num2str(costs)]);
end

[~, best_id] = min(cost_all);
k_best = k_all(:,:,best_id);
C_best = (basis_c*k_best)';

R_a = [null(C_best(:, kDimUnActualized+1:end))';
        C_best(:, kDimUnActualized+1:end)];
T = blkdiag(eye(kDimUnActualized), R_a);

b_NG = [zeros(size(N, 1), 1); b_G];
v_star = NG\b_NG;
w_av = C_best*v_star;


disp('============================================================');
disp('          Begin solving for force commands');
disp('============================================================');
% unactuated dimensions
H = [eye(kDimUnActualized), zeros(kDimUnActualized, kDimActualized)];
% Newton's laws
T_inv = T^-1;
M_newton = [zeros(kDimUnActualized, kDimContactForce) H*T_inv; ...
            T*(Omega')*(Jac_phi_q_all') eye(kDimGeneralized); ...
            Aeq];
b_newton = [zeros(size(H,1), 1); -T*F; beq];

M_free = M_newton(:, [1:kDimContactForce+kDimUnActualized, kDimContactForce+kDimUnActualized+n_af+1:end]);
M_eta_af = M_newton(:, [kDimContactForce+kDimUnActualized+1:kDimContactForce+kDimUnActualized+n_af]);

% prepare the QP
%   variables: [free_force, dual_free_force, eta_af]
n_free = kDimContactForce + kDimUnActualized + n_av;
n_dual_free = size(M_newton, 1);
% 0.5 x'Qx + f'x
qp.Q = diag([zeros(1, n_free + n_dual_free), ones(1, n_af)]);
qp.f = zeros(n_free + n_dual_free + n_af, 1);
% Ax<b
A_temp = [A(:, 1:kDimContactForce), A(:, kDimContactForce+1:end)*T_inv];
A_lambda_eta_u = A_temp(:, 1:kDimContactForce+kDimUnActualized);
A_eta_af = A_temp(:, kDimContactForce+kDimUnActualized+1:kDimContactForce+kDimUnActualized+n_af);
A_eta_av = A_temp(:, kDimContactForce+kDimUnActualized+n_af+1:end);

qp.A = [A_lambda_eta_u A_eta_av zeros(size(A, 1), n_dual_free) A_eta_af];
qp.b = b_A;
% Aeq = beq
qp.Aeq = [2*eye(n_free), M_free', zeros(n_free, n_af);
          M_free, zeros(size(M_free, 1)), M_eta_af];
qp.beq = [zeros(n_free, 1); b_newton];

options = optimoptions('quadprog', 'Display', 'final-detailed');
x = quadprog(qp.Q, qp.f, qp.A, qp.b, qp.Aeq, qp.beq, [], [], [],options);

disp('============================================================');
disp('                  Done.                                     ');
disp('============================================================');

% lambda = x(1:kDimLambda);
eta_af = x(n_free + n_dual_free + 1:end);

% hand_contact_force = lambda(1:2);
% table_contact_force = lambda([3 5]);
% binwall_contact_force = lambda([4 6]);


disp('World frame velocity:');
disp(R_a^-1*[zeros(n_af, 1); w_av]);

disp('World frame force:');
disp(R_a^-1*[eta_af; zeros(n_av, 1)]);


% disp('hand contact force: ');
% disp(hand_contact_force');
% disp('table contact force: ');
% disp(table_contact_force');
% disp('binwall contact force: ');
% disp(binwall_contact_force');

disp('Equality constraints violation:');
disp(sum(abs(qp.beq - qp.Aeq*x)));
disp('Inequality constraints b - Ax > 0 violation:');
b_Ax = qp.b - qp.A*x;
disp(sum(find(b_Ax < 0)));


end