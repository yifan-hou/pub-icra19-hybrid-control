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
kObjectMass = 0.12;
kHandMass = 0.0;
kGravityConstant = 9.8;

% friction
kFrictionCoefficientHand = 0.7; % lower bound to ensure sticking
kFrictionCoefficientTable = 0.4; % upper bound to ensure sliding
kFrictionCoefficientBin = 0.4; % upper bound to ensure sliding
kMinNormalForce = 5; % Newton
kMinNormalForceSliding = -5; % Newton
kMaxNormalForceSliding = 100; % Newton


kDimGeneralized = 5;
kDimUnActualized = 3;
kDimActualized = 2;
kDimLambda = 6;
kDimSlidingFriction = 2;

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
% 2. solve for z
l_diagonal = sqrt(kObjectThickness^2 + kObjectLength^2);
angle_inner_sharp = asin(kObjectThickness/l_diagonal);
p_temp = [kObjectThickness*sin(theta)+l_diagonal/2*cos(theta+angle_inner_sharp);
        l_diagonal/2*sin(theta+angle_inner_sharp)];
p_temp_w = p_WO0 - [kObjectLength/2; kObjectThickness/2];
p_WO = p_temp + p_temp_w;

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
G = [0 1 0 0 0];
b_G = goal_velocity_z;

% Holonomic constraints
Jac_phi_q_all = jac_phi_q_flip_against_corner(p_WO, theta, p_WH, p_OHC, p_OTC, p_OBC);
Jac_phi_q = Jac_phi_q_all(1:end-kDimSlidingFriction, :);

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

