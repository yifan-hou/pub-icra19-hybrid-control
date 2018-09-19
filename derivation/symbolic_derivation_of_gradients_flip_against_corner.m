clear;clc;
% symbols
p_WO     = sym('p_WO',[2,1],'real');
theta    = sym('theta', 'real');
p_WH     = sym('p_WH',[2,1],'real');

% parameters
p_OHC = sym('p_OHC', [2,1], 'real');
p_OTC = sym('p_OTC', [2,1], 'real');
p_OBC = sym('p_OBC', [2,1], 'real');

R_WO = aa2mat(theta, [1 0 0]');
R_WO = R_WO(2:3, 2:3);
% Hand contact; table contacts; bin contacts
holonomic_constraint = sym('Phi', [6, 1], 'real');
% Hand contact
holonomic_constraint(1:2) = p_WH - (R_WO*p_OHC+p_WO);
% table non-penetration
holonomic_constraint(3) = [0 1] * (R_WO*p_OTC+p_WO);
% bin wall non-penetration
holonomic_constraint(4) = [1 0] * (R_WO*p_OBC+p_WO);
% table sliding
holonomic_constraint(5) = [1 0] * (R_WO*p_OTC+p_WO);
% bin wall sliding friction
holonomic_constraint(6) = [0 1] * (R_WO*p_OBC+p_WO);

holonomic_constraint = simplify(holonomic_constraint);

save generated/derivation.mat;
disp('step 1 Done');

%% ---------------------------------------------------------------
%           calculate derivatives
% ---------------------------------------------------------------
% load generated/derivation
deriv_q  = @(f) [
        diff(f,'p_WO1'), diff(f,'p_WO2'), ...
        diff(f,'theta'), ...
        diff(f,'p_WH1'), diff(f,'p_WH2')];

Phi_q  = simplify(deriv_q(holonomic_constraint));
disp('step 2 done');

%% ---------------------------------------------------------------
%           write to file
% ---------------------------------------------------------------
matlabFunction(Phi_q, 'File', 'generated/jac_phi_q_flip_against_corner', ...
    'vars', {p_WO, theta, p_WH, p_OHC, p_OTC, p_OBC});
save generated/derivation.mat;
disp('step 3 done');