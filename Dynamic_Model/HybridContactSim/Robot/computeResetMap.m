function [dq_p, p_hat] = computeResetMap(x,contactMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:8);
dq = x(9:16);
u = x(17:21);

% Compute A at the current state, only select rows for the current
% contact mode
A = computeFrogA(q')';
% A = A(contactMode,:);

if contactMode == 1
    A = A(1:2,:);
elseif contactMode == 2
    A = A(3:4,:);
elseif isempty(contactMode)
    A = [];
else
    A = A;
end


% Extract the number of constraints
c = size(A,1);

% Compute block matrix inverse
[M,C,N,Y] = computeDynamicMatricesFrog(q',dq',u');
blockMatrixInv = computeBlockMatrixInverse(x,contactMode);

% Compute the mass matrix
% m = 1;
% M = m*eye(2);

% Solve for post impact states and impulses
sol = blockMatrixInv*[M*dq; zeros(c,1)]; % Solve equation for dq_p and p_hat

dq_p = sol(1:2);    % Extract dq_p
p_hat = sol(3:end);   % Extract p_hat
