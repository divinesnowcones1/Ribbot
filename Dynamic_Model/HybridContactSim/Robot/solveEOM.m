function [ddq,lambda] = solveEOM(x,contactMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:8);
dq = x(9:16);
u = x(17:21);

% Compute A and dA at the current state, only select rows for the current
% contact mode
A = computeFrogA(q')';
dA = computeFrogdA(q',dq')';

if contactMode == 1
    A = A(1:2,:);
    dA = dA(1:2,:);
elseif contactMode == 2
    A = A(3:4,:);
    dA = dA(3:4,:);
elseif isempty(contactMode)
    A = [];
    dA = [];
else
    A = A;
    dA = dA;
end

% A = A(contactMode,:);
% dA = dA(contactMode,:);

% Extract the number of constraints
c = size(A,1);

% Compute EOM matrices
[M,C,N,Y] =  computeDynamicMatricesFrog(q',dq',u');
Y = [Y;zeros(3,1)];
% Compute block matrix inverse
blockMatrixInv = computeBlockMatrixInverse(x,contactMode);

% Solve EOM
sol = blockMatrixInv*[Y - N;zeros(c,1)] - blockMatrixInv*[C;dA]*dq;

% Extract accelerations and contact forces
ddq = sol(1:8);
if length(sol)>=9
    lambda = sol(9:end);
else
    lambda = [];
end
end