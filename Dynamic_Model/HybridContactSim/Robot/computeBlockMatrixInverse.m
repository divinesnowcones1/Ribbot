function blockMatrixInv = computeBlockMatrixInverse(x,contactMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:8);
dq = x(9:16);
u = x(17:21);

% Compute A at the current state, only select rows for the current
% contact mode
A = computeFrogA(q)';
% A = A(contactMode,:);

if contactMode == 1
    A = A(1:2,:);
elseif contactMode == 2
    A = A(3:4,:);
elseif isempty(contactMode)
    A = 0;
else
    A = A;
end

[M,C,N,Y] = computeDynamicMatricesFrog(q,dq,u);

% Extract the number of constraints
c = size(A,1);

% % Compute mass matrix
% m = 1;
% M = m*eye(2);

% Compute block matrix inverse
blockMatrixInv = inv([M A';A, zeros(c,c)]);