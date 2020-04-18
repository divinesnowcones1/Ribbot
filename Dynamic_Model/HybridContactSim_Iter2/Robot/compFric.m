function dq = compFric(x, contactMode, fricMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:2);
dq = x(3:4);

U = [0 1; 1 mu; -1 mu];

[ddq, lambda] = solveEOM(x, contactMode);

dq_mag = sqrt(dq'*dq);

fz = lambda;
fx = lambda;

N = U*[fx;fz];


end

