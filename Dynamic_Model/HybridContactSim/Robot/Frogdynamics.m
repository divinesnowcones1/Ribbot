function dx = dynamics(t,x, tspan)
dx = zeros(4,1);

q = x(1:8);
dq = x(9:16);
u = [0;0;0;0;0];

dx(1:8) = dq;
A = computeFrogA(q');           %returns A'
dA = computeFrogdA(q',dq');     %return dA'
[M,C,N,Y] = computeDynamicMatricesFrog(q',dq',u');
Y = [Y;zeros(3,1)]; %Adding zeros for x0 y0 and q0
lamda =  (A'/M*A)\(A'/M*(Y-C*dq-N)+dA'*dq);
dx(9:16) = M\(Y-C*dq-N-A*lamda);    %%ddx

end