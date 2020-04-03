%% Prepare Workspace
clear;
clc;

%% Parameters
%Variables
syms q dq ddq [1 7] real
syms q0 x0 y0 real
syms dq0 dx0 dy0 real
syms ddq0 ddx0 ddy0 real
syms tau [1 7] real
syms l [1 3] real
syms l4m1 l4m2 l5m1 l5m2 l6 real
syms m I [1 6] real
syms mu real
q = [q2 q3 q4 q5 q6 x0 y0 q0];
dq = [dq2 dq3 dq4 dq5 dq6 dx0 dy0 dq0];
ddq = [ddq2 ddq3 ddq4 ddq5 ddq6 ddx0 ddy0 ddq0];
tau = [tau2 tau3 tau4 tau5 tau6];

%Constants
g = 9.81;   %gravity

%% Kinematics
%% Homogenous Transformations
% Back Leg Transforms
% Transform from the contact at the foot to the center of mass/object frame
gc1t1 = [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1];
gt1 = [1 0 0 -l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
g12 = [cos(q2) -sin(q2) 0 -l1/2; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l2/2;0 1 0 0;0 0 1 0;0 0 0 1];
g23 = [cos(q3) -sin(q3) 0 l2/2; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l3/2;0 1 0 0;0 0 1 0;0 0 0 1];
% g34 = [cos(q4) -sin(q4) 0 -l3/2; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l4m1;0 1 0 0;0 0 1 0;0 0 0 1];
g3s1 = [cos(q4) -sin(q4) 0 -l3/2; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1];
gc1s1 = gc1t1*gt1*g12*g23*g3s1; % Forward Kinematics

% s1 to t1
gps1 = [cos(q0) -sin(q0) 0 x0;sin(q0) cos(q0) 0 y0;0 0 1 0;0 0 0 1]*[1 0 0 l4m1;0 1 0 0;0 0 1 0;0 0 0 1];
gs13 = [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l3/2;0 1 0 0;0 0 1 0;0 0 0 1];
g32 = [cos(q3) -sin(q3) 0 l3/2; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l2/2;0 1 0 0;0 0 1 0;0 0 0 1];
g21 = [cos(q2) -sin(q2) 0 -l2/2; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
g1t = [1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
gt1c1 = [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1];
gs1t1 = gs13*g32*g21*g1t;    % Forward Kinematics

% s1 to c1
gs1c1 = [cos(q0) -sin(q0) 0 gs1t1(1,4);sin(q0) cos(q0) 0 gs1t1(2,4);0 0 1 0;0 0 0 1];

%Front Leg Transforms
% Transform from the contact at the foot to the center of mass/object frame
gc2t2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];
gt26 = [1 0 0 -l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g65 = [cos(q6) -sin(q6) 0 -l6/2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l5m2;0 1 0 0;0 0 1 0;0 0 0 1];
g54 = [cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l4m2;0 1 0 0;0 0 1 0;0 0 0 1];
gc2s2 = gc2t2*gt26*g65*[cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1];
gc24 = gc2t2*gt26*g65*g54;   % Forward Kinematics

%s2  to t2
gps2 = [cos(q0) -sin(q0) 0 x0;sin(q0) cos(q0) 0 y0;0 0 1 0;0 0 0 1]*[1 0 0 -l4m2;0 1 0 0;0 0 1 0;0 0 0 1];
gs25 = [cos(q5) -sin(q5) 0 0; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l5m1;0 1 0 0;0 0 1 0;0 0 0 1];
g56 = [cos(q6) -sin(q6) 0 l5m2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g6t = [1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
gt2c2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];
gs2t2 = gs25*g56*g6t;    % Forward Kinematics

% s2 to c2
gs2c2 = [cos(q0) -sin(q0) 0 gs2t2(1,4);sin(q0) cos(q0) 0 gs2t2(2,4);0 0 1 0;0 0 0 1];
% changed the transform to remove dependency on q1 and q7 (angle with
% ground) and put in terms of q0 and the other link angle based on HW7 Q2

%% Contact Kinematics
% Compute Force Contact Basis
B = [eye(2);zeros(4,2)];

% Fricton Cone requirement (U = fric_conditions*force >= 0)
mu = 1;
U = [0 1; 1 mu; -1 mu; 0 mu; 0 mu];

% Compute Hand Jacobian
% Back Leg
Adgc1s1 = tform2adjoint(inv(gs1c1));
gs1f1_inv = inv(gs1t1);
% J1s1f1 = rbvel2twist(simplify(diff(gs1f1 , q1)*gs1f1_inv));
J2s1f1 = rbvel2twist(simplify(diff(gs1t1 , q2)*gs1f1_inv));
J3s1f1 = rbvel2twist(simplify(diff(gs1t1 , q3)*gs1f1_inv));
J4s1f1 = rbvel2twist(simplify(diff(gs1t1 , q4)*gs1f1_inv));
Jss1f1 = simplify([J2s1f1 J3s1f1 J4s1f1]);
JhBack = B'*Adgc1s1*Jss1f1; % Back leg hand Jacobian

%Front Leg
Adgc2s2 = tform2adjoint(inv(gs2c2));
gs2f2_inv = inv(gs2t2);
% J7s2f2 = rbvel2twist(simplify(diff(gs2f2 , q7)*gs2f2_inv));
J6s2f2 = rbvel2twist(simplify(diff(gs2t2 , q6)*gs2f2_inv));
J5s2f2 = rbvel2twist(simplify(diff(gs2t2 , q5)*gs2f2_inv));
Jss2f2 = simplify([J5s2f2 J6s2f2]);
JhFront = B'*Adgc2s2*Jss2f2;    % Front leg hand Jacobian

% Hand Jacobian
Jh = [JhBack zeros(size(JhBack,1),size(JhFront,2)); zeros(size(JhFront,1),size(JhBack,2)) JhFront];

% Compute Grasp Map
%Simplified transform to use q0 x0 and y0 instead of gc14 and reliance on
%q1 & q7
gc1o = [cos(q0), -sin(q0), 0,x0
    sin(q0), cos(q0), 0, y0
    0, 0, 1, 0
    0, 0, 0, 1];
Adgc1o = tform2adjoint(gc1o);
Gs1 = -1*Adgc1o'*B;

% gc2o = gc24;
gc2o = gc1o;
Adgc2o = tform2adjoint(gc2o);
Gs2 = -1*Adgc2o'*B;

Gs = [Gs1 Gs2]; % Grasp map

% Compute friction contact
fc = [0;g;0;g];
% subs(Gs*fc,[q1,q2,q3,q4,q5,q6,q7],[pi/6,-pi/6,pi/6,-pi/6,0,pi/6,pi/6]);

%% Dyanamics 
%% Self Manipulation Dynamics
%Manually created based on HW7_2, accounts for position and orientation of
%O wrt to P
gpo = [cos(q0) -sin(q0) 0 x0; sin(q0) cos(q0) 0 y0; 0 0 1 0; 0 0 0 1];

%Transforms from P to links
gp5 = gps2*gs25;
gp6 = gp5*g56;
gp3 = gps1*gs13;
gp2 = gp3*g32;
gp1 = gp2*g21;

Jbpl3 = [zeros(6,2), rbvel2twist(simplify(gp3\diff(gp3 , q4))), zeros(6,2)];
Jbpl2 = [zeros(6,1), rbvel2twist(simplify(gp2\diff(gp2 , q3))), rbvel2twist(simplify(gp2\diff(gp2 , q4))), zeros(6,2)];
Jbpl1 = [rbvel2twist(simplify(gp1\diff(gp1 , q2))), rbvel2twist(simplify(gp1\diff(gp1 , q3))), rbvel2twist(simplify(gp1\diff(gp1 , q4))), zeros(6,2)];
Jbpl5 = [zeros(6,3), rbvel2twist(simplify(gp5\diff(gp5 , q5))), zeros(6,1)];
Jbpl6 = [zeros(6,3), rbvel2twist(simplify(gp6\diff(gp6 , q5))), rbvel2twist(simplify(gp6\diff(gp6 , q6)))];

% Compute mass matrix
M1 = [m1 0 0 0 0 0; 0 m1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I1];
M2 = [m2 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I2];
M3 = [m3 0 0 0 0 0; 0 m3 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I3];
Mo = [m4 0 0 0 0 0; 0 m4 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I4];
M5 = [m5 0 0 0 0 0; 0 m5 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I5];
M6 = [m6 0 0 0 0 0; 0 m6 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I6];

%Accounts for dx0, dy0, dtheta0
Jbpo = simplify([rbvel2twist(simplify(gpo\diff(gpo , x0))), rbvel2twist(simplify(gpo\diff(gpo , y0))), rbvel2twist(simplify(gpo\diff(gpo , q0)))]);

Adpl3 = tform2adjoint(inv(gp3))*Jbpo;
Adpl2 = tform2adjoint(inv(gp2))*Jbpo;
Adpl1 = tform2adjoint(inv(gp1))*Jbpo;
Adpl5 = tform2adjoint(inv(gp5))*Jbpo;
Adpl6 = tform2adjoint(inv(gp6))*Jbpo;

% Compute totalienrtial matrix
M11 = simplify(Jbpl1'*M1*Jbpl1 + Jbpl2'*M2*Jbpl2 + Jbpl3'*M3*Jbpl3 + Jbpl5'*M5*Jbpl5 + Jbpl6'*M6*Jbpl6);
M12 = simplify(Jbpl1'*M1*Adpl1 + Jbpl2'*M2*Adpl2 + Jbpl3'*M3*Adpl3 + Jbpl5'*M5*Adpl5 + Jbpl6'*M6*Adpl6); 
M21 = simplify(Adpl1'*M1*Jbpl1 + Adpl2'*M2*Jbpl2 + Adpl3'*M3*Jbpl3 + Adpl5'*M5*Jbpl5 + Adpl6'*M6*Jbpl6);
M22 = simplify(Adpl1'*M1*Adpl1 + Adpl2'*M2*Adpl2 + Adpl3'*M3*Adpl3 + Adpl5'*M5*Adpl5 + Adpl6'*M6*Adpl6 + Jbpo'*Mo*Jbpo);
Mbar = simplify([M11 M12; M21 M22]);

% Compute Coriolis Matrix
C  = sym(zeros(length(q),length(q)));
for ii = 1:length(q)
    for jj = 1:length(q)
        for kk = 1:length(q)
            C(ii,jj) = C(ii,jj) + 1/2*(diff(Mbar(ii,jj),q(kk)) + diff(Mbar(ii,kk),q(jj)) - diff(Mbar(jj,kk),q(ii)))*dq(kk);
        end
    end
end
C = simplify(C);

%Potential Energy and Nonlinear Forces (Spring forces here?)
% V = m1*g*gc11(2,4) + m2*g*gc12(2,4) + m3*g*gc13(2,4) + m4*g*y0 + m5*g*gc25(2,4) + m6*g*gc26(2,4);
% h3 = y0 + l4m1*sin(q4) - (l3/2)*sin(q4+q3);
% h2 = y0 + l4m1*sin(q4) - l3*sin(q4+q3) - (l2/2)*sin(q4+q3+q2);
% h1 = y0 + l4m1*sin(q4) - l3*sin(q4+q3) - l2*sin(q4+q3+q2) - l1/2*sin(q4+q3+q2+q1);
% h5 = y0 - l4m2*sin(q4) - l5m1*sin(q4+q5);
% h6 = y0 - l4m2*sin(q4) - (l5m1+l5m2)*sin(q4+q5) - (l6/2)*sin(q4+q5+q6);
h5 = simplify(gp5(2,4));
h6 = simplify(gp6(2,4));
h3 = simplify(gp3(2,4));
h2 = simplify(gp2(2,4));
h1 = simplify(gp1(2,4));
V = simplify(m1*g*h1 + m2*g*h2 + m3*g*h3 + m4*g*y0 + m5*g*h5 + m6*g*h6);
N = simplify(jacobian(V,q)');
% N = [diff(V, q1); diff(V, q2); diff(V, q3); diff(V, q5); diff(V, q6); diff(V, q4)];

% Compute A and dA
A = simplify([-Jh';Gs(1:2,:);Gs(6,:)]);
dA = sym(zeros(size(A)));
for i = 1:length(A)
    dA = dA + diff(A, q(i))*dq(i);  % Chain rule
end
dA = simplify(dA);

% Compute torque
Y = tau';

%% Generate matlabfunctions for simulation
%Robot Rear Half dimensions
h1 = 1/3;
%Robot Front Half Dimensions
h2 = 1/3;

m = [0.01 0.01 0.01 0.6 0.1 0.01];L = [1 1 1 1/3 1/3 1/3 1/6 1];

g = 9.8;
L = [.3 .5 .5 .3 .3 .2 .2 .25];
I = [(1/12)*m(1)*L(1)^2, (1/12)*m(2)*L(2)^2, (1/12)*m(3)*L(3)^2, (1/12)*m(4)*(L(4)^2 + h1^2), (1/12)*m(5)*(L(5)^2 + h2^2), (1/12)*m(6)*L(6)^2];

gpt1 = gp1*[1 0 0 l1/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
gpj2 = gp2*[1 0 0 -l2/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
gpj3 = gp3*[1 0 0 l3/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
gpj4 = gpo*[1 0 0 l4m1; 0 1 0 0; 0 0 1 0; 0 0 0 1];
gpj5 = gpo*[1 0 0 l4m2; 0 1 0 0; 0 0 1 0; 0 0 0 1];
gpt2 = gp6*[1 0 0 l6/2; 0 1 0 0; 0 0 1 0; 0 0 0 1];

a = [gpt1(2,4);gpt2(2,4)];
% a = [0;0];

a = subs(a,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6],L);
A = subs(A,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6],L);
dA = subs(dA,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6],L);
Mbar = subs(Mbar,[I1,I2,I3,I4,I5,I6,l1,l2,l3,l6,l4m1,l4m2,l5m1,l5m2,m1,m2,m3,m4,m5,m6],[I,L,m]);
C = subs(C,[I1,I2,I3,I4,I5,I6,l1,l2,l3,l6,l4m1,l4m2,l5m1,l5m2,m1,m2,m3,m4,m5,m6],[I,L,m]);
N = subs(N,[I1,I2,I3,I4,I5,I6,l1,l2,l3,l6,l4m1,l4m2,l5m1,l5m2,m1,m2,m3,m4,m5,m6],[I,L,m]);

matlabFunction(a,'File', 'computeFroga', 'Vars', {q});
matlabFunction(A,'File', 'computeFrogA', 'Vars', {q});
matlabFunction(dA,'File', 'computeFrogdA', 'Vars', {q,dq});
matlabFunction(Mbar,C,N,Y, 'File', 'computeDynamicMatricesFrog', 'Outputs', {'M','C','N','Y'}, 'Vars', {q,dq,tau});


%% Trajectory Optimizaton

% xi = zeros(16,1); %8 xi and 8 dxi
%       q2      q3      q4      q5      q6   x0  y0   q0     dq
xi = [pi/18; -pi/18; -5*pi/6; 5*pi/6; -pi/6; 1; 0.5; pi/6; zeros(8,1)];
ui = [1; zeros(4,1)];

takeoffphase = 0.4; %seconds 
dt = 0.01;
tspan = 0:dt:takeoffphase;
N = length(tspan);
vars = [xi;ui];

% Code to produce initial dynamically feasible trajectory with ode45
[t,x] = ode45(@Frogdynamics,tspan,xi', [], tspan);
X0 = [x,ui'.*ones(N,length(ui))];

%Got ode45 to run and produce some sort of x values, need to plot to see
%what is actually happening then move on to optimization

% Xi = zeros(N,numel(vars));
% 
% options = optimoptions('fmincon','Display', 'iter', 'MaxFunctionEvaluations', 1e5);
% [sol, cost] = fmincon(@minTorque, Xi, [],[],[],[],[],[],@nonlincon, options);
%
% xf = sol(:,1:4);
animateFrog(x,dt);

%% Initialization
% 
% % Define start and stop times, set a dt to keep outputs at constant time
% % interval
% tstart = 0;
% tfinal = 0.5;
% dt = 0.01;
% 
% % Initialize state and contact mode
% qi = [pi/18; -pi/18; -5*pi/6; 5*pi/6; -pi/6; 1; 0.5; pi/6]; 
% dqi = zeros(8,1);
% ui = zeros(5,1);
% disp(['Initial condition: [', num2str(qi'), ']''.'])
% xi = [qi;dqi;ui];
% contactMode = [];
% 
% %Main Loop
% 
% % Tell ode45 what event function to use and set max step size to make sure
% % we don't miss a zero crossing
% options = odeset('Events', @guardFunctions,'MaxStep',0.01);
% 
% % Initialize output arrays
% tout = tstart;
% xout = xi.';
% teout = [];
% xeout = [];
% ieout = [];
% while tstart < tfinal
%     % Initialize simulation time vector
%     tspan = [tstart:dt:tfinal];
%     
%     % Simulate
%     [t,x,te,xe,ie] = ode45(@dynamics,tspan,xi,options,contactMode);
%     
%     % Sometimes the events function will record a nonterminal event if the
%     % initial condition is a zero. We want to ignore this, so we will only
%     % use the last row in the terminal state, time, and index.
%     if ~isempty(ie)
%         te = te(end,:);
%         xe = xe(end,:);
%         ie = ie(end,:);
%     end
%     
%     % Log output
%     nt = length(t);
%     tout = [tout; t(2:nt)];
%     xout = [xout; x(2:nt,:)];
%     teout = [teout; te];
%     xeout = [xeout; xe];
%     ieout = [ieout; ie];
%     
%     % Quit if simulation completes
%     if isempty(ie) 
%         disp('Final time reached');
%         break; % abort if simulation has completed
%     end
%     
%     % If flag was caused by a_i < 0 (i not in contact mode), compute the
%     % proper contact mode via IV complemetarity
%     if any(ie <= length(setdiff([1:3], contactMode)))
%         % Determine next contact mode
%         contactMode = FcompIV(xe');
%         
%         % Compute the reset map
%         [dq_p, p_hat] = computeResetMap(xe',contactMode);
%         xe = [xe(1:2),dq_p'];
%         
%         % Report contact mode transition
%         disp(['Transition to contact mode {', num2str(contactMode'), '} at time t = ', num2str(te), ' s.'])
%     end
%     
%     % Check to see if there should be liftoff (positive lambda), if so
%     % compute the proper contact mode via FA complementarity
%     [ddq,lambda] = solveEOM(xe',contactMode);
%     if (-lambda)<=0
%         contactMode = compFA(xe');
%         
%         % Report contact mode transition
%         disp(['Transition to contact mode {', num2str(contactMode'), '} at time t = ', num2str(te), ' s.'])
%     end
%     
%     % Update initial conditions for next iteration
%     x0 = xe';
%     tstart = t(end);
%     
%     % Stop if the particle comes to a rest
%     if all(abs(dq_p)<1e-6)
%         break;
%     end
% end
% 
% animateFrog(xout, dt);

%% Draw Robot
close;
Fig = figure('Color', 'w');

%[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6];
L = [.3 .5 .5 .6 0 .4 0 .25];
% L = [1 1 1 1/3 1/2 1/3 1/2 0.5];

%[ q1  q2  q3  q4  q5  q6  q7]
%[-30 -30 -30 -30 120 -30 -30]
Q = [-pi/11, pi/18, -pi/18, -5*pi/6, 5*pi/6, -pi/6, -pi/8]; 

% Create trace of trajectory and particle object
% h = animatedline('LineStyle', ':', 'LineWidth', 1.5);

% Set up axes
axis equal
xlabel('x')
ylabel('y')

% % draw
base = [0;0];
gc1t1 = [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1];
gt1 = [1 0 0 -l1;0 1 0 0;0 0 1 0;0 0 0 1];
g12 = [cos(q2) -sin(q2) 0 0; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l2;0 1 0 0;0 0 1 0;0 0 0 1];
g23 = [cos(q3) -sin(q3) 0 0; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l3;0 1 0 0;0 0 1 0;0 0 0 1];
g34 = [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l4m1;0 1 0 0;0 0 1 0;0 0 0 1];
g45 = [cos(q5) -sin(q5) 0 0;sin(q5) cos(q5) 0 0;0 0 1 0;0 0 0 1]*[1 0 0 l5m1;0 1 0 0;0 0 1 0;0 0 0 1];
g56 = [cos(q6) -sin(q6) 0 0; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l6;0 1 0 0;0 0 1 0;0 0 0 1];

gc11 = gc1t1*gt1;
gc12 = gc11*g12;
gc13 = gc12*g23;
gc14 = gc13*g34;
gc15 = gc14*g45;
gc16 = gc15*g56;

% g6t = [1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
% gt2c2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];

joint1 = [gc11(1,4);gc11(2,4)];
joint2 = [gc12(1,4);gc12(2,4)];
joint3 = [gc13(1,4);gc13(2,4)];
joint4 = [gc14(1,4);gc14(2,4)];
joint5 = [gc15(1,4);gc15(2,4)];
joint6 = [gc16(1,4);gc16(2,4)];

joint1d = subs(joint1,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);
joint2d = subs(joint2,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);
joint3d = subs(joint3,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);
joint4d = subs(joint4,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);
joint5d = subs(joint5,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);
joint6d = subs(joint6,[l1 l2 l3 l4m1 l4m2 l5m1 l5m2 l6 q1 q2 q3 q4 q5 q6 q7], [L,Q]);

% joint1 = subs(joint1,[q1 q2 l1 l2], [Q(1),Q(2),L(1),L(2)]);
manipulator = line([base(1), joint1d(1), joint2d(1) joint3d(1) joint4d(1), joint5d(1), joint6d(1)], [base(2), joint1d(2), joint2d(2), joint3d(2), joint4d(2), joint5d(2), joint6d(2)], 'Color', [0;0;0],'LineStyle','-');
% endeffector = line(joint2(1), joint2(2), 'Color', [1;0;0],'Marker','.', 'MarkerSize', 20);
% addpoints(maipulator);
