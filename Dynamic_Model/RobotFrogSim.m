clear;
clc;

%Variables
syms q dq ddq [1 7] real
syms q0 x0 y0 real
syms dq0 dx0 dy0 real
syms ddq0 ddx0 ddy0 real
syms l [1 3] real
syms l4m1 l4m2 l5m1 l5m2 l6 real
syms m I [1 6] real
syms mu g real
q = [q2 q3 q4 q5 q6 x0 y0 q0];
dq = [dq2 dq3 dq4 dq5 dq6 dx0 dy0 dq0];
ddq = [ddq2 ddq3 ddq4 ddq5 ddq6 ddx0 ddy0 ddq0];

%Constants
% g = 9.81;   %gravity

%% Kinematics

%% Homogenous Transformations
%Back Leg Transforms
%c1 to o/4
gc1t1 = [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1];
gt1 = [1 0 0 -l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
g12 = [cos(q2) -sin(q2) 0 -l1/2; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l2/2;0 1 0 0;0 0 1 0;0 0 0 1];
g23 = [cos(q3) -sin(q3) 0 l2/2; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l3/2;0 1 0 0;0 0 1 0;0 0 0 1];
g34 = [cos(q4) -sin(q4) 0 -l3/2; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l4m1;0 1 0 0;0 0 1 0;0 0 0 1];
g3s1 = [cos(q4) -sin(q4) 0 -l3/2; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1];

gc11 = gc1t1*gt1;
gc12 = gc11*g12;
gc13 = gc12*g23;
gc14 = gc13*g34;

gc1s1 = gc13*g3s1;

%s1 or p to t1/f1
gps1 = [1 0 0 -l4m1;0 1 0 0;0 0 1 0;0 0 0 1];
gs13 = [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l3/2;0 1 0 0;0 0 1 0;0 0 0 1];
g32 = [cos(q3) -sin(q3) 0 l3/2; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l2/2;0 1 0 0;0 0 1 0;0 0 0 1];
g21 = [cos(q2) -sin(q2) 0 -l2/2; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
g1t = [1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
gt1c1 = [cos(q1) -sin(q1) 0 0;sin(q1) cos(q1) 0 0;0 0 1 0;0 0 0 1];

gs1f1 = gs13*g32*g21*g1t;
gs1c1 = gs1f1*gt1c1;

%Front Leg Transforms
%c2  to o/4
gc2t2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];
gt26 = [1 0 0 -l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g65 = [cos(q6) -sin(q6) 0 -l6/2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l5m2;0 1 0 0;0 0 1 0;0 0 0 1];
g54 = [cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l4m2;0 1 0 0;0 0 1 0;0 0 0 1];

gc26 = gc2t2*gt26;
gc25 = gc26*g65;
gc2s2 = gc2t2*gt26*g65*[cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1];
gc24 = gc2t2*gt26*g65*g54;


%s2  to t2/f2
gps2 = [1 0 0 l4m2;0 1 0 0;0 0 1 0;0 0 0 1];
gs25 = [cos(q5) -sin(q5) 0 0; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l5m1;0 1 0 0;0 0 1 0;0 0 0 1];
g56 = [cos(q6) -sin(q6) 0 l5m2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g6t = [1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
gt2c2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];

gs2f2 = gs25*g56*g6t;
gs2c2 = gs2f2*gt2c2;

%% Draw Robot
% Fig = figure('Color', 'w');
% 
% % Create trace of trajectory and particle object
% h = animatedline('LineStyle', ':', 'LineWidth', 1.5);
% manipulator = [];
% endeffector = [];
% 
% % Set up axes
% axis equal
% axis([xmin xmax ymin ymax])
% xlabel('x')
% ylabel('y')
% 
% % draw
% base = [0;0];
% L = 1;
% for ii = 1:length(x)
%     a = tic;
%     
%     set(gcf,'DoubleBuffer','on');
%     
%     q1 = x(ii,1);
%     q2 = x(ii,2);
%     joint1 = [L*cos(q1);L*sin(q1)];
%     joint2 = [L*cos(q1) + L*cos(q1+q2);L*sin(q1) + L*sin(q1+q2)]; 
%     delete(manipulator);
%     delete(endeffector);
%     manipulator = line([base(1), joint1(1), joint2(1)], [base(2), joint1(2), joint2(2)], 'Color', [0;0;0],'LineStyle','-');
%     endeffector = line(joint2(1), joint2(2), 'Color', [1;0;0],'Marker','.', 'MarkerSize', 20);
%     addpoints(h,joint2(1), joint2(2));
% 
% end

%% Jacobians
%Back Leg Jacobian
gc14_inv = inv(gc14);
%Jacobian for each individual link
J1 = rbvel2twist(simplify(gc14_inv*diff(gc14 , q1)));
J2 = rbvel2twist(simplify(gc14_inv*diff(gc14 , q2)));
J3 = rbvel2twist(simplify(gc14_inv*diff(gc14 , q3)));
J4 = rbvel2twist(simplify(gc14_inv*diff(gc14 , q4)));
J5 = rbvel2twist(simplify(gc14_inv*diff(gc14 , q5)));

%Jacobian combining effect of all previous linkages
J1a = [J1, J2, J3, J4];
J2a = [J2, J3, J4, zeros(6,1)];
J3a = [J3, J4, zeros(6,2)];
J4a = [J4, zeros(6,3)];

%Front Leg Jacobian


%% Contact Kinematics

%Force Contact Basis
B12 = [eye(2);zeros(4,2)];

%Fricton Cone requirement (U = fric_conditions*force >= 0)
U = [0 0 1; 1 0 mu; -1 0 mu; 0 1 mu; 0 -1 mu];

%Hand Jacobian
%Back Leg
Adgc1s1 = tform2adjoint(inv(gs1c1));
gs1f1_inv = inv(gs1f1);

% J1s1f1 = rbvel2twist(simplify(diff(gs1f1 , q1)*gs1f1_inv));
J2s1f1 = rbvel2twist(simplify(diff(gs1f1 , q2)*gs1f1_inv));
J3s1f1 = rbvel2twist(simplify(diff(gs1f1 , q3)*gs1f1_inv));
J4s1f1 = rbvel2twist(simplify(diff(gs1f1 , q4)*gs1f1_inv));

Jss1f1 = simplify([J2s1f1 J3s1f1 J4s1f1]);

JhBack = B12'*Adgc1s1*Jss1f1;

%Front Leg
Adgc2s2 = tform2adjoint(inv(gs2c2));
gs2f2_inv = inv(gs2f2);

% J7s2f2 = rbvel2twist(simplify(diff(gs2f2 , q7)*gs2f2_inv));
J6s2f2 = rbvel2twist(simplify(diff(gs2f2 , q6)*gs2f2_inv));
J5s2f2 = rbvel2twist(simplify(diff(gs2f2 , q5)*gs2f2_inv));

Jss2f2 = simplify([J5s2f2 J6s2f2]);

JhFront = B12'*Adgc2s2*Jss2f2;

Jh = [JhBack zeros(size(JhBack,1),size(JhFront,2)); zeros(size(JhFront,1),size(JhBack,2)) JhFront];

%Grasp Map
gc1o = gc14;
Adgc1o = tform2adjoint(gc1o);
Gs1 = -1*Adgc1o'*B12;

gc2o = gc24;
Adgc2o = tform2adjoint(gc2o);
Gs2 = -1*Adgc2o'*B12;

Gs = [Gs1 Gs2];
fc = [0;g;0;g];
% subs(Gs*fc,[q1,q2,q3,q4,q5,q6,q7],[pi/6,-pi/6,pi/6,-pi/6,0,pi/6,pi/6]);
%% Self Manipulation Dynamics
%Inertial Matrix
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

M11 = simplify(Jbpl1'*M1*Jbpl1 + Jbpl2'*M2*Jbpl2 + Jbpl3'*M3*Jbpl3 + Jbpl5'*M5*Jbpl5 + Jbpl6'*M6*Jbpl6);
M12 = simplify(Jbpl1'*M1*Adpl1 + Jbpl2'*M2*Adpl2 + Jbpl3'*M3*Adpl3 + Jbpl5'*M5*Adpl5 + Jbpl6'*M6*Adpl6); 
M21 = simplify(Adpl1'*M1*Jbpl1 + Adpl2'*M2*Jbpl2 + Adpl3'*M3*Jbpl3 + Adpl5'*M5*Jbpl5 + Adpl6'*M6*Jbpl6);
M22 = simplify(Adpl1'*M1*Adpl1 + Adpl2'*M2*Adpl2 + Adpl3'*M3*Adpl3 + Adpl5'*M5*Adpl5 + Adpl6'*M6*Adpl6 + Jbpo'*Mo*Jbpo);
Mbar = simplify([M11 M12; M21 M22]);

%% Coriolis Matrix
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
h3 = y0 + l4m1*sin(q4) - (l3/2)*sin(q4+q3)
h2 = y0 + l4m1*sin(q4) - l3*sin(q4+q3) - (l2/2)*sin(q4+q3+q2);
h1 = y0 + l4m1*sin(q4) - l3*sin(q4+q3) - l2*sin(q4+q3+q2) - l1/2*sin(q4+q3+q2+q1);
h5 = y0 - l4m2*sin(q4) - l5m1*sin(q4+q5);
h6 = y0 - l4m2*sin(q4) - (l5m1+l5m2)*sin(q4+q5) - (l6/2)*sin(q4+q5+q6);
V = simplify(m1*g*h1 + m2*g*h2 + m3*g*h3 + m4*g*y0 + m5*g*h5 + m6*g*h6);
N = simplify(jacobian(V,q)')
% N = [diff(V, q1); diff(V, q2); diff(V, q3); diff(V, q5); diff(V, q6); diff(V, q4)];

A = simplify([-Jh';Gs(1:2,:);Gs(6,:)]);
dA = sym(zeros(size(A)));
for i = 1:length(A)
    dA = dA + diff(A, q(i))*dq(i);  % Chain rule
end

