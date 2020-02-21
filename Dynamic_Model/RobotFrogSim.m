clear;
clc;

%Variables
syms q dq [1 7] real
syms l [1 3] real
syms l4m1 l4m2 l5m1 l5m2 l6 real
syms m I [1 6] real
syms mu

%Constants
g = 9.81;   %gravity

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

%s1 to t1/f1
gs13 = [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l3/2;0 1 0 0;0 0 1 0;0 0 0 1];
g32 = [cos(q3) -sin(q3) 0 l3/2; sin(q3) cos(q3) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l2/2;0 1 0 0;0 0 1 0;0 0 0 1];
g21 = [cos(q2) -sin(q2) 0 -l2/2; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];
g1t = [1 0 0 l1/2;0 1 0 0;0 0 1 0;0 0 0 1];

gs1f1 = gs13*g32*g21*g1t;
% gs1f11 = g3s1*g23*g12*gt1;
gs1f11 = inv(gt1*g12*g23*g3s1);

%Front Leg Transforms
%c2  to o/4
gc2t2 = [cos(q7) -sin(q7) 0 0;sin(q7) cos(q7) 0 0;0 0 1 0;0 0 0 1];
gt26 = [1 0 0 -l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g65 = [cos(q6) -sin(q6) 0 -l6/2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l5m2;0 1 0 0;0 0 1 0;0 0 0 1];
g54 = [cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 -l4m2;0 1 0 0;0 0 1 0;0 0 0 1];

gc24 = gc2t2*gt26*g65*g54;
gc2s2 = gc2t2*gt26*g65*[cos(q5) -sin(q5) 0 -l5m1; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1];

%s2 to t2/f2
gs25 = [cos(q5) -sin(q5) 0 0; sin(q5) cos(q5) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l5m1;0 1 0 0;0 0 1 0;0 0 0 1];
g56 = [cos(q6) -sin(q6) 0 l5m2; sin(q6) cos(q6) 0 0; 0 0 1 0; 0 0 0 1]*[1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];
g6t = [1 0 0 l6/2;0 1 0 0;0 0 1 0;0 0 0 1];

gs2f2 = gs25*g56*g6t;

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
Adgc1s1 = tform2adjoint(gc1s1);
gf1s1 = inv(gs1f1);
% gs1f1_inv = inv(gs1f11);

J1s1f1 = rbvel2twist(simplify(diff(gs1f1 , q1)*gf1s1));
J2s1f1 = rbvel2twist(simplify(diff(gs1f1 , q2)*gf1s1));
J3s1f1 = rbvel2twist(simplify(diff(gs1f1 , q3)*gf1s1));
J4s1f1 = rbvel2twist(simplify(diff(gs1f1 , q4)*gf1s1));

% J1s1f11 = rbvel2twist(simplify(diff(gs1f11 , q1)*gs1f1_inv));
% J2s1f11 = rbvel2twist(simplify(diff(gs1f11 , q2)*gs1f1_inv));
% J3s1f11 = rbvel2twist(simplify(diff(gs1f11 , q3)*gs1f1_inv));
% J4s1f11 = rbvel2twist(simplify(diff(gs1f11 , q4)*gs1f1_inv));

Jss1f1 = [J1s1f1 J2s1f1 J3s1f1 J4s1f1];

JhBack = B12'*Adgc1s1*Jss1f1;

%Front Leg
Adgc2s2 = tform2adjoint(gc2s2);
gs2f2_inv = inv(gs2f2);

J7s2f2 = rbvel2twist(simplify(diff(gs2f2 , q7)*gs2f2_inv));
J6s2f2 = rbvel2twist(simplify(diff(gs2f2 , q6)*gs2f2_inv));
J5s2f2 = rbvel2twist(simplify(diff(gs2f2 , q5)*gs2f2_inv));

Jss2f2 = [J7s2f2 J6s2f2 J5s2f2];

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


%% Dynamics

% M1 = [m1 0 0 0 0 0; 0 m1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I1];
% M2 = [m2 0 0 0 0 0; 0 m2 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I2];
% M3 = [m3 0 0 0 0 0; 0 m3 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I3];
% M4 = [m4 0 0 0 0 0; 0 m4 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I4];
% M5 = [m5 0 0 0 0 0; 0 m5 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 I5];
% 
% %Inertial Matrix
% M = J1a'*M1*J1a + J2a'*M2*J2a + J3a'*M3*J3a + J4a'*M4*J4a + J5a'*M5*J5a;
% 
% %Corriolis Forces
% C  = sym(zeros(length(q),length(q)));
% for i = 1:length(q)
%     for j = 1:length(q)
%         for k = 1:length(q)
%             C(i,j) = C(i,j) + 1/2*(diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)))*dq(k);
%         end
%     end
% end
% C = simplify(C);
% 
% %Potential Energy and Nonlinear Forces (Spring forces here?)
% V = m1*g*gc1(2,4) + m2*g*gc2(2,4) + m3*g*gc3(2,4) + m4*g*gc4(2,4) + m5*g*gc5(2,4);
% N = [diff(V, q1); diff(V, q2); diff(V, q3); diff(V, q4); diff(V, q5)];

