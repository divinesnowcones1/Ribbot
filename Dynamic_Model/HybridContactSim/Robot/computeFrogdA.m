function dA = computeFrogdA(in1,in2)
%COMPUTEFROGDA
%    DA = COMPUTEFROGDA(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    20-Mar-2020 17:02:48

dq0 = in2(:,8);
dq2 = in2(:,1);
dq3 = in2(:,2);
dq4 = in2(:,3);
dq5 = in2(:,4);
dq6 = in2(:,5);
dx0 = in2(:,6);
dy0 = in2(:,7);
q0 = in1(:,8);
q2 = in1(:,1);
q3 = in1(:,2);
q4 = in1(:,3);
q5 = in1(:,4);
q6 = in1(:,5);
t2 = cos(q0);
t3 = sin(q0);
t6 = -dq0;
t7 = -dy0;
t8 = -q0;
t9 = -q4;
t10 = -q5;
t4 = dq0.*t2;
t5 = dq0.*t3;
t12 = q0+t9;
t13 = q0+t10;
t16 = q3+q4+t8;
t17 = q5+q6+t8;
t20 = dq5+dq6+t6;
t21 = dq2+dq3+dq4+t6;
t11 = -t4;
t14 = cos(t12);
t15 = cos(t13);
t18 = sin(t12);
t19 = sin(t13);
t22 = cos(t16);
t23 = cos(t17);
t24 = q2+t16;
t25 = sin(t16);
t26 = sin(t17);
t27 = cos(t24);
t28 = sin(t24);
t29 = t14./2.0;
t30 = t15.*(2.0./5.0);
t31 = t18./2.0;
t32 = t19.*(2.0./5.0);
t33 = t22./2.0;
t34 = t23./4.0;
t35 = t25./2.0;
t36 = t26./4.0;
t37 = -t33;
t38 = -t36;
t39 = t27.*(3.0./1.0e+1);
t40 = t28.*(3.0./1.0e+1);
t45 = t30+t34;
t41 = -t39;
t42 = -t40;
t43 = dq2.*t39;
t44 = dq2.*t40;
t46 = t32+t38;
t49 = -dq3.*(t37+t39);
t51 = dq3.*(t37+t39);
t54 = t29+t37+t39;
t47 = t33+t41;
t48 = t35+t42;
t50 = dq3.*t48;
t53 = t31+t48;
t52 = -t50;
dA = reshape([t21.*t39,t43+t51-dq0.*(t37+t39)+dq4.*(t37+t39),t43+t51+dq4.*t54+t6.*t54,0.0,0.0,t5,t4,t7,t21.*t40,t44+t52+dq0.*t48-dq4.*t48,t44+t52+dq0.*t53-dq4.*t53,0.0,0.0,t11,t5,dx0,0.0,0.0,0.0,dq6.*t34+dq5.*t45+t6.*t45,t20.*t34,t5,t4,t7,0.0,0.0,0.0,dq6.*t36+dq0.*t46-dq5.*t46,t20.*t36,t11,t5,dx0],[8,4]);
