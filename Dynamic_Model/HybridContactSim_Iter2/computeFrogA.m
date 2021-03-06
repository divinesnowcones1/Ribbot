function A = computeFrogA(in1)
%COMPUTEFROGA
%    A = COMPUTEFROGA(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    11-Apr-2020 20:18:50

q0 = in1(:,8);
q2 = in1(:,1);
q3 = in1(:,2);
q4 = in1(:,3);
q5 = in1(:,4);
q6 = in1(:,5);
x0 = in1(:,6);
y0 = in1(:,7);
t2 = cos(q0);
t3 = sin(q0);
t4 = -q0;
t5 = -q4;
t6 = -q5;
t7 = -y0;
t8 = -t2;
t9 = -t3;
t10 = q0+t5;
t11 = q0+t6;
t12 = q3+q4+t4;
t13 = q5+q6+t4;
t14 = cos(t12);
t15 = cos(t13);
t16 = q2+t12;
t17 = sin(t12);
t18 = sin(t13);
t19 = cos(t16);
t20 = sin(t16);
t21 = t14./2.0;
t22 = t15./4.0;
t23 = t17./2.0;
t24 = t18./4.0;
t25 = -t22;
t26 = -t23;
t27 = t19.*(3.0./1.0e+1);
t28 = t20.*(3.0./1.0e+1);
t29 = -t27;
A = reshape([t28,t26+t28,t26+t28-sin(t10)./2.0,0.0,0.0,t8,t3,t7,t29,t21+t29,t21+t29-cos(t10)./2.0,0.0,0.0,t9,t8,x0,0.0,0.0,0.0,t24-sin(t11).*(2.0./5.0),t24,t8,t3,t7,0.0,0.0,0.0,t25-cos(t11).*(2.0./5.0),t25,t9,t8,x0],[8,4]);
