function a = computeFrog_a(in1)
%COMPUTEFROG_A
%    A = COMPUTEFROG_A(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    14-Apr-2020 10:07:11

q0 = in1(:,8);
q2 = in1(:,1);
q3 = in1(:,2);
q4 = in1(:,3);
q5 = in1(:,4);
q6 = in1(:,5);
y0 = in1(:,7);
t2 = cos(q0);
t3 = cos(q2);
t4 = cos(q3);
t5 = cos(q4);
t6 = cos(q5);
t7 = cos(q6);
t8 = sin(q0);
t9 = sin(q3);
t10 = sin(q4);
t11 = sin(q5);
t12 = t2.*t5;
t13 = t2.*t10;
t14 = t5.*t8;
t15 = t2.*t11;
t16 = t6.*t8;
t17 = t8.*t10;
t19 = t8.*(3.0./1.0e+1);
t18 = -t17;
t20 = t13+t14;
t21 = t15+t16;
t22 = t12+t18;
t23 = t4.*t20;
t24 = t9.*t22;
t25 = t23+t24;
a = [t13./4.0+t14./4.0+t19-t24./4.0+y0-t20.*(t4./4.0-1.0./4.0)+t25.*(t3.*(3.0./2.0e+1)-1.0./4.0)+sin(q2).*(t4.*t22-t9.*t20).*(3.0./1.0e+1)+t3.*t25.*(3.0./2.0e+1);t15./5.0+t16./5.0-t19+y0+t21.*(t7./8.0+1.0./5.0)+(sin(q6).*(t2.*t6-t8.*t11))./4.0+(t7.*t21)./8.0];
