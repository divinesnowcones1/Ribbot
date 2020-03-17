%%Spine spring analysis of the front body, assuming rear body is static
%% Prepare workspace
close;
clear;
clc;

%% Parameters
% Unit conversions
lbf2N = 0.11298482933333;   % in-lbs to Nm

% Robot parameters
frontMass = 0.2;            % kg of front body 
m = 0.7;                    % kg of robot
g = 9.81;                   % m/s^2
launchTraj = deg2rad(45);   % Desired launch trajectory
HorzDist = 1*0.3048;        % Desired horizontal jump range (ft * m)
launchVel = sqrt((HorzDist*g)/sin(2*launchTraj));   % Launch Velocity (also known as initial velocity)
t = linspace(0.05,0.3,100);           % takeoff phase time step
accel = launchVel./t;                 % input acceleration
spineStartingAngle = deg2rad(45);     % starting spine angle
d = 0.0508;                           % 2 inches - Distance between the Fg_m2 and torsion spring

%% Benchmark Requirements
% Front body force requirements
F_m2 = frontMass*g;
req_sweep_angle = deg2rad(90) - spineStartingAngle;    % required sweep angle from spine launch angle to straight spine

% F_in = accel*frontMass;                       % Input force
% F_dy = F_in*sin(launchTraj) + frontMass*g;    % Desired Fy
% F_dx = F_in*cos(launchTraj);                  % Desired Fx
% F_req = sqrt(F_dy.^2 + F_dx.^2);              % Required forces
% req_sweep_angle = deg2rad(180) - spineStartingAngle;    % required sweep angle from spine launch angle to straight spine

%% List of available right hand torsion springs from McMaster
% [1] max torque
springMatrix = ...
[
    0.05   *   lbf2N;...   % 9271K69    #1
    0.1    *   lbf2N;...   % 9271K78    #2
    0.075  *   lbf2N;...   % 9271K74    #3
    0.1    *   lbf2N;...   % 9271K83    #4
    0.25   *   lbf2N;...   % 9271K665   #5
    0.075  *   lbf2N;...   % 9271K157   #6
    0.15   *   lbf2N;...   % 9271K87    #7
    0.402  *   lbf2N;...   % 9271K93    #8
    0.552  *   lbf2N;...   % 9271K97    #9
    0.13   *   lbf2N;...   % 9271K667   #10
    1.07   *   lbf2N;...   % 9271K668   #11
    0.42   *   lbf2N;...   % 9271K66    #12
    0.55   *   lbf2N;...   % 9271K671   #13
    1.28   *   lbf2N;...   % 9271K672   #14
    0.88   *   lbf2N;...   % 9271K673   #15
    2.679  *   lbf2N;...   % 9271K181   #16
    3.1    *   lbf2N;...   % 9271K674   #17
    1.071  *   lbf2N;...   % 9271K141   #18
    1.5    *   lbf2N;...   % 9271K676   #19
    4.5    *   lbf2N;...   % 9271K231   #20
    3.1    *   lbf2N;...   % 9271K677   #21
    2.15   *   lbf2N;...   % 9271K678   #22
    2.75   *   lbf2N;...   % 9271K681	#23
    7.5    *   lbf2N;...   % 9271K679   #24
    3.5    *   lbf2N;...   % 9271K683   #25
    9.2    *   lbf2N;...   % 9271K682	#26
    4.5    *   lbf2N;...   % 9271K684	#27
    9.2    *   lbf2N;...   % 9271K685	#28
    5.518  *   lbf2N;...   % 9271K271   #29
    10.45  *   lbf2N;...   % 9271K686   #30
    10.446 *   lbf2N;...   % 9271K321   #31
    7.5    *   lbf2N;...   % 9271K687   #32
    12.86  *   lbf2N;...   % 9271K688	#33
    17.14  *   lbf2N;...   % 9271K689	#34
    21     *   lbf2N;...   % 9271K691   #35
    22.5   *   lbf2N;...   % 9271K119   #36
    17.14  *   lbf2N;...   % 9271K692   #37
    34.28  *   lbf2N;...   % 9271K122   #38
    42.86  *   lbf2N;...   % 9271K126   #39
    28     *   lbf2N;...   %  9271K693  #40
    34.29  *   lbf2N;...   %  9271K124	#41
];

%% Spring Calculations
w = req_sweep_angle./t;  % angular velocty [radians/s]
w_avg = mean(w);
alpha = w./t;        % angular acceleration [radians/s^2]
alpha_avg = mean(alpha);
springTorque = F_m2*cos(req_sweep_angle)*d + frontMass*d^2*alpha_avg;
springTorque_avg = mean(springTorque)
