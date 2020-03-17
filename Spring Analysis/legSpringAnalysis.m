%% Prepare workspace
close;
clear;
clc;

%% Parameters
% Unit conversions
lbf2N = 175.1268;   % lbf/in to N/m
in2m = 0.0254;      % in to m

% Robot Parameters
m = 0.7;                    % mass kg %0.7 and 1
g = 9.81;                   % gravity m/s^2
launchTraj = deg2rad(45);   % Desired launch trajectory
HorzDist = 1*0.3048;        % Desired horizontal jump range (ft * m)
launchVel = sqrt((HorzDist*g)/sin(2*launchTraj));   % Launch Velocity (also known as initial velocity)
t = linspace(0.05,0.3,100);             % Jump phase time step
accel = launchVel./t;                   % input acceleration
linkLength = linspace(0.03,0.20,270*2); % %Range of link lengths

%% Benchmark Requiremets
F_in = accel*m;                       % Input force
F_dy = F_in*sin(launchTraj) + m*g;    % Desired Fy
F_dx = F_in*cos(launchTraj);          % Desired Fx
F_req = sqrt(F_dy.^2 + F_dx.^2);      % Required forces
% legThrustAngles = atan2(F_dy,F_dx);   % Leg thrust angles  

%% List of available extension springs from McMaster
%[1] rest length | [2] extended length | [3] spring constant
SpringMatrix = ...
    [3*in2m, 5.94*in2m, 2.2*lbf2N;...  %9044K193    #1
    3*in2m, 7*in2m, 0.46*lbf2N;...     %9654K714    #2
    3*in2m, 6.84*in2m, 0.82*lbf2N;...  %9654K196    #3
    3.5*in2m, 7.19*in2m, 3.2*lbf2N;... %9044K126    #4
    3.5*in2m, 8.67*in2m, 1*lbf2N;...   %9044K108    #5
    3.5*in2m, 9.64*in2m, 1.1*lbf2N;... %9044K136    #6
    3.5*in2m, 6.39*in2m, 2.21*lbf2N;...%9654K375    #7
    4*in2m, 8.1*in2m, 2.93*lbf2N;...   %9654K164    #8
    4*in2m, 7.41*in2m, 1.87*lbf2N;...  %9654K94     #9
    4*in2m, 11.39*in2m, 0.9*lbf2N;...  %9044K124    #10
    4*in2m, 8.38*in2m, 2.7*lbf2N;...   %9044K132    #11
    4.5*in2m, 9.56*in2m, 2.3*lbf2N;... %9044K134    #12
    4.5*in2m, 8.06*in2m, 4.8*lbf2N;... %9044K383    #13
    4.5*in2m, 9.1*in2m, 1.22*lbf2N;... %9654K923    #14
    4.5*in2m, 8.34*in2m, 0.75*lbf2N;...%9654K256    #15
    5*in2m, 7.91*in2m, 0.82*lbf2N;...  %9654K84     #16
    5*in2m, 11.022*in2m, 2*lbf2N;...   %9654K325    #17
    5*in2m, 11.2*in2m, 7.5*lbf2N;...   %9654K738    #18
    5.5*in2m, 8.5*in2m, 2.73*lbf2N;... %9654K533    #19
    5.5*in2m, 7.5*in2m, 3.09*lbf2N;... %9654K471    #20
    6*in2m, 10.65*in2m, 0.88*lbf2N;... %9654K93     #21
    6*in2m, 10.37*in2m, 1.26*lbf2N;... %9654K184    #22
    6.5*in2m, 7.5*in2m, 3.14*lbf2N;... %9654K389    #23
    7*in2m, 12.08*in2m, 3.05*lbf2N;... %9654K925    #24
    7.5*in2m, 15.2*in2m, 1.42*lbf2N;...%9654K253    #25
    8*in2m, 13.68*in2m, 1.26*lbf2N;... %9654K145    #26
    8*in2m, 12.95*in2m, 2.36*lbf2N;... %9654K449    #27
    8*in2m, 12.9*in2m, 3.07*lbf2N];...  %9654K444   #28
%    5*in2m, 7.2*in2m, 30*lb2N];


%% List of available compression springs from McMaster
%[1] rest length | [2] extended length | [3] spring constant
%% Spring Calculations
% Spring constant
k = SpringMatrix(:,3);

% Compute spring displacment range
x_i = SpringMatrix(:,1);
x_f = SpringMatrix(:,2);
x = zeros(size(SpringMatrix,1),100);
for c = 1:size(SpringMatrix,1)
    x(c,:) = linspace(x_i(c),x_f(c),100);   % dividing the initial to final length 100 evenly spaced points
end

% Variable declarations
inc = 1;
y = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
F_y = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
torque = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
angle = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));

% Compute spring parameters
for i = 1:size(SpringMatrix,1)
    for j = 1:size(linkLength,2)
        for l = 1:size(x,2)
            if linkLength(j)*2 > x_f(i)
                y(j,l,i) = sqrt(linkLength(j)^2 - (x(i,l)/2)^2) + sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
                F_y(j,l,i) = k(i)*y(j,l,i)*(1 - x_i(i)/sqrt(4*linkLength(j)^2 - y(j,l,i)^2));
                torque(j,l,i) = (x(i,l) - x_i(i))*k(i)*sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
                angle(j,l,i) = acos((x(i,l)/2)/linkLength(j));
                springs(inc) = i;
                links(inc) = j;
                inc = inc + 1;
            end
        end
    end
end

% Import spring/link parameters to optimize
Fy_avg = mean(F_y,2);

% Peak force
Fy_max = max(F_y, 2);
 
% TorquePeak = max(torque,[],2);
AngleMin = rad2deg(min(angle,[],2));
MinLinkLength = 0.08;


% Sorting out the best spring
inc = 1;
tempAngle = 0;
tempFyavg = 0;
minInitialForce = m*g*1;
maxInitialForce = m*g*1.1;
for i = 1:size(Fy_avg,3)
    for j = 1:size(linkLength,2)
        tempAngle = AngleMin(j,1,i);
        tempFyavg = Fy_avg(j,1,i);
        if Fy_avg(j,1,i) > mean(F_req) && min(F_y(j,length(x),i)) <...
                maxInitialForce && min(F_y(j,length(x),i)) >...
                minInitialForce && linkLength(1,j) > ...
                MinLinkLength && AngleMin(j,1,i) < 10%Selection Criteria
            link(inc) = j;
            spring(inc) = i;
            inc = inc  +  1;
        end
    end
end

%% Display Springs
% for i = 1:length(spring)

% Best spring
    num = 1;
    disp(['Spring Number: #',num2str(spring(num)),' ']);
    SpringMatrix(spring(num),:);
    disp(['Link Length: ',num2str(linkLength(link(num))),' m']);
    figure(num);
    plot(x(spring(num),:)-x_i(spring(num)),F_y(link(num),:,spring(num)))
    hold on
    line([x_i(spring(num))-x_i(spring(num)), x_f(spring(num))-x_i(spring(num))],[mean(F_req), mean(F_req)],'Color','red','LineStyle','--');
    line([x_i(spring(num))-x_i(spring(num)), x_f(spring(num))-x_i(spring(num))],[Fy_avg(link(num),1,spring(num)), Fy_avg(link(num),1,spring(num))],'Color','green','LineStyle','--');
    hold off
    xlabel('Displacement (m)');
    ylabel('Fy (N)');
    title('Vertical Force to X Position');

    figure(num+1);
    plot(rad2deg(angle(link(num),:,spring(num))),F_y(link(num),:,spring(num)));
    xlabel('Theta');
    ylabel('Fy (N)');
    title('Vertical Force to Theta');

% end

