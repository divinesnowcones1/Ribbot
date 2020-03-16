%%Spine spring analysis of the front body, assuming rear body is static
%% Prepare workspace
close;
clear;
clc;

%% Parameters
% Assuming the body is 0.75 kg of overall robot mass, front body
frontMass = 0.2;          % mass kg of front body %0.7 and 1
g = 9.81;         % gravity m/s^2

% Unit conversions
lbf2N = 0.11298482933333;   % in-lbs to Nm
in2m = 0.0254;      % in to m

%% Benchmark Requirements
% Overall robot requirements
launchTraj = deg2rad(45);   % Desired launch trajectory
HorzDist = 1*0.3048;        % Desired horizontal jump range (ft * m)
launchVel = sqrt((HorzDist*g)/sin(2*launchTraj));   % Launch Velocity (also known as initial velocity)
t = linspace(0.05,0.3,100);           % Jump phase time step
accel = launchVel./t;                 % input acceleration

% Front body force requirements
F_in = accel*frontMass;               % Input force
F_dy = F_in*sin(launchTraj) + frontMass*g;    % Desired Fy
F_dx = F_in*cos(launchTraj);          % Desired Fx
F_req = sqrt(F_dy.^2 + F_dx.^2);      % Required forces

% %Range of body lengths
% bodyLength = linspace(0.03,0.20,270*2); % meters

%% List of available right hand torsion springs from McMaster
%[1] leg length | [2] spring length @ max torque | [3] max torque
SpringMatrix = ...
    [ 
    0.375 * in2m,   0.086  *   in2m, 0.05   *   lbf2N;...   % 9271K69    #1
    0.5   * in2m,   0.107  *   in2m, 0.1    *   lbf2N;...   % 9271K78    #2
    0.5   * in2m,   0.15   *   in2m, 0.075  *   lbf2N;...   % 9271K74    #3
    0.75  * in2m,   0.075  *   in2m, 0.1    *   lbf2N;...   % 9271K83    #4
    0.75  * in2m,   0.173  *   in2m, 0.25   *   lbf2N;...   % 9271K665   #5
    0.75  * in2m,   0.087  *   in2m, 0.075  *   lbf2N;...   % 9271K157   #6
    0.75  * in2m,   0.109  *   in2m, 0.15   *   lbf2N;...   % 9271K87    #7
    0.75  * in2m,   0.206  *   in2m, 0.402  *   lbf2N;...   % 9271K93    #8
    1     * in2m,   0.238  *   in2m, 0.552  *   lbf2N;...   % 9271K97    #9
    0.375 * in2m,   0.09   *   in2m, 0.13   *   lbf2N;...   % 9271K667   #10
    1.25  * in2m,   0.29   *   in2m, 1.07   *   lbf2N;...   % 9271K668   #11
    1     * in2m,   0.151  *   in2m, 0.42   *   lbf2N;...   % 9271K66    #12
    1     * in2m,   0.175  *   in2m, 0.55   *   lbf2N;...   % 9271K671   #13
    1.25  * in2m,   0.315  *   in2m, 1.28   *   lbf2N;...   % 9271K672   #14
    1     * in2m,   0.2    *   in2m, 0.88   *   lbf2N;...   % 9271K673   #15
    1.25  * in2m,   0.45   *   in2m, 2.679  *   lbf2N;...   % 9271K181   #16
    2     * in2m,   0.47   *   in2m, 3.1    *   lbf2N;...   % 9271K674   #17
    1.25  * in2m,   0.212  *   in2m, 1.071  *   lbf2N;...   % 9271K141   #18
    2     * in2m,   0.242  *   in2m, 1.5    *   lbf2N;...   % 9271K676   #19
    2     * in2m,   0.56   *   in2m, 4.5    *   lbf2N;...   % 9271K231   #20
    2     * in2m,   0.365  *   in2m, 3.1    *   lbf2N;...   % 9271K677   #21
    2     * in2m,   0.28   *   in2m, 2.15   *   lbf2N;...   % 9271K678   #22
    2     * in2m,   0.292  *   in2m, 2.75   *   lbf2N;...   % 9271K681	 #23
    2     * in2m,   0.64   *   in2m, 7.5    *   lbf2N;...   % 9271K679   #24
    2     * in2m,   0.39   *   in2m, 3.5    *   lbf2N;...   % 9271K683   #25
    2     * in2m,   0.689  *   in2m, 9.2    *   lbf2N;...   % 9271K682	 #26
    2     * in2m,   0.425  *   in2m, 4.5    *   lbf2N;...   % 9271K684	 #27
    2     * in2m,   0.612  *   in2m, 9.2    *   lbf2N;...   % 9271K685	 #28
    2     * in2m,   0.475  *   in2m, 5.518  *   lbf2N;...   % 9271K271   #29
    2     * in2m,   0.637  *   in2m, 10.45  *   lbf2N;...   % 9271K686   #30
    2     * in2m,   0.66   *   in2m, 10.446 *   lbf2N;...   % 9271K321   #31
    2     * in2m,   0.5    *   in2m, 7.5    *   lbf2N;...   % 9271K687   #32
    2.5   * in2m,   0.694  *   in2m, 12.86  *   lbf2N;...   % 9271K688	 #33
    3     * in2m,   0.872  *   in2m, 17.14  *   lbf2N;...   % 9271K689	 #34
    3.5   * in2m,   1.05   *   in2m, 21     *   lbf2N;...   % 9271K691   #35
    3.5   * in2m,   0.998  *   in2m, 22.5   *   lbf2N;...   % 9271K119   #36
    3     * in2m,   0.775  *   in2m, 17.14  *   lbf2N;...   % 9271K692   #37
    4     * in2m,   1.438  *   in2m, 34.28  *   lbf2N;...   % 9271K122   #38
    4     * in2m,   1.553  *   in2m, 42.86  *   lbf2N;...   % 9271K126   #39
    4     * in2m,   0.92   *   in2m, 28     *   lbf2N;...   %  9271K693  #40
    4     * in2m,   1.188  *   in2m, 34.29  *   lbf2N;...   %  9271K124	 #41
    ];
    %    5* in2m, 7.2*in2m, 30*lb2N];

%% Spring Calculations
% Spring torque
springTorque = SpringMatrix(:,3);

% Compute spring displacment range
x_i = SpringMatrix(:,1);    % spring leg length at rest
x_f = SpringMatrix(:,2);    % spring leg length @ max torque
x = zeros(size(SpringMatrix,1),100);
for c = 1:size(SpringMatrix,1)
    x(c,:) = linspace(x_i(c),x_f(c),100);   % dividing the initial to final length 100 evenly spaced points
end

% Variable declarations
inc = 1;
% springForce = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
% F_y = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
% springTorque = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
% angle = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));

springForce = zeros(size(x,2), size(SpringMatrix,1));


% Compute spring parameters
for i = 1:size(SpringMatrix,1)
    for l = 1:size(x,2)
        springForce(i,l) = springTorque(i)*x_i(i);
        springs(inc) = i;
%         angle(j,l,i) = acos((x(i,l)/2)/bodyLength(j));
        inc = inc + 1;
    end
end

% % Compute spring parameters
% for i = 1:size(SpringMatrix,1)
%     for j = 1:size(linkLength,2)
%         for l = 1:size(x,2)
%             if linkLength(j)*2 > x_f(i)
%                 springForce(j,l,i) = springTorque(i)*x_i(i);
% %                 y(j,l,i) = sqrt(linkLength(j)^2 - (x(i,l)/2)^2) + sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
% %                 F_y(j,l,i) = k(i)*y(j,l,i)*(1 - x_i(i)/sqrt(4*linkLength(j)^2 - y(j,l,i)^2));
% %                 torque(j,l,i) = (x(i,l) - x_i(i))*k(i)*sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
% %                 angle(j,l,i) = acos((x(i,l)/2)/linkLength(j));
%                 springs(inc) = i;
%                 links(inc) = j;
%                 inc = inc + 1;
%             end
%         end
%     end
% end

% Import spring/link parameters to optimize
springForce_avg = mean(springForce,2);

% Peak
springForce_max = max(springForce, 2);  % peak force
springTorque_max = max(springTorque,2);     % peak torque
% AngleMin = rad2deg(min(angle,[],2));

% Sorting out the best spring
inc = 1;
tempAngle = 0;
tempspringForce_avg = 0;
minInitialForce = frontMass*g*1;
maxInitialForce = frontMass*g*1.1;
for i = 1:size(springForce_avg, 3)
%     tempAngle = AngleMin(i);
    tempspringForce_avg = springForce_avg(i);
    if springForce_avg(l,i) > mean(F_req) && min(springForce(length(x),i))...
            < maxInitialForce && min(springForce(length(x),i)) > minInitialForce 
        spring(inc) = i;
        inc = inc  +  1;
    end
end

% % Sorting out the best spring
% inc = 1;
% tempAngle = 0;
% tempFyavg = 0;
% minInitialForce = frontMass*g*1;
% maxInitialForce = frontMass*g*1.1;
% for i = 1:size(springForce_avg,3)
%     for j = 1:size(linkLength,2)
%         tempAngle = AngleMin(j,1,i);
%         tempFyavg = springForce_avg(j,1,i);
%         if springForce_avg(j,1,i) > mean(F_req) && min(springForce(j,length(x),i)) <...
%                 maxInitialForce && min(springForce(j,length(x),i)) >...
%                 minInitialForce && linkLength(1,j) > ...
%                 MinLinkLength && AngleMin(j,1,i) < 10%Selection Criteria
%             link(inc) = j;
%             spring(inc) = i;
%             inc = inc  +  1;
%         end
%     end
% end

%% Display Springs
% for i = 1:length(spring)

% Best spring
    num = 1;
    disp(['Spring Number: #',num2str(spring(num)),' ']);
    SpringMatrix(spring(num),:);
    disp(['Link Length: ',num2str(linkLength(link(num))),' m']);
    figure(num);
    plot(x(spring(num),:)-x_i(spring(num)),springForce(link(num),:,spring(num)))
    hold on
    line([x_i(spring(num))-x_i(spring(num)), x_f(spring(num))-x_i(spring(num))],[mean(F_req), mean(F_req)],'Color','red','LineStyle','--');
    line([x_i(spring(num))-x_i(spring(num)), x_f(spring(num))-x_i(spring(num))],[springForce_avg(link(num),1,spring(num)), springForce_avg(link(num),1,spring(num))],'Color','green','LineStyle','--');
    hold off
    xlabel('Displacement (m)');
    ylabel('Fy (N)');
    title('Vertical Force to X Position');

    figure(num+1);
    plot(rad2deg(angle(link(num),:,spring(num))),springForce(link(num),:,spring(num)));
    xlabel('Theta');
    ylabel('Fy (N)');
    title('Vertical Force to Theta');

% end


