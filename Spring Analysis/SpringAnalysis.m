% Constants
m = 0.5;
g = 9.81;
lb2N = 0.112985;
in2m = 0.0254;
%Requirements
launchTraj = deg2rad(45);
HorzDist = 0.1524;

%Launch Velocity
vf = sqrt((HorzDist*g)/sin(2*launchTraj));

%Jump force requirements
t = linspace(0.05,0.5,100);     %Jump phase time
accel = vf./t;  
F = accel*m;                    %Desired forces
Fy = F*sin(launchTraj) + m*g;
Fx = F*cos(launchTraj);
Freq = sqrt(Fy.^2 + Fx.^2);
ThrustAngles = atan2(Fy,Fx);

%Range of link lengths
linkLength = linspace(0.03,0.20,340); %cm


%[1] rest length | [2] extended length | [3] spring const
SpringMatrix = [3*in2m, 5.94*in2m, 2.2*lb2N;... %9044K193
    3*in2m, 7*in2m, 0.46*lb2N;...     %9654K714
    3*in2m, 6.84*in2m, 0.82*lb2N;...  %9654K196
    3.5*in2m, 7.19*in2m, 3.2*lb2N;... %9044K126
    3.5*in2m, 8.67*in2m, 1*lb2N;...   %9044K108
    3.5*in2m, 9.64*in2m, 1.1*lb2N;... %9044K136
    3.5*in2m, 6.39*in2m, 2.21*lb2N;...%9654K375   
    4*in2m, 8.1*in2m, 2.93*lb2N;...   %9654K164
    4*in2m, 7.41*in2m, 1.87*lb2N;...  %9654K94
    4*in2m, 11.39*in2m, 0.9*lb2N;...  %9044K124
    4*in2m, 8.38*in2m, 2.7*lb2N;...   %9044K132
    4.5*in2m, 9.56*in2m, 2.3*lb2N;... %9044K134
    4.5*in2m, 8.06*in2m, 4.8*lb2N;... %9044K383
    4.5*in2m, 9.1*in2m, 1.22*lb2N;... %9654K923
    4.5*in2m, 8.34*in2m, 0.75*lb2N;...%9654K256
    5*in2m, 7.91*in2m, 0.82*lb2N;...  %9654K84
    5*in2m, 11.022*in2m, 2*lb2N;...   %9654K325
    5*in2m, 11.2*in2m, 7.5*lb2N;...   %9654K738
    5.5*in2m, 8.5*in2m, 2.73*lb2N;... %9654K533
    5.5*in2m, 7.5*in2m, 3.09*lb2N;... %9654K471
    6*in2m, 10.65*in2m, 0.88*lb2N;... %9654K93
    6*in2m, 10.37*in2m, 1.26*lb2N;... %9654K184
    6.5*in2m, 7.5*in2m, 3.14*lb2N;... %9654K385
    7*in2m, 12.08*in2m, 3.05*lb2N;... %9654K925
    7.5*in2m, 15.2*in2m, 1.42*lb2N;...%9654K253
    8*in2m, 13.68*in2m, 1.26*lb2N;... %9654K145
    8*in2m, 12.95*in2m, 2.36*lb2N;... %9654K449
    8*in2m, 12.9*in2m, 3.07*lb2N];...  %9654K444
%     5*in2m, 7.2*in2m, 30*lb2N];

%Spring constant
k = SpringMatrix(:,3);

%Each spring displacment range
xi = SpringMatrix(:,1);
xf = SpringMatrix(:,2);
x = zeros(size(SpringMatrix,1),100);
for m = 1:size(SpringMatrix,1)
    x(m,:) = linspace(xi(m),xf(m),100);
end

%Variable declarations
inc = 1;
y = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
Fy = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
torque = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));
angle = zeros(size(linkLength,2),size(x,2),size(SpringMatrix,1));

%Spring Calculations
for i = 1:size(SpringMatrix,1)
    for j = 1:size(linkLength,2)
        for l = 1:size(x,2)
            if linkLength(j)*2 > xf(i)
                y(j,l,i) = sqrt(linkLength(j)^2 - (x(i,l)/2)^2) + sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
                Fy(j,l,i) = k(i)*y(j,l,i)*(1 - xi(i)/sqrt(4*linkLength(j)^2 - y(j,l,i)^2));
                torque(j,l,i) = (x(i,l) - xi(i))*k(i)*sqrt(linkLength(j)^2 - (x(i,l)/2)^2);
                angle(j,l,i) = acos((x(i,l)/2)/linkLength(j));
                springs(inc) = i;
                links(inc) = j;
                inc = inc + 1;
            end
        end
    end
end

%Average spring force
Fyavg = mean(Fy,2);

%Sorting out the best spring
inc = 1;
Fybest = 0;
for i = 1:size(Fyavg,3)
    for j = 1:size(linkLength,2)
        temp = Fyavg(j,1,i);
        if Fyavg(j,1,i) > min(Freq)
            row(inc) = j;
            col(inc) = i;
            inc = inc  +  1;
        end
        if temp > Fybest
            Fybest = Fyavg(j,1,i);
            link = j;
            spring = i;
        end
    end
end

%Best spring 
disp(['Spring Number: ',num2str(spring),'']);
SpringMatrix(spring,:)
linkLength(link)
Fybest
figure(2);
plot(rad2deg(angle(link,:,spring)),torque(link,:,spring));
title('Spring/Link angle vs torque');
figure(3);
plot(x(spring,:),Fy(link,:,spring));
title('Distance vs Fy');