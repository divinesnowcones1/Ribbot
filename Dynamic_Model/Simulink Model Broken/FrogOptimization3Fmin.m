%% Optimization
FrogCommands;
Frog_Robot_Parameters;

% global aug_spinePos
% global aug_spine_der
% global aug_traj_times
% global spineOptPosition
% global tsTraj

%Spine Angle, Spine Angular Vel
spineStartAngle = -30;
fLegAngle = 20;
fLegLength = 5;
footAngle = -60;
x0 = -1;
%Spine Torque
% u0 = 0;
vars = [spineStartAngle,fLegAngle,fLegLength,footAngle,x0];
numVars = length(vars);

numTrajPts = numTrajPts;
dt = takeOffPeriod/(numTrajPts-1);
% tsTraj = dt;
tspan = linspace(0,totalTime,dt);

X0 = [spineStartAngle, fLegAngle, fLegLength, footAngle, linspace(spineStartAngle,-.001,numTrajPts)];

% pen = simulateFrog(X0,numTrajPts);

options = optimoptions('fmincon','Display','iter','MaxFunctionEvaluations',1e5);

UB = [0,30,6,-55,0*ones(1,numTrajPts)]; %180*ones(numTrajPts,1) 20*ones(numTrajPts,1)];
LB = [-60,-20,3,-95,-60*ones(1,numTrajPts)]; %-180*ones(numTrajPts,1) -20*ones(numTrajPts,1)];

costFunc = @(p)simulateFrog(p,numTrajPts,numVars);
[sol, cost] = fmincon(costFunc,X0,[],[],[],[],LB,UB,[],options);

%% 
function penalty = simulateFrog(params,numTrajPts,numVars)
%     global spineOptPosition
    
    Frog_Robot_Parameters;
    FrogCommands;  
    
%     sampleTime = dt;
    
%     tsTraj = 0.001;
    dt = takeOffPeriod/(numTrajPts-1);
    
    world_damping = 20;     % Translational damping
    world_rot_damping = 10; % Rotational damping
    
%     waitTimePts = takeOffStartTime/dt;
    % Extract simulation inputs from parameters
    %     footPos  = params(numTrajPts+1);
%     shoulderPos = params(numTrajPts+2)  
    spineStartAngle = params(1);
    fLegAngle = params(2);
    fLegLength = params(3);
    footAngle = params(4);
    spinePos  = params(numVars:numVars+numTrajPts-1);
    
    traj_times = linspace(takeOffStartTime,takeOffPeriod+takeOffStartTime,numTrajPts);
%     traj_times = linspace(takeOffStartTime,takeOffStartTime+takeOffPeriod,numTrajPts);
    
%     aug_spinePos = [startingAngle*ones(1,waitTimePts), spinePos, spinePos(end)*ones(1,waitTimePts)];
%     aug_traj_times = [linspace(0,takeOffStartTime-dt,waitTimePts), traj_times, linspace(takeOffStartTime+takeOffPeriod+dt,takeOffStartTime*2+takeOffPeriod,waitTimePts)]; 

%     totalTrajPts = numel(aug_spinePos);

%     spinePos  = params(1:takeOffTimeSteps);
%     spinePos = [spinePos,spinePos(end)*ones(1,totalTimeSteps - takeOffTimeSteps)];
%     footPos  = params(takeOffTimeSteps+1:takeOffTimeSteps+totalTimeSteps);
%     shoulderPos = params(takeOffTimeSteps+totalTimeSteps+1:takeOffTimeSteps+totalTimeSteps*2)    
%     traj_times = linspace(0,totalTime,totalTimeSteps);
%      
%     [q,spine_der,foot_der,shoulder_der]= createSmoothTrajectory( ... 
%         spinePos,footPos,shoulderPos,totalTime,takeOffPeriod,[0 numTrajPts/2]);

    [q,spine_der]= createSmoothTrajectory( ... 
        spinePos,traj_times,numTrajPts,takeOffPeriod,spineStartAngle,[0 takeOffPeriod/2]);
    aug_spine_der = [zeros(1,waitTimePts), spine_der, zeros(1,ballisticTimePts)];
    aug_spinePos = [spineStartAngle*ones(1,waitTimePts), spinePos, spinePos(end)*ones(1,ballisticTimePts)];
    aug_traj_times = linspace(0,totalTime,totalNumPts);
    
    spineOptPosition = double([aug_traj_times', aug_spinePos']);
%     spinePosition  = double(spineOptPosition);
    ShoulderPosition = [linspace(0,totalTime,totalNumPts)' ones(totalNumPts,1)*fLegAngle];
    LegPosition = [linspace(0,totalTime,totalNumPts)' ...
    [150*ones(waitTimePts,1); linspace(150,40,takeOffPts)'; 40*ones(ballisticTimePts,1)],...
    [-75*ones(waitTimePts,1); linspace(-75,-20,takeOffPts)'; -20*ones(ballisticTimePts,1)],...
    [footAngle*ones(waitTimePts,1); linspace(footAngle,footAngle+55,takeOffPts)'; (footAngle+55)*ones(ballisticTimePts,1)]];
    % Simulate the model
    
    warning off
    DataDump = sim('FrogRobotSim.slx','StopTime',num2str(totalTime),'SrcWorkspace','current','FastRestart','on');
    
    % Unpack logged data
    BodyData = DataDump.BodyData;
    angle = BodyData.q.Data;
    wy = BodyData.wy.Data;
    horzDist = BodyData.x.Data;
    height = BodyData.z.Data;
    horzVel = BodyData.vx.Data;
    vertVel = BodyData.vz.Data;
    spineTorq = BodyData.spineTorq.Data;
%     measBody = get(simout.simout,'measBody').Values;
%     xMax = max(abs(measBody.X.Data));
%     yEnd = measBody.Y.Data(end);
%     tEnd = simout.tout(end);
   
    % Calculate penalty from logged data
    
    %   Longitudinal (Y) distance traveled without falling 
    %   (ending simulation early) increases reward
    velReward = 0;
    
    for i = waitTimePts:waitTimePts+takeOffPts*2
%         positiveReward = sign(horzVel(i))*horzVel(i).^2 * sign(vertVel(i))*vertVel(i).^2;
        velReward = sign(horzVel(i))*sign(vertVel(i))* sqrt(horzVel(i).^2 + vertVel(i).^2) + velReward;
    end
    
    positiveReward = velReward;
    %   Lateral (Y) displacement and trajectory aggressiveness 
    %   (number of times the derivative flips signs) decreases reward
    %   NOTE: Set lower limits to prevent divisions by zero
%     aggressiveness = 0;
%     diffs = [diff(hip_motion) diff(knee_motion) diff(ankle_motion)];
%     for idx = 1:numel(diffs)-1
%         if (sign(diffs(idx)/diffs(idx+1))<0) && mod(idx,N) 
%              aggressiveness = aggressiveness + 1;            
%         end
%     end
%     
    torqCost = 0;
    wyCost = 0;
    
    for i = waitTimePts:waitTimePts+takeOffPts*2
       torqCost = torqCost + spineTorq(i)^2;
       wyCost = wyCost + wy(i)^2;
       
    end
%     wyCost = wy(end);
    negativeReward = torqCost + wyCost;% * angle;
    
    %   Negative sign needed since the optimization minimizes cost function     
    if negativeReward == 0
        penalty = 1;   
    else
        penalty = -positiveReward/negativeReward;   
    end
    %Good
%         Horizontal distance
%         Landing angle
    %Bad
%         Rotational Vel of body
%         High deceleration of spine (impact)?
%         Aggressiveness of joint trajectory
    %Optional Good
%         Retraction of hindleg before landing
%         Low Landing Forces

end


%%

function [q,spine_der] = createSmoothTrajectory(spine,traj_times,numTrajPts,takeOffPeriod,spineStartAngle,evalTimes)
% Generates cubic spline trajectory parameters given waypoints
% for 3 joints (ankle, knee, hip) and total gait period
% Copyright 2017-2019 The MathWorks, Inc.

% Create necessary values for calculations
% numPts = numel(spine);
% numPtsSpine = numel(spine);
% numPtsFoot = numel(foot);
% spineTraj_times = linspace(0,takeOffPeriod,numPtsSpine);
% footTraj_times = linspace(0,totalTime,numPtsFoot);
% traj_times = linspace(0,takeOffPeriod,numPts);

% Calculate derivatives
% dtSpine = takeOffPeriod/(numPtsSpine-1);
% dtFoot = totalTime/(numPtsFoot-1);
% dt = takeOffPeriod/(numPts-1);

% spine_der = [0.5*( diff([startingAngle,spine(1:end-1)]) + diff([spine(2:end) spine(end)]) )/dt, 0];
dt = takeOffPeriod/(numTrajPts - 1);
spine_der = [0.5*( diff([spineStartAngle, spine(1:end-1)]) + diff(spine(1:end)) )/dt, 0];

% foot_der = [0, 0.5*( diff(foot(1:end-1)) + diff(foot(2:end)) )/dt, 0];
% shoulder_der = [0, 0.5*( diff(shoulder(1:end-1)) + diff(shoulder(2:end)) )/dt, 0];

% Set initial conditions
% Evaluate the trajectory at the start and halfway points for right and
% left legs, respectively
% q = cubicpolytraj([spine;foot;shoulder],traj_times,evalTimes,...
%     'VelocityBoundaryCondition',[spine_der;foot_der;shoulder_der]);

q = cubicpolytraj(spine,traj_times,evalTimes,...
    'VelocityBoundaryCondition',spine_der);

end

