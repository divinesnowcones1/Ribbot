%% Reference Commands
totalTime = 2;
timeStep = 0.01;
totalTimeSteps = totalTime/timeStep;
takeOffPeriod = 0.2;    %sec
waitTimeSteps = totalTime*(1-takeOffPeriod/totalTime)/timeStep;
takeOffTimeSteps = totalTimeSteps - waitTimeSteps;

%Leg Position command vectors: Time, Knee Angle, Hip Angle, Foot Angle
LegPosition = [linspace(0,totalTime,totalTimeSteps)' [150*ones(waitTimeSteps,1); linspace(150,40,takeOffTimeSteps)'], [-75*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)'], [-75*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)']];

SpinePosition = [linspace(0,totalTime,totalTimeSteps)' zeros(totalTimeSteps,1)];
ShoulderPosition = [linspace(0,totalTime,totalTimeSteps)' ones(totalTimeSteps,1)*20];