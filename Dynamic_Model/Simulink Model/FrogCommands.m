%% Reference Commands
totalTime = 1;
timeStep = 0.01;
totalTimeSteps = totalTime/timeStep;

landPeriod = 0.2;
takeOffPeriod = 0.2;    %sec

waitTimeSteps = totalTime*(1-(landPeriod + takeOffPeriod)/totalTime)/timeStep;
takeOffTimeSteps = 0.5*(totalTimeSteps - waitTimeSteps);
landOffTimeSteps = 0.5*(totalTimeSteps - waitTimeSteps);

size(linspace(0,totalTime,totalTimeSteps)')
size(150*ones(waitTimeSteps,1))
size(linspace(150,40,takeOffTimeSteps)')
size (linspace(150,40,landOffTimeSteps)')

%Leg Position command vectors: Time, Knee Angle, Hip Angle, Foot Angle
LegPosition = [linspace(0,totalTime,totalTimeSteps)',...
    [150*ones(waitTimeSteps,1); linspace(150,40,takeOffTimeSteps)'; 150*ones(landOffTimeSteps,1)],...
    [-68*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)'; -68*ones(landOffTimeSteps,1)],...
    [-68*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)'; -68*ones(landOffTimeSteps,1)]];
SpinePosition = [linspace(0,totalTime,totalTimeSteps)', zeros(totalTimeSteps,1)];
ShoulderPosition = [linspace(0,totalTime,totalTimeSteps)', ones(totalTimeSteps,1)*20];
