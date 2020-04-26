
# Tutorials
Simulink Walking Robot Example Download
https://www.mathworks.com/matlabcentral/fileexchange/64227-matlab-and-simulink-robotics-arena-walking-robot

Walk through of Walking Robot Example	
https://www.youtube.com/watch?v=4rfxWYpE7MM
https://www.mathworks.com/videos/modeling-and-simulation-of-walking-robots-1576560207573.html

Contact Modeling with Simulink - more detail on how it work if needed
https://www.mathworks.com/videos/matlab-and-simulink-robotics-arena-introduction-to-contact-modeling-part-1-1502715532761.html

# Tasks
<del> 1) Obtain accurate dimensions and weights of robot components <del>

<del>2) Model the leg as spring torque vs time<del>

3) Manual iteration of spine starting angle, torque, and start time
	Torque/Start Time: input vector
	Spine Starting Angle: Spine subsystem, Spine Joint, State Targets, Specify Position Target, Value

4) Optimization
	1) Dynamic equations of robot
	2) Collocation of dynamic equations
	3) Objective functions and implementation with fmincon and simulation

5) Reference tracker for spine control

# Possible Plots
starting time vs distance
sweep speed vs distance (2 free parameters)
angle vs distance

angle vs distance with fixed starting time and variable speed
sweep speed vs distance with either fixed starting or ending time and variable angle
angle vs distance with optimized parameters (sweep speed, starting time)

# New Variable Names
%% Parameters
totalTime = 2;
timeStep = 0.01;
totalTimeSteps = totalTime/timeStep;
takeOffPeriod = 0.2;    %sec
waitTimeSteps = totalTime*(1-takeOffPeriod/totalTime)/timeStep;
takeOffTimeSteps = totalTimeSteps - waitTimeSteps;

%% Reference Commands
% Time, Hip Angle, Foot Angle
LegPosition = [linspace(0,totalTime,totalTimeSteps)', [-75*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)'], [-75*ones(waitTimeSteps,1); linspace(-75,-20,takeOffTimeSteps)']];
SpinePosition = [linspace(0,totalTime,totalTimeSteps)', zeros(totalTimeSteps,1)];
ShoulderPosition = [linspace(0,totalTime,totalTimeSteps)', ones(totalTimeSteps,1)*20];

