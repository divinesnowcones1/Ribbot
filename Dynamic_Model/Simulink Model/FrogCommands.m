%% Reference Commands
t = 2;
dt = 0.01;
tDiv = t/dt;
JumpPhase = 0.1;    %sec
tInitial = t*(1-JumpPhase/t)/dt;
tJump = tDiv - tInitial;

%Time, Knee Angle, Hip Angle, Foot Angle
LegPosition = [linspace(0,t,tDiv)' [150*ones(tInitial,1); linspace(150,40,tJump)'], [-75*ones(tInitial,1); linspace(-75,-20,tJump)'], [-75*ones(tInitial,1); linspace(-75,-20,tJump)']];
SpinePosition = [linspace(0,t,tDiv)' zeros(tDiv,1)];
ShoulderPosition = [linspace(0,t,tDiv)' ones(tDiv,1)*20];