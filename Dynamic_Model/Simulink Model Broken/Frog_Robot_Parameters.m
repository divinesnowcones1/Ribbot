%% Robot Parameters
mass = 1;
g = 9.81;

%Foot Parameters
footLength = 3;     %cm
footWidth = 5;      %cm
footHeight = 6;     %cm
footMass = 50;      %g

%Leg Parameters
bLegRadius = 0.5;   %cm
bLegLength = 11.45; %cm
bLegMass = 100;     %g
fLegRadius = 0.5;   %cm
fLegLength = 5;     %cm
fLegMass = 50;      %g

%Rear Body Parameters
rBodyLength = 9; %cm
rBodyWidth = 9;     %cm
rBodyHeight = 9;    %cm
rBodyMass = 300;    %g

%Front Body Parameters
fBodyLength = 9; %cm
fBodyWidth = 9;     %cm
fBodyHeight = 9;    %cm
fBodyMass = 150;    %g

%Spine Parameters
hingeWidth = fBodyWidth;        %cm
hingeLength = fBodyLength/2.5;  %cm
hingeThickness = 0.25;          %cm
hingeMass = 150;                %g

%World Parameters
height_plane = 0.025;
plane_z = height_plane; 
plane_x = 3;
plane_y = 50;
init_height = footHeight + bLegLength + fBodyHeight/2 + plane_z;

world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

%Motion Parameters
motion_time_constant = 0.001;

% Contact and friction parameters
contactSphereRadius = 0.25;
contact_stiffness = mass*g/0.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)

contact_point_radius = 0.0001; %m
