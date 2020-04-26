%% Leg parameters
%Foot Parameters
footLength = 8;     % cm
footWidth = 3;      % cm
footHeight = 0.5;   % cm
footMass = 0.06;

%Back Leg Parameters (cm)
rUpperLegLength = 0.63;
rUpperLegWidth = 1.91;
rUpperLegHeight = 11.45;
rUpperLegDim = [rUpperLegLength, rUpperLegWidth, rUpperLegHeight];
rUpperLegMass = 0.1;

rLowerLegLength = 0.63;
rLowerLegWidth = 1.91;
rLowerLegHeight = 11.45;
rLowerLegDim = [rLowerLegLength, rLowerLegWidth, rLowerLegHeight];
rLowerLegMass = 0.1;

%Front Leg Parameters (cm)
fLegLength = 0.32;
fLegWidth = 1.27;
fLegHeight = 7.06*2;
fLegDim = [fLegLength, fLegWidth, fLegHeight];
fLegMass = 0.00188;

%% Body Parameters
%Rear Body Parameters
rBodyLength = 12.70;    % cm
rBodyWidth = 9;         % cm
rBodyHeight = 9;        % cm
rBodyMass = 0.19;

%Front Body Parameters
fBodyLength = 12.70;    % cm
fBodyWidth = 9;         % cm
fBodyHeight = 9;        % cm
fBodyMass = 0.12;

%% World Parameters
height_plane = 0.025;
plane_z = height_plane; 
plane_x = 3;
plane_y = 50;
init_height = footHeight + rLowerLegHeight + fBodyHeight/2 + plane_z;

world_damping = 0;      % Translational damping for 6-DOF joint [N/m]
world_rot_damping = 0;  % Rotational damping for 6-DOF joint [N*m/(rad/s)]

%% Motion Parameters
mass = footMass + rUpperLegMass + rLowerLegMass + fLegMass + rBodyMass + fBodyMass; % kg 0.5719
g = 9.81;

motion_time_constant = 0.001;

% Contact and friction parameters
contact_stiffness = mass*g/0.001;          % Approximated at weight (N) / desired displacement (m)
contact_damping = contact_stiffness/10; % Tuned based on contact stiffness value
mu_s = 0.9;     % Static friction coefficient: Around that of rubber-asphalt
mu_k = 0.8;     % Kinetic friction coefficient: Lower than the static coefficient
mu_vth = 0.1;   % Friction velocity threshold (m/s)

contact_point_radius = 0.0001; %m
