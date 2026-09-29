%% R-Theta-Z Wafer Transfer Robot Parameters
% Engineering model inspired by semiconductor wafer-handling robots.
% Not a model of any specific commercial robot.

% R-axis
m_r = 5.0;
b_r = 1.0;

% Theta-axis
J_theta = 0.5;
b_theta = 0.1;

% Z-axis
m_z = 4.0;
b_z = 1.0;

% Gravity
g = 9.81;

% Actuator limits
F_r_max = 100;        % R force:      -F_r_max .. F_r_max (N)
T_theta_max = 20;     % Theta torque: -T_theta_max .. T_theta_max (N*m)
F_z_max = 100;        % Z force:       F_z_min .. F_z_max (N)
F_z_min = 0;          % Z motor can only push up

% Initial conditions
r_initial = 0.1;
theta_initial = 0;
z_initial = 0.1;

% Simulation
simulation_time = 2.0;
move_time = simulation_time;                 % quintic move duration (s)
num_points = 1000;                           % matches Python simulations
Ts = simulation_time / (num_points - 1);     % controller sample time (s)

% PID gains (same as Python)
Kp_r = 300;     Ki_r = 20;     Kd_r = 30;
Kp_theta = 100; Ki_theta = 5;  Kd_theta = 10;
Kp_z = 300;     Ki_z = 20;     Kd_z = 30;

% Controller switches (1 = on, 0 = off)
ff_enable = 1;        % trajectory feedforward (inertia*acc + damping*vel)
gravity_comp = 1;     % gravity compensation on the Z axis

% Disturbances: step force/torque on the plant at disturbance_time (0 = off)
disturbance_time = 2.5;   % s
d_r = 0;                  % N
d_theta = 0;              % N*m
d_z = 0;                  % N (negative = extra downward load)

% Sensor noise: standard deviation of position measurement (0 = off)
noise_r = 0;              % m
noise_theta = 0;          % rad
noise_z = 0;              % m

% Known-disturbance feedforward, applied at disturbance_time (0 = off)
% e.g. dff_z = m_wafer*g compensates the weight of a picked-up wafer
dff_r = 0;                % N
dff_theta = 0;            % N*m
dff_z = 0;                % N

% Model mismatch: true plant inertia/mass = nominal * plant_mass_scale
% (controller feedforward and gravity compensation keep the nominal values)
plant_mass_scale = 1;

% PID derivative filter coefficient (rad/s), used when the filter is on.
% The filter is off by default to match the Python PID exactly.
N_filter = 100;

% Motion targets
r_final = 0.4;
theta_final = pi / 2;
z_final = 0.25;