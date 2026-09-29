%% Build the R-axis PID closed-loop model
%
% Creates models/r_axis_pid.slx:
%
%   r_ref -> (+/-) -> Discrete PID -> F_r -> [R-Axis Plant] -> r
%              ^                                         |
%              +-----------------------------------------+
%
% The PID block is configured to match src/controller.py:
%   integral   += e * Ts                (Backward Euler)
%   derivative  = (e - e_prev) / Ts     (no derivative filter, e_prev = 0)
%
% The plant is the same structure verified in r_axis_plant.slx.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));

model = 'r_axis_pid';
model_file = fullfile(project_root, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

%% Reference and controller
add_block('simulink/Sources/From Workspace', [model '/R Reference'], ...
    'VariableName', 'r_ref_data', ...
    'SampleTime', 'Ts', ...
    'Position', [20 90 100 120]);

add_block('simulink/Math Operations/Sum', [model '/Error Sum'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [150 85 180 125]);

add_block('simulink/Discrete/Discrete PID Controller', [model '/R PID'], ...
    'P', 'Kp_r', 'I', 'Ki_r', 'D', 'Kd_r', ...
    'SampleTime', 'Ts', ...
    'IntegratorMethod', 'Backward Euler', ...
    'UseFilter', 'off', ...
    'InitialConditionForIntegrator', '0', ...
    'DifferentiatorICPrevScaledInput', '0', ...
    'Position', [230 85 290 125]);

%% Plant subsystem
plant = [model '/R-Axis Plant'];
add_block('built-in/Subsystem', plant, 'Position', [350 80 450 130]);

add_block('built-in/Inport', [plant '/F_r'], 'Position', [30 93 60 107]);

add_block('simulink/Math Operations/Sum', [plant '/Force Sum'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [100 85 130 125]);

add_block('simulink/Math Operations/Gain', [plant '/Inverse Mass'], ...
    'Gain', '1/m_r', 'Position', [170 90 220 120]);

add_block('simulink/Continuous/Integrator', [plant '/Velocity Integrator'], ...
    'InitialCondition', '0', 'Position', [270 90 300 120]);

add_block('simulink/Continuous/Integrator', [plant '/Position Integrator'], ...
    'InitialCondition', 'r_initial', 'Position', [370 90 400 120]);

add_block('simulink/Math Operations/Gain', [plant '/Damping'], ...
    'Gain', 'b_r', 'Orientation', 'left', ...
    'Position', [260 170 310 200]);

add_block('built-in/Outport', [plant '/r'], 'Position', [460 93 490 107]);
add_block('built-in/Outport', [plant '/r_dot'], 'Position', [460 23 490 37]);

add_line(plant, 'F_r/1', 'Force Sum/1');
h = add_line(plant, 'Force Sum/1', 'Inverse Mass/1');
set(h, 'Name', 'F_net');
h = add_line(plant, 'Inverse Mass/1', 'Velocity Integrator/1');
set(h, 'Name', 'r_ddot');
h = add_line(plant, 'Velocity Integrator/1', 'Position Integrator/1');
set(h, 'Name', 'r_dot');
add_line(plant, 'Velocity Integrator/1', 'Damping/1', 'autorouting', 'smart');
add_line(plant, 'Velocity Integrator/1', 'r_dot/1', 'autorouting', 'smart');
add_line(plant, 'Damping/1', 'Force Sum/2', 'autorouting', 'smart');
add_line(plant, 'Position Integrator/1', 'r/1');

%% Logging
add_block('simulink/Sinks/To Workspace', [model '/Log r'], ...
    'VariableName', 'r', 'SaveFormat', 'Timeseries', ...
    'Position', [540 90 600 120]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_ref'], ...
    'VariableName', 'r_ref', 'SaveFormat', 'Timeseries', ...
    'Position', [150 10 210 40]);

add_block('simulink/Sinks/To Workspace', [model '/Log F_r'], ...
    'VariableName', 'F_r', 'SaveFormat', 'Timeseries', ...
    'Position', [350 10 410 40]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_dot'], ...
    'VariableName', 'r_dot', 'SaveFormat', 'Timeseries', ...
    'Position', [540 150 600 180]);

add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'NumInputPorts', '2', 'Position', [540 220 580 270]);

%% Signal lines
h = add_line(model, 'R Reference/1', 'Error Sum/1');
set(h, 'Name', 'r_ref');
add_line(model, 'R Reference/1', 'Log r_ref/1', 'autorouting', 'smart');
add_line(model, 'R Reference/1', 'Scope/1', 'autorouting', 'smart');

h = add_line(model, 'Error Sum/1', 'R PID/1');
set(h, 'Name', 'error');

h = add_line(model, 'R PID/1', 'R-Axis Plant/1');
set(h, 'Name', 'F_r');
add_line(model, 'R PID/1', 'Log F_r/1', 'autorouting', 'smart');

h = add_line(model, 'R-Axis Plant/1', 'Log r/1');
set(h, 'Name', 'r');
add_line(model, 'R-Axis Plant/1', 'Scope/2', 'autorouting', 'smart');
add_line(model, 'R-Axis Plant/1', 'Error Sum/2', 'autorouting', 'smart');

add_line(model, 'R-Axis Plant/2', 'Log r_dot/1', 'autorouting', 'smart');

%% Solver: fixed step equal to the controller sample time
set_param(model, ...
    'StopTime', 'simulation_time', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'Ts');

save_system(model, model_file);
close_system(model);

fprintf('Saved %s\n', model_file);
