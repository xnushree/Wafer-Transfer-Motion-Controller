%% Build the R-axis PID + feedforward model
%
% Creates models/r_axis_pid_ff.slx:
%
%   F_r = PID(r_ref - r) + m_r * r_acc_ref + b_r * r_vel_ref
%
% The feedforward term is the force the plant model needs to follow the
% reference exactly; the PID only corrects the remaining error.
% Matches src/feedforward_controller.py with gravity = 0.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));

model = 'r_axis_pid_ff';
model_file = fullfile(project_root, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

%% Reference signals
add_block('simulink/Sources/From Workspace', [model '/R Reference'], ...
    'VariableName', 'r_ref_data', 'SampleTime', 'Ts', ...
    'Position', [20 90 100 120]);

add_block('simulink/Sources/From Workspace', [model '/R Velocity Reference'], ...
    'VariableName', 'r_vel_data', 'SampleTime', 'Ts', ...
    'Position', [20 200 100 230]);

add_block('simulink/Sources/From Workspace', [model '/R Acceleration Reference'], ...
    'VariableName', 'r_acc_data', 'SampleTime', 'Ts', ...
    'Position', [20 270 100 300]);

%% Feedback path (same PID settings as r_axis_pid.slx)
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

%% Feedforward path
add_block('simulink/Math Operations/Gain', [model '/FF Damping'], ...
    'Gain', 'b_r', 'Position', [160 200 210 230]);

add_block('simulink/Math Operations/Gain', [model '/FF Mass'], ...
    'Gain', 'm_r', 'Position', [160 270 210 300]);

add_block('simulink/Math Operations/Sum', [model '/FF Sum'], ...
    'Inputs', '++', 'IconShape', 'rectangular', ...
    'Position', [260 225 290 275]);

add_block('simulink/Math Operations/Sum', [model '/Force Sum'], ...
    'Inputs', '++', 'IconShape', 'rectangular', ...
    'Position', [340 90 370 150]);

%% Plant
add_r_axis_plant([model '/R-Axis Plant'], [420 95 520 145]);

%% Logging
add_block('simulink/Sinks/To Workspace', [model '/Log r'], ...
    'VariableName', 'r', 'SaveFormat', 'Timeseries', ...
    'Position', [600 100 660 130]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_ref'], ...
    'VariableName', 'r_ref', 'SaveFormat', 'Timeseries', ...
    'Position', [150 10 210 40]);

add_block('simulink/Sinks/To Workspace', [model '/Log F_r'], ...
    'VariableName', 'F_r', 'SaveFormat', 'Timeseries', ...
    'Position', [420 10 480 40]);

add_block('simulink/Sinks/To Workspace', [model '/Log F_ff'], ...
    'VariableName', 'F_ff', 'SaveFormat', 'Timeseries', ...
    'Position', [340 300 400 330]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_dot'], ...
    'VariableName', 'r_dot', 'SaveFormat', 'Timeseries', ...
    'Position', [600 170 660 200]);

add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'NumInputPorts', '2', 'Position', [600 240 640 290]);

%% Signal lines
h = add_line(model, 'R Reference/1', 'Error Sum/1');
set(h, 'Name', 'r_ref');
add_line(model, 'R Reference/1', 'Log r_ref/1', 'autorouting', 'smart');
add_line(model, 'R Reference/1', 'Scope/1', 'autorouting', 'smart');

h = add_line(model, 'Error Sum/1', 'R PID/1');
set(h, 'Name', 'error');

h = add_line(model, 'R PID/1', 'Force Sum/1');
set(h, 'Name', 'F_pid');

h = add_line(model, 'R Velocity Reference/1', 'FF Damping/1');
set(h, 'Name', 'r_vel_ref');
h = add_line(model, 'R Acceleration Reference/1', 'FF Mass/1');
set(h, 'Name', 'r_acc_ref');
add_line(model, 'FF Damping/1', 'FF Sum/1', 'autorouting', 'smart');
add_line(model, 'FF Mass/1', 'FF Sum/2', 'autorouting', 'smart');

h = add_line(model, 'FF Sum/1', 'Force Sum/2', 'autorouting', 'smart');
set(h, 'Name', 'F_ff');
add_line(model, 'FF Sum/1', 'Log F_ff/1', 'autorouting', 'smart');

h = add_line(model, 'Force Sum/1', 'R-Axis Plant/1');
set(h, 'Name', 'F_r');
add_line(model, 'Force Sum/1', 'Log F_r/1', 'autorouting', 'smart');

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
