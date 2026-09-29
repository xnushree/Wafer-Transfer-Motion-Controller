%% Build the open-loop R-axis plant model
%
% Creates models/r_axis_plant.slx implementing
%
%   m_r * r_ddot + b_r * r_dot = F_r
%
% i.e.  Force -> (+/-) -> 1/m_r -> Integrator -> Integrator -> r
%                  ^                    |  (r_dot)
%                  +------ b_r ---------+
%
% A constant test force F_test drives the plant so it can be checked
% against the analytical solution (see test_r_axis_plant.m).

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));

% Test input for the open-loop plant check
F_test = 1.0;   % N

model = 'r_axis_plant';
model_file = fullfile(project_root, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

%% Blocks
add_block('simulink/Sources/Step', [model '/F_r'], ...
    'Time', '0', 'Before', '0', 'After', 'F_test', ...
    'Position', [30 90 60 120]);

add_block('simulink/Math Operations/Sum', [model '/Force Sum'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [110 85 140 125]);

add_block('simulink/Math Operations/Gain', [model '/Inverse Mass'], ...
    'Gain', '1/m_r', ...
    'Position', [190 90 240 120]);

add_block('simulink/Continuous/Integrator', [model '/Velocity Integrator'], ...
    'InitialCondition', '0', ...
    'Position', [290 90 320 120]);

add_block('simulink/Continuous/Integrator', [model '/Position Integrator'], ...
    'InitialCondition', 'r_initial', ...
    'Position', [400 90 430 120]);

add_block('simulink/Math Operations/Gain', [model '/Damping'], ...
    'Gain', 'b_r', 'Orientation', 'left', ...
    'Position', [280 170 330 200]);

add_block('simulink/Sinks/To Workspace', [model '/Log r'], ...
    'VariableName', 'r', 'SaveFormat', 'Timeseries', ...
    'Position', [500 90 560 120]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_dot'], ...
    'VariableName', 'r_dot', 'SaveFormat', 'Timeseries', ...
    'Position', [400 20 460 50]);

add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'NumInputPorts', '2', ...
    'Position', [500 150 540 200]);

%% Signal lines
add_line(model, 'F_r/1', 'Force Sum/1');

h = add_line(model, 'Force Sum/1', 'Inverse Mass/1');
set(h, 'Name', 'F_net');

h = add_line(model, 'Inverse Mass/1', 'Velocity Integrator/1');
set(h, 'Name', 'r_ddot');

h = add_line(model, 'Velocity Integrator/1', 'Position Integrator/1');
set(h, 'Name', 'r_dot');
add_line(model, 'Velocity Integrator/1', 'Damping/1', 'autorouting', 'smart');
add_line(model, 'Velocity Integrator/1', 'Log r_dot/1', 'autorouting', 'smart');
add_line(model, 'Velocity Integrator/1', 'Scope/2', 'autorouting', 'smart');

add_line(model, 'Damping/1', 'Force Sum/2', 'autorouting', 'smart');

h = add_line(model, 'Position Integrator/1', 'Log r/1');
set(h, 'Name', 'r');
add_line(model, 'Position Integrator/1', 'Scope/1', 'autorouting', 'smart');

%% Solver settings
set_param(model, ...
    'StopTime', 'simulation_time', ...
    'Solver', 'ode45', ...
    'RelTol', '1e-8', ...
    'AbsTol', '1e-10', ...
    'MaxStep', '1e-3');

save_system(model, model_file);
close_system(model);

fprintf('Saved %s\n', model_file);
