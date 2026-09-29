%% Build the three-axis R-Theta-Z robot model
%
% Creates models/robot_3axis.slx: three independent axis loops
% (add_axis_loop) plus forward kinematics for the desired and actual
% end-effector positions.
%
% This first version uses the decoupled dynamics (same as Python):
% each axis only feels its own force/torque. Coupling terms
% (centrifugal/Coriolis, r-dependent rotational inertia) can be added later.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

model = 'robot_3axis';
model_file = fullfile(project_root, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

axes_list = {'r', 'theta', 'z'};
input_names = {'F_r', 'T_theta', 'F_z'};

for k = 1:3
    cfg = axis_config(axes_list{k});
    loop = [cfg.label ' Axis'];
    top = 40 + (k - 1) * 260;
    add_axis_loop([model '/' loop], [100 top 230 top + 200], cfg);

    % Per-axis logs: q, q_ref, u
    name = axes_list{k};
    logs = {
        name,               1, [360 top      420 top + 30]
        [name '_ref'],      3, [360 top + 50 420 top + 80]
        input_names{k},     4, [360 top + 100 420 top + 130]
        };
    for j = 1:size(logs, 1)
        add_block('simulink/Sinks/To Workspace', [model '/Log ' logs{j, 1}], ...
            'VariableName', logs{j, 1}, 'SaveFormat', 'Timeseries', ...
            'Position', logs{j, 3});
        add_line(model, sprintf('%s/%d', loop, logs{j, 2}), ...
            ['Log ' logs{j, 1} '/1'], 'autorouting', 'smart');
    end
end

%% Forward kinematics (actual and desired)
add_forward_kinematics([model '/Actual FK'], [520 200 620 300]);
add_forward_kinematics([model '/Desired FK'], [520 420 620 520]);

loops = {'R Axis', 'Theta Axis', 'Z Axis'};
for k = 1:3
    add_line(model, [loops{k} '/1'], sprintf('Actual FK/%d', k), 'autorouting', 'smart');
    add_line(model, [loops{k} '/3'], sprintf('Desired FK/%d', k), 'autorouting', 'smart');
end

add_block('simulink/Sinks/To Workspace', [model '/Log ee'], ...
    'VariableName', 'ee', 'SaveFormat', 'Timeseries', ...
    'Position', [700 235 760 265]);
add_block('simulink/Sinks/To Workspace', [model '/Log ee_ref'], ...
    'VariableName', 'ee_ref', 'SaveFormat', 'Timeseries', ...
    'Position', [700 455 760 485]);

h = add_line(model, 'Actual FK/1', 'Log ee/1');
set(h, 'Name', 'ee_xyz');
h = add_line(model, 'Desired FK/1', 'Log ee_ref/1');
set(h, 'Name', 'ee_xyz_ref');

%% Solver: fixed step equal to the controller sample time
set_param(model, ...
    'StopTime', 'simulation_time', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'Ts');

save_system(model, model_file);
close_system(model);

fprintf('Saved %s\n', model_file);
