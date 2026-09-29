%% Build the R-axis PID + feedforward model with an internal trajectory
%
% Creates models/r_axis_pid_ff_traj.slx from r_axis_pid_ff.slx by replacing
% the three From Workspace reference blocks with one Quintic Trajectory
% block (r_initial -> r_final over simulation_time). The model no longer
% needs reference arrays in the workspace.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

source_model = 'r_axis_pid_ff';
model = 'r_axis_pid_ff_traj';
model_file = fullfile(project_root, 'models', [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
if bdIsLoaded(source_model)
    close_system(source_model, 0);
end

load_system(source_model);
save_system(source_model, model_file);   % model is now named r_axis_pid_ff_traj

%% Remove the workspace reference sources
sources = {'R Reference', 'R Velocity Reference', 'R Acceleration Reference'};
for k = 1:numel(sources)
    block = [model '/' sources{k}];
    lines = get_param(block, 'LineHandles');
    delete_line(lines.Outport);
    delete_block(block);
end

%% Add the trajectory generator
add_quintic_trajectory([model '/R Trajectory'], [20 150 110 250], ...
    'r_initial', 'r_final', 'simulation_time');

add_block('simulink/Sinks/To Workspace', [model '/Log r_vel_ref'], ...
    'VariableName', 'r_vel_ref', 'SaveFormat', 'Timeseries', ...
    'Position', [150 320 210 350]);

add_block('simulink/Sinks/To Workspace', [model '/Log r_acc_ref'], ...
    'VariableName', 'r_acc_ref', 'SaveFormat', 'Timeseries', ...
    'Position', [150 380 210 410]);

%% Reconnect
h = add_line(model, 'R Trajectory/1', 'Error Sum/1', 'autorouting', 'smart');
set(h, 'Name', 'r_ref');
add_line(model, 'R Trajectory/1', 'Log r_ref/1', 'autorouting', 'smart');
add_line(model, 'R Trajectory/1', 'Scope/1', 'autorouting', 'smart');

h = add_line(model, 'R Trajectory/2', 'FF Damping/1', 'autorouting', 'smart');
set(h, 'Name', 'r_vel_ref');
add_line(model, 'R Trajectory/2', 'Log r_vel_ref/1', 'autorouting', 'smart');

h = add_line(model, 'R Trajectory/3', 'FF Mass/1', 'autorouting', 'smart');
set(h, 'Name', 'r_acc_ref');
add_line(model, 'R Trajectory/3', 'Log r_acc_ref/1', 'autorouting', 'smart');

save_system(model);
close_system(model);

fprintf('Saved %s\n', model_file);
