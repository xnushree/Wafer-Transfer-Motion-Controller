function build_axis_model(model_file, cfg)
%BUILD_AXIS_MODEL Build a single-axis closed-loop model (PID + optional FF)
%
%   The loop itself is built by add_axis_loop:
%
%   u = PID(q_ref - q) + ff_enable * (inertia*q_acc_ref + damping*q_vel_ref)
%       + gravity_comp * gravity_force
%
%   ff_enable (workspace variable, 0 or 1) switches feedforward off/on so
%   the same model gives both the PID and the PID + FF result.
%   gravity_comp (workspace variable, 0 or 1) switches gravity
%   compensation for axes with gravity. The Python simulations always
%   apply it (gravity_comp = 1).
%
%   Logged: q, q_dot, q_ref, u, u_ff

[~, model] = fileparts(model_file);

if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

add_axis_loop([model '/' cfg.label ' Axis'], [150 60 280 260], cfg);

%% Logging
logs = {
    'q',     [400 50 460 80]
    'q_dot', [400 100 460 130]
    'q_ref', [400 150 460 180]
    'u',     [400 200 460 230]
    'u_ff',  [400 250 460 280]
    };
for k = 1:size(logs, 1)
    add_block('simulink/Sinks/To Workspace', [model '/Log ' logs{k, 1}], ...
        'VariableName', logs{k, 1}, 'SaveFormat', 'Timeseries', ...
        'Position', logs{k, 2});
    add_line(model, sprintf('%s Axis/%d', cfg.label, k), ...
        ['Log ' logs{k, 1} '/1'], 'autorouting', 'smart');
end

add_block('simulink/Sinks/Scope', [model '/Scope'], ...
    'NumInputPorts', '2', 'Position', [400 320 440 370]);
add_line(model, [cfg.label ' Axis/3'], 'Scope/1', 'autorouting', 'smart');
add_line(model, [cfg.label ' Axis/1'], 'Scope/2', 'autorouting', 'smart');

%% Solver: fixed step equal to the controller sample time
set_param(model, ...
    'StopTime', 'simulation_time', ...
    'SolverType', 'Fixed-step', ...
    'Solver', 'ode4', ...
    'FixedStep', 'Ts');

save_system(model, model_file);
close_system(model);

fprintf('Saved %s\n', model_file);

end
