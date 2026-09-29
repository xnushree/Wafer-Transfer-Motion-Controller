%% Z axis (with gravity): plant check, then controller comparison
%
% Reference: quintic 0.1 m -> 0.25 m in 2 s (same as the Python simulations).
% Cases:
%   PID + gravity compensation          (Python "PID")
%   PID + FF + gravity compensation     (Python "PID + feedforward")
%   PID only, no gravity compensation   (shows why compensation is needed)

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

cfg = axis_config('z');

%% 1. Open-loop plant check: hold force m_z*g plus 1 N upward
verify_axis_plant(cfg, m_z * g + 1.0);

%% 2. Closed loop
model_file = fullfile(project_root, 'models', 'z_axis_model.slx');
build_axis_model(model_file, cfg);

gravity_comp = 1;
ff_enable = 0;
out_pid = sim('z_axis_model');
ff_enable = 1;
out_ff = sim('z_axis_model');

gravity_comp = 0;
ff_enable = 0;
out_nogc = sim('z_axis_model');
gravity_comp = 1;

cases = {out_pid, out_ff, out_nogc};
names = {'PID + GC', 'PID + FF + GC', 'PID, no GC'};
t = out_pid.q.Time;

fprintf('\nZ-Axis Controller Comparison (Simulink)\n');
fprintf('---------------------------------------\n');
fprintf('%-16s %9s %13s %13s %13s %13s\n', 'Case', 'RMSE(mm)', 'MaxErr(mm)', ...
    'FinalErr(mm)', 'MinF_z(N)', 'MaxF_z(N)');
errors = cell(1, 3);
for k = 1:3
    e = cases{k}.q_ref.Data - cases{k}.q.Data;
    errors{k} = e;
    fprintf('%-16s %9.3f %13.3f %13.3f %13.2f %13.2f\n', names{k}, ...
        sqrt(mean(e .^ 2)) * 1000, max(abs(e)) * 1000, e(end) * 1000, ...
        min(cases{k}.u.Data), max(cases{k}.u.Data));
end
fprintf('Python: PID 2.17 / 3.25 mm, PID+FF 0.18 / 0.28 mm (RMSE / max)\n');
fprintf('Actuator range: 0 to %.0f N, hold force m_z*g = %.2f N\n', F_z_max, m_z * g);

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);

subplot(3, 1, 1);
plot(t, out_pid.q_ref.Data * 1000, 'k', 'LineWidth', 2); hold on;
for k = 1:3
    plot(t, cases{k}.q.Data * 1000, '--', 'LineWidth', 1.5);
end
ylabel('Z Position (mm)');
title('Z-Axis with Gravity: Controller Comparison (Simulink)');
legend(['Desired', names], 'Location', 'northwest');
grid on;

subplot(3, 1, 2);
for k = 1:3
    plot(t, errors{k} * 1000, 'LineWidth', 1.5); hold on;
end
ylabel('Error (mm)');
legend(names, 'Location', 'northeast');
grid on;

subplot(3, 1, 3);
for k = 1:3
    plot(t, cases{k}.u.Data, 'LineWidth', 1.5); hold on;
end
yline(m_z * g, ':k', 'm_z g');
xlabel('Time (s)');
ylabel('Force F_z (N)');
legend(names, 'Location', 'southeast');
grid on;

results_file = fullfile(project_root, 'results', 'z_axis_controller_comparison.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
