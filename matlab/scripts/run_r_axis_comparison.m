%% Compare R-axis PID vs PID + feedforward (Simulink)
%
% Runs r_axis_pid.slx and r_axis_pid_ff.slx on the same quintic reference
% (0.1 m -> 0.4 m in 2 s) and reports tracking performance.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

[t_ref, r_des, r_vel, r_acc] = quintic_trajectory(r_initial, r_final, simulation_time, num_points);
r_ref_data = [t_ref, r_des];
r_vel_data = [t_ref, r_vel];
r_acc_data = [t_ref, r_acc];

out_pid = sim('r_axis_pid');
out_ff = sim('r_axis_pid_ff');

t = out_pid.r.Time;
error_pid = out_pid.r_ref.Data - out_pid.r.Data;
error_ff = out_ff.r_ref.Data - out_ff.r.Data;

rmse_pid = sqrt(mean(error_pid .^ 2));
rmse_ff = sqrt(mean(error_ff .^ 2));
max_pid = max(abs(error_pid));
max_ff = max(abs(error_ff));

fprintf('R-Axis Controller Comparison (Simulink)\n');
fprintf('---------------------------------------\n');
fprintf('                    PID        PID+FF     (Python PID / PID+FF)\n');
fprintf('RMSE (mm)        : %6.2f     %6.2f     (5.58 / 0.36)\n', rmse_pid * 1000, rmse_ff * 1000);
fprintf('Max abs err (mm) : %6.2f     %6.2f     (8.42 / 0.56)\n', max_pid * 1000, max_ff * 1000);
fprintf('Final error (mm) : %6.2f     %6.2f\n', error_pid(end) * 1000, error_ff(end) * 1000);
fprintf('Peak |F_r| (N)   : %6.2f     %6.2f\n', max(abs(out_pid.F_r.Data)), max(abs(out_ff.F_r.Data)));
fprintf('RMSE reduction   : %.1f %%\n', 100 * (1 - rmse_ff / rmse_pid));

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);

subplot(3, 1, 1);
plot(t, out_pid.r_ref.Data * 1000, 'k', 'LineWidth', 2); hold on;
plot(t, out_pid.r.Data * 1000, '--', 'LineWidth', 1.5);
plot(t, out_ff.r.Data * 1000, '-.', 'LineWidth', 1.5);
ylabel('R Position (mm)');
title('R-Axis: PID vs PID + Feedforward (Simulink)');
legend('Desired', 'PID', 'PID + FF', 'Location', 'northwest');
grid on;

subplot(3, 1, 2);
plot(t, error_pid * 1000, 'LineWidth', 1.5); hold on;
plot(t, error_ff * 1000, 'LineWidth', 1.5);
ylabel('Error (mm)');
legend('PID', 'PID + FF', 'Location', 'northeast');
grid on;

subplot(3, 1, 3);
plot(t, out_pid.F_r.Data, 'LineWidth', 1.5); hold on;
plot(t, out_ff.F_r.Data, 'LineWidth', 1.5);
plot(t, out_ff.F_ff.Data, ':', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Force (N)');
legend('PID total', 'PID + FF total', 'FF part only', 'Location', 'northeast');
grid on;

results_file = fullfile(project_root, 'results', 'r_axis_controller_comparison.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
