%% Simulate the R-axis PID closed loop and report tracking performance
%
% Reference: quintic 0.1 m -> 0.4 m in 2 s (same as the Python dynamics
% simulation), so the results can be compared with the Python PID.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

[t_ref, r_des, ~, ~] = quintic_trajectory(r_initial, r_final, simulation_time, num_points);
r_ref_data = [t_ref, r_des];

out = sim('r_axis_pid');

t = out.r.Time;
r_sim = out.r.Data;
r_ref_sim = out.r_ref.Data;
F_sim = out.F_r.Data;

error = r_ref_sim - r_sim;

rmse = sqrt(mean(error .^ 2));
max_error = max(abs(error));
final_error = error(end);

% Python PID results (simulation/controller_comparison.py, same model/gains)
python_rmse = 5.58e-3;
python_max_error = 8.42e-3;

fprintf('R-Axis PID Closed Loop (Simulink)\n');
fprintf('---------------------------------\n');
fprintf('RMSE              : %.2f mm   (Python: %.2f mm)\n', rmse * 1000, python_rmse * 1000);
fprintf('Max abs error     : %.2f mm   (Python: %.2f mm)\n', max_error * 1000, python_max_error * 1000);
fprintf('Final error       : %.2f mm\n', final_error * 1000);
fprintf('Peak |force|      : %.2f N    (limit %.0f N)\n', max(abs(F_sim)), F_r_max);

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);

subplot(3, 1, 1);
plot(t, r_ref_sim * 1000, 'LineWidth', 2); hold on;
plot(t, r_sim * 1000, '--', 'LineWidth', 1.5);
ylabel('R Position (mm)');
title('R-Axis PID Tracking (Simulink)');
legend('Desired', 'Actual', 'Location', 'northwest');
grid on;

subplot(3, 1, 2);
plot(t, error * 1000, 'LineWidth', 1.5);
ylabel('Error (mm)');
grid on;

subplot(3, 1, 3);
plot(t, F_sim, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Force F_r (N)');
grid on;

results_file = fullfile(project_root, 'results', 'r_axis_pid_tracking.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
