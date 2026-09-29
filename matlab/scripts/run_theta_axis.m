%% Theta axis: plant check, then PID vs PID + feedforward
%
% Reference: quintic 0 -> pi/2 rad in 2 s (same as the Python simulations).

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

cfg = axis_config('theta');

%% 1. Open-loop plant check (constant 0.1 N*m torque)
verify_axis_plant(cfg, 0.1);

%% 2. Closed loop
model_file = fullfile(project_root, 'models', 'theta_axis_model.slx');
build_axis_model(model_file, cfg);

ff_enable = 0;
out_pid = sim('theta_axis_model');
ff_enable = 1;
out_ff = sim('theta_axis_model');

t = out_pid.q.Time;
error_pid = out_pid.q_ref.Data - out_pid.q.Data;
error_ff = out_ff.q_ref.Data - out_ff.q.Data;

rmse_pid = sqrt(mean(error_pid .^ 2));
rmse_ff = sqrt(mean(error_ff .^ 2));
max_pid = max(abs(error_pid));
max_ff = max(abs(error_ff));

fprintf('\nTheta-Axis Controller Comparison (Simulink)\n');
fprintf('-------------------------------------------\n');
fprintf('                      PID        PID+FF     (Python PID / PID+FF)\n');
fprintf('RMSE (deg)         : %6.3f     %6.3f     (0.45 / 0.11)\n', rad2deg(rmse_pid), rad2deg(rmse_ff));
fprintf('Max abs err (deg)  : %6.3f     %6.3f     (0.68 / 0.17)\n', rad2deg(max_pid), rad2deg(max_ff));
fprintf('Final error (deg)  : %6.3f     %6.3f\n', rad2deg(error_pid(end)), rad2deg(error_ff(end)));
fprintf('Peak |torque| (Nm) : %6.3f     %6.3f     (limit %.0f N*m)\n', ...
    max(abs(out_pid.u.Data)), max(abs(out_ff.u.Data)), T_theta_max);
fprintf('RMSE reduction     : %.1f %%\n', 100 * (1 - rmse_ff / rmse_pid));

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);

subplot(3, 1, 1);
plot(t, rad2deg(out_pid.q_ref.Data), 'k', 'LineWidth', 2); hold on;
plot(t, rad2deg(out_pid.q.Data), '--', 'LineWidth', 1.5);
plot(t, rad2deg(out_ff.q.Data), '-.', 'LineWidth', 1.5);
ylabel('\theta (deg)');
title('Theta-Axis: PID vs PID + Feedforward (Simulink)');
legend('Desired', 'PID', 'PID + FF', 'Location', 'northwest');
grid on;

subplot(3, 1, 2);
plot(t, rad2deg(error_pid), 'LineWidth', 1.5); hold on;
plot(t, rad2deg(error_ff), 'LineWidth', 1.5);
ylabel('Error (deg)');
legend('PID', 'PID + FF', 'Location', 'northeast');
grid on;

subplot(3, 1, 3);
plot(t, out_pid.u.Data, 'LineWidth', 1.5); hold on;
plot(t, out_ff.u.Data, 'LineWidth', 1.5);
plot(t, out_ff.u_ff.Data, ':', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Torque (N\cdotm)');
legend('PID total', 'PID + FF total', 'FF part only', 'Location', 'northeast');
grid on;

results_file = fullfile(project_root, 'results', 'theta_axis_controller_comparison.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
