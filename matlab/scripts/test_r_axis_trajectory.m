%% Verify the Simulink Quintic Trajectory block
%
% 1. The block's position/velocity/acceleration must equal
%    quintic_trajectory.m at every sample.
% 2. The closed loop must give the same tracking result as
%    r_axis_pid_ff.slx (Phase 3), since only the reference source changed.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

out = sim('r_axis_pid_ff_traj');

[~, r_des, r_vel, r_acc] = quintic_trajectory(r_initial, r_final, simulation_time, num_points);

% Logged signals are sampled every Ts, one per trajectory point
pos_diff = max(abs(out.r_ref.Data(:) - r_des));
vel_diff = max(abs(out.r_vel_ref.Data(:) - r_vel));
acc_diff = max(abs(out.r_acc_ref.Data(:) - r_acc));

fprintf('Quintic Trajectory block vs quintic_trajectory.m\n');
fprintf('------------------------------------------------\n');
fprintf('Samples compared   : %d\n', numel(r_des));
fprintf('Max position diff  : %.3e m\n', pos_diff);
fprintf('Max velocity diff  : %.3e m/s\n', vel_diff);
fprintf('Max accel diff     : %.3e m/s^2\n', acc_diff);

fprintf('Start / end pos    : %.4f / %.4f m\n', out.r_ref.Data(1), out.r_ref.Data(end));
fprintf('Start / end vel    : %.2e / %.2e m/s\n', out.r_vel_ref.Data(1), out.r_vel_ref.Data(end));
fprintf('Start / end accel  : %.2e / %.2e m/s^2\n', out.r_acc_ref.Data(1), out.r_acc_ref.Data(end));

error = out.r_ref.Data - out.r.Data;
rmse = sqrt(mean(error .^ 2));
max_error = max(abs(error));

fprintf('\nClosed-loop PID + FF tracking with internal trajectory\n');
fprintf('RMSE          : %.4f mm   (Phase 3: 0.02 mm)\n', rmse * 1000);
fprintf('Max abs error : %.4f mm   (Phase 3: 0.03 mm)\n', max_error * 1000);

tolerance = 1e-9;
if pos_diff < tolerance && vel_diff < tolerance && acc_diff < tolerance
    fprintf('RESULT: PASS (tolerance %.0e)\n', tolerance);
else
    fprintf('RESULT: FAIL (tolerance %.0e)\n', tolerance);
end

%% Plot the reference profile
t = out.r_ref.Time;
fig = figure('Visible', 'off', 'Position', [100 100 800 800]);

subplot(3, 1, 1);
plot(t, out.r_ref.Data * 1000, 'LineWidth', 1.5);
ylabel('Position (mm)');
title('R-Axis Quintic Trajectory (Simulink block)');
grid on;

subplot(3, 1, 2);
plot(t, out.r_vel_ref.Data * 1000, 'LineWidth', 1.5);
ylabel('Velocity (mm/s)');
grid on;

subplot(3, 1, 3);
plot(t, out.r_acc_ref.Data * 1000, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Acceleration (mm/s^2)');
grid on;

results_file = fullfile(project_root, 'results', 'r_axis_quintic_trajectory.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
