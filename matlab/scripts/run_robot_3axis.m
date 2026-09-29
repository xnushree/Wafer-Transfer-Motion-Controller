%% Three-axis robot: joint and Cartesian tracking, PID vs PID + FF
%
% Move: r 0.1 -> 0.4 m, theta 0 -> 90 deg, z 0.1 -> 0.25 m in 2 s.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

gravity_comp = 1;
ff_enable = 0;
out_pid = sim('robot_3axis');
ff_enable = 1;
out_ff = sim('robot_3axis');

outs = {out_pid, out_ff};
names = {'PID', 'PID + FF'};

%% Joint-space metrics
fprintf('Three-Axis Robot (Simulink) - joint tracking\n');
fprintf('--------------------------------------------\n');
fprintf('%-10s %12s %12s %14s %14s %12s %12s\n', 'Case', ...
    'R RMSE(mm)', 'R max(mm)', 'Th RMSE(deg)', 'Th max(deg)', 'Z RMSE(mm)', 'Z max(mm)');
for k = 1:2
    o = outs{k};
    er = o.r_ref.Data - o.r.Data;
    et = o.theta_ref.Data - o.theta.Data;
    ez = o.z_ref.Data - o.z.Data;
    fprintf('%-10s %12.3f %12.3f %14.3f %14.3f %12.3f %12.3f\n', names{k}, ...
        sqrt(mean(er .^ 2)) * 1000, max(abs(er)) * 1000, ...
        rad2deg(sqrt(mean(et .^ 2))), rad2deg(max(abs(et))), ...
        sqrt(mean(ez .^ 2)) * 1000, max(abs(ez)) * 1000);
end

%% Cartesian (end-effector) metrics
fprintf('\nThree-Axis Robot (Simulink) - end-effector tracking\n');
fprintf('---------------------------------------------------\n');
fprintf('%-10s %14s %14s %16s\n', 'Case', 'RMSE (mm)', 'Max (mm)', 'Final (mm)');
cart_error = cell(1, 2);
for k = 1:2
    o = outs{k};
    p = squeeze(o.ee.Data);
    p_ref = squeeze(o.ee_ref.Data);
    if size(p, 1) ~= 3
        p = p';
        p_ref = p_ref';
    end
    e = vecnorm(p_ref - p, 2, 1);
    cart_error{k} = e;
    fprintf('%-10s %14.3f %14.3f %16.3f\n', names{k}, ...
        sqrt(mean(e .^ 2)) * 1000, max(e) * 1000, e(end) * 1000);
end

fprintf('\nPeak |F_r| %.2f N, |T_theta| %.2f N*m, F_z %.2f..%.2f N (PID + FF)\n', ...
    max(abs(out_ff.F_r.Data)), max(abs(out_ff.T_theta.Data)), ...
    min(out_ff.F_z.Data), max(out_ff.F_z.Data));

%% 3D path plot
t = out_pid.r.Time;
p_ref = squeeze(out_ff.ee_ref.Data);
p_pid = squeeze(out_pid.ee.Data);
p_ff = squeeze(out_ff.ee.Data);
if size(p_ref, 1) ~= 3
    p_ref = p_ref'; p_pid = p_pid'; p_ff = p_ff';
end

fig = figure('Visible', 'off', 'Position', [100 100 1100 500]);

subplot(1, 2, 1);
plot3(p_ref(1, :) * 1000, p_ref(2, :) * 1000, p_ref(3, :) * 1000, 'k', 'LineWidth', 2); hold on;
plot3(p_pid(1, :) * 1000, p_pid(2, :) * 1000, p_pid(3, :) * 1000, '--', 'LineWidth', 1.5);
plot3(p_ff(1, :) * 1000, p_ff(2, :) * 1000, p_ff(3, :) * 1000, '-.', 'LineWidth', 1.5);
plot3(p_ref(1, 1) * 1000, p_ref(2, 1) * 1000, p_ref(3, 1) * 1000, 'go', 'MarkerFaceColor', 'g');
plot3(p_ref(1, end) * 1000, p_ref(2, end) * 1000, p_ref(3, end) * 1000, 'ro', 'MarkerFaceColor', 'r');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('End-Effector Path');
legend('Desired', 'PID', 'PID + FF', 'Start', 'End', 'Location', 'northeast');
grid on; axis equal; view(-40, 25);

subplot(1, 2, 2);
plot(t, cart_error{1} * 1000, 'LineWidth', 1.5); hold on;
plot(t, cart_error{2} * 1000, 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('Cartesian error (mm)');
title('End-Effector Position Error');
legend(names, 'Location', 'northeast');
grid on;

results_file = fullfile(project_root, 'results', 'robot_3axis_tracking.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
