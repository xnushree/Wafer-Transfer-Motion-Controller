%% Anti-windup demonstration (theta axis, PID + FF, aggressive move)
%
% The 0.25 s move asks for ~74 N*m while the theta motor is limited to
% +/-20 N*m. With a stronger integral gain the integrator winds up during
% saturation and causes a large overshoot afterwards; clamping anti-windup
% stops the integration while the output is saturated.
%
% Ki_theta is raised ONLY for this demonstration (the project default in
% parameters.m is unchanged). Closed-loop stability requires
% Ki_theta < (b_theta + Kd_theta) * Kp_theta / J_theta = 2020.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

model = 'theta_axis_model';
build_axis_model(fullfile(project_root, 'models', [model '.slx']), axis_config('theta'));

move_time = 0.25;
simulation_time = 3.0;
ff_enable = 1;
band_deg = 0.1;                    % settling band

load_system(model);
pid_block = [model '/Theta Axis/PID'];

%% 1. Gain sweep: overshoot with and without anti-windup
Ki_values = [5 50 100 200 400];
fprintf('Anti-windup gain sweep (theta, %.2f s move, limit +/-%g N*m)\n', move_time, T_theta_max);
fprintf('%8s %20s %20s %12s\n', 'Ki', 'Overshoot no AW', 'Overshoot AW', 'Reduction');
for Ki_theta = Ki_values
    overshoot = zeros(1, 2);
    modes = {'none', 'clamping'};
    for m = 1:2
        set_param(pid_block, 'AntiWindupMode', modes{m});
        o = sim(model);
        overshoot(m) = max(rad2deg(o.q.Data)) - rad2deg(theta_final);
    end
    fprintf('%8g %17.2f deg %17.2f deg %11.0f%%\n', Ki_theta, overshoot(1), overshoot(2), ...
        100 * (1 - overshoot(2) / overshoot(1)));
end

%% 2. Detailed comparison at Ki_theta = 200
Ki_theta = 200;
T_limit = T_theta_max;
cases = {'Unlimited', 'Limited, no AW', 'Limited + clamping AW'};
outs = cell(1, 3);
for c = 1:3
    if c == 1
        T_theta_max = inf;
        set_param(pid_block, 'AntiWindupMode', 'clamping');
    elseif c == 2
        T_theta_max = T_limit;
        set_param(pid_block, 'AntiWindupMode', 'none');
    else
        T_theta_max = T_limit;
        set_param(pid_block, 'AntiWindupMode', 'clamping');
    end
    outs{c} = sim(model);
end
T_theta_max = T_limit;
close_system(model, 0);   % discard the temporary anti-windup changes

fprintf('\nDetailed comparison at Ki_theta = %g\n', Ki_theta);
fprintf('%-24s %14s %14s %16s\n', 'Case', 'Overshoot', 'Settle (s)', 'Peak |T| (N*m)');
for c = 1:3
    o = outs{c};
    t = o.q.Time;
    q = rad2deg(o.q.Data);
    e = rad2deg(o.q_ref.Data - o.q.Data);
    outside = find(abs(e) > band_deg, 1, 'last');
    if isempty(outside)
        settle = '0.000';
    elseif outside == numel(e)
        settle = 'not settled';
    else
        settle = sprintf('%.3f', t(outside + 1));
    end
    fprintf('%-24s %10.2f deg %14s %16.2f\n', cases{c}, max(q) - rad2deg(theta_final), ...
        settle, max(abs(o.u.Data)));
end

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 900 850]);
styles = {'-', '--', '-'};
colors = [0 0.447 0.741; 0.850 0.325 0.098; 0.494 0.184 0.556];
widths = [1.5, 2, 2];

subplot(3, 1, 1);
plot(outs{1}.q_ref.Time, rad2deg(outs{1}.q_ref.Data), 'k:', 'LineWidth', 2); hold on;
for c = 1:3
    plot(outs{c}.q.Time, rad2deg(outs{c}.q.Data), styles{c}, 'Color', colors(c, :), 'LineWidth', widths(c));
end
ylabel('\theta (deg)');
title(sprintf('Anti-Windup Demonstration: \\theta axis, K_i = %g, %.2f s move', Ki_theta, move_time));
legend(['Desired', cases], 'Location', 'southeast');
grid on;

subplot(3, 1, 2);
for c = 1:3
    plot(outs{c}.q.Time, rad2deg(outs{c}.q_ref.Data - outs{c}.q.Data), styles{c}, 'Color', colors(c, :), 'LineWidth', widths(c)); hold on;
end
ylabel('Error (deg)');
grid on;

subplot(3, 1, 3);
for c = 1:3
    plot(outs{c}.u.Time, outs{c}.u.Data, styles{c}, 'Color', colors(c, :), 'LineWidth', widths(c)); hold on;
end
yline(T_limit, 'r:', 'limit');
yline(-T_limit, 'r:', 'limit');
ylim([-80 80]);
xlabel('Time (s)');
ylabel('Torque (N\cdotm)');
grid on;

results_file = fullfile(project_root, 'results', 'antiwindup_demo.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
