%% Actuator saturation and anti-windup study (three-axis robot, PID + FF)
%
% A deliberately aggressive move (move_time = 0.25 s instead of 2 s) asks
% for more force/torque than the actuators can deliver. Three cases:
%
%   A. Unlimited actuators                    (ideal reference)
%   B. Limited actuators, no anti-windup      (like the Python clip)
%   C. Limited actuators, clamping anti-windup
%
% This is a stress test of the control design, not a recommended motion.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

model = 'robot_3axis';
move_time = 0.25;          % s, aggressive move
simulation_time = 1.0;     % s, leave time to see settling (Ts unchanged)
ff_enable = 1;
gravity_comp = 1;

limits = struct('F_r_max', F_r_max, 'T_theta_max', T_theta_max, ...
    'F_z_max', F_z_max, 'F_z_min', F_z_min);

load_system(model);
pid_blocks = strcat(model, '/', {'R Axis', 'Theta Axis', 'Z Axis'}, '/PID');

case_names = {'A: unlimited', 'B: limited, no AW', 'C: limited + AW'};
outs = cell(1, 3);
for c = 1:3
    if c == 1
        F_r_max = inf; T_theta_max = inf; F_z_max = inf; F_z_min = -inf;
    else
        F_r_max = limits.F_r_max; T_theta_max = limits.T_theta_max;
        F_z_max = limits.F_z_max; F_z_min = limits.F_z_min;
    end
    if c == 2
        aw_mode = 'none';
    else
        aw_mode = 'clamping';
    end
    for k = 1:3
        set_param(pid_blocks{k}, 'AntiWindupMode', aw_mode);
    end
    outs{c} = sim(model);
end
close_system(model, 0);   % discard the temporary anti-windup changes

F_r_max = limits.F_r_max; T_theta_max = limits.T_theta_max;
F_z_max = limits.F_z_max; F_z_min = limits.F_z_min;

%% Metrics
axis_names = {'r', 'theta', 'z'};
force_names = {'F_r', 'T_theta', 'F_z'};
labels = {'R', 'Theta', 'Z'};
scale = [1000, 180 / pi, 1000];      % mm, deg, mm
units = {'mm', 'deg', 'mm'};
band = [0.1, 0.01, 0.1];              % settling band in display units
u_lim = [limits.F_r_max, limits.T_theta_max, limits.F_z_max];
u_low = [-limits.F_r_max, -limits.T_theta_max, limits.F_z_min];

fprintf('Saturation study: %.2f s move, %.1f s simulated\n', move_time, simulation_time);
fprintf('Settling = time after which |error| stays below the band\n\n');

for k = 1:3
    fprintf('%s axis (band %.2g %s)\n', labels{k}, band(k), units{k});
    fprintf('  %-20s %12s %12s %12s %12s %14s\n', 'Case', 'MaxErr', 'Overshoot', ...
        'Settle(s)', 'FinalErr', 'Peak |u|');
    for c = 1:3
        o = outs{c};
        t = o.(axis_names{k}).Time;
        q = o.(axis_names{k}).Data * scale(k);
        q_ref = o.([axis_names{k} '_ref']).Data * scale(k);
        u = o.(force_names{k}).Data;
        e = q_ref - q;

        q_final = q_ref(end);
        direction = sign(q_final - q_ref(1));
        overshoot = max(0, max(direction * (q - q_final)));

        outside = find(abs(e) > band(k), 1, 'last');
        if isempty(outside)
            settle = 0;
        elseif outside == numel(e)
            settle = NaN;
        else
            settle = t(outside + 1);
        end

        fprintf('  %-20s %12.3f %12.3f %12s %12.4f %14.2f\n', case_names{c}, ...
            max(abs(e)), overshoot, settle_str(settle), e(end), max(abs(u)));
    end
    fprintf('  Actuator limit: %g .. %g\n\n', u_low(k), u_lim(k));
end

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 1100 900]);
styles = {'-', '--', '-.'};
for k = 1:3
    subplot(3, 2, 2 * k - 1);
    o = outs{1};
    plot(o.r.Time, o.([axis_names{k} '_ref']).Data * scale(k), 'k:', 'LineWidth', 2); hold on;
    for c = 1:3
        o = outs{c};
        plot(o.(axis_names{k}).Time, o.(axis_names{k}).Data * scale(k), styles{c}, 'LineWidth', 1.5);
    end
    ylabel(sprintf('%s (%s)', labels{k}, units{k}));
    if k == 1
        title('Position');
        legend(['Desired', case_names], 'Location', 'southeast');
    end
    grid on;

    subplot(3, 2, 2 * k);
    for c = 1:3
        o = outs{c};
        plot(o.(force_names{k}).Time, o.(force_names{k}).Data, styles{c}, 'LineWidth', 1.5); hold on;
    end
    yline(u_lim(k), 'r:');
    yline(u_low(k), 'r:');
    ylabel(strrep(force_names{k}, '_', '\_'));
    if k == 1
        title('Actuator command (limits dotted red)');
    end
    grid on;
end
subplot(3, 2, 5); xlabel('Time (s)');
subplot(3, 2, 6); xlabel('Time (s)');

results_file = fullfile(project_root, 'results', 'saturation_antiwindup_study.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);

function s = settle_str(value)
if isnan(value)
    s = 'not settled';
else
    s = sprintf('%.3f', value);
end
end
