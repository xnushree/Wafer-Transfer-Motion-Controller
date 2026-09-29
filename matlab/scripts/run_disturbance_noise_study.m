%% Disturbance and sensor-noise study (three-axis robot)
%
% Normal 2 s move, then the robot holds position. At t = 2.5 s a step
% disturbance acts on every axis:
%   R:     2 N push (e.g. cable drag / light contact)
%   Theta: 0.5 N*m torque
%   Z:     extra weight of a 300 mm wafer (~0.128 kg) -> -m_wafer * g
% Sensor noise (std): 10 um on R and Z, 0.005 deg on theta.
%
% Cases:
%   1 Nominal                 (PID + FF)
%   2 Disturbance only        (PID + FF)
%   3 Noise only              (PID + FF)
%   4 Disturbance + noise     (PID)
%   5 Disturbance + noise     (PID + FF)

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

model = 'robot_3axis';
move_time = 2.0;
simulation_time = 4.0;    % Ts unchanged
disturbance_time = 2.5;
gravity_comp = 1;

m_wafer = 0.128;          % kg, approximate 300 mm silicon wafer
dist = struct('d_r', 2.0, 'd_theta', 0.5, 'd_z', -m_wafer * g);
noise = struct('noise_r', 10e-6, 'noise_theta', deg2rad(0.005), 'noise_z', 10e-6);

case_names = {'Nominal (FF)', 'Disturbance (FF)', 'Noise (FF)', ...
    'Dist + noise (PID)', 'Dist + noise (FF)'};
case_dist = [0 1 0 1 1];
case_noise = [0 0 1 1 1];
case_ff = [1 1 1 0 1];

outs = cell(1, 5);
for c = 1:5
    d_r = case_dist(c) * dist.d_r;
    d_theta = case_dist(c) * dist.d_theta;
    d_z = case_dist(c) * dist.d_z;
    noise_r = case_noise(c) * noise.noise_r;
    noise_theta = case_noise(c) * noise.noise_theta;
    noise_z = case_noise(c) * noise.noise_z;
    ff_enable = case_ff(c);
    outs{c} = sim(model);
end

%% Metrics
axis_names = {'r', 'theta', 'z'};
force_names = {'F_r', 'T_theta', 'F_z'};
labels = {'R', 'Theta', 'Z'};
scale = [1000, 180 / pi, 1000];
units = {'mm', 'deg', 'mm'};
band = [0.1, 0.01, 0.1];
force_units = {'N', 'N*m', 'N'};

t = outs{1}.r.Time;
in_move = t <= move_time;
before_dist = t > 0.5 & t < disturbance_time;
after_dist = t >= disturbance_time;
last_half = t >= simulation_time - 0.5;

fprintf('Disturbance at %.1f s: R %.1f N, Theta %.2f N*m, Z %.3f N (wafer %.3f kg)\n', ...
    disturbance_time, dist.d_r, dist.d_theta, dist.d_z, m_wafer);
fprintf('Noise std: R %.0f um, Theta %.3f deg, Z %.0f um\n\n', ...
    noise.noise_r * 1e6, rad2deg(noise.noise_theta), noise.noise_z * 1e6);

for k = 1:3
    fprintf('%s axis (recovery band %.2g %s)\n', labels{k}, band(k), units{k});
    fprintf('  %-20s %13s %15s %13s %15s %17s\n', 'Case', 'Move RMSE', ...
        'Peak after dist', 'Recovery(s)', 'Final mean err', 'u jitter std');
    u_nominal = outs{1}.(force_names{k}).Data;
    for c = 1:5
        o = outs{c};
        e = (o.([axis_names{k} '_ref']).Data - o.(axis_names{k}).Data) * scale(k);
        u = o.(force_names{k}).Data;

        e_after = e(after_dist);
        t_after = t(after_dist);
        if case_dist(c)
            outside = find(abs(e_after) > band(k), 1, 'last');
            if isempty(outside)
                recovery = '0.000';
            elseif outside == numel(e_after)
                recovery = 'not recovered';
            else
                recovery = sprintf('%.3f', t_after(outside + 1) - disturbance_time);
            end
        else
            recovery = '-';
        end

        fprintf('  %-20s %10.4f %s %12.4f %s %13s %12.5f %s %11.4f %s\n', case_names{c}, ...
            sqrt(mean(e(in_move) .^ 2)), units{k}, max(abs(e_after)), units{k}, ...
            recovery, mean(e(last_half)), units{k}, ...
            std(u(before_dist) - u_nominal(before_dist)), force_units{k});
    end
    fprintf('\n');
end

%% Plots
fig = figure('Visible', 'off', 'Position', [100 100 1150 900]);
colors = [0 0.447 0.741; 0.850 0.325 0.098; 0.929 0.694 0.125; 0.494 0.184 0.556; 0.466 0.674 0.188];
plot_cases = [2 3 4 5];

for k = 1:3
    subplot(3, 2, 2 * k - 1);
    for c = plot_cases
        o = outs{c};
        e = (o.([axis_names{k} '_ref']).Data - o.(axis_names{k}).Data) * scale(k);
        plot(t, e, 'Color', colors(c, :), 'LineWidth', 1.2); hold on;
    end
    xline(disturbance_time, 'k:', 'disturbance');
    xline(move_time, 'k:', 'move end');
    ylabel(sprintf('%s error (%s)', labels{k}, units{k}));
    if k == 1
        title('Tracking error (true position)');
        legend(case_names(plot_cases), 'Location', 'northwest');
    end
    grid on;

    subplot(3, 2, 2 * k);
    for c = [1 5]
        plot(t, outs{c}.(force_names{k}).Data, 'Color', colors(c, :), 'LineWidth', 1.2); hold on;
    end
    xline(disturbance_time, 'k:');
    ylabel(sprintf('%s (%s)', strrep(force_names{k}, '_', '\_'), force_units{k}));
    if k == 1
        title('Actuator command');
        legend(case_names([1 5]), 'Location', 'northeast');
    end
    grid on;
end
subplot(3, 2, 5); xlabel('Time (s)');
subplot(3, 2, 6); xlabel('Time (s)');

results_file = fullfile(project_root, 'results', 'disturbance_noise_study.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
