%% Controller comparison (three-axis robot)
%
% Test: normal 2 s quintic move, hold, then at t = 2.5 s a disturbance
% (R 2 N, Theta 0.5 N*m, Z weight of a picked-up wafer). Sensor noise on.
%
% Controllers:
%   A  PID                    (baseline gains, as in Python)
%   B  PID + FF               (baseline gains, as in Python)
%   C  PID + FF, retuned      (pole placement at omega, derivative filter)
%   D  C + payload FF         (wafer weight added to Z gravity comp at pickup)
%
% Scenarios:
%   Nominal         plant matches the controller model
%   Mismatch +20%   true masses/inertia 20% higher than the controller assumes
%
% Results describe this model and these conditions only.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

model = 'robot_3axis';
move_time = 2.0;
simulation_time = 4.0;       % Ts unchanged
disturbance_time = 2.5;
gravity_comp = 1;

m_wafer = 0.128;             % kg
d_r = 2.0;  d_theta = 0.5;  d_z = -m_wafer * g;
noise_r = 10e-6;  noise_theta = deg2rad(0.005);  noise_z = 10e-6;

omega = 15;                  % rad/s, retuned closed-loop pole location
baseline = struct('Kp_r', Kp_r, 'Ki_r', Ki_r, 'Kd_r', Kd_r, ...
    'Kp_theta', Kp_theta, 'Ki_theta', Ki_theta, 'Kd_theta', Kd_theta, ...
    'Kp_z', Kp_z, 'Ki_z', Ki_z, 'Kd_z', Kd_z);
tuned = struct();
[tuned.Kp_r, tuned.Ki_r, tuned.Kd_r] = pid_pole_placement(m_r, b_r, omega);
[tuned.Kp_theta, tuned.Ki_theta, tuned.Kd_theta] = pid_pole_placement(J_theta, b_theta, omega);
[tuned.Kp_z, tuned.Ki_z, tuned.Kd_z] = pid_pole_placement(m_z, b_z, omega);

fprintf('Retuned gains (omega = %g rad/s, derivative filter N = %g rad/s)\n', omega, N_filter);
fprintf('  R:     Kp %8.1f  Ki %9.1f  Kd %6.1f\n', tuned.Kp_r, tuned.Ki_r, tuned.Kd_r);
fprintf('  Theta: Kp %8.1f  Ki %9.1f  Kd %6.1f\n', tuned.Kp_theta, tuned.Ki_theta, tuned.Kd_theta);
fprintf('  Z:     Kp %8.1f  Ki %9.1f  Kd %6.1f\n\n', tuned.Kp_z, tuned.Ki_z, tuned.Kd_z);

controllers = struct( ...
    'name',   {'A: PID', 'B: PID+FF', 'C: PID+FF tuned', 'D: C + payload FF'}, ...
    'gains',  {baseline, baseline, tuned, tuned}, ...
    'ff',     {0, 1, 1, 1}, ...
    'filter', {'off', 'off', 'on', 'on'}, ...
    'dff_z',  {0, 0, 0, m_wafer * g});
scenarios = struct('name', {'Nominal', 'Mismatch +20%'}, 'mass_scale', {1.0, 1.2});

load_system(model);
pid_blocks = strcat(model, '/', {'R Axis', 'Theta Axis', 'Z Axis'}, '/PID');

outs = cell(numel(scenarios), numel(controllers));
for s = 1:numel(scenarios)
    plant_mass_scale = scenarios(s).mass_scale;
    for c = 1:numel(controllers)
        ctrl = controllers(c);
        gain_names = fieldnames(ctrl.gains);
        for k = 1:numel(gain_names)
            eval([gain_names{k} ' = ctrl.gains.(gain_names{k});']);
        end
        ff_enable = ctrl.ff;
        dff_z = ctrl.dff_z;
        for k = 1:3
            set_param(pid_blocks{k}, 'UseFilter', ctrl.filter);
        end
        outs{s, c} = sim(model);
    end
end
close_system(model, 0);   % discard the temporary filter settings

%% Metrics
axis_names = {'r', 'theta', 'z'};
force_names = {'F_r', 'T_theta', 'F_z'};
labels = {'R', 'Theta', 'Z'};
scale = [1000, 180 / pi, 1000];
units = {'mm', 'deg', 'mm'};
band = [0.1, 0.01, 0.1];
u_offset = [0, 0, m_z * g];            % remove the static hold force for Z effort

t = outs{1, 1}.r.Time;
in_move = t <= move_time;
settle_window = t >= move_time & t < disturbance_time;
jitter_window = t >= move_time + 0.1 & t < disturbance_time;
after_dist = t >= disturbance_time;

rows = {};
for s = 1:numel(scenarios)
    fprintf('==================== %s plant ====================\n', scenarios(s).name);
    for k = 1:3
        fprintf('%s axis (band %.2g %s)\n', labels{k}, band(k), units{k});
        fprintf('  %-18s %9s %9s %9s %9s %10s %10s %9s %9s\n', 'Controller', ...
            'RMSE', 'MaxErr', 'Overshoot', 'Settle', 'EffortRMS', 'Jitter', 'DistPeak', 'Recover');
        for c = 1:numel(controllers)
            o = outs{s, c};
            q = o.(axis_names{k}).Data * scale(k);
            q_ref = o.([axis_names{k} '_ref']).Data * scale(k);
            u = o.(force_names{k}).Data;
            e = q_ref - q;

            q_final = q_ref(end);
            direction = sign(q_final - q_ref(1));
            overshoot = max(0, max(direction * (q(settle_window) - q_final)));

            settle = time_to_band(t(settle_window), e(settle_window), band(k), move_time);
            recover = time_to_band(t(after_dist), e(after_dist), band(k), disturbance_time);

            effort = sqrt(mean((u(in_move) - u_offset(k)) .^ 2));
            jitter = std(u(jitter_window) - movmean(u(jitter_window), 25));
            dist_peak = max(abs(e(after_dist)));

            fprintf('  %-18s %9.4f %9.4f %9.4f %9s %10.3f %10.4f %9.4f %9s\n', ...
                controllers(c).name, sqrt(mean(e(in_move) .^ 2)), max(abs(e(in_move))), ...
                overshoot, fmt_time(settle), effort, jitter, dist_peak, fmt_time(recover));

            rows(end + 1, :) = {scenarios(s).name, labels{k}, controllers(c).name, ...
                sqrt(mean(e(in_move) .^ 2)), max(abs(e(in_move))), overshoot, settle, ...
                effort, jitter, dist_peak, recover, units{k}}; %#ok<SAGROW>
        end
    end
    fprintf('\n');
end
fprintf('RMSE/MaxErr during the move; Overshoot/Settle after the move (s from move end);\n');
fprintf('EffortRMS = RMS actuator command during the move (Z: minus m_z*g);\n');
fprintf('Jitter = high-frequency actuator std while holding (noise effect);\n');
fprintf('DistPeak/Recover = peak error after the disturbance and time back inside the band.\n');

metrics = cell2table(rows, 'VariableNames', {'Scenario', 'Axis', 'Controller', ...
    'MoveRMSE', 'MoveMaxErr', 'Overshoot', 'SettleTime_s', 'EffortRMS', ...
    'Jitter', 'DisturbancePeak', 'RecoveryTime_s', 'Unit'});
csv_file = fullfile(project_root, 'results', 'controller_comparison.csv');
writetable(metrics, csv_file);
fprintf('Saved %s\n', csv_file);

%% Plots: tracking error for each controller, both scenarios
fig = figure('Visible', 'off', 'Position', [100 100 1200 900]);
colors = [0 0.447 0.741; 0.850 0.325 0.098; 0.929 0.694 0.125; 0.494 0.184 0.556];
for s = 1:numel(scenarios)
    for k = 1:3
        subplot(3, 2, 2 * (k - 1) + s);
        for c = 1:numel(controllers)
            o = outs{s, c};
            e = (o.([axis_names{k} '_ref']).Data - o.(axis_names{k}).Data) * scale(k);
            plot(t, e, 'Color', colors(c, :), 'LineWidth', 1.2); hold on;
        end
        xline(move_time, 'k:');
        xline(disturbance_time, 'k:');
        ylabel(sprintf('%s error (%s)', labels{k}, units{k}));
        if k == 1
            title(sprintf('%s plant', scenarios(s).name));
            legend({controllers.name}, 'Location', 'southwest');
        end
        if k == 3
            xlabel('Time (s)  (move ends at 2 s, disturbance at 2.5 s)');
        end
        grid on;
    end
end

results_file = fullfile(project_root, 'results', 'controller_comparison.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);
fprintf('Saved %s\n', results_file);

function value = time_to_band(t, e, band, t0)
% Time (from t0) after which |e| stays inside the band; Inf if it never does.
outside = find(abs(e) > band, 1, 'last');
if isempty(outside)
    value = 0;
elseif outside == numel(e)
    value = Inf;
else
    value = t(outside + 1) - t0;
end
end

function s = fmt_time(value)
if isinf(value)
    s = '>window';
else
    s = sprintf('%.3f', value);
end
end
