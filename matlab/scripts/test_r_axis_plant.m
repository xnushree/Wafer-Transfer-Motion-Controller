%% Verify the open-loop R-axis plant against the analytical solution
%
% For a constant force F starting from rest at r0:
%
%   r_dot(t) = (F/b) * (1 - exp(-b t / m))
%   r(t)     = r0 + (F/b) * (t - (m/b) * (1 - exp(-b t / m)))

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));

F_test = 1.0;   % N, must match build_r_axis_plant.m

addpath(fullfile(project_root, 'models'));
out = sim('r_axis_plant');

t = out.r.Time;
r_sim = out.r.Data;
r_dot_sim = out.r_dot.Data;

tau = m_r / b_r;
r_dot_exact = (F_test / b_r) * (1 - exp(-t / tau));
r_exact = r_initial + (F_test / b_r) * (t - tau * (1 - exp(-t / tau)));

r_error = max(abs(r_sim - r_exact));
r_dot_error = max(abs(r_dot_sim - r_dot_exact));

fprintf('R-axis plant open-loop check (F = %.1f N)\n', F_test);
fprintf('-----------------------------------------\n');
fprintf('Final r      : %.6f m   (exact %.6f m)\n', r_sim(end), r_exact(end));
fprintf('Final r_dot  : %.6f m/s (exact %.6f m/s)\n', r_dot_sim(end), r_dot_exact(end));
fprintf('Max |r error|     : %.3e m\n', r_error);
fprintf('Max |r_dot error| : %.3e m/s\n', r_dot_error);

tolerance = 1e-6;
if r_error < tolerance && r_dot_error < tolerance
    fprintf('RESULT: PASS (tolerance %.0e)\n', tolerance);
else
    fprintf('RESULT: FAIL (tolerance %.0e)\n', tolerance);
end

%% Plot
fig = figure('Visible', 'off');

subplot(2, 1, 1);
plot(t, r_exact * 1000, 'LineWidth', 2); hold on;
plot(t, r_sim * 1000, '--', 'LineWidth', 1.5);
ylabel('R Position (mm)');
title('R-Axis Plant: Simulink vs Analytical (F = 1 N)');
legend('Analytical', 'Simulink', 'Location', 'northwest');
grid on;

subplot(2, 1, 2);
plot(t, r_dot_exact * 1000, 'LineWidth', 2); hold on;
plot(t, r_dot_sim * 1000, '--', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('R Velocity (mm/s)');
legend('Analytical', 'Simulink', 'Location', 'northwest');
grid on;

results_file = fullfile(project_root, 'results', 'r_axis_plant_check.png');
exportgraphics(fig, results_file, 'Resolution', 200);
close(fig);

fprintf('Saved %s\n', results_file);
