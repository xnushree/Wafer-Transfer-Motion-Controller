function passed = verify_axis_plant(cfg, u_test)
%VERIFY_AXIS_PLANT Open-loop check of add_axis_plant against the exact solution
%
%   Drives the plant with a constant input u_test from rest and compares
%   with the analytical response of  J q_ddot + b q_dot = u - gravity_force:
%
%     q_dot(t) = (u_eff/b) (1 - exp(-t/tau)),  tau = J/b
%     q(t)     = q0 + (u_eff/b) (t - tau (1 - exp(-t/tau)))
%
%   Builds a temporary model in memory (not saved).

model = 'tmp_axis_plant_check';
if bdIsLoaded(model)
    close_system(model, 0);
end
new_system(model);

add_block('simulink/Sources/Constant', [model '/u'], ...
    'Value', num2str(u_test, 17), 'Position', [30 95 80 125]);
add_axis_plant([model '/Plant'], [130 90 230 140], cfg);
add_block('simulink/Sinks/To Workspace', [model '/Log q'], ...
    'VariableName', 'q', 'SaveFormat', 'Timeseries', 'Position', [300 90 360 120]);
add_block('simulink/Sinks/To Workspace', [model '/Log q_dot'], ...
    'VariableName', 'q_dot', 'SaveFormat', 'Timeseries', 'Position', [300 150 360 180]);
add_line(model, 'u/1', 'Plant/1');
add_line(model, 'Plant/1', 'Log q/1');
add_line(model, 'Plant/2', 'Log q_dot/1');

set_param(model, 'StopTime', 'simulation_time', 'Solver', 'ode45', ...
    'RelTol', '1e-8', 'AbsTol', '1e-10', 'MaxStep', '1e-3');

out = sim(model);
close_system(model, 0);

J = evalin('base', cfg.inertia);
b = evalin('base', cfg.damping);
q0 = evalin('base', cfg.q_initial);
if isempty(cfg.gravity_force)
    gravity = 0;
else
    gravity = evalin('base', cfg.gravity_force);
end

t = out.q.Time;
u_eff = u_test - gravity;
tau = J / b;
q_dot_exact = (u_eff / b) * (1 - exp(-t / tau));
q_exact = q0 + (u_eff / b) * (t - tau * (1 - exp(-t / tau)));

q_error = max(abs(out.q.Data - q_exact));
q_dot_error = max(abs(out.q_dot.Data - q_dot_exact));

tolerance = 1e-6;
passed = q_error < tolerance && q_dot_error < tolerance;

fprintf('%s plant open-loop check (u = %g)\n', cfg.label, u_test);
fprintf('  Final q     : %.6f  (exact %.6f)\n', out.q.Data(end), q_exact(end));
fprintf('  Final q_dot : %.6f  (exact %.6f)\n', out.q_dot.Data(end), q_dot_exact(end));
fprintf('  Max errors  : q %.2e, q_dot %.2e\n', q_error, q_dot_error);
if passed
    fprintf('  RESULT: PASS (tolerance %.0e)\n', tolerance);
else
    fprintf('  RESULT: FAIL (tolerance %.0e)\n', tolerance);
end

end
