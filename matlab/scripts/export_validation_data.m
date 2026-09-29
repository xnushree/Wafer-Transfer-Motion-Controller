%% Export three-axis Simulink results for the Python cross-check
%
% Runs robot_3axis for PID and PID + FF (gravity compensation on, no
% disturbance, no noise, nominal plant) and writes one CSV per case:
%
%   results/simulink_validation_pid.csv
%   results/simulink_validation_pid_ff.csv
%
% Columns: t, r, theta, z, r_ref, theta_ref, z_ref, F_r, T_theta, F_z
% The Python script simulation/simulink_validation.py in the Python
% project reads these files.

project_root = fileparts(fileparts(mfilename('fullpath')));
run(fullfile(project_root, 'scripts', 'parameters.m'));
addpath(fullfile(project_root, 'scripts'));
addpath(fullfile(project_root, 'models'));

gravity_comp = 1;
cases = struct('name', {'pid', 'pid_ff'}, 'ff', {0, 1});

for k = 1:numel(cases)
    ff_enable = cases(k).ff;
    out = sim('robot_3axis');

    t = out.r.Time;
    data = [t, ...
        out.r.Data, out.theta.Data, out.z.Data, ...
        out.r_ref.Data, out.theta_ref.Data, out.z_ref.Data, ...
        out.F_r.Data, out.T_theta.Data, out.F_z.Data];

    names = {'t', 'r', 'theta', 'z', 'r_ref', 'theta_ref', 'z_ref', ...
        'F_r', 'T_theta', 'F_z'};
    file = fullfile(project_root, 'results', ...
        ['simulink_validation_' cases(k).name '.csv']);
    writetable(array2table(data, 'VariableNames', names), file);

    fprintf('Saved %s (%d samples, Ts = %.6f s)\n', file, numel(t), Ts);
end
