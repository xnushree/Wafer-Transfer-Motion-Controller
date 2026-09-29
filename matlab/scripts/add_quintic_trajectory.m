function add_quintic_trajectory(block, position, q0, qf, T)
%ADD_QUINTIC_TRAJECTORY Add a masked quintic trajectory generator subsystem
%
%   add_quintic_trajectory(block, position, q0, qf, T)
%
%   Outputs 1: q, 2: q_dot, 3: q_ddot, sampled every Ts (workspace variable).
%   q0, qf, T are mask parameters given as strings (e.g. 'r_initial').
%   After t = T the output holds qf with zero velocity and acceleration.
%   Same profile as quintic_trajectory.m and src/trajectory.py.

add_block('built-in/Subsystem', block, 'Position', position);

add_block('simulink/Sources/Digital Clock', [block '/Clock'], ...
    'SampleTime', 'Ts', 'Position', [30 90 70 120]);

fcn = [block '/Quintic'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
    'Position', [130 70 250 140]);

add_block('built-in/Outport', [block '/q'], 'Position', [320 73 350 87]);
add_block('built-in/Outport', [block '/q_dot'], 'Position', [320 98 350 112]);
add_block('built-in/Outport', [block '/q_ddot'], 'Position', [320 123 350 137]);

chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin({
    'function [q, q_dot, q_ddot] = quintic(t, q0, qf, T)'
    '% Rest-to-rest quintic: zero velocity and acceleration at both ends'
    'tau = min(max(t / T, 0), 1);'
    'd = qf - q0;'
    'q = q0 + d * (10*tau^3 - 15*tau^4 + 6*tau^5);'
    'q_dot = d / T * (30*tau^2 - 60*tau^3 + 30*tau^4);'
    'q_ddot = d / T^2 * (60*tau - 180*tau^2 + 120*tau^3);'
    }, newline);

for name = {'q0', 'qf', 'T'}
    data = chart.find('-isa', 'Stateflow.Data', 'Name', name{1});
    data.Scope = 'Parameter';
end

add_line(block, 'Clock/1', 'Quintic/1');
add_line(block, 'Quintic/1', 'q/1');
add_line(block, 'Quintic/2', 'q_dot/1');
add_line(block, 'Quintic/3', 'q_ddot/1');

mask = Simulink.Mask.create(block);
mask.Type = 'Quintic Trajectory';
mask.Description = ['Rest-to-rest quintic trajectory from q0 to qf over T seconds. ' ...
    'Outputs position, velocity and acceleration.'];
mask.addParameter('Name', 'q0', 'Prompt', 'Start position q0', 'Value', q0);
mask.addParameter('Name', 'qf', 'Prompt', 'Final position qf', 'Value', qf);
mask.addParameter('Name', 'T', 'Prompt', 'Duration T (s)', 'Value', T);
mask.Display = 'disp(''Quintic Trajectory'')';

end
