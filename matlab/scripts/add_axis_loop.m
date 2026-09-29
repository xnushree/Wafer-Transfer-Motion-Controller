function add_axis_loop(loop, position, cfg)
%ADD_AXIS_LOOP Add a closed-loop single-axis subsystem (trajectory + PID + FF + plant)
%
%   u_ff_total = ff_enable * (inertia*q_acc_ref + damping*q_vel_ref)
%                + known disturbance FF (cfg.disturbance_ff from disturbance_time)
%                + gravity_comp * gravity_force
%   u          = sat( PID(q_ref - q) + u_ff_total )
%
%   Actuator saturation and anti-windup:
%   The PID output is limited to [u_min - u_ff_total, u_max - u_ff_total],
%   so the total command reaches the actuator limit exactly when the PID
%   output is clipped. The PID block's anti-windup (default 'clamping')
%   then stops the integrator from winding up while saturated.
%   Set AntiWindupMode on the PID block to 'none' to disable it.
%   The Actuator Limit block is the physical hard limit.
%
%   A step disturbance (cfg.disturbance at disturbance_time) acts on the
%   plant after the actuator, and Gaussian sensor noise (std cfg.noise_std)
%   is added to the measured position used for feedback. Output q is the
%   true position. Both default to 0 in parameters.m.
%
%   Outputs: 1 q, 2 q_dot, 3 q_ref, 4 u (applied), 5 u_ff
%   Workspace variables: ff_enable, gravity_comp, move_time, Ts and the
%   names in cfg.

has_gravity = ~isempty(cfg.gravity_force);

add_block('built-in/Subsystem', loop, 'Position', position);

%% Trajectory and feedback
add_quintic_trajectory([loop '/Trajectory'], [20 150 110 250], ...
    cfg.q_initial, cfg.q_final, 'move_time');

add_block('simulink/Math Operations/Sum', [loop '/Error Sum'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [170 85 200 125]);

add_block('simulink/Discrete/Discrete PID Controller', [loop '/PID'], ...
    'P', cfg.Kp, 'I', cfg.Ki, 'D', cfg.Kd, ...
    'SampleTime', 'Ts', ...
    'IntegratorMethod', 'Backward Euler', ...
    'UseFilter', 'off', ...
    'InitialConditionForIntegrator', '0', ...
    'DifferentiatorICPrevScaledInput', '0', ...
    'N', 'N_filter', ...
    'FilterMethod', 'Backward Euler', ...
    'LimitOutput', 'on', ...
    'SatLimitsSource', 'external', ...
    'AntiWindupMode', 'clamping', ...
    'Position', [240 75 300 135]);

%% Feedforward
add_block('simulink/Math Operations/Gain', [loop '/FF Damping'], ...
    'Gain', cfg.damping, 'Position', [170 200 220 230]);

add_block('simulink/Math Operations/Gain', [loop '/FF Inertia'], ...
    'Gain', cfg.inertia, 'Position', [170 270 220 300]);

add_block('simulink/Math Operations/Sum', [loop '/FF Sum'], ...
    'Inputs', '++', 'IconShape', 'rectangular', ...
    'Position', [260 225 290 275]);

add_block('simulink/Math Operations/Gain', [loop '/FF Enable'], ...
    'Gain', 'ff_enable', 'Position', [320 235 370 265]);

add_block('simulink/Sources/Step', [loop '/Known Disturbance FF'], ...
    'Time', 'disturbance_time', 'Before', '0', 'After', cfg.disturbance_ff, ...
    'Position', [330 370 360 400]);

if has_gravity
    add_block('simulink/Sources/Constant', [loop '/Gravity Compensation'], ...
        'Value', ['gravity_comp*' cfg.gravity_force], 'Position', [310 310 380 340]);
    ff_total_signs = '+++';
else
    ff_total_signs = '++';
end

add_block('simulink/Math Operations/Sum', [loop '/FF Total'], ...
    'Inputs', ff_total_signs, 'IconShape', 'rectangular', ...
    'Position', [410 240 440 290]);

add_block('simulink/Math Operations/Sum', [loop '/Force Sum'], ...
    'Inputs', '++', 'IconShape', 'rectangular', ...
    'Position', [480 85 510 145]);

%% PID output allowance: actuator limits minus the feedforward share
add_block('simulink/Sources/Constant', [loop '/u_max'], ...
    'Value', cfg.u_max, 'Position', [20 20 80 40]);
add_block('simulink/Sources/Constant', [loop '/u_min'], ...
    'Value', cfg.u_min, 'Position', [20 290 80 310]);

add_block('simulink/Math Operations/Sum', [loop '/Upper Allowance'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [170 20 200 55]);
add_block('simulink/Math Operations/Sum', [loop '/Lower Allowance'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [170 330 200 365]);

%% Physical actuator limit
add_block('simulink/Discontinuities/Saturation', [loop '/Actuator Limit'], ...
    'UpperLimit', cfg.u_max, 'LowerLimit', cfg.u_min, ...
    'Position', [550 100 590 130]);

%% Disturbance: step force/torque acting on the plant (not seen by the controller)
add_block('simulink/Sources/Step', [loop '/Disturbance'], ...
    'Time', 'disturbance_time', 'Before', '0', 'After', cfg.disturbance, ...
    'Position', [550 170 580 200]);
add_block('simulink/Math Operations/Sum', [loop '/Disturbance Sum'], ...
    'Inputs', '++', 'IconShape', 'round', ...
    'Position', [600 105 620 125]);

%% Sensor noise: added to the measured position used for feedback
add_block('simulink/Sources/Random Number', [loop '/Sensor Noise'], ...
    'Mean', '0', 'Variance', [cfg.noise_std '^2'], ...
    'Seed', cfg.noise_seed, 'SampleTime', 'Ts', ...
    'Position', [630 190 670 220]);
add_block('simulink/Math Operations/Sum', [loop '/Measurement Sum'], ...
    'Inputs', '++', 'IconShape', 'round', ...
    'Position', [750 170 770 190]);

%% Plant
add_axis_plant([loop '/Plant'], [630 90 730 140], cfg);

%% Outputs
outports = {
    'q',     [800 103 830 117]
    'q_dot', [800 173 830 187]
    'q_ref', [800 23 830 37]
    'u',     [800 243 830 257]
    'u_ff',  [800 313 830 327]
    };
for k = 1:size(outports, 1)
    add_block('built-in/Outport', [loop '/' outports{k, 1}], ...
        'Position', outports{k, 2});
end

%% Signal lines
h = add_line(loop, 'Trajectory/1', 'Error Sum/1', 'autorouting', 'smart');
set(h, 'Name', 'q_ref');
add_line(loop, 'Trajectory/1', 'q_ref/1', 'autorouting', 'smart');

h = add_line(loop, 'Trajectory/2', 'FF Damping/1', 'autorouting', 'smart');
set(h, 'Name', 'q_vel_ref');
h = add_line(loop, 'Trajectory/3', 'FF Inertia/1', 'autorouting', 'smart');
set(h, 'Name', 'q_acc_ref');

add_line(loop, 'FF Damping/1', 'FF Sum/1', 'autorouting', 'smart');
add_line(loop, 'FF Inertia/1', 'FF Sum/2', 'autorouting', 'smart');
add_line(loop, 'FF Sum/1', 'FF Enable/1');

h = add_line(loop, 'FF Enable/1', 'FF Total/1', 'autorouting', 'smart');
set(h, 'Name', 'u_ff');
add_line(loop, 'FF Enable/1', 'u_ff/1', 'autorouting', 'smart');
h = add_line(loop, 'Known Disturbance FF/1', 'FF Total/2', 'autorouting', 'smart');
set(h, 'Name', 'u_dff');
if has_gravity
    h = add_line(loop, 'Gravity Compensation/1', 'FF Total/3', 'autorouting', 'smart');
    set(h, 'Name', 'u_gravity');
end

h = add_line(loop, 'Error Sum/1', 'PID/1');
set(h, 'Name', 'error');
h = add_line(loop, 'PID/1', 'Force Sum/1');
set(h, 'Name', 'u_pid');
h = add_line(loop, 'FF Total/1', 'Force Sum/2', 'autorouting', 'smart');
set(h, 'Name', 'u_ff_total');

add_line(loop, 'u_max/1', 'Upper Allowance/1', 'autorouting', 'smart');
add_line(loop, 'FF Total/1', 'Upper Allowance/2', 'autorouting', 'smart');
add_line(loop, 'u_min/1', 'Lower Allowance/1', 'autorouting', 'smart');
add_line(loop, 'FF Total/1', 'Lower Allowance/2', 'autorouting', 'smart');
add_line(loop, 'Upper Allowance/1', 'PID/2', 'autorouting', 'smart');
add_line(loop, 'Lower Allowance/1', 'PID/3', 'autorouting', 'smart');

h = add_line(loop, 'Force Sum/1', 'Actuator Limit/1');
set(h, 'Name', 'u_cmd');
h = add_line(loop, 'Actuator Limit/1', 'Disturbance Sum/1');
set(h, 'Name', 'u');
add_line(loop, 'Actuator Limit/1', 'u/1', 'autorouting', 'smart');
h = add_line(loop, 'Disturbance/1', 'Disturbance Sum/2', 'autorouting', 'smart');
set(h, 'Name', 'd');
add_line(loop, 'Disturbance Sum/1', 'Plant/1');

h = add_line(loop, 'Plant/1', 'q/1');
set(h, 'Name', 'q');
add_line(loop, 'Plant/1', 'Measurement Sum/1', 'autorouting', 'smart');
add_line(loop, 'Sensor Noise/1', 'Measurement Sum/2', 'autorouting', 'smart');
h = add_line(loop, 'Measurement Sum/1', 'Error Sum/2', 'autorouting', 'smart');
set(h, 'Name', 'q_meas');
h = add_line(loop, 'Plant/2', 'q_dot/1', 'autorouting', 'smart');
set(h, 'Name', 'q_dot');

end
