function add_axis_plant(plant, position, cfg)
%ADD_AXIS_PLANT Add a single-axis plant subsystem at the given block path
%
%   s * inertia * q_ddot + damping * q_dot + s * gravity_force = u
%
%   s = plant_mass_scale (workspace variable, 1 = nominal). Values other
%   than 1 model a plant heavier/lighter than the controller assumes.
%
%   Input 1: u (force or torque)   Output 1: q   Output 2: q_dot
%   cfg fields (strings of workspace variables/expressions):
%     inertia, damping, q_initial, gravity_force ('' for none)

has_gravity = ~isempty(cfg.gravity_force);

add_block('built-in/Subsystem', plant, 'Position', position);

add_block('built-in/Inport', [plant '/u'], 'Position', [30 93 60 107]);

if has_gravity
    sum_signs = '+--';
else
    sum_signs = '+-';
end
add_block('simulink/Math Operations/Sum', [plant '/Force Sum'], ...
    'Inputs', sum_signs, 'IconShape', 'rectangular', ...
    'Position', [100 80 130 130]);

add_block('simulink/Math Operations/Gain', [plant '/Inverse Inertia'], ...
    'Gain', ['1/(' cfg.inertia '*plant_mass_scale)'], 'Position', [170 90 230 120]);

add_block('simulink/Continuous/Integrator', [plant '/Velocity Integrator'], ...
    'InitialCondition', '0', 'Position', [270 90 300 120]);

add_block('simulink/Continuous/Integrator', [plant '/Position Integrator'], ...
    'InitialCondition', cfg.q_initial, 'Position', [370 90 400 120]);

add_block('simulink/Math Operations/Gain', [plant '/Damping'], ...
    'Gain', cfg.damping, 'Orientation', 'left', ...
    'Position', [260 170 310 200]);

add_block('built-in/Outport', [plant '/q'], 'Position', [460 93 490 107]);
add_block('built-in/Outport', [plant '/q_dot'], 'Position', [460 23 490 37]);

add_line(plant, 'u/1', 'Force Sum/1');
h = add_line(plant, 'Force Sum/1', 'Inverse Inertia/1');
set(h, 'Name', 'u_net');
h = add_line(plant, 'Inverse Inertia/1', 'Velocity Integrator/1');
set(h, 'Name', 'q_ddot');
h = add_line(plant, 'Velocity Integrator/1', 'Position Integrator/1');
set(h, 'Name', 'q_dot');
add_line(plant, 'Velocity Integrator/1', 'Damping/1', 'autorouting', 'smart');
add_line(plant, 'Velocity Integrator/1', 'q_dot/1', 'autorouting', 'smart');
add_line(plant, 'Damping/1', 'Force Sum/2', 'autorouting', 'smart');
add_line(plant, 'Position Integrator/1', 'q/1');

if has_gravity
    add_block('simulink/Sources/Constant', [plant '/Gravity Load'], ...
        'Value', ['plant_mass_scale*' cfg.gravity_force], 'Position', [20 230 90 260]);
    add_line(plant, 'Gravity Load/1', 'Force Sum/3', 'autorouting', 'smart');
end

end
