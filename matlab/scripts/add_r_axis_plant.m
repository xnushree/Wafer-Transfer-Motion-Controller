function add_r_axis_plant(plant, position)
%ADD_R_AXIS_PLANT Add the R-axis plant subsystem at the given block path
%
%   m_r * r_ddot + b_r * r_dot = F_r
%
%   Input 1: F_r      Output 1: r      Output 2: r_dot
%   Uses workspace variables m_r, b_r, r_initial.

add_block('built-in/Subsystem', plant, 'Position', position);

add_block('built-in/Inport', [plant '/F_r'], 'Position', [30 93 60 107]);

add_block('simulink/Math Operations/Sum', [plant '/Force Sum'], ...
    'Inputs', '+-', 'IconShape', 'rectangular', ...
    'Position', [100 85 130 125]);

add_block('simulink/Math Operations/Gain', [plant '/Inverse Mass'], ...
    'Gain', '1/m_r', 'Position', [170 90 220 120]);

add_block('simulink/Continuous/Integrator', [plant '/Velocity Integrator'], ...
    'InitialCondition', '0', 'Position', [270 90 300 120]);

add_block('simulink/Continuous/Integrator', [plant '/Position Integrator'], ...
    'InitialCondition', 'r_initial', 'Position', [370 90 400 120]);

add_block('simulink/Math Operations/Gain', [plant '/Damping'], ...
    'Gain', 'b_r', 'Orientation', 'left', ...
    'Position', [260 170 310 200]);

add_block('built-in/Outport', [plant '/r'], 'Position', [460 93 490 107]);
add_block('built-in/Outport', [plant '/r_dot'], 'Position', [460 23 490 37]);

add_line(plant, 'F_r/1', 'Force Sum/1');
h = add_line(plant, 'Force Sum/1', 'Inverse Mass/1');
set(h, 'Name', 'F_net');
h = add_line(plant, 'Inverse Mass/1', 'Velocity Integrator/1');
set(h, 'Name', 'r_ddot');
h = add_line(plant, 'Velocity Integrator/1', 'Position Integrator/1');
set(h, 'Name', 'r_dot');
add_line(plant, 'Velocity Integrator/1', 'Damping/1', 'autorouting', 'smart');
add_line(plant, 'Velocity Integrator/1', 'r_dot/1', 'autorouting', 'smart');
add_line(plant, 'Damping/1', 'Force Sum/2', 'autorouting', 'smart');
add_line(plant, 'Position Integrator/1', 'r/1');

end
