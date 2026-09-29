function add_forward_kinematics(block, position)
%ADD_FORWARD_KINEMATICS Add an R-Theta-Z -> Cartesian subsystem
%
%   Inputs: 1 r, 2 theta, 3 z     Output: [x; y; z]
%   x = r cos(theta),  y = r sin(theta),  z = z
%   Same as src/robot_kinematics.py forward_kinematics.

add_block('built-in/Subsystem', block, 'Position', position);

add_block('built-in/Inport', [block '/r'], 'Position', [30 43 60 57]);
add_block('built-in/Inport', [block '/theta'], 'Position', [30 93 60 107]);
add_block('built-in/Inport', [block '/z'], 'Position', [30 143 60 157]);

fcn = [block '/FK'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
    'Position', [130 50 250 150]);

chart = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', fcn);
chart.Script = strjoin({
    'function p = fk(r, theta, z)'
    '% Cylindrical (R-Theta-Z) to Cartesian end-effector position'
    'p = [r * cos(theta); r * sin(theta); z];'
    }, newline);

add_block('built-in/Outport', [block '/xyz'], 'Position', [310 93 340 107]);

add_line(block, 'r/1', 'FK/1');
add_line(block, 'theta/1', 'FK/2');
add_line(block, 'z/1', 'FK/3');
add_line(block, 'FK/1', 'xyz/1');

end
