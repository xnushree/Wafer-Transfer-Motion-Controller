function cfg = axis_config(axis_name)
%AXIS_CONFIG Model settings for one robot axis ('r', 'theta' or 'z')
%
%   All values are strings naming workspace variables from parameters.m,
%   so the Simulink models stay linked to the single parameter file.

switch axis_name
    case 'r'
        cfg.label = 'R';
        cfg.inertia = 'm_r';
        cfg.damping = 'b_r';
        cfg.q_initial = 'r_initial';
        cfg.q_final = 'r_final';
        cfg.Kp = 'Kp_r';  cfg.Ki = 'Ki_r';  cfg.Kd = 'Kd_r';
        cfg.gravity_force = '';
        cfg.u_min = '-F_r_max';  cfg.u_max = 'F_r_max';
        cfg.disturbance = 'd_r';  cfg.noise_std = 'noise_r';  cfg.noise_seed = '11';  cfg.disturbance_ff = 'dff_r';

    case 'theta'
        cfg.label = 'Theta';
        cfg.inertia = 'J_theta';
        cfg.damping = 'b_theta';
        cfg.q_initial = 'theta_initial';
        cfg.q_final = 'theta_final';
        cfg.Kp = 'Kp_theta';  cfg.Ki = 'Ki_theta';  cfg.Kd = 'Kd_theta';
        cfg.gravity_force = '';
        cfg.u_min = '-T_theta_max';  cfg.u_max = 'T_theta_max';
        cfg.disturbance = 'd_theta';  cfg.noise_std = 'noise_theta';  cfg.noise_seed = '22';  cfg.disturbance_ff = 'dff_theta';

    case 'z'
        cfg.label = 'Z';
        cfg.inertia = 'm_z';
        cfg.damping = 'b_z';
        cfg.q_initial = 'z_initial';
        cfg.q_final = 'z_final';
        cfg.Kp = 'Kp_z';  cfg.Ki = 'Ki_z';  cfg.Kd = 'Kd_z';
        cfg.gravity_force = 'm_z*g';
        cfg.u_min = 'F_z_min';  cfg.u_max = 'F_z_max';
        cfg.disturbance = 'd_z';  cfg.noise_std = 'noise_z';  cfg.noise_seed = '33';  cfg.disturbance_ff = 'dff_z';

    otherwise
        error('Unknown axis "%s". Use ''r'', ''theta'' or ''z''.', axis_name);
end

cfg.name = axis_name;

end
