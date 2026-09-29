function [time, position, velocity, acceleration] = quintic_trajectory(start_pos, end_pos, duration, num_points)
%QUINTIC_TRAJECTORY Rest-to-rest quintic profile (MATLAB twin of src/trajectory.py)
%
%   Zero velocity and zero acceleration at both ends:
%   s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5,  tau = t / duration

time = linspace(0, duration, num_points)';
tau = time / duration;
delta = end_pos - start_pos;

position = start_pos + delta * (10 * tau.^3 - 15 * tau.^4 + 6 * tau.^5);
velocity = delta / duration * (30 * tau.^2 - 60 * tau.^3 + 30 * tau.^4);
acceleration = delta / duration^2 * (60 * tau - 180 * tau.^2 + 120 * tau.^3);

end
