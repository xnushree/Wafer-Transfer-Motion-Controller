function [Kp, Ki, Kd] = pid_pole_placement(J, b, omega)
%PID_POLE_PLACEMENT PID gains placing all closed-loop poles at s = -omega
%
%   Plant J*q_ddot + b*q_dot = u with PID u = Kp e + Ki int(e) + Kd e_dot
%   gives the characteristic polynomial
%       J s^3 + (b + Kd) s^2 + Kp s + Ki
%   Matching it to J (s + omega)^3 = J (s^3 + 3 omega s^2 + 3 omega^2 s + omega^3):
%       Kd = 3 J omega - b,   Kp = 3 J omega^2,   Ki = J omega^3
%
%   Continuous-time design; valid when omega * Ts << 1.

Kp = 3 * J * omega^2;
Ki = J * omega^3;
Kd = 3 * J * omega - b;

end
