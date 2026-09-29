# Engineering notes

These notes explain the design choices and findings behind the results in
the README. As there, every number is a simulation result for this model.

## 1. Python vs Simulink: what the cross-check found

The Simulink model ([robot_3axis.slx](../matlab/models/robot_3axis.slx)) uses a Discrete PID Controller with a
backward-Euler integral and an unfiltered derivative, sampled every 2 ms. The
plant is continuous and solved with the fixed-step ode4 (RK4) solver, also
at 2 ms. The script
[export_validation_data.m](../matlab/scripts/export_validation_data.m) writes the Simulink results to
CSV, and
[simulation/simulink_validation.py](../simulation/simulink_validation.py) compares them with two Python
runs.

In the first Python run the timing matches Simulink: at each sample the
controller reads q(t_k), the force is held constant until the next sample,
and the plant takes one RK4 step. This run differs from Simulink by at most
5.6×10⁻¹³ mm and 2.9×10⁻¹³ degrees, which is rounding error. So the model,
trajectory, PID and feedforward are the same in both tools.

The second run is the original loop from
[controller_comparison.py](../simulation/controller_comparison.py). It
differs from Simulink by up to 0.57 mm and 0.17 degrees. The reason is that
it computes the command for step i from desired[i] and position[i-1], then
integrates with semi-implicit Euler. The error it reports is therefore
measured one sample late, which adds roughly velocity × dt. For the R axis
that is 0.28 m/s × 2 ms ≈ 0.56 mm, almost exactly the 0.563 mm maximum error
it reported for PID + FF. With PID alone the real tracking error (about
8.8 mm) is much larger, so the effect hardly shows. With PID + FF the real
error is only 0.026 mm, so the reported number was almost entirely the
artifact.

This means the earlier claim of a 93.5% RMSE reduction from feedforward was
too low. With correct timing, feedforward reduces the R RMSE from 5.67 mm to
0.016 mm (99.7%), in this ideal case where the model is exact. All the
simulation scripts were then switched to the sampled timing, using
[CylindricalRobotDynamics.rk4_step](../src/robot_dynamics.py) for the ones that simulate dynamics. The
old loop is kept in [simulink_validation.py](../simulation/simulink_validation.py) for reference only.

## 2. Choosing move durations from limits

For a rest-to-rest quintic over a distance d in time T:

$$
v_{peak} = 1.875 \frac{d}{T}, \qquad a_{peak} = 5.7735 \frac{d}{T^2}
$$

[WaferTransferSequence.move_duration](../src/wafer_transfer.py) takes, over all joints, the larger of
1.875·d/v_max and sqrt(5.7735·d/a_max), adds 20% and rounds up to the next
0.1 s. Short moves come out quick and long ones gentle without any tuning,
and the motion is limited by what the wafer can tolerate. The motors are
nowhere near saturation (the peak radial force is 3.6 N of the 100 N
available).

## 3. Safety layer design

Limits are checked over the whole planned trajectory before it runs, rather
than only at each instant, so an unsafe move never starts.

Forbidden zones are cylindrical wedges, each defined by a range of θ, r and
z. This matches how an R-θ-Z arm moves, and checking a point takes only
three comparisons.

A move that would enter a zone is rejected rather than corrected. Moving the
target to the edge of the zone could still sweep the arm through it. The
real fix is to move the joints in a different order, and the planner already
does that: retract, rotate, then extend.

In ROS 2 the safety node checks each plan again, independently of the
planner, as a second line of defence (see
[safety_node.py](../ros2/src/wafer_transfer_robot/wafer_transfer_robot/safety_node.py)). Launching with unsafe_planner:=true
simulates a planner that ignores the zones; the safety node rejects its plan
and the robot does not move.

## 4. ROS 2 timing: why references are looked up by timestamp

At 100 Hz, the offline tests gave these peak R errors depending on which
reference the controller compared the measurement with:

- the reference at the measurement's own timestamp: 0.25 mm
- the same, but with the command arriving one step late: 0.79 mm
- the current reference, with a measurement one message old: 4.24 mm

For this reason
[robot_sim_node](../ros2/src/wafer_transfer_robot/wafer_transfer_robot/robot_sim_node.py) publishes /clock and every node uses sim
time. The plan is published once with a start time, and the controller
([controller_node](../ros2/src/wafer_transfer_robot/wafer_transfer_robot/controller_node.py))
samples it at the timestamp of each joint state message.

## 5. Controlled stop

The first version of the fault handling froze the reference at the current
position. With the arm retracting at about 0.3 m/s, the PID braked at about
2 m/s², twice the R acceleration limit. It overshot by 23 mm and then moved
back.

[ControlledStop](../ros2/src/wafer_transfer_robot/wafer_transfer_robot/sampler.py) instead keeps following the planned path, which has already
been checked, but slows down the plan's clock. Its rate goes smoothly from 1
to 0 (as 1 − smoothstep) over a stop time chosen so that the peak braking
stays within the acceleration limits. With a fault during the R retract at
t = 6.0 s, the peak deceleration was 0.85 m/s² (limit 1.0) and the tracking
error stayed under 0.26 mm. With a fault while rotating and rising at
t = 7.5 s, the peaks were 2.95 rad/s² on θ (limit 4.0) and 0.28 m/s² on Z
(limit 0.5), with errors of 0.02 degrees and 0.09 mm.

A version that slowed the clock down linearly reached 1.30 m/s², above the
limit, because the braking force switched on all at once. Easing it in with
the smoothstep fixed that.

## 6. Coupled dynamics and computed torque

Treating the radial stage as a point mass m_r at radius r
([src/coupled_dynamics.py](../src/coupled_dynamics.py)) gives

$$
M(q) = \mathrm{diag}(m_r,\ J_0 + m_r r^2,\ m_z), \qquad
C(q, \dot{q})\dot{q} = \begin{bmatrix} -m_r r \dot{\theta}^2 \\ 2 m_r r \dot{r} \dot{\theta} \\ 0 \end{bmatrix}
$$

[tests/test_coupled_dynamics.py](../tests/test_coupled_dynamics.py) checks three things. With coupled=False the
model reproduces the decoupled one exactly. A spinning arm is pushed
outwards at r θ̇². With no torque and no damping, the angular momentum
(J_0 + m_r r²) θ̇ stays constant to 1e-8 while the arm extends.

The computed-torque controller
([src/computed_torque_controller.py](../src/computed_torque_controller.py))
applies

$$
\tau = M(q)\left(\ddot{q}_d + \mathrm{PID}(e)\right) + C(q, \dot{q})\dot{q} + B\dot{q} + G
$$

With an exact model this turns each axis into a unit double integrator.

To keep the comparison fair, the computed-torque PID gains are the baseline
gains divided by each axis' nominal inertia (300/5 for R, 100/0.5 for θ and
300/4 for Z). At nominal conditions it is then the same as PID + FF, so any
difference on the coupled plant comes from the model and not from higher
gains. The R RMSE for the 2 s move on each plant:

| | Decoupled plant | Coupled plant |
|---|---|---|
| PID | 5.670 mm | 8.738 mm |
| PID + FF (decoupled model) | 0.016 mm | 5.573 mm |
| Computed torque (coupled model) | 5.064 mm | 0.024 mm |

Each model-based controller does very well on the plant it was designed for
and poorly on the other one, because feedforward is only as good as its
model. On the coupled plant the rotational inertia ranges from 0.55 to
1.30 kg·m² across the reach, which is why a feedforward that assumes
constant inertia falls short.

## 7. Known modelling limits

The coupled model is a point-mass approximation, and the Simulink models and
ROS 2 nodes still use the decoupled one. The zones are only checked against
the fork tip, while the wafer (drawn 200 mm across in RViz) sweeps a larger
area. The plant parameters are illustrative and were not identified from
hardware.
