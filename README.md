# Six-Axis Cartesian Admittance Control

MATLAB design and verification workspace for the **6-DoF robotic arm hand-guiding controller**. The repository contains foundational 1D studies and six-axis modeling, discretization, robustness, and parameter-design experiments.

## Main Design Tool — `6D/design_6axis_admittance.m`

This is the **primary design script**. It automatically calculates six virtual inertia values (`M`) and six virtual damping values (`D`) for the diagonal, zero-stiffness Cartesian admittance model:

```math
\mathbf M\dot{\mathbf V}+\mathbf D\mathbf V=\mathbf w
```

The goal is to match desired hand-guiding speed and responsiveness **while satisfying specified combined-motion limits**, rather than manually tuning parameters or clipping the output.

### Editable inputs (Section 1)

| Variable | Description |
| --- | --- |
| `w_design` | Six-axis operator force/torque envelope: X, Y, Z (N); Rx, Ry, Rz (N·m) |
| `V_target` | Desired steady linear/angular velocity for each axis (m/s; rad/s) |
| `rise90_target` | Desired time to reach 90% of steady velocity for each axis (s) |
| `speedMax` | Combined translation and rotation speed limits |
| `accMax` | Combined linear and angular acceleration limits |
| `releaseMax` | Translational release-travel limit and rotational displacement proxy |
| `margin` | Fractional design margin below specified limits (default: 5%) |
| `speedWeight`, `riseWeight` | Relative importance of matching speed and response-time targets |
| `Ts` | Discrete sampling period (default: 0.002 s / 500 Hz) |

### Calculations and optimization

1. **Initial analytical design:** Compute `D_initial = w_design ./ V_target` and `M_initial = D_initial .* rise90_target / log(10)`.
2. **Constrained tuning:** Use MATLAB **`fmincon` with Sequential Quadratic Programming (SQP)** to optimize all 12 positive parameters. The objective minimizes weighted squared logarithmic deviations from target steady velocities and 90% response times.
3. **Analytical constraints:** Enforce combined translation/rotation bounds on steady speed, force-reversal acceleration, and ideal release displacement, with the specified margin. These are based on the assumed componentwise wrench envelope and ideal mass–damper model.
4. **Performance analysis:** Calculate achieved velocities and rise times, continuous-time and Tustin discrete poles, time constants, 10–90% rise times, 2% settling times, and theoretical step-response overshoot.
5. **Dynamic verification:** Apply the same six-axis push–reverse–release wrench to the initial and optimized **500 Hz Tustin controllers**; compare peak commanded speeds and accelerations and plot all four combined-motion responses **without output saturation**.

### Outputs and engineering value

The script reports the **optimized `M` and `D` vectors** (X, Y, Z, Rx, Ry, Rz), target-versus-achieved performance tables, optimization cost/iterations, analytical constraint checks, transient-response characteristics, and force-reversal plots. Parameters and response arrays remain available in the MATLAB workspace.

This provides a **repeatable, constraint-based method** for selecting virtual admittance parameters, quantifying the trade-off between responsiveness and commanded motion limits, and supplying candidate coefficients for later embedded implementation.

## Run

In MATLAB, open the repository as the Current Folder, then run:

```matlab
main
design_6axis_admittance
```

`main.m` configures the MATLAB paths. The design script requires **MATLAB Optimization Toolbox** (`fmincon`). Other analysis scripts may require Control System Toolbox; the 1D `.slx` model requires Simulink.

## Repository organization

- **`1D/`** — single-axis models, discretization comparisons, analysis, and a Simulink model.
- **`6D/`** — main parameter optimizer, six-axis model functions, analysis experiments, and tests.

**Scope:** The design inputs and motion limits are illustrative, not hardware-validated specifications. The optimized solution is local to the stated mathematical problem. These checks validate the *nominal commanded Cartesian response*, not actuator dynamics, sensor processing, full robot stability, or physical human–robot interaction safety.
