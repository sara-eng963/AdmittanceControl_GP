# Six-Axis Admittance — Reusable Simulink Model

This folder contains MATLAB scripts to **generate** a graphical, editable Simulink controller model and a standalone testbench. The generated .slx files must be built and verified on a MATLAB machine; **they are not included until that is done**.

## Why a Model Reference?

The six-axis admittance dynamics are independent of the F/T sensor model. Keeping each as a separately testable model allows a final `HandGuiding_Testbench.slx` to connect the two through model-reference blocks without copying implementation logic.

## Build (at repository root)

```matlab
main
design_6axis_admittance
addpath(fullfile(pwd,'6D','simulink'))
build_admittance_simulink(M,D,Ts)
summary = verify_admittance_simulink;
```

The design script computes the optimized six-axis virtual inertia `M` and damping `D` and defines `Ts=0.002` s. The builder refuses to overwrite any existing .slx or parameter snapshot. Generated files are saved to this directory:

- `Admittance_6D.slx` — reusable reference model with six parallel discrete transfer-function channels; port order `[Fx,Fy,Fz,Tx,Ty,Tz]` to `[vx,vy,vz,wx,wy,wz]`.
- `Admittance_6D_Testbench.slx` — replay a six-axis wrench from the workspace into the reference model; record/plot twist.
- `Admittance_6D_parameters.mat` — exact coefficient design snapshot (`M,D,Ts,a,b`).
- `results/` — generated sample-by-sample verification evidence (ignored by Git).

## Mathematical contract

For each axis:

```text
M_i * vdot_i + D_i * v_i = w_i
v_i[k] = a_i * v_i[k-1] + b_i * (w_i[k] + w_i[k-1])
a_i = (2*M_i-D_i*Ts)/(2*M_i+D_i*Ts)
b_i = Ts/(2*M_i+D_i*Ts)
H_i(z) = b_i*(z+1)/(z-a_i)
```

Block coefficients are numerically snapshotted from the optimizer; **regenerate after changing the design**. The Simulink model uses direct discrete transfer-function blocks, zero initial filter states, and fixed 500 Hz scheduling. All six transfer functions are independently inspectable by double-clicking their blocks.

The verifier drives all six axes, calls the existing `6D/models/admittance_6d_tustin.m` function for its numerical reference, and asserts a per-sample maximum error below `1e-10`. This is a software-model equivalence test, *not* a firmware or hardware test. The solver and reference must start with the same zero initial states.

**Not implemented:** velocity/acceleration clamps, saturation policies, sensor validation, coordinate transforms, supervisor authority, drive dynamics, deterministic physical scheduling or robot safety. Those are separate system behaviors and must not be inferred from a successful test. The standalone model deliberately represents the **nominal, unconstrained six-axis virtual mass-damper** only.

## Final integration

Later, create a top model:

```text
Wrench source -> FT sensor model -> Admittance_6D (Model block)
              -> measured vs reference twist / diagnostics
```

The F/T model must output valid, calibrated 6-by-1 wrenches in the control-model's chosen frame with consistent force and torque units. Any incorrect or stale sensor measurement must be handled by a fault gate, not silently treated as zero force.

**Execution status:** MATLAB/Simulink are not available in the authoring environment, so the generator and its verifier are committed as unexecuted source. An actual passing MATLAB run is required before claiming the .slx models and verified results exist.
