# Six-Axis Admittance Control — Graduation Project

MATLAB models, experiments, and parameter design for the 6-DoF Cartesian admittance controller used in the graduation-project robotic arm.

The nominal controller uses a diagonal virtual mass–damper model:

`M * dV/dt + D * V = wrench`

Translation uses force (N) and linear velocity (m/s); rotation uses torque (N·m) and angular velocity (rad/s). The nominal discrete controller runs at **500 Hz** (`Ts = 0.002 s`) using Tustin discretization. Design inputs and motion limits in the example scripts are **illustrative**, not validated robot hardware limits.

## Quick start

1. In MATLAB, open this repository as the **Current Folder**.
2. Run `main` once to add the 1D and 6D folders to the MATLAB path.
3. Run `design_6axis_admittance` to calculate six virtual inertia and damping values with constrained optimization (MATLAB Optimization Toolbox required).
4. Run `test_admittance_6d` for the basic six-axis backward-Euler model test, or open any experiment in `1D/analysis/` or `6D/analysis/`.

Alternatively, run scripts directly from the MATLAB Editor after running `main`.

## Repository layout

```text
AdmittanceControl_GP/
├── main.m                          # Configure MATLAB paths; does not run tests
├── 1D/
│   ├── models/                     # 1D continuous, state-space and discretization functions
│   ├── analysis/                   # 1D pole, response, sampling, bias and method studies
│   └── simulink/
│       └── admittance_1d.slx       # Historical single-axis Simulink model
└── 6D/
    ├── design_6axis_admittance.m   # Main six-axis constrained parameter optimizer
    ├── models/                     # 6D admittance, translation and SO(3) integration
    ├── analysis/                   # 6D frequency, time response, robustness and design experiments
    └── tests/
        └── test_admittance_6d.m   # Basic six-axis backward-Euler regression
```

**Models** are reusable MATLAB functions. **Analysis** files are standalone engineering experiments. The main 6D optimizer is kept at the top of `6D/` so it is easy to find. Historical experiments have been preserved, including earlier designs that intentionally demonstrate constraint violations or saturation effects; they are not the final selected controller.

## Principal workflow

`6D/design_6axis_admittance.m`:

- Calculates initial parameters from desired wrench, steady velocity and 90% response time.
- Uses `fmincon` (SQP) to tune 12 positive values (six inertia and six damping) against combined speed, force-reversal acceleration and ideal release-displacement bounds.
- Checks the nominal continuous and Tustin discrete velocity poles.
- Reports time constants, 10–90% rise time, 2% settling time and ideal first-order velocity overshoot.
- Compares original and optimized six-axis Tustin force-reversal responses **without output saturation**.

## Software and limitations

- Developed for MATLAB R2025b; many model/analysis scripts use Control System Toolbox.
- The automatic 6D design requires **Optimization Toolbox** (`fmincon`).
- Opening `1D/simulink/admittance_1d.slx` requires Simulink.
- The scripts validate ideal commanded Cartesian responses, not the complete robot, drives, F/T sensor, human–robot interaction stability or physical safety.
- If you have a MATLAB session with paths from an older checkout, run `main` again after pulling the reorganized repository.

## Organization notes

This cleanup reorganizes files and fixes MATLAB project paths. Existing control and analysis source files have **not** been rewritten or merged, so earlier results remain reproducible with the appropriate toolboxes. The `main.m` initializer and this README are the only text files introduced/updated during the folder cleanup.
