# F/T Sensor Laboratory — Phase 1

Synthetic-only 500 Hz six-axis force/torque signal bench. No robot simulation, hardware interface, or motor commands.

## Run

MATLAB with Signal Processing Toolbox required. From repo root:

```matlab
main
addpath(fullfile(pwd,'6D','FT_Sensor'))
addpath(fullfile(pwd,'6D','FT_Sensor','tests'))
test_ft_lab_phase1
results = run_ft_laboratory;
```

Generated artifacts (in ignored results/ folder): signals.csv, metrics.csv, experiment.mat and fx_filter_comparison.png.

The scripts generate an analytic six-axis wrench, synthetic Gaussian noise, bias, drift, same-unit cross-axis coupling, saturation, and quantization. They compare raw measurements to causal EMA and second-order Butterworth low-pass filtering.

Metrics include the unloaded-window Fx noise and offset, first threshold-crossing response time, and separate force and torque RMS errors. The 10 Hz cutoff and all sensor characteristics are illustrative, not real hardware specifications. No packet or serial protocol is implemented in this phase.

Future: phase 2 integrates the existing Tustin controller; phase 3 tests portable sensor-protocol parsing through WSL byte streams; phase 4 characterizes and replays measurements from actual USB–RS485 hardware.

MATLAB is not available in the authoring environment. Tests are written but **not yet executed in MATLAB**.
