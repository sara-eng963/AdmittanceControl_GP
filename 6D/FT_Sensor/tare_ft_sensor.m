function [bias_est, noise_seed] = tare_ft_sensor( ...
    cfg, noise_mean, noise_variance, alpha, bButter, aButter)
% TARE_FT_SENSOR Estimate the synthetic F/T sensor zero-load bias.
%
% Uses the existing FT_Sensor_Lab Simulink model.
% Returns the estimated bias and independent evaluation noise seeds.

%% 1. Prepare unloaded sensor input

Ts = cfg.Ts;

tTare = (0:Ts:0.5)';

tare_wrench_ts = timeseries(zeros(numel(tTare),6),tTare);

calibration_seed = cfg.seed + (0:5)';


%% 2. Configure calibration simulation

model = 'FT_Sensor_Lab';

load_system(model);

in = Simulink.SimulationInput(model);

in = in.setVariable('cfg',cfg);
in = in.setVariable('Ts',Ts);
in = in.setVariable('ft_wrench_ts',tare_wrench_ts);

in = in.setVariable('noise_mean',noise_mean);
in = in.setVariable('noise_variance',noise_variance);
in = in.setVariable('noise_seed',calibration_seed);

in = in.setVariable('bias_est',zeros(6,1));

in = in.setVariable('alpha',alpha);
in = in.setVariable('bButter',bButter);
in = in.setVariable('aButter',aButter);

in = in.setModelParameter( ...
    'StopTime','0.5', ...
    'ReturnWorkspaceOutputs','on');


%% 3. Run calibration

out = sim(in);

rawTS = out.get('ft_raw_ts');

raw = squeeze(rawTS.Data);

assert(isequal(size(raw),[numel(rawTS.Time),6]), ...
    'Unexpected sensor measurement dimensions.');


%% 4. Estimate sensor bias

quiet = rawTS.Time(:) >= 0.1 & ...
        rawTS.Time(:) < 0.4;

bias_est = mean(raw(quiet,:),1)';


%% 5. Independent noise seeds for evaluation

noise_seed = calibration_seed + 1000;


%% 6. Display calibration results

fprintf('\n=== F/T SENSOR TARE COMPLETE ===\n');

axisNames = ["Fx";"Fy";"Fz";"Mx";"My";"Mz"];

units = ["N";"N";"N";"Nm";"Nm";"Nm"];

disp(table(axisNames,bias_est,units));

end