%% HAND-GUIDING SENSITIVITY — STAGE 1: ZERO-FORCE MOTION
% Place beside setup_handguiding_lab.m at the repository root and click Run.
%
% Purpose
%   With commanded applied wrench identically zero, quantify unwanted
%   commanded Cartesian velocities and their time-integrated displacement
%   caused by synthetic F/T measurement bias-estimation error and noise.
%
% Runs
%   EMA nominal cutoff: 10, 20, 30 Hz
%   Gaussian sensor-noise standard deviation: 0.5x, 1x, 2x baseline
%   9 integration simulations + 3 tare simulations (one per noise level)
%
% Results appear only in MATLAB tables, figures, and workspace.
% No files or directories are created by this script.
%
% This experiment reuses the existing referenced subsystems and does not
% edit their .slx files, change the saved Manual Switch, or perform any
% real-hardware tests. It does not prove physical robot motion or safety.

%% 1. Locate the repository and initialize if needed
repoRoot = fileparts(mfilename('fullpath'));
assert(~isempty(repoRoot), 'Save this script in the repository root first.');
addpath(repoRoot);
addpath(fullfile(repoRoot,'6D'));
addpath(fullfile(repoRoot,'6D','models'));
addpath(fullfile(repoRoot,'6D','simulink'));
addpath(fullfile(repoRoot,'6D','FT_Sensor'));
addpath(fullfile(repoRoot,'6D','FT_Sensor','simulink'));

needed = {'cfg','M','D','Ts','A6','B6','ft_wrench_ts', ...
          'noise_mean','noise_variance','noise_seed'};
missingSetup = false;
for j = 1:numel(needed)
    if exist(needed{j},'var') ~= 1
        missingSetup = true;
        break;
    end
end
if missingSetup
    setupFile = fullfile(repoRoot,'setup_handguiding_lab.m');
    assert(isfile(setupFile), 'Missing setup script: %s',setupFile);
    fprintf('Parameters are missing: initializing hand-guiding lab once.\n');
    run(setupFile);
end
for j = 1:numel(needed)
    assert(exist(needed{j},'var') == 1, ...
        'Missing %s; review setup_handguiding_lab.m.',needed{j});
end

%% 2. Validate the fixed experiment configuration
assert(isfinite(Ts) && abs(Ts-0.002)<1e-12 && ...
       abs(cfg.Ts-Ts)<1e-12, 'Expected 500 Hz, Ts = 0.002 s.');
Fs = 1/Ts;
fcList_Hz = [10 20 30];
noiseScales = [0.5 1 2];

% Copy the time grid only; do not modify the original hand-guiding test.
t = ft_wrench_ts.Time(:);
N = numel(t);
assert(N>=501 && abs(t(1))<1e-12 && ...
    max(abs(diff(t)-Ts))<1e-9, ...
    'Expected a uniformly sampled time grid starting at zero.');
assert(numel(A6)==6 && numel(B6)==6 && ...
       numel(noise_mean)==6 && numel(noise_variance)==6 && ...
       numel(noise_seed)==6, ...
       'Expected six-axis parameters and noise configuration.');
assert(all(isfinite(noise_variance(:))) && ...
       all(noise_variance(:)>=0), 'Invalid noise variance.');
assert(all(fcList_Hz>0 & fcList_Hz<Fs/2), ...
       'Cutoff must be below Nyquist.');

% An unloaded, true six-axis wrench for the *entire* test.
Wzero = zeros(N,6);
zeroWrenchTS = timeseries(Wzero,t);
tareWindow = (t>=0.1 & t<0.4);
assert(nnz(tareWindow)>=20, 'Tare window is too short.');

sensorModel = 'FT_Sensor_Lab';
systemModel = 'HandGuiding_Testbench';
load_system(sensorModel);
load_system(systemModel);
switchBlocks = find_system(systemModel,'SearchDepth',1, ...
    'BlockType','ManualSwitch');
assert(numel(switchBlocks)==1, ...
    'Expected one top-level Manual Switch in %s.',systemModel);
switchPath = switchBlocks{1};

% Tested integration wiring: 'sw'='0' selects lower (EMA) input.
% The upper switch input is Butterworth.

% An independent noise realization is used for tare versus evaluation.
% Within each experiment, all cutoff cases reuse exactly the same noise.
baseSeed = double(noise_seed(:));
tareSeed = baseSeed;
testSeed = baseSeed + 1000;
baseNoiseVar = double(noise_variance(:));

p = struct('cfg',cfg, 'M',M, 'D',D, 'Ts',Ts, ...
    'A6',A6, 'B6',B6, 'ft_wrench_ts',zeroWrenchTS, ...
    'noise_mean',noise_mean, 'noise_variance',baseNoiseVar, ...
    'noise_seed',tareSeed, ...
    'alpha',1-exp(-2*pi*10/Fs), ...
    'bButter',[], 'aButter',[], ...
    'bias_est',zeros(6,1));
[p.bButter,p.aButter] = butter(2,10/(Fs/2),'low');

%% 3. Evaluate the three noise levels and three EMA cutoffs
nL = numel(noiseScales);
nF = numel(fcList_Hz);
nCases = nL*nF;

NoiseScale = zeros(nCases,1);
Cutoff_Hz = zeros(nCases,1);
TransPeakSpeed_mm_s = zeros(nCases,1);
TransRMSSpeed_mm_s = zeros(nCases,1);
RotPeakSpeed_rad_s = zeros(nCases,1);
RotRMSSpeed_rad_s = zeros(nCases,1);
FinalTranslationDrift_mm = zeros(nCases,1);
PeakTranslationExcursion_mm = zeros(nCases,1);
FinalAngularIntegral_rad = zeros(nCases,1);
PeakAngularIntegral_rad = zeros(nCases,1);

% Per-axis results for inspecting asymmetry between sensor channels.
AxisNames = ["X";"Y";"Z";"Rx";"Ry";"Rz"];
AxisNoiseScale = zeros(nCases*6,1);
AxisCutoff_Hz = zeros(nCases*6,1);
Axis = strings(nCases*6,1);
PeakVelocity = zeros(nCases*6,1);
RMSVelocity = zeros(nCases*6,1);
FinalIntegratedCommand = zeros(nCases*6,1);
PeakIntegratedCommand = zeros(nCases*6,1);
VelocityUnits = strings(nCases*6,1);
IntegratedUnits = strings(nCases*6,1);

% Keep traces in the MATLAB workspace; dimensions: time x axis x cutoff x level.
VelocityTraces = zeros(N,6,nF,nL);
IntegratedTraces = zeros(N,6,nF,nL);
BiasEstimates = zeros(nL,6);

for iL = 1:nL
    factor = noiseScales(iL);
    p.noise_variance = baseNoiseVar*(factor^2);

    % Tare is estimated from one independent unloaded sensor run per level.
    % Keep the same tare across cutoffs to avoid confounding comparisons.
    p.noise_seed = tareSeed;
    p.bias_est = zeros(6,1);
    tareOut = sim(localInput(sensorModel,p,t(end)));
    rawTare = localSignal(tareOut,'ft_raw_ts',t);
    p.bias_est = mean(rawTare(tareWindow,:),1)';
    BiasEstimates(iL,:) = p.bias_est(:)';

    % Evaluation uses a DIFFERENT noise realization from calibration.
    p.noise_seed = testSeed;

    for iF = 1:nF
        fc = fcList_Hz(iF);
        p.alpha = 1-exp(-2*pi*fc/Fs);
        [p.bButter,p.aButter] = butter(2,fc/(Fs/2),'low');

        integrationInput = localInput(systemModel,p,t(end));
        integrationInput = setBlockParameter(integrationInput, ...
            switchPath,'sw','0');             % EMA selected
        integrationOut = sim(integrationInput);
        V = localSignal(integrationOut,'twist_integrated_ts',t);

        % The simulated output is commanded twist, not measured robot motion.
        % ZOH integration: velocity at t(k) applies until t(k+1).
        Q = [zeros(1,6); cumsum(V(1:end-1,:)*Ts,1)];

        VelocityTraces(:,:,iF,iL) = V;
        IntegratedTraces(:,:,iF,iL) = Q;

        translationSpeed = sqrt(sum(V(:,1:3).^2,2));
        rotationSpeed = sqrt(sum(V(:,4:6).^2,2));
        translationExcursion = sqrt(sum(Q(:,1:3).^2,2));
        angularIntegral = sqrt(sum(Q(:,4:6).^2,2));

        r = (iL-1)*nF+iF;
        NoiseScale(r) = factor;
        Cutoff_Hz(r) = fc;
        TransPeakSpeed_mm_s(r) = 1000*max(translationSpeed);
        TransRMSSpeed_mm_s(r) = 1000*sqrt(mean(translationSpeed.^2));
        RotPeakSpeed_rad_s(r) = max(rotationSpeed);
        RotRMSSpeed_rad_s(r) = sqrt(mean(rotationSpeed.^2));
        FinalTranslationDrift_mm(r) = 1000*norm(Q(end,1:3));
        PeakTranslationExcursion_mm(r) = 1000*max(translationExcursion);
        FinalAngularIntegral_rad(r) = norm(Q(end,4:6));
        PeakAngularIntegral_rad(r) = max(angularIntegral);

        axIds = (r-1)*6+(1:6);
        axisFactor = [1000 1000 1000 1 1 1]';
        AxisNoiseScale(axIds) = factor;
        AxisCutoff_Hz(axIds) = fc;
        Axis(axIds) = AxisNames;
        PeakVelocity(axIds) = max(abs(V),[],1)'.*axisFactor;
        RMSVelocity(axIds) = sqrt(mean(V.^2,1))'.*axisFactor;
        FinalIntegratedCommand(axIds) = Q(end,:)'.*axisFactor;
        PeakIntegratedCommand(axIds) = max(abs(Q),[],1)'.*axisFactor;
        VelocityUnits(axIds) = ["mm/s";"mm/s";"mm/s"; ...
                               "rad/s";"rad/s";"rad/s"];
        IntegratedUnits(axIds) = ["mm";"mm";"mm"; ...
                                 "rad";"rad";"rad"];

        fprintf('Zero force: %.1fx noise, EMA %2g Hz (%d/%d)\n', ...
            factor,fc,r,nCases);
    end
end

%% 4. Collect numerical results in MATLAB workspace
ZeroForceSummary = table(NoiseScale,Cutoff_Hz, ...
    TransPeakSpeed_mm_s,TransRMSSpeed_mm_s, ...
    RotPeakSpeed_rad_s,RotRMSSpeed_rad_s, ...
    FinalTranslationDrift_mm,PeakTranslationExcursion_mm, ...
    FinalAngularIntegral_rad,PeakAngularIntegral_rad);

ZeroForceByAxis = table(AxisNoiseScale,AxisCutoff_Hz,Axis, ...
    PeakVelocity,RMSVelocity,FinalIntegratedCommand, ...
    PeakIntegratedCommand,VelocityUnits,IntegratedUnits, ...
    'VariableNames',{'NoiseScale','Cutoff_Hz','Axis', ...
    'PeakVelocity','RMSVelocity','FinalIntegratedCommand', ...
    'PeakIntegratedCommand','VelocityUnits','IntegratedUnits'});

%% 5. Tradeoff charts, split by synthetic noise level
f1 = figure('Name','Zero-force sensitivity — EMA','Color','w');
tiledlayout(2,2,'Padding','compact');

nexttile;
for iL = 1:nL
    rows = NoiseScale==noiseScales(iL);
    plot(Cutoff_Hz(rows),TransRMSSpeed_mm_s(rows),'o-', ...
        'LineWidth',1.3); hold on;
end
xlabel('EMA nominal cutoff (Hz)');
ylabel('RMS translation speed (mm/s)');
title('Unwanted commanded velocity'); grid on;
legend(compose('%.1fx noise',noiseScales),'Location','best');

nexttile;
for iL = 1:nL
    rows = NoiseScale==noiseScales(iL);
    plot(Cutoff_Hz(rows),FinalTranslationDrift_mm(rows),'o-', ...
        'LineWidth',1.3); hold on;
end
xlabel('EMA nominal cutoff (Hz)');
ylabel('Final translation integral (mm)');
title('Net integrated command'); grid on;

nexttile;
for iL = 1:nL
    rows = NoiseScale==noiseScales(iL);
    plot(Cutoff_Hz(rows),RotRMSSpeed_rad_s(rows),'o-', ...
        'LineWidth',1.3); hold on;
end
xlabel('EMA nominal cutoff (Hz)');
ylabel('RMS angular speed (rad/s)');
title('Unwanted angular command'); grid on;

nexttile;
for iL = 1:nL
    rows = NoiseScale==noiseScales(iL);
    plot(Cutoff_Hz(rows),PeakTranslationExcursion_mm(rows),'o-', ...
        'LineWidth',1.3); hold on;
end
xlabel('EMA nominal cutoff (Hz)');
ylabel('Peak translation integral (mm)');
title('Largest accumulated excursion'); grid on;


% Time histories at baseline noise (1x), X axis, all EMA cutoffs.
baselineIndex = find(noiseScales==1,1,'first');
f2 = figure('Name','Zero force — baseline noise traces','Color','w');
tiledlayout(2,1,'Padding','compact');

nexttile;
for iF = 1:nF
    plot(t,1000*VelocityTraces(:,1,iF,baselineIndex), ...
        'LineWidth',1.2); hold on;
end
ylabel('Commanded X velocity (mm/s)');
title('Zero true force: noise-induced velocity'); grid on;
legend(compose('%g Hz',fcList_Hz),'Location','best');

nexttile;
for iF = 1:nF
    plot(t,1000*IntegratedTraces(:,1,iF,baselineIndex), ...
        'LineWidth',1.2); hold on;
end
xlabel('Time (s)');
ylabel('Integrated X command (mm)');
title('Accumulated requested displacement (ZOH)'); grid on;


%% 6. Display summary; do not declare arbitrary acceptability thresholds
fprintf('\n=== ZERO-FORCE HAND-GUIDING SENSITIVITY (EMA) ===\n');
disp(ZeroForceSummary);
fprintf('Results available in ZeroForceSummary and ZeroForceByAxis.\n');
fprintf('Figures remain open in MATLAB. No files were exported.\n');
fprintf(['All values are synthetic commanded velocity and time-integrated ' ...
    'command, NOT measured motion. Results use one independent evaluation ' ...
    'noise realization per noise level, shared across cutoffs.\n']);

%% Local helpers: same interfaces as the completed cutoff sweep
function in = localInput(model,p,stopTime)
    in = Simulink.SimulationInput(model);
    fields = fieldnames(p);
    for n = 1:numel(fields)
        in = setVariable(in,fields{n},p.(fields{n}));
    end
    in = setModelParameter(in, ...
        'StopTime',num2str(stopTime,17), ...
        'ReturnWorkspaceOutputs','on');
end

function data = localSignal(out,varName,t)
    ts = out.get(varName);
    assert(isa(ts,'timeseries'), ...
        'Missing timeseries output "%s"; check To Workspace.',varName);
    data = squeeze(ts.Data);
    N = numel(t);
    if isequal(size(data),[6 N]), data = data'; end
    assert(isequal(size(data),[N 6]), ...
        'Unexpected dimensions for "%s" (expected %d by 6).',varName,N);
    assert(numel(ts.Time)==N && ...
        max(abs(ts.Time(:)-t(:)))<1e-9, ...
        'Sample time mismatch for "%s".',varName);
    assert(all(isfinite(data(:))), ...
        'Nonfinite signal in "%s".',varName);
end