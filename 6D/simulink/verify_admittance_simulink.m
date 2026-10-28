function summary = verify_admittance_simulink()
%VERIFY_ADMITTANCE_SIMULINK Simulink vs. production-design MATLAB reference.
% Requires generated Admittance_6D*.slx and the existing MATLAB Tustin model.
% Uses only nominal unbounded dynamics; no guidance authorization or motors.

folder=fileparts(mfilename('fullpath'));
addpath(folder);
addpath(fullfile(fileparts(folder),'models'));
p=load(fullfile(folder,'Admittance_6D_parameters.mat'), ...
    'M','D','Ts');
assert(abs(p.Ts-0.002)<1e-12, ...
    'Wrong controller sample time.');
assert(exist(fullfile(folder,'Admittance_6D.slx'),'file')==2);
assert(exist(fullfile(folder,'Admittance_6D_Testbench.slx'),'file')==2);

t=(0:p.Ts:4)';
N=numel(t);
W=zeros(N,6);
W(t>=0.5 & t<1.5,1)=5.0;
W(t>=2.0 & t<2.3,1)=-3.0;
W(t>=0.7 & t<2.2,2)=2*sin(2*pi*1.2*(t(t>=0.7 & t<2.2)-0.7));
W(t>=1.3 & t<1.8,3)=1.5;
W(t>=1.1 & t<1.6,4)=0.25;
mask=t>=0.8 & t<2.8;
W(mask,5)=0.2*sin(2*pi*0.8*(t(mask)-0.8));
W(t>=2.4 & t<2.9,6)=-0.15;

% Existing MATLAB function is the numerical oracle.
reference=zeros(N,6);
for k=2:N
    reference(k,:)=admittance_6d_tustin( ...
        reference(k-1,:)',W(k-1,:)',W(k,:)', ...
        p.M,p.D,p.Ts)';
end

input=Simulink.SimulationInput('Admittance_6D_Testbench');
input=input.setVariable('ft_wrench_ts',timeseries(W,t));
input=input.setModelParameter('StopTime',num2str(t(end),17));
out=sim(input);
trace=out.get('twist_ts');
assert(isa(trace,'timeseries'), 'Missing time-series output.');
tout=trace.Time(:);
actual=squeeze(trace.Data);
if isequal(size(actual),[6 N]), actual=actual.'; end
assert(isequal(size(actual),[N 6]), ...
    'Unexpected six-axis output dimensions.');
assert(numel(tout)==N && max(abs(tout-t))<1e-10, ...
    'Output sample times do not match 500 Hz input grid.');

error=actual-reference;
maxAxisError=max(abs(error),[],1);
maxError=max(maxAxisError);
tolerance=1e-10;
assert(all(isfinite(actual(:))));
assert(maxError<tolerance, ...
    'Simulink/MATLAB Tustin mismatch; max error = %.3g',maxError);

outputDir=fullfile(folder,'results');
if ~isfolder(outputDir), mkdir(outputDir); end
outTable=table(t,'VariableNames',{'time_s'});
axes={'vx_mps','vy_mps','vz_mps','wx_radps','wy_radps','wz_radps'};
for i=1:6
    outTable.(['sim_' axes{i}])=actual(:,i);
    outTable.(['ref_' axes{i}])=reference(:,i);
    outTable.(['error_' axes{i}])=error(:,i);
end
writetable(outTable,fullfile(outputDir,'admittance_simulink_comparison.csv'));
summary=struct('max_error',maxError, ...
    'max_per_axis',maxAxisError, ...
    'samples',N,'Ts',p.Ts,'pass',true);
save(fullfile(outputDir,'admittance_simulink_verification.mat'), ...
    'summary','p','t','W','actual','reference');
fprintf('PASS: Simulink agrees with six-axis Tustin MATLAB reference.\n');
fprintf('Max absolute error: %.3g (limit %.1e)\n',maxError,tolerance);
end
