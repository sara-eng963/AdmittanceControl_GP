function results = run_ft_laboratory(outputDir)
% Phase 1: synthetic-only six-axis F/T data generation and filtering.
if nargin < 1 || isempty(outputDir)
    outputDir = fullfile(fileparts(mfilename('fullpath')),'results');
end
cfg = ft_lab_config();
[t,Wtrue] = ft_generate_wrench(cfg);
Wmeas = ft_sensor_model(t,Wtrue,cfg);
filtered = ft_compare_filters(Wmeas,cfg);
metrics = ft_lab_metrics(t,Wtrue,filtered,cfg);
if ~isfolder(outputDir), mkdir(outputDir); end
names = {'Fx_N','Fy_N','Fz_N','Tx_Nm','Ty_Nm','Tz_Nm'};
data = table(t,'VariableNames',{'time_s'});
for i=1:6
    data.(['true_' names{i}]) = Wtrue(:,i);
    data.(['measured_' names{i}]) = Wmeas(:,i);
    data.(['ema_' names{i}]) = filtered.ema(:,i);
    data.(['butter_' names{i}]) = filtered.butterworth(:,i);
end
writetable(data,fullfile(outputDir,'signals.csv'));
writetable(metrics,fullfile(outputDir,'metrics.csv'));
save(fullfile(outputDir,'experiment.mat'), ...
    'cfg','t','Wtrue','Wmeas','filtered','metrics');
f = figure('Name','Synthetic F/T X-axis filtering');
subplot(2,1,1);
plot(t,Wtrue(:,1),'k',t,Wmeas(:,1),'Color',[0.7 0.7 0.7]); hold on;
plot(t,filtered.ema(:,1),'b',t,filtered.butterworth(:,1),'r');
ylabel('F_x (N)'); legend('True','Measured','EMA','Butterworth'); grid on;
subplot(2,1,2);
plot(t,filtered.ema(:,1)-Wtrue(:,1),'b', ...
    t,filtered.butterworth(:,1)-Wtrue(:,1),'r');
xlabel('Time (s)'); ylabel('F_x error (N)'); grid on;
exportgraphics(f,fullfile(outputDir,'fx_filter_comparison.png'));
disp(metrics);
results = struct('config',cfg,'time_s',t,'true',Wtrue, ...
    'measured',Wmeas,'filtered',filtered,'metrics',metrics);
end
