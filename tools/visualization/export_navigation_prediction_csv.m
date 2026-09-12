function export_navigation_prediction_csv(outputDir, simOut, predictionLog)
%EXPORT_NAVIGATION_PREDICTION_CSV Compare forecast and realized UKF uncertainty.
% Targets after simulation StopTime remain NaN, never extrapolated.
if nargin < 3
    predictionLog = evalin('base','mpc_navigation_prediction_log');
end
if ~isfolder(outputDir)
    mkdir(outputDir);
end
logs = simOut.logsout;
[time,Prows] = signalRows(logs,'ukf_covariance',36);
[~,indices] = unique(time,'last');
time = time(indices);
Prows = Prows(indices,:);
state = alignedSignal(logs,'ukf_state',6,time);
truth = alignedSignal(logs,'truth_position_eci',3,time);
lambda = alignedSignal(logs,'lambda',1,time);
command = alignedSignal(logs,'lambda_command',1,time);
fresh = alignedSignal(logs,'gnss_update',1,time);
score = alignedSignal(logs,'NIS',1,time);
sigma = nan(size(time));
posError = vecnorm(state(:,1:3)-truth,2,2);
posNees = nan(size(time));
for k=1:numel(time)
    P = reshape(Prows(k,:),6,6);
    Ppos = (P(1:3,1:3)+P(1:3,1:3).')/2;
    sigma(k) = sqrt(max(max(eig(Ppos)),0));
    e = (state(k,1:3)-truth(k,:)).';
    if rcond(Ppos)>1e-12 && all(isfinite(e))
        posNees(k) = e.'*(Ppos\e);
    end
end
navigation = table(time,sigma,posError,posNees,lambda,command,fresh,score, ...
    'VariableNames',{'time_s','sigma_max_pos_m','position_error_m', ...
    'position_nees','lambda_effective','lambda_command','gnss_update','pseudo_nis'});
writetable(navigation,fullfile(outputDir,'navigation_uncertainty.csv'));

if isempty(predictionLog.origin_time)
    warning('INOAS:EmptyPredictionLog','No navigation forecast was logged.');
    return;
end
nNodes = size(predictionLog.target_time,1);
origin = repelem(predictionLog.origin_time,nNodes);
target = predictionLog.target_time(:);
Ppred = reshape(predictionLog.P_eci,6,6,[]);
predSigma = nan(size(target));
actualSigma = nan(size(target));
matrixError = nan(size(target));
for k=1:numel(target)
    P = Ppred(:,:,k);
    Ppos = (P(1:3,1:3)+P(1:3,1:3).')/2;
    predSigma(k) = sqrt(max(max(eig(Ppos)),0));
    [distance,j] = min(abs(time-target(k)));
    if distance <= 1e-7
        actualSigma(k) = sigma(j);
        actual = reshape(Prows(j,:),6,6);
        % Compare position blocks only: full-state entries have mixed units.
        matrixError(k) = norm(Ppos-actual(1:3,1:3),'fro') / ...
            max(norm(actual(1:3,1:3),'fro'),eps);
    end
end
updates = predictionLog.gnss_updates(:);
mode = repmat(string(predictionLog.mode),numel(target),1);
comparison = table(origin,target,target-origin,predSigma,actualSigma,matrixError,updates,mode, ...
    'VariableNames',{'origin_time_s','target_time_s','lead_time_s', ...
    'predicted_sigma_max_pos_m','realized_sigma_max_pos_m', ...
    'position_covariance_relative_error','assumed_gnss_update_count','forecast_mode'});
writetable(comparison,fullfile(outputDir,'navigation_prediction.csv'));
save(fullfile(outputDir,'navigation_prediction.mat'),'predictionLog','navigation','comparison');
fprintf('Navigation forecast and UKF comparison exported to %s\n',outputDir);
end

function [time,values] = signalRows(logs,name,width)
element = logs.getElement(name);
assert(~isempty(element),'INOAS:MissingNavigationLog','Missing logged signal %s.',name);
ts = element.Values;
time = double(ts.Time(:));
if ts.IsTimeFirst
    values = reshape(double(ts.Data),numel(time),width);
else
    values = reshape(double(ts.Data),width,numel(time)).';
end
end

function values = alignedSignal(logs,name,width,time)
[sourceTime,source] = signalRows(logs,name,width);
[sourceTime,index] = unique(sourceTime,'last');
source = source(index,:);
if numel(sourceTime)==1
    values = nan(numel(time),width);
    values(abs(time-sourceTime)<1e-7,:) = source;
else
    values = interp1(sourceTime,source,time,'linear',NaN);
end
end
