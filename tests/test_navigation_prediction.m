function tests = test_navigation_prediction
tests = functiontests(localfunctions);
end

function setupOnce(testCase)
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'matlab'));
end

function testLinearCovarianceMatchesRiccati(testCase)
[cfg,F,H,x,P] = fixture();
cfg.mode = 'scheduled';
[actual, forecast] = predictNavigationCovarianceProfile(x,P,0,(1:12).', ...
    zeros(3,1),[1;0;1],cfg);
for k = 1:12
    P = F*P*F.'+cfg.Q;
    K = (P*H.')/(H*P*H.'+cfg.Raux);
    E = eye(6)-K*H;
    P = E*P*E.'+K*cfg.Raux*K.';
    if mod(k,3)==0
        K = P/(P+cfg.Rgnss);
        E = eye(6)-K;
        P = E*P*E.'+K*cfg.Rgnss*K.';
    end
    verifyEqual(testCase,actual(:,:,k),P,'AbsTol',1e-8);
end
verifyEqual(testCase,find(forecast.gnss_update),[3;6;9;12]);
end

function testCommonTimesDoNotDependOnMpcGrid(testCase)
[cfg,~,~,x,P] = fixture();
[fine,~] = predictNavigationCovarianceProfile(x,P,0,(3:3:36).',zeros(3,1),[],cfg);
[coarse,~] = predictNavigationCovarianceProfile(x,P,0,(12:12:36).',zeros(3,1),[],cfg);
verifyEqual(testCase,coarse,fine(:,:,4:4:12),'AbsTol',1e-12);
end

function testAuxiliaryUpdatesReduceCovariance(testCase)
[cfg,~,~,x,P] = fixture();
withAux = predictNavigationCovarianceProfile(x,P,0,[3;30;60],zeros(3,1),[],cfg);
cfg.mode = 'no_measurements';
without = predictNavigationCovarianceProfile(x,P,0,[3;30;60],zeros(3,1),[],cfg);
for k=1:3
    verifyGreaterThanOrEqual(testCase,min(eig(without(:,:,k)-withAux(:,:,k))),-1e-7);
end
end

function testScheduledFreshFixAndDelayedStatus(testCase)
[cfg,~,~,x,P] = fixture();
cfg.mode = 'scheduled';
[~,forecast] = predictNavigationCovarianceProfile(x,P,122,(123:129).', ...
    zeros(3,1),[0;298;1],cfg);
verifyEqual(testCase,forecast.lambda,logical([0;1;1;1;1;1;1]));
verifyEqual(testCase,forecast.time(forecast.gnss_update),[126;129]);
end

function testNoForecastReacquisitionWhenUnhealthy(testCase)
[cfg,~,~,x,P] = fixture();
aux = predictNavigationCovarianceProfile(x,P,0,[3;6;9],zeros(3,1),[],cfg);
cfg.mode = 'scheduled';
[scheduled,forecast] = predictNavigationCovarianceProfile(x,P,0,[3;6;9], ...
    zeros(3,1),[0;299;0],cfg);
verifyEqual(testCase,scheduled,aux,'AbsTol',1e-12);
verifyFalse(testCase,any(forecast.gnss_update));
end

function testInvalidTimesAreRejected(testCase)
[cfg,~,~,x,P] = fixture();
verifyError(testCase,@() predictNavigationCovarianceProfile(x,P,0,2.5, ...
    zeros(3,1),[],cfg),'INOAS:NavigationGrid');
end

function testDutyCycleAndEmergency(testCase)
[lambda,age] = navigationDutyCycleStep(true,59,true,0,60,300,12,1);
verifyFalse(testCase,lambda);
verifyEqual(testCase,age,0);
[lambda,age] = navigationDutyCycleStep(false,299,true,0,60,300,12,1);
verifyTrue(testCase,lambda);
verifyEqual(testCase,age,0);
[lambda,~] = navigationDutyCycleStep(false,3,true,12,60,300,12,1);
verifyTrue(testCase,lambda);
[lambda,~] = navigationDutyCycleStep(false,399,false,100,60,300,12,1);
verifyFalse(testCase,lambda);
end

function testOrbitalForecastIsFiniteAndPositive(testCase)
[cfg,~,~,~,P] = fixture();
cfg.stateTransitionFcn = @myStateTransitionFcn;
cfg.measurementFcn = @myMeasurementFcn;
cfg.alpha = 1e-3;
x = [7714430;0;0;0;7200;0];
[Pnodes,~] = predictNavigationCovarianceProfile(x,P,0,[3;12;375], ...
    zeros(3,1),[],cfg);
verifyTrue(testCase,all(isfinite(Pnodes),'all'));
for k=1:3
    verifyGreaterThanOrEqual(testCase,min(eig(Pnodes(:,:,k))),-1e-8);
end
end

function [cfg,F,H,x,P] = fixture()
F = [eye(3),eye(3);zeros(3),eye(3)];
H = [eye(3),zeros(3);0,0,1,0,0,0];
x = [100;20;30;1;2;3];
P = diag([25,25,25,.01,.01,.01]);
cfg = struct('mode','aux_only','sampleTime',1, ...
    'stateTransitionFcn',@(x,u) F*x+[.5*eye(3);eye(3)]*u, ...
    'measurementFcn',@(x) H*x,'alpha',0.1,'beta',2,'kappa',0, ...
    'Q',.01*[eye(3)/3,eye(3)/2;eye(3)/2,eye(3)], ...
    'Raux',4e6*eye(4),'Rgnss',diag([25,25,25,.01,.01,.01]), ...
    'gnssSampleTime',3,'gnssEpoch',0,'onDuration',60, ...
    'offDuration',300,'scoreThreshold',12);
end
