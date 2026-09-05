%RUN_EXPA Pasada A: coste computacional y perfil de la keep-out zone.
% SECUENCIAL a proposito: los tiempos de solve solo son comparables si se
% miden sin contencion de CPU. Barre las 6 configuraciones en un solo proceso.
% --- rutas derivadas de la posicion de este fichero ------------------------
% El estudio se localiza a si mismo. Antes cada script llevaba la ruta absoluta
% escrita a mano, lo que impedia reproducir ninguna cifra en otra maquina.
STUDY_DIR = fileparts(mfilename('fullpath'));
REPO_ROOT = fileparts(STUDY_DIR);
OUT_DIR   = fullfile(STUDY_DIR, 'out');
FIG_DIR   = fullfile(STUDY_DIR, 'figures');
WORK_DIR  = fullfile(STUDY_DIR, 'work');
for d_ = {OUT_DIR, FIG_DIR, WORK_DIR}
    if ~exist(d_{1},'dir'); mkdir(d_{1}); end
end
addpath(REPO_ROOT); addpath(genpath(fullfile(REPO_ROOT,'matlab')));
addpath(fullfile(REPO_ROOT,'models')); addpath(STUDY_DIR);
% --------------------------------------------------------------------------

addpath(STUDY_DIR);
fprintf('MATLAB %s | maxNumCompThreads = %d\n\n', version, maxNumCompThreads);
STUDY_T0 = tic;

for STUDY_I = 1:6
    CFG_ID = STUDY_I;
    fprintf('\n========================================================\n');
    try
        run(fullfile(STUDY_DIR,'inoas_setup.m'));

        STUDY_TA = [0 300 600 700 750 800 850];
        A = struct('t',STUDY_TA,'solve',zeros(1,numel(STUDY_TA)), ...
                   'iters',zeros(1,numel(STUDY_TA)), ...
                   'unorm',zeros(1,numel(STUDY_TA)),'satfrac',zeros(1,numel(STUDY_TA)), ...
                   'slack',zeros(1,numel(STUDY_TA)),'dsafe1',zeros(1,numel(STUDY_TA)), ...
                   'dsafeN',zeros(1,numel(STUDY_TA)));
        for kk_ = 1:numel(STUDY_TA)
            t_ = STUDY_TA(kk_);
            ik_ = min(floor(t_/h)+1, Ntimesteps);
            xt_ = r_p_full((ik_-1)*6 + (1:6));
            xe_ = xt_ + [30; -20; 15; 0.02; -0.01; 0.01];
            clear MPC_INOAS;              % solve limpio, sin warm start heredado
            clear mpc_dsafe_log_time mpc_dsafe_log_first mpc_dsafe_log_max
            tt_ = tic; [uc_,~,sl_] = MPC_INOAS(xe_, STUDY_PNAV, t_); A.solve(kk_) = toc(tt_);
            A.unorm(kk_)   = norm(uc_);
            A.satfrac(kk_) = max(abs(uc_))/u_max;
            A.slack(kk_)   = max(sl_);
            A.dsafe1(kk_)  = mpc_dsafe_log_first(end);
            A.dsafeN(kk_)  = mpc_dsafe_log_max(end);
        end

        RES = struct();
        RES.name = STUDY_CFG.name; RES.h = h; RES.Np = Np; RES.Th = h*Np;
        RES.fair = STUDY_CFG.fair; RES.du_max = du_max; RES.u_max = u_max;
        RES.Q_cov44 = Q_cov_mpc(4,4); RES.slackWeight = slackWeight;
        RES.nvars = 3*Np + Np;
        RES.A = A;
        RES.solveA_mean = mean(A.solve); RES.solveA_max = max(A.solve);
        RES.rt_ratio = mean(A.solve)/h;
        RES.dsafe_first = A.dsafe1(1); RES.dsafe_horizon = A.dsafeN(1);
        % Tiempo hasta u_max limitado por du_max, en segundos fisicos
        RES.t_to_umax = ceil(u_max/du_max)*h;

        fprintf('  solve: media %.3f s | max %.3f s | presupuesto h=%g s | ratio %.2f\n', ...
            RES.solveA_mean, RES.solveA_max, h, RES.rt_ratio);
        fprintf('  dsafe: paso 1 = %.2f m | fin de horizonte = %.2f m\n', ...
            RES.dsafe_first, RES.dsafe_horizon);
        fprintf('  rampa: %d pasos x %g s = %.0f s para llegar de 0 a u_max\n', ...
            ceil(u_max/du_max), h, RES.t_to_umax);
        fprintf('  max|u|/u_max en t=800 s: %.3f | slack max: %.2e\n', ...
            A.satfrac(STUDY_TA==800), max(A.slack));

        save(fullfile(STUDY_OUT, sprintf('expA_%02d_%s.mat', CFG_ID, STUDY_CFG.name)), 'RES');
        fprintf('  EXPA_DONE %s\n', STUDY_CFG.name);
    catch ME
        fprintf('!!! CONFIG %d FALLO: %s\n', STUDY_I, ME.message);
        for s_ = 1:numel(ME.stack)
            fprintf('    en %s linea %d\n', ME.stack(s_).name, ME.stack(s_).line);
        end
    end
end
fprintf('\n\nPASADA A COMPLETA en %.1f minutos\n', toc(STUDY_T0)/60);
