%DIAG_REFERENCE Fidelidad de la trayectoria de referencia segun el paso h.
% get_nominal_trajectory integra con RK4 al paso del MPC. Con h = 3 s eso es
% inofensivo, pero el escenario de conjuncion realista necesita h del orden de
% 60 s para que Np*h alcance una orbita, y ahi RK4 puede dejar de ser valido.
% Si la referencia no es una trayectoria fisica, el controlador quema propelente
% persiguiendo algo que no existe.
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

REPO = REPO_ROOT;
addpath(REPO); addpath(genpath(fullfile(REPO,'matlab')));
addpath(STUDY_DIR);

TF = 12000;
fine = fullfile(tempdir, 'inoas_reffine_diag.mat');
firstPass = true;

for hh = [3 10 30 60]
    setpref('inoas','skipBatchClear', true);
    setpref('inoas','mpcTuneConfig', struct('h',hh,'Np',20));
    simulationStopTime = TF;
    mpcQuiet = true;
    evalc(sprintf("run(fullfile('%s','initialize_inoas_simulation.m'))", REPO));

    if firstPass
        get_nominal_trajectory(0.25, ceil(TF/0.25)+2, fine, ...
            'a',a,'ecc',ecc,'incl',inc,'RAAN',RAAN,'argp',w,'nu',theta);
        firstPass = false;
    end
    F = load(fine, 'x_ref_hist', 't_ref');

    tp = 0:600:(TF-600);
    e = zeros(size(tp));
    for q = 1:numel(tp)
        ik = min(round(tp(q)/hh)+1, Ntimesteps);
        xr = r_p_full((ik-1)*6 + (1:6));
        xf = interp1(F.t_ref, F.x_ref_hist(1:3,:).', tp(q)).';
        e(q) = norm(xr(1:3) - xf);
    end
    fprintf('h = %2d s -> error de la referencia: rms %12.1f m, max %12.1f m\n', ...
        hh, rms(e), max(e));
end
fprintf('\nDIAG_REF_OK\n');
