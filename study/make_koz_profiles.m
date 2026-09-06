%MAKE_KOZ_PROFILES Perfil de la zona de exclusion inflada, en DOS configuraciones.
%
% La figura 2 se venia generando dentro de make_figures.m con dos constantes de
% vuelo sobrescritas a mano (sigma_nav_max = inf en vez de 32.1 m, safetyCost =
% 0.2 en vez de 3). La razon estaba en un comentario y era razonable como
% diagnostico: el techo satura las dos curvas al mismo valor y taparia el efecto
% que la figura demuestra. Pero de ahi no salia, ni al pie de figura ni al README,
% y la figura acababa rotulandose "k = 3" mientras los datos eran de k = 0.2.
%
% Aqui se generan las dos y se guardan por separado, para que la figura pueda
% enseñar la de vuelo como resultado y la de diagnostico como lo que es.
%
%   flight     : sigma_nav_max = 32.1 m, safetyCost = 3   (lo que corre a bordo)
%   uncapped   : sigma_nav_max = inf,   safetyCost = 3   (sin el techo, para ver
%                                                         acumular el ruido)
%
% Se deja safetyCost = 3 en las dos: es el k que el resto del paper declara, y
% cambiarlo solo reescala el eje.

% --- rutas derivadas de la posicion de este fichero ------------------------
STUDY_DIR = fileparts(mfilename('fullpath'));
REPO_ROOT = fileparts(STUDY_DIR);
OUT_DIR   = fullfile(STUDY_DIR, 'out');
if ~exist(OUT_DIR,'dir'); mkdir(OUT_DIR); end
addpath(REPO_ROOT); addpath(genpath(fullfile(REPO_ROOT,'matlab')));
addpath(fullfile(REPO_ROOT,'models')); addpath(STUDY_DIR);
% --------------------------------------------------------------------------

P_nav = diag([12^2 12^2 12^2 0.05^2 0.05^2 0.05^2]);
casos = {struct('h',5,'Np',100), struct('h',10,'Np',50), struct('h',4,'Np',125)};
variantes = { struct('name',"flight",   'cap', 32.1, 'k', 3), ...
              struct('name',"uncapped", 'cap', inf,  'k', 3) };

for kv = 1:numel(variantes)
    V = variantes{kv};
    prof = struct('legacy', {cell(1,3)}, 'vanloan', {cell(1,3)}, 'tt', {cell(1,3)});
    for modo = ["legacy","vanloan"]
        for kc = 1:3
            c = casos{kc};
            setpref('inoas','skipBatchClear',true);
            setpref('inoas','mpcTuneConfig', struct('h',c.h,'Np',c.Np));
            mpcQuiet = true;
            evalc("run(fullfile(REPO_ROOT,'initialize_inoas_simulation.m'))");
            mpcQuiet = true;
            mpcVanLoanQ = strcmp(modo,"vanloan");
            sigma_nav_max = V.cap;
            safetyCost    = V.k;
            dsafeSnapshotTimesMpc = 600;
            clear MPC_INOAS mpc_dsafe_snapshot_profile
            ik = min(floor(600/h)+1, Ntimesteps);
            xt = r_p_full((ik-1)*6 + (1:6)) + [30;-20;15;0.02;-0.01;0.01];
            MPC_INOAS(xt, P_nav, 600);
            pr = evalin('base','mpc_dsafe_snapshot_profile');
            prof.(modo){kc} = pr(:,end);
            prof.tt{kc} = (1:numel(pr(:,end)))' * c.h;
        end
    end
    prof.cap = V.cap; prof.k = V.k;
    outf = fullfile(OUT_DIR, sprintf('koz_profiles_%s.mat', V.name));
    save(outf, '-struct', 'prof');

    fprintf('--- %s (sigma_nav_max = %g, k = %g) ---\n', V.name, V.cap, V.k);
    for kc = 1:3
        fprintf('  h = %2g s: radio final por paso %7.1f m, van Loan %7.1f m\n', ...
            casos{kc}.h, prof.legacy{kc}(end), prof.vanloan{kc}(end));
    end
    % dispersion entre mallas en los instantes que las tres comparten
    tcom = intersect(intersect(prof.tt{1}, prof.tt{2}), prof.tt{3});
    for modo = ["legacy","vanloan"]
        sp = zeros(numel(tcom),1);
        for it = 1:numel(tcom)
            v = zeros(1,3);
            for kc = 1:3
                v(kc) = prof.(modo){kc}(prof.tt{kc} == tcom(it));
            end
            sp(it) = max(v) - min(v);
        end
        [mx, imx] = max(sp);
        fprintf('  %-8s dispersion entre mallas: max %5.1f m en t = %4.0f s, %5.1f m en t = 500 s\n', ...
            modo, mx, tcom(imx), sp(end));
    end
end
fprintf('KOZ_PROFILES_OK\n');
