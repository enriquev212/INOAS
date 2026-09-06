"""Exporta a CSV todo lo que se dibuja en el paper.

Las campanas se corren en MATLAB y dejan .mat en study/out. Las figuras se
dibujan en Python. En medio va esto: un unico punto donde se leen los .mat y se
escriben tablas planas en study/pyfigs/data. Los scripts de figuras no vuelven a
tocar un .mat, de modo que cualquiera puede reproducir o auditar una figura con
las tablas y sin MATLAB, y los CSV se pueden adjuntar como material suplementario.

    python export_data.py

Todas las unidades quedan explicitas en el nombre de la columna.
"""
import json
import os

import numpy as np
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), 'out')       # study/out, los .mat
DATA = os.path.join(HERE, 'data')                      # study/pyfigs/data, los CSV
os.makedirs(DATA, exist_ok=True)

META = {}


def load(name):
    return sio.loadmat(os.path.join(OUT, name), squeeze_me=True, struct_as_record=False)


def write_csv(name, header, rows, note=''):
    path = os.path.join(DATA, name)
    with open(path, 'w', encoding='utf-8', newline='') as fh:
        if note:
            fh.write('# ' + note + '\n')
        fh.write(','.join(header) + '\n')
        for r in rows:
            fh.write(','.join('' if v is None else
                              ('%.10g' % v if isinstance(v, (int, float, np.floating, np.integer))
                               else str(v)) for v in r) + '\n')
    print('  %-34s %6d filas' % (name, len(rows)))


def cells(x):
    """Un cell array de MATLAB como lista, tolerando el caso de un solo elemento."""
    if isinstance(x, np.ndarray) and x.dtype == object:
        return list(x.ravel())
    return [x]


# --------------------------------------------------------------------------
# 1. Monte Carlo de la maniobra: 1500 geometrias x 6 sigma_nav x 4 sigma_obj
# --------------------------------------------------------------------------
print('Monte Carlo de conjunciones')
rows = []
k = 1
while os.path.exists(os.path.join(OUT, 'mc_cam_%02d.mat' % k)):
    d = load('mc_cam_%02d.mat' % k)
    for m in cells(d['M']):
        rows.append([
            float(m.sig), float(m.sig_obj), float(m.dInc), float(m.vrel),
            float(m.bang), float(m.miss0), float(m.sens), float(m.dtarget),
            float(m.dv), int(m.ok),
        ])
    META['d_safe0_m'] = float(d['D_SAFE0'])
    META['k_sigma'] = float(d['K_SIGMA'])
    META['T_orb_s'] = float(d['T_orb'])
    META['t_burn_s'] = float(d['T_BURN'])
    k += 1
write_csv('mc_cases.csv',
          ['sigma_nav_m', 'sigma_obj_m', 'dInc_deg', 'vrel_ms', 'bplane_angle_deg',
           'miss0_m', 'sens_m_per_mms', 'd_target_m', 'dv_ms', 'converged'],
          rows,
          note='Un caso por fila. dv_ms es el impulso tangencial planificado (signo = sentido); '
               'sens_m_per_mms es la separacion abierta en el encuentro por mm/s aplicado un '
               'medio periodo antes.')

# Mediana por celda, tal y como la calculo la campana
d = load('mc_surface.mat')
SN, SO, MED = np.atleast_1d(d['SN']), np.atleast_1d(d['SO']), np.atleast_2d(d['MED'])
write_csv('mc_surface.csv', ['sigma_nav_m', 'sigma_obj_m', 'median_dv_mms'],
          [[float(SN[j]), float(SO[i]), float(MED[i, j])]
           for i in range(len(SO)) for j in range(len(SN))],
          note='Mediana de |dv| sobre las %d geometrias de cada celda.' % int(d['nGeom']))
META['n_geometries'] = int(d['nGeom'])

# Percentiles, calculados aqui a partir de las mismas filas que van a
# mc_cases.csv y no leidos de mc_summary.mat.
#
# mc_summary.mat es de una campana anterior: lleva fecha ANTERIOR a la de los
# mc_cam_*.mat que deberia resumir, y su fzero de 1/1500 delata que se calculo
# sobre 1500 casos por sigma, es decir con un solo nivel de sigma_obj, cuando los
# ficheros actuales traen 6000 (cuatro niveles). Sus percentiles no se pueden
# reconciliar con los datos actuales, asi que se recalculan.
arr = np.asarray(rows, dtype=float)
c_sn, c_so, c_dv = 0, 1, 8
DV = np.abs(arr[:, c_dv]) * 1e3          # mm/s
SN_L = np.unique(arr[:, c_sn])
SO_L = np.unique(arr[:, c_so])


def _pct(v):
    q = np.percentile(v, [10, 50, 90])
    return [q[0], q[1], q[2], float(np.mean(v)), float(np.mean(v < 1e-6))]


# Por celda: sin ninguna hipotesis sobre como se reparten las efemerides
write_csv('mc_percentiles.csv',
          ['sigma_nav_m', 'sigma_obj_m', 'n', 'p10_dv_mms', 'p50_dv_mms',
           'p90_dv_mms', 'mean_dv_mms', 'frac_no_manoeuvre'],
          [[sn_, so_, int(np.sum(m))] + _pct(DV[m])
           for sn_ in SN_L for so_ in SO_L
           for m in [(arr[:, c_sn] == sn_) & (arr[:, c_so] == so_)]],
          note='Percentiles de |dv| por celda (sigma_nav, sigma_obj), sobre las 1500 '
               'geometrias de cada una.')

# Marginal: hace falta un peso por calidad de efemeride, y aqui se toma el
# uniforme sobre los cuatro niveles. Es una hipotesis, no un dato, y por eso va
# escrita en la cabecera del fichero.
write_csv('mc_marginal.csv',
          ['sigma_nav_m', 'n', 'p10_dv_mms', 'p50_dv_mms', 'p90_dv_mms',
           'mean_dv_mms', 'frac_no_manoeuvre'],
          [[sn_, int(np.sum(m))] + _pct(DV[m])
           for sn_ in SN_L for m in [arr[:, c_sn] == sn_]],
          note='Marginalizado sobre sigma_obj con PESO UNIFORME en los cuatro niveles '
               '(10, 30, 100, 300 m): es una hipotesis sobre la calidad de las '
               'efemerides del catalogo, no una medida. Por celda, sin esa hipotesis, '
               'esta mc_percentiles.csv. frac_no_manoeuvre es la fraccion de '
               'geometrias que ya cumplen la distancia segura sin maniobrar.')

# --------------------------------------------------------------------------
# 2. Zona de exclusion inflada: por paso frente a van Loan
# --------------------------------------------------------------------------
print('Perfiles de zona de exclusion')
# Dos configuraciones, generadas por make_koz_profiles.m. La de vuelo lleva las
# constantes que corren a bordo; la otra quita el techo de sigma para que se vea
# acumular el ruido de proceso. La version anterior mezclaba las dos: generaba
# con k = 0.2 y sin techo, y rotulaba k = 3.
for variante in ('flight', 'uncapped'):
    d = load('koz_profiles_%s.mat' % variante)
    rows = []
    for i, (tt, lg, vl) in enumerate(zip(cells(d['tt']), cells(d['legacy']),
                                         cells(d['vanloan']))):
        tt, lg, vl = np.atleast_1d(tt), np.atleast_1d(lg), np.atleast_1d(vl)
        h = float(np.median(np.diff(tt))) if tt.size > 1 else float('nan')
        for t, a, b in zip(tt, lg, vl):
            rows.append([i + 1, h, float(t), float(a), float(b)])
    write_csv('koz_profiles_%s.csv' % variante,
              ['case', 'step_h_s', 't_into_horizon_s', 'radius_perstep_m',
               'radius_vanloan_m'], rows,
              note='Radio de exclusion inflado d0 + k*sigma a lo largo del horizonte, con '
                   'la covarianza propagada de dos maneras, para tres particiones del mismo '
                   'horizonte de 500 s. Configuracion %s: sigma_nav_max = %g m, k = %g.'
                   % (variante, float(d['cap']), float(d['k'])))
    META['koz_%s' % variante] = {'sigma_nav_max_m': float(d['cap']),
                                 'k_sigma': float(d['k'])}

# --------------------------------------------------------------------------
# 3. Coste de resolver: restriccion dentro del QP frente a planificar y seguir
# --------------------------------------------------------------------------
print('Tiempos de resolucion')
d = load('bench_consec_quadprog_final.mat')
res, samples = [], []
for f in cells(d['filas']):
    res.append([int(f.h), int(f.Np), int(f.n), float(f.med), float(f.p90), float(f.p99),
                float(f.worst), float(f.ok)])
    for t in np.atleast_1d(f.ts):
        samples.append([int(f.h), int(f.Np), float(t)])
write_csv('timing_qp_cases.csv',
          ['step_h_s', 'Np', 'n_solves', 'median_s', 'p90_s', 'p99_s', 'worst_s',
           'pct_converged'], res,
          note='Restriccion de colision dentro del QP. El plazo de tiempo real es h. '
               'pct_converged es 100*mean(exitflag>0), es decir cuantos QP resolvio el '
               'solver, no cuantos llegaron a tiempo: eso se cuenta sobre '
               'timing_qp_samples.csv.')
write_csv('timing_qp_samples.csv', ['step_h_s', 'Np', 'solve_time_s'], samples)

d = load('primary_timing.mat')
Dg = d['Dg']
write_csv('timing_plan_track.csv',
          ['t_s', 'solve_time_s', 'u_norm_ms2', 'violation_ms2', 'exitflag', 'qp_fallback'],
          [[float(a), float(b), float(c), float(e), int(g), int(h)] for a, b, c, e, g, h in
           zip(Dg.t, Dg.solve_time, Dg.u_norm, Dg.violation, Dg.exitflag, Dg.qp_fallback)],
          note='Arquitectura propuesta: el guiado planifica la maniobra fuera del QP y el MPC '
               'solo sigue la referencia. Plazo de tiempo real 30 s.')

# --------------------------------------------------------------------------
# 4. Ejecucion de la maniobra
# --------------------------------------------------------------------------
print('Ejecucion de la maniobra')
d = load('test_cam_retarget.mat')
P, SC, Tf = d['PLAN'], d['SC'], d['Tf']
write_csv('execution.csv',
          ['t_rel_tca_min', 't_s', 'u_norm_um_s2', 'tracking_error_m'],
          [[(float(t) - float(SC.t_tca)) / 60.0, float(t), float(u) * 1e6, float(e)]
           for t, u, e in zip(d['tv'], d['un'], d['err'])])
META['execution'] = {
    'dv_planned_mms': abs(float(P.dv)) * 1e3,
    'dv_spent_mms': float(d['dv_cum']) * 1e3,
    'miss_before_m': float(P.miss_before),
    'miss_after_m': float(P.miss_after),
    'd_target_m': float(d['d_target']),
    'miss_achieved_m': float(Tf.miss),
    'sigma_bplane_m': float(d['sigma_b']),
    't_burn_rel_tca_min': (float(P.t_burn) - float(SC.t_tca)) / 60.0,
    'retrograde': bool(P.retrograde),
    'v_rel_ms': float(SC.v_rel_mag),
    'dInc_deg': float(SC.dInc_deg),
    'pass_time_s': float(SC.pass_time),
    'guidance_step_s': float(d['H_MPC']),
    'Np': int(d['NP_MPC']),
    'u_max_ms2': 1.0 / 24.0,
}

# --------------------------------------------------------------------------
# 5. Navegacion GNSS a ciclo de trabajo
# --------------------------------------------------------------------------
print('Cadena de navegacion')
d = load('nav_chain.mat')
NAV = d['NAV']
write_csv('nav_sigma.csv', ['t_s', 'sigma_nav_m', 'time_since_fix_s'],
          [[float(a), float(b), float(c)] for a, b, c in zip(NAV.t, NAV.sigma, NAV.gap)],
          note='Serie temporal de la incertidumbre de navegacion del filtro.')
edges = np.atleast_1d(NAV.edges).astype(float)
ctr = edges[:-1] + np.diff(edges) / 2
write_csv('nav_law.csv',
          ['bin_lo_s', 'bin_hi_s', 'bin_centre_s', 'sigma_nav_m', 'count'],
          [[edges[i], edges[i + 1], ctr[i], float(np.atleast_1d(NAV.law)[i]),
            int(np.atleast_1d(NAV.cnt)[i])] for i in range(len(ctr))
           if int(np.atleast_1d(NAV.cnt)[i]) > 0],
          note='MEDIANA de sigma en cada intervalo de tiempo transcurrido desde la ultima '
               'medida (run_navigation_chain.m usa median, no mean).')
write_csv('nav_duty.csv', ['t_s', 'gnss_on'],
          [[float(a), int(b)] for a, b in zip(NAV.t_lambda, NAV.lam if hasattr(NAV, 'lam')
                                              else getattr(NAV, 'lambda'))],
          note='1 = receptor encendido. La ley de ciclo de trabajo es la variable de decision.')
META['nav'] = {'on_fraction': float(NAV.onFrac),
               'sigma_fresh_m': float(np.atleast_1d(NAV.law)[0]),
               'sigma_saturated_m': float(np.max(NAV.sigma)),
               'window_s': float(NAV.tf)}

# --------------------------------------------------------------------------
# 6. Sensibilidad del replanteo a la incertidumbre
# --------------------------------------------------------------------------
print('Barrido de replanteo')
d = load('cam_retarget_sweep.mat')
write_csv('retarget_sweep.csv',
          ['sigma_nav_m', 'sigma_combined_m', 'd_target_m', 'dv_ms'],
          [[float(s.sig), float(s.sig_comb), float(s.dt), float(s.dv)]
           for s in cells(d['SW'])],
          note='La distancia segura exigida crece con la covarianza combinada nave-objeto, '
               'y con ella el impulso necesario.')

# --------------------------------------------------------------------------
# 7. Dilucion de la probabilidad y antelacion
# --------------------------------------------------------------------------
print('Probabilidad de colision')
d = load('cam_demo.mat')
# cam_demo.m monta la covarianza como diag([s s s].^2), asi que su 'sigma' es POR
# EJE, mientras que el resto del estudio (mc_cam, la cadena de navegacion, la
# figura 1) usa el radio 3D sqrt(trace(P)). Son la misma magnitud a un factor
# sqrt(3), y mezclarlas hacia que la figura 8 sombrease la banda 4.8-32.1 m
# (radio 3D) sobre un eje graduado por eje. Se exportan las dos columnas y el
# convenio queda escrito.
write_csv('pc_nav_sweep.csv',
          ['sigma_nav_per_axis_m', 'sigma_nav_3d_m', 'miss_required_m', 'dv_ms',
           'Pc_before', 'Pc_after'],
          [[float(s.sig), float(s.sig) * np.sqrt(3.0), float(s.miss_req),
            float(s.dv), float(s.Pc0), float(s.Pc1)]
           for s in cells(d['nav_sweep'])],
          note='Pc con la formulacion de Foster. Pc_before decrece con sigma por dilucion: '
               'una covarianza mayor reparte la masa de probabilidad. sigma_nav_3d_m = '
               'sqrt(3)*sigma_nav_per_axis_m es el convenio que usan las figuras 1, 5, 6 '
               'y 7; dv_ms es una cota SUPERIOR, porque cam_demo.m limita la busqueda a '
               'la rama posigrada.')
write_csv('pc_lead_sweep.csv', ['lead_time_s', 'dv_required_ms', 'miss_m', 'Pc'],
          [[float(s.lead), float(s.dv_req), float(s.miss), float(s.Pc)]
           for s in cells(d['sweep'])],
          note='Impulso necesario segun la antelacion con que se aplica.')

with open(os.path.join(DATA, 'meta.json'), 'w', encoding='utf-8') as fh:
    json.dump(META, fh, indent=2, sort_keys=True)
print('  %-34s' % 'meta.json')
print('listo ->', DATA)
