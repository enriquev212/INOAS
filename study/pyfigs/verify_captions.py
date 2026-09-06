"""Comprueba que cada cifra de figures.tex sale de los CSV.

    python verify_captions.py

Un pie de figura es la parte del articulo que mas facilmente se queda atras: se
escribe una vez y sobrevive a varias regeneraciones de la figura. Aqui cada
numero que aparece en el pie se recalcula desde data/*.csv y se compara con la
cadena que hay escrita. Si una campana cambia y el pie no, esto lo dice.
"""
import io
import os
import sys

import numpy as np

import inoas_style as st

TEX = io.open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           'figures.tex'), encoding='utf-8').read()

M = st.meta()
P = st.read_csv('mc_percentiles.csv')
D = st.read_csv('mc_cases.csv')
KA = st.read_csv('koz_profiles_flight.csv')
KB = st.read_csv('koz_profiles_uncapped.csv')
SW = st.read_csv('nav_duty_sweep.csv')
AUX = st.read_csv('nav_aux_sensitivity.csv')
PC = st.read_csv('pc_nav_sweep.csv')
X = M['execution']

cell = (D['sigma_nav_m'] == 4.8) & (D['sigma_obj_m'] == 10)
DWELL = 2.0 * D['d_target_m'][cell] / D['vrel_ms'][cell]


def mesh_spread(K, col):
    cases = np.unique(K['case'])
    tt = [K['t_into_horizon_s'][K['case'] == c] for c in cases]
    com = np.intersect1d(np.intersect1d(tt[0], tt[1]), tt[2])
    sp = [max(K[col][(K['case'] == c) & (K['t_into_horizon_s'] == t)][0]
              for c in cases)
          - min(K[col][(K['case'] == c) & (K['t_into_horizon_s'] == t)][0]
                for c in cases) for t in com]
    return float(np.max(sp))


def rel_mean(so):
    m = P['sigma_obj_m'] == so
    x, y = P['sigma_nav_m'][m], P['mean_dv_mms'][m]
    lo = y[np.argmin(np.abs(x - M['nav']['sigma_fresh_m']))]
    hi = y[np.argmin(np.abs(x - M['nav']['sigma_saturated_m']))]
    return 100.0 * (hi - lo) / lo


i_op = int(np.argmin(np.abs(SW['duty_pct'] - 100 * M['nav']['on_fraction'])))

CHECKS = [
    # (figura, que es, valor recalculado, como aparece en el pie)
    (1, 'aumento medio a sigma_obj=10 m', rel_mean(10), '$+62\\,\\%$'),
    (1, 'aumento medio a sigma_obj=300 m', rel_mean(300), '$+0.5\\,\\%$'),
    (1, 'factor entre extremos', rel_mean(10) / rel_mean(300), 'factor of $128$'),
    (1, 'fraccion sin maniobra',
     100 * P['frac_no_manoeuvre'][(P['sigma_obj_m'] == 10) &
                                  (P['sigma_nav_m'] == 4.8)][0], '$49.7\\,\\%$'),
    (2, 'saturacion en vuelo', float(np.max(KA['radius_vanloan_m'])), '$249.7$\\,m'),
    (2, 'dispersion por paso en vuelo', mesh_spread(KA, 'radius_perstep_m'), '$17.6$\\,m'),
    (2, 'dispersion por paso sin techo', mesh_spread(KB, 'radius_perstep_m'), '$619$\\,m'),
    (3, 'permanencia mediana [ms]', 1e3 * float(np.median(DWELL)), '$34$\\,ms'),
    (3, 'razon paso/permanencia', 30.0 / float(np.median(DWELL)), '$886$ times'),
    (3, 'P(deteccion) al paso desplegado',
     100 * float(np.median(np.minimum(1, DWELL / 30.0))), '$0.11\\,\\%$'),
    (3, 'P(deteccion) a h = 3 s',
     100 * float(np.median(np.minimum(1, DWELL / 3.0))), '$1.1\\,\\%$'),
    (4, 'impulso planificado [mm/s]', X['dv_planned_mms'], '$21.8$\\,mm/s'),
    (4, 'impulso gastado [mm/s]', X['dv_spent_mms'], '$33.2$\\,mm/s'),
    (4, 'fallo logrado [m]', X['miss_achieved_m'], '$382.6$\\,m'),
    (4, 'sigma combinada [m]', X['sigma_bplane_m'], '$\\sigma_c = 77.5$\\,m'),
    (5, 'techo con receptor apagado', SW['ceiling_m'][0], '$32.1$\\,m'),
    (5, 'media al ciclo de trabajo volado', SW['mean_m'][i_op], '$24.3$\\,m'),
    (5, 'media con receptor siempre activo', SW['mean_m'][-1], '$3.7$\\,m'),
    (5, 'techo con sensor auxiliar de 300 m',
     AUX['ceiling_m'][AUX['sigma_aux_m'] == 300][0], '$72.9$\\,m'),
    (8, 'Pc mas alta', PC['Pc_before'].max(), '$8.8\\times10^{-4}$'),
    (8, 'Pc mas baja', PC['Pc_before'].min(), '$1.6\\times10^{-4}$'),
    (8, 'fallo exigido minimo [m]', PC['miss_required_m'].min(), '$333 \\rightarrow 835$'),
    (8, 'impulso minimo [mm/s]', 1e3 * np.abs(PC['dv_ms']).min(),
     '$18.2 \\rightarrow 53.5$\\,mm/s'),
]

print('%-4s %-38s %12s   %s' % ('fig', 'magnitud', 'recalculado', 'en el pie'))
print('-' * 92)
malas = 0
for fig, que, val, patron in CHECKS:
    ok = patron in TEX
    if not ok:
        malas += 1
    print('%-4d %-38s %12.4g   %s%s'
          % (fig, que, val, 'si' if ok else 'NO ENCONTRADO -> ', '' if ok else patron))

print()
if malas:
    print('%d cifras del pie no se encontraron: o el pie esta atrasado o el '
          'patron de esta comprobacion lo esta.' % malas)
    sys.exit(1)
print('%d cifras comprobadas, todas presentes en figures.tex' % len(CHECKS))
