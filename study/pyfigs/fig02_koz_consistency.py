"""Figura 2: el radio de exclusion tiene que ser funcion del tiempo, no del mallado.

El radio inflado es d0 + k*sigma(t), y sigma(t) sale de propagar la covarianza a
lo largo del horizonte. El codigo heredado sumaba un incremento de ruido de
proceso FIJO POR PASO, con lo que el resultado dependia de en cuantos trozos se
partiera el horizonte. Discretizar la densidad espectral continua sobre el paso
con van Loan lo arregla: las tres particiones colapsan en una curva, como deben,
porque el crecimiento fisico de la incertidumbre no sabe nada del mallado.

La version anterior de esta figura estaba generada con dos constantes de vuelo
sobrescritas a mano (sigma_nav_max = inf y safetyCost = 0.2) mientras se rotulaba
"k = 3". Ahora se dibujan las dos configuraciones y cada una dice cual es:

(a) Configuracion de vuelo (sigma_nav_max = 32.1 m, k = 3). El techo de sigma
    satura las dos formulaciones en 249.7 m, asi que el defecto solo se ve donde
    el techo aun no actua: 17.6 m de dispersion entre mallas a t = 60 s, y cero
    a partir de ~90 s.
(b) Sin el techo, que es donde se ve la formulacion desnuda. La dispersion entre
    mallas llega a 619 m y el esquema por paso se queda entre 1207 y 1826 m
    frente a los 3512 m consistentes: no solo depende del mallado, sino que
    infla de menos por un factor de 2 a 3.

Los dos paneles juntos son el argumento honesto: el defecto de formulacion es
grande, y lo que lo tapa en vuelo es el techo de sigma, que a su vez es una
hipotesis sobre el sensor auxiliar (figura 5) y no una propiedad del guiado.
"""
import numpy as np

import inoas_style as st

st.use()
M = st.meta()
MK = {'o': 0, 's': 1, '^': 2}

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.6, ncols=2,
                            gridspec_kw={'wspace': 0.30})


def draw(ax, variante, ylim, yticks, xlim=(0, 500)):
    K = st.read_csv('koz_profiles_%s.csv' % variante)
    cases = np.unique(K['case'])
    steps = []
    for i, c in enumerate(cases):
        m = K['case'] == c
        t = K['t_into_horizon_s'][m]
        h = float(np.median(np.diff(t)))
        steps.append(h)
        o = np.argsort(t)
        mk = st.MARKERS[i]
        # marcadores escalonados: las tres curvas van Loan son la misma
        idx = np.arange(6 + 4 * i, len(t), 12)
        ax.plot(t[o], K['radius_perstep_m'][m][o], color=st.C['red'], lw=1.0,
                marker=mk, markevery=list(idx), markersize=3.4,
                markerfacecolor='white', markeredgecolor=st.C['red'], zorder=3)
        ax.plot(t[o], K['radius_vanloan_m'][m][o], color=st.C['green'], lw=1.2,
                marker=mk, markevery=list(idx), markersize=3.4,
                markerfacecolor='white', markeredgecolor=st.C['green'], zorder=4,
                label='$h$ = %g s' % h)
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_yticks(yticks)
    ax.set_xlabel('Time into the prediction horizon  [s]')
    return K, steps


# ---------------------------------------------------------------- panel (a)
# El efecto vive en los primeros 100 s: a partir de ahi el techo satura las dos
# formulaciones y el resto del horizonte es una meseta que no aporta nada.
KA, steps = draw(axA, 'flight', (210, 258), [210, 220, 230, 240, 250],
                 xlim=(0, 150))
cap = M['koz_flight']['sigma_nav_max_m']
k = M['koz_flight']['k_sigma']
d0 = M['d_safe0_m']
# La saturacion no es d0 + k*sigma_max sino d0 + k*sqrt(sigma_max^2 + sigma_obj^2):
# el techo acota solo la parte de la nave, y la del objeto sigue sumando. Se
# rotula el valor medido y no el derivado, que se queda 3.4 m corto.
sat = float(np.max(KA['radius_vanloan_m']))
axA.axhline(sat, color=st.C['ink'], lw=0.8, ls=(0, (4, 1.6)), zorder=2)
st.label_line(axA, 78, sat + 0.9,
              r'both saturate at %.1f m, where the $\sigma$ cap binds' % sat,
              st.C['ink'], ha='center', va='bottom', fontsize=6.8)
st.label_line(axA, 96, 228, 'per-step: 17.6 m of mesh' + '\n' + 'spread at $t$ = 60 s',
              st.C['red'], ha='left', va='center', fontsize=6.8,
              linespacing=1.25)
st.label_line(axA, 148, 213, 'flat to $t$ = 500 s', st.C['grey'],
              ha='right', va='bottom', fontsize=6.6)
axA.set_ylabel('Inflated keep-out radius\n$d_0 + k\\,\\sigma(t)$  [m]')
st.finish(axA, legend=True, loc='lower left')
st.panel_tag(axA, '(a)', x=-0.19)
axA.annotate('flight: $\\sigma_{\\max}$ = %.1f m' % cap, (0.40, 0.30),
             xycoords='axes fraction', fontsize=7, color=st.C['ink'],
             va='top', ha='left')

# ---------------------------------------------------------------- panel (b)
KB, _ = draw(axB, 'uncapped', (0, 3900), [0, 1000, 2000, 3000])
axB.set_ylabel('Inflated keep-out radius\nwithout the cap  [m]')
st.label_line(axB, 250, 3300, 'van Loan: one curve', st.C['green'],
              ha='center', va='bottom', fontsize=7)
st.label_line(axB, 430, 1000, 'per-step:\n619 m apart', st.C['red'],
              ha='right', va='center', fontsize=7, linespacing=1.25)
st.finish(axB, legend=True, loc='upper left')
st.panel_tag(axB, '(b)', x=-0.19)
axB.annotate('no cap: $\\sigma_{\\max} = \\infty$', (0.03, 0.62),
             xycoords='axes fraction', fontsize=7, color=st.C['ink'],
             va='top', ha='left')

st.save(fig, 'fig02_koz_consistency')


# ---------------------------------------------------------------- numeros
def report(name, K):
    cases = np.unique(K['case'])
    tt = [K['t_into_horizon_s'][K['case'] == c] for c in cases]
    com = np.intersect1d(np.intersect1d(tt[0], tt[1]), tt[2])
    print('  %s' % name)
    for col, lab in (('radius_perstep_m', 'per-step'), ('radius_vanloan_m', 'van Loan')):
        sp = []
        for t in com:
            v = [K[col][(K['case'] == c) & (K['t_into_horizon_s'] == t)][0] for c in cases]
            sp.append(max(v) - min(v))
        sp = np.asarray(sp)
        print('     %-9s final %7.1f m   dispersion max %6.1f m en t = %3.0f s, '
              '%5.1f m en t = 500 s'
              % (lab, K[col][K['t_into_horizon_s'] == com[-1]].mean(),
                 sp.max(), com[int(np.argmax(sp))], sp[-1]))


report('vuelo   (sigma_max = %.1f m, k = %g)' % (cap, k), KA)
report('sin techo (sigma_max = inf, k = %g)' % M['koz_uncapped']['k_sigma'], KB)
print('  d0 = %.0f m, techo del radio = %.1f m' % (d0, d0 + k * cap))
