"""Figura 1: el acoplamiento navegacion-guiado esta condicionado por la efemeride.

(a) Mediana del impulso necesario frente a la incertidumbre de navegacion, una
    curva por calidad de efemeride del objeto. La banda gris es el rango que
    realmente produce la ley de ciclo de trabajo medida en la figura 5.
(b) Lo mismo leido como sensibilidad: cuanto crece la mediana al recorrer esa
    banda. Tres ordenes de magnitud entre una efemeride de 10 m y una de 300 m.

El mensaje es que apretar la navegacion solo compra algo cuando la efemeride del
objeto es mejor que la propia banda; con una efemeride de catalogo tipica el
guiado no distingue si el receptor GNSS estuvo encendido el 20 % o el 100 % del
tiempo.
"""
import numpy as np

import inoas_style as st

st.use()
S = st.read_csv('mc_surface.csv')
M = st.meta()

SO = np.unique(S['sigma_obj_m'])
SN = np.unique(S['sigma_nav_m'])
SIG_FRESH = M['nav']['sigma_fresh_m']
SIG_SAT = M['nav']['sigma_saturated_m']

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.35, ncols=2,
                            gridspec_kw={'width_ratios': [1.45, 1], 'wspace': 0.30})

# ---------------------------------------------------------------- panel (a)
axA.axvspan(SIG_FRESH, SIG_SAT, color=st.C['vlgrey'], zorder=0, lw=0)
for i, so in enumerate(SO):
    m = S['sigma_obj_m'] == so
    x, y = S['sigma_nav_m'][m], S['median_dv_mms'][m]
    o = np.argsort(x)
    axA.plot(x[o], y[o], color=st.CYCLE[i], marker=st.MARKERS[i],
             markerfacecolor=st.CYCLE[i], markeredgecolor='white',
             label=r'$\sigma_{\mathrm{obj}} = %g$ m' % so, zorder=3 + i)

axA.set_xscale('log')
axA.set_xlim(4, 300)
axA.set_ylim(0, 58)
axA.set_xticks([5, 10, 30, 100, 300])
st.plain_log(axA, 'x')
axA.set_xlabel(r'Navigation uncertainty at the encounter, $\sigma_{\mathrm{nav}}$  [m]')
axA.set_ylabel(r'Median manoeuvre  $\Delta v$  [mm s$^{-1}$]')
# El hueco entre la curva de 100 m y la de 300 m esta vacio en todo el rango, y
# la leyenda a dos columnas deja libre esa franja.
st.label_line(axA, np.sqrt(SIG_FRESH * SIG_SAT), 34.0,
              'measured range of the' + '\n' + 'duty-cycled GNSS law',
              st.C['grey'], ha='center', va='center', fontsize=6.8,
              linespacing=1.3)
st.finish(axA, legend=True, loc='upper left', ncol=2)
st.panel_tag(axA, '(a)', x=-0.13)

# ---------------------------------------------------------------- panel (b)
rel = []
for so in SO:
    m = S['sigma_obj_m'] == so
    x, y = S['sigma_nav_m'][m], S['median_dv_mms'][m]
    lo = y[np.argmin(np.abs(x - SIG_FRESH))]
    hi = y[np.argmin(np.abs(x - SIG_SAT))]
    rel.append(100.0 * (hi - lo) / lo)
rel = np.asarray(rel)

axB.plot(SO, rel, color=st.C['grey'], lw=0.9, zorder=2)
for i, (so, r) in enumerate(zip(SO, rel)):
    axB.plot([so], [r], marker=st.MARKERS[i], color=st.CYCLE[i],
             markerfacecolor=st.CYCLE[i], markeredgecolor='white',
             markersize=5, zorder=3)
    axB.annotate(('%.0f %%' if r >= 10 else '%.1f %%') % r, (so, r),
                 textcoords='offset points', xytext=(0, 7), ha='center',
                 fontsize=6.8, color=st.CYCLE[i])

axB.set_xscale('log')
axB.set_yscale('log')
axB.set_xlim(7, 560)
axB.set_ylim(0.3, 1.2e4)
axB.set_xticks([10, 30, 100, 300])
st.plain_log(axB, 'x')
axB.set_xlabel(r'Object ephemeris uncertainty, $\sigma_{\mathrm{obj}}$  [m]')
axB.set_ylabel('Increase in median $\\Delta v$ across' + '\n' + 'the measured band  [%]')
st.finish(axB)
st.panel_tag(axB, '(b)', x=-0.26)

st.save(fig, 'fig01_coupling')

for so, r in zip(SO, rel):
    print('    sigma_obj = %5.0f m  ->  %+8.1f %%' % (so, r))
print('    banda medida: sigma_nav %.1f -> %.1f m, %d geometrias por celda'
      % (SIG_FRESH, SIG_SAT, M['n_geometries']))
