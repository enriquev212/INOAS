"""Figura 1: el acoplamiento navegacion-guiado esta condicionado por la efemeride.

(a) Impulso medio necesario frente a la incertidumbre de navegacion, una curva
    por calidad de efemeride del objeto. La banda gris es el rango que produce la
    ley de ciclo de trabajo de la figura 5.
(b) Lo mismo leido como sensibilidad: cuanto crece el impulso al recorrer esa
    banda. Dos ordenes de magnitud entre una efemeride de 10 m y una de 300 m.

El mensaje es que apretar la navegacion solo compra algo cuando la efemeride del
objeto es mejor que la propia banda; con una efemeride de catalogo tipica el
guiado no distingue si el receptor GNSS estuvo encendido el 20 % o el 100 % del
tiempo.

Por que la MEDIA y no la mediana. La version anterior anunciaba +3254 % a
sigma_obj = 10 m, y ese numero no se sostiene: a sigma_nav = 4.8 m el 49.7 % de
las geometrias no necesitan maniobra, asi que la mediana (0.1163 mm/s) es
simplemente la quinta muestra no nula por encima de un atomo de masa en cero. Un
bootstrap emparejado la deja exactamente en cero en el 38.7 % de los remuestreos
y da un intervalo del 95 % de [443 %, infinito). La media no tiene ese problema,
es la magnitud que de verdad dimensiona un deposito, y da un contraste igual de
claro: +62.1 % frente a +0.5 %, un factor de 124 entre la mejor y la peor
efemeride. La mediana sigue disponible en mc_percentiles.csv.
"""
import numpy as np

import inoas_style as st

st.use()
P = st.read_csv('mc_percentiles.csv')
M = st.meta()

SO = np.unique(P['sigma_obj_m'])
SN = np.unique(P['sigma_nav_m'])
SIG_FRESH = M['nav']['sigma_fresh_m']
SIG_SAT = M['nav']['sigma_saturated_m']

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.35, ncols=2,
                            gridspec_kw={'width_ratios': [1.45, 1], 'wspace': 0.30})

# ---------------------------------------------------------------- panel (a)
axA.axvspan(SIG_FRESH, SIG_SAT, color=st.C['vlgrey'], zorder=0, lw=0)
for i, so in enumerate(SO):
    m = P['sigma_obj_m'] == so
    x, y = P['sigma_nav_m'][m], P['mean_dv_mms'][m]
    o = np.argsort(x)
    axA.plot(x[o], y[o], color=st.CYCLE[i], marker=st.MARKERS[i],
             markerfacecolor=st.CYCLE[i], markeredgecolor='white',
             label=r'$\sigma_{\mathrm{obj}} = %g$ m' % so, zorder=3 + i)

axA.set_xscale('log')
axA.set_xlim(4, 300)
axA.set_ylim(0, 78)
axA.set_xticks([5, 10, 30, 100, 300])
axA.set_yticks([0, 20, 40, 60])
st.plain_log(axA, 'x')
axA.set_xlabel(r'Navigation uncertainty at the encounter, $\sigma_{\mathrm{nav}}$  [m]')
axA.set_ylabel(r'Mean manoeuvre  $\Delta v$  [mm s$^{-1}$]')
st.label_line(axA, np.sqrt(SIG_FRESH * SIG_SAT), 44,
              'measured range of the' + '\n' + 'duty-cycled GNSS law',
              st.C['grey'], ha='center', va='center', fontsize=6.8,
              linespacing=1.3)
st.finish(axA, legend=True, loc='upper left', ncol=2)
st.panel_tag(axA, '(a)', x=-0.13)

# ---------------------------------------------------------------- panel (b)
rel, frac0 = [], []
for so in SO:
    m = P['sigma_obj_m'] == so
    x, y = P['sigma_nav_m'][m], P['mean_dv_mms'][m]
    f = P['frac_no_manoeuvre'][m]
    lo = y[np.argmin(np.abs(x - SIG_FRESH))]
    hi = y[np.argmin(np.abs(x - SIG_SAT))]
    rel.append(100.0 * (hi - lo) / lo)
    frac0.append(100.0 * f[np.argmin(np.abs(x - SIG_FRESH))])
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
axB.set_ylim(0.3, 260)
axB.set_xticks([10, 30, 100, 300])
axB.set_yticks([1, 10, 100])
st.plain_log(axB, 'x')
st.plain_log(axB, 'y')
axB.set_xlabel(r'Object ephemeris uncertainty, $\sigma_{\mathrm{obj}}$  [m]')
axB.set_ylabel('Increase in mean $\\Delta v$ across' + '\n' + 'the measured band  [%]')
st.finish(axB)
st.panel_tag(axB, '(b)', x=-0.26)

st.save(fig, 'fig01_coupling')

# ---------------------------------------------------------------- numeros
print('  banda medida: sigma_nav %.1f -> %.1f m, %d geometrias por celda'
      % (SIG_FRESH, SIG_SAT, M['n_geometries']))
for so, r, f in zip(SO, rel, frac0):
    m = P['sigma_obj_m'] == so
    x = P['sigma_nav_m'][m]
    lo = P['mean_dv_mms'][m][np.argmin(np.abs(x - SIG_FRESH))]
    hi = P['mean_dv_mms'][m][np.argmin(np.abs(x - SIG_SAT))]
    med_lo = P['p50_dv_mms'][m][np.argmin(np.abs(x - SIG_FRESH))]
    print('  sigma_obj = %5.0f m : media %6.2f -> %6.2f mm/s (%+7.1f %%)   '
          'mediana en la banda baja %6.3f mm/s, %4.1f %% sin maniobra'
          % (so, lo, hi, r, med_lo, f))
print('  contraste entre la mejor y la peor efemeride: factor %.0f'
      % (rel[0] / rel[-1]))
