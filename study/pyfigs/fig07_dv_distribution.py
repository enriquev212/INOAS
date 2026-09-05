"""Figura 7: el presupuesto de delta-v es una cola, no una mediana.

Un presupuesto de propulsante no se dimensiona con el caso tipico sino con el
percentil alto, asi que la figura ensena la dispersion completa sobre las 1500
geometrias de conjuncion y no solo tres lineas de percentil.

(a) Distribucion acumulada empirica del impulso en el punto de operacion medido
    (sigma_nav = 32.1 m, GNSS saturado), una curva por calidad de efemeride. Se
    condiciona en sigma_obj porque es esa variable, y no la navegacion, la que
    abre la cola: la ordenada en |dv| = 0 es la fraccion de geometrias que ya
    cumplen la distancia segura sin maniobrar.
(b) El presupuesto propiamente dicho: mediana y banda p10-p90 marginalizadas
    sobre la calidad de efemeride, frente a la incertidumbre de navegacion. La
    banda gris vertical es el rango que produce el ciclo de trabajo del receptor.

El motivo de separar los dos paneles es que el panel (a) esta condicionado a un
sigma_obj concreto y el (b) esta marginalizado sobre el. Marginalizar exige un
peso por calidad de efemeride, y el que se toma es el uniforme sobre los cuatro
niveles simulados: es una hipotesis y no una medida, asi que el rotulo del eje lo
dice y la banda del (b) no se puede leer como una curva mas del (a).

El p10 marginal es 0 hasta sigma_nav = 40 m porque entre el 14 y el 22 % de las
geometrias ya cumplen la distancia segura sin maniobrar. Que la banda toque el
suelo es el dato, no un fallo de dibujo.
"""
import matplotlib.patheffects as pe
import numpy as np

import inoas_style as st

st.use()
D = st.read_csv('mc_cases.csv')
G = st.read_csv('mc_marginal.csv')
M = st.meta()

DV = np.abs(D['dv_ms']) * 1000.0          # impulso en mm/s, sin signo
SN, SO = D['sigma_nav_m'], D['sigma_obj_m']
SIG_FRESH = M['nav']['sigma_fresh_m']
SIG_SAT = M['nav']['sigma_saturated_m']

SO_LEVELS = [10.0, 30.0, 100.0, 300.0]
SN_OP = 32.1                               # nivel tabulado del punto de operacion
XMAX = 150.0

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.45, ncols=2,
                            gridspec_kw={'width_ratios': [1.12, 1], 'wspace': 0.24})

# ---------------------------------------------------------------- panel (a)
# ECDF con el escalon inicial explicito: el salto en x = 0 es la masa de
# geometrias que no necesitan maniobra, que es informacion de mision.
# Cuatro trazos distintos ademas de cuatro colores: en gris el rojo, el verde y
# el azul de la paleta caen casi en la misma luminancia, y la figura tiene que
# seguir leyendose impresa en blanco y negro.
ECDF_DASH = [(None, None), (4.0, 1.5), (1.8, 1.4), (6.0, 1.5, 1.6, 1.5)]

stats_a = []
for i, so in enumerate(SO_LEVELS):
    d = np.sort(DV[(SN == SN_OP) & (SO == so)])
    n = d.size
    y = np.arange(1, n + 1) / n * 100.0
    axA.step(np.concatenate(([0.0], d)), np.concatenate(([0.0], y)),
             where='post', color=st.CYCLE[i], lw=1.15, zorder=3 + i,
             dashes=ECDF_DASH[i], label=r'%g m' % so)
    stats_a.append((so, n, (d == 0).mean() * 100.0, np.median(d),
                    np.percentile(d, 90), d.mean(), d.max()))

# La linea del 90 % es la que se lee para presupuestar; se marca donde la cruza
# cada poblacion para que el p90 sea un punto y no una lectura mental.
axA.axhline(90.0, color=st.C['grey'], lw=0.6, ls=(0, (2.4, 1.8)), zorder=2)
for i, (so, n, fz, m50, m90, mu, mx) in enumerate(stats_a):
    axA.plot([m90], [90.0], marker='o', markersize=3.4, color=st.CYCLE[i],
             markeredgecolor='white', zorder=7)
st.label_line(axA, 2.0, 91.5, r'$p_{90}$', st.C['grey'],
              ha='left', va='bottom', fontsize=6.8)

axA.set_xlim(0, XMAX)
axA.set_ylim(0, 101)
axA.set_xticks([0, 30, 60, 90, 120, 150])
axA.set_yticks([0, 25, 50, 75, 100])
axA.set_xlabel(r'Planned impulse magnitude, $|\Delta v|$  [mm s$^{-1}$]')
axA.set_ylabel('Cumulative fraction of\nthe 1500 geometries  [%]')

# La zona bajo la curva de 300 m esta vacia: es el unico sitio donde caben las
# notas sin cruzar ninguna curva ni la leyenda. Tres lineas y no cuatro, para
# que el bloque quede por debajo de la rejilla del 25 % y no la parta.
frac_in = (np.sort(DV[(SN == SN_OP) & (SO == SO_LEVELS[-1])]) <= XMAX).mean() * 100.0
tA = axA.text(XMAX * 0.99, 24,
              r'$\sigma_{\mathrm{nav}} = 32.1$ m (saturated GNSS)' '\n'
              r'no manoeuvre needed: %.0f %% at $\sigma_{\mathrm{obj}} = 10$ m,'
              r' %.0f %% at 30 m' '\n'
              r'300 m curve leaves the frame at %.1f %%, max %.0f mm s$^{-1}$' % (
                  stats_a[0][2], stats_a[1][2], frac_in, stats_a[3][6]),
              fontsize=6.8, color=st.C['ink'], ha='right', va='top',
              linespacing=1.4, zorder=7)
tA.set_path_effects([pe.withStroke(linewidth=2.2, foreground='white')])

# Parche blanco opaco bajo la leyenda: sin el, la rejilla del 50 % atraviesa la
# entrada de 30 m. El parche no se ve sobre el fondo blanco del panel.
lg = st.finish(axA, grid='y', legend=True, loc='center right',
               title=r'$\sigma_{\mathrm{obj}}$', borderaxespad=0.0, borderpad=0.3,
               handlelength=2.2, frameon=True, facecolor='white',
               edgecolor='none', framealpha=1.0)
lg.get_title().set_fontsize(7)
lg.get_frame().set_linewidth(0)
st.panel_tag(axA, '(a)', x=-0.135)

# ---------------------------------------------------------------- panel (b)
x = G['sigma_nav_m']
o = np.argsort(x)
x, p10, p50, p90 = x[o], G['p10_dv_mms'][o], G['p50_dv_mms'][o], G['p90_dv_mms'][o]
mean = G['mean_dv_mms'][o]

axB.axvspan(SIG_FRESH, SIG_SAT, color=st.C['vlgrey'], zorder=0, lw=0)
axB.fill_between(x, p10, p90, color=st.C['blue'], alpha=0.16, lw=0, zorder=2)
axB.plot(x, p90, color=st.C['blue'], lw=0.8, zorder=3)
axB.plot(x, p10, color=st.C['blue'], lw=0.8, zorder=3)
axB.plot(x, mean, color=st.C['orange'], lw=1.0, ls=(0, (3.2, 1.6)), zorder=4)
axB.plot(x, p50, color=st.C['ink'], lw=1.2, marker='o', markersize=3.0,
         markerfacecolor='white', markeredgecolor=st.C['ink'], zorder=5)

i_op = int(np.argmin(np.abs(x - SIG_SAT)))
axB.plot([x[i_op]] * 2, [0, p90[i_op]], color=st.C['red'], lw=0.7,
         ls=(0, (1.6, 1.6)), zorder=3)
axB.plot([x[i_op]], [p90[i_op]], marker='D', markersize=3.6,
         color=st.C['red'], markeredgecolor='white', zorder=6)
axB.plot([x[i_op]], [p50[i_op]], marker='D', markersize=3.6,
         color=st.C['red'], markeredgecolor='white', zorder=6)

axB.set_xscale('log')
axB.set_xlim(4, 300)
axB.set_ylim(0, 112)
axB.set_xticks([5, 10, 30, 100, 300])
axB.set_yticks([0, 25, 50, 75, 100])
st.plain_log(axB, 'x')
axB.set_xlabel(r'Navigation uncertainty at the encounter, $\sigma_{\mathrm{nav}}$  [m]')
# La poblacion del panel (b) no es la del (a): aqui esta marginalizada sobre la
# calidad de efemeride, y decirlo en el rotulo del eje evita que se lea la
# mediana del (a) en el (b).
# Las unidades van en la misma linea que dv: partir el rotulo justo delante de
# ellas dejaria leer "sigma_obj [mm/s]", que es dimensionalmente falso.
axB.set_ylabel(r'Manoeuvre  $\Delta v$  [mm s$^{-1}$],' + '\n'
               + r'marginal over $\sigma_{\mathrm{obj}}$')

# Sin halo: estos cuatro rotulos caen sobre la banda gris o sobre el relleno de
# la banda p10-p90, y ahi el halo blanco se ve como un borde sucio.
# El hueco entre el p90 (53 mm/s) y la nota del presupuesto esta vacio en toda
# la banda; abajo ya no cabe, porque el p10 marginal es cero.
st.label_line(axB, np.sqrt(SIG_FRESH * SIG_SAT), 80, 'duty-cycled GNSS',
              st.C['grey'], ha='center', va='center', fontsize=6.8, halo=False)
st.label_line(axB, 168, 78, r'$p_{10}\!-\!p_{90}$', st.C['blue'],
              ha='center', va='center', fontsize=7, halo=False)
st.label_line(axB, 6.2, 24.5, 'mean', st.C['orange'],
              ha='left', va='bottom', fontsize=7, halo=False)
st.label_line(axB, 6.2, 3.5, 'median', st.C['ink'],
              ha='left', va='bottom', fontsize=7, halo=False)

# Sin halo: aqui no cruza ninguna rejilla y el halo se veria sobre el gris.
axB.text(4.35, 110,
         r'budget at $\sigma_{\mathrm{nav}} = 32.1$ m:  $p_{90} = %.1f$ mm s$^{-1}$'
         '\n' r'median %.1f,  mean %.1f mm s$^{-1}$'
         % (p90[i_op], p50[i_op], mean[i_op]),
         fontsize=6.8, color=st.C['ink'], ha='left', va='top',
         linespacing=1.4, zorder=7)

st.finish(axB, grid='y')
st.panel_tag(axB, '(b)', x=-0.20)

st.save(fig, 'fig07_dv_distribution')

# ---------------------------------------------------------------- numeros
print('  (a) ECDF at sigma_nav = %.1f m, n = %d geometries per curve' % (SN_OP, stats_a[0][1]))
for so, n, fz, m50, m90, mu, mx in stats_a:
    print('      sigma_obj = %3.0f m : no-manoeuvre %5.1f %%  p50 %6.2f  p90 %6.2f'
          '  mean %6.2f  max %6.2f mm/s' % (so, fz, m50, m90, mu, mx))
print('      frame cut at %.0f mm/s: the 300 m curve leaves it at %.1f %%' % (XMAX, frac_in))
print('  (b) marginal over sigma_obj')
for xi, a, b, c, d in zip(x, p10, p50, p90, mean):
    print('      sigma_nav = %5.1f m : p10 %5.2f  p50 %5.2f  p90 %6.2f  mean %5.2f mm/s'
          % (xi, a, b, c, d))
print('      operating point sigma_nav = %.1f m : p90 = %.1f mm/s, median = %.1f,'
      ' mean = %.1f' % (x[i_op], p90[i_op], p50[i_op], mean[i_op]))
print('      median %.1f -> %.1f mm/s and p90 %.1f -> %.1f mm/s across the range'
      % (p50[0], p50[-1], p90[0], p90[-1]))
