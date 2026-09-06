"""Figura 5: que fija la cota de navegacion y que compra el ciclo de trabajo.

La version anterior de esta figura vendia los 32.1 m como "el coste medido de
apagar el receptor GNSS". Es falso, y la auditoria lo confirmo reconstruyendo la
recursion de covarianza: 32.1 m es el punto fijo de Riccati de un canal auxiliar
que en el modelo esta SIEMPRE activo (myMeasurementFcn entrega posicion en tres
ejes mas altitud con R = diag([100^2 100^2 100^2 50^2]) y sin puerta de
habilitacion, models/inoas_model.slx system_68.xml). La cota es exactamente la
misma con el receptor apagado del todo.

Lo que el ciclo de trabajo fija no es el techo sino la media. La figura separa
las dos cosas, que es el enunciado que el paper si puede defender:

(a) sigma frente al tiempo transcurrido desde la ultima correccion. Los puntos
    son la corrida medida, la linea es la recursion de covarianza que los
    reproduce, y la de trazos es el techo con el receptor apagado PARA SIEMPRE.
    Que la asintota coincida con esa linea es justamente el argumento.
(b) Barrido del ciclo de trabajo. El techo es plano en todo el rango; la media
    temporal cae de 32.1 a 3.7 m. Eso es lo que se compra encendiendo el
    receptor, y es un resultado distinto y mas modesto que el anterior.

La cota depende de lo que se suponga disponible mientras el GNSS esta apagado y
no del guiado: con un sensor auxiliar de 300 m por eje el techo sube a 72.9 m. El
pie de figura tiene que decirlo, porque es la primera pregunta de un revisor.

El panel de la serie temporal de la version anterior se ha retirado: repetia lo
que dice (a) y no dejaba sitio para lo que de verdad hay que demostrar.
"""
import numpy as np

import inoas_style as st
import nav_duty_model as nav

st.use()
LAW = st.read_csv('nav_law.csv')
SW = st.read_csv('nav_duty_sweep.csv')
AUX = st.read_csv('nav_aux_sensitivity.csv')
M = st.meta()

DUTY_OP = 100.0 * M['nav']['on_fraction']
CEIL = float(SW['ceiling_m'][SW['duty_pct'] == 0][0])

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.45, ncols=2,
                            gridspec_kw={'wspace': 0.30})

# ---------------------------------------------------------------- panel (a)
# Modelo con el mismo patron 60 s encendido / 300 s apagado de la corrida medida
sch = nav.duty_schedule(60, 300, 20000)
sig = nav.propagate(sch)
gap = np.zeros(len(sch))
g = 0
for k, on in enumerate(sch):
    g = 0 if on else g + 1
    gap[k] = g
half = len(sch) // 2
gg, ss = gap[half:], sig[half:]
uniq = np.unique(gg)
med = np.array([np.median(ss[gg == u]) for u in uniq])

axA.axhline(CEIL, ls=(0, (4, 1.6)), color=st.C['red'], lw=1.0, zorder=2,
            label='receiver never on')
axA.plot(uniq, med, '-', color=st.C['blue'], lw=1.2, zorder=3,
         label='covariance recursion')
# La casilla 0-30 s la llenan sobre todo muestras con el receptor ENCENDIDO
# (hueco = 0), asi que dibujarla en su centro, 15 s, la pondria donde el modelo
# ya lleva 15 s de deriva libre y fingiria un desacuerdo que no existe. Va en
# x = 0, que es lo que de verdad mide: el suelo con correccion reciente.
free = LAW['bin_lo_s'] >= 30
axA.plot(LAW['bin_centre_s'][free], LAW['sigma_nav_m'][free], 'o',
         color=st.C['ink'], markerfacecolor='white', markeredgecolor=st.C['ink'],
         markersize=3.6, markeredgewidth=0.8, zorder=4,
         label='measured, 4000 s run')
axA.plot([0], [LAW['sigma_nav_m'][0]], 'o', color=st.C['green'],
         markerfacecolor=st.C['green'], markeredgecolor='white', markersize=4.6,
         markeredgewidth=0.7, zorder=5)
st.label_line(axA, 7, LAW['sigma_nav_m'][0] + 0.6,
              '%.1f m with the receiver on' % LAW['sigma_nav_m'][0],
              st.C['green'], ha='left', va='bottom', fontsize=6.8)

axA.set_xlim(0, 300)
axA.set_ylim(0, 40)
axA.set_yticks([0, 10, 20, 30])
axA.set_xlabel('Time since the last GNSS fix  [s]')
axA.set_ylabel(r'Navigation uncertainty  $\sigma_{\mathrm{nav}}$  [m]')
st.label_line(axA, 150, CEIL + 1.6,
              '%.1f m: reached with the receiver off' % CEIL,
              st.C['red'], ha='center', va='bottom', fontsize=6.8)
st.finish(axA, legend=True, loc='lower right')
st.panel_tag(axA, '(a)', x=-0.17)

# ---------------------------------------------------------------- panel (b)
axB.axvspan(0, DUTY_OP, color=st.C['vlgrey'], zorder=0, lw=0)
axB.plot(SW['duty_pct'], SW['ceiling_m'], '-', color=st.C['red'], lw=1.2,
         zorder=3, label='ceiling')
axB.plot(SW['duty_pct'], SW['mean_m'], '-', color=st.C['blue'], lw=1.2,
         zorder=3, label='time mean')
axB.plot(SW['duty_pct'], SW['floor_m'], color=st.C['grey'], lw=0.9,
         ls=(0, (1.4, 1.4)), zorder=3, label='floor')

i_op = int(np.argmin(np.abs(SW['duty_pct'] - DUTY_OP)))
axB.plot([DUTY_OP], [SW['mean_m'][i_op]], 'D', color=st.C['blue'],
         markeredgecolor='white', markersize=4.4, zorder=5)
axB.annotate('%.1f %% duty cycle:\nmean %.1f m' % (DUTY_OP, SW['mean_m'][i_op]),
             (DUTY_OP, SW['mean_m'][i_op]), textcoords='offset points',
             xytext=(10, 7), fontsize=6.8, color=st.C['blue'], linespacing=1.3)

axB.set_xlim(0, 100)
axB.set_ylim(0, 40)
axB.set_yticks([0, 10, 20, 30])
axB.set_xlabel('GNSS receiver duty cycle  [%]')
axB.set_ylabel(r'$\sigma_{\mathrm{nav}}$ over the cycle  [m]')
st.label_line(axB, 66, CEIL + 1.6, 'the ceiling does not move', st.C['red'],
              ha='center', va='bottom', fontsize=6.8)
st.finish(axB, legend=True, loc='center left')
st.panel_tag(axB, '(b)', x=-0.19)

st.save(fig, 'fig05_navigation')

# ---------------------------------------------------------------- numeros
print('  ceiling with the receiver never on       : %.2f m' % CEIL)
print('  ceiling at the operating duty cycle      : %.2f m  (%.2f %% on)'
      % (SW['ceiling_m'][i_op], DUTY_OP))
print('  time-mean sigma, off / operating / always : %.2f / %.2f / %.2f m'
      % (SW['mean_m'][0], SW['mean_m'][i_op], SW['mean_m'][-1]))
print('  floor whenever the receiver is on at all  : %.2f m' % SW['floor_m'][i_op])
print('  measured run: fresh %.2f m, saturated %.2f m'
      % (M['nav']['sigma_fresh_m'], M['nav']['sigma_saturated_m']))
# Comparar por PERTENENCIA a la casilla, no por su centro: la casilla 0-30 s la
# llenan sobre todo muestras con el receptor encendido (hueco = 0), y evaluar el
# modelo en hueco = 15 s comparia dos cosas distintas.
print('  model vs measurement, per bin:')
err = []
for lo, hi, s_ in zip(LAW['bin_lo_s'], LAW['bin_hi_s'], LAW['sigma_nav_m']):
    m_ = (gg >= lo) & (gg < hi)
    if not m_.any():
        continue
    p_ = float(np.median(ss[m_]))
    err.append(abs(p_ - s_) / s_)
    print('     %4.0f-%-4.0f s  measured %6.2f m  model %6.2f m  %+6.1f %%'
          % (lo, hi, s_, p_, 100 * (p_ - s_) / s_))
print('     median error %.1f %%  (the first bin is the outlier: the real filter '
      'gates on GNSS health, the model does not)' % (100 * np.median(err)))
print('  ceiling vs the assumed auxiliary sensor:')
for s_, c_ in zip(AUX['sigma_aux_m'], AUX['ceiling_m']):
    print('     sigma_aux = %6.0f m/axis  ->  ceiling %6.1f m' % (s_, c_))
