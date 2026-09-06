"""Figura 4: un encuentro ejecutado de principio a fin.

(a) Modulo del comando de aceleracion y coste acumulado.
(b) Error de seguimiento del MPC respecto a la referencia rediseniada.

Por que el eje de tiempo esta partido. El registro dura 116 min y todo lo que
pasa ocurre en los 50 min que siguen al impulso: a partir de -62 min el comando
esta tres ordenes de magnitud por debajo del pico y el error esta en milimetros.
Con el eje entero y lineal, el 72 % del ancho impreso seria una recta plana y la
maniobra se quedaria en 0.64 in. Partiendo el eje, el tramo activo se estira 1.5x
(0.95 in) y la muesca entre los dos lobulos del comando pasa de 0.6 a 1.0 mm, que
es la diferencia entre verla y no verla. El tramo de deriva se comprime 2.8x pero
no se recorta: esa planitud es el resultado -- el plan se calcula una vez y no se
revisa -- y ademas su hueco es el sitio natural para el balance de Delta v, que
es la unica cifra del encuentro que no es una serie temporal.

Por que el Delta v acumulado va sobre el mismo panel que el comando. El sobrecoste
del 53 % respecto al plan es la contrapartida honesta de la arquitectura: el
guiado planifica un impulso y el MPC lo entrega como un empuje continuo repartido
en ~40 min. Poniendo la integral encima de la senial que la genera, el lector ve
de donde sale el exceso sin cambiar de panel.

Sin leyenda: tres curvas en total, cada una rotulada donde no pisa a nadie. Todo
lo que pertenece al eje de Delta v va en morado -- curva, referencia del plan,
rotulos, espina y numeros del eje -- porque la referencia discontinua del plan
cruza el panel a una altura que, leida contra el eje primario, seria un valor
plausible de |u|; el color es lo unico que impide esa lectura falsa.
"""
import numpy as np

import inoas_style as st

st.use()
E = st.read_csv('execution.csv')
M = st.meta()
X = M['execution']

t = E['t_rel_tca_min']
u = E['u_norm_um_s2']
err = E['tracking_error_m']

DT = X['guidance_step_s']
U_MAX_UM = X['u_max_ms2'] * 1e6
T_BURN = X['t_burn_rel_tca_min']
DV_PLAN = X['dv_planned_mms']
DV_SPENT = X['dv_spent_mms']

# El comando es constante en cada paso de guiado, asi que su integral discreta es
# exactamente el Delta v que consume el propulsor.
dv_cum = np.cumsum(u) * DT * 1e-3

u_pk = u.max()
t_pk = t[np.argmax(u)]
e_pk = err.max()
t_epk = t[np.argmax(err)]
e_rms = np.sqrt(np.mean(err ** 2))
overhead = 100.0 * (DV_SPENT - DV_PLAN) / DV_PLAN

# --- geometria del eje partido -------------------------------------------
XBRK = -62.0                 # donde deja de haber senial visible
XL = (-116.8, XBRK)
XR = (XBRK, 2.6)
WR = [2.4, 1.0]
WSPACE = 0.06

# Limites atados a los datos y no escritos a mano. Al corregir la ley de guiado
# (suelo de 150 m en vez del radio de cuerpo duro, y sigma como radio 3D) el pico
# paso de 39.6 a 68.5 um/s^2 y el error de 2.4 a 4.2 m, y unos limites fijos
# recortaban las dos curvas.
U_TOP = float(np.ceil(u_pk / 10.0) * 10.0 + 12.0)
DV_TOP = float(np.ceil(DV_SPENT / 5.0) * 5.0 + 8.0)
E_TOP = float(np.ceil(e_pk * 1.35 * 10.0) / 10.0)

coast = t >= XBRK
e_coast = err[coast].max()
u_coast = u[coast].max()

fig, AX = st.figure(width=st.SINGLE, height=3.15, nrows=2, ncols=2,
                    sharex='col', sharey='row',
                    gridspec_kw={'width_ratios': WR, 'wspace': WSPACE,
                                 'hspace': 0.20,
                                 'height_ratios': [1.15, 1.0]})
(axAL, axAR), (axBL, axBR) = AX
# Margenes fijados a mano: hay que reservar sitio para el segundo eje de la
# derecha sin que el recorte 'tight' ensanche la figura mas alla de la columna.
fig.subplots_adjust(left=0.165, right=0.845, bottom=0.125, top=0.945)

# --- panel (a): comando y coste ------------------------------------------
axA2L, axA2R = axAL.twinx(), axAR.twinx()
for ax, a2, xlim in ((axAL, axA2L, XL), (axAR, axA2R, XR)):
    ax.fill_between(t, 0, u, step='post', color=st.C['blue'], alpha=0.16,
                    lw=0, zorder=2)
    ax.plot(t, u, drawstyle='steps-post', color=st.C['blue'], lw=1.0, zorder=3)
    a2.axhline(DV_PLAN, color=st.C['purple'], lw=0.6, ls=(0, (3.0, 1.8)),
               zorder=2)
    a2.plot(t, dv_cum, drawstyle='steps-post', color=st.C['purple'], lw=1.0,
            zorder=4)
    ax.set_xlim(*xlim)
    ax.set_ylim(0, U_TOP)
    a2.set_xlim(*xlim)
    a2.set_ylim(0, DV_TOP)
    a2.set_zorder(ax.get_zorder() + 1)
    a2.patch.set_visible(False)
    for s in a2.spines.values():
        s.set_visible(False)
    a2.tick_params(left=False, right=False, labelleft=False, labelright=False)

# Marcas derivadas del limite, para que cubran el pico en vez de quedarse cortas
axAL.set_yticks(np.arange(0, U_TOP, 20.0))
axAL.set_ylabel('Commanded acceleration\n' + r'$|u|$  [$\mu$m s$^{-2}$]')

# --- panel (b): error de seguimiento --------------------------------------
for ax, xlim in ((axBL, XL), (axBR, XR)):
    ax.plot(t, err, color=st.C['green'], lw=1.0, zorder=3)
    ax.set_xlim(*xlim)
    ax.set_ylim(0, E_TOP)
axBL.set_yticks(np.arange(0, E_TOP, 1.0 if E_TOP < 4 else 2.0))
axBL.set_ylabel('Tracking error\n[m]')

# --- marcas de los dos instantes -----------------------------------------
# En el panel de arriba la marca se prolonga por encima del marco hasta tocar su
# rotulo. Sin ese trozo de linea los dos rotulos quedan alineados en la banda
# superior de la figura y se leen como un titulo; con el, cada uno cuelga
# visiblemente de su instante y se lee como lo que es, una llamada a un evento.
for ax, x0 in ((axAL, T_BURN), (axAR, 0.0)):
    ax.plot([x0, x0], [0, 1.012], transform=ax.get_xaxis_transform(),
            color=st.C['grey'], lw=0.7, ls=(0, (1.1, 1.3)), zorder=1,
            clip_on=False)
axBL.axvline(T_BURN, color=st.C['grey'], lw=0.7, ls=(0, (1.1, 1.3)), zorder=1)
axBR.axvline(0.0, color=st.C['grey'], lw=0.7, ls=(0, (1.1, 1.3)), zorder=1)

# --- acabado de los cuatro ejes -------------------------------------------
axAL.set_xticks([-110, -100, -90, -80, -70])
axAR.set_xticks([-40, -20, 0])
for ax in (axAL, axAR, axBL, axBR):
    st.finish(ax, grid='y')
for ax in (axAR, axBR):
    ax.spines['left'].set_visible(False)
    ax.tick_params(left=False, labelleft=False)

# El eje derecho de Delta v se saca al borde exterior de la figura, sobre el
# tramo comprimido, donde no cruza ningun dato.
axA2R.spines['right'].set_visible(True)
axA2R.spines['right'].set_position(('outward', 3))
axA2R.spines['right'].set_linewidth(0.6)
axA2R.spines['right'].set_color(st.C['purple'])
axA2R.yaxis.set_ticks_position('right')
axA2R.set_yticks(np.arange(0, DV_TOP, 10.0))
axA2R.tick_params(right=True, labelright=True, colors=st.C['purple'])
axA2R.set_ylabel(r'$\Delta v$  [mm s$^{-1}$]',
                 color=st.C['purple'], rotation=270, va='bottom', labelpad=13)
axA2R.yaxis.set_label_position('right')

# etiqueta de tiempo centrada bajo el par de ejes
half = 0.5 * (WR[0] + WR[1] + WSPACE * 0.5 * (WR[0] + WR[1])) / WR[0]
axBL.set_xlabel('Time relative to the encounter  [min]')
axBL.xaxis.set_label_coords(half, -0.23)

# --- rotulos --------------------------------------------------------------
axAL.plot([t_pk], [u_pk], marker='o', ms=2.8, color=st.C['blue'],
          markeredgecolor='white', markeredgewidth=0.6, zorder=6)
st.label_line(axAL, -105.2, U_TOP * 0.985,
              'peak %.1f $\\mu$m s$^{-2}$\n%.3f %% of $u_{\\max}$'
              % (u_pk, 100 * u_pk / U_MAX_UM),
              st.C['blue'], ha='left', va='top', fontsize=6.8, linespacing=1.25)
st.label_line(axA2L, -98.5, 7.6, r'cumulative $\Delta v$', st.C['purple'],
              ha='left', va='center', fontsize=6.8)

# El rotulo del gasto va 1.8 mm/s por encima de su meseta: justo lo necesario
# para que la rejilla de 20 mm/s pase entre los dos sin rozar el texto.
st.label_line(axA2R, XBRK + 3, DV_SPENT + 1.8, '%.2f spent' % DV_SPENT,
              st.C['purple'], ha='left', va='bottom', fontsize=6.8)
st.label_line(axA2R, XBRK + 3, DV_PLAN - 0.7, '%.2f planned' % DV_PLAN,
              st.C['purple'], ha='left', va='top', fontsize=6.8)
axA2R.annotate('', xy=(-31, DV_SPENT), xytext=(-31, DV_PLAN),
               arrowprops=dict(arrowstyle='<->', lw=0.6, color=st.C['ink'],
                               shrinkA=0, shrinkB=0, mutation_scale=5))
st.label_line(axA2R, -28, 0.5 * (DV_PLAN + DV_SPENT), '$+$%.0f %%' % overhead,
              st.C['ink'], ha='left', va='center', fontsize=6.8)

axBL.plot([t_epk], [e_pk], marker='o', ms=2.8, color=st.C['green'],
          markeredgecolor='white', markeredgewidth=0.6, zorder=6)
st.label_line(axBL, -95.0, E_TOP * 0.97,
              'rms %.2f m\npeak %.2f m' % (e_rms, e_pk), st.C['green'],
              ha='left', va='top', fontsize=6.8, linespacing=1.25)
# El resultado de seguridad del encuentro no es una serie temporal, pero es la
# cifra que justifica toda la maniobra: va en el hueco que deja el error al caer.
st.label_line(axBL, -95.0, E_TOP * 0.50,
              'miss  %.1f m $\\rightarrow$ %.1f m\n'
              'target $d_0+3\\sigma_{\\mathrm{c}} = %.1f$ m'
              % (X['miss_before_m'], X['miss_achieved_m'], X['d_target_m']),
              st.C['grey'], ha='left', va='center', fontsize=6.8,
              linespacing=1.25)
st.label_line(axBR, XBRK + 3, E_TOP * 0.20,
              'no re-planning:\n$|e|\\leq%.1f$ mm' % (e_coast * 1e3),
              st.C['grey'], ha='left', va='bottom', fontsize=6.8,
              linespacing=1.25)

# los dos instantes, rotulados fuera del area de datos
axAL.text(T_BURN + 1.0, 1.015, 'retrograde impulse, $t=%.1f$ min' % T_BURN,
          transform=axAL.get_xaxis_transform(), ha='left', va='bottom',
          fontsize=6.8, color=st.C['grey'])
axAR.text(-1.5, 1.015, 'encounter', transform=axAR.get_xaxis_transform(),
          ha='right', va='bottom', fontsize=6.8, color=st.C['grey'])

st.panel_tag(axAL, '(a)', x=-0.235, y=1.02)
st.panel_tag(axBL, '(b)', x=-0.235, y=1.02)


def break_marks():
    """Las dos barras del corte, sobre el eje inferior desplazado 3 pt."""
    fig.canvas.draw()
    for axl, axr in ((axAL, axAR), (axBL, axBR)):
        for ax, xf in ((axl, 1.0), (axr, 0.0)):
            bb = ax.get_window_extent()
            w = bb.width / fig.dpi
            h = bb.height / fig.dpi
            y0 = -3.0 / 72.0 / h
            ax.plot([xf - 0.026 / w, xf + 0.026 / w],
                    [y0 - 0.040 / h, y0 + 0.040 / h], transform=ax.transAxes,
                    color=st.C['ink'], lw=0.6, clip_on=False, zorder=9,
                    solid_capstyle='butt')


break_marks()
st.save(fig, 'fig04_execution')

print('    geometry     : dInc = %.2f deg, v_rel = %.1f km/s, pass = %.3f s'
      % (X['dInc_deg'], X['v_rel_ms'] * 1e-3, X['pass_time_s']))
print('    miss distance: %.1f m -> %.2f m   (target %.2f m, sigma_b = %.2f m)'
      % (X['miss_before_m'], X['miss_achieved_m'], X['d_target_m'],
         X['sigma_bplane_m']))
print('    impulse      : %.2f mm/s at t = %.1f min' % (DV_PLAN, T_BURN))
print('    MPC spends   : %.2f mm/s  -> +%.0f %% over the plan'
      % (DV_SPENT, overhead))
print('    command      : peak %.2f um/s2 at t = %.1f min = %.3f %% of u_max'
      % (u_pk, t_pk, 100 * u_pk / U_MAX_UM))
print('    tracking     : rms %.3f m, peak %.3f m at t = %.1f min'
      % (e_rms, e_pk, t_epk))
print('    coast t > %.0f min: |u| < %.4f um/s2, |e| < %.1f mm'
      % (XBRK, u_coast, e_coast * 1e3))
print('    burn epoch   : %.1f min = %.3f orbital periods before TCA (T_orb '
      '= %.0f s)' % (T_BURN, abs(T_BURN) * 60.0 / M['T_orb_s'], M['T_orb_s']))
