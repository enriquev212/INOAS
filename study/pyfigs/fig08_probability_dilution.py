"""Figura 8: por que el guiado restringe una distancia y no una probabilidad.

Las dos mitades de la figura recorren exactamente el mismo barrido de
incertidumbre de navegacion y se contradicen a proposito:

(a) Probabilidad de colision de Foster con la distancia de paso FIJA en
    120 m, la del encuentro sin maniobra. Al crecer la covarianza la misma masa
    de probabilidad se reparte sobre un area mayor, asi que Pc BAJA. Una ley que
    dispare por umbral de Pc premia por tanto una navegacion peor, que es justo
    el fallo de diseno que la figura acusa.

(b) La distancia de paso que exige el diseno, keep-out inflado con la covarianza
    combinada, y el impulso que cuesta. Ambos crecen de forma monotona con la
    incertidumbre, que es el comportamiento que se quiere.

Se apilan en vez de superponerse con dos ejes porque el mensaje es el signo de
la pendiente: con solo cuatro puntos por curva, dos ejes gemelos obligan al
lector a emparejar curva y eje antes de leer nada, y aqui lo unico que hay que
ver es que una baja mientras la otra sube sobre la misma abscisa.

Decisiones de composicion, todas tomadas midiendo las cajas de texto contra la
rejilla en coordenadas de pantalla, no a ojo:

  * El impulso no se rotula pegado a cada marcador sino en una franja de valores
    al pie de (b), alineada con cada sigma. Pegado al marcador chocaba con la
    rejilla y con el tramo que sube hacia el ultimo punto; en la franja no puede
    chocar con nada y sigue emparejado por abscisa. Ademas evita un segundo eje:
    el impulso es proporcional a la distancia exigida con un 4 % de dispersion,
    asi que una segunda curva seria la misma curva dos veces.
  * (a) lleva rejilla cada 5 unidades, no cada 2, y ticks menores sin rejilla
    cada unidad. Con rejilla cada 2 no existe ningun hueco de 42 pt libre, y el
    bloque rojo de tres lineas cortaba una linea de rejilla por el medio de las
    letras.
  * La banda gris del ciclo de trabajo empieza en 4.8 m, fuera del eje; se
    rotula con sus dos extremos para que su borde izquierdo recortado no se lea
    como si la banda empezase donde empieza el eje.

Los datos son del demostrador de conjuncion autonomo, con su propia hipotesis de
covarianza del objeto (sigma_obj = 150 m); no son la misma magnitud que el
retarget_sweep del resto del articulo y no deben mezclarse en un panel.

Dos advertencias que el pie de figura tiene que recoger:

  * El eje va en radio 3D sqrt(trace(P)), que es el convenio de las figuras 1,
    5, 6 y 7. cam_demo.m trabaja por eje; la conversion es el factor sqrt(3) y
    esta hecha en el CSV. La version anterior sombreaba la banda medida
    4.8-32.1 m, que es radio 3D, sobre un eje graduado por eje.
  * El impulso es una cota SUPERIOR: cam_demo.m limita la busqueda a la rama
    posigrada, que es justamente la ley que c2bb207 saco de plan_cam. Resolviendo
    la misma parabola en la rama retrograda sale entre un 13 y un 38 % mas barato.
    El sentido del resultado no cambia, porque es la MONOTONIA lo que se compara
    contra Pc, pero los valores absolutos no son los que planificaria el guiado.
"""
import matplotlib.patheffects as pe
import numpy as np
from matplotlib.ticker import MultipleLocator

import inoas_style as st

# Mismo halo blanco que st.label_line: los rotulos caen sobre la rejilla
# punteada y sin el se lee la rejilla entre las letras.
HALO = [pe.withStroke(linewidth=1.8, foreground='white')]

st.use()
P = st.read_csv('pc_nav_sweep.csv')
M = st.meta()

# Eje en radio 3D, que es el convenio de las figuras 1, 5, 6 y 7. cam_demo.m
# trabaja por eje, y sombrear sobre ese eje la banda medida 4.8-32.1 m, que es
# radio 3D, la ponia un factor sqrt(3) fuera de sitio.
o = np.argsort(P['sigma_nav_3d_m'])
SN = P['sigma_nav_3d_m'][o]
PC_B = P['Pc_before'][o]
PC_A = P['Pc_after'][o]
MISS = P['miss_required_m'][o]
DV = P['dv_ms'][o] * 1e3            # m/s -> mm/s

# Distancia de paso del encuentro sin maniobra del propio demostrador: es la
# que se mantiene fija en el panel (a)
MISS_FIXED = M['execution']['miss_before_m']
BAND = (M['nav']['sigma_fresh_m'], M['nav']['sigma_saturated_m'])

XLIM = (17.0, 520.0)
XTICKS = [20, 70, 170, 430]

r_pc = PC_B[0] / PC_B[-1]
r_sn = SN[-1] / SN[0]
r_miss = MISS[-1] / MISS[0]
r_dv = DV[-1] / DV[0]

fig, (axA, axB) = st.figure(width=st.SINGLE, height=2.92, nrows=2, sharex=True,
                            gridspec_kw={'hspace': 0.20})

# ---------------------------------------------------------------- panel (a)
# Pc en unidades de 1e-4 para poder usar eje lineal desde cero: la caida se lee
# como caida y no como un artefacto de recortar el eje.
axA.axvspan(BAND[0], BAND[1], color=st.C['vlgrey'], zorder=0, lw=0)
axA.axhline(1.0, color=st.C['grey'], lw=0.7, ls=(0, (3, 2)), zorder=1)
axA.plot(SN, PC_B * 1e4, color=st.C['red'], marker='o', markersize=4.0,
         markerfacecolor=st.C['red'], markeredgecolor='white', zorder=3)

axA.set_ylim(0, 10.4)
axA.set_yticks([0, 5, 10])
axA.yaxis.set_minor_locator(MultipleLocator(1))
axA.set_ylabel('$P_c$ at a fixed\n%.0f m miss  [$\\times 10^{-4}$]' % MISS_FIXED)
axA.yaxis.set_label_coords(-0.16, 0.5)
# Bloque rojo alojado entre las rejillas de 5 y 10, por encima de la curva.
axA.text(0.985, 0.915,
         '$P_c$ falls %.1f$\\times$ while $\\sigma_{\\mathrm{nav}}$\n'
         'grows %.0f$\\times$: a $P_c$ trigger\nrewards worse navigation' % (r_pc, r_sn),
         transform=axA.transAxes, ha='right', va='top', fontsize=6.9,
         color=st.C['red'], linespacing=1.35, zorder=6).set_path_effects(HALO)
st.label_line(axA, 10.6, 1.5, 'conventional $10^{-4}$ screening threshold',
              st.C['grey'], ha='left', va='bottom', fontsize=6.6)
# La banda se rotula aqui, y con sus dos extremos, porque arranca en 4.8 m y el
# eje empieza en 10: sin los numeros el borde recortado enganaria.
st.label_line(axA, np.sqrt(XLIM[0] * BAND[1]), 6.53,
              'duty-cycled GNSS\n%.1f to %.1f m' % BAND,
              st.C['grey'], ha='center', va='center', fontsize=6.6,
              linespacing=1.3)
st.finish(axA)
st.panel_tag(axA, '(a)', x=-0.155)

# ---------------------------------------------------------------- panel (b)
axB.axvspan(BAND[0], BAND[1], color=st.C['vlgrey'], zorder=0, lw=0)
axB.plot(SN, MISS, color=st.C['blue'], marker='s', markersize=3.8,
         markerfacecolor=st.C['blue'], markeredgecolor='white', zorder=3)

axB.set_ylim(0, 950)
axB.set_yticks([0, 300, 600, 900])
axB.yaxis.set_minor_locator(MultipleLocator(100))
axB.set_ylabel('Required miss\ndistance  [m]')
axB.yaxis.set_label_coords(-0.16, 0.5)
axB.set_xscale('log')
axB.set_xlim(*XLIM)
axB.set_xticks(XTICKS)
st.plain_log(axB, 'x')
axB.set_xlabel(r'Navigation uncertainty at the encounter, $\sigma_{\mathrm{nav}}$  [m]')

# Franja de valores del impulso al pie del panel: cada cifra sobre su sigma, en
# el hueco que la curva nunca ocupa porque la distancia exigida no baja de 333 m.
DV_ROW_Y = 219.0
DV_LABEL_Y = 90.0
for x, dv in zip(SN, DV):
    axB.text(x, DV_ROW_Y, '%.1f' % dv, ha='center', va='center', fontsize=6.9,
             color=st.C['orange'], zorder=6).set_path_effects(HALO)
axB.text(XLIM[0] * 1.05, DV_LABEL_Y, '$\\Delta v$ required  [mm s$^{-1}$]',
         ha='left', va='center', fontsize=6.9,
         color=st.C['orange'], zorder=6).set_path_effects(HALO)

axB.text(XLIM[0] * 1.05, 750.0,
         'Inflated keep-out: miss $\\times$%.1f, $\\Delta v$ $\\times$%.1f'
         % (r_miss, r_dv),
         ha='left', va='center', fontsize=6.9,
         color=st.C['blue'], zorder=6).set_path_effects(HALO)
st.finish(axB)
st.panel_tag(axB, '(b)', x=-0.155)

st.save(fig, 'fig08_probability_dilution')

print('    miss held fixed at %.1f m, sigma_obj = 150 m (standalone demonstrator)' % MISS_FIXED)
print('    duty-cycled GNSS band: sigma_nav %.2f -> %.2f m (%.1f %% receiver on)'
      % (BAND[0], BAND[1], 100.0 * M['nav']['on_fraction']))
print('    sigma_nav   Pc_before    Pc_after   miss_req    dv')
for s, pb, pa, m_, d in zip(SN, PC_B, PC_A, MISS, DV):
    print('    %6.0f m   %.3e  %.3e  %6.1f m  %5.2f mm/s' % (s, pb, pa, m_, d))
print('    dilution : Pc %.3e -> %.3e  (/%.1f) while sigma_nav x%.0f'
      % (PC_B[0], PC_B[-1], r_pc, r_sn))
print('    design   : miss %.1f -> %.1f m (x%.2f), dv %.2f -> %.2f mm/s (x%.2f)'
      % (MISS[0], MISS[-1], r_miss, DV[0], DV[-1], r_dv))
print('    Pc margin over the 1e-4 screening threshold: x%.2f -> x%.2f'
      % (PC_B[0] / 1e-4, PC_B[-1] / 1e-4))
