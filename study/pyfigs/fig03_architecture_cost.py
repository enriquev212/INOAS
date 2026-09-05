"""Figura 3: meter la restriccion de colision dentro del QP no cierra en tiempo real.

El argumento no esta en la mediana sino en la cola. Las medianas de las cuatro
configuraciones con la restriccion dentro del QP (0.03 a 1.1 s) parecen holgadas
frente a su plazo, que es el propio paso h; lo que rompe el planificador son los
solves que se disparan a 45 y 77 s. Por eso la figura dibuja la distribucion
completa, no una barra de medianas:

(a) Un punto por solve, con jitter, sobre eje logaritmico, y el plazo de cada
    configuracion marcado sobre su propio grupo. Se ve directamente cuanta masa
    queda por encima de la linea. El grupo propuesto va separado y sombreado
    porque es otra arquitectura, no otro ajuste de la misma.
(b) La misma poblacion vista como cola: fraccion de solves mas lentos que un
    tiempo dado, normalizando cada configuracion por SU plazo. Al normalizar,
    las cinco configuraciones son comparables en un unico eje y el plazo es la
    vertical x = 1. La distancia horizontal entre la nube propuesta y esa
    vertical es el margen, y se lee en decadas.

El color identifica la configuracion igual en los dos paneles, y las etiquetas
del eje de (a) son ya la clave completa: una leyenda las repetiria palabra por
palabra y costaria una fila entera de altura. Por eso (b) no lleva leyenda, solo
la nota de que el color es el mismo y el rotulo directo de la curva propuesta.
En (b) cada configuracion lleva ademas su propio trazo discontinuo: las curvas
roja y naranja casi se superponen y en gris no se distinguirian por color.

Nota sobre los datos: la columna pct_converged de timing_qp_cases.csv (94-96 %)
no tiene nada que ver con el plazo. Es 100*mean(exitflag>0), o sea cuantos QP
resolvio el solver. La fraccion que llega tarde hay que contarla sobre las
muestras, y contra h da 14.9, 6.2, 0 y 14.8 %. Son dos fracasos distintos y la
figura anota el segundo, que es el que se puede contar mirando el panel (a).
"""
import numpy as np

import inoas_style as st

st.use()
S = st.read_csv('timing_qp_samples.csv')
K = st.read_csv('timing_qp_cases.csv')
P = st.read_csv('timing_plan_track.csv')
M = st.meta()

D_PROP = float(M['execution']['guidance_step_s'])
NP_PROP = int(M['execution']['Np'])

# Orden: de peor a mejor margen, para que el panel se lea como una escalera que
# termina en la arquitectura propuesta.
QP_CFG = [(5, 150), (3, 125), (5, 100), (10, 50)]
QP_COL = [st.C['red'], st.C['orange'], st.C['purple'], st.C['blue']]

G = []
for (h, npv), col in zip(QP_CFG, QP_COL):
    m = (S['step_h_s'] == h) & (S['Np'] == npv)
    G.append({'v': np.sort(S['solve_time_s'][m]), 'd': float(h), 'c': col,
              'tick': '$h$ = %g s\n$N_p$ = %d' % (h, npv),
              'lab': '$h$ = %g s, $N_p$ = %d' % (h, npv)})
G.append({'v': np.sort(P['solve_time_s']), 'd': D_PROP, 'c': st.C['green'],
          'tick': '$h$ = %g s\n$N_p$ = %d' % (D_PROP, NP_PROP),
          'lab': 'proposed (outside QP)'})

for g in G:
    v, d = g['v'], g['d']
    g['med'] = float(np.median(v))
    g['worst'] = float(v.max())
    g['late'] = 100.0 * float(np.mean(v > d))
    g['margin'] = d / g['worst']

XPOS = [0.0, 1.0, 2.0, 3.0, 4.45]
XMAX = 5.40
XBAND = XPOS[-1] - 0.45   # borde izquierdo de la banda sombreada
HALF = 0.30            # semiancho del segmento de plazo
JIT = 0.20             # semiancho del jitter
rng = np.random.default_rng(7)


def label_y(w):
    """Altura para el rotulo del peor caso, esquivando la rejilla de decadas.

    El rotulo va encima del punto mas alto del grupo, pero la rejilla es
    logaritmica y una linea punteada que atraviesa el texto es exactamente lo
    que delata una figura mal acabada. Si el punto esta a menos de media decada
    de la linea de arriba, el rotulo se sube por encima de esa linea; si hay
    sitio de sobra, se queda pegado al punto.
    """
    e = np.log10(w)
    if 1.0 - (e - np.floor(e)) < 0.55:
        return 10.0 ** (np.floor(e) + 1.0) * 1.15
    return w * 1.30


fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.55, ncols=2,
                            gridspec_kw={'width_ratios': [1.32, 1], 'wspace': 0.24})

# ---------------------------------------------------------------- panel (a)
axA.axvspan(XBAND, XMAX, color=st.C['vlgrey'], lw=0, zorder=0)

for x, g in zip(XPOS, G):
    v = g['v']
    xs = x + rng.uniform(-JIT, JIT, v.size)
    late = v > g['d']
    axA.plot(xs[~late], v[~late], ls='none', marker='o', ms=1.9, mew=0,
             color=g['c'], alpha=0.45, zorder=2)
    axA.plot(xs[late], v[late], ls='none', marker='o', ms=2.2, mew=0,
             color=g['c'], alpha=0.95, zorder=3)
    # Mediana: barra con halo blanco para que se lea sobre la nube de puntos.
    axA.plot([x - HALF, x + HALF], [g['med']] * 2, color='white', lw=2.6, zorder=4)
    axA.plot([x - HALF, x + HALF], [g['med']] * 2, color=st.C['ink'], lw=1.3, zorder=5)
    # Plazo de tiempo real de esa configuracion.
    axA.plot([x - HALF - 0.10, x + HALF + 0.10], [g['d']] * 2, color=st.C['ink'],
             lw=1.0, ls=(0, (2.6, 1.4)), zorder=6)

axA.set_yscale('log')
axA.set_xlim(-0.68, XMAX)
axA.set_xticks(XPOS)
axA.set_xticklabels([g['tick'] for g in G])
# set_yticks ensancha la vista para meter una marca que caiga fuera, asi que el
# limite se fija DESPUES o no es el que dice el codigo.
axA.set_yticks([1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2])
axA.set_ylim(9.5e-4, 2.8e2)
st.plain_log(axA, 'y')
axA.set_ylabel('QP solve time  [s]')
axA.tick_params(axis='x', length=0)

# Peor caso de cada grupo, encima de su punto mas alto.
for x, g in zip(XPOS, G):
    txt = ('%.0f s' % g['worst']) if g['worst'] >= 1 else ('%.0f ms' % (1e3 * g['worst']))
    # El del grupo propuesto cae sobre la banda gris lisa y sin datos cerca: el
    # halo blanco solo abriria un boquete en la banda.
    st.label_line(axA, x, label_y(g['worst']), txt, g['c'], ha='center',
                  va='bottom', fontsize=6.8, halo=g is not G[-1])

# Margen de la arquitectura propuesta: del peor solve a su plazo, en decadas.
# El rotulo va centrado en el hueco entre las lineas de 1 s y 10 s, no en el
# centro geometrico de la flecha, para que la rejilla no lo atraviese.
xp = XPOS[-1]
axA.annotate('', xy=(xp + 0.34, D_PROP), xytext=(xp + 0.34, G[-1]['worst']),
             arrowprops={'arrowstyle': '<->', 'lw': 0.7, 'color': st.C['ink'],
                         'shrinkA': 1.5, 'shrinkB': 1.5})
axA.text(xp + 0.46, 3.16, '%.0f$\\times$\nmargin' % G[-1]['margin'],
         color=st.C['ink'], ha='left', va='center', fontsize=6.8,
         linespacing=1.15, zorder=7)

# Glosario de los dos trazos, junto al grupo mas despejado. 'median' cabe a la
# derecha de su barra porque la banda sombreada empieza en XBAND.
st.label_line(axA, XPOS[3], G[3]['d'] * 1.45, 'real-time\ndeadline', st.C['ink'],
              ha='center', va='bottom', fontsize=6.8, linespacing=1.15)
st.label_line(axA, XPOS[3] + 0.36, G[3]['med'], 'median', st.C['ink'],
              ha='left', va='center', fontsize=6.8)

# Dos filas bajo las etiquetas de grupo: tamano de muestra y fraccion tardia.
# La segunda es la cifra que el lector puede contar en la nube de puntos.
for yrow, cap in ((-0.205, 'solves:'), (-0.305, 'past deadline:')):
    axA.text(-0.03, yrow, cap, transform=axA.transAxes, ha='right', va='top',
             fontsize=6.8, color=st.C['grey'])
for x, g in zip(XPOS, G):
    axA.text(x, -0.205, '%d' % g['v'].size, transform=axA.get_xaxis_transform(),
             ha='center', va='top', fontsize=6.8, color=st.C['grey'])
    col = st.C['red'] if g['late'] > 0 else st.C['green']
    axA.text(x, -0.305, ('%.1f%%' % g['late']) if g['late'] > 0 else 'none',
             transform=axA.get_xaxis_transform(), ha='center', va='top',
             fontsize=6.8, color=col)

# Cabeceras: que arquitectura es cada bloque.
tb = axA.get_xaxis_transform()
axA.plot([XPOS[0] - 0.42, XPOS[3] + 0.42], [1.045] * 2, transform=tb,
         color=st.C['grey'], lw=0.6, clip_on=False)
axA.plot([XBAND, XMAX], [1.045] * 2, transform=tb,
         color=st.C['green'], lw=0.6, clip_on=False)
axA.text(1.5, 1.075, 'collision constraint inside the QP', transform=tb,
         ha='center', va='bottom', fontsize=7, color=st.C['grey'])
axA.text(0.5 * (XBAND + XMAX), 1.075, 'proposed', transform=tb,
         ha='center', va='bottom', fontsize=7, color=st.C['green'])

st.finish(axA, grid='y')
st.panel_tag(axA, '(a)', x=-0.135, y=1.075)

# ---------------------------------------------------------------- panel (b)
axB.axvspan(1.0, 1e2, color=st.C['vlgrey'], lw=0, zorder=0)
axB.axvline(1.0, color=st.C['ink'], lw=0.8, ls=(0, (2.6, 1.4)), zorder=1)

for i, g in enumerate(G):
    x = g['v'] / g['d']
    y = 100.0 * (len(x) - np.arange(len(x))) / len(x)
    prop = g is G[-1]
    axB.step(x, y, where='post', color=g['c'], lw=1.6 if prop else 1.0,
             dashes=(None, None) if prop else st.DASHES[i + 1],
             zorder=4 if prop else 3, solid_joinstyle='miter')
    axB.plot([x[-1]], [y[-1]], marker='o', ms=2.8, mew=0.6, color=g['c'],
             markerfacecolor='white', zorder=5)

axB.set_xscale('log')
axB.set_yscale('log')
axB.set_yticks([1, 10, 100])
axB.set_xlim(5e-5, 2.6e1)
axB.set_ylim(0.30, 2.2e2)
st.plain_log(axB, 'y')
axB.set_xlabel('Solve time / real-time deadline  [-]')
axB.set_ylabel('Solves slower than the abscissa  [%]')
# Rotulos directos: la banda sombreada y la curva heroe. Sin halo, porque los
# dos caen en zonas vacias y el halo solo dejaria un cerco blanco en la banda.
st.label_line(axB, 1.38, 130, 'past deadline', st.C['ink'], ha='left',
              va='bottom', fontsize=6.8, halo=False)
st.label_line(axB, 8.0e-5, 128, 'proposed', st.C['green'], ha='left',
              va='bottom', fontsize=7, halo=False)
# El color ya identifica cada configuracion en (a); repetir alli las cinco
# etiquetas en una leyenda costaria una fila de figura y no anadiria nada.
st.label_line(axB, 5.0e-3, 0.58, 'colours as in (a)', st.C['grey'], ha='left',
              va='center', fontsize=6.8, halo=False)
st.finish(axB, grid='y')
st.panel_tag(axB, '(b)', x=-0.20, y=1.075)

st.save(fig, 'fig03_architecture_cost')

print('  configuration        n   median [s]   p90 [s]   worst [s]   late vs h   margin')
for g in G:
    v, d = g['v'], g['d']
    print('  %-20s %4d  %10.4f  %8.3f  %10.3f  %8.1f %%  %8.1f x'
          % (g['lab'].replace('$', '').replace('\\', ''), v.size, g['med'],
             float(np.percentile(v, 90)), g['worst'], g['late'], g['margin']))
print('  proposed: median %.2f ms, worst %.1f ms, deadline %.0f s'
      % (1e3 * G[-1]['med'], 1e3 * G[-1]['worst'], D_PROP))
print('  proposed margin on the median: %.0f x  (%.1f decades)'
      % (D_PROP / G[-1]['med'], np.log10(D_PROP / G[-1]['med'])))
# Dos fracasos distintos que conviene no confundir: que el solver no converja y
# que converja tarde. El fichero de resumen trae el primero; el segundo se cuenta
# aqui sobre las muestras.
print('  convergence vs deadline, per configuration:')
for h, npv, conv, p90 in zip(K['step_h_s'], K['Np'], K['pct_converged'],
                             K['p90_s']):
    m = (S['step_h_s'] == h) & (S['Np'] == npv)
    v = S['solve_time_s'][m]
    print('    h = %2g s, Np = %3d   solver converged %5.1f %%   within the %g s '
          'deadline %5.1f %%   p90 %7.3f s (file %7.3f s)'
          % (h, npv, conv, h, 100.0 * np.mean(v <= h), np.percentile(v, 90), p90))
