"""Figura 5: lo que cuesta de verdad apagar el receptor GNSS.

El argumento del articulo es que el ciclo de trabajo del GNSS no degrada la
navegacion sin limite, sino que la degrada hasta un techo. Eso es lo que hay que
ver de un vistazo, asi que la figura esta partida en dos lecturas del mismo
registro:

(a) La serie medida: el tren de encendidos arriba y el sigma del filtro debajo.
    Se dibuja una ventana de 900 s (300-1200 s) y no los 4000 s completos por dos
    razones. Primera, a 3.45 in de ancho 4000 s serian doce dientes de sierra de
    0.25 in cada uno: se veria el patron pero no la forma de cada diente, que es
    justo lo que hay que juzgar. Segunda, esa ventana es la unica del registro que
    contiene las tres longitudes de hueco que existen (40, 182 y 300 s); los
    2900 s restantes repiten el hueco de 300 s de forma identica. Con las tres
    juntas se ve que el hueco corto se queda muy por debajo del techo y que los
    dos largos dan exactamente la misma meseta.

(b) La misma serie colapsada sobre el tiempo transcurrido desde la ultima medida.
    Los dientes de sierra caen unos sobre otros: el sigma no depende de cuando
    ocurre el hueco sino solo de su duracion, y por encima de ~90 s ya no depende
    ni de eso. Ese techo es el numero que la capa de guiado consume.

Los dos paneles comparten limites y marcas del eje y a proposito: (b) es (a)
plegado, y compartir la escala lo hace evidente sin decirlo.

Dos decisiones de (b) que no son cosmeticas:

  * Las medianas por casilla se dibujan como marcadores sueltos, sin unirlas.
    Unirlas con rectas mentia: entre la casilla 0-30 s y la de 30-60 s la cuerda
    pasa 4.8 m por debajo de las muestras que dice resumir a los 20 s (8.2 m
    frente a 12.9 m), justo en el tramo de subida que es lo que se juzga. La
    nube cruda ya dibuja la curva; el marcador solo tiene que decir donde cae la
    ley tabulada.
  * Cada mediana se coloca en la mediana del tiempo desde la ultima medida de su
    casilla, no en el centro geometrico. Como sigma crece de forma monotona con
    ese tiempo, la mediana de sigma en una casilla es exactamente sigma en la
    mediana del tiempo, asi que todos los marcadores caen sobre la nube por
    construccion. Lo importante es la primera casilla: la dominan las 260
    muestras tomadas con el receptor encendido, cuyo tiempo desde la ultima
    medida es 0, y dibujarla en el centro de la casilla (15 s) la dejaba flotando
    muy por debajo de la nube, como si el resumen estuviera mal ajustado.

Las muestras con el receptor encendido se pintan en verde, el mismo verde del
horario del panel (a): son otra poblacion, no ruido de la ley de deriva.
"""
import numpy as np
from matplotlib.ticker import MultipleLocator

import inoas_style as st

st.use()
D = st.read_csv('nav_duty.csv')
S = st.read_csv('nav_sigma.csv')
L = st.read_csv('nav_law.csv')
M = st.meta()['nav']

SIG_FRESH = M['sigma_fresh_m']
SIG_SAT = M['sigma_saturated_m']
ON_FRAC = M['on_fraction']

# --- tramos de receptor encendido -----------------------------------------
# El CSV de la programacion son 4001 muestras de una onda cuadrada; dibujarla
# punto a punto es tinta sin informacion. Se reduce a la lista de intervalos.
t, on = D['t_s'], D['gnss_on']
edge = np.diff(np.concatenate(([0.0], on, [0.0])))
i0 = np.where(edge > 0)[0]
i1 = np.where(edge < 0)[0] - 1
SEG = [(t[a], t[b] + 1.0) for a, b in zip(i0, i1)]
GAP = [(a[1], b[0]) for a, b in zip(SEG[:-1], SEG[1:])]

W0, W1 = 300.0, 1200.0          # ventana del panel (a)
YLIM = (0.0, 37.0)
YTICKS = [0, 10, 20, 30]

# Hueco corto: su pico se queda por debajo del techo y ese contraste es el
# que demuestra que la saturacion es una propiedad del hueco, no del reloj.
gshort = min(GAP, key=lambda g: g[1] - g[0])
mshort = (S['t_s'] >= gshort[0]) & (S['t_s'] <= gshort[1])
ishort = np.argmax(S['sigma_nav_m'][mshort])
PEAK_SHORT = S['sigma_nav_m'][mshort][ishort]
PEAK_SHORT_TSF = S['time_since_fix_s'][mshort][ishort]

# Tiempo en el que el filtro entra en el 1 % del techo.
T99 = S['time_since_fix_s'][S['sigma_nav_m'] >= 0.99 * SIG_SAT].min()

# Dos poblaciones distintas en el mismo registro: con medida (tiempo desde la
# ultima medida = 0) y en deriva libre. Mezclarlas en gris es lo que hacia que el
# origen del panel (b) pareciera dispersion de la ley.
FIX = S['time_since_fix_s'] == 0.0
DRIFT = ~FIX

# Posicion en x de cada mediana por casilla: la mediana del tiempo desde la
# ultima medida dentro de la casilla. Ver la cabecera.
BIN_X = np.array([np.median(S['time_since_fix_s'][(S['time_since_fix_s'] >= lo) &
                                                  (S['time_since_fix_s'] < hi)])
                  for lo, hi in zip(L['bin_lo_s'], L['bin_hi_s'])])

# Dispersion entre huecos a igual tiempo desde la ultima medida: es la medida de
# cuanto colapsa el diente de sierra, y sale del propio registro.
COLLAPSE = max(np.ptp(S['sigma_nav_m'][S['time_since_fix_s'] == v])
               for v in np.unique(S['time_since_fix_s'][DRIFT]))

fig, axes = st.figure(width=st.SINGLE, height=3.15, nrows=4,
                      gridspec_kw={'height_ratios': [0.14, 1.0, 0.30, 1.0],
                                   'hspace': 0.07})
axR, axA, axS, axB = axes
axS.set_axis_off()

# ---------------------------------------------------------------- panel (a)
# Franja de encendidos. Va pegada al eje de abajo y comparte su escala en x,
# de modo que cada caida de sigma se lee sobre el pulso que la produce.
axR.broken_barh([(a, b - a) for a, b in SEG], (0.0, 1.0),
                facecolor=st.C['green'], edgecolor='none', zorder=3)
# El ciclo de trabajo es el numero que resume el panel, asi que se escribe
# dentro del hueco mas ancho que cae en la ventana: ahi no hay barras que estorben.
gwide = max([g for g in GAP if g[0] >= W0 and g[1] <= W1],
            key=lambda g: g[1] - g[0])
axR.text(0.5 * (gwide[0] + gwide[1]), 0.5, '%.2f %% duty cycle' % (100 * ON_FRAC),
         ha='center', va='center', fontsize=6.8, color=st.C['grey'])
axR.set_xlim(W0, W1)
axR.set_ylim(0, 1)
for s in axR.spines.values():
    s.set_visible(False)
axR.set_xticks([])
axR.set_yticks([0.5])
axR.set_yticklabels(['GNSS'])
# El rotulo de la franja va en verde: es lo unico que hace falta para que se lea
# que verde = receptor encendido, aqui y en el sombreado de las dos graficas.
axR.tick_params(axis='y', length=0, pad=2.0, colors=st.C['green'])
st.panel_tag(axR, '(a)', x=-0.155, y=0.9)

for a, b in SEG:                      # el mismo horario, por detras de la traza
    axA.axvspan(a, b, color=st.C['green'], alpha=0.10, lw=0, zorder=0)

axA.axhline(SIG_SAT, color=st.C['red'], lw=0.8, ls=(0, (4, 1.6)), zorder=2)
axA.axhline(SIG_FRESH, color=st.C['blue'], lw=0.6, ls=(0, (1.2, 1.4)), zorder=2)

mw = (S['t_s'] >= W0 - 6) & (S['t_s'] <= W1 + 6)
axA.plot(S['t_s'][mw], S['sigma_nav_m'][mw], color=st.C['blue'], lw=1.1,
         solid_joinstyle='round', zorder=4)

axA.set_xlim(W0, W1)
axA.set_ylim(*YLIM)
# Las marcas empiezan en el propio origen del eje: si la primera etiqueta fuera
# 400 el panel parece un eje cortado. Las menores cada 100 s dan la lectura fina
# sin anadir etiquetas.
axA.set_xticks([300, 600, 900, 1200])
axA.xaxis.set_minor_locator(MultipleLocator(100))
axA.set_yticks(YTICKS)
axA.set_xlabel('Time  [s]')
axA.set_ylabel(r'Position $\sigma_{\mathrm{nav}}$  [m]')

st.label_line(axA, W1 - 12, SIG_SAT + 1.0,
              r'$\sigma_\infty = %.1f$ m' % SIG_SAT, st.C['red'],
              ha='right', va='bottom', fontsize=7)
# La banda libre por encima del techo es el unico sitio donde cabe decir que esto
# es un recorte; sin ese aviso el panel se lee como si fuera todo el registro.
# Sin halo: cae sobre el sombreado de encendido y el halo le abriria muescas
# blancas; no hay ninguna linea debajo de la que haya que despegarlo.
st.label_line(axA, W0 + 10, SIG_SAT + 1.0,
              '%.0f–%.0f s of the %.0f s record' % (W0, W1, M['window_s']),
              st.C['grey'], ha='left', va='bottom', fontsize=6.8, halo=False)
# El suelo se rotula por debajo de su propia referencia y en el centro del hueco
# ancho: es la unica franja del panel donde no hay traza, sombreado ni rejilla.
st.label_line(axA, 0.5 * (gwide[0] + gwide[1]), 2.2,
              r'%.1f m just after a fix' % SIG_FRESH, st.C['blue'],
              ha='center', va='center', fontsize=6.8)
st.label_line(axA, 0.5 * (gshort[0] + gshort[1]), PEAK_SHORT + 0.7,
              '%.0f s gap' '\n' '%.1f m' % (gshort[1] - gshort[0], PEAK_SHORT),
              st.C['ink'], ha='center', va='bottom', fontsize=6.8,
              linespacing=1.05)
st.finish(axA)

# ---------------------------------------------------------------- panel (b)
# Todas las muestras crudas, no solo el resumen: que caigan sobre una sola
# curva es la prueba de que existe una ley, y ningun promedio la ensena.
axB.axhline(SIG_SAT, color=st.C['red'], lw=0.8, ls=(0, (4, 1.6)), zorder=2)
axB.axhline(SIG_FRESH, color=st.C['blue'], lw=0.6, ls=(0, (1.2, 1.4)), zorder=2)
axB.plot(S['time_since_fix_s'][DRIFT], S['sigma_nav_m'][DRIFT], ls='none',
         marker='o', ms=1.9, mfc=st.C['lgrey'], mec='none', zorder=3)
axB.plot(S['time_since_fix_s'][FIX], S['sigma_nav_m'][FIX], ls='none',
         marker='o', ms=1.9, mfc=st.C['green'], mec='none', zorder=4)
axB.plot(BIN_X, L['sigma_nav_m'], ls='none', marker='o', ms=3.2,
         mfc=st.C['blue'], mec='white', mew=0.6, zorder=5)

axB.set_xlim(-8, 304)
axB.set_ylim(*YLIM)
axB.set_xticks([0, 60, 120, 180, 240, 300])
axB.xaxis.set_minor_locator(MultipleLocator(30))
axB.set_yticks(YTICKS)
axB.set_xlabel('Time since last GNSS fix  [s]')
axB.set_ylabel(r'Position $\sigma_{\mathrm{nav}}$  [m]')

# La columna verde en x = 0 es la unica parte del panel que no es deriva libre;
# rotularla es lo que explica que la ley arranque en 4.8 m y no en cero. El
# rotulo va justo debajo de la columna y sin guia: cualquier linea de union
# saldria pegada a la referencia de 4.8 m y la taparia.
st.label_line(axB, 4.0, 2.3, 'receiver on', st.C['green'],
              ha='left', va='center', fontsize=6.8)
st.label_line(axB, 300, SIG_SAT + 1.0, r'$\sigma_\infty = %.1f$ m' % SIG_SAT,
              st.C['red'], ha='right', va='bottom', fontsize=7)
st.label_line(axB, 302, 8.0,
              'median per 30 s bin  ($n$ = %d–%d)'
              % (L['count'].min(), L['count'].max()),
              st.C['blue'], ha='right', va='center', fontsize=6.8)
st.label_line(axB, 302, 2.3,
              '%d samples from all %d gaps' % (DRIFT.sum(), len(GAP)),
              st.C['grey'], ha='right', va='center', fontsize=6.8)
# Las dos guias arrancan en la esquina superior izquierda del texto (relpos):
# por defecto matplotlib traza la linea desde el centro de la caja y la recorta
# contra el borde, con lo que acaba en el aire por encima del rotulo.
axB.annotate(r'within 1 %% of $\sigma_\infty$' '\n' r'after %.0f s' % T99,
             xy=(T99, 0.99 * SIG_SAT), xytext=(T99 + 15, 28.0),
             fontsize=6.8, color=st.C['ink'], ha='left', va='top',
             arrowprops=dict(arrowstyle='-', lw=0.6, color=st.C['grey'],
                             relpos=(0.0, 1.0), shrinkA=1.5, shrinkB=1.5))
# El pico del hueco corto del panel (a) es un punto de esta misma curva. Decirlo
# aqui es lo que convierte (b) en el plegado de (a) y no en otra grafica: el
# hueco corto no llega al techo porque se corta a los 39 s, no por otra razon.
axB.annotate('the %.0f s gap of (a) stops here' % (gshort[1] - gshort[0]),
             xy=(PEAK_SHORT_TSF, PEAK_SHORT),
             xytext=(PEAK_SHORT_TSF + 17, 18.0),
             fontsize=6.8, color=st.C['ink'], ha='left', va='top',
             arrowprops=dict(arrowstyle='-', lw=0.6, color=st.C['grey'],
                             relpos=(0.0, 1.0), shrinkA=1.5, shrinkB=1.5))
st.finish(axB)
st.panel_tag(axB, '(b)', x=-0.155, y=1.02)

st.save(fig, 'fig05_navigation')

# --------------------------------------------------------------- trazabilidad
print('    receiver on            : %.2f %% of the %.0f s window'
      % (100.0 * ON_FRAC, M['window_s']))
GAPLEN = sorted({'%.0f' % (b - a) for a, b in GAP}, key=float)
print('    on/off pattern         : %d bursts of %.0f s, gaps of %s s'
      % (len(SEG), np.median([b - a for a, b in SEG]), '/'.join(GAPLEN)))
print('    sigma just after a fix : %.2f m' % SIG_FRESH)
print('    sigma saturated        : %.2f m  (x%.2f)'
      % (SIG_SAT, SIG_SAT / SIG_FRESH))
print('    within 1 %% of sigma_inf: %.0f s after the last fix' % T99)
print('    shortest gap (%.0f s)   : peaks at %.2f m after %.0f s without a fix,'
      ' %.0f %% of sigma_inf'
      % (gshort[1] - gshort[0], PEAK_SHORT, PEAK_SHORT_TSF,
         100.0 * PEAK_SHORT / SIG_SAT))
print('    panel (a) window       : %.0f-%.0f s (contains the 40/182/300 s gaps)'
      % (W0, W1))
print('    panel (b) samples      : %d free-drift + %d with a fix = %d, %d bins,'
      ' n = %d-%d per bin'
      % (DRIFT.sum(), FIX.sum(), len(S['t_s']), len(L['count']),
         L['count'].min(), L['count'].max()))
print('    saw-tooth collapse     : %d gaps agree to %.1e m at equal time'
      ' since fix' % (len(GAP), COLLAPSE))
print('    sigma with the receiver on: median %.2f m over %d samples'
      % (np.median(S['sigma_nav_m'][FIX]), FIX.sum()))
