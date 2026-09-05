"""Figura 2: el radio de exclusion tiene que ser funcion del tiempo, no del mallado.

El radio inflado es d0 + k*sigma(t), y sigma(t) sale de propagar la covarianza a
lo largo del horizonte de prediccion. El codigo heredado sumaba un incremento de
ruido de proceso FIJO POR PASO, de modo que la incertidumbre acumulada acababa
siendo proporcional al numero de pasos: partir el mismo horizonte de 500 s en
125, 100 o 50 tramos daba tres radios distintos. Discretizar la PSD continua
sobre el paso con van Loan elimina esa dependencia, porque la integral del ruido
de proceso sobre el intervalo no sabe nada de como se ha troceado.

Se dibuja asi por dos motivos:

  * El color codifica el METODO (rojo = incremento por paso, verde = van Loan) y
    el marcador codifica el PASO. Con esa separacion, el abanico rojo y la curva
    verde unica se leen de un vistazo, que es justamente el mensaje.
  * Las tres curvas verdes son identicas hasta el ultimo digito del dato, asi que
    superpuestas parecen una sola. Para que no se lea como que solo hay una curva
    se hacen dos cosas: marcadores distintos escalonados en t sobre la misma
    linea, y el panel (b), que mide directamente la dispersion max-min entre
    mallados. La dispersion roja crece hasta 41 m; la verde es cero en todos los
    instantes comunes, con la precision con la que el CSV guarda los radios.

El panel (b) es el que da el numero: 41.3 m de error de modelado a 500 s, solo
por cambiar el mallado, sobre un radio que ademas queda infravalorado respecto al
valor correcto.
"""
import matplotlib.lines as mlines
import numpy as np

import inoas_style as st

st.use()
D = st.read_csv('koz_profiles.csv')
M = st.meta()

# --- los tres mallados del MISMO horizonte de 500 s -----------------------
BY_H = {}
for c in np.unique(D['case']):
    m = D['case'] == c
    h = float(D['step_h_s'][m][0])
    BY_H[h] = {'t': D['t_into_horizon_s'][m],
               'ps': D['radius_perstep_m'][m],
               'vl': D['radius_vanloan_m'][m]}
HS = sorted(BY_H)                                   # 4, 5, 10 s

MK = {4.0: 'o', 5.0: 's', 10.0: '^'}
# Marcadores escalonados en t: sobre la curva verde unica los tres juegos tienen
# que quedar visibles uno al lado de otro, no encima.
MARK_T = {4.0: [78, 208, 338, 448], 5.0: [110, 240, 370, 480],
          10.0: [140, 270, 400, 500]}

D0 = M['d_safe0_m']
K = M['k_sigma']
T_END = 500.0

fig, (axA, axB) = st.figure(width=st.SINGLE, height=3.05, nrows=2, sharex=True,
                            gridspec_kw={'height_ratios': [2.7, 1.0],
                                         'hspace': 0.15})

# ------------------------------------------------------------------ panel (a)
for h in HS:
    d = BY_H[h]
    idx = [int(np.argmin(np.abs(d['t'] - tt))) for tt in MARK_T[h]]
    for key, col, z in (('ps', st.C['red'], 3), ('vl', st.C['green'], 4)):
        axA.plot(d['t'], d[key], color=col, zorder=z)
        axA.plot(d['t'][idx], d[key][idx], color=col, marker=MK[h], lw=0,
                 markerfacecolor=col, markeredgecolor='white', zorder=z + 2)

axA.set_xlim(0, 512)
axA.set_ylim(146, 405)
axA.set_yticks([150, 200, 250, 300, 350, 400])
axA.set_ylabel('Inflated keep-out radius\n$d_0 + k\\,\\sigma(t)$  [m]')

# Los rotulos van en coordenadas de datos y no en fraccion de eje: asi se pueden
# meter en los huecos entre lineas de rejilla, que estan cada 50 m, en vez de
# quedar cruzados por una de ellas.
st.label_line(axA, 8, 396, '$d_0$ = %g m,  $k$ = %g' % (D0, K), st.C['grey'],
              ha='left', va='top', fontsize=6.8)
st.label_line(axA, 200, 386, 'van Loan: three meshes, one curve', st.C['green'],
              ha='left', va='center', fontsize=6.8)
# El rotulo rojo se alinea por la derecha y va corto: la franja libre bajo el
# abanico solo es lo bastante alta al final del horizonte.
st.label_line(axA, 502, 157, 'per-step: three curves', st.C['red'],
              ha='right', va='bottom', fontsize=6.8)

# Leyenda solo del paso, y sin linea en el simbolo: el trazo codifica el metodo
# (rojo o verde), asi que una linea negra en la leyenda anunciaria una tercera
# curva que no existe. Solo el marcador, en tinta neutra.
handles = [mlines.Line2D([], [], color='none', marker=MK[h], lw=0,
                         markerfacecolor=st.C['ink'], markeredgecolor='white',
                         markersize=4.2, label='$h$ = %g s' % h) for h in HS]
_lg = st.finish(axA, legend=True, handles=handles, loc='upper left',
                bbox_to_anchor=(0.0, 0.90))
# Las filas de la leyenda van cada 20 m y la rejilla cada 50 m: no hay altura a la
# que las tres queden holgadas entre lineas, asi que alguna acaba cruzada por uno
# de los punteados. Fondo blanco sin borde, que abre un hueco limpio en la linea;
# es el mismo recurso que el halo de los rotulos, solo que rectangular.
_lg.set_frame_on(True)
_lg.get_frame().set_facecolor('white')
_lg.get_frame().set_edgecolor('none')
_lg.get_frame().set_alpha(1.0)

# El hueco entre el abanico rojo y la curva verde es la otra mitad del problema:
# el esquema por paso no solo depende del mallado, ademas se queda corto. El
# rotulo dice per-step y no legacy para no dar dos nombres a la misma curva.
_ps_end = np.array([BY_H[h]['ps'][-1] for h in HS])
_vl_end = BY_H[HS[0]]['vl'][-1]
SHORT_LO = _vl_end - _ps_end.max()
SHORT_HI = _vl_end - _ps_end.min()
st.label_line(axA, 8, 296, 'per-step also under-inflates:\n%.0f to %.0f m short at $t$ = 500 s'
              % (SHORT_LO, SHORT_HI), st.C['grey'],
              ha='left', va='top', fontsize=6.8, linespacing=1.35)
st.panel_tag(axA, '(a)', x=-0.185, y=1.0)

# ------------------------------------------------------------------ panel (b)
# Dispersion entre mallados sobre los instantes que los tres comparten.
tc = np.array(sorted(set(BY_H[4.0]['t']) & set(BY_H[5.0]['t']) & set(BY_H[10.0]['t'])))
spread = {}
for key in ('ps', 'vl'):
    s = []
    for tt in tc:
        v = [BY_H[h][key][int(np.argmin(np.abs(BY_H[h]['t'] - tt)))] for h in HS]
        s.append(max(v) - min(v))
    spread[key] = np.asarray(s)

axB.fill_between(tc, 0, spread['ps'], color=st.C['red'], alpha=0.11, lw=0, zorder=2)
axB.plot(tc, spread['ps'], color=st.C['red'], zorder=3)
axB.plot(tc, spread['vl'], color=st.C['green'], zorder=4)

axB.set_xlim(0, 512)
axB.set_ylim(-3.5, 54)
axB.set_yticks([0, 20, 40])
axB.set_xticks([0, 100, 200, 300, 400, 500])
axB.set_xlabel('Time into the 500 s prediction horizon  [s]')
axB.set_ylabel('Mesh spread of\nthe radius  [m]')

# El rotulo rojo cabe sobre el extremo de la curva; el verde tiene que quedarse a
# la izquierda y algo por encima del cero, que es la unica franja donde ni la
# curva roja ni su relleno llegan todavia.
st.label_line(axB, T_END, 43.5, 'per-step: %.1f m at $t$ = 500 s' % spread['ps'][-1],
              st.C['red'], ha='right', va='bottom', fontsize=6.8)
st.label_line(axB, 12, 8.0, 'van Loan: 0 m', st.C['green'],
              ha='left', va='bottom', fontsize=6.8)
st.finish(axB)
st.panel_tag(axB, '(b)', x=-0.185, y=1.0)

st.save(fig, 'fig02_koz_consistency')

# --- numeros que se citan en el texto ------------------------------------
print('    horizonte 500 s, d0 = %g m, k = %g' % (D0, K))
for h in HS:
    d = BY_H[h]
    print('    h = %5.1f s (%3d pasos):  per-step %7.2f m   van Loan %7.2f m'
          % (h, round(T_END / h), d['ps'][-1], d['vl'][-1]))
ps_end = np.array([BY_H[h]['ps'][-1] for h in HS])
vl_end = np.array([BY_H[h]['vl'][-1] for h in HS])
print('    dispersion a 500 s:  per-step %.2f m (%.1f %% de la media)   van Loan %.2e m'
      % (np.ptp(ps_end), 100 * np.ptp(ps_end) / ps_end.mean(), np.ptp(vl_end)))
print('    infravaloracion del per-step frente a van Loan a 500 s: %.1f a %.1f m'
      ' (en la figura, %.0f a %.0f m)'
      % ((vl_end.mean() - ps_end.max()), (vl_end.mean() - ps_end.min()),
         SHORT_LO, SHORT_HI))
print('    sigma(500 s) van Loan = %.2f m ; per-step = %.2f a %.2f m'
      % ((vl_end.mean() - D0) / K, (ps_end.min() - D0) / K, (ps_end.max() - D0) / K))
print('    dispersion per-step a 100/300 s: %.2f / %.2f m'
      % (spread['ps'][np.argmin(np.abs(tc - 100))], spread['ps'][np.argmin(np.abs(tc - 300))]))
