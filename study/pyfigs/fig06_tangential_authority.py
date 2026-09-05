"""Figura 6: un impulso tangencial pierde autoridad en un encuentro de frente.

El guiado abre la distancia de paso con un unico impulso tangencial medio periodo
antes del encuentro. Ese impulso cambia el semieje y desplaza al satelite A LO
LARGO DE LA TRAYECTORIA; lo que llega al plano del encuentro es solo la parte de
ese desplazamiento que cae en dicho plano, y esa proyeccion va como cos(dInc/2).
La velocidad relativa de un cruce de planos va como 2*v*sin(dInc/2). Las dos
cosas son la misma variable vista de dos maneras, asi que la autoridad tiene una
cota cerrada en funcion de v_rel que se cierra a cero cuando v_rel -> 2*v.

Por que esta figura y no un scatter: hay 1500 geometrias en 3.45 in, y dibujarlas
como puntos da una mancha azul sin estructura. Con densidad hexagonal se ve lo
unico que importa: la nube esta pegada por debajo de la cota, ninguna geometria
la cruza, y la cota se derrumba en el ultimo tercio del eje. La mediana por bin
se queda entre 0.49 y 0.66 de la cota en los ocho bins, o sea que el derrumbe no
es un efecto del muestreo sino de la propia cota. El color es un recuento, y el
muestreo uniforme en dInc concentra casos a v_rel alta porque v_rel va como
sin(dInc/2); esa parte del gradiente es del muestreo, no de la fisica.

La malla hexagonal se deja deliberadamente gruesa y el color, en cuatro clases.
Con 1500 puntos una malla fina deja tres o cuatro casos por celda: el mapa se
llena de ruido de Poisson y de huecos vacios dentro de la nube, y el lector lee
como estructura lo que solo es el sorteo. Con la malla de 14x8 la mediana sube a
6 casos por hexagono y el pico a 53, y las clases se tragan el ruido que queda.

El panel (a) es pura geometria y no depende de la celda (sigma_nav, sigma_obj);
el panel (b) si, porque d_target = R + k*sigma la fija, asi que la celda va
escrita dentro del panel (b). Traduce la geometria a coste: el mismo requisito de
separacion cuesta unas tres veces mas delta-v en la banda de cabeza. Es el limite
operativo que el articulo declara para la maniobra de un solo impulso tangencial.
"""
import numpy as np
import matplotlib.patheffects as pe
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.ticker import FixedLocator, FuncFormatter

import inoas_style as st

st.use()
D = st.read_csv('mc_cases.csv')
M = st.meta()

# La geometria se repite identica en las 24 celdas (sigma_nav, sigma_obj), asi
# que una sola celda ya contiene las 1500 geometrias distintas del Monte Carlo.
SIG_NAV, SIG_OBJ = 4.8, 10.0
cell = (D['sigma_nav_m'] == SIG_NAV) & (D['sigma_obj_m'] == SIG_OBJ)
vrel = D['vrel_ms'][cell] / 1000.0
sens = np.abs(D['sens_m_per_mms'][cell])
miss0 = D['miss0_m'][cell]
dtar = D['d_target_m'][cell]
dv = np.abs(D['dv_ms'][cell]) * 1000.0
need = miss0 < dtar

T_ORB = M['T_orb_s']
V_ORB = 7.188                 # km/s, velocidad orbital circular del caso de estudio
VMAX = 2.0 * V_ORB            # cruce de frente, dInc = 180 deg
K_SIG = M['k_sigma']


def envelope(v):
    """Cota de autoridad 3*T_orb*cos(dInc/2) escrita en funcion de v_rel."""
    c = np.sqrt(np.maximum(1.0 - (v / VMAX) ** 2, 0.0))
    return 3.0 * T_ORB * c / 1000.0


# Bins comunes a los dos paneles: comparten eje x, luego tienen que compartir
# tambien la particion o las dos curvas resumen no serian comparables.
EDGES = np.linspace(2.4, 14.4, 9)
CTR = 0.5 * (EDGES[:-1] + EDGES[1:])


def binstat(x, y, q):
    out = np.full(len(CTR), np.nan)
    for i in range(len(CTR)):
        m = (x >= EDGES[i]) & (x < EDGES[i + 1])
        if m.sum() >= 8:
            out[i] = np.percentile(y[m], q)
    return out


med_s = binstat(vrel, sens, 50)
med_dv = binstat(vrel[need], dv[need], 50)
lo_dv = binstat(vrel[need], dv[need], 10)
hi_dv = binstat(vrel[need], dv[need], 90)

# Color en cuatro clases y no en rampa continua. Con 1500 casos la mediana de un
# hexagono es del orden de 6, asi que la desviacion de Poisson es casi la mitad
# del recuento: una rampa continua pinta ese ruido como si fuera relieve y dos
# celdas vecinas con 5 y 9 casos salen de colores claramente distintos sin que
# haya nada fisico detras. Con clases 1-2 / 3-9 / 10-29 / >=30 el ruido se queda
# dentro de la clase y solo se ve el gradiente que aguanta el muestreo. Los
# cuatro colores son los de la paleta y su claridad es monotona, asi que la
# figura sigue separandose impresa en gris.
LEVELS = [1, 3, 10, 30]
CMAP = ListedColormap([st.C['vlgrey'], st.C['lgrey'], st.C['blue'], st.C['ink']])

XLIM = (2.3, 14.7)            # el muestreo llega a 14.15 km/s; 14.7 deja sitio a 2v
YLIM_A = (0.0, 21.0)

# st.figure() es lo que marca el tamano exacto de la caja recortada; llamarla y
# quitar el eje unico sale mas barato que reproducir aqui el ajuste de save().
fig, _ax0 = st.figure(width=st.SINGLE, height=3.05)
_ax0.remove()
gs = fig.add_gridspec(2, 2, width_ratios=[1.0, 0.040],
                      height_ratios=[1.28, 1.0], hspace=0.16, wspace=0.055)
axA = fig.add_subplot(gs[0, 0])
cax = fig.add_subplot(gs[0, 1])
axB = fig.add_subplot(gs[1, 0], sharex=axA)

# ---------------------------------------------------------------- panel (a)
hb = axA.hexbin(vrel, sens, gridsize=(14, 8), extent=(XLIM[0], XLIM[1],
                YLIM_A[0], YLIM_A[1]), mincnt=1, cmap=CMAP, linewidths=0.0,
                zorder=2)
# Cortes en progresion logaritmica: el pico esta en la punta donde la cota se
# cierra y unos cortes lineales dejarian en la misma clase todo lo demas.
CNT = np.ma.compressed(hb.get_array())   # recuentos de los hexagonos ocupados
CNT_MAX = float(CNT.max())
hb.set_norm(BoundaryNorm(LEVELS + [CNT_MAX + 1], CMAP.N))

# Los hexagonos son celdas finitas y sangran por encima de la cota justo en la
# punta, donde la curva es casi vertical. Se tapa con blanco por encima de la
# cota analitica: el borde de la nube queda exactamente donde esta el limite y
# no se inventa ni se esconde ningun caso, porque ninguno la cruza. Contra la
# cota tal como se dibuja, la escrita en funcion de v_rel, el maximo de
# |sens|/cota es 0.9995 en las 1500 geometrias. (Evaluada en cambio con el dInc
# de cada fila da 1.0006, pero esa no es la curva que se pinta.)
vm = np.linspace(XLIM[0], XLIM[1], 600)
axA.fill_between(vm, envelope(vm), YLIM_A[1], color='white', lw=0, zorder=3)

vv = np.linspace(XLIM[0], VMAX, 400)
axA.plot(vv, envelope(vv), color=st.C['red'], lw=1.1, zorder=4)
axA.axvline(VMAX, color=st.C['grey'], lw=0.7, ls=(0, (1.2, 1.2)), zorder=4)
# Halo blanco: la clase mas densa del mapa es del mismo color que esta linea, y
# sin el la mediana se perdia justo en la punta, que es donde hay que leerla.
mline, = axA.plot(CTR, med_s, color=st.C['ink'], lw=0.9, marker='o', ms=2.6,
                  markerfacecolor='white', markeredgewidth=0.7, zorder=5)
mline.set_path_effects([pe.withStroke(linewidth=2.3, foreground='white'),
                        pe.Normal()])

axA.set_xlim(*XLIM)
axA.set_ylim(*YLIM_A)
axA.set_yticks([0, 5, 10, 15, 20])
# Se dibuja el modulo: sens cambia de signo con la geometria del plano b y lo que
# limita la cota es la magnitud, no el sentido.
axA.set_ylabel('Miss opened per impulse\n'
               r'$|\partial d/\partial\Delta v|$  [m per mm s$^{-1}$]')
# Sin rejilla en (a): sobre un mapa de densidad no aporta y obligaria a dejar la
# mascara blanca por debajo, que es justo lo que hay que tapar.
st.finish(axA, grid='none')
# labelbottom, no setp: con sharex cualquier retoque posterior de los ticks de
# axB regenera las etiquetas de axA y las volveria a hacer visibles.
axA.tick_params(axis='x', labelbottom=False)

st.label_line(axA, 8.4, 17.3, 'authority bound\n$3\\,T_{\\mathrm{orb}}\\cos(\\Delta i/2)$',
              st.C['red'], ha='left', va='bottom', fontsize=7, linespacing=1.2)
st.label_line(axA, 2.9, 11.7, 'binned median', st.C['ink'],
              ha='left', va='bottom', fontsize=6.8)
# v_orb y no v a secas: v sin subindice no esta definido en ningun sitio de la
# figura y el lector no tiene por que ir a buscarlo al pie.
axA.text(14.02, 12.3, 'head-on, $v_{\\mathrm{rel}} = 2v_{\\mathrm{orb}}$',
         rotation=90, ha='center', va='center', fontsize=6.8, color=st.C['grey'])
st.panel_tag(axA, '(a)', x=-0.175, y=1.02)

cb = fig.colorbar(hb, cax=cax, spacing='uniform')
cb.outline.set_linewidth(0.6)
cb.outline.set_edgecolor(st.C['ink'])
# Las marcas caen en las fronteras entre clases: son cuatro bloques iguales y el
# de arriba se lee como '30 o mas'.
cb.ax.yaxis.set_major_locator(FixedLocator(LEVELS))
cb.ax.yaxis.set_minor_locator(FixedLocator([]))
cb.ax.yaxis.set_major_formatter(FuncFormatter(lambda v, p: '%g' % v))
cb.ax.tick_params(labelsize=6.5, width=0.6, length=2.0, pad=1.5)
# 'per bin' seria ambiguo: en esta misma figura 'bin' es el bin de v_rel de la
# mediana. El recuento es por hexagono.
cb.set_label('Geometries per hexagon', fontsize=7, labelpad=3)

# ---------------------------------------------------------------- panel (b)
axB.fill_between(CTR, lo_dv, hi_dv, color=st.C['blue'], alpha=0.16, lw=0,
                 zorder=2)
axB.plot(CTR, med_dv, color=st.C['blue'], lw=1.1, marker='o', ms=2.8,
         markerfacecolor='white', markeredgewidth=0.7, zorder=4)

axB.set_xlim(*XLIM)
axB.set_ylim(0, 27)
axB.set_yticks([0, 10, 20])
axB.set_xticks([4, 6, 8, 10, 12, 14])
axB.set_xlabel('Encounter relative velocity, $v_{\\mathrm{rel}}$  [km s$^{-1}$]')
axB.set_ylabel('Planned impulse\n$|\\Delta v|$  [mm s$^{-1}$]')
axB.axvline(VMAX, color=st.C['grey'], lw=0.7, ls=(0, (1.2, 1.2)), zorder=1)
st.finish(axB, grid='y')

# El rotulo de la banda no cabe sobre su propio borde: la banda sube y el texto
# acababa cortado por el percentil 90. Se saca al hueco libre y se ata con una
# guia corta que arranca DENTRO del relleno: si solo lo roza por fuera se lee
# como una barra de error suelta en vez de como una llamada.
st.label_line(axB, 3.7, 11.0, u'10–90 %', st.C['blue'], ha='left', va='bottom',
              fontsize=6.8)
axB.plot([4.35, 4.35], [6.6, 10.7], color=st.C['blue'], lw=0.5, zorder=3,
         solid_capstyle='butt')
# Sin halo: el rotulo cae dentro del relleno palido, donde el azul se lee solo, y
# el halo se comia el hueco que lo separa del marcador que tiene encima.
st.label_line(axB, 6.7, 4.1, 'median', st.C['blue'], ha='left', va='top',
              halo=False, fontsize=6.8)
# El panel (b) SI depende de la celda, porque d_target = R + k*sigma sale de ella:
# con sigma_obj = 300 m maniobran las 1500 geometrias. Si la celda no se escribe,
# el 755 se lee como una propiedad del escenario, y no lo es.
axB.text(2.4, 26.6, ('%d of %d geometries need a manoeuvre\n'
         r'($\sigma_{\mathrm{nav}} = %.1f$ m, $\sigma_{\mathrm{obj}} = %.0f$ m)')
         % (int(need.sum()), need.size, SIG_NAV, SIG_OBJ), fontsize=6.8,
         color=st.C['grey'], ha='left', va='top', linespacing=1.25)
st.panel_tag(axB, '(b)', x=-0.175, y=1.02)

st.save(fig, 'fig06_tangential_authority')

# ------------------------------------------------------------------ numeros
near = sens > 0.9 * envelope(vrel)
print('    v_rel head-on 2v            = %.3f km/s' % VMAX)
print('    bound at 2.5 / 10 / 14 km/s = %.2f / %.2f / %.2f m per mm/s'
      % (envelope(2.5), envelope(10.0), envelope(14.0)))
print('    max |sens| in the sample    = %.2f m per mm/s' % sens.max())
print('    geometries within 10 %% of the bound = %d of %d (%.1f %%)'
      % (near.sum(), near.size, 100.0 * near.mean()))
print('    bin [km/s]  n   med|s|  bound  ratio   n_man  med dv  p90 dv')
for i, c in enumerate(CTR):
    m = (vrel >= EDGES[i]) & (vrel < EDGES[i + 1])
    print('    %4.1f-%4.1f %5d %7.2f %6.2f %6.2f %6d %7.2f %7.2f'
          % (EDGES[i], EDGES[i + 1], m.sum(), med_s[i], envelope(c),
             med_s[i] / envelope(c), (m & need).sum(), med_dv[i], hi_dv[i]))
print('    median dv, first -> last bin = %.2f -> %.2f mm/s (x%.2f)'
      % (med_dv[0], med_dv[-1], med_dv[-1] / med_dv[0]))
print('    p90 dv,    first -> last bin = %.2f -> %.2f mm/s (x%.2f)'
      % (hi_dv[0], hi_dv[-1], hi_dv[-1] / hi_dv[0]))
print('    k_sigma = %.0f, cell sigma_nav = %.1f m, sigma_obj = %.0f m'
      % (K_SIG, SIG_NAV, SIG_OBJ))
print('    hexbin 14x8: %d occupied hexagons, median %.0f cases, max %.0f'
      % (CNT.size, np.median(CNT), CNT_MAX))
