"""Figura 3: una restriccion muestreada en la rejilla no ve la conjuncion.

La version anterior de esta figura sostenia que meter la restriccion de colision
dentro del QP no cierra en tiempo real, y lo apoyaba en un banco de tiempos. Ese
argumento se ha retirado por dos razones que la auditoria dejo firmes:

  1. Los tiempos publicados (14.9 % fuera de plazo, peores casos de 45 y 77 s) se
     midieron con codigo anterior a a7701b4 y 8d55826. Relanzado el mismo banco
     contra HEAD, las cuatro configuraciones cumplen su plazo, con 100 % de
     convergencia y 0.86 s de peor caso.
  2. Peor aun: con el codigo actual la restriccion no influye en la solucion.
     Resolviendo con dsafe0 = 150 m y con dsafe0 = 0 el mando difiere 9.9e-09
     m/s^2 sobre 3.0e-04 m/s^2, con la nominal pasando a 50 m del objeto. El QP
     que se cronometraba era de seguimiento, no de evitacion.

El argumento que si aguanta no es de coste sino de correccion, y es mas fuerte: a
las velocidades de una conjuncion real la restriccion muestreada es CIEGA. El
objeto permanece dentro de la esfera de exclusion durante 2d/v_rel, que a 10 km/s
son 34 ms, mientras el guiado la evalua cada h segundos. Si el instante del
encuentro es uniforme respecto a la rejilla, la probabilidad de que alguna
muestra caiga dentro es min(1, 2d/(v_rel*h)): un 1.1 % con h = 3 s y un 0.11 %
con el paso de 30 s desplegado. Reducir h no lo arregla, porque la probabilidad
solo crece como 1/h.

De ahi la arquitectura: la restriccion hay que imponerla DONDE ocurre el minimo,
resolviendo el instante de maxima aproximacion de forma continua, y no en los
nudos de una rejilla que casi con seguridad no lo contienen.

(a) Permanencia dentro de la zona de exclusion frente a la velocidad de
    encuentro, para las 1500 geometrias muestreadas, contra los pasos de guiado.
(b) Probabilidad de que la rejilla vea algo, en funcion del paso.
"""
import numpy as np

import inoas_style as st

st.use()
D = st.read_csv('mc_cases.csv')
M = st.meta()

# Una celda basta: la geometria se repite en las 24 combinaciones de sigma
cell = (D['sigma_nav_m'] == 4.8) & (D['sigma_obj_m'] == 10)
VREL = D['vrel_ms'][cell]
DTGT = D['d_target_m'][cell]
DWELL = 2.0 * DTGT / VREL                      # [s] dentro de la esfera
STEPS = [3.0, 10.0, 30.0, 60.0]
H_OP = M['execution']['guidance_step_s']
d_q = np.median(DTGT)

fig, (axA, axB) = st.figure(width=st.DOUBLE, height=2.5, ncols=2,
                            gridspec_kw={'wspace': 0.32})

# ---------------------------------------------------------------- panel (a)
for h in STEPS:
    col = st.C['ink'] if h == H_OP else st.C['grey']
    axA.axhline(h * 1e3, color=col, lw=0.8,
                ls='-' if h == H_OP else (0, (3.5, 1.8)), zorder=2)
    axA.annotate('$h$ = %g s' % h, (14.5, h * 1e3), fontsize=6.6, color=col,
                 ha='right', va='bottom', xytext=(0, 1.5),
                 textcoords='offset points')

axA.scatter(VREL / 1e3, DWELL * 1e3, s=4, color=st.C['blue'], alpha=0.20,
            linewidths=0, zorder=3)
vv = np.linspace(2.4, 14.4, 200)
axA.plot(vv, 2 * np.median(DTGT) / (vv * 1e3) * 1e3, '-', color=st.C['red'],
         lw=1.2, zorder=4)
st.label_line(axA, 4.4, 2 * np.median(DTGT) / 4.4e3 * 1e3 * 1.6,
              r'$2\,d_{\mathrm{safe}}/v_{\mathrm{rel}}$', st.C['red'],
              ha='left', va='bottom', fontsize=7)

# El hueco vacio del panel ES el argumento, asi que se acota y se mide en vez de
# dejarlo como espacio muerto.
v_ann = np.median(VREL) / 1e3
# La MEDIANA DE LA PERMANENCIA, no la permanencia en la mediana de v_rel: son dos
# cuentas distintas (886 frente a 892) y el pie de figura cita esta.
d_ann = np.median(DWELL) * 1e3
axA.annotate('', xy=(v_ann, d_ann), xytext=(v_ann, H_OP * 1e3),
             arrowprops=dict(arrowstyle='<->', lw=0.7, color=st.C['ink'],
                             shrinkA=0, shrinkB=0))
st.label_line(axA, v_ann + 0.35, np.sqrt(d_ann * H_OP * 1e3),
              '%.0f$\\times$ longer' % (H_OP * 1e3 / d_ann), st.C['ink'],
              ha='left', va='center', fontsize=7)

axA.set_yscale('log')
axA.set_xlim(2, 14.8)
axA.set_ylim(15, 3e5)
axA.set_yticks([1e2, 1e3, 1e4, 1e5])
axA.set_xlabel(r'Encounter relative velocity, $v_{\mathrm{rel}}$  [km s$^{-1}$]')
axA.set_ylabel('Time the object spends inside\nthe keep-out sphere  [ms]')
st.finish(axA)
st.panel_tag(axA, '(a)', x=-0.19)

# ---------------------------------------------------------------- panel (b)
hh = np.logspace(np.log10(0.5), np.log10(120), 200)
for q, lab, col, dash in [(90, r'$v_{\mathrm{rel}}$ p90', st.C['red'], (1.4, 1.4)),
                          (50, 'median', st.C['blue'], (None, None)),
                          (10, 'p10', st.C['green'], (4, 1.6))]:
    v_q = np.percentile(VREL, q)
    axB.plot(hh, 100 * np.minimum(1.0, 2 * d_q / (v_q * hh)), color=col,
             lw=1.2, dashes=dash, zorder=3, label=lab)

axB.axvline(H_OP, color=st.C['ink'], lw=0.8, zorder=2)
p_op = 100 * min(1.0, 2 * d_q / (np.median(VREL) * H_OP))
axB.plot([H_OP], [p_op], 'D', color=st.C['blue'], markeredgecolor='white',
         markersize=4.4, zorder=5)
axB.annotate('deployed $h$ = %g s:  %.2f %%' % (H_OP, p_op), (H_OP, p_op),
             textcoords='offset points', xytext=(-7, 7), ha='right',
             fontsize=6.8, color=st.C['ink'])

axB.set_xscale('log')
axB.set_yscale('log')
axB.set_xlim(0.5, 120)
axB.set_ylim(0.02, 40)
axB.set_xticks([1, 3, 10, 30, 100])
axB.set_yticks([0.1, 1, 10])
st.plain_log(axB, 'x')
st.plain_log(axB, 'y')
axB.set_xlabel('Guidance step  $h$  [s]')
axB.set_ylabel('Chance the grid ever sees the\nobject inside the sphere  [%]')
st.finish(axB, legend=True, loc='upper right')
st.panel_tag(axB, '(b)', x=-0.19)

st.save(fig, 'fig03_architecture_cost')

# ---------------------------------------------------------------- numeros
print('  %d geometrias, v_rel %.2f a %.2f km/s (mediana %.2f)'
      % (VREL.size, VREL.min() / 1e3, VREL.max() / 1e3, np.median(VREL) / 1e3))
print('  d_safe: mediana %.1f m (p10 %.1f, p90 %.1f)'
      % (d_q, np.percentile(DTGT, 10), np.percentile(DTGT, 90)))
print('  permanencia dentro de la esfera: mediana %.1f ms (p10 %.1f, p90 %.1f)'
      % (1e3 * np.median(DWELL), 1e3 * np.percentile(DWELL, 10),
         1e3 * np.percentile(DWELL, 90)))
print('  P(la rejilla ve algo), por paso de guiado:')
for h in [1.0] + STEPS:
    p = np.minimum(1.0, DWELL / h)
    print('     h = %5.1f s : mediana %6.3f %%   p90 %6.3f %%   mejor caso %6.3f %%'
          % (h, 100 * np.median(p), 100 * np.percentile(p, 90), 100 * p.max()))
print('  con el paso desplegado h = %g s la mediana es %.3f %%' % (H_OP, p_op))
