# Figuras del articulo

Las campanas se corren en MATLAB y dejan `.mat` en `study/out`. Las figuras del
articulo se dibujan aqui, en Python, a partir de tablas planas. En medio hay un
unico punto de contacto, `export_data.py`, de modo que ninguna figura vuelve a
abrir un `.mat` y cualquiera puede reproducir o auditar una figura con los CSV y
sin MATLAB. Los CSV valen ademas como material suplementario.

## Uso

```bash
python export_data.py     # solo si han cambiado los .mat de study/out
python make_all.py        # regenera las ocho figuras
python check_figures.py   # audita los PDF antes de enviar
```

La salida va a `study/figures_py`: PDF vectorial para LaTeX y PNG a 600 ppp para
revisar.

## Piezas

| fichero | que hace |
|---|---|
| `export_data.py` | lee `study/out/*.mat` y escribe `data/*.csv` + `data/meta.json`. Unidades explicitas en cada nombre de columna. |
| `inoas_style.py` | tipografia, paleta, acabado de ejes y guardado. Un solo sitio donde se decide como se ve una figura. |
| `figNN_*.py` | una figura cada uno. Leen solo `data/`, imprimen al final las cifras que dibujan. |
| `make_all.py` | los llama en orden y avisa si alguno falla. |
| `check_figures.py` | tamano exacto, fuentes incrustadas, nada de Type 3, nada de mapas de bits. |
| `figures.tex` | los ocho bloques `\begin{figure}` con sus pies, listos para pegar. |

## Decisiones de estilo

Times con matematicas STIX, que es lo que compone IEEEtran; marco abierto y
desplazado 3 pt; rejilla horizontal punteada por detras de los datos; paleta de
Paul Tol, distinguible con daltonismo y separable en gris, siempre acompanada de
marcador o trazo distinto porque la figura tiene que sobrevivir a una impresion
en blanco y negro.

`save()` deja el PDF con el tamano exacto que pidio `figure()`, 3.45 in de
columna o 7.16 in de pagina. Importa: `bbox_inches='tight'` recorta despues de
dibujar y devolvia PDF de 5.94 in donde se habian pedido 7.16; al estirar eso en
LaTeX hasta el ancho de pagina se estira tambien la tipografia y los 8 pt dejan
de ser 8 pt. Por eso las figuras se incluyen **sin escalar**:

```latex
\includegraphics{fig01_coupling}          % correcto
\includegraphics[width=\textwidth]{...}   % NO: reescala la letra
```

## Las ocho figuras

1. **`fig01_coupling`** (pagina). El acoplamiento navegacion-guiado esta
   condicionado por la efemeride del objeto: +3254 % de impulso mediano al
   recorrer la banda medida si el objeto se conoce a 10 m, +0.5 % si se conoce a
   300 m.
2. **`fig02_koz_consistency`** (columna). El radio de exclusion inflado tiene que
   depender del tiempo y no de la particion del horizonte. Por paso, tres
   particiones del mismo horizonte de 500 s dan tres radios que se separan hasta
   41.3 m; con van Loan las tres colapsan en una curva.
3. **`fig03_architecture_cost`** (pagina). Meter la restriccion de colision
   dentro del QP no cierra en tiempo real: el 14.9 % de los QP tarda mas que su
   paso de guiado y el peor caso llega a 77 s. Planificar fuera del QP resuelve
   en 2.8 ms de mediana con 480x de margen sobre el peor caso.
4. **`fig04_execution`** (columna). Un encuentro completo: cruce de planos de
   88.15 deg a 10.0 km/s, impulso retrogrado de 12.6 mm/s a -112.5 min, fallo de
   120.0 -> 257.4 m contra un objetivo de 257.4 m, error de seguimiento 0.67 m
   rms. El MPC gasta 19.2 mm/s para entregar un plan de 12.6: un 53 % de
   sobrecoste por seguir una referencia continua en vez de disparar un impulso.
5. **`fig05_navigation`** (columna). Lo que cuesta apagar el receptor: encendido
   el 19.45 % del tiempo, sigma pasa de 4.8 m recien corregido a 32.1 m saturado,
   y satura en 89 s. El coste de apagarlo esta acotado.
6. **`fig06_tangential_authority`** (columna). Un impulso tangencial pierde toda
   autoridad en un encuentro frontal: la envolvente va como cos(dInc/2) y se
   cierra a cero cuando v_rel tiende a 2 v_orb.
7. **`fig07_dv_distribution`** (pagina). El presupuesto de propulsante es una
   cola, no una mediana.
8. **`fig08_probability_dilution`** (columna). Con el fallo fijo, la probabilidad
   de colision BAJA al empeorar la navegacion (8.8e-4 -> 1.6e-4). Disenar contra
   un umbral de Pc premiaria tener mala navegacion; por eso el diseno infla una
   distancia con la covarianza, que si crece de forma monotona.

## Lo que estas figuras no dicen

Se deja por escrito para que no se sobreinterprete ninguna y para que el pie de
figura lo recoja donde haga falta.

- **Fig. 2**: por debajo de t ~ 150 s las tres curvas por paso se separan menos de
  2 m y se ven como una sola. Solo hay un horizonte (500 s) y tres pasos, asi que
  la figura no puede ensenar el orden de convergencia del error en h.
- **Fig. 3**: las muestras son pocas y desiguales (41 a 134 QP por configuracion),
  de modo que cada peor caso descansa sobre uno o dos QP. No conviene apoyarse en
  los cuantiles extremos. `pct_converged` en `timing_qp_cases.csv` es
  `100*mean(exitflag>0)`: cuantos QP resolvio el solver, no cuantos llegaron a
  tiempo. Son dos fracasos distintos.
- **Fig. 4**: el resultado de seguridad, 120.0 -> 257.4 m, es una anotacion y no
  una serie dibujada: `execution.csv` no lleva la distancia de maxima
  aproximacion frente al tiempo. Tampoco lleva las componentes del mando, asi que
  el caracter retrogrado del impulso solo aparece escrito.
- **Fig. 5**: el colapso del diente de sierra es exacto, las 12 ventanas coinciden
  a 1.7e-5 m, asi que el panel (b) demuestra repetibilidad de una propagacion
  determinista, no dispersion estadistica.
- **Fig. 6**: el color es un recuento crudo y el estudio muestrea dInc uniforme
  mientras v_rel va como sin(dInc/2), de modo que parte del gradiente de densidad
  es el muestreo y no la fisica (108 casos en el primer intervalo de v_rel frente
  a 336 en el ultimo). Ninguna columna registra si un caso alcanzo realmente
  d_target, asi que la figura no puede senalar que geometrias frontales son
  simplemente inviables con un solo impulso tangencial.
- **Fig. 7**: marginalizar sobre la calidad de efemeride exige un peso, y el que
  se usa es el uniforme sobre los cuatro niveles simulados. Es una hipotesis
  sobre el catalogo, no una medida; sin ella esta `mc_percentiles.csv`, celda a
  celda. La figura da impulso por encuentro: no hay tasa de conjunciones ni
  duracion de mision en los datos, asi que nada de aqui se convierte en un
  presupuesto anual ni en un tamano de deposito.
- **Fig. 8**: el barrido es el del demostrador de conjuncion aislado, con su
  propia hipotesis de covarianza del objeto (sigma_obj = 150 m). Su "fallo
  requerido" NO es la misma magnitud que la de `retarget_sweep.csv`, y los dos
  conjuntos no se mezclan en un panel. El umbral de 1e-4 es convencional y es lo
  unico de la figura que no sale de los datos.

## Nota sobre `mc_summary.mat`

`export_data.py` **no** lo lee. Lleva fecha anterior a la de los `mc_cam_*.mat`
que deberia resumir y su `fzero` de 1/1500 delata que se calculo sobre 1500 casos
por sigma, es decir con un solo nivel de `sigma_obj`, cuando los ficheros
actuales traen 6000. Sus percentiles no se reconcilian con los datos actuales.
Los percentiles se recalculan aqui a partir de las mismas filas que van a
`mc_cases.csv`, de modo que las dos tablas no pueden discrepar.
