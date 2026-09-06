# Figuras del articulo

Las campanas se corren en MATLAB y dejan `.mat` en `study/out`. Las figuras del
articulo se dibujan aqui, en Python, a partir de tablas planas. En medio hay un
unico punto de contacto, `export_data.py`, de modo que ninguna figura vuelve a
abrir un `.mat` y cualquiera puede reproducir o auditar una figura con los CSV y
sin MATLAB. Los CSV valen ademas como material suplementario.

## Uso

```bash
python export_data.py       # solo si han cambiado los .mat de study/out
python nav_duty_model.py    # regenera el barrido de ciclo de trabajo (figura 5)
python make_all.py          # regenera las ocho figuras
python check_figures.py     # audita los PDF antes de enviar
```

La salida va a `study/figures_py`: PDF vectorial para LaTeX y PNG a 600 ppp.

## Piezas

| fichero | que hace |
|---|---|
| `export_data.py` | lee `study/out/*.mat` y escribe `data/*.csv` + `data/meta.json`. Unidades explicitas en cada nombre de columna. |
| `nav_duty_model.py` | reconstruye la recursion de covarianza del filtro, se valida contra la corrida medida y barre el ciclo de trabajo. Es lo que sostiene la figura 5. |
| `inoas_style.py` | tipografia, paleta, acabado de ejes y guardado. |
| `figNN_*.py` | una figura cada uno. Leen solo `data/`, imprimen al final las cifras que dibujan. |
| `make_all.py` | los llama en orden y avisa si alguno falla. |
| `check_figures.py` | tamano exacto, fuentes incrustadas, nada de Type 3, nada de mapas de bits. |
| `figures.tex` | los ocho bloques `\begin{figure}` con sus pies, listos para pegar. |

## Convenios

**sigma_nav es el radio 3-D `sqrt(trace(P_pos))`** en todas las figuras. Parte
del codigo fuente trabaja por eje (`cam_demo.m`, `test_cam_retarget.m` montan la
covarianza como `diag([s s s].^2)`); la conversion `sqrt(3)` esta hecha en los
CSV y anotada en su cabecera. Antes no lo estaba, y la figura 8 llego a sombrear
la banda medida 4.8-32.1 m, que es radio 3-D, sobre un eje graduado por eje.

`save()` deja el PDF con el tamano exacto que pidio `figure()`, 3.45 in de
columna o 7.16 in de pagina, porque `bbox_inches='tight'` recorta despues de
dibujar y devolvia PDF de 5.94 in donde se habian pedido 7.16. Las figuras se
incluyen **sin escalar**:

```latex
\includegraphics{fig01_coupling}          % correcto
\includegraphics[width=\textwidth]{...}   % NO: reescala la letra
```

## Las ocho figuras

1. **`fig01_coupling`** (pagina). El acoplamiento navegacion-guiado esta
   condicionado por la efemeride del objeto: +62 % de impulso medio al recorrer
   la banda medida si el objeto se conoce a 10 m, +0.5 % si se conoce a 300 m.
2. **`fig02_koz_consistency`** (pagina). El radio de exclusion inflado tiene que
   depender del tiempo y no de la particion del horizonte. Sin el techo de sigma,
   tres particiones del mismo horizonte dan radios separados 619 m; con van Loan
   colapsan en una curva. En configuracion de vuelo el techo satura las dos en
   249.7 m y solo queda 17.6 m de dispersion a t = 60 s.
3. **`fig03_architecture_cost`** (pagina). Una restriccion muestreada en la
   rejilla no ve la conjuncion: el objeto pasa 34 ms dentro de la esfera y el
   paso de guiado es 886 veces mas largo. Probabilidad de detectarlo, 0.11 %.
4. **`fig04_execution`** (columna). Un encuentro completo: cruce de planos de
   88.15 deg a 10.0 km/s, impulso retrogrado de 21.8 mm/s a -112.5 min, fallo de
   120.0 -> 382.6 m, error de seguimiento 1.16 m rms. El MPC gasta 33.2 mm/s
   para entregar un plan de 21.8: un 53 % de sobrecoste.
5. **`fig05_navigation`** (pagina). Que hace y que no hace el ciclo de trabajo:
   el techo de 32.1 m lo pone un canal auxiliar siempre activo y no el receptor;
   lo que el ciclo compra es la media, 24.3 m al 19.45 % de encendido.
6. **`fig06_tangential_authority`** (columna). Un impulso tangencial pierde toda
   autoridad en un encuentro frontal.
7. **`fig07_dv_distribution`** (pagina). El presupuesto de propulsante es una
   cola, no una mediana: p90 = 53.7 mm/s en el punto de operacion.
8. **`fig08_probability_dilution`** (columna). Con el fallo fijo, la probabilidad
   de colision BAJA al empeorar la navegacion. Disenar contra un umbral de Pc
   premiaria tener mala navegacion.

## Lo que estas figuras no dicen

Se deja por escrito para que no se sobreinterprete ninguna y para que el pie de
figura lo recoja donde haga falta.

- **Fig. 1**: el estadistico es la MEDIA y no la mediana, a proposito. A
  sigma_obj = 10 m y sigma_nav = 4.8 m el 49.7 % de las geometrias no necesitan
  maniobra, asi que la mediana descansa sobre un atomo en cero: un bootstrap
  emparejado la deja exactamente en cero en el 38.7 % de los remuestreos y el
  cociente da un intervalo del 95 % de [443 %, infinito). El "+3254 %" que
  aparecia en versiones anteriores no es defendible; el +62 % si.
- **Fig. 2**: la version anterior estaba generada con dos constantes de vuelo
  sobrescritas (sigma_nav_max = inf y safetyCost = 0.2) mientras se rotulaba
  k = 3. Ahora se dibujan las dos configuraciones y cada una dice cual es. En
  vuelo el defecto queda casi tapado por el techo de sigma, que a su vez es la
  hipotesis del sensor auxiliar de la figura 5 y no una propiedad del guiado.
- **Fig. 3**: no dice nada sobre coste de computo. El argumento anterior, que la
  restriccion dentro del QP no cerraba en tiempo real, se ha retirado: sus
  numeros se midieron con codigo anterior a `a7701b4` y `8d55826`, y relanzado
  contra HEAD el mismo banco cumple todos los plazos. Ademas, con el codigo
  actual esa restriccion no influye en la solucion: con `dsafe0 = 150` y con
  `dsafe0 = 0` el mando difiere 9.9e-09 m/s^2 sobre 3.0e-04 m/s^2. El QP que se
  cronometraba era de seguimiento, no de evitacion. La probabilidad de deteccion
  supone ademas el instante del encuentro uniforme respecto a la rejilla.
- **Fig. 4**: el resultado de seguridad, 120.0 -> 382.6 m, es una anotacion y no
  una serie dibujada: `execution.csv` no lleva la distancia de maxima
  aproximacion frente al tiempo. Tampoco lleva las componentes del mando, asi que
  el caracter retrogrado del impulso solo aparece escrito. El suelo de seguridad
  es ya `d0 = 150 m`, el mismo del controlador y del resto del paper: antes usaba
  el radio de cuerpo duro de 5 m y sigma por eje, de modo que invertia una ley
  distinta de la de las figuras 1, 6 y 7.
- **Fig. 5**: la cota de 32.1 m depende por completo de un canal auxiliar que en
  el modelo esta siempre activo, con 100 m por eje en posicion y 50 m en altitud,
  y que el propio init llama "synthetic internal sensor". No hay un sensor de a
  bordo obvio que entregue eso en un CubeSat, y el pie de figura tiene que
  presentarlo como hipotesis. La cota va como la raiz de esa calidad: con 300 m
  por eje serian 72.9 m. El modelo lineal reproduce la corrida medida con 0.0 %
  de error mediano salvo en la primera casilla, donde el filtro real tiene una
  puerta por salud del GNSS que el modelo no tiene.
- **Fig. 6**: el color es un recuento crudo y el estudio muestrea dInc uniforme
  mientras v_rel va como sin(dInc/2), de modo que parte del gradiente de densidad
  es el muestreo y no la fisica. Ninguna columna registra si un caso alcanzo
  realmente d_target.
- **Fig. 7**: marginalizar sobre la calidad de efemeride exige un peso, y el que
  se usa es el uniforme sobre los cuatro niveles simulados. Es una hipotesis
  sobre el catalogo, no una medida; sin ella esta `mc_percentiles.csv`. La figura
  da impulso por encuentro: no hay tasa de conjunciones ni duracion de mision.
  Su punto de operacion hereda la hipotesis del sensor auxiliar.
- **Fig. 8**: el barrido es el del demostrador aislado, con su propia hipotesis
  de covarianza del objeto, un elipsoide anisotropo diag(150, 60, 60) m en ECI.
  Su buscador ya no esta limitado a la rama posigrada: resuelve la misma parabola
  cerrada que plan_cam y devuelve la raiz de menor modulo, que en las cuatro es
  retrograda, entre un 13 y un 38 % mas barata que la anterior. El umbral de 1e-4
  es convencional y es lo unico de la figura que no sale de los datos.

## Procedencia de los datos

- `mc_summary.mat` **no se lee**: lleva fecha anterior a la de los `mc_cam_*.mat`
  que deberia resumir y se calculo con un solo nivel de `sigma_obj`. Los
  percentiles se recalculan desde las mismas filas que van a `mc_cases.csv`.
- `bench_consec_quadprog_final.mat` **ya no se lee**: era la fuente de la figura 3
  anterior. Se conserva en `study/out` junto a `bench_consec_actual.mat`, que es
  la medida contra HEAD, para que la diferencia quede documentada.
- `pct_converged` en `timing_qp_cases.csv` es `100*mean(exitflag>0)`, o sea
  cuantos QP resolvio el solver, no cuantos llegaron a tiempo. Son dos fracasos
  distintos.
- Los `mc_cam_*.mat` se generaron con el integrador de referencia anterior a
  `8d55826`. El error resultante esta acotado y medido: menos de 0.054 mm/s por
  caso y menos de 0.04 mm/s en cualquier percentil, de modo que las cifras de
  cabecera no se mueven a la precision con la que se citan.

El detalle completo, con fichero y linea, en `study/AUDITORIA.md`.
