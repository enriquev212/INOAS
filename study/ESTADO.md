# Estado para el paper

Resumen de donde esta cada cosa despues de la auditoria y las reparaciones.
Fecha: 6 de septiembre de 2026. Rama `fix/mpc-open-points`, nada subido a GitHub.

## Las ocho figuras

Todas se regeneran con `cd study/pyfigs && python make_all.py` y pasan las tres
comprobaciones: `check_figures.py` (tamano exacto, fuentes incrustadas, sin
Type 3), `verify_captions.py` (las 23 cifras de los pies recalculadas desde los
CSV) y la inspeccion visual.

| fig | estado | resultado |
|---|---|---|
| 1 | rehecha | acoplamiento condicionado por la efemeride: +62 % frente a +0.5 %, factor 128 |
| 2 | regenerada | dispersion entre mallas de 619 m sin techo, 17.6 m en vuelo |
| 3 | nueva | la rejilla ve la conjuncion el 0.11 % de las veces; el paso es 886x la permanencia |
| 4 | relanzada | 120.0 -> 382.6 m, plan 21.8 mm/s, gasto 33.2 mm/s, seguimiento 1.16 m rms |
| 5 | nueva | el techo de 32.1 m lo pone el sensor auxiliar; el ciclo compra la media, 24.3 m |
| 6 | intacta | la autoridad tangencial se cierra en el encuentro frontal |
| 7 | intacta | presupuesto: p90 = 53.7 mm/s en el punto de operacion |
| 8 | corregida | dilucion de Pc; impulsos 18.2 -> 53.5 mm/s, retrogrados |

## Lo que hay que decidir antes de escribir

1. **El sensor auxiliar.** Toda la cota de navegacion descansa en un canal que
   entrega posicion en tres ejes a 100 m y altitud a 50 m, siempre activo, y que
   el propio init llama sintetico. Hay que decidir como se presenta: como
   hipotesis declarada (actualizacion desde tierra, determinacion de orbita a
   bordo) o quitarlo y aceptar que sigma crece sin cota entre correcciones. La
   figura 5 esta escrita para la primera opcion y trae la sensibilidad.

2. **Que es el modelo de Simulink en el paper.** No contiene plan_cam ni ninguna
   referencia al objeto: solo corre el lazo de navegacion. Las conjunciones
   salen de los scripts de study/. Es una division legitima, pero el texto tiene
   que decirla; ahora mismo ningun documento del repositorio lo hace.

3. **Peso sobre la calidad de efemeride.** La figura 7 marginaliza con peso
   uniforme sobre los cuatro niveles simulados. Si hay un prior de catalogo
   mejor, cambia el p90 del presupuesto.

## Lo que queda sin hacer

- La columna `converged` de mc_cases.csv vale 1 en las 36000 filas porque
  mc_cam.m comprueba `isfinite(dv)` en vez de la verificacion posterior de
  plan_cam. Arreglarlo obliga a relanzar la campana (seis bloques).
- Los `mc_cam_*.mat` se generaron con el integrador de referencia anterior a
  8d55826. El error esta acotado y medido: menos de 0.04 mm/s en cualquier
  percentil, asi que las cifras de cabecera no se mueven.
- La rama `bplane_tca` de MPC_INOAS solo es alcanzable desde
  study/test_bplane_mpc.m: `mpcDebrisMode` no se asigna en ningun sitio, asi que
  desde un clon limpio corre siempre `sphere_grid`.
- El paper no esta escrito y no esta en este repositorio.
- Nada esta subido a GitHub: 16 commits en local.

## Auditoria

`study/AUDITORIA.md` trae los 46 hallazgos con fichero y linea. Estan cerrados
los 2 bloqueantes y los serios que tocaban figuras. Los que quedan abiertos son
los tres puntos de decision de arriba y los cuatro de la lista anterior.

