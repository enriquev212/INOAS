# Prueba de navegacion en MATLAB Online

El ZIP contiene una carpeta `INOAS`. Entrar en esa carpeta, donde estan
`run_navigation_trial.m`, `initialize_inoas_simulation.m` y `models/`.

En la Command Window:

```matlab
run_navigation_trial
```

El comando inicializa todo, ejecuta `inoas_model` hasta **1000 s** y exporta
los CSV y MAT a una carpeta nueva dentro de `results/campaign/`.
No requiere modificar ni guardar bloques manualmente.

Configuracion de esta prueba:

- Estado y covarianza procedentes del UKF.
- Predictor a `Ts = 1 s`, con correcciones auxiliares y sin futuras fijas GNSS.
- MPC: `h = 3 s`, `Np = 125`, horizonte de 375 s.
- `sigma_pos = sigma_alt = 2000 m`, `gamma = 3`, radio nominal de 150 m.
- El filtro real sigue recibiendo GNSS cuando lo habilita el supervisor.

Para probar despues la hipotesis de futuras fijas programadas:

```matlab
run_navigation_trial("navigation_scheduled")
```

Cada ejecucion utiliza un directorio con fecha y hora. Conservar toda la
carpeta de resultados, incluidos `navigation_prediction.csv`,
`navigation_uncertainty.csv` y los MAT. No basta con `metrics.csv` para
comprobar las conmutaciones y la incertidumbre.

Estas pruebas no demuestran por si solas ventajas del disparo por eventos.
El Pseudo NIS utiliza las sigmas actuales; los pulsos de 500 m no tienen
garantizada su deteccion. Todavia no se ha validado el encuentro completo
con esta version ni se ha realizado Monte Carlo. Las semillas siguen el
comportamiento de la inicializacion existente; estas dos ejecuciones no son
automaticamente un experimento con ruido emparejado.

## Version anterior y vuelta atras

La version anterior a este conjunto de cambios es el commit **0555bf8**.
Se conserva en el historial y se entrega un ZIP independiente de respaldo.
Para deshacer el nuevo cambio se utilizara `git revert` sobre el commit de
la prueba, generando un commit inverso. No hace falta borrar el historial
ni los resultados. Si se usa el ZIP anterior, extraerlo en otra carpeta;
no mezclar sus archivos con los de la prueba nueva.

`legacy_open_loop` solo cambia el metodo de propagacion de covarianza:
**no equivale a volver a la version anterior** del proyecto.

El detalle de archivos, conexiones, hipotesis y pruebas esta en
[navigation-covariance-prediction.md](docs/navigation-covariance-prediction.md).
