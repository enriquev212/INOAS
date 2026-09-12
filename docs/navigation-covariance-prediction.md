# Prediccion de covarianza de navegacion

Esta version entrega al MPC el estado posterior del UKF y su covarianza,
ambos en ECI. El radio utiliza una prediccion de la calidad del navegador
con actualizaciones auxiliares. No es una certificacion de incertidumbre de
la trayectoria fisica respecto al plan de control actual.

## Cambios exactos

| Archivo o bloque | Cambio |
|---|---|
| `initialize_inoas_simulation.m`, tras `P0_kalman` | Parametros compartidos `ukfAlpha`, `ukfBeta`, `ukfKappa`, duraciones GNSS 60/300 s y umbral heuristico 12. |
| Inicializacion, seccion `MPC covariance inflation` | `covariancePredictionModeMpc = "navigation_aux_only"`. `Q_cov_mpc` queda exclusivamente para la comparacion historica. |
| `matlab/MPC_INOAS.m`, entrada opcional | Recibe `nav_status = [lambda_efectiva; edad_del_modo; salud_GNSS]`. Solo lo necesita el modo con calendario. |
| `MPC_INOAS`, antes del bucle de restricciones | Calcula el perfil UKF a `Ts=1 s`, usando `Q_matrix`, `R_matrix` y `R_gnss` del workspace. |
| `MPC_INOAS`, bucle de restricciones | Sustituye la propagacion `Phi*P*Phi'+Q_cov` por la consulta de `P_nav(:,:,i)`, excepto en modo historico. |
| `MPC_INOAS`, indices de referencia y debris | `timeStep+i-1` pasa a `timeStep+i`: estado predicho, referencia, debris y covarianza se comparan todos en `t+i*h`. |
| `inoas_model/Mux`, entradas 1 y 2 | Reciben posicion y velocidad del UKF directamente. La covarianza sigue viniendo del mismo UKF. Los switches GNSS/UKF ya no alimentan el MPC. |
| `Instrument Decision/Instrument Decision FSM` | Usa `instrument_decision.m` y expone un segundo puerto con el estado del supervisor. |
| `Navigation Status Delay` | Retarda ese estado un `Ts`, igual que `Unit Delay1` retarda `lambda`. Un ZOH a `h` lo entrega al nuevo puerto 4 del MPC. |
| `Kalman Filter/MATLAB Function` | La puerta de fija nueva usa `gnss_sample_time` y `gnssFixEpoch`, en lugar del 3 literal. |
| `Kalman Filter/Pseudo NIS` | Usa `R_matrix` y `R_gnss`; elimina los antiguos 100/50 m internos. Sigue siendo un score de residuo normalizado por R, no un NIS con covarianza de innovacion. |

El archivo `tools/configure_navigation_prediction.m` contiene el cambio
completo del SLX en codigo revisable. Es idempotente y no guarda automaticamente.
El SLX distribuido ya incorpora esos cambios. Para reproducirlos:

```matlab
addpath('matlab','tools');
configure_navigation_prediction('inoas_model');
save_system('inoas_model');
```

La correccion del Pseudo NIS puede cambiar las conmutaciones: con una sigma
auxiliar de 2000 m, un sesgo aislado de 500 m en cada eje aporta solo
`3*(500/2000)^2 = 0.1875` al score ideal sin otros errores. No se debe afirmar
que esos pulsos disparan el umbral 12 sin comprobarlo en las nuevas corridas.
No se ha cambiado el umbral para forzar una deteccion.

## Algoritmo

`predictNavigationCovarianceProfile.m` recibe el estado y la covarianza
posteriores en el instante actual. Construye un objeto `unscentedKalmanFilter`
de MathWorks con las mismas funciones y parametros que el UKF del modelo.
No modifica el filtro real.

En cada segundo futuro:

1. Propaga con `myStateTransitionFcn` y `Q_matrix`.
2. Actualiza con `myMeasurementFcn` y `R_matrix`.
3. En modo con calendario, actualiza con `H_GNSS = I_6` y `R_gnss` solo si
   el supervisor previsto permite GNSS y corresponde una fija nueva.
4. Almacena la covarianza posterior. El MPC consulta los nodos `t+i*h`.

Las medidas futuras se sustituyen por su media predicha, dando innovacion
cero. Para el canal auxiliar no lineal se obtiene esa media con `residual`;
no se aproxima directamente por `h(media del estado)`. El GNSS lineal usa
la forma de Joseph para actualizar P. No se consultan errores GNSS futuros,
datos de verdad del plant ni disparos futuros por Pseudo NIS.

La aceleracion ECI nominal se mantiene en el ultimo comando conocido durante
el horizonte. Las perturbaciones, errores de ejecucion y cambios futuros de
comando no quedan certificados por este perfil. Es una prediccion nominal
del UKF, no una reproduccion exacta de sus futuras realizaciones no lineales.

El radio conserva:

```matlab
dsafe_i = dsafe0 + safetyCost * sqrt(lambda_max(P_nav_pos_i));
% dsafe0 = 150 m; safetyCost = 3.
```

`3*sqrt(lambda_max)` no equivale automaticamente a una probabilidad 3D de
99.73% ni a una probabilidad de colision: falta, entre otras cosas, la
incertidumbre del objeto y la del seguimiento del plan.

## Modos

| `covariancePredictionModeMpc` | Hipotesis |
|---|---|
| `"navigation_aux_only"` | Base: auxiliares cada segundo y ninguna fija GNSS futura. No es un peor caso absoluto: supone auxiliares operativos y ruido segun el modelo. |
| `"navigation_scheduled"` | Refinamiento: calendario compartido 60/300 s, estado efectivo retrasado y salud actual mantenida. No anticipa activaciones por Pseudo NIS ni recuperacion de una senal actualmente no saludable. |
| `"legacy_open_loop"` | Comparacion del metodo anterior de propagacion CW con `Q_cov_mpc`. No revierte el resto de correcciones ni reproduce exactamente resultados antiguos. |

El reloj absoluto y `gnssFixEpoch` determinan la fase de las fijas cada 3 s;
por eso no hace falta inferir la fase a partir de la ultima medida recibida.
Activar GNSS entre dos fijas no implica una correccion inmediata. El perfil
arranca tras las correcciones actuales y no vuelve a asimilar esa muestra.

Con las mismas condiciones iniciales, entrada nominal y calendario, el perfil
en un instante comun no depende del muestreo `h` del MPC. Cambiar `h` en el
lazo cerrado si puede cambiar la trayectoria y las decisiones. Las restricciones
siguen impuestas solo en los nodos: este cambio no certifica la separacion
entre ellos. `h` debe ser multiplo entero de `Ts`; el propagador actual exige
`Ts=1 s`. El puerto del MPC sigue dimensionado para un maximo de 125 pasos.

## Ejecucion y exportacion

Desde la raiz INOAS en MATLAB o MATLAB Online:

```matlab
addpath('matlab');
addpath(genpath('tools'));
setpref('inoas','mpcTuneConfig',struct( ...
    'covariancePredictionModeMpc',"navigation_aux_only"));
run_baseline_campaign("results/campaign/navigation_aux_only",1000);
```

Para la segunda ejecucion, cambiar el modo a `"navigation_scheduled"` y usar
otro directorio de salida. Las semillas deben fijarse de forma equivalente
antes de compilar/simular si se quiere comparar corridas emparejadas; la
inicializacion general existente sigue usando `rng('shuffle')`.

La exportacion mantiene los CSV existentes y anade:

- `navigation_uncertainty.csv`: sigma maxima del UKF, error real de posicion,
  NEES de posicion, lambda ordenada/efectiva, fija nueva y Pseudo NIS.
- `navigation_prediction.csv`: origen y destino del pronostico, sigma predicha
  y realizada, diferencia relativa entre bloques de posicion y numero de
  correcciones GNSS supuestas.
- `navigation_prediction.mat`: matrices completas y tablas anteriores.

Los destinos posteriores a StopTime tienen comparacion `NaN`; no se extrapola
el filtro real. En modo sin futuras fijas es normal predecir una covarianza
mayor que la realizada cuando si llegan fijas. Incluso con calendario pueden
existir diferencias por innovaciones, no linealidad o cambios del supervisor.
El log `NIS` se conserva por compatibilidad; en los CSV nuevos se etiqueta
`pseudo_nis`. La coherencia estadistica se debe comprobar tambien contra el
error real en multiples realizaciones.

## Verificacion

```matlab
results = runtests('tests/test_navigation_prediction.m');
assertSuccess(results);
addpath('tools');
run_navigation_smoke_test;
```

Las pruebas cubren Riccati lineal, nodos comunes para h=3/12, beneficio de
las medidas auxiliares, fase GNSS, falta de salud GNSS, temporizadores,
rechazo de tiempos incompatibles y propagacion orbital hasta 375 s.
El smoke test compila y simula una copia temporal con Np=125 durante 9 s;
despues prueba el calendario hasta 66 s con Np=5, incluyendo la conmutacion.
No regenera los MAT de referencia del repositorio. Estas pruebas no sustituyen
una simulacion completa del encuentro de 1000 s ni la campana estadistica.

Comprobacion local del 12-09-2026, MATLAB R2026a Update 4: ocho pruebas
numericas superadas; simulacion de 9 s con Np=125 superada; simulacion de
66 s con Np=5 y calendario superada; exportadores nuevo y anterior verificados.
En la segunda prueba, la orden cambia a cero en t=59 s y la lambda efectiva
en t=60 s, coincidiendo con el estado retrasado recibido por el predictor.
Persisten los avisos previos de dimensiones inferidas de Ground/Demux.

## Base matematica

- [MathWorks: unscentedKalmanFilter](https://www.mathworks.com/help/control/ref/unscentedkalmanfilter.html).
- [MathWorks: algoritmos EKF y UKF](https://www.mathworks.com/help/ident/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html).
- [MathWorks: residual y covarianza de innovacion](https://www.mathworks.com/help/ident/ref/extendedkalmanfilter.residual.html).

La coincidencia de matrices asegura coherencia de configuracion, pero no
demuestra que el modelo de ruido del sensor sea fisicamente correcto. El
CWNA y la calibracion del ruido IMU se mantienen tal como estaban para
aislar este cambio de arquitectura.
