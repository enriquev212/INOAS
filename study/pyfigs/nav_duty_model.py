"""Que compra realmente apagar el receptor GNSS.

El paper venia diciendo que sigma satura en 32.1 m por el ciclo de trabajo. No es
asi: 32.1 m es el punto fijo de Riccati de un canal auxiliar que en el modelo
esta SIEMPRE activo. myMeasurementFcn entrega posicion en tres ejes mas altitud
con R = diag([100^2 100^2 100^2 50^2]) y sin puerta de habilitacion
(models/inoas_model.slx, system_68.xml, HasMeasurementEnablePort1 = off), asi que
la cota existe con el receptor apagado del todo.

Lo que el ciclo de trabajo fija no es el techo sino cuanto tiempo se pasa cerca
del suelo. Este modulo reconstruye la recursion de covarianza del filtro para un
patron de encendido cualquiera, se valida contra la unica corrida medida, y
barre el ciclo de trabajo de 0 a 100 % para separar las dos cosas.

Se usa el modelo lineal porque el UKF sobre esta medida es practicamente lineal:
la posicion entra directa y la altitud es una proyeccion radial cuyo gradiente
apenas gira en 4000 s. La validacion contra la corrida medida lo confirma.

    python nav_duty_model.py        # imprime la validacion y el barrido
"""
import numpy as np

# --- constantes del filtro, tal y como las fija initialize_inoas_simulation.m --
TS = 1.0                                   # inoas_estimator_dt()
Q = np.diag([1.0, 1.0, 1.0, 1e-2, 1e-2, 1e-2])          # :141 Q_matrix
R_AUX = np.diag([100.0**2, 100.0**2, 100.0**2, 50.0**2])  # :138 R_matrix
R_GNSS = np.diag([5.0**2, 5.0**2, 5.0**2, 0.1**2, 0.1**2, 0.1**2])  # :146
P0 = np.diag([1e6, 1e6, 1e6, 100.0, 100.0, 100.0])      # :150 P0_kalman
R_EARTH = 6378137.0
A_ORB = 7714511.6                          # semieje de la orbita de INOAS


def _matrices():
    """Transicion a 1 s y las dos matrices de observacion."""
    Phi = np.eye(6)
    Phi[0:3, 3:6] = TS * np.eye(3)
    u = np.array([1.0, 0.0, 0.0])          # direccion radial; su giro en 1 s es
    H_aux = np.zeros((4, 6))               # despreciable frente a R
    H_aux[0:3, 0:3] = np.eye(3)
    H_aux[3, 0:3] = u
    H_gnss = np.eye(6)
    return Phi, H_aux, H_gnss


def propagate(schedule, P=None):
    """Recorre el patron de encendido y devuelve sqrt(trace(P_pos)) en cada paso.

    schedule : vector 0/1, un elemento por segundo. 1 = receptor encendido.
    """
    Phi, H_aux, H_gnss = _matrices()
    P = P0.copy() if P is None else P.copy()
    out = np.empty(len(schedule))
    for k, on in enumerate(schedule):
        P = Phi @ P @ Phi.T + Q
        for H, R in ((H_aux, R_AUX),) + (((H_gnss, R_GNSS),) if on else ()):
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.solve(S, np.eye(S.shape[0]))
            P = (np.eye(6) - K @ H) @ P
            P = 0.5 * (P + P.T)
        out[k] = np.sqrt(np.trace(P[0:3, 0:3]))
    return out


def duty_schedule(t_on, t_off, n):
    """Patron periodico t_on segundos encendido, t_off apagado."""
    period = np.concatenate([np.ones(int(t_on)), np.zeros(int(t_off))])
    return np.tile(period, int(np.ceil(n / len(period))))[:n]


def steady(t_on, t_off, n=40000, burn=0.5):
    """Techo, suelo y media temporal de sigma en regimen, para un ciclo dado."""
    s = propagate(duty_schedule(t_on, t_off, n))
    s = s[int(burn * n):]
    return s.max(), s.min(), s.mean()


if __name__ == '__main__':
    # --- 1) el techo con el receptor apagado PARA SIEMPRE -------------------
    s = propagate(np.zeros(20000))
    print('Canal auxiliar solo, GNSS nunca encendido')
    print('   sigma en regimen = %.3f m   (el paper lo atribuye al ciclo de trabajo)'
          % s[-1])

    # --- 2) validacion contra la unica corrida medida -----------------------
    import inoas_style as st
    law = st.read_csv('nav_law.csv')
    meas = propagate(duty_schedule(60, 300, 20000))
    # tiempo desde la ultima correccion en el mismo patron
    sch = duty_schedule(60, 300, 20000)
    gap = np.zeros(len(sch))
    g = 0
    for k, on in enumerate(sch):
        g = 0 if on else g + 1
        gap[k] = g
    half = len(sch) // 2
    print('\nValidacion contra nav_law.csv (60 s encendido / 300 s apagado)')
    print('   %8s %10s %10s %8s' % ('hueco[s]', 'medido[m]', 'modelo[m]', 'error'))
    errs = []
    for lo, hi, ctr, sig, _n in zip(law['bin_lo_s'], law['bin_hi_s'],
                                    law['bin_centre_s'], law['sigma_nav_m'],
                                    law['count']):
        m = (gap[half:] >= lo) & (gap[half:] < hi)
        if not m.any():
            continue
        pred = np.median(meas[half:][m])
        errs.append(abs(pred - sig) / sig)
        print('   %4.0f-%-4.0f %9.2f %10.2f %7.1f %%'
              % (lo, hi, sig, pred, 100 * (pred - sig) / sig))
    print('   error mediano %.1f %%' % (100 * np.median(errs)))

    # --- 3) que compra el ciclo de trabajo ---------------------------------
    print('\nBarrido del ciclo de trabajo (periodo de 360 s)')
    print('   %7s %10s %10s %12s' % ('activo', 'techo[m]', 'suelo[m]', 'media[m]'))
    for frac in [0.0, 0.05, 0.10, 0.1667, 0.25, 0.50, 0.75, 1.0]:
        t_on = round(360 * frac)
        if t_on == 0:
            hi = lo = mu = propagate(np.zeros(20000))[-1]
        elif t_on == 360:
            hi = lo = mu = propagate(np.ones(20000))[-1]
        else:
            hi, lo, mu = steady(t_on, 360 - t_on)
        print('   %6.1f %% %10.2f %10.2f %12.2f' % (100 * frac, hi, lo, mu))
    print('\n   El techo no se mueve con el ciclo de trabajo: lo fija el canal')
    print('   auxiliar. Lo que el ciclo compra es la media.')

    # --- 4) de que depende entonces el techo -------------------------------
    # Es la pregunta inmediata de un revisor: si la cota la pone el sensor
    # auxiliar, cuanto vale la cota si ese sensor es peor de lo supuesto.
    print('\nSensibilidad del techo a la calidad del sensor auxiliar')
    print('   (sigma por eje del canal auxiliar; en el modelo son 100 m)')
    print('   %10s %12s' % ('sigma_aux', 'techo[m]'))
    base = R_AUX.copy()
    for sp in [30.0, 100.0, 300.0, 1000.0, 3000.0]:
        R_AUX[0, 0] = R_AUX[1, 1] = R_AUX[2, 2] = sp ** 2
        R_AUX[3, 3] = (sp / 2.0) ** 2
        print('   %8.0f m %11.1f' % (sp, propagate(np.zeros(20000))[-1]))
    R_AUX[:] = base
    print('\n   El techo va casi como sqrt(sigma_aux): no es una propiedad del')
    print('   guiado ni del ciclo de trabajo, sino de lo que se suponga')
    print('   disponible mientras el receptor esta apagado.')

    # --- 5) tablas para la figura 5 ----------------------------------------
    import os
    DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')

    fr = np.concatenate([[0.0], np.linspace(0.02, 1.0, 25)])
    rows = []
    for f in fr:
        t_on = int(round(360 * f))
        if t_on <= 0:
            hi = lo = mu = propagate(np.zeros(20000))[-1]
        elif t_on >= 360:
            hi = lo = mu = propagate(np.ones(20000))[-1]
        else:
            hi, lo, mu = steady(t_on, 360 - t_on, n=20000)
        rows.append((100 * f, hi, lo, mu))
    with open(os.path.join(DATA, 'nav_duty_sweep.csv'), 'w', encoding='utf-8') as fh:
        fh.write('# Recursion de covarianza del filtro para un ciclo de trabajo periodico\n'
                 '# de 360 s. ceiling = sigma en el peor instante del ciclo, floor = en el\n'
                 '# mejor, mean = media temporal. El techo lo fija el canal auxiliar\n'
                 '# siempre activo, no el ciclo de trabajo.\n')
        fh.write('duty_pct,ceiling_m,floor_m,mean_m\n')
        for r in rows:
            fh.write('%.4g,%.6g,%.6g,%.6g\n' % r)
    print('\n   escrito data/nav_duty_sweep.csv (%d puntos)' % len(rows))

    base = R_AUX.copy()
    with open(os.path.join(DATA, 'nav_aux_sensitivity.csv'), 'w', encoding='utf-8') as fh:
        fh.write('# Techo de sigma_nav con el receptor GNSS apagado para siempre, en\n'
                 '# funcion de la calidad supuesta del sensor auxiliar. En el modelo el\n'
                 '# sensor auxiliar tiene 100 m por eje.\n')
        fh.write('sigma_aux_m,ceiling_m\n')
        for sp in [20, 30, 50, 100, 200, 300, 500, 1000, 2000, 3000]:
            R_AUX[0, 0] = R_AUX[1, 1] = R_AUX[2, 2] = float(sp) ** 2
            R_AUX[3, 3] = (float(sp) / 2.0) ** 2
            fh.write('%g,%.6g\n' % (sp, propagate(np.zeros(20000))[-1]))
    R_AUX[:] = base
    print('   escrito data/nav_aux_sensitivity.csv')
