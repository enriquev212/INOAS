# INOAS Project - Dassault's Challenge

This repository contains all Matlab, Simulink, and Python documents related to the INOAS Dassault's challenge.

## Main Files

The project relies on two core files that work together:

1.  **`PRUEBA_SPACECRAFT.slx`**: The principal Simulink model constituting the heart of the system simulation. This version contains the Kalman Filter working properly, as well as the PD controller.
2.  **`MPCcontrollerSpacecraft.slx`**: The MPC is added to the previous version of the simulator, replacing the PD controller. The debris avoidance algorithm is currently being added.
3.  **`Deciss_GNSS_MPCcontrolledSpacecraft.slx`**`: The Instrument Decision is implemented along with the rest of the blocks.
4.  **`GNSS_MPCcontrollerSpacecraft.slx`**: Refined Simulink model which implements the MPC, the GNSS telemetry and the Instrument Decision. Once the MPC works correctly, the whole system will be tested.
5.  **`Script_Councurso_SPacecraft.mlx`**: A Live Script containing all necessary parameters and inputs required by the Simulink model to function correctly. **This script must be run before starting the simulation.**
6.  **`gnss_data.m`**: Initialize script whcih sets all the data needed to run properly the GNSS algorithms .

---

## Simulink Model Overview

Below is the top-level view of the `PRUEBA_SPACECRAFT.slx` model, highlighting its modular architecture.

![Initial Simulink Model Architecture](./main_simulink_model.png)

![Final Simulink Model Architecture](./final_simulink_model.png)

---

## Detailed Model Structure

The model is structured into 5 distinct and completely modular sections:

### 1) Modelo Planta satélite (Satellite Plant Model)
* Implements a non-linear and highly complex model of Spacecraft Dynamics based on the **"Spacecraft Dynamics"** block from the Aerospace Blockset.
* Includes additional relevant environmental perturbations.

### 3) Parámetro Activación GNSS (GNSS Activation Parameter)
* This is a key switch parameter that allows the system to toggle and turn off the GNSS sensor simulation.
* This feature is critical for testing the system's reliance on secondary measures, such as the **Altimeter, Magnetometer, and IMU**.

### 5) Estimación de estados (State Estimation)
Includes two major sub-components:
* **3a) GNSS**: Simulation and processing of Global Navigation Satellite System data.
* **3b) Kalman Filter**: Implemented for sensor fusion and precise state determination. An unscented Kalman Filter has been made up in order to estimate correctly the state vector considering the simulated measurements. This measurements come from the plant, which simulates with good precission the real system, taking into account its non-linearities, and some wite noise is added.

### 6) Lógica de la controladora & Cálculo Valor consigna (Controller Logic & Setpoint Calculation)
* **Current State**: A Proportional-Derivative (PD) controller is currently implemented, with a *windup* issue identified in the error handling that needs correction.
* **Future Work**: A Model Predictive Controller (MPC) will be implemented in this section specifically to resolve the **Debris Avoidance** issue.

### 7) Control de Actitud (Attitude Control)
* This section has been designed but is not yet fully studied or integrated with the main spacecraft dynamics.
