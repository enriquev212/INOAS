# IEEE Aerospace 2027 Paper and Citation

## Paper

- **Title:** Robust MPC-Based Collision Avoidance Guidance and Safe Duty-Cycled
  GNSS Navigation for LEO CubeSats
- **Authors:** Alberto Fernández-Acero Campoamor, Enrique Valverde Sacristán,
  Álvaro Yuste Pubill, Guzmán Grande González, Júlia Soler i Pla and
  Changxiang Xu (ISAE-SUPAERO)
- **Conference:** 2027 IEEE Aerospace Conference, Yellowstone Conference Center,
  Big Sky, Montana, March 6-13, 2027
- **Session:** 12.01 Orbital, Surface and Payload/Instrument Mission Operations
- **Paper number:** 2437 (abstract accepted on July 6, 2026)
- **Status:** Full paper submitted; review decision pending.

The paper is the CubeSat adaptation of the INOAS architecture developed for the
Student Aerospace Challenge, evaluated in a paired Monte Carlo campaign under
degraded GNSS. The results are summarised in
[Results](results.md#paper-results).

## Platform and Input Assumptions

The paper evaluates covariance-aware MPC collision avoidance coupled with
duty-cycled GNSS/UKF navigation on a 3U CubeSat-class platform. Earlier challenge
material is kept separately in [project history](history.md).

- **Platform.** The reference orbit is inspired by Sentinel-6A, and the
  spacecraft is a 3U CubeSat-class platform. Its cross-sectional area, drag
  coefficient and reflectivity coefficient follow the STF-1 assumptions; its
  mass and GNSS module reference are assumptions of the study.
- **GNSS receiver.** The receiver power model takes the Pumpkin GPSRM 1, which
  integrates a NovAtel OEM719, as reference. The dual-frequency GPS/Galileo
  processing assumed in the paper would need one of the optional
  multi-frequency OEM719 variants.
- **Actuation.** The per-axis acceleration bound
  `u_max = F_max / m = 0.10 N / 3.99 kg ≈ 0.02506 m/s^2` is motivated by
  Seeker-class cold-gas propulsion. It is a feasibility envelope, not the
  reported effective maneuvering thrust.
- **GNSS data.** GNSS error and availability profiles are derived from
  Sentinel-6A positioning data over a 24-hour arc, with controlled degradations
  in the number of tracked satellites and in satellite geometry. Raw PPP
  processing is not executed inside the closed-loop simulations.
- **Auxiliary sensors.** The auxiliary channel is represented by three synthetic
  Cartesian position measurements with a 2000 m standard deviation. It feeds the
  UKF and the pseudo-NIS discrepancy alarm.

The paper's architecture and supporting-plane geometry are available as
[PNG previews and vector PDFs](assets/paper/README.md). Their relation to the
executed simulation is explained in [model architecture](model-architecture.md).

## Paper Configuration

| Parameter | Value |
| --- | --- |
| Reference orbit | `a = 7714.43 km`, `e = 9.5e-5`, `i = 66.04 deg`, RAAN 116.6 deg, argument of perigee 90 deg, initial true anomaly 131 deg |
| Platform | 3.99 kg, area 0.03 m², `CR = 1.0`, `CD = 2.2` |
| Plant forces | Earth gravity to degree 2, Sun and Moon point masses, solar radiation pressure, atmospheric drag with constant density 1.35e-13 kg/m³ |
| UKF and reference/debris propagators | Central gravity and J2 |
| Guidance model | Unperturbed Clohessy-Wiltshire dynamics |
| Simulation duration | 6743 s (about one orbital period) |
| Estimator / GNSS / MPC steps | 1 s / 3 s / 12 s |
| MPC horizon | 60 steps (720 s) |
| Encounter | Design epoch `te = 1500 s`; relative position `[50, 0, 0] m` and velocity `[300, 100, 0] m/s` at `te`; without avoidance, the reference passes 15.8 m from the debris |
| Safety radius | Keep-out `d0 = 150 m` and inflation factor `γ = 3`; constant 295 m radius for comparison |
| Actuation | 0.02506 m/s² per axis, 0.007 m/s² per MPC step, 10% actuator uncertainty |
| GNSS quality gate | Valid solution, `Nsat ≥ 5`, `0 < PDOP ≤ 6` |
| GNSS rejection intervals | [700, 980), [2000, 2260) and [3500, 3550) s |
| Receiver timing | 35 s minimum acquisition delay for every policy, 60 s nominal tracking, 300 s nominal OFF |
| Receiver policies | Full GNSS (always powered); Fixed-Time (fixed 96 s ON / 300 s OFF calendar); Reactive (nominal timing, reacquisition on poor tracking, and a pseudo-NIS alarm with threshold 10.3) |
| Auxiliary bias pulses | +5000 m per axis over [600, 620) s; +3500 m per axis over [2000, 2020) s |
| Receiver power model | OFF 0.025 W, TRACKING 1.8 W, ACQUIRING 1.3 × TRACKING |
| Monte Carlo campaign | 50 paired replicates × 5 configurations (250 runs) |

Debris position uncertainty is neglected in the paper to isolate the behavior of
the controller.

### Public Model and Reproducibility

`main` uses the AUX3 model from former branch `feat/aux3-on-alberto` at `721c0eb`,
preserved under the `archive/aux3-on-alberto-20261009` tag.
The default runner selects Reactive/adaptive operation, 6743 s, and seed 42.
Full GNSS, Fixed-Time and the constant 295 m radius are explicit run options.
The Simulink file itself is unchanged from the AUX3 source revision.

The original 50-run seeds, randomized initial errors and complete campaign
outputs have not been supplied with this public update. The default single
run retains the AUX3 demonstration's initial estimation error, which can be
overridden through `InitialError`. Running this repository therefore does not
by itself reproduce the reported ensemble medians. See
[model provenance](model-provenance.md) and [run instructions](how-to-run.md).

## Acknowledgements

The paper acknowledges Prof. Miguel Gómez-López and Dr. Iñigo Cortés Vidal for
their guidance and feedback during its preparation.

## Citation

If you refer to this project or the paper, please use:

```bibtex
@inproceedings{fernandezacero2027inoas,
  title = {Robust {MPC}-Based Collision Avoidance Guidance and Safe Duty-Cycled {GNSS} Navigation for {LEO} {CubeSats}},
  author = {Fernández-Acero Campoamor, Alberto and Valverde Sacristán, Enrique and Yuste Pubill, Álvaro and Grande González, Guzmán and Soler i Pla, Júlia and Xu, Changxiang},
  booktitle = {Proceedings of the 2027 IEEE Aerospace Conference},
  address = {Big Sky, Montana, USA},
  year = {2027},
  note = {Submitted; review decision pending. Abstract accepted July 6, 2026, paper no. 2437}
}
```

The repository also includes machine-readable citation metadata in
[`CITATION.cff`](../CITATION.cff).
