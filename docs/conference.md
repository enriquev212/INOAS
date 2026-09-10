# Conference Adaptation and Citation

## IEEE Aerospace Conference Adaptation

The project is currently being adapted into a paper for the **2027 IEEE
Aerospace Conference**, held at the Yellowstone Conference Center in Big Sky,
Montana, March 6-13, 2027.

The abstract was accepted on **July 6, 2026**:

- **Title:** Robust MPC-Based Collision Avoidance Guidance and Safe Duty-Cycled
  GNSS Navigation for LEO CubeSats
- **Session:** 12.01 Orbital, Surface and Payload/Instrument Mission Operations
- **Paper number:** 2437

For the conference version, the original Student Aerospace Challenge architecture
is being adapted toward a LEO CubeSat scenario. This includes scaling the
physical system and mission assumptions to better match CubeSat-class
constraints while preserving the main contribution: robust MPC-based collision
avoidance coupled with safe duty-cycled GNSS/UKF navigation.

The default simulation setup keeps the existing Sentinel-like INOAS reference
orbit for the guidance geometry, while replacing the original large-spacecraft
physical assumptions with a representative STF-1-inspired 3U CubeSat-class bus
from duty-cycled GPS POD literature: an approximate 3U mass, a NovAtel
OEM615-class dual-frequency GNSS receiver, and STF-1 drag/SRP assumptions
(`CD = 2.2`, `area = 0.03 m^2`, `CR = 1.0`). The encounter geometry remains a
configurable INOAS guidance scenario layered on top of this CubeSat-class
platform model.

The actuation assumption is intentionally separated from STF-1, which is a POD
reference and not a propulsion reference. The conference setup uses NASA/JSC
Seeker 1.0 as an actuator-architecture reference: a 3U free-flying inspection
CubeSat with a cold-gas 6-DOF propulsion architecture. Seeker is used only to
frame the actuator architecture and individual-thruster scale, not as a claim of
full operational mission success. The MPC acceleration limit is derived from a
Seeker-class individual cold-gas thruster scale of `F_control = 0.10 N`, giving
the per-axis optimizer box constraint `u_max = F_control/m_sat`. This bound is a
feasibility envelope, not the reported effective manoeuvring thrust.

For flight-demonstrated proximity operations, the more direct reference is CPOD:
two 3U CubeSats that performed autonomous RPO on orbit with 3-DOF translational
control, including experiments from intersatellite distances up to `997 km` and
a reported minimum separation of `361 m`. Propulsive feasibility should
therefore be argued from the measured MPC command and impulse demand, and then
contextualized against Seeker-class actuator scale and CPOD-class flown 3U RPO
capability.

The Sentinel-6A-derived GNSS-quality profile is retained for its time structure
and as an optimistic navigation-quality case. It should not be read as the raw
autonomous performance of an OEM615 receiver on a generic CubeSat. Likewise,
the STF-1 `CD`, `area`, and `CR` values are documented bus/perturbation
assumptions; the current guidance validation model does not use drag/SRP as an
active disturbance model.

## Citation

If you refer to this project or the conference adaptation, please use:

```bibtex
@inproceedings{fernandezacero2027inoas,
  title = {Robust {MPC}-Based Collision Avoidance Guidance and Safe Duty-Cycled {GNSS} Navigation for {LEO} {CubeSats}},
  author = {Fernandez-Acero Campoamor, Alberto and Valverde Sacristán, Enrique and Yuste Pubill, Álvaro and Grande González, Guzmán and Soler i Pla, Julia and Xu, Changxiang},
  booktitle = {Proceedings of the 2027 IEEE Aerospace Conference},
  address = {Big Sky, Montana, USA},
  year = {2027},
  note = {To appear; abstract accepted July 6, 2026, paper no. 2437}
}
```

The repository also includes machine-readable citation metadata in
[`CITATION.cff`](../CITATION.cff).
