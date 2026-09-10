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
orbit for the guidance geometry, while the physical spacecraft assumptions are
scaled toward an STF-1-inspired 3U CubeSat case from duty-cycled GPS POD
literature: an approximate 3U mass, a NovAtel OEM615-class dual-frequency GNSS
receiver, and STF-1 drag/SRP assumptions (`CD = 2.2`, `area = 0.03 m^2`,
`CR = 1.0`). The encounter geometry remains a configurable INOAS guidance
scenario layered on top of this CubeSat-class platform model.

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
