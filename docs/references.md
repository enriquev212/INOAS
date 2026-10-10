# Paper References

Selected technical references from the submitted IEEE Aerospace 2027 paper.
Bracketed numbers follow its bibliography, not a separate repository numbering.
This page replaces the mixed challenge/WP7 reading list; earlier documentation
remains recoverable through [archive tags](model-provenance.md#archived-development-branches).
Third-party papers, books and manuals are cited, not redistributed.

## GNSS Processing and Receiver Operation

- **[11]** E. Gill, J. Morton, P. Axelrad, D. M. Akos, M. Centrella, and
  S. Speretta, "Overview of space-capable global navigation satellite systems
  receivers: Heritage, status and the trend towards miniaturization,"
  *Sensors*, vol. 23, no. 17, article 7648, 2023.
- **[13]** S. Lantto, "Precise orbit determination of CubeSats using duty
  cycled GPS observations," Master's thesis, West Virginia University, 2018,
  Graduate Theses, Dissertations, and Problem Reports, no. 6035.
- **[22]** J. Sanz Subirana, J. M. Juan Zornoza, and M. Hernandez-Pajares,
  *GNSS Data Processing, Volume I: Fundamentals and Algorithms*, European
  Space Agency, Technical Memorandum TM-23/1, May 2013.
  [ESA reference](https://gssc.esa.int/navipedia/GNSS_Book/ESA_GNSS-Book_TM-23_Vol_I.pdf).
- **[23]** L. Prange, E. Orliac, R. Dach, D. Arnold, G. Beutler, S. Schaer,
  and A. Jaggi, "CODE's five-system orbit and clock solution: The challenges
  of multi-GNSS data analysis," *Journal of Geodesy*, vol. 91, no. 4,
  pp. 345-360, 2017.
- **[24]** S. Schaer, A. Villiger, D. Arnold, R. Dach, L. Prange, and
  A. Jaggi, "The CODE ambiguity-fixed clock and phase bias analysis products:
  Generation, properties, and performance," *Journal of Geodesy*, vol. 95,
  article 81, 2021.
- **[31]** A. J. Hansen, *Global Positioning System (GPS) Civil Monitoring
  Performance Specification*, 3rd ed., U.S. Department of Transportation,
  John A. Volpe National Transportation Systems Center, Technical Report
  DOT-VNTSC-FAA-20-08, August 2020.
  [Specification](https://archive.gps.gov/technical/ps/2020-civil-monitoring-performance-specification.pdf).
  Its PDOP-availability criterion motivates the selected geometry threshold;
  it is not a guarantee of accuracy for this simulated GPS/Galileo receiver.
- **[35]** Hexagon | NovAtel, *OEM719 Product Sheet*, document D21049,
  version 10, June 2026.
  [Product sheet](https://hexagondownloads.blob.core.windows.net/public/Novatel/assets/Documents/Papers/OEM719-Product-Sheet/OEM719-Product-Sheet.pdf).
- **[36]** Pumpkin, Inc., *GPSRM 1 GPS Receiver Module User Manual*, document
  UM-10, June 2020.
  [User manual](https://www.pumpkininc.com/space/manual/UM-10_Pumpkin_GPSRM_1.pdf).
- **[37]** N. Linty, L. Lo Presti, F. Dovis, and P. Crosta, "Performance
  analysis of duty-cycle power saving techniques in GNSS mass-market
  receivers," in *Proceedings of the IEEE/ION PLANS*, 2014, pp. 1096-1104.
- **[38]** V. Bellad, M. G. Petovello, and G. Lachapelle, "Tracking and
  position errors in GNSS receivers with intermittent signal tracking,"
  *NAVIGATION: Journal of the Institute of Navigation*, vol. 63, no. 2,
  pp. 193-204, June 2016.

The simulated power assumptions are given in [paper configuration](conference.md#paper-configuration).
References to products and processing methods do not establish the software
provenance of the supplied dataset; see [data limitations](../data/README.md).

## Estimation and Residual Monitoring

- **[32]** Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, *Estimation with
  Applications to Tracking and Navigation: Theory Algorithms and Software*,
  John Wiley & Sons, 2001.
- **[33]** O. Garcia Crespillo, A. Grosch, J. Skaloud, and M. Meurer,
  "Innovation vs residual KF based GNSS/INS autonomous integrity monitoring
  in single fault scenario," in *Proceedings of ION GNSS+ 2017*, Portland,
  Oregon, 2017, pp. 2126-2136.
- **[34]** R. Isermann, "Model-based fault-detection and diagnosis - status
  and applications," *Annual Reviews in Control*, vol. 29, no. 1,
  pp. 71-85, 2005.
- **[39]** S. J. Julier and J. K. Uhlmann, "Unscented filtering and nonlinear
  estimation," *Proceedings of the IEEE*, vol. 92, no. 3, pp. 401-422, 2004.
- **[41]** R. van der Merwe and E. A. Wan, "The square-root unscented Kalman
  filter for state and parameter estimation," in *Proceedings of the IEEE
  International Conference on Acoustics, Speech, and Signal Processing
  (ICASSP)*, vol. 6, 2001, pp. 3461-3464.
- **[42]** The MathWorks, Inc., "Extended and Unscented Kalman Filter
  Algorithms for Online State Estimation," MATLAB and Simulink documentation.
  [Algorithm documentation](https://www.mathworks.com/help/ident/ug/extended-and-unscented-kalman-filter-algorithms-for-online-state-estimation.html).

The implemented auxiliary score is a normalized posterior-residual
**pseudo-NIS**, not the standard innovation-based NIS. These references do
not provide a chi-square calibration for the threshold used here.

## Orbital Dynamics and MPC Guidance

- **[5]** S. Di Cairano, H. Park, and I. Kolmanovsky, "Model predictive
  control approach for guidance of spacecraft rendezvous and proximity
  maneuvering," *International Journal of Robust and Nonlinear Control*,
  vol. 22, no. 12, pp. 1398-1427, 2012.
- **[6]** C. Jewison, R. S. Erwin, and A. Saenz-Otero, "Model predictive
  control with ellipsoid obstacle constraints for spacecraft rendezvous,"
  *IFAC-PapersOnLine*, vol. 48, no. 9, pp. 257-262, 2015.
- **[7]** X. Wang, Y. Li, X. Zhang, R. Zhang, and D. Yang, "Model predictive
  control for close-proximity maneuvering of spacecraft with adaptive
  convexification of collision avoidance constraints," *Advances in Space
  Research*, vol. 71, no. 1, pp. 477-491, 2023.
- **[40]** O. Montenbruck and E. Gill, *Satellite Orbits: Models, Methods
  and Applications*, Berlin, Germany: Springer, 2000.
- **[43]** W. H. Clohessy and R. S. Wiltshire, "Terminal guidance system
  for satellite rendezvous," *Journal of the Aerospace Sciences*, vol. 27,
  no. 9, pp. 653-658, 1960.
- **[44]** F. Gavilan, R. Vazquez, and E. F. Camacho, "Chance-constrained
  model predictive control for spacecraft rendezvous with disturbance
  estimation," *Control Engineering Practice*, vol. 20, no. 2,
  pp. 111-122, 2012.
- **[45]** S. H. Zak, "Model-based predictive control (MPC)," ECE 680
  Course Notes, Purdue University, 2011.
  [Course notes](https://engineering.purdue.edu/~zak/Second_ed/MPC_handout.pdf).

The present safety-radius and supporting-plane formulation is described in
[model architecture](model-architecture.md#guidance-and-safety-radius) and
[covariance prediction](navigation-covariance-prediction.md). References to
chance-constrained guidance do not make this implementation a certified
collision-probability bound.

## Actuator Reference Class

- **[46]** B. Banker and S. Askew, "Seeker 1.0: Prototype robotic free
  flying inspector mission overview," in *Proceedings of the 33rd Annual
  AIAA/USU Conference on Small Satellites*, paper SSC19-XI-04, 2019.
  [Conference paper](https://digitalcommons.usu.edu/smallsat/2019/all2019/151/).
- **[47]** C. D. Radke, M. Atwell, and B. Studak, "Design, development,
  and certification, of the Seeker robotic free flier propulsion system,"
  *AIAA Propulsion and Energy 2019 Forum*, paper AIAA 2019-3956, 2019.

These motivate the actuator reference class. The 0.10 N per-axis scale is a
study feasibility envelope, not a measured effective thrust for the maneuver.
