# Auditoria de preparacion para el paper

Auditoria adversaria de cinco frentes sobre el repositorio, los datos y las ocho
figuras. Cada hallazgo lo levanto un agente y lo intento refutar otro; aqui solo
estan los que sobrevivieron. "overstated" = el defecto es real pero menor de lo
que se afirmo, y el enunciado que sigue es ya el corregido.

Recuento: **2 bloqueantes, 13 serios, 17 menores, 14 notas**.

Los dos bloqueantes y el de la figura 2 estan ademas verificados a mano,
reproduciendo el numero de forma independiente.


## Bloqueantes (2)

### B.1  [provenance] confirmed

  Fig. 3's headline numbers (up to 14.9 % of solves overrunning, worst cases of 45 and
  77 s; figures.tex:47-56, README.md:60-63) are exported by export_data.py:152 from
  study/out/bench_consec_quadprog_final.mat, which predates commits a7701b4 and 8d55826
  to matlab/MPC_INOAS.m — the latter rewriting the collision-constraint frame and fixing
  a refStep off-by-one — and re-running the same benchmark unchanged against current
  HEAD d1e9291 (study/out/bench_consec_actual.mat, same case list and solve counts
  134/81/41/81) gives 0 % overruns, 100 % convergence and a 0.86 s worst case, so the
  caption's numbers were measured on a constraint geometry the repo has since declared
  wrong and cannot be reproduced from the code the paper describes.

### B.2  [reviewer] confirmed

  The 32.1 m saturation that figures.tex sells as the "measured cost of duty-cycling the
  GNSS receiver" is the Riccati fixed point of an ungated aiding channel in the UKF
  (models/inoas_model.slx, simulink/systems/system_68.xml, SID 130: MeasurementFcn1 =
  myMeasurementFcn with HasMeasurementEnablePort1 = off and MeasurementNoise1 = R_matrix
  = diag([100^2,100^2,100^2,50^2]) from initialize_inoas_simulation.m:133-138,
  physically a "Magnetometer Noise" + "Altimetry Noise" pair added to truth position in
  system_139.xml) — rebuilding the covariance recursion from R_matrix, R_gnss and
  Q_matrix alone yields 32.125 m with GNSS never enabled and reproduces the entire
  nav_law.csv curve to under 1 %, so the bound would be identical at 0 % duty cycle, the
  19.45 % on-fraction sets only how often sigma is pulled back to 4.8 m, and the same
  32.1 m also fixes fig01's band edge, fig07's operating point (p90 = 53.7 mm/s) and
  sigma_nav_max in MPC_INOAS.m — the repo admits the aiding at
  initialize_inoas_simulation.m:631-633 but neither the fig05 caption nor
  pyfigs/README.md's caveat list discloses it.


## Serios (13)

### S.1  [staleness] overstated

  Fig. 3's QP-internal arm (study/out/bench_consec_quadprog_final.mat, 2026-09-05
  14:11:58, the sole source of the caption's "14.9 %" and "45 and 77 s" via
  export_data.py:152) predates commit 8d55826 (17:55:46), which rewrote the very
  sphere_grid constraint rows that benchmark times, while the figure's other arm
  (primary_timing.mat, 17:55:27) was regenerated inside that same commit — so the two
  bars are measured at different code versions, and the in-flight re-run at HEAD
  (bench_consec_actual.mat) returns 100 % within-deadline and 0.8585 s worst against the
  caption's 14.9 % and 45/77 s; the finding's attribution to b8d8e89 (whose own message
  quotes the 45 s as the result of its own safetyCost change) and to a7701b4 (whose
  jacobian fix lives in the bplane_tca branch this benchmark never enters), and its
  "723x -> 13.7x margin collapse" (the advertised 480x comes from the proposed arm's
  primary_timing.mat and is unaffected), are all wrong.

### S.2  [staleness] confirmed

  study/bench_consecutive.m:27 defaults BENCH_TAG='actual' and :75-77 saves to
  bench_consec_<tag>.mat, so the in-flight re-run wrote a new untracked
  study/out/bench_consec_actual.mat (2026-09-06 00:18:50) reporting worst-case 0.86 s
  and 0 % of solves late, while all three consumers of that benchmark —
  study/pyfigs/export_data.py:152, study/make_paper_figures.m:73 and
  study/make_arch_figure.m:27 — hardcode the older bench_consec_quadprog_final.mat
  (2026-09-05 14:11:58, still byte-identical to HEAD at blob 3cd4c57) whose 45.1 s /
  77.5 s / 14.93 %-late values I reproduced exactly from timing_qp_samples.csv and which
  appear verbatim in figures.tex:50-51; since no consumer, no CSV note line and no
  meta.json field records the tag, MAT date or code version, re-running export_data.py
  would regenerate timing_qp_cases.csv unchanged and Fig. 3 would keep numbers the
  newest measurement of the same four-case grid contradicts by roughly 50x.

### S.3  [staleness] confirmed

  Fig. 2's headline (41.3 m per-step mesh spread at t = 500 s, figures.tex:37-38 /
  README.md:56-59) is reproducible current-code output, but study/make_figures.m:103-104
  overrides the flight constants that commit b8d8e89 established (sigma_nav_max = inf
  instead of 32.1 m, safetyCost = 0.2 instead of 3) with the reason recorded only in a
  Spanish code comment and absent from both the caption and README's "what these figures
  do not say"; under the deployed values the curves saturate at 249.7 m and the quoted t
  = 500 s spread is exactly zero for every mesh and both schemes (though up to 17.6 m of
  spread survives near t = 60 s before the cap binds), and the sigma of 352-1121 m the
  plotted curves require contradicts the 32.1 m saturation the same paper asserts in
  Figs. 5 and 8.

### S.4  [provenance] confirmed

  The symbol sigma_nav denotes two quantities a factor sqrt(3) apart: a 3D radius
  sqrt(trace(P)) in Figs 1, 5, 6 and 7 (MPC_INOAS.m:691 -> nav_chain -> mc_cam.m:55,128
  which correctly uses (sg^2/3)*eye(3)) and a per-axis sigma in Figs 4 and 8
  (test_cam_retarget.m:50,119 and cam_demo.m:83,137 use diag([s s s].^2), verified by
  sigma_combined^2 - sigma_nav^2 = 5477.99 in retarget_sweep.csv), with the collision
  made concrete by the fact that fig01/fig07/fig08 carry the identical axis label and
  the identical tick values {12, 40, 100, 250} for different physical quantities, and
  that fig08_probability_dilution.py:66,84,110 shades the 3D-radius duty-cycle band
  4.8-32.1 m across both panels of an axis whose own units put that band at 2.75-18.54
  m; the Fig 4 side is a documentation gap rather than an error, since SIG_TCA = 40
  never reaches figures.tex, README.md or meta.json and its per-axis reading (69.3 m
  3D-equivalent) makes the 257.4 m requirement more conservative, not less.

### S.5  [provenance] confirmed

  Fig. 1's headline "+3254 %" (annotated by study/pyfigs/fig01_coupling.py:71, quoted in
  figures.tex:23 and README.md:53) reproduces exactly from the shipped CSVs and its
  provenance chain is clean, but its denominator - the median dv at (sigma_nav = 4.8,
  sigma_obj = 10) - is the mean of the 5th and 6th smallest non-zero impulses sitting on
  top of a 49.67 % (745/1500) atom at exactly zero, so a paired bootstrap gives it a 95
  % interval of [443 %, +infinity) with the denominator median exactly zero in 38.7 % of
  resamples; the four-significant-figure quotation is unsupportable and no caveat exists
  anywhere (Fig. 1 is the only figure missing from README's "what these figures cannot
  support" list), even though the qualitative contrast against +0.5001 % at sigma_obj =
  300 m (95 % CI [0.24, 0.73] %, zero no-manoeuvre cases) is robust and the fix is to
  quote a statistic off the median, e.g. the mean at +62.1 % (CI [58, 66] %), rather
  than the equally unstable "factor of 34".

### S.6  [claims] overstated

  fig08's required-impulse row (25.2 -> 60.5 mm/s, figures.tex:135) is a posigrade-only
  upper bound, because cam_demo.m clamps its secant to dv >= 0 (lines 186, 194) - the
  exact law that commit c2bb207 stripped out of plan_cam.m as "wrong three times over"
  while adding cam_demo.m to the repo with it intact - and re-solving cam_demo's own
  miss(dv) parabola gives retrograde roots of 18.2/20.2/27.7/53.5 mm/s, 13-38% cheaper;
  the claimed "2.16x for the same encounter" is not like-for-like, since fig08's TCA is
  at 3 orbits and fig04's at 1.5 (cam_demo.m:36 vs test_cam_retarget.m:34), which turns
  the identical ECI covariance diag([150 60 60])^2 into 123.3 m of projected sigma for
  fig08 against 74.0 m for fig04, and the Mahalanobis criterion in fact lowers the
  required miss (394.0 -> 360.7 m) rather than raising it.

### S.7  [claims] confirmed

  The GNSS duty cycle documented in README.md:36-40 and docs/model-architecture.md:23-31
  as covariance-driven is in fact a fixed 60 s on / 300 s off timer with GNSS-health
  emergency exits and an NIS >= 12 early return: the FSM in chart_72.xml takes no
  covariance argument, the Q_gnss inport (SID 316) is wired to a Terminator (SID 328) in
  system_314.xml, the Compute J block (SID 296) has no incoming line at all and drives
  only a scope, and its threshold max_cov = 2000 (initialize_inoas_simulation.m:160) is
  read by nothing - a divergence that has existed since the first model commit and that
  affects repo prose and dead plumbing only, since figures.tex and the exported CSVs
  make no covariance claim and the 19.4500% on-fraction reproduces exactly from
  nav_duty.csv.

### S.8  [claims] confirmed

  The safety-floor and sigma-convention corrections that commit c2bb207 declares in its
  own message (dsafe0 = 150 m rather than R_hb = 5 m, and sqrt(trace(P)) as a 3-D radius
  rather than a per-axis sigma) were applied only to study/mc_cam.m:66,128, so the
  closed-loop demo behind Fig. 4 (study/test_cam_retarget.m:50,53) and the Pc
  demonstrator behind Fig. 8 (study/cam_demo.m:137) still execute the pre-fix law: Fig.
  4's headline plan of 12.57 mm/s against a required 257.4 m becomes 21.8 mm/s against
  382.6 m under the corrected law used for Figs. 1, 6 and 7, and Fig. 8 overlays the
  measured 4.8-32.1 m 3-D-radius band on a per-axis sigma axis.

### S.9  [claims] confirmed

  Fig. 3's caption (study/pyfigs/figures.tex:47-48) claims without qualification that
  enforcing the collision constraint inside the QP does not close in real time, yet its
  own h=10/Np=50 case meets its 10 s deadline with 83.9x margin and zero late solves —
  which the figure itself already plots and labels "past deadline: none"
  (fig03_architecture_cost.py:157-161), so this is caption over-generalisation rather
  than concealment — while the two arms also differ in step (5 s vs 30 s), horizon (Np
  150 vs 60, 600 vs 240 decision variables) and dsafe0 (150 vs 0) with nothing held
  constant and no such caveat in README.md:90-94; worse, the current-code re-run in
  study/out/bench_consec_actual.mat has all four QP-internal cases deadline-compliant
  (worst 0.86 s, 100% converged), so the headline sentence must be rewritten against
  fresh data, not merely softened.

### S.10  [reproducible] confirmed

  bench_consecutive.m:27 defaults BENCH_TAG='actual' and saves bench_consec_actual.mat,
  a name no consumer reads (export_data.py:152, make_paper_figures.m:73,
  make_arch_figure.m:27 all hard-load bench_consec_quadprog_final.mat with no fallback
  and no tag ever set anywhere in the repo), so the re-run that landed at 2026-09-06
  00:18:50 sits untracked and unread while figure 3 keeps rendering the 2026-09-05
  14:11:58 pre-8d55826 measurement — and the two disagree materially: worst-case solve
  45.1/77.5 s and 14.9/14.8 % past deadline in the file the paper uses versus 0.86/0.14
  s and 0 % late in the fresh one.

### S.11  [reviewer] confirmed

  Confirmed: the +3254 % headline of Fig. 1 (study/pyfigs/figures.tex:23, computed at
  study/pyfigs/fig01_coupling.py:65-70, echoed at study/pyfigs/README.md:53) divides by
  the median of a distribution that is 49.67 % point mass at zero (745 of 1500 rows,
  cell sigma_nav=4.8/sigma_obj=10), so its denominator 0.11625 mm/s is merely the mean
  of the 5th and 6th smallest non-zero values and a paired bootstrap makes it exactly
  zero in 39.0 % of resamples with a finite-draw ratio interval of [+418 %, +287672 %];
  the direction of the effect is robust (100 % of resamples) and the underlying
  conclusion stands, but the magnitude must be restated as the absolute median shift
  (+3.78 mm/s at sigma_obj=10 vs +0.22 mm/s at 300, a 17.6x contrast) or as the stable
  mean (+62.1 %) or p90 (+50.9 %), with frac_no_manoeuvre disclosed — and unlike figures
  2-8 this caveat appears nowhere in the README's own limitations section.

### S.12  [reviewer] overstated

  Figure 3's headline sentence "enforcing the collision constraint inside the QP does
  not close in real time" is not supported as a causal architectural result, because the
  two populations share no operating point — test_cam_retarget.m:69 sets dsafe0=0 so the
  "proposed" QP has no collision rows at all, runs at h=30 s/Np=60 on a 10 km/s
  conjunction at TCA=10115 s, while bench_consecutive.m probes h=3-10 s/Np=50-150 open-
  loop on the default 10 m/s LVLH fly-by at t=800 s — and one QP-internal configuration
  (h=10, Np=50) meets its own deadline with 0/41 overruns and 84x worst-case margin; the
  caption's numbers are nonetheless exact ("up to 14.9%") and that zero-overrun row is
  plotted and labelled "none past deadline" on the figure itself, so the fix is to
  restate the claim as a problem-size result (median grows 33x from Np=50 to Np=150,
  overruns appear at Np>=100) plus a matched-configuration run, not to withdraw a wrong
  number.

### S.13  [reviewer] overstated

  Fig. 1's headline "+3254 %" (figures.tex:23, README.md:53, computed at
  fig01_coupling.py:57-63) divides by a median of 0.1163 mm/s that sits only five
  samples above a 49.67 % atom of zero-manoeuvre cases, so it swings to ~1605 % under a
  six-sample shift and becomes undefined if the atom reaches 50 % — 0.26 binomial SE
  away — and it inflates by 52x the same effect measured with atom-free statistics
  (+62.1 % on the mean, +50.8 % on the p90); the conditionality itself is real and
  window-independent, driven by the quadrature d_target = 150 + 3*sqrt(sigma_nav^2/3 +
  sigma_obj^2*u'Pu) which shifts +38.6 m at sigma_obj=10 versus +2.8 m at 300 and orders
  monotonically across all four levels on every atom-free metric, so the fix is to print
  a stable estimator and state the miss0 ~ U(30,300) window as a scope limit, not to
  widen the sampling.


## Menores (17)

### M.1  [staleness] overstated

  study/out/chain_summary.mat (header 17:43:23, nGeom = 1500, fresh-regime fzero exactly
  1/1500) holds percentiles bit-identical to the superseded mc_summary.mat and computed
  over a 9000-case single-sigma_obj campaign that the 36000-case mc_cam_*.mat added four
  minutes later in the same commit c2bb207 replaced; unlike mc_summary.mat it is warned
  about nowhere, but nothing in the repo reads it and no paper number rests on it, since
  figures.tex quotes only the current data (+3254 %/+0.5 %, 12.5/53.7 mm/s, both of
  which I reproduced exactly from the CSVs) — and the "8 % of the manoeuvre delta-v"
  claim comes from commit b8d8e89, not from this file, whose own numbers yield 2 %.

### M.2  [staleness] overstated

  The six study/out/mc_cam_*.mat blocks (headers 17:47:25-31) and the mc_surface.mat
  derived from them were produced by the pre-8d55826 reference integrator - provable
  from content rather than timestamps, since the |v_perp| implied by every block's
  stored (dInc, vrel) is 7187.016436408 m/s, matching my re-integration of the un-
  substepped 30 s RK4 reference at t=10110 s to 2.2e-10 m/s and missing the substepped
  one by 1.517e-4 m/s - so mc_cases.csv and Figs. 1, 6 and 7 are not reproducible from
  HEAD and this should be disclosed in study/pyfigs/README.md; but the resulting error
  is bounded, measured over 4800 matched old-vs-new cases at <=0.054 mm/s per case and
  <=0.04 mm/s on any percentile (and <=0.08 mm/s under a maximally adversarial coherent
  perturbation of the campaign's own 36000 rows), leaving the headline 12.5 / 53.7 mm/s
  unchanged at printed precision, so a full six-block re-run is not required before
  submission.

### M.3  [provenance] overstated

  Fig. 8's captioned object-covariance assumption "sigma_obj = 150 m" (figures.tex:137,
  README.md:115, fig08_probability_dilution.py:39 and :146) misdescribes the anisotropic
  ECI ellipsoid diag([150 60 60]) m of cam_demo.m:76, whose B-plane projection has
  principal sigmas of 60.0 and 128.6 m; taken literally it yields Pc(12) = 4.0e-4 and a
  fall ratio of 2.97 instead of the caption's own 8.8e-4 and 5.59, and neither it nor
  the hard-body radius nor the fixed miss is exported to pc_nav_sweep.csv, so the Pc
  column cannot be re-derived from the supplementary tables — but the plotted data
  reproduces exactly from cam_demo.mat (3e-11), the mis-sourced fixed miss from
  meta['execution'] differs from the correct value by 0.35 mm and prints as "120" either
  way, and the 120 m and 150 m shared with test_cam_retarget.mat are
  get_conjunction_scenario.m's documented defaults, not a coincidence.

### M.4  [provenance] confirmed

  The Fig. 2 caveat at study/pyfigs/README.md:87-89 is factually wrong — recomputed from
  the same data/koz_profiles.csv the figure reads, the max-min spread of the three per-
  step curves crosses 2 m at t ~ 78 s and reaches 6.59 m at t = 150 s
  (fig02_koz_consistency.py:165-166 itself prints 3.31 m at t = 100 s, and the rendered
  fig02 PNG shows the red band visibly splitting from t ~ 115 s), so the threshold
  should read t ~ 80 s rather than 150 s; the misstatement is likely a max-min-versus-
  closest-pair slip (h = 4 vs h = 5 does cross 2 m at t = 154.5 s), is confined to the
  internal README caveat since the Fig. 2 caption in figures.tex is correct, and errs
  against the paper's own argument by making the per-step scheme look less mesh-
  dependent than it is.

### M.5  [provenance] confirmed

  study/pyfigs/README.md:99-101 (copying the fig05_navigation.py:251-252 printout, which
  pairs `len(GAP)`=12 with an unrelated statistic) claims "las 12 ventanas coinciden a
  1.7e-5 m", but the metric at fig05_navigation.py:100-101 compares only windows sharing
  an exact elapsed time, so eleven windows agree to 1.588e-05 m for elapsed 2-182 s,
  only ten for elapsed 185-299 s where the quoted 1.748e-05 m maximum actually occurs,
  and the 40 s window — the very one the figures.tex:85 caption's "whatever the length
  of the gap" rests on — is never compared with any other because its samples fall on a
  disjoint 3 s grid (elapsed 3,6,...,39 s, thirteen singleton buckets of zero spread).

### M.6  [claims] overstated

  The avoidance-manoeuvre law is implemented three times — matlab/plan_cam.m (backing
  fig04 and, via primary_timing.mat saved at test_cam_retarget.m:145, the proposed-
  architecture branch of fig03), an inlined copy at study/mc_cam.m:140-155 that is
  algorithmically identical to it (fig01, fig06, fig07), and study/cam_demo.m:174-200
  solveForMiss, which is the pre-fix positive-dv-only secant algorithm that commit
  c2bb207 declared wrong yet added in that same commit, and which supplies only the
  "25.2 -> 60.5 mm/s" row of fig08(b); the duplication is a drift hazard rather than a
  wrong result, mc_cam additionally omits plan_cam's post-solve verification so
  mc_cases.csv's `converged` column is 1 for all 36,000 rows, and whether fig08's two dv
  values are actually inflated could not be verified without running MATLAB.

### M.7  [claims] overstated

  The `converged` column of the supplementary mc_cases.csv is the constant 1 across all
  36000 rows because study/mc_cam.m:156 tests only `ok = isfinite(dv)` instead of
  matlab/plan_cam.m:101-108's 0.5 m re-propagation check (and mc_cam.m:110-115 discards
  ten exact miss-curve points that would have supported one), so the column should be
  renamed or dropped before the CSVs ship — but it is read by no figure script and no
  caption, study/pyfigs/README.md already states that no column records whether a case
  reached d_target, and an independent re-propagation of 493 solved cases out to 205
  mm/s shows the +/-5 mm/s parabola is accurate to 0.0995% in dv (max miss residual 0.91
  m on a 957 m target, p90 shifting +0.02%), so neither the p90 = 53.7 mm/s tank figure
  nor fig01's +3254% headline — whose two operands, 0.116 and 3.899 mm/s, both lie
  inside the probe interval — is affected.

### M.8  [claims] overstated

  The van Loan discretisation (matlab/MPC_INOAS.m:1020, called only at :319 and :390,
  both gated behind `dsafe0 > 0`) is exercised by the QP-internal runs behind fig02 and
  fig03a (dsafe0 = 150 inherited from initialize_inoas_simulation.m:563) but not by the
  plan+track run behind fig03b and fig04, which sets `dsafe0 = 0` at
  study/test_cam_retarget.m:69 and applies its 3-sigma margin in guidance via a static
  d_target instead; fig02's numbers are nonetheless fully reproducible (41.2918 m per-
  step spread and exactly zero van Loan spread at t = 500 s, recomputed from
  data/koz_profiles.csv) and no existing caption misattributes them, so this is a
  caption-scoping item for the paper text, not a defect in the data.

### M.9  [claims] overstated

  models/inoas_model.slx contains no reference to plan_cam, get_conjunction_scenario or
  any debris symbol and runs only the 10 m/s co-orbital companion of
  initialize_inoas_simulation.m:540-541 with P_debris_pos = diag([5 5 5].^2) (:623)
  under the default "sphere_grid" mode (MPC_INOAS.m:792), so every conjunction result in
  the paper is produced by study/ scripts calling MPC_INOAS outside Simulink (only fig.
  5's navigation record comes from the .slx, via run_navigation_chain.m:60) and the
  paper must scope its integrated-model claim accordingly -- but the init comment at
  :616-622 says only that the catalogue covariance would collapse the QP in the model's
  own 50 m baseline, not that the paper's 120 m conjunction collapses it, and that
  conjunction demonstrably converges (miss 120.00 -> 257.39 m against a 257.39 m target,
  data/meta.json) in a closed-loop, non-Simulink MPC run.

### M.10  [reproducible] overstated

  `study/pyfigs/export_data.py:202` hardcodes `'u_max_ms2': 1.0 / 24.0` — the only non-
  derived value among all META entries — because `study/out/test_cam_retarget.mat`
  genuinely omits u_max (verified by whosmat: only H_MPC, NP_MPC, PLAN, SC, Tf,
  d_target, dv_cum, err, sigma_b, tv, un), so figure 4's "0.095 % of the 1 N authority"
  (figures.tex:70) is currently correct and reproduces exactly from execution.csv
  (39.556 um/s^2 / 41666.7 = 0.0949 %) but duplicates a constant defined at
  initialize_inoas_simulation.m:250 and overridable at 252-254; since plan_cam.m never
  reads u_max and the MPC box bound sits 1000x from active, changing m_sat or F_control
  would regenerate every .mat and CSV identically while meta.json and the caption
  silently kept 1/24 — a real provenance gap with precedent (expA_*.mat still store the
  pre-12U u_max = 0.05 while expB_*.mat store 1/24), but not a wrong published number.

### M.11  [reproducible] overstated

  study/README.md (blob 8b16c236, unchanged since c2bb207 and therefore predating both
  later figure generations) ends its reproduction recipe at lines 15-18 with the four
  MATLAB scripts that render the superseded study/figures sets, never names study/pyfigs
  (which no file outside that directory references anywhere in the repo), and omits the
  only producers of two of export_data.py's nine inputs - cam_demo.mat and
  bench_consec_quadprog_final.mat, the latter requiring the undocumented
  BENCH_TAG='quadprog_final' - so fig03 and fig08 cannot be regenerated end-to-end from
  any documentation; the recipe's campaign half is however still current (it produces
  seven of the nine inputs, including koz_profiles.mat via make_figures.m:116, which the
  auditor's proposed fix would wrongly drop), and the paper1-6/fig04 name overlap is
  cosmetic since graphicspath resolves only to figures_py/.

### M.12  [reproducible] overstated

  study/test_cam_retarget.m bypasses inoas_setup.m entirely (line 26 `REPO = REPO_ROOT`,
  line 40), so the work/cN isolation cannot be pointed at the figure-4 producer and each
  run unconditionally rewrites the untracked scratch files data/referenceTrajectory.mat
  and data/debrisTrajectory.mat — a latent, already-documented (study/README.md:37-41)
  millisecond-wide race between the save at get_nominal_trajectory.m:32 and the read-
  back at initialize_inoas_simulation.m:431, with no observed instance and no effect on
  figure 4, whose execution.csv is a uniform 30 s grid over 3180-10140 s reproducing
  every caption number exactly; the `STUDY_FORCE_MAIN = true` at line 25 is dead code
  with no consumer in this script, so deleting it would fix nothing.

### M.13  [reproducible] confirmed

  README.md:91 and matlab/README.md:24 still index matlab/instrument_decision.m,
  deliberately deleted in commit a7701b4 as a non-executing duplicate of the Stateflow
  FSM that actually runs (models/inoas_model.slx, simulink/stateflow/chart_72.xml,
  "Instrument Decision/Instrument Decision FSM", 60/300 windows vs the deleted copy's
  90/10), while the same self-declared "file-by-file guide" omits 7 of the 21 files in
  matlab/ - including plan_cam.m, which appears in no .md or .tex file anywhere in the
  repository and is called only from study/test_cam_retarget.m - and README.md:74-85
  omits the tracked study/ tree; a documentation-navigation defect only, since
  docs/model-architecture.md:19-30 describes the FSM in prose and commit a7701b4 plus
  study/patch_charts.m:83-110 preserve its rationale and edit history.

### M.14  [reviewer] overstated

  The Fig. 6 caption's head-on closure claim (figures.tex:95-100) is an analytic limit
  at Δi = 180 deg while study/mc_cam.m:90 samples Δi = 20 + 140*rand, so the 1500
  geometries stop at 159.94 deg where the bound is still 3.57 of 20.23 m per mm/s (17.7%
  of maximum) — but the caption already labels the envelope "analytic", the plot marks 2
  v_orb beyond the data, and the bound is attained by the data (max |sens|/bound =
  0.9995 overall, p90 0.98 in the top v_rel decile), so the defect is only that the
  sampled Δi range is disclosed nowhere in figures.tex, README.md or meta.json, fixable
  with one clause rather than a restatement or a re-run.

### M.15  [reviewer] overstated

  mc_cam.m propagates 13 miss-distance nodes per geometry but identifies the parabola
  from only the +/-5 mm/s stencil and saves none of the other 10, so the 79.6 % of Delta
  v values falling outside that stencil (including the 12.5 mm/s median and 53.7 mm/s
  p90 budget behind figures 1, 6 and 7) cannot be residual-checked from the repo; the
  extrapolation itself is very likely sound — cam_demo.mat's six truly propagated points
  fit a single quadratic to within 15 mm out to 60.5 mm/s, and figures 4 and 8 do not
  use the parabola at all — so this is a traceability gap rather than a wrong number,
  and the sharper defect is that mc_cam.m reimplements plan_cam.m's planner with a 4x
  narrower probe while dropping its |miss_after - d_target| < 0.5 m verification guard,
  leaving mc_cases.csv's "converged" column meaning only that the discriminant was non-
  negative.

### M.16  [reviewer] overstated

  study/mc_cam.m:140-156 inlines the parabola inversion instead of calling
  matlab/plan_cam.m, but the inlined solver is a bit-exact transcription of
  plan_cam.m:83-97 written in the same commit (c2bb207) — verified identical on 278,201
  random parabolas — so the real gap is not a different guidance law but (i) the absence
  of plan_cam.m:103-104's achieved-miss verification, which leaves the exported
  `converged` column true for all 36,000 rows and mc_surface.m:40's `& ok` filter inert,
  and (ii) an undocumented probe-size difference (5 vs 20 mm/s) whose numerical effect
  is unverified; the genuinely divergent planner is study/cam_demo.m:174-200, which
  still uses the positive-only secant that commit c2bb207 declared wrong and which feeds
  fig08.

### M.17  [reviewer] overstated

  The `converged` column of mc_cases.csv (study/pyfigs/export_data.py:63,72) exports
  mc_cam.m's `ok` flag, which is provably 1 for all 36000 rows -- in the manoeuvre
  branch m_min <= miss0 < d_target forces disc > 0 whenever Aq > 0 -- so it records only
  "the local parabola had a real root" and not plan_cam.m's genuine feasibility test
  (matlab/plan_cam.m:101-104, a nonlinear re-propagation checking abs(miss_after -
  d_target) < 0.5); the defect is a reviewer-facing naming and documentation gap only,
  because no figure script or caption reads the column, the `& ok` filters in
  mc_analyse.m:47 and mc_surface.m:39 are no-ops that corrupt no percentile, and
  study/pyfigs/README.md:105-107 already states that no column records whether a case
  reached d_target -- and the suggested remedy of evaluating the parabola at the
  returned dv is a tautology returning d_target exactly, since miss_curve is never
  stored, so the real check requires re-running the campaign.


## Notas (14)

### N.1  [provenance] overstated

  The h = 10 s / Np = 50 case genuinely has 0/41 overruns (worst 0.119 s against its 10
  s deadline, 84x margin), so Fig. 3's opening clause is an architectural generalisation
  with one exception among its four QP-internal tunings — but that exception is not
  hidden in the CSV: fig03_architecture_cost.py:157-160 draws it in the published figure
  as a green "none" in the "past deadline:" row with its 119 ms worst case labelled
  above its own dashed 10 s deadline, the script docstring (line 29) writes the zero
  out, and figures.tex:50 already says "up to 14.9 %", leaving only an optional caption-
  wording nicety rather than a defect, while the proposed fix's premise is wrong because
  inoas_setup.m:47 gives that case the same 500 s horizon as the h = 5 s / Np = 100
  case.

### N.2  [provenance] overstated

  Fig. 4's equal "257.4 m achieved / 257.4 m required" is correct and traceable rather
  than concealing anything — plan_cam.m:80-104 root-solves the impulse so that the miss
  equals d_target by construction, with a declared 0.5 m acceptance band (line 104) that
  the 1.29 mm closed-loop residual sits far inside — so the only defensible action is to
  add one sentence stating that the planner solves d_target exactly, since no paper-
  facing text (figures.tex:64-73, README.md Fig. 4 entries) says so; the claim that a
  19.8 m shift in d_target would "flip the result" is mechanically wrong, because
  d_target is an input to that root solve and shifting it moves the achieved miss with
  it, changing only the propellant (retarget_sweep.csv: d_target 229.9 -> 787.2 m, |dv|
  10.5 -> 50.2 mm/s).

### N.3  [provenance] overstated

  Only one of the three is a hardcoded constant that matters - export_data.py:202 writes
  u_max as the literal 1.0/24.0 because test_cam_retarget.m:108 does not save u_max to
  the .mat, a latent drift risk that is currently numerically correct and is duly
  exported to meta.json and read by fig04_execution.py:43 - while fig06's V_ORB = 7.188
  is a plotted-envelope constant, not a caption number, and is exactly recoverable as
  (2*pi*mu/T_orb_s)^(1/3) from meta.json, and Fig. 5's 19.45 % is not hardcoded at all
  but exported as meta.json nav.on_fraction (export_data.py:227) and read by
  fig05_navigation.py:59, leaving only a documented 0.02 pp endpoint-convention gap
  against a naive 4001-row mean of nav_duty.csv.

### N.4  [provenance] confirmed

  Fig. 6's caption states an analytic limit ("the envelope closes as v_rel -> 2 v_orb,
  so no amount of Delta v opens the miss") that the plotted dataset cannot exhibit — the
  sample stops at v_rel = 14.154 km/s, 98.46 % of 2 v_orb, with max |sens|/envelope =
  0.9995 and zero violations — while mc_cases.csv's 'converged' column is 1 in all 36000
  rows not by chance but by construction, since study/mc_cam.m:132-156 only reaches the
  root solve when miss0 < d_target, which makes the discriminant strictly positive for
  any convex miss^2 fit; the column therefore records nothing, and the feasibility
  caveat that README.md:105-107 states plainly is the one half of the README's Fig. 6
  note that the caption drops, even though it already carries the other half and already
  labels the curve "analytic" — no figure reads the column and no published number is
  wrong, so this is a caption-wording and data-hygiene issue, not a correctness one.

### N.5  [claims] overstated

  The "bplane_tca" QP constraint (MPC_INOAS.m:245-341) is reachable only from
  study/test_bplane_mpc.m and feeds none of the eight figures, but this is a documented
  opt-in variant ("all opt-in, default behaviour unchanged", commit 8232f4f) that was
  deliberately superseded by the plan+track architecture in the very next commit
  (7f75009) and is claimed nowhere in figures.tex, study/pyfigs/README.md or docs/ — so
  the only exposure is that the paper's methods text must not describe it as part of the
  delivered controller; note also that the delivered pipeline sets dsafe0 = 0
  (study/test_cam_retarget.m:69) and therefore carries no debris constraint in the QP at
  all, and that B-plane geometry is not dead since matlab/cam_bplane.m feeds fig08
  through study/cam_demo.m and cam_demo.mat.

### N.6  [claims] overstated

  MPCObjectiveScaled (matlab/MPC_INOAS.m:884-914) is genuinely unreachable dead code
  orphaned by commit 5d090b1, which deleted its sole call site while leaving the body —
  and it now encodes a materially different cost than the live QP (raw-slack penalty vs
  Dsl-scaled slack, Q vs Q_eff) — but no published number depends on it, the two config
  variables bundled into the finding are live reads of a deliberately-documented
  default-with-override idiom rather than dead code, and the proposed deletion would
  erase the last in-repo trace of the fmincon baseline the paper's timing figure
  indicts.

### N.7  [claims] overstated

  matlab/ and study/ each carry cam_bplane.m, cam_find_tca.m and inoas_dyn.m, but the
  pairs are the identical git blob (3eac525c, 4dfc521c, 491e0ae0) with the working-tree
  md5 difference caused solely by core.autocrlf=true, and the claimed cause of shadowing
  is wrong — in study/test_cam_retarget.m, study/test_bplane_mpc.m and study/mc_cam.m
  the last addpath before any helper call is initialize_inoas_simulation.m:31
  `addpath(genpath(matlab))`, which puts matlab/ ahead of study/ — so this is a zero-
  impact duplicate-source-of-truth hygiene note (latent drift risk, no drift to date, no
  paper number affected), not a path-resolution defect.

### N.8  [reproducible] overstated

  study/pyfigs ships no dependency manifest and pins no matplotlib version (the only one
  in the repo, tools/visualization/requirements.txt, is the unrelated GIF pipeline's),
  which is worth fixing for tidiness, but the alleged harm does not occur: the fallback
  at inoas_style.py:58 resolves to STIXGeneral, which is bundled with every matplotlib
  install and which I measured at within +1.46% max / +0.25% mean of Times New Roman
  across the eleven real fig04 label strings, so a reviewer without Times gets a
  typographically equivalent figure rather than the "different metrics and possibly
  overlapping annotations" claimed, and no paper number depends on the renderer since
  all eight figures read committed CSVs.

### N.9  [reproducible] overstated

  README.md:19-20 still carries the Student Aerospace Challenge headline numbers with no
  staleness marker, and one of them is superseded - the 82% GNSS reduction is the
  complement of the 18.23% duty cycle that study/run_navigation_chain.m:9-11 says came
  from a counter known to be wrong, against 19.45% on-time recomputed from nav_duty.csv
  - but the rest of the finding does not hold: docs/results.md is explicitly disclaimed
  at lines 3-10 by commit c2bb207, 154 m reproduces exactly as 150 + 0.2*sqrt(3*12^2)
  from initialize_inoas_simulation.m:563 and study/inoas_setup.m:63-64, and it does not
  contradict figure 4, whose 257.4 m is 5 + 3*84.13 (hard-body radius plus 3 sigma in
  the B-plane) for a 10 km/s crossing rather than the 10 m/s SAC fly-by.

### N.10  [reproducible] overstated

  .gitignore has no `work/` entry, so the ~18 MB of per-block repo copies that
  study/sync_work.m:25-36 writes into study/work/c1..c6 appear as untracked files -- a
  cosmetic hygiene gap only, since the README documents sync_work as the prerequisite at
  study/README.md:39-41 and the parallel recipe does not silently degrade:
  mc_cam.m:42-44 has no directory-existence fallback, so a missing MC_REPO makes
  mc_cam.m:76 fail loudly with MATLAB:run:FileNotFound rather than racing on the shared
  data/referenceTrajectory.mat (the cited inoas_setup.m:29-35 fallback is a documented
  single-process choice on a code path the recipe never touches).

### N.11  [reproducible] overstated

  cam_bplane.m, cam_find_tca.m and inoas_dyn.m are duplicated between matlab/ and study/
  (copied verbatim in c2bb207 and the same git blob at every commit since, so no result
  can differ between them), but the claimed shadowing direction is wrong:
  study/test_cam_retarget.m:40 runs initialize_inoas_simulation.m, whose unconditional
  addpath(genpath(repoRoot/matlab)) at lines 30-33 executes after the last
  addpath(STUDY_DIR) at line 28 and before the cam_find_tca call at line 97, so matlab/
  resolves there and the finding reduces to a duplication-hygiene note with no bearing
  on the 257.4 m miss distance in figures.tex:71-72.

### N.12  [reproducible] overstated

  study/pyfigs/export_data.py:57 discovers Monte Carlo blocks with a filename-probing
  while-loop that stops at the first gap, and neither the blocks (study/mc_cam.m:170
  saves no MC_CHUNK/MC_NCHUNK/MC_N) nor meta.json records how many were read, so a
  changed block count would move mc_cases.csv, mc_percentiles.csv and mc_marginal.csv
  with no assertion firing — but the change is recorded, not silent (the n columns, now
  1500 per cell and 6000 marginal, move with it), and its size is sampling noise rather
  than a shift: dropping any one of the six blocks moves the quoted 53.7 mm/s p90 only
  to 52.5-54.3 mm/s, less than the 1.05 mm/s standard error of the estimate itself and
  nothing like problem B's 62.1 -> 53.7.

### N.13  [reviewer] overstated

  Fig. 6's caption and the pyfigs README never state that the vertical scatter beneath
  the cos(Delta i/2) envelope is the B-plane miss-vector orientation |sin(bplane_angle)|
  - an independently sampled U(0,360 deg) nuisance that modulates |sens| about 3x -
  which is a one-clause documentation gap rather than a misattributed mechanism, because
  the head-on claim, the envelope framing and panel (b)'s 3.1x factor (3.05x with
  B-plane orientation held fixed) all survive intact, and on the quantity that reaches
  the paper, required Delta v, head-on geometry costs 3.09x the median against only
  1.28x for the weakest 5% of B-plane orientations, whose vanishing first-order
  sensitivity is recovered by the exactly quadratic miss-versus-dv relation.

### N.14  [reviewer] overstated

  The two ends of the "4.8-32.1 m" navigation band are indeed computed by different
  statistics (export_data.py:228 takes the 0-30 s bin median from
  run_navigation_chain.m:97, export_data.py:229 takes a global maximum), but this is a
  labelling nit rather than a defect: sigma saturates on a plateau so the global max and
  the like-for-like 270-300 s bin median differ by 6.1e-5 m, the mc_surface.csv
  sigma_nav grid is seeded exactly at [4.8, ..., 32.1] so fig01's snapped +3254 / +57 /
  +5.0 / +0.5 % sensitivities are bit-identical under either proposed fix, the shaded
  region would move 0.36 % of the log-axis width, and both endpoints are already
  documented as a post-fix floor and a saturation ceiling in nav_law.csv's header note,
  fig05_navigation.py's docstring and the figures.tex captions.


## Descartados por la verificacion (8)

- [staleness] The provenance of the Monte Carlo blocks cannot be established from the repository: the isolated wor
- [staleness] bench_consecutive.m is the one script where matlab/ outranks study/, a live trap for its documented 
- [provenance] Fig. 6's 'factor of 3.1' is specific to the most optimistic (sigma_nav, sigma_obj) cell, which the c
- [reproducible] Two MATLAB toolboxes needed to regenerate figure 4's .mat are not in the requirements list, and the 
- [reproducible] The Simulink model's MPC block is hardwired to Np = 125, so figure 4's configuration cannot be run i
- [reviewer] 32.1 m is not a forced operating point: sigma recovers to the floor in 40 s and 20 GNSS windows fall
- [reviewer] fig02's van Loan curve sits 112-154 m above the per-step curves and above the operational cap, which
- [reviewer] The p99 column is numerically the maximum for every QP-internal configuration
