# Localisation benchmark: the unchanged filters on a new network: San Francisco (SoMa), Cybercab-inspired proxy

Open loop: each drive is recorded once with ground-truth driving; every estimator then processes the same sensor log in the same arrival order and is scored against the same truth in the fixed map frame, with no per-trajectory alignment. Errors are rear-axle positions sampled at 50 Hz from each estimator's initialisation; lateral error is the part across the true heading. A run fails when the position error ever exceeds 5.0 m after initialising (failed runs stay in every statistic).

Drives: 120 (2 routes x 4 conditions x 15 seeds); drives not completing their route: 0. Toolchain: MSVC 195136260. Engine commit at configure: 5f2c004f82d02e0c9de5aae3f6b1b5dcc1747330.

Vehicle: cybercab_proxy_v1 (kinematic bicycle (Cybercab-inspired proxy; dimensions are research assumptions)); seeds 1 to 15. The extended metrics of every run are in runs.jsonl.

## nominal_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.62 (3) | 0.94 | 1.54 | 23 | - | - | - | - | - | - | 7.3 |
| ekf_int | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.62 (3) | 0.94 | 1.54 | 23 | 0 | 0 | - | - | - | - | 7.6 |
| ekf_int2 | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.62 (3) | 0.94 | 1.54 | 23 | 0 | 0 | - | 0 | 0 | - | 7.6 |
| ekf_kerb | 30 | 0 | 0.089 | 0.109 | 0.232 | 0.65 | 0.043 | 0.09 | - | - | 1.23 (3) | 0.89 | 1.57 | 25 | 0 | 0 | 100.0% | - | - | - | 115.0 |
| ekf_kerb2 | 30 | 0 | 0.089 | 0.109 | 0.232 | 0.65 | 0.043 | 0.09 | - | - | 1.23 (3) | 0.89 | 1.57 | 25 | 0 | 0 | 100.0% | 0 | 0 | 0 | 114.2 |
| dr | 30 | 21 | 4.593 | 5.075 | 9.397 | 11.86 | 6.627 | 1.09 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.456 | 0.458 | 0.671 | 2.80 | 0.483 | 20.80 | - | - | 2.51 (2) | 0.92 | 1.38 | 8 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 0 | 1.280 | 1.268 | 1.946 | 4.20 | 0.947 | 22.80 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 0 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## gnss_outage_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | - | - | - | - | - | - | 7.1 |
| ekf_int | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | - | - | - | 7.5 |
| ekf_int2 | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | 0 | 0 | - | 7.4 |
| ekf_kerb | 30 | 0 | 0.161 | 0.195 | 0.469 | 0.91 | 0.043 | 0.09 | 0.03 | 0.0 | 1.36 (3) | 0.89 | 1.57 | 14 | 0 | 0 | 100.0% | - | - | - | 118.1 |
| ekf_kerb2 | 30 | 0 | 0.161 | 0.195 | 0.469 | 0.91 | 0.043 | 0.09 | 0.03 | 0.0 | 1.36 (3) | 0.89 | 1.57 | 14 | 0 | 0 | 100.0% | 0 | 0 | 0 | 116.7 |
| dr | 30 | 21 | 4.593 | 5.075 | 9.397 | 11.86 | 6.627 | 1.09 | 4.75 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 26 | 3.483 | 4.331 | 10.370 | 25.04 | 5.612 | 20.82 | 9.84 | 0.7 | 2.22 (2) | 0.76 | 1.39 | 4 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 64.945 | 62.924 | 169.243 | 203.54 | 0.903 | 22.82 | 201.90 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 0 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## gnss_drift_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 2.037 | 2.052 | 4.111 | 4.90 | 3.777 | 0.96 | - | - | 151.14 (3) | 0.47 | 2.92 | 594 | - | - | - | - | - | - | 7.3 |
| ekf_int | 30 | 0 | 2.019 | 2.017 | 4.101 | 4.66 | 3.774 | 0.94 | - | - | 149.75 (3) | 0.47 | 2.27 | 360 | 0 | 3 | - | - | - | - | 7.5 |
| ekf_int2 | 30 | 0 | 2.019 | 2.017 | 4.101 | 4.66 | 3.774 | 0.94 | - | - | 149.75 (3) | 0.47 | 2.27 | 360 | 0 | 3 | - | 0 | 0 | - | 7.7 |
| ekf_kerb | 30 | 0 | 0.419 | 0.998 | 2.560 | 4.11 | 0.661 | 0.11 | - | - | 36.11 (3) | 0.73 | 21.41 | 6273 | 0 | 7 | 94.5% | - | - | - | 118.6 |
| ekf_kerb2 | 30 | 0 | 0.419 | 0.902 | 2.288 | 4.11 | 0.046 | 0.09 | - | - | 25.04 (3) | 0.78 | 22.91 | 7238 | 0 | 3 | 98.9% | 0 | 1 | 16 | 118.7 |
| dr | 30 | 21 | 4.593 | 5.075 | 9.397 | 11.86 | 6.627 | 1.09 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 2.041 | 2.016 | 4.100 | 4.90 | 3.588 | 20.87 | - | - | 48.93 (2) | 0.54 | 1.38 | 8 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 15 | 2.279 | 2.266 | 4.590 | 6.38 | 3.615 | 22.86 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 1 of 30 runs; ekf_kerb2 vs ekf_kerb in 13 of 30 runs; ekf_int2 vs ekf in 3 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## kerb_map_error_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | - | - | - | - | - | - | 7.1 |
| ekf_int | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | - | - | - | 7.3 |
| ekf_int2 | 30 | 0 | 0.424 | 0.439 | 1.031 | 2.21 | 0.957 | 0.40 | 1.22 | 0.2 | 1.70 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | 0 | 0 | - | 7.3 |
| ekf_kerb | 30 | 0 | 0.685 | 0.700 | 1.674 | 2.61 | 0.623 | 0.14 | 1.65 | 3.1 | 52.03 (3) | 0.63 | 2.12 | 396 | 0 | 8 | 84.5% | - | - | - | 120.8 |
| ekf_kerb2 | 30 | 0 | 0.671 | 0.680 | 1.669 | 2.81 | 0.613 | 0.09 | 1.70 | 2.1 | 50.70 (3) | 0.67 | 1.96 | 436 | 0 | 7 | 98.2% | 0 | 0 | 72 | 122.9 |
| dr | 30 | 21 | 4.593 | 5.075 | 9.397 | 11.86 | 6.627 | 1.09 | 4.75 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 26 | 3.483 | 4.331 | 10.370 | 25.04 | 5.612 | 20.82 | 9.84 | 0.7 | 2.22 (2) | 0.76 | 1.39 | 4 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 64.945 | 62.924 | 169.243 | 203.54 | 0.903 | 22.82 | 201.90 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 21 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 4 of 30 runs.

## Failed runs

- SF-A nominal_sf seed 1: dr failed (max error 11.86 m)
- SF-A nominal_sf seed 4: dr failed (max error 7.96 m)
- SF-A nominal_sf seed 5: dr failed (max error 6.59 m)
- SF-A nominal_sf seed 6: dr failed (max error 10.71 m)
- SF-A nominal_sf seed 7: dr failed (max error 8.03 m)
- SF-A nominal_sf seed 8: dr failed (max error 5.06 m)
- SF-A nominal_sf seed 9: dr failed (max error 6.52 m)
- SF-A nominal_sf seed 10: dr failed (max error 7.22 m)
- SF-A nominal_sf seed 12: dr failed (max error 6.27 m)
- SF-A nominal_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_outage_sf seed 1: dr failed (max error 11.86 m)
- SF-A gnss_outage_sf seed 1: gnss_cv failed (max error 11.86 m)
- SF-A gnss_outage_sf seed 1: gnss_hold failed (max error 202.69 m)
- SF-A gnss_outage_sf seed 2: gnss_cv failed (max error 6.20 m)
- SF-A gnss_outage_sf seed 2: gnss_hold failed (max error 202.88 m)
- SF-A gnss_outage_sf seed 3: gnss_cv failed (max error 9.49 m)
- SF-A gnss_outage_sf seed 3: gnss_hold failed (max error 203.50 m)
- SF-A gnss_outage_sf seed 4: dr failed (max error 7.96 m)
- SF-A gnss_outage_sf seed 4: gnss_cv failed (max error 19.48 m)
- SF-A gnss_outage_sf seed 4: gnss_hold failed (max error 201.83 m)
- SF-A gnss_outage_sf seed 5: dr failed (max error 6.59 m)
- SF-A gnss_outage_sf seed 5: gnss_cv failed (max error 24.82 m)
- SF-A gnss_outage_sf seed 5: gnss_hold failed (max error 203.13 m)
- SF-A gnss_outage_sf seed 6: dr failed (max error 10.71 m)
- SF-A gnss_outage_sf seed 6: gnss_cv failed (max error 5.31 m)
- SF-A gnss_outage_sf seed 6: gnss_hold failed (max error 202.67 m)
- SF-A gnss_outage_sf seed 7: dr failed (max error 8.03 m)
- SF-A gnss_outage_sf seed 7: gnss_cv failed (max error 21.00 m)
- SF-A gnss_outage_sf seed 7: gnss_hold failed (max error 203.13 m)
- SF-A gnss_outage_sf seed 8: dr failed (max error 5.06 m)
- SF-A gnss_outage_sf seed 8: gnss_hold failed (max error 202.36 m)
- SF-A gnss_outage_sf seed 9: dr failed (max error 6.52 m)
- SF-A gnss_outage_sf seed 9: gnss_cv failed (max error 11.15 m)
- SF-A gnss_outage_sf seed 9: gnss_hold failed (max error 202.46 m)
- SF-A gnss_outage_sf seed 10: dr failed (max error 7.22 m)
- SF-A gnss_outage_sf seed 10: gnss_cv failed (max error 8.64 m)
- SF-A gnss_outage_sf seed 10: gnss_hold failed (max error 201.93 m)
- SF-A gnss_outage_sf seed 11: gnss_cv failed (max error 11.83 m)
- SF-A gnss_outage_sf seed 11: gnss_hold failed (max error 202.93 m)
- SF-A gnss_outage_sf seed 12: dr failed (max error 6.27 m)
- SF-A gnss_outage_sf seed 12: gnss_hold failed (max error 202.85 m)
- SF-A gnss_outage_sf seed 13: gnss_cv failed (max error 19.55 m)
- SF-A gnss_outage_sf seed 13: gnss_hold failed (max error 203.34 m)
- SF-A gnss_outage_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_outage_sf seed 14: gnss_cv failed (max error 10.02 m)
- SF-A gnss_outage_sf seed 14: gnss_hold failed (max error 201.72 m)
- SF-A gnss_outage_sf seed 15: gnss_cv failed (max error 14.65 m)
- SF-A gnss_outage_sf seed 15: gnss_hold failed (max error 202.16 m)
- SF-A gnss_drift_sf seed 1: dr failed (max error 11.86 m)
- SF-A gnss_drift_sf seed 2: gnss_hold failed (max error 6.22 m)
- SF-A gnss_drift_sf seed 3: gnss_hold failed (max error 6.36 m)
- SF-A gnss_drift_sf seed 4: dr failed (max error 7.96 m)
- SF-A gnss_drift_sf seed 5: dr failed (max error 6.59 m)
- SF-A gnss_drift_sf seed 5: gnss_hold failed (max error 5.13 m)
- SF-A gnss_drift_sf seed 6: dr failed (max error 10.71 m)
- SF-A gnss_drift_sf seed 6: gnss_hold failed (max error 5.53 m)
- SF-A gnss_drift_sf seed 7: dr failed (max error 8.03 m)
- SF-A gnss_drift_sf seed 8: dr failed (max error 5.06 m)
- SF-A gnss_drift_sf seed 9: dr failed (max error 6.52 m)
- SF-A gnss_drift_sf seed 9: gnss_hold failed (max error 5.25 m)
- SF-A gnss_drift_sf seed 10: dr failed (max error 7.22 m)
- SF-A gnss_drift_sf seed 12: dr failed (max error 6.27 m)
- SF-A gnss_drift_sf seed 12: gnss_hold failed (max error 6.38 m)
- SF-A gnss_drift_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_drift_sf seed 14: gnss_hold failed (max error 5.69 m)
- SF-A gnss_drift_sf seed 15: gnss_hold failed (max error 6.36 m)
- SF-A kerb_map_error_sf seed 1: dr failed (max error 11.86 m)
- SF-A kerb_map_error_sf seed 1: gnss_cv failed (max error 11.86 m)
- SF-A kerb_map_error_sf seed 1: gnss_hold failed (max error 202.69 m)
- SF-A kerb_map_error_sf seed 2: gnss_cv failed (max error 6.20 m)
- SF-A kerb_map_error_sf seed 2: gnss_hold failed (max error 202.88 m)
- SF-A kerb_map_error_sf seed 3: gnss_cv failed (max error 9.49 m)
- SF-A kerb_map_error_sf seed 3: gnss_hold failed (max error 203.50 m)
- SF-A kerb_map_error_sf seed 4: dr failed (max error 7.96 m)
- SF-A kerb_map_error_sf seed 4: gnss_cv failed (max error 19.48 m)
- SF-A kerb_map_error_sf seed 4: gnss_hold failed (max error 201.83 m)
- SF-A kerb_map_error_sf seed 5: dr failed (max error 6.59 m)
- SF-A kerb_map_error_sf seed 5: gnss_cv failed (max error 24.82 m)
- SF-A kerb_map_error_sf seed 5: gnss_hold failed (max error 203.13 m)
- SF-A kerb_map_error_sf seed 6: dr failed (max error 10.71 m)
- SF-A kerb_map_error_sf seed 6: gnss_cv failed (max error 5.31 m)
- SF-A kerb_map_error_sf seed 6: gnss_hold failed (max error 202.67 m)
- SF-A kerb_map_error_sf seed 7: dr failed (max error 8.03 m)
- SF-A kerb_map_error_sf seed 7: gnss_cv failed (max error 21.00 m)
- SF-A kerb_map_error_sf seed 7: gnss_hold failed (max error 203.13 m)
- SF-A kerb_map_error_sf seed 8: dr failed (max error 5.06 m)
- SF-A kerb_map_error_sf seed 8: gnss_hold failed (max error 202.36 m)
- SF-A kerb_map_error_sf seed 9: dr failed (max error 6.52 m)
- SF-A kerb_map_error_sf seed 9: gnss_cv failed (max error 11.15 m)
- SF-A kerb_map_error_sf seed 9: gnss_hold failed (max error 202.46 m)
- SF-A kerb_map_error_sf seed 10: dr failed (max error 7.22 m)
- SF-A kerb_map_error_sf seed 10: gnss_cv failed (max error 8.64 m)
- SF-A kerb_map_error_sf seed 10: gnss_hold failed (max error 201.93 m)
- SF-A kerb_map_error_sf seed 11: gnss_cv failed (max error 11.83 m)
- SF-A kerb_map_error_sf seed 11: gnss_hold failed (max error 202.93 m)
- SF-A kerb_map_error_sf seed 12: dr failed (max error 6.27 m)
- SF-A kerb_map_error_sf seed 12: gnss_hold failed (max error 202.85 m)
- SF-A kerb_map_error_sf seed 13: gnss_cv failed (max error 19.55 m)
- SF-A kerb_map_error_sf seed 13: gnss_hold failed (max error 203.34 m)
- SF-A kerb_map_error_sf seed 14: dr failed (max error 10.76 m)
- SF-A kerb_map_error_sf seed 14: gnss_cv failed (max error 10.02 m)
- SF-A kerb_map_error_sf seed 14: gnss_hold failed (max error 201.72 m)
- SF-A kerb_map_error_sf seed 15: gnss_cv failed (max error 14.65 m)
- SF-A kerb_map_error_sf seed 15: gnss_hold failed (max error 202.16 m)
- SF-B nominal_sf seed 1: dr failed (max error 11.53 m)
- SF-B nominal_sf seed 2: dr failed (max error 5.05 m)
- SF-B nominal_sf seed 4: dr failed (max error 7.38 m)
- SF-B nominal_sf seed 5: dr failed (max error 6.34 m)
- SF-B nominal_sf seed 6: dr failed (max error 10.44 m)
- SF-B nominal_sf seed 7: dr failed (max error 7.64 m)
- SF-B nominal_sf seed 8: dr failed (max error 5.59 m)
- SF-B nominal_sf seed 9: dr failed (max error 6.56 m)
- SF-B nominal_sf seed 10: dr failed (max error 7.35 m)
- SF-B nominal_sf seed 12: dr failed (max error 6.73 m)
- SF-B nominal_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_outage_sf seed 1: dr failed (max error 11.53 m)
- SF-B gnss_outage_sf seed 1: gnss_cv failed (max error 11.99 m)
- SF-B gnss_outage_sf seed 1: gnss_hold failed (max error 202.90 m)
- SF-B gnss_outage_sf seed 2: dr failed (max error 5.05 m)
- SF-B gnss_outage_sf seed 2: gnss_cv failed (max error 5.57 m)
- SF-B gnss_outage_sf seed 2: gnss_hold failed (max error 202.98 m)
- SF-B gnss_outage_sf seed 3: gnss_cv failed (max error 9.59 m)
- SF-B gnss_outage_sf seed 3: gnss_hold failed (max error 203.48 m)
- SF-B gnss_outage_sf seed 4: dr failed (max error 7.38 m)
- SF-B gnss_outage_sf seed 4: gnss_cv failed (max error 19.64 m)
- SF-B gnss_outage_sf seed 4: gnss_hold failed (max error 202.00 m)
- SF-B gnss_outage_sf seed 5: dr failed (max error 6.34 m)
- SF-B gnss_outage_sf seed 5: gnss_cv failed (max error 25.04 m)
- SF-B gnss_outage_sf seed 5: gnss_hold failed (max error 203.22 m)
- SF-B gnss_outage_sf seed 6: dr failed (max error 10.44 m)
- SF-B gnss_outage_sf seed 6: gnss_hold failed (max error 202.75 m)
- SF-B gnss_outage_sf seed 7: dr failed (max error 7.64 m)
- SF-B gnss_outage_sf seed 7: gnss_cv failed (max error 21.21 m)
- SF-B gnss_outage_sf seed 7: gnss_hold failed (max error 203.22 m)
- SF-B gnss_outage_sf seed 8: dr failed (max error 5.59 m)
- SF-B gnss_outage_sf seed 8: gnss_hold failed (max error 202.46 m)
- SF-B gnss_outage_sf seed 9: dr failed (max error 6.56 m)
- SF-B gnss_outage_sf seed 9: gnss_cv failed (max error 11.27 m)
- SF-B gnss_outage_sf seed 9: gnss_hold failed (max error 202.41 m)
- SF-B gnss_outage_sf seed 10: dr failed (max error 7.35 m)
- SF-B gnss_outage_sf seed 10: gnss_cv failed (max error 8.60 m)
- SF-B gnss_outage_sf seed 10: gnss_hold failed (max error 202.00 m)
- SF-B gnss_outage_sf seed 11: gnss_cv failed (max error 11.47 m)
- SF-B gnss_outage_sf seed 11: gnss_hold failed (max error 203.02 m)
- SF-B gnss_outage_sf seed 12: dr failed (max error 6.73 m)
- SF-B gnss_outage_sf seed 12: gnss_cv failed (max error 5.01 m)
- SF-B gnss_outage_sf seed 12: gnss_hold failed (max error 202.94 m)
- SF-B gnss_outage_sf seed 13: gnss_cv failed (max error 19.75 m)
- SF-B gnss_outage_sf seed 13: gnss_hold failed (max error 203.54 m)
- SF-B gnss_outage_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_outage_sf seed 14: gnss_cv failed (max error 9.24 m)
- SF-B gnss_outage_sf seed 14: gnss_hold failed (max error 201.77 m)
- SF-B gnss_outage_sf seed 15: gnss_cv failed (max error 13.69 m)
- SF-B gnss_outage_sf seed 15: gnss_hold failed (max error 202.25 m)
- SF-B gnss_drift_sf seed 1: dr failed (max error 11.53 m)
- SF-B gnss_drift_sf seed 2: dr failed (max error 5.05 m)
- SF-B gnss_drift_sf seed 2: gnss_hold failed (max error 6.22 m)
- SF-B gnss_drift_sf seed 3: gnss_hold failed (max error 6.36 m)
- SF-B gnss_drift_sf seed 4: dr failed (max error 7.38 m)
- SF-B gnss_drift_sf seed 5: dr failed (max error 6.34 m)
- SF-B gnss_drift_sf seed 6: dr failed (max error 10.44 m)
- SF-B gnss_drift_sf seed 6: gnss_hold failed (max error 5.53 m)
- SF-B gnss_drift_sf seed 7: dr failed (max error 7.64 m)
- SF-B gnss_drift_sf seed 8: dr failed (max error 5.59 m)
- SF-B gnss_drift_sf seed 9: dr failed (max error 6.56 m)
- SF-B gnss_drift_sf seed 9: gnss_hold failed (max error 5.19 m)
- SF-B gnss_drift_sf seed 10: dr failed (max error 7.35 m)
- SF-B gnss_drift_sf seed 12: dr failed (max error 6.73 m)
- SF-B gnss_drift_sf seed 12: gnss_hold failed (max error 6.38 m)
- SF-B gnss_drift_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_drift_sf seed 14: gnss_hold failed (max error 5.69 m)
- SF-B gnss_drift_sf seed 15: gnss_hold failed (max error 6.36 m)
- SF-B kerb_map_error_sf seed 1: dr failed (max error 11.53 m)
- SF-B kerb_map_error_sf seed 1: gnss_cv failed (max error 11.99 m)
- SF-B kerb_map_error_sf seed 1: gnss_hold failed (max error 202.90 m)
- SF-B kerb_map_error_sf seed 2: dr failed (max error 5.05 m)
- SF-B kerb_map_error_sf seed 2: gnss_cv failed (max error 5.57 m)
- SF-B kerb_map_error_sf seed 2: gnss_hold failed (max error 202.98 m)
- SF-B kerb_map_error_sf seed 3: gnss_cv failed (max error 9.59 m)
- SF-B kerb_map_error_sf seed 3: gnss_hold failed (max error 203.48 m)
- SF-B kerb_map_error_sf seed 4: dr failed (max error 7.38 m)
- SF-B kerb_map_error_sf seed 4: gnss_cv failed (max error 19.64 m)
- SF-B kerb_map_error_sf seed 4: gnss_hold failed (max error 202.00 m)
- SF-B kerb_map_error_sf seed 5: dr failed (max error 6.34 m)
- SF-B kerb_map_error_sf seed 5: gnss_cv failed (max error 25.04 m)
- SF-B kerb_map_error_sf seed 5: gnss_hold failed (max error 203.22 m)
- SF-B kerb_map_error_sf seed 6: dr failed (max error 10.44 m)
- SF-B kerb_map_error_sf seed 6: gnss_hold failed (max error 202.75 m)
- SF-B kerb_map_error_sf seed 7: dr failed (max error 7.64 m)
- SF-B kerb_map_error_sf seed 7: gnss_cv failed (max error 21.21 m)
- SF-B kerb_map_error_sf seed 7: gnss_hold failed (max error 203.22 m)
- SF-B kerb_map_error_sf seed 8: dr failed (max error 5.59 m)
- SF-B kerb_map_error_sf seed 8: gnss_hold failed (max error 202.46 m)
- SF-B kerb_map_error_sf seed 9: dr failed (max error 6.56 m)
- SF-B kerb_map_error_sf seed 9: gnss_cv failed (max error 11.27 m)
- SF-B kerb_map_error_sf seed 9: gnss_hold failed (max error 202.41 m)
- SF-B kerb_map_error_sf seed 10: dr failed (max error 7.35 m)
- SF-B kerb_map_error_sf seed 10: gnss_cv failed (max error 8.60 m)
- SF-B kerb_map_error_sf seed 10: gnss_hold failed (max error 202.00 m)
- SF-B kerb_map_error_sf seed 11: gnss_cv failed (max error 11.47 m)
- SF-B kerb_map_error_sf seed 11: gnss_hold failed (max error 203.02 m)
- SF-B kerb_map_error_sf seed 12: dr failed (max error 6.73 m)
- SF-B kerb_map_error_sf seed 12: gnss_cv failed (max error 5.01 m)
- SF-B kerb_map_error_sf seed 12: gnss_hold failed (max error 202.94 m)
- SF-B kerb_map_error_sf seed 13: gnss_cv failed (max error 19.75 m)
- SF-B kerb_map_error_sf seed 13: gnss_hold failed (max error 203.54 m)
- SF-B kerb_map_error_sf seed 14: dr failed (max error 10.94 m)
- SF-B kerb_map_error_sf seed 14: gnss_cv failed (max error 9.24 m)
- SF-B kerb_map_error_sf seed 14: gnss_hold failed (max error 201.77 m)
- SF-B kerb_map_error_sf seed 15: gnss_cv failed (max error 13.69 m)
- SF-B kerb_map_error_sf seed 15: gnss_hold failed (max error 202.25 m)

Recovery 999 means the error did not stay below 0.5 m for 1 s before the drive ended. Latency and runtime are wall-clock times on the machine that ran the campaign and are not deterministic.
