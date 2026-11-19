# Localisation benchmark: vehicle transfer: San Francisco (SoMa) with the I-PACE proxy

Open loop: each drive is recorded once with ground-truth driving; every estimator then processes the same sensor log in the same arrival order and is scored against the same truth in the fixed map frame, with no per-trajectory alignment. Errors are rear-axle positions sampled at 50 Hz from each estimator's initialisation; lateral error is the part across the true heading. A run fails when the position error ever exceeds 5.0 m after initialising (failed runs stay in every statistic).

Drives: 120 (2 routes x 4 conditions x 15 seeds); drives not completing their route: 0. Toolchain: MSVC 195136260. Engine commit at configure: 5f2c004f82d02e0c9de5aae3f6b1b5dcc1747330.

Vehicle: ipace_proxy_v1 (kinematic bicycle (approximate I-PACE dimensions)); seeds 1 to 15. The extended metrics of every run are in runs.jsonl.

## nominal_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.64 (3) | 0.94 | 1.55 | 24 | - | - | - | - | - | - | 7.3 |
| ekf_int | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.64 (3) | 0.94 | 1.55 | 24 | 0 | 0 | - | - | - | - | 7.6 |
| ekf_int2 | 30 | 0 | 0.184 | 0.197 | 0.347 | 0.78 | 0.294 | 0.35 | - | - | 1.64 (3) | 0.94 | 1.55 | 24 | 0 | 0 | - | 0 | 0 | - | 7.5 |
| ekf_kerb | 30 | 0 | 0.090 | 0.109 | 0.231 | 0.66 | 0.044 | 0.09 | - | - | 1.27 (3) | 0.89 | 1.57 | 25 | 0 | 0 | 100.0% | - | - | - | 114.8 |
| ekf_kerb2 | 30 | 0 | 0.090 | 0.109 | 0.231 | 0.66 | 0.044 | 0.09 | - | - | 1.27 (3) | 0.89 | 1.57 | 25 | 0 | 0 | 100.0% | 0 | 0 | 0 | 116.1 |
| dr | 30 | 21 | 4.590 | 5.072 | 9.393 | 11.86 | 6.621 | 1.09 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.521 | 0.523 | 0.692 | 3.39 | 0.502 | 20.79 | - | - | 3.28 (2) | 0.92 | 1.38 | 8 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 0 | 1.304 | 1.299 | 1.970 | 4.20 | 1.125 | 22.78 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 0 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## gnss_outage_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | - | - | - | - | - | - | 7.2 |
| ekf_int | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | - | - | - | 7.4 |
| ekf_int2 | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | 0 | 0 | - | 7.4 |
| ekf_kerb | 30 | 0 | 0.161 | 0.195 | 0.469 | 0.86 | 0.044 | 0.09 | 0.03 | 0.0 | 1.39 (3) | 0.90 | 1.57 | 14 | 0 | 0 | 100.0% | - | - | - | 118.6 |
| ekf_kerb2 | 30 | 0 | 0.161 | 0.195 | 0.469 | 0.86 | 0.044 | 0.09 | 0.03 | 0.0 | 1.39 (3) | 0.90 | 1.57 | 14 | 0 | 0 | 100.0% | 0 | 0 | 0 | 120.3 |
| dr | 30 | 21 | 4.590 | 5.072 | 9.393 | 11.86 | 6.621 | 1.09 | 4.75 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 26 | 3.490 | 4.334 | 10.360 | 25.02 | 5.599 | 20.81 | 9.84 | 0.7 | 2.99 (2) | 0.75 | 1.39 | 4 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 64.944 | 62.920 | 169.227 | 203.52 | 1.081 | 22.90 | 201.90 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 0 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## gnss_drift_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 2.035 | 2.051 | 4.108 | 4.89 | 3.772 | 0.95 | - | - | 151.17 (3) | 0.47 | 2.92 | 587 | - | - | - | - | - | - | 7.2 |
| ekf_int | 30 | 0 | 2.017 | 2.015 | 4.098 | 4.66 | 3.770 | 0.94 | - | - | 149.77 (3) | 0.47 | 2.27 | 353 | 0 | 3 | - | - | - | - | 7.5 |
| ekf_int2 | 30 | 0 | 2.017 | 2.015 | 4.098 | 4.66 | 3.770 | 0.94 | - | - | 149.77 (3) | 0.47 | 2.27 | 353 | 0 | 3 | - | 0 | 0 | - | 7.5 |
| ekf_kerb | 30 | 0 | 0.425 | 1.006 | 2.594 | 4.12 | 0.696 | 0.11 | - | - | 36.39 (3) | 0.73 | 21.36 | 6256 | 0 | 7 | 94.4% | - | - | - | 117.9 |
| ekf_kerb2 | 30 | 0 | 0.425 | 0.920 | 2.340 | 4.12 | 0.047 | 0.09 | - | - | 26.31 (3) | 0.78 | 22.67 | 7142 | 0 | 4 | 98.5% | 0 | 1 | 16 | 116.5 |
| dr | 30 | 21 | 4.590 | 5.072 | 9.393 | 11.86 | 6.621 | 1.09 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 2.058 | 2.032 | 4.097 | 4.90 | 3.590 | 20.85 | - | - | 49.72 (2) | 0.54 | 1.38 | 8 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 17 | 2.295 | 2.287 | 4.618 | 6.45 | 3.638 | 22.80 | - | - | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 2 of 30 runs; ekf_kerb2 vs ekf_kerb in 13 of 30 runs; ekf_int2 vs ekf in 3 of 30 runs; ekf_kerb2 vs ekf_int2 in 30 of 30 runs.

## kerb_map_error_sf

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | Fault recoveries | Heading re-anchors | Kerb re-localisations | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | - | - | - | - | - | - | 7.1 |
| ekf_int | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | - | - | - | 7.4 |
| ekf_int2 | 30 | 0 | 0.424 | 0.439 | 1.029 | 2.21 | 0.955 | 0.40 | 1.21 | 0.2 | 1.72 (3) | 0.95 | 1.55 | 16 | 0 | 0 | - | 0 | 0 | - | 7.3 |
| ekf_kerb | 30 | 0 | 0.689 | 0.736 | 1.777 | 2.68 | 0.623 | 0.14 | 1.78 | 3.1 | 55.20 (3) | 0.62 | 2.21 | 459 | 0 | 13 | 84.6% | - | - | - | 122.3 |
| ekf_kerb2 | 30 | 0 | 0.668 | 0.695 | 1.653 | 2.85 | 0.613 | 0.09 | 1.59 | 2.0 | 52.94 (3) | 0.68 | 2.00 | 433 | 0 | 9 | 98.1% | 0 | 0 | 69 | 123.4 |
| dr | 30 | 21 | 4.590 | 5.072 | 9.393 | 11.86 | 6.621 | 1.09 | 4.75 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |
| gnss_cv | 30 | 26 | 3.490 | 4.334 | 10.360 | 25.02 | 5.599 | 20.81 | 9.84 | 0.7 | 2.99 (2) | 0.75 | 1.39 | 4 | - | - | - | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 64.944 | 62.920 | 169.227 | 203.52 | 1.081 | 22.90 | 201.90 | 999.0 | - | - | - | 0 | - | - | - | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int2 vs ekf_int in 0 of 30 runs; ekf_kerb2 vs ekf_kerb in 23 of 30 runs; ekf_int2 vs ekf in 0 of 30 runs; ekf_kerb2 vs ekf_int2 in 2 of 30 runs.

## Failed runs

- SF-A nominal_sf seed 1: dr failed (max error 11.86 m)
- SF-A nominal_sf seed 4: dr failed (max error 7.96 m)
- SF-A nominal_sf seed 5: dr failed (max error 6.58 m)
- SF-A nominal_sf seed 6: dr failed (max error 10.71 m)
- SF-A nominal_sf seed 7: dr failed (max error 8.01 m)
- SF-A nominal_sf seed 8: dr failed (max error 5.06 m)
- SF-A nominal_sf seed 9: dr failed (max error 6.52 m)
- SF-A nominal_sf seed 10: dr failed (max error 7.21 m)
- SF-A nominal_sf seed 12: dr failed (max error 6.26 m)
- SF-A nominal_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_outage_sf seed 1: dr failed (max error 11.86 m)
- SF-A gnss_outage_sf seed 1: gnss_cv failed (max error 11.84 m)
- SF-A gnss_outage_sf seed 1: gnss_hold failed (max error 202.68 m)
- SF-A gnss_outage_sf seed 2: gnss_cv failed (max error 6.20 m)
- SF-A gnss_outage_sf seed 2: gnss_hold failed (max error 202.88 m)
- SF-A gnss_outage_sf seed 3: gnss_cv failed (max error 9.49 m)
- SF-A gnss_outage_sf seed 3: gnss_hold failed (max error 203.50 m)
- SF-A gnss_outage_sf seed 4: dr failed (max error 7.96 m)
- SF-A gnss_outage_sf seed 4: gnss_cv failed (max error 19.45 m)
- SF-A gnss_outage_sf seed 4: gnss_hold failed (max error 201.83 m)
- SF-A gnss_outage_sf seed 5: dr failed (max error 6.58 m)
- SF-A gnss_outage_sf seed 5: gnss_cv failed (max error 24.80 m)
- SF-A gnss_outage_sf seed 5: gnss_hold failed (max error 203.12 m)
- SF-A gnss_outage_sf seed 6: dr failed (max error 10.71 m)
- SF-A gnss_outage_sf seed 6: gnss_cv failed (max error 5.31 m)
- SF-A gnss_outage_sf seed 6: gnss_hold failed (max error 202.65 m)
- SF-A gnss_outage_sf seed 7: dr failed (max error 8.01 m)
- SF-A gnss_outage_sf seed 7: gnss_cv failed (max error 21.00 m)
- SF-A gnss_outage_sf seed 7: gnss_hold failed (max error 203.13 m)
- SF-A gnss_outage_sf seed 8: dr failed (max error 5.06 m)
- SF-A gnss_outage_sf seed 8: gnss_hold failed (max error 202.34 m)
- SF-A gnss_outage_sf seed 9: dr failed (max error 6.52 m)
- SF-A gnss_outage_sf seed 9: gnss_cv failed (max error 11.15 m)
- SF-A gnss_outage_sf seed 9: gnss_hold failed (max error 202.45 m)
- SF-A gnss_outage_sf seed 10: dr failed (max error 7.21 m)
- SF-A gnss_outage_sf seed 10: gnss_cv failed (max error 8.62 m)
- SF-A gnss_outage_sf seed 10: gnss_hold failed (max error 201.92 m)
- SF-A gnss_outage_sf seed 11: gnss_cv failed (max error 11.82 m)
- SF-A gnss_outage_sf seed 11: gnss_hold failed (max error 202.93 m)
- SF-A gnss_outage_sf seed 12: dr failed (max error 6.26 m)
- SF-A gnss_outage_sf seed 12: gnss_hold failed (max error 202.85 m)
- SF-A gnss_outage_sf seed 13: gnss_cv failed (max error 19.55 m)
- SF-A gnss_outage_sf seed 13: gnss_hold failed (max error 203.32 m)
- SF-A gnss_outage_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_outage_sf seed 14: gnss_cv failed (max error 10.02 m)
- SF-A gnss_outage_sf seed 14: gnss_hold failed (max error 201.72 m)
- SF-A gnss_outage_sf seed 15: gnss_cv failed (max error 14.65 m)
- SF-A gnss_outage_sf seed 15: gnss_hold failed (max error 202.15 m)
- SF-A gnss_drift_sf seed 1: dr failed (max error 11.86 m)
- SF-A gnss_drift_sf seed 2: gnss_hold failed (max error 6.22 m)
- SF-A gnss_drift_sf seed 3: gnss_hold failed (max error 6.38 m)
- SF-A gnss_drift_sf seed 4: dr failed (max error 7.96 m)
- SF-A gnss_drift_sf seed 5: dr failed (max error 6.58 m)
- SF-A gnss_drift_sf seed 5: gnss_hold failed (max error 5.13 m)
- SF-A gnss_drift_sf seed 6: dr failed (max error 10.71 m)
- SF-A gnss_drift_sf seed 6: gnss_hold failed (max error 5.80 m)
- SF-A gnss_drift_sf seed 7: dr failed (max error 8.01 m)
- SF-A gnss_drift_sf seed 7: gnss_hold failed (max error 5.06 m)
- SF-A gnss_drift_sf seed 8: dr failed (max error 5.06 m)
- SF-A gnss_drift_sf seed 9: dr failed (max error 6.52 m)
- SF-A gnss_drift_sf seed 9: gnss_hold failed (max error 5.42 m)
- SF-A gnss_drift_sf seed 10: dr failed (max error 7.21 m)
- SF-A gnss_drift_sf seed 12: dr failed (max error 6.26 m)
- SF-A gnss_drift_sf seed 12: gnss_hold failed (max error 6.45 m)
- SF-A gnss_drift_sf seed 14: dr failed (max error 10.76 m)
- SF-A gnss_drift_sf seed 14: gnss_hold failed (max error 5.71 m)
- SF-A gnss_drift_sf seed 15: gnss_hold failed (max error 6.38 m)
- SF-A kerb_map_error_sf seed 1: dr failed (max error 11.86 m)
- SF-A kerb_map_error_sf seed 1: gnss_cv failed (max error 11.84 m)
- SF-A kerb_map_error_sf seed 1: gnss_hold failed (max error 202.68 m)
- SF-A kerb_map_error_sf seed 2: gnss_cv failed (max error 6.20 m)
- SF-A kerb_map_error_sf seed 2: gnss_hold failed (max error 202.88 m)
- SF-A kerb_map_error_sf seed 3: gnss_cv failed (max error 9.49 m)
- SF-A kerb_map_error_sf seed 3: gnss_hold failed (max error 203.50 m)
- SF-A kerb_map_error_sf seed 4: dr failed (max error 7.96 m)
- SF-A kerb_map_error_sf seed 4: gnss_cv failed (max error 19.45 m)
- SF-A kerb_map_error_sf seed 4: gnss_hold failed (max error 201.83 m)
- SF-A kerb_map_error_sf seed 5: dr failed (max error 6.58 m)
- SF-A kerb_map_error_sf seed 5: gnss_cv failed (max error 24.80 m)
- SF-A kerb_map_error_sf seed 5: gnss_hold failed (max error 203.12 m)
- SF-A kerb_map_error_sf seed 6: dr failed (max error 10.71 m)
- SF-A kerb_map_error_sf seed 6: gnss_cv failed (max error 5.31 m)
- SF-A kerb_map_error_sf seed 6: gnss_hold failed (max error 202.65 m)
- SF-A kerb_map_error_sf seed 7: dr failed (max error 8.01 m)
- SF-A kerb_map_error_sf seed 7: gnss_cv failed (max error 21.00 m)
- SF-A kerb_map_error_sf seed 7: gnss_hold failed (max error 203.13 m)
- SF-A kerb_map_error_sf seed 8: dr failed (max error 5.06 m)
- SF-A kerb_map_error_sf seed 8: gnss_hold failed (max error 202.34 m)
- SF-A kerb_map_error_sf seed 9: dr failed (max error 6.52 m)
- SF-A kerb_map_error_sf seed 9: gnss_cv failed (max error 11.15 m)
- SF-A kerb_map_error_sf seed 9: gnss_hold failed (max error 202.45 m)
- SF-A kerb_map_error_sf seed 10: dr failed (max error 7.21 m)
- SF-A kerb_map_error_sf seed 10: gnss_cv failed (max error 8.62 m)
- SF-A kerb_map_error_sf seed 10: gnss_hold failed (max error 201.92 m)
- SF-A kerb_map_error_sf seed 11: gnss_cv failed (max error 11.82 m)
- SF-A kerb_map_error_sf seed 11: gnss_hold failed (max error 202.93 m)
- SF-A kerb_map_error_sf seed 12: dr failed (max error 6.26 m)
- SF-A kerb_map_error_sf seed 12: gnss_hold failed (max error 202.85 m)
- SF-A kerb_map_error_sf seed 13: gnss_cv failed (max error 19.55 m)
- SF-A kerb_map_error_sf seed 13: gnss_hold failed (max error 203.32 m)
- SF-A kerb_map_error_sf seed 14: dr failed (max error 10.76 m)
- SF-A kerb_map_error_sf seed 14: gnss_cv failed (max error 10.02 m)
- SF-A kerb_map_error_sf seed 14: gnss_hold failed (max error 201.72 m)
- SF-A kerb_map_error_sf seed 15: gnss_cv failed (max error 14.65 m)
- SF-A kerb_map_error_sf seed 15: gnss_hold failed (max error 202.15 m)
- SF-B nominal_sf seed 1: dr failed (max error 11.53 m)
- SF-B nominal_sf seed 2: dr failed (max error 5.05 m)
- SF-B nominal_sf seed 4: dr failed (max error 7.39 m)
- SF-B nominal_sf seed 5: dr failed (max error 6.34 m)
- SF-B nominal_sf seed 6: dr failed (max error 10.43 m)
- SF-B nominal_sf seed 7: dr failed (max error 7.62 m)
- SF-B nominal_sf seed 8: dr failed (max error 5.59 m)
- SF-B nominal_sf seed 9: dr failed (max error 6.55 m)
- SF-B nominal_sf seed 10: dr failed (max error 7.34 m)
- SF-B nominal_sf seed 12: dr failed (max error 6.71 m)
- SF-B nominal_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_outage_sf seed 1: dr failed (max error 11.53 m)
- SF-B gnss_outage_sf seed 1: gnss_cv failed (max error 11.98 m)
- SF-B gnss_outage_sf seed 1: gnss_hold failed (max error 202.89 m)
- SF-B gnss_outage_sf seed 2: dr failed (max error 5.05 m)
- SF-B gnss_outage_sf seed 2: gnss_cv failed (max error 5.56 m)
- SF-B gnss_outage_sf seed 2: gnss_hold failed (max error 202.97 m)
- SF-B gnss_outage_sf seed 3: gnss_cv failed (max error 9.59 m)
- SF-B gnss_outage_sf seed 3: gnss_hold failed (max error 203.48 m)
- SF-B gnss_outage_sf seed 4: dr failed (max error 7.39 m)
- SF-B gnss_outage_sf seed 4: gnss_cv failed (max error 19.62 m)
- SF-B gnss_outage_sf seed 4: gnss_hold failed (max error 202.00 m)
- SF-B gnss_outage_sf seed 5: dr failed (max error 6.34 m)
- SF-B gnss_outage_sf seed 5: gnss_cv failed (max error 25.02 m)
- SF-B gnss_outage_sf seed 5: gnss_hold failed (max error 203.21 m)
- SF-B gnss_outage_sf seed 6: dr failed (max error 10.43 m)
- SF-B gnss_outage_sf seed 6: gnss_hold failed (max error 202.74 m)
- SF-B gnss_outage_sf seed 7: dr failed (max error 7.62 m)
- SF-B gnss_outage_sf seed 7: gnss_cv failed (max error 21.20 m)
- SF-B gnss_outage_sf seed 7: gnss_hold failed (max error 203.22 m)
- SF-B gnss_outage_sf seed 8: dr failed (max error 5.59 m)
- SF-B gnss_outage_sf seed 8: gnss_hold failed (max error 202.44 m)
- SF-B gnss_outage_sf seed 9: dr failed (max error 6.55 m)
- SF-B gnss_outage_sf seed 9: gnss_cv failed (max error 11.27 m)
- SF-B gnss_outage_sf seed 9: gnss_hold failed (max error 202.40 m)
- SF-B gnss_outage_sf seed 10: dr failed (max error 7.34 m)
- SF-B gnss_outage_sf seed 10: gnss_cv failed (max error 8.59 m)
- SF-B gnss_outage_sf seed 10: gnss_hold failed (max error 201.99 m)
- SF-B gnss_outage_sf seed 11: gnss_cv failed (max error 11.45 m)
- SF-B gnss_outage_sf seed 11: gnss_hold failed (max error 203.02 m)
- SF-B gnss_outage_sf seed 12: dr failed (max error 6.71 m)
- SF-B gnss_outage_sf seed 12: gnss_cv failed (max error 5.01 m)
- SF-B gnss_outage_sf seed 12: gnss_hold failed (max error 202.94 m)
- SF-B gnss_outage_sf seed 13: gnss_cv failed (max error 19.74 m)
- SF-B gnss_outage_sf seed 13: gnss_hold failed (max error 203.52 m)
- SF-B gnss_outage_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_outage_sf seed 14: gnss_cv failed (max error 9.24 m)
- SF-B gnss_outage_sf seed 14: gnss_hold failed (max error 201.77 m)
- SF-B gnss_outage_sf seed 15: gnss_cv failed (max error 13.68 m)
- SF-B gnss_outage_sf seed 15: gnss_hold failed (max error 202.23 m)
- SF-B gnss_drift_sf seed 1: dr failed (max error 11.53 m)
- SF-B gnss_drift_sf seed 2: dr failed (max error 5.05 m)
- SF-B gnss_drift_sf seed 2: gnss_hold failed (max error 6.22 m)
- SF-B gnss_drift_sf seed 3: gnss_hold failed (max error 6.38 m)
- SF-B gnss_drift_sf seed 4: dr failed (max error 7.39 m)
- SF-B gnss_drift_sf seed 5: dr failed (max error 6.34 m)
- SF-B gnss_drift_sf seed 6: dr failed (max error 10.43 m)
- SF-B gnss_drift_sf seed 6: gnss_hold failed (max error 5.61 m)
- SF-B gnss_drift_sf seed 7: dr failed (max error 7.62 m)
- SF-B gnss_drift_sf seed 8: dr failed (max error 5.59 m)
- SF-B gnss_drift_sf seed 8: gnss_hold failed (max error 5.20 m)
- SF-B gnss_drift_sf seed 9: dr failed (max error 6.55 m)
- SF-B gnss_drift_sf seed 9: gnss_hold failed (max error 5.34 m)
- SF-B gnss_drift_sf seed 10: dr failed (max error 7.34 m)
- SF-B gnss_drift_sf seed 12: dr failed (max error 6.71 m)
- SF-B gnss_drift_sf seed 12: gnss_hold failed (max error 6.44 m)
- SF-B gnss_drift_sf seed 14: dr failed (max error 10.94 m)
- SF-B gnss_drift_sf seed 14: gnss_hold failed (max error 5.71 m)
- SF-B gnss_drift_sf seed 15: gnss_hold failed (max error 6.38 m)
- SF-B kerb_map_error_sf seed 1: dr failed (max error 11.53 m)
- SF-B kerb_map_error_sf seed 1: gnss_cv failed (max error 11.98 m)
- SF-B kerb_map_error_sf seed 1: gnss_hold failed (max error 202.89 m)
- SF-B kerb_map_error_sf seed 2: dr failed (max error 5.05 m)
- SF-B kerb_map_error_sf seed 2: gnss_cv failed (max error 5.56 m)
- SF-B kerb_map_error_sf seed 2: gnss_hold failed (max error 202.97 m)
- SF-B kerb_map_error_sf seed 3: gnss_cv failed (max error 9.59 m)
- SF-B kerb_map_error_sf seed 3: gnss_hold failed (max error 203.48 m)
- SF-B kerb_map_error_sf seed 4: dr failed (max error 7.39 m)
- SF-B kerb_map_error_sf seed 4: gnss_cv failed (max error 19.62 m)
- SF-B kerb_map_error_sf seed 4: gnss_hold failed (max error 202.00 m)
- SF-B kerb_map_error_sf seed 5: dr failed (max error 6.34 m)
- SF-B kerb_map_error_sf seed 5: gnss_cv failed (max error 25.02 m)
- SF-B kerb_map_error_sf seed 5: gnss_hold failed (max error 203.21 m)
- SF-B kerb_map_error_sf seed 6: dr failed (max error 10.43 m)
- SF-B kerb_map_error_sf seed 6: gnss_hold failed (max error 202.74 m)
- SF-B kerb_map_error_sf seed 7: dr failed (max error 7.62 m)
- SF-B kerb_map_error_sf seed 7: gnss_cv failed (max error 21.20 m)
- SF-B kerb_map_error_sf seed 7: gnss_hold failed (max error 203.22 m)
- SF-B kerb_map_error_sf seed 8: dr failed (max error 5.59 m)
- SF-B kerb_map_error_sf seed 8: gnss_hold failed (max error 202.44 m)
- SF-B kerb_map_error_sf seed 9: dr failed (max error 6.55 m)
- SF-B kerb_map_error_sf seed 9: gnss_cv failed (max error 11.27 m)
- SF-B kerb_map_error_sf seed 9: gnss_hold failed (max error 202.40 m)
- SF-B kerb_map_error_sf seed 10: dr failed (max error 7.34 m)
- SF-B kerb_map_error_sf seed 10: gnss_cv failed (max error 8.59 m)
- SF-B kerb_map_error_sf seed 10: gnss_hold failed (max error 201.99 m)
- SF-B kerb_map_error_sf seed 11: gnss_cv failed (max error 11.45 m)
- SF-B kerb_map_error_sf seed 11: gnss_hold failed (max error 203.02 m)
- SF-B kerb_map_error_sf seed 12: dr failed (max error 6.71 m)
- SF-B kerb_map_error_sf seed 12: gnss_cv failed (max error 5.01 m)
- SF-B kerb_map_error_sf seed 12: gnss_hold failed (max error 202.94 m)
- SF-B kerb_map_error_sf seed 13: gnss_cv failed (max error 19.74 m)
- SF-B kerb_map_error_sf seed 13: gnss_hold failed (max error 203.52 m)
- SF-B kerb_map_error_sf seed 14: dr failed (max error 10.94 m)
- SF-B kerb_map_error_sf seed 14: gnss_cv failed (max error 9.24 m)
- SF-B kerb_map_error_sf seed 14: gnss_hold failed (max error 201.77 m)
- SF-B kerb_map_error_sf seed 15: gnss_cv failed (max error 13.68 m)
- SF-B kerb_map_error_sf seed 15: gnss_hold failed (max error 202.23 m)

Recovery 999 means the error did not stay below 0.5 m for 1 s before the drive ended. Latency and runtime are wall-clock times on the machine that ran the campaign and are not deterministic.
