# Localisation benchmark: development and regression data for the integrity and kerb increment (network R2)

Open loop: each drive is recorded once with ground-truth driving; every estimator then processes the same sensor log in the same arrival order and is scored against the same truth in the fixed map frame, with no per-trajectory alignment. Errors are rear-axle positions sampled at 50 Hz from each estimator's initialisation; lateral error is the part across the true heading. A run fails when the position error ever exceeds 5.0 m after initialising (failed runs stay in every statistic).

Drives: 210 (3 routes x 7 conditions x 10 seeds); drives not completing their route: 0. Toolchain: GCC 14.2.0. Engine commit at configure: 7781fe2351ef59ec5fdc3f39add02e95c2e50290.

## nominal_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.179 | 0.205 | 0.355 | 0.66 | 0.265 | 0.46 | - | - | 1.78 (3) | 0.95 | 1.57 | 24 | - | - | - | 7.8 |
| ekf_int | 30 | 0 | 0.179 | 0.205 | 0.355 | 0.66 | 0.265 | 0.46 | - | - | 1.78 (3) | 0.95 | 1.57 | 24 | 0 | 0 | - | 8.6 |
| ekf_kerb | 30 | 0 | 0.080 | 0.091 | 0.180 | 0.66 | 0.044 | 0.09 | - | - | 1.28 (3) | 0.88 | 1.59 | 20 | 0 | 0 | 100.0% | 385.5 |
| dr | 30 | 16 | 4.230 | 5.124 | 10.502 | 13.56 | 7.440 | 1.24 | - | - | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.623 | 0.626 | 1.234 | 3.49 | 0.666 | 26.34 | - | - | 4.92 (2) | 0.90 | 1.40 | 5 | - | - | - | 0.1 |
| gnss_hold | 30 | 0 | 1.247 | 1.246 | 1.975 | 3.86 | 1.210 | 24.96 | - | - | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 0 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## gnss_outage_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.217 | 0.251 | 0.452 | 1.03 | 0.320 | 0.46 | 0.40 | 0.0 | 1.63 (3) | 0.94 | 1.58 | 15 | - | - | - | 7.6 |
| ekf_int | 30 | 0 | 0.217 | 0.251 | 0.452 | 1.03 | 0.320 | 0.46 | 0.40 | 0.0 | 1.63 (3) | 0.94 | 1.58 | 15 | 0 | 0 | - | 8.5 |
| ekf_kerb | 30 | 0 | 0.081 | 0.093 | 0.182 | 0.66 | 0.044 | 0.09 | 0.04 | 0.0 | 1.25 (3) | 0.88 | 1.59 | 15 | 0 | 0 | 100.0% | 380.5 |
| dr | 30 | 16 | 4.230 | 5.124 | 10.502 | 13.56 | 7.440 | 1.24 | 5.57 | 999.0 | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 30 | 7.123 | 9.227 | 18.727 | 77.06 | 12.367 | 33.47 | 35.10 | 0.7 | 4.64 (2) | 0.82 | 1.41 | 3 | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 43.517 | 41.393 | 113.460 | 127.94 | 47.077 | 30.37 | 119.71 | 999.0 | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 0 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## multipath_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 8 | 0.285 | 1.721 | 5.645 | 7.97 | 4.218 | 0.46 | - | - | 39.90 (3) | 0.87 | 67.76 | 3920 | - | - | - | 7.0 |
| ekf_int | 30 | 0 | 0.202 | 0.223 | 0.389 | 0.66 | 0.291 | 0.46 | - | - | 1.71 (3) | 0.93 | 59.58 | 3144 | 2208 | 0 | - | 7.9 |
| ekf_kerb | 30 | 0 | 0.081 | 0.093 | 0.184 | 0.66 | 0.044 | 0.09 | - | - | 1.26 (3) | 0.88 | 86.00 | 3144 | 2208 | 0 | 100.0% | 360.7 |
| dr | 30 | 16 | 4.230 | 5.124 | 10.502 | 13.56 | 7.440 | 1.24 | - | - | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 30 | 3.631 | 10.193 | 11.357 | 142.25 | 7.328 | 28.36 | - | - | 30.85 (2) | 0.69 | 9.72 | 3093 | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 2.979 | 2.937 | 6.892 | 10.46 | 5.114 | 46.22 | - | - | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 19 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## wheel_slip_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.180 | 0.205 | 0.356 | 0.66 | 0.266 | 0.46 | - | - | 1.78 (3) | 0.95 | 1.57 | 23 | - | - | - | 7.1 |
| ekf_int | 30 | 0 | 0.180 | 0.205 | 0.356 | 0.66 | 0.266 | 0.46 | - | - | 1.78 (3) | 0.95 | 1.57 | 23 | 0 | 0 | - | 7.9 |
| ekf_kerb | 30 | 0 | 0.080 | 0.092 | 0.181 | 0.66 | 0.044 | 0.09 | - | - | 1.28 (3) | 0.88 | 1.59 | 20 | 0 | 0 | 100.0% | 361.7 |
| dr | 30 | 30 | 19.617 | 19.731 | 33.614 | 38.38 | 31.560 | 1.24 | - | - | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.623 | 0.626 | 1.234 | 3.49 | 0.666 | 26.34 | - | - | 4.92 (2) | 0.90 | 1.40 | 5 | - | - | - | 0.1 |
| gnss_hold | 30 | 0 | 1.247 | 1.246 | 1.975 | 3.86 | 1.210 | 24.96 | - | - | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 0 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## imu_bias_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.227 | 0.283 | 0.470 | 2.45 | 0.449 | 1.30 | - | - | 18.95 (3) | 0.79 | 1.98 | 186 | - | - | - | 7.2 |
| ekf_int | 30 | 0 | 0.227 | 0.260 | 0.463 | 1.92 | 0.442 | 1.30 | - | - | 18.57 (3) | 0.79 | 1.89 | 148 | 0 | 1 | - | 8.1 |
| ekf_kerb | 30 | 0 | 0.079 | 0.092 | 0.181 | 0.66 | 0.044 | 0.28 | - | - | 4.36 (3) | 0.80 | 1.59 | 21 | 0 | 0 | 100.0% | 420.6 |
| dr | 30 | 18 | 4.506 | 5.605 | 11.643 | 17.91 | 8.615 | 2.66 | - | - | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.623 | 0.626 | 1.234 | 3.49 | 0.666 | 26.34 | - | - | 4.92 (2) | 0.90 | 1.40 | 5 | - | - | - | 0.1 |
| gnss_hold | 30 | 0 | 1.247 | 1.246 | 1.975 | 3.86 | 1.210 | 24.96 | - | - | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 1 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## delay_drop_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.200 | 0.217 | 0.381 | 0.56 | 0.288 | 0.45 | - | - | 1.74 (3) | 0.95 | 1.56 | 20 | - | - | - | 35.9 |
| ekf_int | 30 | 0 | 0.200 | 0.217 | 0.381 | 0.56 | 0.288 | 0.45 | - | - | 1.74 (3) | 0.95 | 1.56 | 20 | 0 | 0 | - | 38.1 |
| ekf_kerb | 30 | 0 | 0.084 | 0.097 | 0.203 | 0.55 | 0.044 | 0.09 | - | - | 1.22 (3) | 0.88 | 1.59 | 16 | 0 | 0 | 100.0% | 1379.7 |
| dr | 30 | 16 | 4.414 | 5.094 | 10.533 | 12.29 | 7.123 | 1.16 | - | - | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 0 | 0.799 | 0.808 | 1.509 | 3.52 | 0.823 | 24.76 | - | - | 1.10 (2) | 0.92 | 1.31 | 0 | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 4.284 | 4.381 | 6.908 | 12.15 | 0.938 | 17.38 | - | - | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 0 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## combined_h

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 30 | 0 | 0.204 | 0.241 | 0.425 | 0.78 | 0.317 | 0.43 | 0.31 | 0.0 | 1.64 (3) | 0.95 | 44.42 | 2114 | - | - | - | 19.8 |
| ekf_int | 30 | 0 | 0.208 | 0.242 | 0.427 | 0.78 | 0.318 | 0.43 | 0.31 | 0.0 | 1.64 (3) | 0.95 | 44.41 | 2233 | 1307 | 0 | - | 22.2 |
| ekf_kerb | 30 | 0 | 0.092 | 0.102 | 0.219 | 0.66 | 0.044 | 0.09 | 0.10 | 0.0 | 1.24 (3) | 0.88 | 59.87 | 2232 | 1308 | 0 | 100.0% | 721.2 |
| dr | 30 | 30 | 9.423 | 10.528 | 18.519 | 23.14 | 15.960 | 1.19 | 2.84 | 999.0 | - | - | - | 0 | - | - | - | 0.1 |
| gnss_cv | 30 | 30 | 5.723 | 10.968 | 15.682 | 147.70 | 11.176 | 26.69 | 7.07 | 3.5 | 10.56 (2) | 0.69 | 7.94 | 2027 | - | - | - | 0.1 |
| gnss_hold | 30 | 30 | 21.299 | 21.880 | 60.685 | 102.71 | 5.266 | 44.32 | 87.53 | 999.0 | - | - | - | 0 | - | - | - | 0.1 |

Lower RMSE, paired by drive: ekf_int vs ekf in 11 of 30 runs; ekf_kerb vs ekf in 30 of 30 runs; ekf_kerb vs ekf_int in 30 of 30 runs.

## Failed runs

- R2-A nominal_h seed 1: dr failed (max error 12.06 m)
- R2-A nominal_h seed 4: dr failed (max error 7.84 m)
- R2-A nominal_h seed 5: dr failed (max error 5.95 m)
- R2-A nominal_h seed 6: dr failed (max error 10.67 m)
- R2-A nominal_h seed 7: dr failed (max error 8.50 m)
- R2-A nominal_h seed 8: dr failed (max error 6.11 m)
- R2-A nominal_h seed 9: dr failed (max error 7.26 m)
- R2-A nominal_h seed 10: dr failed (max error 8.83 m)
- R2-A gnss_outage_h seed 1: dr failed (max error 12.06 m)
- R2-A gnss_outage_h seed 1: gnss_cv failed (max error 26.03 m)
- R2-A gnss_outage_h seed 1: gnss_hold failed (max error 127.39 m)
- R2-A gnss_outage_h seed 2: gnss_cv failed (max error 20.10 m)
- R2-A gnss_outage_h seed 2: gnss_hold failed (max error 127.51 m)
- R2-A gnss_outage_h seed 3: gnss_cv failed (max error 18.59 m)
- R2-A gnss_outage_h seed 3: gnss_hold failed (max error 127.43 m)
- R2-A gnss_outage_h seed 4: dr failed (max error 7.84 m)
- R2-A gnss_outage_h seed 4: gnss_cv failed (max error 23.92 m)
- R2-A gnss_outage_h seed 4: gnss_hold failed (max error 127.76 m)
- R2-A gnss_outage_h seed 5: dr failed (max error 5.95 m)
- R2-A gnss_outage_h seed 5: gnss_cv failed (max error 27.85 m)
- R2-A gnss_outage_h seed 5: gnss_hold failed (max error 126.86 m)
- R2-A gnss_outage_h seed 6: dr failed (max error 10.67 m)
- R2-A gnss_outage_h seed 6: gnss_cv failed (max error 27.40 m)
- R2-A gnss_outage_h seed 6: gnss_hold failed (max error 127.94 m)
- R2-A gnss_outage_h seed 7: dr failed (max error 8.50 m)
- R2-A gnss_outage_h seed 7: gnss_cv failed (max error 27.41 m)
- R2-A gnss_outage_h seed 7: gnss_hold failed (max error 126.90 m)
- R2-A gnss_outage_h seed 8: dr failed (max error 6.11 m)
- R2-A gnss_outage_h seed 8: gnss_cv failed (max error 16.63 m)
- R2-A gnss_outage_h seed 8: gnss_hold failed (max error 127.68 m)
- R2-A gnss_outage_h seed 9: dr failed (max error 7.26 m)
- R2-A gnss_outage_h seed 9: gnss_cv failed (max error 28.93 m)
- R2-A gnss_outage_h seed 9: gnss_hold failed (max error 127.67 m)
- R2-A gnss_outage_h seed 10: dr failed (max error 8.83 m)
- R2-A gnss_outage_h seed 10: gnss_cv failed (max error 38.54 m)
- R2-A gnss_outage_h seed 10: gnss_hold failed (max error 127.20 m)
- R2-A multipath_h seed 1: dr failed (max error 12.06 m)
- R2-A multipath_h seed 1: gnss_cv failed (max error 59.50 m)
- R2-A multipath_h seed 1: gnss_hold failed (max error 10.46 m)
- R2-A multipath_h seed 2: gnss_cv failed (max error 10.79 m)
- R2-A multipath_h seed 2: gnss_hold failed (max error 9.32 m)
- R2-A multipath_h seed 3: gnss_cv failed (max error 17.39 m)
- R2-A multipath_h seed 3: gnss_hold failed (max error 9.56 m)
- R2-A multipath_h seed 4: dr failed (max error 7.84 m)
- R2-A multipath_h seed 4: gnss_cv failed (max error 9.07 m)
- R2-A multipath_h seed 4: gnss_hold failed (max error 9.80 m)
- R2-A multipath_h seed 5: dr failed (max error 5.95 m)
- R2-A multipath_h seed 5: gnss_cv failed (max error 10.96 m)
- R2-A multipath_h seed 5: gnss_hold failed (max error 8.53 m)
- R2-A multipath_h seed 6: dr failed (max error 10.67 m)
- R2-A multipath_h seed 6: gnss_cv failed (max error 62.14 m)
- R2-A multipath_h seed 6: gnss_hold failed (max error 8.91 m)
- R2-A multipath_h seed 7: dr failed (max error 8.50 m)
- R2-A multipath_h seed 7: gnss_cv failed (max error 43.92 m)
- R2-A multipath_h seed 7: gnss_hold failed (max error 8.90 m)
- R2-A multipath_h seed 8: dr failed (max error 6.11 m)
- R2-A multipath_h seed 8: gnss_cv failed (max error 10.60 m)
- R2-A multipath_h seed 8: gnss_hold failed (max error 9.79 m)
- R2-A multipath_h seed 9: dr failed (max error 7.26 m)
- R2-A multipath_h seed 9: gnss_cv failed (max error 10.24 m)
- R2-A multipath_h seed 9: gnss_hold failed (max error 9.40 m)
- R2-A multipath_h seed 10: dr failed (max error 8.83 m)
- R2-A multipath_h seed 10: gnss_cv failed (max error 10.08 m)
- R2-A multipath_h seed 10: gnss_hold failed (max error 8.66 m)
- R2-A wheel_slip_h seed 1: dr failed (max error 33.88 m)
- R2-A wheel_slip_h seed 2: dr failed (max error 30.27 m)
- R2-A wheel_slip_h seed 3: dr failed (max error 23.57 m)
- R2-A wheel_slip_h seed 4: dr failed (max error 31.21 m)
- R2-A wheel_slip_h seed 5: dr failed (max error 24.27 m)
- R2-A wheel_slip_h seed 6: dr failed (max error 31.20 m)
- R2-A wheel_slip_h seed 7: dr failed (max error 31.16 m)
- R2-A wheel_slip_h seed 8: dr failed (max error 31.18 m)
- R2-A wheel_slip_h seed 9: dr failed (max error 30.26 m)
- R2-A wheel_slip_h seed 10: dr failed (max error 24.13 m)
- R2-A imu_bias_h seed 1: dr failed (max error 12.43 m)
- R2-A imu_bias_h seed 3: dr failed (max error 8.68 m)
- R2-A imu_bias_h seed 4: dr failed (max error 8.49 m)
- R2-A imu_bias_h seed 5: dr failed (max error 9.48 m)
- R2-A imu_bias_h seed 6: dr failed (max error 11.86 m)
- R2-A imu_bias_h seed 7: dr failed (max error 9.18 m)
- R2-A imu_bias_h seed 8: dr failed (max error 6.44 m)
- R2-A imu_bias_h seed 9: dr failed (max error 8.22 m)
- R2-A imu_bias_h seed 10: dr failed (max error 11.60 m)
- R2-A delay_drop_h seed 1: dr failed (max error 12.29 m)
- R2-A delay_drop_h seed 1: gnss_hold failed (max error 9.83 m)
- R2-A delay_drop_h seed 2: gnss_hold failed (max error 11.16 m)
- R2-A delay_drop_h seed 3: gnss_hold failed (max error 11.72 m)
- R2-A delay_drop_h seed 4: dr failed (max error 7.74 m)
- R2-A delay_drop_h seed 4: gnss_hold failed (max error 11.09 m)
- R2-A delay_drop_h seed 5: dr failed (max error 7.24 m)
- R2-A delay_drop_h seed 5: gnss_hold failed (max error 10.42 m)
- R2-A delay_drop_h seed 6: dr failed (max error 10.71 m)
- R2-A delay_drop_h seed 6: gnss_hold failed (max error 9.85 m)
- R2-A delay_drop_h seed 7: dr failed (max error 7.80 m)
- R2-A delay_drop_h seed 7: gnss_hold failed (max error 10.13 m)
- R2-A delay_drop_h seed 8: dr failed (max error 6.40 m)
- R2-A delay_drop_h seed 8: gnss_hold failed (max error 9.75 m)
- R2-A delay_drop_h seed 9: dr failed (max error 7.43 m)
- R2-A delay_drop_h seed 9: gnss_hold failed (max error 9.76 m)
- R2-A delay_drop_h seed 10: dr failed (max error 8.04 m)
- R2-A delay_drop_h seed 10: gnss_hold failed (max error 9.72 m)
- R2-A combined_h seed 1: dr failed (max error 20.55 m)
- R2-A combined_h seed 1: gnss_cv failed (max error 14.97 m)
- R2-A combined_h seed 1: gnss_hold failed (max error 92.14 m)
- R2-A combined_h seed 2: dr failed (max error 15.92 m)
- R2-A combined_h seed 2: gnss_cv failed (max error 47.04 m)
- R2-A combined_h seed 2: gnss_hold failed (max error 90.39 m)
- R2-A combined_h seed 3: dr failed (max error 9.68 m)
- R2-A combined_h seed 3: gnss_cv failed (max error 33.36 m)
- R2-A combined_h seed 3: gnss_hold failed (max error 93.08 m)
- R2-A combined_h seed 4: dr failed (max error 17.45 m)
- R2-A combined_h seed 4: gnss_cv failed (max error 27.44 m)
- R2-A combined_h seed 4: gnss_hold failed (max error 90.11 m)
- R2-A combined_h seed 5: dr failed (max error 10.84 m)
- R2-A combined_h seed 5: gnss_cv failed (max error 9.55 m)
- R2-A combined_h seed 5: gnss_hold failed (max error 89.50 m)
- R2-A combined_h seed 6: dr failed (max error 18.30 m)
- R2-A combined_h seed 6: gnss_cv failed (max error 8.28 m)
- R2-A combined_h seed 6: gnss_hold failed (max error 91.60 m)
- R2-A combined_h seed 7: dr failed (max error 17.05 m)
- R2-A combined_h seed 7: gnss_cv failed (max error 10.53 m)
- R2-A combined_h seed 7: gnss_hold failed (max error 90.72 m)
- R2-A combined_h seed 8: dr failed (max error 17.41 m)
- R2-A combined_h seed 8: gnss_cv failed (max error 10.83 m)
- R2-A combined_h seed 8: gnss_hold failed (max error 90.12 m)
- R2-A combined_h seed 9: dr failed (max error 16.61 m)
- R2-A combined_h seed 9: gnss_cv failed (max error 41.28 m)
- R2-A combined_h seed 9: gnss_hold failed (max error 93.10 m)
- R2-A combined_h seed 10: dr failed (max error 11.57 m)
- R2-A combined_h seed 10: gnss_cv failed (max error 11.51 m)
- R2-A combined_h seed 10: gnss_hold failed (max error 90.17 m)
- R2-B nominal_h seed 1: dr failed (max error 13.56 m)
- R2-B nominal_h seed 4: dr failed (max error 8.92 m)
- R2-B nominal_h seed 5: dr failed (max error 7.19 m)
- R2-B nominal_h seed 6: dr failed (max error 11.98 m)
- R2-B nominal_h seed 7: dr failed (max error 9.00 m)
- R2-B nominal_h seed 8: dr failed (max error 5.89 m)
- R2-B nominal_h seed 9: dr failed (max error 7.42 m)
- R2-B nominal_h seed 10: dr failed (max error 8.29 m)
- R2-B gnss_outage_h seed 1: dr failed (max error 13.56 m)
- R2-B gnss_outage_h seed 1: gnss_cv failed (max error 63.10 m)
- R2-B gnss_outage_h seed 1: gnss_hold failed (max error 120.03 m)
- R2-B gnss_outage_h seed 2: gnss_cv failed (max error 77.06 m)
- R2-B gnss_outage_h seed 2: gnss_hold failed (max error 119.76 m)
- R2-B gnss_outage_h seed 3: gnss_cv failed (max error 64.72 m)
- R2-B gnss_outage_h seed 3: gnss_hold failed (max error 120.20 m)
- R2-B gnss_outage_h seed 4: dr failed (max error 8.92 m)
- R2-B gnss_outage_h seed 4: gnss_cv failed (max error 70.14 m)
- R2-B gnss_outage_h seed 4: gnss_hold failed (max error 120.10 m)
- R2-B gnss_outage_h seed 5: dr failed (max error 7.19 m)
- R2-B gnss_outage_h seed 5: gnss_cv failed (max error 76.94 m)
- R2-B gnss_outage_h seed 5: gnss_hold failed (max error 119.63 m)
- R2-B gnss_outage_h seed 6: dr failed (max error 11.98 m)
- R2-B gnss_outage_h seed 6: gnss_cv failed (max error 73.68 m)
- R2-B gnss_outage_h seed 6: gnss_hold failed (max error 120.09 m)
- R2-B gnss_outage_h seed 7: dr failed (max error 9.00 m)
- R2-B gnss_outage_h seed 7: gnss_cv failed (max error 72.57 m)
- R2-B gnss_outage_h seed 7: gnss_hold failed (max error 119.76 m)
- R2-B gnss_outage_h seed 8: dr failed (max error 5.89 m)
- R2-B gnss_outage_h seed 8: gnss_cv failed (max error 69.36 m)
- R2-B gnss_outage_h seed 8: gnss_hold failed (max error 119.35 m)
- R2-B gnss_outage_h seed 9: dr failed (max error 7.42 m)
- R2-B gnss_outage_h seed 9: gnss_cv failed (max error 64.15 m)
- R2-B gnss_outage_h seed 9: gnss_hold failed (max error 120.09 m)
- R2-B gnss_outage_h seed 10: dr failed (max error 8.29 m)
- R2-B gnss_outage_h seed 10: gnss_cv failed (max error 58.32 m)
- R2-B gnss_outage_h seed 10: gnss_hold failed (max error 120.07 m)
- R2-B multipath_h seed 1: dr failed (max error 13.56 m)
- R2-B multipath_h seed 1: gnss_cv failed (max error 15.49 m)
- R2-B multipath_h seed 1: gnss_hold failed (max error 9.40 m)
- R2-B multipath_h seed 2: ekf failed (max error 7.20 m)
- R2-B multipath_h seed 2: gnss_cv failed (max error 8.95 m)
- R2-B multipath_h seed 2: gnss_hold failed (max error 9.83 m)
- R2-B multipath_h seed 3: ekf failed (max error 6.75 m)
- R2-B multipath_h seed 3: gnss_cv failed (max error 16.58 m)
- R2-B multipath_h seed 3: gnss_hold failed (max error 9.81 m)
- R2-B multipath_h seed 4: ekf failed (max error 7.07 m)
- R2-B multipath_h seed 4: dr failed (max error 8.92 m)
- R2-B multipath_h seed 4: gnss_cv failed (max error 16.93 m)
- R2-B multipath_h seed 4: gnss_hold failed (max error 10.43 m)
- R2-B multipath_h seed 5: ekf failed (max error 6.10 m)
- R2-B multipath_h seed 5: dr failed (max error 7.19 m)
- R2-B multipath_h seed 5: gnss_cv failed (max error 28.98 m)
- R2-B multipath_h seed 5: gnss_hold failed (max error 9.96 m)
- R2-B multipath_h seed 6: dr failed (max error 11.98 m)
- R2-B multipath_h seed 6: gnss_cv failed (max error 27.85 m)
- R2-B multipath_h seed 6: gnss_hold failed (max error 9.96 m)
- R2-B multipath_h seed 7: ekf failed (max error 7.97 m)
- R2-B multipath_h seed 7: dr failed (max error 9.00 m)
- R2-B multipath_h seed 7: gnss_cv failed (max error 9.81 m)
- R2-B multipath_h seed 7: gnss_hold failed (max error 9.17 m)
- R2-B multipath_h seed 8: ekf failed (max error 6.20 m)
- R2-B multipath_h seed 8: dr failed (max error 5.89 m)
- R2-B multipath_h seed 8: gnss_cv failed (max error 142.25 m)
- R2-B multipath_h seed 8: gnss_hold failed (max error 8.81 m)
- R2-B multipath_h seed 9: ekf failed (max error 6.12 m)
- R2-B multipath_h seed 9: dr failed (max error 7.42 m)
- R2-B multipath_h seed 9: gnss_cv failed (max error 79.24 m)
- R2-B multipath_h seed 9: gnss_hold failed (max error 9.29 m)
- R2-B multipath_h seed 10: ekf failed (max error 6.83 m)
- R2-B multipath_h seed 10: dr failed (max error 8.29 m)
- R2-B multipath_h seed 10: gnss_cv failed (max error 17.88 m)
- R2-B multipath_h seed 10: gnss_hold failed (max error 9.27 m)
- R2-B wheel_slip_h seed 1: dr failed (max error 34.88 m)
- R2-B wheel_slip_h seed 2: dr failed (max error 33.20 m)
- R2-B wheel_slip_h seed 3: dr failed (max error 26.40 m)
- R2-B wheel_slip_h seed 4: dr failed (max error 33.98 m)
- R2-B wheel_slip_h seed 5: dr failed (max error 28.16 m)
- R2-B wheel_slip_h seed 6: dr failed (max error 38.38 m)
- R2-B wheel_slip_h seed 7: dr failed (max error 33.98 m)
- R2-B wheel_slip_h seed 8: dr failed (max error 32.89 m)
- R2-B wheel_slip_h seed 9: dr failed (max error 35.68 m)
- R2-B wheel_slip_h seed 10: dr failed (max error 29.54 m)
- R2-B imu_bias_h seed 1: dr failed (max error 15.02 m)
- R2-B imu_bias_h seed 2: dr failed (max error 9.50 m)
- R2-B imu_bias_h seed 4: dr failed (max error 12.29 m)
- R2-B imu_bias_h seed 5: dr failed (max error 7.87 m)
- R2-B imu_bias_h seed 6: dr failed (max error 17.91 m)
- R2-B imu_bias_h seed 7: dr failed (max error 12.43 m)
- R2-B imu_bias_h seed 8: dr failed (max error 9.80 m)
- R2-B imu_bias_h seed 9: dr failed (max error 14.05 m)
- R2-B imu_bias_h seed 10: dr failed (max error 9.61 m)
- R2-B delay_drop_h seed 1: dr failed (max error 12.10 m)
- R2-B delay_drop_h seed 1: gnss_hold failed (max error 9.90 m)
- R2-B delay_drop_h seed 2: gnss_hold failed (max error 11.60 m)
- R2-B delay_drop_h seed 3: gnss_hold failed (max error 12.15 m)
- R2-B delay_drop_h seed 4: dr failed (max error 8.83 m)
- R2-B delay_drop_h seed 4: gnss_hold failed (max error 10.38 m)
- R2-B delay_drop_h seed 5: dr failed (max error 7.07 m)
- R2-B delay_drop_h seed 5: gnss_hold failed (max error 10.98 m)
- R2-B delay_drop_h seed 6: dr failed (max error 12.04 m)
- R2-B delay_drop_h seed 6: gnss_hold failed (max error 9.57 m)
- R2-B delay_drop_h seed 7: dr failed (max error 8.37 m)
- R2-B delay_drop_h seed 7: gnss_hold failed (max error 11.19 m)
- R2-B delay_drop_h seed 8: dr failed (max error 6.16 m)
- R2-B delay_drop_h seed 8: gnss_hold failed (max error 9.92 m)
- R2-B delay_drop_h seed 9: dr failed (max error 7.64 m)
- R2-B delay_drop_h seed 9: gnss_hold failed (max error 9.91 m)
- R2-B delay_drop_h seed 10: dr failed (max error 8.83 m)
- R2-B delay_drop_h seed 10: gnss_hold failed (max error 11.52 m)
- R2-B combined_h seed 1: dr failed (max error 20.24 m)
- R2-B combined_h seed 1: gnss_cv failed (max error 12.40 m)
- R2-B combined_h seed 1: gnss_hold failed (max error 101.26 m)
- R2-B combined_h seed 2: dr failed (max error 16.92 m)
- R2-B combined_h seed 2: gnss_cv failed (max error 117.64 m)
- R2-B combined_h seed 2: gnss_hold failed (max error 101.94 m)
- R2-B combined_h seed 3: dr failed (max error 10.66 m)
- R2-B combined_h seed 3: gnss_cv failed (max error 8.09 m)
- R2-B combined_h seed 3: gnss_hold failed (max error 102.66 m)
- R2-B combined_h seed 4: dr failed (max error 18.44 m)
- R2-B combined_h seed 4: gnss_cv failed (max error 12.02 m)
- R2-B combined_h seed 4: gnss_hold failed (max error 101.22 m)
- R2-B combined_h seed 5: dr failed (max error 13.16 m)
- R2-B combined_h seed 5: gnss_cv failed (max error 12.30 m)
- R2-B combined_h seed 5: gnss_hold failed (max error 102.13 m)
- R2-B combined_h seed 6: dr failed (max error 23.14 m)
- R2-B combined_h seed 6: gnss_cv failed (max error 27.32 m)
- R2-B combined_h seed 6: gnss_hold failed (max error 102.23 m)
- R2-B combined_h seed 7: dr failed (max error 18.18 m)
- R2-B combined_h seed 7: gnss_cv failed (max error 27.14 m)
- R2-B combined_h seed 7: gnss_hold failed (max error 102.18 m)
- R2-B combined_h seed 8: dr failed (max error 17.01 m)
- R2-B combined_h seed 8: gnss_cv failed (max error 10.75 m)
- R2-B combined_h seed 8: gnss_hold failed (max error 101.64 m)
- R2-B combined_h seed 9: dr failed (max error 20.04 m)
- R2-B combined_h seed 9: gnss_cv failed (max error 18.25 m)
- R2-B combined_h seed 9: gnss_hold failed (max error 101.52 m)
- R2-B combined_h seed 10: dr failed (max error 14.60 m)
- R2-B combined_h seed 10: gnss_cv failed (max error 28.00 m)
- R2-B combined_h seed 10: gnss_hold failed (max error 102.71 m)
- R2-C gnss_outage_h seed 1: gnss_cv failed (max error 35.51 m)
- R2-C gnss_outage_h seed 1: gnss_hold failed (max error 68.02 m)
- R2-C gnss_outage_h seed 2: gnss_cv failed (max error 31.36 m)
- R2-C gnss_outage_h seed 2: gnss_hold failed (max error 68.51 m)
- R2-C gnss_outage_h seed 3: gnss_cv failed (max error 41.59 m)
- R2-C gnss_outage_h seed 3: gnss_hold failed (max error 66.95 m)
- R2-C gnss_outage_h seed 4: gnss_cv failed (max error 36.61 m)
- R2-C gnss_outage_h seed 4: gnss_hold failed (max error 68.00 m)
- R2-C gnss_outage_h seed 5: gnss_cv failed (max error 33.76 m)
- R2-C gnss_outage_h seed 5: gnss_hold failed (max error 68.30 m)
- R2-C gnss_outage_h seed 6: gnss_cv failed (max error 35.98 m)
- R2-C gnss_outage_h seed 6: gnss_hold failed (max error 67.56 m)
- R2-C gnss_outage_h seed 7: gnss_cv failed (max error 33.02 m)
- R2-C gnss_outage_h seed 7: gnss_hold failed (max error 68.11 m)
- R2-C gnss_outage_h seed 8: gnss_cv failed (max error 36.71 m)
- R2-C gnss_outage_h seed 8: gnss_hold failed (max error 67.59 m)
- R2-C gnss_outage_h seed 9: gnss_cv failed (max error 39.07 m)
- R2-C gnss_outage_h seed 9: gnss_hold failed (max error 67.25 m)
- R2-C gnss_outage_h seed 10: gnss_cv failed (max error 35.58 m)
- R2-C gnss_outage_h seed 10: gnss_hold failed (max error 67.55 m)
- R2-C multipath_h seed 1: gnss_cv failed (max error 11.22 m)
- R2-C multipath_h seed 1: gnss_hold failed (max error 8.72 m)
- R2-C multipath_h seed 2: gnss_cv failed (max error 13.95 m)
- R2-C multipath_h seed 2: gnss_hold failed (max error 9.29 m)
- R2-C multipath_h seed 3: gnss_cv failed (max error 53.64 m)
- R2-C multipath_h seed 3: gnss_hold failed (max error 8.16 m)
- R2-C multipath_h seed 4: gnss_cv failed (max error 9.88 m)
- R2-C multipath_h seed 4: gnss_hold failed (max error 9.50 m)
- R2-C multipath_h seed 5: gnss_cv failed (max error 11.47 m)
- R2-C multipath_h seed 5: gnss_hold failed (max error 9.29 m)
- R2-C multipath_h seed 6: gnss_cv failed (max error 45.09 m)
- R2-C multipath_h seed 6: gnss_hold failed (max error 8.73 m)
- R2-C multipath_h seed 7: gnss_cv failed (max error 11.56 m)
- R2-C multipath_h seed 7: gnss_hold failed (max error 8.83 m)
- R2-C multipath_h seed 8: gnss_cv failed (max error 103.97 m)
- R2-C multipath_h seed 8: gnss_hold failed (max error 8.78 m)
- R2-C multipath_h seed 9: gnss_cv failed (max error 8.83 m)
- R2-C multipath_h seed 9: gnss_hold failed (max error 8.79 m)
- R2-C multipath_h seed 10: gnss_cv failed (max error 29.12 m)
- R2-C multipath_h seed 10: gnss_hold failed (max error 10.15 m)
- R2-C wheel_slip_h seed 1: dr failed (max error 18.70 m)
- R2-C wheel_slip_h seed 2: dr failed (max error 17.26 m)
- R2-C wheel_slip_h seed 3: dr failed (max error 14.98 m)
- R2-C wheel_slip_h seed 4: dr failed (max error 18.33 m)
- R2-C wheel_slip_h seed 5: dr failed (max error 14.58 m)
- R2-C wheel_slip_h seed 6: dr failed (max error 17.05 m)
- R2-C wheel_slip_h seed 7: dr failed (max error 18.80 m)
- R2-C wheel_slip_h seed 8: dr failed (max error 17.72 m)
- R2-C wheel_slip_h seed 9: dr failed (max error 16.67 m)
- R2-C wheel_slip_h seed 10: dr failed (max error 13.61 m)
- R2-C delay_drop_h seed 1: gnss_hold failed (max error 8.37 m)
- R2-C delay_drop_h seed 2: gnss_hold failed (max error 9.29 m)
- R2-C delay_drop_h seed 3: gnss_hold failed (max error 9.28 m)
- R2-C delay_drop_h seed 4: gnss_hold failed (max error 10.18 m)
- R2-C delay_drop_h seed 5: gnss_hold failed (max error 10.20 m)
- R2-C delay_drop_h seed 6: gnss_hold failed (max error 9.34 m)
- R2-C delay_drop_h seed 7: gnss_hold failed (max error 8.07 m)
- R2-C delay_drop_h seed 8: gnss_hold failed (max error 8.22 m)
- R2-C delay_drop_h seed 9: gnss_hold failed (max error 8.74 m)
- R2-C delay_drop_h seed 10: gnss_hold failed (max error 8.45 m)
- R2-C combined_h seed 1: dr failed (max error 10.53 m)
- R2-C combined_h seed 1: gnss_cv failed (max error 51.53 m)
- R2-C combined_h seed 1: gnss_hold failed (max error 43.55 m)
- R2-C combined_h seed 2: dr failed (max error 9.00 m)
- R2-C combined_h seed 2: gnss_cv failed (max error 50.50 m)
- R2-C combined_h seed 2: gnss_hold failed (max error 44.36 m)
- R2-C combined_h seed 3: dr failed (max error 6.97 m)
- R2-C combined_h seed 3: gnss_cv failed (max error 147.70 m)
- R2-C combined_h seed 3: gnss_hold failed (max error 43.37 m)
- R2-C combined_h seed 4: dr failed (max error 10.15 m)
- R2-C combined_h seed 4: gnss_cv failed (max error 50.90 m)
- R2-C combined_h seed 4: gnss_hold failed (max error 45.43 m)
- R2-C combined_h seed 5: dr failed (max error 6.48 m)
- R2-C combined_h seed 5: gnss_cv failed (max error 58.12 m)
- R2-C combined_h seed 5: gnss_hold failed (max error 44.70 m)
- R2-C combined_h seed 6: dr failed (max error 9.13 m)
- R2-C combined_h seed 6: gnss_cv failed (max error 48.46 m)
- R2-C combined_h seed 6: gnss_hold failed (max error 43.55 m)
- R2-C combined_h seed 7: dr failed (max error 10.53 m)
- R2-C combined_h seed 7: gnss_cv failed (max error 54.28 m)
- R2-C combined_h seed 7: gnss_hold failed (max error 43.99 m)
- R2-C combined_h seed 8: dr failed (max error 9.47 m)
- R2-C combined_h seed 8: gnss_cv failed (max error 48.34 m)
- R2-C combined_h seed 8: gnss_hold failed (max error 44.31 m)
- R2-C combined_h seed 9: dr failed (max error 8.62 m)
- R2-C combined_h seed 9: gnss_cv failed (max error 48.94 m)
- R2-C combined_h seed 9: gnss_hold failed (max error 44.24 m)
- R2-C combined_h seed 10: dr failed (max error 5.70 m)
- R2-C combined_h seed 10: gnss_cv failed (max error 49.55 m)
- R2-C combined_h seed 10: gnss_hold failed (max error 44.00 m)

Recovery 999 means the error did not stay below 0.5 m for 1 s before the drive ended. Latency and runtime are wall-clock times on the machine that ran the campaign and are not deterministic.
