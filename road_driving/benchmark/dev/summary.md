# Localisation benchmark: development split (network R1)

Open loop: each drive is recorded once with ground-truth driving; every estimator then processes the same sensor log in the same arrival order and is scored against the same truth in the fixed map frame, with no per-trajectory alignment. Errors are rear-axle positions sampled at 50 Hz from each estimator's initialisation. A run fails when the position error ever exceeds 5.0 m after initialising (failed runs stay in every statistic).

Drives: 90 (3 routes x 6 conditions x 5 seeds); drives not completing their route: 0. Toolchain: GCC 14.2.0. Engine commit at configure: 66e81250786942903d74b3d56e929b55293a707a.

## nominal

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.183 | 0.214 | 0.367 | 0.63 | 0.37 | - | - | 1.92 (3) | 0.92 | 1.55 | 9 | 12.2 |
| dr | 15 | 8 | 3.651 | 4.281 | 8.803 | 10.92 | 0.96 | - | - | - | - | - | 0 | 0.2 |
| gnss_cv | 15 | 0 | 0.585 | 0.587 | 0.866 | 3.41 | 24.40 | - | - | 4.24 (2) | 0.91 | 1.38 | 4 | 0.1 |
| gnss_hold | 15 | 0 | 1.266 | 1.295 | 2.021 | 3.86 | 25.15 | - | - | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## gnss_outage

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.253 | 0.293 | 0.575 | 1.31 | 0.43 | 0.51 | 0.1 | 1.93 (3) | 0.93 | 1.56 | 6 | 21.8 |
| dr | 15 | 8 | 3.651 | 4.281 | 8.803 | 10.92 | 0.96 | 4.26 | 999.0 | - | - | - | 0 | 0.2 |
| gnss_cv | 15 | 14 | 3.339 | 11.241 | 12.074 | 102.02 | 27.34 | 20.97 | 0.8 | 4.04 (2) | 0.85 | 1.39 | 3 | 0.2 |
| gnss_hold | 15 | 15 | 33.806 | 34.371 | 88.711 | 136.18 | 26.61 | 100.15 | 999.0 | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## multipath

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.272 | 0.426 | 0.470 | 2.75 | 0.49 | - | - | 3.74 (3) | 0.91 | 22.64 | 1503 | 21.5 |
| dr | 15 | 8 | 3.651 | 4.281 | 8.803 | 10.92 | 0.96 | - | - | - | - | - | 0 | 0.3 |
| gnss_cv | 15 | 15 | 1.662 | 1.864 | 4.800 | 11.73 | 25.20 | - | - | 16.82 (2) | 0.78 | 4.60 | 893 | 0.2 |
| gnss_hold | 15 | 15 | 2.001 | 1.968 | 4.432 | 7.52 | 36.46 | - | - | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## wheel_slip

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.183 | 0.217 | 0.371 | 0.63 | 0.37 | - | - | 1.92 (3) | 0.92 | 1.55 | 11 | 22.2 |
| dr | 15 | 15 | 12.566 | 11.943 | 18.288 | 21.32 | 0.96 | - | - | - | - | - | 0 | 0.3 |
| gnss_cv | 15 | 0 | 0.585 | 0.587 | 0.866 | 3.41 | 24.40 | - | - | 4.24 (2) | 0.91 | 1.38 | 4 | 0.2 |
| gnss_hold | 15 | 0 | 1.266 | 1.295 | 2.021 | 3.86 | 25.15 | - | - | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## imu_bias

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.286 | 0.289 | 0.540 | 0.74 | 1.25 | - | - | 27.17 (3) | 0.52 | 2.04 | 48 | 22.5 |
| dr | 15 | 15 | 9.257 | 9.841 | 22.006 | 35.80 | 6.04 | - | - | - | - | - | 0 | 0.3 |
| gnss_cv | 15 | 0 | 0.585 | 0.587 | 0.866 | 3.41 | 24.40 | - | - | 4.24 (2) | 0.91 | 1.38 | 4 | 0.2 |
| gnss_hold | 15 | 0 | 1.266 | 1.295 | 2.021 | 3.86 | 25.15 | - | - | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## delay_drop

| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | GNSS NIS mean | GNSS rejected | p99 latency (us) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ekf | 15 | 0 | 0.183 | 0.221 | 0.389 | 0.61 | 0.39 | - | - | 1.86 (3) | 0.94 | 1.57 | 7 | 87.1 |
| dr | 15 | 9 | 3.971 | 4.256 | 8.562 | 10.53 | 1.12 | - | - | - | - | - | 0 | 0.3 |
| gnss_cv | 15 | 0 | 0.726 | 0.709 | 1.251 | 3.67 | 23.00 | - | - | 1.26 (2) | 0.91 | 1.34 | 1 | 0.2 |
| gnss_hold | 15 | 15 | 3.696 | 3.717 | 5.676 | 10.16 | 17.83 | - | - | - | - | - | 0 | 0.1 |

EKF RMSE lower than: dr in 15 of 15 runs; gnss_cv in 15 of 15 runs; gnss_hold in 15 of 15 runs.

## Failed runs

- R1-A nominal seed 1: dr failed (max error 9.66 m)
- R1-A nominal seed 4: dr failed (max error 5.95 m)
- R1-A gnss_outage seed 1: dr failed (max error 9.66 m)
- R1-A gnss_outage seed 1: gnss_cv failed (max error 21.67 m)
- R1-A gnss_outage seed 1: gnss_hold failed (max error 100.58 m)
- R1-A gnss_outage seed 2: gnss_cv failed (max error 30.47 m)
- R1-A gnss_outage seed 2: gnss_hold failed (max error 100.33 m)
- R1-A gnss_outage seed 3: gnss_cv failed (max error 32.01 m)
- R1-A gnss_outage seed 3: gnss_hold failed (max error 100.14 m)
- R1-A gnss_outage seed 4: dr failed (max error 5.95 m)
- R1-A gnss_outage seed 4: gnss_cv failed (max error 12.53 m)
- R1-A gnss_outage seed 4: gnss_hold failed (max error 101.68 m)
- R1-A gnss_outage seed 5: gnss_cv failed (max error 21.77 m)
- R1-A gnss_outage seed 5: gnss_hold failed (max error 100.17 m)
- R1-A multipath seed 1: dr failed (max error 9.66 m)
- R1-A multipath seed 1: gnss_cv failed (max error 8.41 m)
- R1-A multipath seed 1: gnss_hold failed (max error 6.72 m)
- R1-A multipath seed 2: gnss_cv failed (max error 6.45 m)
- R1-A multipath seed 2: gnss_hold failed (max error 6.30 m)
- R1-A multipath seed 3: gnss_cv failed (max error 7.23 m)
- R1-A multipath seed 3: gnss_hold failed (max error 6.16 m)
- R1-A multipath seed 4: dr failed (max error 5.95 m)
- R1-A multipath seed 4: gnss_cv failed (max error 5.19 m)
- R1-A multipath seed 4: gnss_hold failed (max error 6.17 m)
- R1-A multipath seed 5: gnss_cv failed (max error 6.00 m)
- R1-A multipath seed 5: gnss_hold failed (max error 5.66 m)
- R1-A wheel_slip seed 1: dr failed (max error 21.32 m)
- R1-A wheel_slip seed 2: dr failed (max error 16.43 m)
- R1-A wheel_slip seed 3: dr failed (max error 11.25 m)
- R1-A wheel_slip seed 4: dr failed (max error 17.90 m)
- R1-A wheel_slip seed 5: dr failed (max error 11.73 m)
- R1-A imu_bias seed 1: dr failed (max error 8.03 m)
- R1-A imu_bias seed 2: dr failed (max error 8.82 m)
- R1-A imu_bias seed 3: dr failed (max error 13.90 m)
- R1-A imu_bias seed 4: dr failed (max error 8.59 m)
- R1-A imu_bias seed 5: dr failed (max error 15.81 m)
- R1-A delay_drop seed 1: dr failed (max error 9.35 m)
- R1-A delay_drop seed 1: gnss_hold failed (max error 8.34 m)
- R1-A delay_drop seed 2: gnss_hold failed (max error 9.66 m)
- R1-A delay_drop seed 3: gnss_hold failed (max error 9.75 m)
- R1-A delay_drop seed 4: dr failed (max error 5.64 m)
- R1-A delay_drop seed 4: gnss_hold failed (max error 10.16 m)
- R1-A delay_drop seed 5: dr failed (max error 5.80 m)
- R1-A delay_drop seed 5: gnss_hold failed (max error 8.26 m)
- R1-B nominal seed 1: dr failed (max error 9.49 m)
- R1-B nominal seed 4: dr failed (max error 6.73 m)
- R1-B nominal seed 5: dr failed (max error 5.24 m)
- R1-B gnss_outage seed 1: dr failed (max error 9.49 m)
- R1-B gnss_outage seed 1: gnss_cv failed (max error 102.02 m)
- R1-B gnss_outage seed 1: gnss_hold failed (max error 92.01 m)
- R1-B gnss_outage seed 2: gnss_cv failed (max error 82.22 m)
- R1-B gnss_outage seed 2: gnss_hold failed (max error 92.21 m)
- R1-B gnss_outage seed 3: gnss_cv failed (max error 98.33 m)
- R1-B gnss_outage seed 3: gnss_hold failed (max error 92.00 m)
- R1-B gnss_outage seed 4: dr failed (max error 6.73 m)
- R1-B gnss_outage seed 4: gnss_cv failed (max error 91.44 m)
- R1-B gnss_outage seed 4: gnss_hold failed (max error 91.86 m)
- R1-B gnss_outage seed 5: dr failed (max error 5.24 m)
- R1-B gnss_outage seed 5: gnss_cv failed (max error 84.46 m)
- R1-B gnss_outage seed 5: gnss_hold failed (max error 91.11 m)
- R1-B multipath seed 1: dr failed (max error 9.49 m)
- R1-B multipath seed 1: gnss_cv failed (max error 6.06 m)
- R1-B multipath seed 1: gnss_hold failed (max error 6.72 m)
- R1-B multipath seed 2: gnss_cv failed (max error 7.75 m)
- R1-B multipath seed 2: gnss_hold failed (max error 6.21 m)
- R1-B multipath seed 3: gnss_cv failed (max error 7.24 m)
- R1-B multipath seed 3: gnss_hold failed (max error 6.00 m)
- R1-B multipath seed 4: dr failed (max error 6.73 m)
- R1-B multipath seed 4: gnss_cv failed (max error 6.67 m)
- R1-B multipath seed 4: gnss_hold failed (max error 6.17 m)
- R1-B multipath seed 5: dr failed (max error 5.24 m)
- R1-B multipath seed 5: gnss_cv failed (max error 6.93 m)
- R1-B multipath seed 5: gnss_hold failed (max error 5.66 m)
- R1-B wheel_slip seed 1: dr failed (max error 19.19 m)
- R1-B wheel_slip seed 2: dr failed (max error 17.34 m)
- R1-B wheel_slip seed 3: dr failed (max error 13.35 m)
- R1-B wheel_slip seed 4: dr failed (max error 18.01 m)
- R1-B wheel_slip seed 5: dr failed (max error 17.10 m)
- R1-B imu_bias seed 1: dr failed (max error 21.74 m)
- R1-B imu_bias seed 2: dr failed (max error 21.04 m)
- R1-B imu_bias seed 3: dr failed (max error 17.43 m)
- R1-B imu_bias seed 4: dr failed (max error 22.42 m)
- R1-B imu_bias seed 5: dr failed (max error 18.95 m)
- R1-B delay_drop seed 1: dr failed (max error 9.20 m)
- R1-B delay_drop seed 1: gnss_hold failed (max error 7.71 m)
- R1-B delay_drop seed 2: gnss_hold failed (max error 9.66 m)
- R1-B delay_drop seed 3: gnss_hold failed (max error 9.75 m)
- R1-B delay_drop seed 4: dr failed (max error 6.52 m)
- R1-B delay_drop seed 4: gnss_hold failed (max error 10.16 m)
- R1-B delay_drop seed 5: dr failed (max error 5.99 m)
- R1-B delay_drop seed 5: gnss_hold failed (max error 8.26 m)
- R1-C nominal seed 1: dr failed (max error 10.92 m)
- R1-C nominal seed 4: dr failed (max error 6.35 m)
- R1-C nominal seed 5: dr failed (max error 5.47 m)
- R1-C gnss_outage seed 1: dr failed (max error 10.92 m)
- R1-C gnss_outage seed 1: gnss_hold failed (max error 135.30 m)
- R1-C gnss_outage seed 2: gnss_cv failed (max error 9.24 m)
- R1-C gnss_outage seed 2: gnss_hold failed (max error 135.40 m)
- R1-C gnss_outage seed 3: gnss_cv failed (max error 11.46 m)
- R1-C gnss_outage seed 3: gnss_hold failed (max error 136.06 m)
- R1-C gnss_outage seed 4: dr failed (max error 6.35 m)
- R1-C gnss_outage seed 4: gnss_cv failed (max error 10.64 m)
- R1-C gnss_outage seed 4: gnss_hold failed (max error 135.40 m)
- R1-C gnss_outage seed 5: dr failed (max error 5.47 m)
- R1-C gnss_outage seed 5: gnss_cv failed (max error 12.06 m)
- R1-C gnss_outage seed 5: gnss_hold failed (max error 136.18 m)
- R1-C multipath seed 1: dr failed (max error 10.92 m)
- R1-C multipath seed 1: gnss_cv failed (max error 10.07 m)
- R1-C multipath seed 1: gnss_hold failed (max error 6.25 m)
- R1-C multipath seed 2: gnss_cv failed (max error 7.44 m)
- R1-C multipath seed 2: gnss_hold failed (max error 6.41 m)
- R1-C multipath seed 3: gnss_cv failed (max error 11.73 m)
- R1-C multipath seed 3: gnss_hold failed (max error 6.80 m)
- R1-C multipath seed 4: dr failed (max error 6.35 m)
- R1-C multipath seed 4: gnss_cv failed (max error 7.04 m)
- R1-C multipath seed 4: gnss_hold failed (max error 7.52 m)
- R1-C multipath seed 5: dr failed (max error 5.47 m)
- R1-C multipath seed 5: gnss_cv failed (max error 6.26 m)
- R1-C multipath seed 5: gnss_hold failed (max error 6.89 m)
- R1-C wheel_slip seed 1: dr failed (max error 20.63 m)
- R1-C wheel_slip seed 2: dr failed (max error 17.72 m)
- R1-C wheel_slip seed 3: dr failed (max error 11.49 m)
- R1-C wheel_slip seed 4: dr failed (max error 17.31 m)
- R1-C wheel_slip seed 5: dr failed (max error 13.81 m)
- R1-C imu_bias seed 1: dr failed (max error 28.15 m)
- R1-C imu_bias seed 2: dr failed (max error 28.51 m)
- R1-C imu_bias seed 3: dr failed (max error 34.55 m)
- R1-C imu_bias seed 4: dr failed (max error 29.58 m)
- R1-C imu_bias seed 5: dr failed (max error 35.80 m)
- R1-C delay_drop seed 1: dr failed (max error 10.53 m)
- R1-C delay_drop seed 1: gnss_hold failed (max error 8.35 m)
- R1-C delay_drop seed 2: gnss_hold failed (max error 10.09 m)
- R1-C delay_drop seed 3: gnss_hold failed (max error 8.61 m)
- R1-C delay_drop seed 4: dr failed (max error 5.95 m)
- R1-C delay_drop seed 4: gnss_hold failed (max error 9.77 m)
- R1-C delay_drop seed 5: dr failed (max error 5.65 m)
- R1-C delay_drop seed 5: gnss_hold failed (max error 9.42 m)

Recovery 999 means the error did not stay below 0.5 m for 1 s before the drive ended. Latency and runtime are wall-clock times on the machine that ran the campaign and are not deterministic.
