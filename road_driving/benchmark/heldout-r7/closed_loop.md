# Closed-loop driving on estimated state: held-out evaluation of the unique re-join after pose corrections (network R7)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. "no re-join" is the stack evaluated on R3 and R4. "R6 re-join" re-joins every correction the estimator verified against the map (kerb registration), as evaluated on R6. Every other row re-joins only those whose kerb scan fits the map uniquely, as tested at the registered pose or by the re-localisation search; nothing jumps on simulator truth.

## R7-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 80.1 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 39035 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf_int, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf_int, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf_int2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf_int2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.549 | -0.202 | 2 | 0.138 | 0 of 38763 | 0 | 0 | 0 | 0.169 | 0.61 |
| ekf_kerb, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38765 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf_int, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf_int, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf_int2, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf_int2, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 34764 | 0 | 10 | 3 | 0.267 | 1.06 |
| ekf_kerb, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf_kerb, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf_kerb2, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf_kerb2, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf_kerb, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf_kerb2, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.6 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38769 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 20806 | 0 | 10 | 4 | 2.035 | 8.58 |
| ekf, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 20806 | 0 | 10 | 4 | 2.035 | 8.58 |
| ekf_int, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 27676 | 0 | 10 | 4 | 0.336 | 1.03 |
| ekf_int, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 27676 | 0 | 10 | 4 | 0.336 | 1.03 |
| ekf_int2, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 27676 | 0 | 10 | 4 | 0.336 | 1.03 |
| ekf_int2, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 27676 | 0 | 10 | 4 | 0.336 | 1.03 |
| ekf_kerb, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38744 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf_kerb, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38744 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf_kerb2, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38743 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38743 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38744 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf_kerb2, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 79.5 | - | 0.193 | 0.045 | 0 | 0.000 | 0 of 38743 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf, no re-join, kerb_return_r7 | 10 | COMPLETED 3, STOPPED_EN_ROUTE 7 | 3 of 10 | 79.4 | - | 1.500 | -5.602 | 9 | 5.538 | 342 of 25377 | 0 | 6 | 0 | 2.379 | 6.56 |
| ekf, kerb_return_r7 | 10 | COMPLETED 3, STOPPED_EN_ROUTE 7 | 3 of 10 | 79.4 | - | 1.500 | -5.602 | 9 | 5.538 | 342 of 25377 | 0 | 6 | 0 | 2.379 | 6.56 |
| ekf_int, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -5.602 | 8 | 5.538 | 278 of 27139 | 0 | 4 | 1 | 1.827 | 6.56 |
| ekf_int, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -5.602 | 8 | 5.538 | 278 of 27139 | 0 | 4 | 1 | 1.827 | 6.56 |
| ekf_int2, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.420 | 8 | 4.356 | 278 of 27122 | 0 | 2 | 1 | 1.809 | 4.98 |
| ekf_int2, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.420 | 8 | 4.356 | 278 of 27122 | 0 | 2 | 1 | 1.809 | 4.98 |
| ekf_kerb, no re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.4 | - | 1.500 | -10.272 | 8 | 10.218 | 382 of 29707 | 0 | 3 | 0 | 1.662 | 11.85 |
| ekf_kerb, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.5 | - | 1.500 | -10.272 | 8 | 10.218 | 382 of 29727 | 0 | 3 | 0 | 1.662 | 11.85 |
| ekf_kerb2, no re-join, kerb_return_r7 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 79.9 | - | 1.500 | -4.406 | 7 | 4.342 | 271 of 22216 | 0 | 0 | 0 | 1.842 | 4.77 |
| ekf_kerb2, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 79.4 | - | 1.500 | -4.406 | 7 | 4.342 | 271 of 26143 | 0 | 0 | 0 | 1.669 | 4.77 |
| ekf_kerb, R6 re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.5 | - | 1.500 | -10.273 | 8 | 10.220 | 382 of 29747 | 0 | 3 | 0 | 1.663 | 11.85 |
| ekf_kerb2, R6 re-join, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 79.4 | - | 1.500 | -4.406 | 7 | 4.342 | 271 of 26143 | 0 | 0 | 0 | 1.669 | 4.77 |
| ekf, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -8.172 | 10 | 8.111 | 0 of 12783 | 0 | 10 | 0 | 3.278 | 10.30 |
| ekf, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -8.172 | 10 | 8.111 | 0 of 12783 | 0 | 10 | 0 | 3.278 | 10.30 |
| ekf_int, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -8.172 | 10 | 8.111 | 0 of 12783 | 0 | 10 | 0 | 3.280 | 10.30 |
| ekf_int, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -8.172 | 10 | 8.111 | 0 of 12783 | 0 | 10 | 0 | 3.280 | 10.30 |
| ekf_int2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.939 | 10 | 5.876 | 0 of 12783 | 0 | 10 | 0 | 1.726 | 7.86 |
| ekf_int2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.939 | 10 | 5.876 | 0 of 12783 | 0 | 10 | 0 | 1.726 | 7.86 |
| ekf_kerb, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -13.919 | 10 | 13.873 | 0 of 12976 | 0 | 13 | 0 | 7.523 | 26.26 |
| ekf_kerb, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -13.919 | 10 | 13.873 | 0 of 12976 | 0 | 13 | 0 | 7.523 | 26.26 |
| ekf_kerb2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -6.487 | 10 | 6.424 | 0 of 12976 | 0 | 8 | 0 | 1.622 | 8.58 |
| ekf_kerb2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -6.487 | 10 | 6.424 | 0 of 12976 | 0 | 8 | 0 | 1.622 | 8.58 |
| ekf_kerb, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -13.909 | 10 | 13.862 | 0 of 12976 | 0 | 13 | 0 | 7.523 | 26.26 |
| ekf_kerb2, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.496 | -6.277 | 10 | 6.214 | 0 of 12976 | 0 | 8 | 0 | 1.622 | 8.59 |
| ekf, no re-join, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.534 | 10 | 4.470 | 237 of 28860 | 0 | 5 | 0 | 1.581 | 5.59 |
| ekf, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.534 | 10 | 4.470 | 237 of 28860 | 0 | 5 | 0 | 1.581 | 5.59 |
| ekf_int, no re-join, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.304 | 10 | 4.240 | 237 of 29430 | 0 | 2 | 0 | 1.519 | 4.72 |
| ekf_int, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.304 | 10 | 4.240 | 237 of 29430 | 0 | 2 | 0 | 1.519 | 4.72 |
| ekf_int2, no re-join, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.304 | 10 | 4.240 | 237 of 29392 | 0 | 2 | 0 | 1.510 | 4.72 |
| ekf_int2, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.304 | 10 | 4.240 | 237 of 29392 | 0 | 2 | 0 | 1.510 | 4.72 |
| ekf_kerb, no re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 79.6 | - | 1.500 | -4.148 | 10 | 4.084 | 177 of 31176 | 0 | 1 | 0 | 1.364 | 4.59 |
| ekf_kerb, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 79.7 | - | 1.500 | -4.148 | 10 | 4.084 | 177 of 31177 | 0 | 1 | 0 | 1.364 | 4.59 |
| ekf_kerb2, no re-join, kerb_faults_r7 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 80.2 | - | 1.500 | -4.148 | 7 | 4.084 | 177 of 25495 | 0 | 1 | 0 | 1.403 | 4.59 |
| ekf_kerb2, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.148 | 8 | 4.084 | 177 of 29602 | 0 | 1 | 0 | 1.222 | 4.59 |
| ekf_kerb, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 79.7 | - | 1.500 | -4.148 | 10 | 4.084 | 177 of 31176 | 0 | 1 | 0 | 1.364 | 4.59 |
| ekf_kerb2, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 79.5 | - | 1.500 | -4.148 | 8 | 4.084 | 177 of 29602 | 0 | 1 | 0 | 1.222 | 4.59 |
| ekf, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 0.877 | 8.00 |
| ekf, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 0.877 | 8.00 |
| ekf_int, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 1.181 | 5.47 |
| ekf_int, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 1.181 | 5.47 |
| ekf_int2, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 1.181 | 5.47 |
| ekf_int2, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.202 | 2 | 0.138 | 0 of 18393 | 0 | 15 | 9 | 1.181 | 5.47 |
| ekf_kerb, no re-join, combined_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.6 | - | 1.499 | -2.570 | 5 | 2.507 | 136 of 35716 | 0 | 2 | 0 | 0.751 | 3.49 |
| ekf_kerb, combined_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.6 | - | 1.499 | -2.570 | 5 | 2.507 | 136 of 35715 | 0 | 2 | 0 | 0.751 | 3.49 |
| ekf_kerb2, no re-join, combined_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 79.5 | - | 1.496 | -2.084 | 5 | 2.023 | 83 of 35923 | 0 | 2 | 0 | 0.451 | 3.37 |
| ekf_kerb2, combined_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.5 | - | 1.496 | -2.084 | 4 | 2.023 | 83 of 36499 | 0 | 2 | 0 | 0.451 | 3.37 |
| ekf_kerb, R6 re-join, combined_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.6 | - | 1.499 | -2.570 | 5 | 2.507 | 136 of 35715 | 0 | 2 | 0 | 0.751 | 3.49 |
| ekf_kerb2, R6 re-join, combined_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 79.5 | - | 1.496 | -2.084 | 4 | 2.023 | 83 of 36499 | 0 | 2 | 0 | 0.451 | 3.37 |

## R7-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 76.7 | - | 0.193 | 0.178 | 0 | 0.000 | 0 of 37350 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf_int, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf_int, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf_int2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf_int2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.420 | 0.001 | 0 | 0.000 | 0 of 37208 | 0 | 0 | 0 | 0.199 | 0.66 |
| ekf_kerb, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 37206 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf_int, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf_int, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf_int2, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf_int2, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.579 | 0.001 | 0 | 0.000 | 0 of 36761 | 0 | 10 | 7 | 0.196 | 0.66 |
| ekf_kerb, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf_kerb, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf_kerb2, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf_kerb2, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf_kerb, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf_kerb2, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37223 | 0 | 0 | 0 | 0.075 | 0.66 |
| ekf, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.485 | 0.001 | 0 | 0.000 | 0 of 19309 | 0 | 10 | 8 | 1.933 | 8.64 |
| ekf, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.485 | 0.001 | 0 | 0.000 | 0 of 19309 | 0 | 10 | 8 | 1.933 | 8.64 |
| ekf_int, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.684 | 0.001 | 0 | 0.000 | 0 of 40696 | 0 | 10 | 8 | 0.179 | 0.66 |
| ekf_int, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.684 | 0.001 | 0 | 0.000 | 0 of 40696 | 0 | 10 | 8 | 0.179 | 0.66 |
| ekf_int2, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.684 | 0.001 | 0 | 0.000 | 0 of 40696 | 0 | 10 | 8 | 0.179 | 0.66 |
| ekf_int2, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.684 | 0.001 | 0 | 0.000 | 0 of 40696 | 0 | 10 | 8 | 0.179 | 0.66 |
| ekf_kerb, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37194 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf_kerb, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37194 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf_kerb2, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37196 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf_kerb2, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37196 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf_kerb, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37194 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf_kerb2, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.193 | 0.168 | 0 | 0.000 | 0 of 37196 | 0 | 0 | 0 | 0.076 | 0.66 |
| ekf, no re-join, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 85.1 | - | 1.499 | -4.490 | 10 | 4.426 | 37 of 23883 | 0 | 7 | 0 | 2.411 | 5.81 |
| ekf, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 85.1 | - | 1.499 | -4.490 | 10 | 4.426 | 37 of 23883 | 0 | 7 | 0 | 2.411 | 5.81 |
| ekf_int, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 76.4 | - | 1.500 | -4.490 | 10 | 4.426 | 37 of 24579 | 0 | 5 | 0 | 2.069 | 5.00 |
| ekf_int, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 76.4 | - | 1.500 | -4.490 | 10 | 4.426 | 37 of 24579 | 0 | 5 | 0 | 2.069 | 5.00 |
| ekf_int2, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 76.4 | - | 1.499 | -4.490 | 10 | 4.426 | 37 of 24564 | 0 | 5 | 0 | 1.967 | 5.00 |
| ekf_int2, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 76.4 | - | 1.499 | -4.490 | 10 | 4.426 | 37 of 24564 | 0 | 5 | 0 | 1.967 | 5.00 |
| ekf_kerb, no re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 76.5 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 27962 | 0 | 0 | 0 | 1.815 | 4.71 |
| ekf_kerb, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 76.5 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 27959 | 0 | 0 | 0 | 1.815 | 4.71 |
| ekf_kerb2, no re-join, kerb_return_r7 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 75.8 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 20596 | 1 | 0 | 0 | 1.868 | 4.71 |
| ekf_kerb2, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 76.3 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 23620 | 1 | 0 | 0 | 1.754 | 4.71 |
| ekf_kerb, R6 re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 76.5 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 27959 | 0 | 0 | 0 | 1.815 | 4.71 |
| ekf_kerb2, R6 re-join, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 76.3 | - | 1.500 | -4.249 | 10 | 4.185 | 36 of 23620 | 1 | 0 | 0 | 1.754 | 4.71 |
| ekf, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.906 | 5.74 |
| ekf, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.906 | 5.74 |
| ekf_int, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.904 | 5.74 |
| ekf_int, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.904 | 5.74 |
| ekf_int2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.297 | 10 | 3.234 | 0 of 11138 | 0 | 10 | 0 | 1.780 | 5.53 |
| ekf_int2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.297 | 10 | 3.234 | 0 of 11138 | 0 | 10 | 0 | 1.780 | 5.53 |
| ekf_kerb, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -26.325 | 10 | 26.266 | 0 of 11138 | 0 | 13 | 0 | 3.172 | 27.94 |
| ekf_kerb, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -26.325 | 10 | 26.266 | 0 of 11138 | 0 | 13 | 0 | 3.172 | 27.94 |
| ekf_kerb2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.653 | 10 | 7.590 | 0 of 11138 | 0 | 10 | 0 | 2.930 | 9.50 |
| ekf_kerb2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.653 | 10 | 7.590 | 0 of 11138 | 0 | 10 | 0 | 2.930 | 9.50 |
| ekf_kerb, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -20.705 | 10 | 20.645 | 0 of 13958 | 0 | 15 | 0 | 7.656 | 22.14 |
| ekf_kerb2, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.540 | 10 | 7.477 | 0 of 12614 | 0 | 10 | 0 | 2.749 | 9.41 |
| ekf, no re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 10 | 3.245 | 0 of 29870 | 0 | 2 | 0 | 1.340 | 5.59 |
| ekf, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 10 | 3.245 | 0 of 29870 | 0 | 2 | 0 | 1.340 | 5.59 |
| ekf_int, no re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 8 | 3.245 | 0 of 29870 | 0 | 0 | 0 | 1.340 | 4.18 |
| ekf_int, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 8 | 3.245 | 0 of 29870 | 0 | 0 | 0 | 1.340 | 4.18 |
| ekf_int2, no re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 8 | 3.245 | 0 of 29870 | 0 | 0 | 0 | 1.340 | 4.18 |
| ekf_int2, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.3 | - | 1.499 | -3.309 | 8 | 3.245 | 0 of 29870 | 0 | 0 | 0 | 1.340 | 4.18 |
| ekf_kerb, no re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.4 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 30076 | 0 | 0 | 0 | 1.260 | 4.81 |
| ekf_kerb, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.4 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 30078 | 0 | 0 | 0 | 1.260 | 4.81 |
| ekf_kerb2, no re-join, kerb_faults_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 76.5 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 26689 | 0 | 0 | 0 | 1.293 | 4.73 |
| ekf_kerb2, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.4 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 30088 | 0 | 0 | 0 | 1.260 | 4.73 |
| ekf_kerb, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.4 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 30078 | 0 | 0 | 0 | 1.260 | 4.81 |
| ekf_kerb2, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 76.4 | - | 1.500 | -3.024 | 10 | 2.960 | 0 of 30088 | 0 | 0 | 0 | 1.260 | 4.73 |
| ekf, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int2, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int2, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_kerb, no re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.219 | 0.167 | 0 | 0.000 | 0 of 37238 | 0 | 0 | 0 | 0.250 | 3.25 |
| ekf_kerb, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.219 | 0.167 | 0 | 0.000 | 0 of 37238 | 0 | 0 | 0 | 0.250 | 3.25 |
| ekf_kerb2, no re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.219 | 0.166 | 0 | 0.000 | 0 of 37250 | 0 | 0 | 0 | 0.251 | 3.25 |
| ekf_kerb2, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.219 | 0.166 | 0 | 0.000 | 0 of 37250 | 0 | 0 | 0 | 0.251 | 3.25 |
| ekf_kerb, R6 re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.4 | - | 0.219 | 0.167 | 0 | 0.000 | 0 of 37238 | 0 | 0 | 0 | 0.250 | 3.25 |
| ekf_kerb2, R6 re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 76.5 | - | 0.219 | 0.166 | 0 | 0.000 | 0 of 37250 | 0 | 0 | 0 | 0.251 | 3.25 |

## R7-B_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.85 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37393 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int, no re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int2, no re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int2, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_kerb, no re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf_kerb, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf_kerb2, no re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf_kerb2, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf_kerb, R6 re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf_kerb2, R6 re-join, nominal_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37148 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf, no re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf_int, no re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf_int, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf_int2, no re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf_int2, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 4.54 | 0.518 | 0.001 | 0 | 0.000 | 0 of 36558 | 0 | 10 | 7 | 0.193 | 0.66 |
| ekf_kerb, no re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, no re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb, R6 re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf_kerb2, R6 re-join, gnss_outage_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.81 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37150 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf, no re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 4.978 | 9.97 |
| ekf, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 4.978 | 9.97 |
| ekf_int, no re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 0.168 | 0.66 |
| ekf_int, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 0.168 | 0.66 |
| ekf_int2, no re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 0.168 | 0.66 |
| ekf_int2, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.66 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37163 | 0 | 0 | 0 | 0.168 | 0.66 |
| ekf_kerb, no re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb2, no re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb2, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb, R6 re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb2, R6 re-join, multipath_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.84 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37179 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf, no re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 1.800 | 5.51 |
| ekf, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 1.800 | 5.51 |
| ekf_int, no re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 0.847 | 2.88 |
| ekf_int, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 0.847 | 2.88 |
| ekf_int2, no re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 0.847 | 2.88 |
| ekf_int2, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.35 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37124 | 0 | 0 | 0 | 0.847 | 2.88 |
| ekf_kerb, no re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf_kerb, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf_kerb2, no re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf_kerb2, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf_kerb, R6 re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf_kerb2, R6 re-join, kerb_return_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.48 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37093 | 0 | 0 | 0 | 0.968 | 2.44 |
| ekf, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.600 | 5.74 |
| ekf, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.600 | 5.74 |
| ekf_int, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.652 | 5.74 |
| ekf_int, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.446 | 10 | 3.383 | 0 of 11138 | 0 | 10 | 0 | 1.652 | 5.74 |
| ekf_int2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.297 | 10 | 3.234 | 0 of 11138 | 0 | 10 | 0 | 1.580 | 5.53 |
| ekf_int2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.297 | 10 | 3.234 | 0 of 11138 | 0 | 10 | 0 | 1.580 | 5.53 |
| ekf_kerb, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -24.584 | 10 | 24.524 | 0 of 11138 | 0 | 11 | 0 | 1.324 | 26.17 |
| ekf_kerb, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -24.584 | 10 | 24.524 | 0 of 11138 | 0 | 11 | 0 | 1.324 | 26.17 |
| ekf_kerb2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.380 | 10 | 7.317 | 0 of 11138 | 0 | 10 | 0 | 1.324 | 9.35 |
| ekf_kerb2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.380 | 10 | 7.317 | 0 of 11138 | 0 | 10 | 0 | 1.324 | 9.35 |
| ekf_kerb, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -17.047 | 10 | 16.986 | 0 of 11138 | 0 | 11 | 0 | 1.324 | 18.70 |
| ekf_kerb2, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.292 | 10 | 5.229 | 0 of 11138 | 0 | 10 | 0 | 1.324 | 7.78 |
| ekf, no re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int, no re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int2, no re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_int2, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.63 | 0.384 | 0.001 | 0 | 0.000 | 0 of 37164 | 0 | 0 | 0 | 0.173 | 0.66 |
| ekf_kerb, no re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf_kerb, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf_kerb2, no re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf_kerb2, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf_kerb, R6 re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf_kerb2, R6 re-join, kerb_faults_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.30 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37161 | 0 | 0 | 0 | 0.087 | 0.88 |
| ekf, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int2, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_int2, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.926 | 0.001 | 0 | 0.000 | 0 of 31514 | 0 | 10 | 2 | 0.354 | 0.79 |
| ekf_kerb, no re-join, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |
| ekf_kerb, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |
| ekf_kerb2, no re-join, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |
| ekf_kerb2, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |
| ekf_kerb, R6 re-join, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |
| ekf_kerb2, R6 re-join, combined_r7 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.178 | 0 | 0.000 | 0 of 37002 | 0 | 0 | 0 | 0.108 | 0.66 |

## R7-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 90.0 | - | 0.193 | 0.028 | 0 | 0.000 | 0 of 43990 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf_int, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf_int, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf_int2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf_int2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.573 | -0.202 | 2 | 0.138 | 0 of 43801 | 0 | 0 | 0 | 0.176 | 0.61 |
| ekf_kerb, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf_kerb, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf_kerb2, no re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf_kerb2, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf_kerb, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf_kerb2, R6 re-join, nominal_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43836 | 0 | 0 | 0 | 0.069 | 0.61 |
| ekf, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf_int, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf_int, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf_int2, no re-join, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf_int2, gnss_outage_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.928 | -0.202 | 2 | 0.138 | 0 of 38051 | 0 | 10 | 7 | 0.165 | 0.76 |
| ekf_kerb, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, no re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf_kerb2, R6 re-join, gnss_outage_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43828 | 0 | 0 | 0 | 0.072 | 0.61 |
| ekf, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.485 | -8.125 | 3 | 8.061 | 0 of 21170 | 0 | 11 | 6 | 1.690 | 9.06 |
| ekf, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.485 | -8.125 | 3 | 8.061 | 0 of 21170 | 0 | 11 | 6 | 1.690 | 9.06 |
| ekf_int, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.742 | -0.202 | 2 | 0.138 | 0 of 41711 | 0 | 10 | 6 | 0.275 | 0.61 |
| ekf_int, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.742 | -0.202 | 2 | 0.138 | 0 of 41711 | 0 | 10 | 6 | 0.275 | 0.61 |
| ekf_int2, no re-join, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.742 | -0.202 | 2 | 0.138 | 0 of 41711 | 0 | 10 | 6 | 0.275 | 0.61 |
| ekf_int2, multipath_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.742 | -0.202 | 2 | 0.138 | 0 of 41711 | 0 | 10 | 6 | 0.275 | 0.61 |
| ekf_kerb, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb2, no re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb2, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb2, R6 re-join, multipath_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.6 | - | 0.193 | 0.052 | 0 | 0.000 | 0 of 43825 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf, no re-join, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 89.7 | - | 1.500 | -4.984 | 10 | 4.920 | 0 of 27815 | 0 | 6 | 0 | 2.531 | 6.81 |
| ekf, kerb_return_r7 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 89.7 | - | 1.500 | -4.984 | 10 | 4.920 | 0 of 27815 | 0 | 6 | 0 | 2.531 | 6.81 |
| ekf_int, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 89.7 | - | 1.500 | -4.984 | 10 | 4.920 | 0 of 29042 | 0 | 4 | 0 | 2.138 | 6.81 |
| ekf_int, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 89.7 | - | 1.500 | -4.984 | 10 | 4.920 | 0 of 29042 | 0 | 4 | 0 | 2.138 | 6.81 |
| ekf_int2, no re-join, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 89.7 | - | 1.500 | -3.991 | 10 | 3.928 | 0 of 29042 | 0 | 2 | 0 | 2.136 | 5.20 |
| ekf_int2, kerb_return_r7 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 89.7 | - | 1.500 | -3.991 | 10 | 3.928 | 0 of 29042 | 0 | 2 | 0 | 2.136 | 5.20 |
| ekf_kerb, no re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 89.8 | - | 1.500 | -5.671 | 10 | 5.607 | 0 of 32201 | 0 | 2 | 0 | 1.644 | 7.42 |
| ekf_kerb, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 89.8 | - | 1.500 | -5.671 | 10 | 5.607 | 0 of 32201 | 0 | 2 | 0 | 1.644 | 7.42 |
| ekf_kerb2, no re-join, kerb_return_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 90.0 | - | 1.500 | -3.736 | 10 | 3.673 | 0 of 30491 | 0 | 0 | 0 | 1.644 | 5.48 |
| ekf_kerb2, kerb_return_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 90.0 | - | 1.500 | -3.736 | 10 | 3.673 | 0 of 30491 | 0 | 0 | 0 | 1.644 | 5.48 |
| ekf_kerb, R6 re-join, kerb_return_r7 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 89.8 | - | 1.500 | -5.671 | 10 | 5.607 | 0 of 32201 | 0 | 2 | 0 | 1.644 | 7.42 |
| ekf_kerb2, R6 re-join, kerb_return_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 90.0 | - | 1.500 | -3.736 | 10 | 3.673 | 0 of 30491 | 0 | 0 | 0 | 1.644 | 5.48 |
| ekf, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.627 | 10 | 6.564 | 0 of 11502 | 0 | 10 | 0 | 2.976 | 8.80 |
| ekf, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.627 | 10 | 6.564 | 0 of 11502 | 0 | 10 | 0 | 2.976 | 8.80 |
| ekf_int, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.627 | 10 | 6.564 | 0 of 11502 | 0 | 10 | 0 | 2.977 | 8.80 |
| ekf_int, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.627 | 10 | 6.564 | 0 of 11502 | 0 | 10 | 0 | 2.977 | 8.80 |
| ekf_int2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.677 | 10 | 4.614 | 0 of 11502 | 0 | 8 | 0 | 1.566 | 6.74 |
| ekf_int2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.677 | 10 | 4.614 | 0 of 11502 | 0 | 8 | 0 | 1.566 | 6.74 |
| ekf_kerb, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -19.221 | 10 | 19.160 | 0 of 11537 | 0 | 11 | 0 | 8.448 | 21.01 |
| ekf_kerb, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -19.221 | 10 | 19.160 | 0 of 11537 | 0 | 11 | 0 | 8.448 | 21.01 |
| ekf_kerb2, no re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.214 | 10 | 5.150 | 0 of 11537 | 0 | 5 | 0 | 1.662 | 7.95 |
| ekf_kerb2, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.214 | 10 | 5.150 | 0 of 11537 | 0 | 5 | 0 | 1.662 | 7.95 |
| ekf_kerb, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -19.274 | 10 | 19.214 | 0 of 11537 | 0 | 11 | 0 | 8.461 | 21.06 |
| ekf_kerb2, R6 re-join, imu_fault_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.631 | 10 | 5.568 | 0 of 11537 | 0 | 5 | 0 | 1.662 | 7.97 |
| ekf, no re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.498 | -3.817 | 8 | 3.753 | 69 of 34348 | 0 | 4 | 0 | 1.253 | 4.77 |
| ekf, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.498 | -3.817 | 8 | 3.753 | 69 of 34348 | 0 | 4 | 0 | 1.253 | 4.77 |
| ekf_int, no re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.498 | -3.817 | 9 | 3.753 | 69 of 34212 | 0 | 1 | 0 | 1.293 | 4.09 |
| ekf_int, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.498 | -3.817 | 9 | 3.753 | 69 of 34212 | 0 | 1 | 0 | 1.293 | 4.09 |
| ekf_int2, no re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.500 | -3.817 | 9 | 3.753 | 69 of 34224 | 0 | 1 | 0 | 1.293 | 4.09 |
| ekf_int2, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.500 | -3.817 | 9 | 3.753 | 69 of 34224 | 0 | 1 | 0 | 1.293 | 4.09 |
| ekf_kerb, no re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 63 of 36980 | 0 | 0 | 0 | 1.191 | 4.08 |
| ekf_kerb, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 63 of 36981 | 0 | 0 | 0 | 1.191 | 4.08 |
| ekf_kerb2, no re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 37 of 34031 | 0 | 0 | 0 | 1.181 | 4.08 |
| ekf_kerb2, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 37 of 34036 | 0 | 0 | 0 | 1.181 | 4.08 |
| ekf_kerb, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 63 of 36981 | 0 | 0 | 0 | 1.191 | 4.08 |
| ekf_kerb2, R6 re-join, kerb_faults_r7 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 89.7 | - | 1.500 | -3.898 | 8 | 3.834 | 37 of 34036 | 0 | 0 | 0 | 1.181 | 4.08 |
| ekf, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf_int, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf_int, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf_int2, no re-join, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf_int2, combined_r7 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.202 | 2 | 0.138 | 0 of 32228 | 0 | 10 | 4 | 0.360 | 0.72 |
| ekf_kerb, no re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43861 | 0 | 0 | 0 | 0.129 | 0.61 |
| ekf_kerb, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43861 | 0 | 0 | 0 | 0.129 | 0.61 |
| ekf_kerb2, no re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43878 | 0 | 0 | 0 | 0.129 | 0.61 |
| ekf_kerb2, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43878 | 0 | 0 | 0 | 0.129 | 0.61 |
| ekf_kerb, R6 re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43861 | 0 | 0 | 0 | 0.129 | 0.61 |
| ekf_kerb2, R6 re-join, combined_r7 | 10 | COMPLETED 10 | 10 of 10 | 89.7 | - | 0.193 | 0.059 | 0 | 0.000 | 0 of 43878 | 0 | 0 | 0 | 0.129 | 0.61 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
