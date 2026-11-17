# Closed-loop driving on estimated state: held-out evaluation of the second versions (network R4)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260.

## R4-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 73.7 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 35840 | 0 | 0 | 0 | - | - |
| ekf, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.662 | -0.202 | 2 | 0.138 | 0 of 35705 | 0 | 0 | 0 | 0.177 | 0.61 |
| ekf_int, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.662 | -0.202 | 2 | 0.138 | 0 of 35705 | 0 | 0 | 0 | 0.177 | 0.61 |
| ekf_int2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.662 | -0.202 | 2 | 0.138 | 0 of 35705 | 0 | 0 | 0 | 0.177 | 0.61 |
| ekf_kerb, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 35659 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf_kerb2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 35659 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 35333 | 0 | 10 | 6 | 0.269 | 0.93 |
| ekf_int, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 35333 | 0 | 10 | 6 | 0.269 | 0.93 |
| ekf_int2, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 35333 | 0 | 10 | 6 | 0.269 | 0.93 |
| ekf_kerb, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.063 | 0 | 0.000 | 0 of 35663 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf_kerb2, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.063 | 0 | 0.000 | 0 of 35663 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.867 | -0.202 | 2 | 0.138 | 0 of 17685 | 0 | 10 | 7 | 3.393 | 12.38 |
| ekf_int, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.867 | -0.202 | 2 | 0.138 | 0 of 38904 | 0 | 10 | 7 | 0.233 | 0.62 |
| ekf_int2, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.867 | -0.202 | 2 | 0.138 | 0 of 38904 | 0 | 10 | 7 | 0.233 | 0.62 |
| ekf_kerb, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.065 | 0 | 0.000 | 0 of 35664 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf_kerb2, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 0.193 | 0.065 | 0 | 0.000 | 0 of 35623 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -6.997 | 10 | 6.943 | 238 of 13292 | 0 | 11 | 0 | 3.280 | 9.68 |
| ekf_int, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.623 | 10 | 5.559 | 238 of 13292 | 0 | 7 | 0 | 3.277 | 9.80 |
| ekf_int2, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.160 | 10 | 5.102 | 176 of 13230 | 0 | 3 | 0 | 3.132 | 7.15 |
| ekf_kerb, gnss_ramp_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.063 | 0 | 0.000 | 0 of 35644 | 0 | 0 | 0 | 0.082 | 0.61 |
| ekf_kerb2, gnss_ramp_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.063 | 0 | 0.000 | 0 of 35644 | 0 | 0 | 0 | 0.082 | 0.61 |
| ekf, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -10.987 | 4 | 10.926 | 0 of 13565 | 0 | 11 | 4 | 2.258 | 13.96 |
| ekf_int, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -1.065 | 8 | 1.001 | 0 of 13538 | 0 | 10 | 4 | 1.801 | 5.89 |
| ekf_int2, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -1.065 | 8 | 1.001 | 0 of 13538 | 0 | 10 | 4 | 1.801 | 5.89 |
| ekf_kerb, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.219 | 0.059 | 0 | 0.000 | 0 of 35723 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf_kerb2, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.219 | 0.059 | 0 | 0.000 | 0 of 35720 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.020 | -0.202 | 2 | 0.138 | 0 of 25881 | 0 | 10 | 8 | 1.817 | 9.14 |
| ekf_int, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.020 | -0.202 | 2 | 0.138 | 0 of 46881 | 0 | 10 | 8 | 0.296 | 0.61 |
| ekf_int2, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.020 | -0.202 | 2 | 0.138 | 0 of 46881 | 0 | 10 | 8 | 0.296 | 0.61 |
| ekf_kerb, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.817 | 0.139 | 0 | 0.000 | 0 of 46869 | 0 | 10 | 9 | 0.186 | 0.75 |
| ekf_kerb2, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.817 | 0.139 | 0 | 0.000 | 0 of 46869 | 0 | 10 | 9 | 0.186 | 0.75 |
| ekf, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.088 | -0.202 | 2 | 0.138 | 0 of 30679 | 0 | 10 | 5 | 0.248 | 0.72 |
| ekf_int, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.088 | -0.202 | 2 | 0.138 | 0 of 30679 | 0 | 10 | 5 | 0.248 | 0.72 |
| ekf_int2, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.088 | -0.202 | 2 | 0.138 | 0 of 30679 | 0 | 10 | 5 | 0.248 | 0.72 |
| ekf_kerb, combined_r4 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.2 | - | 1.498 | -3.967 | 2 | 3.903 | 0 of 33308 | 0 | 0 | 0 | 0.123 | 4.13 |
| ekf_kerb2, combined_r4 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 0.556 | 0.099 | 0 | 0.000 | 0 of 35617 | 0 | 0 | 0 | 0.123 | 0.91 |

## R4-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 61.7 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 29860 | 0 | 0 | 0 | - | - |
| ekf, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.544 | -0.097 | 1 | 0.033 | 0 of 29731 | 0 | 0 | 0 | 0.197 | 0.66 |
| ekf_int, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.544 | -0.097 | 1 | 0.033 | 0 of 29731 | 0 | 0 | 0 | 0.197 | 0.66 |
| ekf_int2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.544 | -0.097 | 1 | 0.033 | 0 of 29731 | 0 | 0 | 0 | 0.197 | 0.66 |
| ekf_kerb, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.048 | 0 | 0.000 | 0 of 29748 | 0 | 0 | 0 | 0.093 | 0.66 |
| ekf_kerb2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.048 | 0 | 0.000 | 0 of 29748 | 0 | 0 | 0 | 0.093 | 0.66 |
| ekf, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_int, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_int2, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_kerb, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 29747 | 0 | 0 | 0 | 0.089 | 0.66 |
| ekf_kerb2, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 29747 | 0 | 0 | 0 | 0.089 | 0.66 |
| ekf, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | 0.001 | 0 | 0.000 | 11 of 18054 | 0 | 11 | 7 | 2.939 | 14.09 |
| ekf_int, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.924 | 0.001 | 0 | 0.000 | 0 of 38960 | 0 | 10 | 7 | 0.256 | 0.71 |
| ekf_int2, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.924 | 0.001 | 0 | 0.000 | 0 of 38960 | 0 | 10 | 7 | 0.256 | 0.71 |
| ekf_kerb, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 29750 | 0 | 0 | 0 | 0.090 | 0.66 |
| ekf_kerb2, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 29751 | 0 | 0 | 0 | 0.090 | 0.66 |
| ekf, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -7.374 | 9 | 7.310 | 145 of 14301 | 0 | 10 | 0 | 2.943 | 9.98 |
| ekf_int, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -7.374 | 10 | 7.310 | 180 of 13870 | 0 | 7 | 0 | 3.033 | 9.98 |
| ekf_int2, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.856 | 10 | 6.792 | 180 of 13575 | 0 | 5 | 0 | 2.927 | 7.85 |
| ekf_kerb, gnss_ramp_r4 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 61.5 | - | 1.356 | -2.116 | 1 | 2.055 | 0 of 28876 | 0 | 1 | 0 | 0.155 | 4.79 |
| ekf_kerb2, gnss_ramp_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 29716 | 0 | 0 | 0 | 0.155 | 1.06 |
| ekf, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 2.069 | 8.09 |
| ekf_int, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 1.634 | 5.90 |
| ekf_int2, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 1.634 | 5.90 |
| ekf_kerb, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.4 | - | 0.282 | 0.159 | 0 | 0.000 | 0 of 29694 | 0 | 0 | 0 | 0.096 | 0.66 |
| ekf_kerb2, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.4 | - | 0.282 | 0.159 | 0 | 0.000 | 0 of 29695 | 0 | 0 | 0 | 0.096 | 0.66 |
| ekf, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.819 | -0.002 | 0 | 0.000 | 0 of 24565 | 0 | 10 | 5 | 1.630 | 8.93 |
| ekf_int, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.819 | -0.002 | 0 | 0.000 | 0 of 45565 | 0 | 10 | 5 | 0.244 | 0.71 |
| ekf_int2, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.819 | -0.002 | 0 | 0.000 | 0 of 45565 | 0 | 10 | 5 | 0.244 | 0.71 |
| ekf_kerb, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.947 | 0.050 | 0 | 0.000 | 0 of 45755 | 0 | 10 | 9 | 0.171 | 0.66 |
| ekf_kerb2, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.947 | 0.050 | 0 | 0.000 | 0 of 45755 | 0 | 10 | 9 | 0.171 | 0.66 |
| ekf, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_int, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_int2, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_kerb, combined_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.6 | - | 0.193 | 0.050 | 0 | 0.000 | 0 of 29773 | 0 | 0 | 0 | 0.101 | 0.66 |
| ekf_kerb2, combined_r4 | 10 | COMPLETED 10 | 10 of 10 | 61.6 | - | 0.193 | 0.050 | 0 | 0.000 | 0 of 29773 | 0 | 0 | 0 | 0.101 | 0.66 |

## R4-B_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.81 | 0.193 | 0.029 | 0 | 0.000 | 0 of 47338 | 0 | 0 | 0 | - | - |
| ekf, nominal_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.544 | -0.060 | 0 | 0.000 | 0 of 46999 | 0 | 0 | 0 | 0.170 | 0.66 |
| ekf_int, nominal_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.544 | -0.060 | 0 | 0.000 | 0 of 46999 | 0 | 0 | 0 | 0.170 | 0.66 |
| ekf_int2, nominal_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.544 | -0.060 | 0 | 0.000 | 0 of 46999 | 0 | 0 | 0 | 0.170 | 0.66 |
| ekf_kerb, nominal_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.193 | 0.048 | 0 | 0.000 | 0 of 47089 | 0 | 0 | 0 | 0.083 | 0.66 |
| ekf_kerb2, nominal_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.193 | 0.048 | 0 | 0.000 | 0 of 47089 | 0 | 0 | 0 | 0.083 | 0.66 |
| ekf, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_int, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_int2, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | 0.001 | 0 | 0.000 | 0 of 36283 | 0 | 10 | 5 | 0.360 | 1.42 |
| ekf_kerb, gnss_outage_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.193 | 0.046 | 0 | 0.000 | 0 of 47038 | 0 | 0 | 0 | 0.085 | 0.66 |
| ekf_kerb2, gnss_outage_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.193 | 0.046 | 0 | 0.000 | 0 of 47038 | 0 | 0 | 0 | 0.085 | 0.66 |
| ekf, multipath_r4 | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | 1 of 10 | - | 27.53 | 1.499 | 0.001 | 0 | 0.000 | 11 of 18054 | 0 | 12 | 7 | 2.939 | 14.09 |
| ekf_int, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.924 | 0.001 | 0 | 0.000 | 0 of 38960 | 0 | 10 | 7 | 0.256 | 0.71 |
| ekf_int2, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.924 | 0.001 | 0 | 0.000 | 0 of 38960 | 0 | 10 | 7 | 0.256 | 0.71 |
| ekf_kerb, multipath_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.193 | 0.046 | 0 | 0.000 | 0 of 47046 | 0 | 0 | 0 | 0.089 | 0.66 |
| ekf_kerb2, multipath_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.75 | 0.193 | 0.046 | 0 | 0.000 | 0 of 47030 | 0 | 0 | 0 | 0.090 | 0.66 |
| ekf, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -7.374 | 9 | 7.310 | 145 of 14301 | 0 | 10 | 0 | 2.943 | 9.98 |
| ekf_int, gnss_ramp_r4 | 10 | CLOSURE_INCURSION 1, STOPPED_EN_ROUTE 9 | 0 of 10 | - | - | 1.500 | -7.374 | 10 | 7.310 | 180 of 13870 | 0 | 7 | 0 | 3.033 | 9.98 |
| ekf_int2, gnss_ramp_r4 | 10 | CLOSURE_INCURSION 1, STOPPED_EN_ROUTE 9 | 0 of 10 | - | - | 1.500 | -6.856 | 10 | 6.792 | 180 of 13575 | 0 | 5 | 0 | 2.927 | 7.85 |
| ekf_kerb, gnss_ramp_r4 | 10 | STOPPED_AT_CLOSURE 9, STOPPED_EN_ROUTE 1 | 9 of 10 | - | 1.74 | 1.356 | -2.116 | 1 | 2.055 | 0 of 44419 | 0 | 1 | 0 | 0.136 | 4.79 |
| ekf_kerb2, gnss_ramp_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.74 | 0.193 | 0.046 | 0 | 0.000 | 0 of 46984 | 0 | 0 | 0 | 0.136 | 1.06 |
| ekf, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 2.069 | 8.09 |
| ekf_int, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 1.634 | 5.90 |
| ekf_int2, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -0.890 | 10 | 0.826 | 0 of 13396 | 0 | 10 | 7 | 1.634 | 5.90 |
| ekf_kerb, imu_fault_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.282 | 0.159 | 0 | 0.000 | 0 of 47008 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf_kerb2, imu_fault_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.282 | 0.159 | 0 | 0.000 | 0 of 47007 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf, kerb_faults_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 16.10 | 0.819 | -0.002 | 0 | 0.000 | 0 of 24565 | 0 | 10 | 5 | 1.630 | 8.93 |
| ekf_int, kerb_faults_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 16.10 | 0.819 | -0.002 | 0 | 0.000 | 0 of 45565 | 0 | 10 | 5 | 0.244 | 0.71 |
| ekf_int2, kerb_faults_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 16.10 | 0.819 | -0.002 | 0 | 0.000 | 0 of 45565 | 0 | 10 | 5 | 0.244 | 0.71 |
| ekf_kerb, kerb_faults_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 15.41 | 0.947 | 0.050 | 0 | 0.000 | 0 of 45755 | 0 | 10 | 9 | 0.171 | 0.66 |
| ekf_kerb2, kerb_faults_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 15.41 | 0.947 | 0.050 | 0 | 0.000 | 0 of 45755 | 0 | 10 | 9 | 0.171 | 0.66 |
| ekf, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_int, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_int2, combined_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.427 | 0.001 | 0 | 0.000 | 0 of 31240 | 0 | 10 | 6 | 0.282 | 0.69 |
| ekf_kerb, combined_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.74 | 0.193 | 0.050 | 0 | 0.000 | 0 of 47048 | 0 | 0 | 0 | 0.093 | 0.66 |
| ekf_kerb2, combined_r4 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.74 | 0.193 | 0.050 | 0 | 0.000 | 0 of 47048 | 0 | 0 | 0 | 0.093 | 0.66 |

## R4-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 64.4 | - | 0.193 | 0.028 | 0 | 0.000 | 0 of 31245 | 0 | 0 | 0 | - | - |
| ekf, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.3 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 31111 | 0 | 0 | 0 | 0.174 | 0.64 |
| ekf_int, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.3 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 31111 | 0 | 0 | 0 | 0.174 | 0.64 |
| ekf_int2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.3 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 31111 | 0 | 0 | 0 | 0.174 | 0.64 |
| ekf_kerb, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 31092 | 0 | 0 | 0 | 0.071 | 0.64 |
| ekf_kerb2, nominal_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 31092 | 0 | 0 | 0 | 0.071 | 0.64 |
| ekf, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.469 | -0.178 | 1 | 0.114 | 0 of 35362 | 0 | 10 | 5 | 0.311 | 0.97 |
| ekf_int, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.469 | -0.178 | 1 | 0.114 | 0 of 35362 | 0 | 10 | 5 | 0.311 | 0.97 |
| ekf_int2, gnss_outage_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.469 | -0.178 | 1 | 0.114 | 0 of 35362 | 0 | 10 | 5 | 0.311 | 0.97 |
| ekf_kerb, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.065 | 0 | 0.000 | 0 of 31089 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf_kerb2, gnss_outage_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.065 | 0 | 0.000 | 0 of 31089 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf, multipath_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.495 | -0.178 | 1 | 0.114 | 0 of 16756 | 0 | 10 | 6 | 3.760 | 13.41 |
| ekf_int, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 67.9 | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 32557 | 0 | 10 | 6 | 0.245 | 1.00 |
| ekf_int2, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 67.9 | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 32642 | 0 | 10 | 6 | 0.245 | 1.00 |
| ekf_kerb, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.1 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 31068 | 0 | 0 | 0 | 0.079 | 0.64 |
| ekf_kerb2, multipath_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.064 | 0 | 0.000 | 0 of 31088 | 0 | 0 | 0 | 0.079 | 0.64 |
| ekf, gnss_ramp_r4 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 65.6 | - | 1.500 | -5.479 | 10 | 5.415 | 629 of 14752 | 0 | 11 | 0 | 3.274 | 6.86 |
| ekf_int, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.315 | 10 | 5.252 | 762 of 16636 | 0 | 11 | 0 | 4.104 | 7.28 |
| ekf_int2, gnss_ramp_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.896 | 9 | 4.833 | 656 of 18112 | 0 | 11 | 0 | 3.282 | 6.41 |
| ekf_kerb, gnss_ramp_r4 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 64.1 | - | 1.362 | 0.058 | 0 | 0.000 | 0 of 30279 | 0 | 0 | 0 | 0.119 | 3.24 |
| ekf_kerb2, gnss_ramp_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.2 | - | 0.193 | 0.058 | 0 | 0.000 | 0 of 31086 | 0 | 0 | 0 | 0.119 | 1.00 |
| ekf, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.356 | -0.178 | 1 | 0.114 | 0 of 15133 | 0 | 10 | 5 | 1.901 | 6.94 |
| ekf_int, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.356 | -0.178 | 1 | 0.114 | 0 of 15133 | 0 | 10 | 5 | 1.615 | 6.04 |
| ekf_int2, imu_fault_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.356 | -0.178 | 1 | 0.114 | 0 of 15133 | 0 | 10 | 5 | 1.615 | 6.04 |
| ekf_kerb, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.1 | - | 0.220 | 0.064 | 0 | 0.000 | 0 of 31059 | 0 | 0 | 0 | 0.079 | 0.64 |
| ekf_kerb2, imu_fault_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.1 | - | 0.220 | 0.064 | 0 | 0.000 | 0 of 31061 | 0 | 0 | 0 | 0.080 | 0.64 |
| ekf, kerb_faults_r4 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.489 | -5.587 | 2 | 5.523 | 0 of 22402 | 0 | 1 | 0 | 2.337 | 8.77 |
| ekf_int, kerb_faults_r4 | 10 | COMPLETED 10 | 10 of 10 | 66.7 | - | 1.005 | -0.178 | 1 | 0.114 | 0 of 32341 | 0 | 10 | 8 | 0.190 | 0.64 |
| ekf_int2, kerb_faults_r4 | 10 | COMPLETED 10 | 10 of 10 | 66.8 | - | 1.018 | -0.178 | 1 | 0.114 | 0 of 32363 | 0 | 10 | 8 | 0.189 | 0.64 |
| ekf_kerb, kerb_faults_r4 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 66.8 | - | 1.498 | 0.064 | 0 | 0.000 | 0 of 31517 | 0 | 10 | 7 | 0.121 | 0.91 |
| ekf_kerb2, kerb_faults_r4 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 66.9 | - | 1.498 | 0.063 | 0 | 0.000 | 0 of 30781 | 0 | 10 | 7 | 0.123 | 0.92 |
| ekf, combined_r4 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 68.3 | - | 1.500 | -7.708 | 7 | 7.664 | 377 of 23373 | 0 | 12 | 4 | 2.033 | 10.58 |
| ekf_int, combined_r4 | 10 | COMPLETED 3, STOPPED_EN_ROUTE 7 | 3 of 10 | 67.7 | - | 1.500 | -7.708 | 7 | 7.664 | 224 of 23883 | 0 | 12 | 4 | 1.920 | 10.70 |
| ekf_int2, combined_r4 | 10 | COMPLETED 3, STOPPED_EN_ROUTE 7 | 3 of 10 | 67.7 | - | 1.500 | -3.438 | 7 | 3.374 | 224 of 23883 | 0 | 12 | 4 | 1.920 | 4.77 |
| ekf_kerb, combined_r4 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 64.0 | - | 1.497 | -1.583 | 1 | 1.519 | 43 of 26996 | 0 | 0 | 0 | 0.158 | 3.84 |
| ekf_kerb2, combined_r4 | 10 | COMPLETED 10 | 10 of 10 | 64.0 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 30975 | 0 | 0 | 0 | 0.158 | 2.30 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
