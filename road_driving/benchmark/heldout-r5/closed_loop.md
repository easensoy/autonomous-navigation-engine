# Closed-loop driving on estimated state: held-out evaluation of the re-join after pose corrections (network R5)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. "no re-join" is the stack evaluated on R3 and R4: the clearance layer without the re-join after pose corrections, its monitor measuring from the unshifted path. Every other row uses the re-join; nothing jumps on simulator truth.

## R5-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 65.6 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 31845 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf_int, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf_int, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf_int2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf_int2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.4 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 31703 | 0 | 0 | 0 | 0.180 | 0.61 |
| ekf_kerb, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31680 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf_kerb, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31680 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf_kerb2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31680 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf_kerb2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31680 | 0 | 0 | 0 | 0.068 | 0.61 |
| ekf, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 30918 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 37218 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf_int, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 30918 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf_int, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 37218 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf_int2, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 30918 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf_int2, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.462 | -0.202 | 2 | 0.138 | 0 of 37218 | 0 | 10 | 5 | 0.331 | 0.98 |
| ekf_kerb, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31668 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31668 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31668 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31668 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 16902 | 0 | 10 | 4 | 2.339 | 12.01 |
| ekf, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 16902 | 0 | 10 | 4 | 2.339 | 12.01 |
| ekf_int, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 37902 | 0 | 10 | 4 | 0.225 | 0.86 |
| ekf_int, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 37902 | 0 | 10 | 4 | 0.225 | 0.86 |
| ekf_int2, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 37902 | 0 | 10 | 4 | 0.225 | 0.86 |
| ekf_int2, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.200 | -0.202 | 2 | 0.138 | 0 of 37902 | 0 | 10 | 4 | 0.225 | 0.86 |
| ekf_kerb, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31678 | 0 | 0 | 0 | 0.063 | 0.61 |
| ekf_kerb, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31678 | 0 | 0 | 0 | 0.063 | 0.61 |
| ekf_kerb2, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31678 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf_kerb2, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 31678 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf, no re-join, gnss_ramp_r5 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.6 | - | 1.500 | -5.835 | 9 | 5.771 | 448 of 20082 | 0 | 8 | 0 | 2.932 | 6.75 |
| ekf, gnss_ramp_r5 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.6 | - | 1.500 | -5.835 | 9 | 5.771 | 448 of 20078 | 0 | 8 | 0 | 2.932 | 6.75 |
| ekf_int, no re-join, gnss_ramp_r5 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 65.7 | - | 1.500 | -5.836 | 9 | 5.772 | 271 of 19558 | 0 | 4 | 0 | 2.489 | 6.76 |
| ekf_int, gnss_ramp_r5 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 65.3 | - | 1.500 | -5.836 | 9 | 5.772 | 271 of 18814 | 0 | 4 | 0 | 2.573 | 6.76 |
| ekf_int2, no re-join, gnss_ramp_r5 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 65.7 | - | 1.500 | -4.488 | 9 | 4.424 | 271 of 19383 | 0 | 3 | 0 | 2.476 | 5.67 |
| ekf_int2, gnss_ramp_r5 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 65.3 | - | 1.500 | -4.488 | 9 | 4.424 | 271 of 18639 | 0 | 3 | 0 | 2.476 | 5.67 |
| ekf_kerb, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.2 | - | 0.193 | 0.058 | 0 | 0.000 | 0 of 31635 | 0 | 0 | 0 | 0.112 | 1.10 |
| ekf_kerb, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.2 | - | 0.193 | 0.058 | 0 | 0.000 | 0 of 31635 | 0 | 0 | 0 | 0.112 | 1.10 |
| ekf_kerb2, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.2 | - | 0.193 | 0.057 | 0 | 0.000 | 0 of 31637 | 0 | 0 | 0 | 0.112 | 0.93 |
| ekf_kerb2, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.2 | - | 0.193 | 0.057 | 0 | 0.000 | 0 of 31637 | 0 | 0 | 0 | 0.112 | 0.93 |
| ekf, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf_int, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf_int, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf_int2, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf_int2, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.202 | 2 | 0.138 | 0 of 17709 | 0 | 10 | 0 | 1.266 | 4.97 |
| ekf_kerb, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.038 | 0 | 0.000 | 0 of 31693 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf_kerb, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.038 | 0 | 0.000 | 0 of 31693 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf_kerb2, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.038 | 0 | 0.000 | 0 of 31693 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf_kerb2, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.038 | 0 | 0.000 | 0 of 31693 | 0 | 0 | 0 | 0.064 | 0.61 |
| ekf, no re-join, kerb_faults_r5 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 81.4 | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 22721 | 0 | 11 | 7 | 1.722 | 8.50 |
| ekf, kerb_faults_r5 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 81.4 | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 22723 | 0 | 11 | 7 | 1.722 | 8.50 |
| ekf_int, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 41829 | 0 | 10 | 7 | 0.172 | 0.61 |
| ekf_int, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 41829 | 0 | 10 | 7 | 0.172 | 0.61 |
| ekf_int2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 41829 | 0 | 10 | 7 | 0.172 | 0.61 |
| ekf_int2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.919 | -0.202 | 2 | 0.138 | 0 of 41829 | 0 | 10 | 7 | 0.172 | 0.61 |
| ekf_kerb, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.578 | 0.152 | 0 | 0.000 | 0 of 41833 | 0 | 10 | 10 | 0.086 | 0.61 |
| ekf_kerb, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.578 | 0.152 | 0 | 0.000 | 0 of 41833 | 0 | 10 | 10 | 0.086 | 0.61 |
| ekf_kerb2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.578 | 0.152 | 0 | 0.000 | 0 of 41833 | 0 | 10 | 10 | 0.086 | 0.61 |
| ekf_kerb2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.578 | 0.152 | 0 | 0.000 | 0 of 41833 | 0 | 10 | 10 | 0.086 | 0.61 |
| ekf, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -10.295 | 9 | 10.246 | 122 of 22378 | 0 | 18 | 5 | 3.129 | 13.57 |
| ekf, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -10.290 | 10 | 10.241 | 122 of 24384 | 0 | 19 | 5 | 3.117 | 13.56 |
| ekf_int, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -10.295 | 8 | 10.246 | 122 of 22412 | 0 | 17 | 5 | 3.808 | 14.96 |
| ekf_int, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -10.290 | 9 | 10.241 | 122 of 24111 | 0 | 18 | 5 | 3.803 | 14.96 |
| ekf_int2, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.544 | 8 | 3.480 | 122 of 22184 | 0 | 15 | 5 | 1.621 | 6.43 |
| ekf_int2, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -3.545 | 9 | 3.481 | 122 of 23883 | 0 | 16 | 5 | 1.620 | 6.42 |
| ekf_kerb, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.082 | 0 | 0.000 | 0 of 31642 | 0 | 0 | 0 | 0.110 | 0.61 |
| ekf_kerb, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.082 | 0 | 0.000 | 0 of 31642 | 0 | 0 | 0 | 0.110 | 0.61 |
| ekf_kerb2, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.082 | 0 | 0.000 | 0 of 31642 | 0 | 0 | 0 | 0.110 | 0.61 |
| ekf_kerb2, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 65.3 | - | 0.193 | 0.082 | 0 | 0.000 | 0 of 31642 | 0 | 0 | 0 | 0.110 | 0.61 |

## R5-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 71.5 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34740 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf_int, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf_int, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf_int2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf_int2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34524 | 0 | 0 | 0 | 0.189 | 0.64 |
| ekf_kerb, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34505 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34505 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34505 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34505 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int2, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int2, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_kerb, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.031 | 0 | 0.000 | 0 of 34515 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.031 | 0 | 0.000 | 0 of 34515 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb2, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.031 | 0 | 0.000 | 0 of 34515 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb2, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.031 | 0 | 0.000 | 0 of 34515 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 17100 | 0 | 10 | 3 | 2.797 | 10.97 |
| ekf, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 17100 | 0 | 10 | 3 | 2.797 | 10.97 |
| ekf_int, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int2, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int2, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_kerb, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.2 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34521 | 0 | 0 | 0 | 0.088 | 0.64 |
| ekf_kerb, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.2 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34521 | 0 | 0 | 0 | 0.088 | 0.64 |
| ekf_kerb2, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34533 | 0 | 0 | 0 | 0.088 | 0.64 |
| ekf_kerb2, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.1 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34533 | 0 | 0 | 0 | 0.088 | 0.64 |
| ekf, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -6.856 | 9 | 6.793 | 73 of 15741 | 0 | 10 | 0 | 3.106 | 7.90 |
| ekf, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -6.856 | 9 | 6.793 | 73 of 15499 | 0 | 10 | 0 | 3.106 | 7.90 |
| ekf_int, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -6.502 | 9 | 6.438 | 106 of 15944 | 0 | 4 | 0 | 2.703 | 7.86 |
| ekf_int, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -6.502 | 9 | 6.438 | 106 of 16125 | 0 | 3 | 0 | 2.703 | 8.06 |
| ekf_int2, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -4.670 | 9 | 4.606 | 106 of 15944 | 0 | 1 | 0 | 2.165 | 5.91 |
| ekf_int2, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -4.670 | 9 | 4.606 | 106 of 16125 | 0 | 0 | 0 | 2.165 | 5.91 |
| ekf_kerb, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 1.497 | 0.017 | 0 | 0.000 | 42 of 34400 | 0 | 0 | 0 | 0.131 | 2.47 |
| ekf_kerb, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 1.497 | 0.017 | 0 | 0.000 | 42 of 34399 | 0 | 0 | 0 | 0.131 | 2.47 |
| ekf_kerb2, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.017 | 0 | 0.000 | 0 of 34520 | 0 | 0 | 0 | 0.131 | 1.71 |
| ekf_kerb2, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.017 | 0 | 0.000 | 0 of 34520 | 0 | 0 | 0 | 0.131 | 1.71 |
| ekf, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int2, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int2, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_kerb, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34492 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf_kerb, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34492 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf_kerb2, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34492 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf_kerb2, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34492 | 0 | 0 | 0 | 0.082 | 0.64 |
| ekf, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.475 | -2.612 | 3 | 2.553 | 0 of 22730 | 0 | 11 | 7 | 1.717 | 8.01 |
| ekf, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.327 | -2.595 | 2 | 2.536 | 0 of 22601 | 0 | 11 | 7 | 1.647 | 8.01 |
| ekf_int, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 10 | 7 | 0.273 | 0.80 |
| ekf_int, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 10 | 7 | 0.273 | 0.80 |
| ekf_int2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 10 | 7 | 0.273 | 0.80 |
| ekf_int2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 10 | 7 | 0.273 | 0.80 |
| ekf_kerb, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 10 | 9 | 0.146 | 0.64 |
| ekf_kerb, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 10 | 9 | 0.146 | 0.64 |
| ekf_kerb2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 10 | 9 | 0.146 | 0.64 |
| ekf_kerb2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 10 | 9 | 0.146 | 0.64 |
| ekf, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int2, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int2, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_kerb, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34511 | 0 | 0 | 0 | 0.104 | 0.64 |
| ekf_kerb, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34511 | 0 | 0 | 0 | 0.104 | 0.64 |
| ekf_kerb2, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34511 | 0 | 0 | 0 | 0.104 | 0.64 |
| ekf_kerb2, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 71.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 34511 | 0 | 0 | 0 | 0.104 | 0.64 |

## R5-B_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.83 | 0.193 | 0.029 | 0 | 0.000 | 0 of 45051 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf_int, no re-join, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf_int, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf_int2, no re-join, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf_int2, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.61 | 0.576 | -0.178 | 1 | 0.114 | 0 of 44768 | 0 | 0 | 0 | 0.158 | 0.64 |
| ekf_kerb, no re-join, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44794 | 0 | 0 | 0 | 0.078 | 0.64 |
| ekf_kerb, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44794 | 0 | 0 | 0 | 0.078 | 0.64 |
| ekf_kerb2, no re-join, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44794 | 0 | 0 | 0 | 0.078 | 0.64 |
| ekf_kerb2, nominal_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44794 | 0 | 0 | 0 | 0.078 | 0.64 |
| ekf, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int2, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_int2, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.909 | -0.178 | 1 | 0.114 | 0 of 34788 | 0 | 10 | 8 | 0.232 | 0.64 |
| ekf_kerb, no re-join, gnss_outage_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.031 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb, gnss_outage_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.031 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb2, no re-join, gnss_outage_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.031 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf_kerb2, gnss_outage_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.031 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.092 | 0.64 |
| ekf, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 17100 | 0 | 10 | 3 | 2.797 | 10.97 |
| ekf, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 17100 | 0 | 10 | 3 | 2.797 | 10.97 |
| ekf_int, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int2, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_int2, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.585 | -0.178 | 1 | 0.114 | 0 of 38561 | 0 | 10 | 3 | 0.239 | 0.82 |
| ekf_kerb, no re-join, multipath_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.029 | 0 | 0.000 | 0 of 44805 | 0 | 0 | 0 | 0.095 | 0.64 |
| ekf_kerb, multipath_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.193 | 0.029 | 0 | 0.000 | 0 of 44805 | 0 | 0 | 0 | 0.095 | 0.64 |
| ekf_kerb2, no re-join, multipath_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.029 | 0 | 0.000 | 0 of 44782 | 0 | 0 | 0 | 0.095 | 0.64 |
| ekf_kerb2, multipath_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.68 | 0.193 | 0.029 | 0 | 0.000 | 0 of 44782 | 0 | 0 | 0 | 0.095 | 0.64 |
| ekf, no re-join, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_EN_ROUTE 8 | 0 of 10 | - | - | 1.498 | -6.856 | 9 | 6.793 | 73 of 17795 | 0 | 9 | 0 | 3.106 | 7.90 |
| ekf, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_EN_ROUTE 8 | 0 of 10 | - | - | 1.498 | -6.856 | 9 | 6.793 | 73 of 17553 | 0 | 9 | 0 | 3.106 | 7.90 |
| ekf_int, no re-join, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_AT_CLOSURE 4, STOPPED_EN_ROUTE 4 | 4 of 10 | - | 6.70 | 1.498 | -6.502 | 9 | 6.438 | 106 of 19833 | 0 | 4 | 0 | 3.673 | 7.86 |
| ekf_int, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_AT_CLOSURE 4, STOPPED_EN_ROUTE 4 | 4 of 10 | - | 6.70 | 1.500 | -6.502 | 9 | 6.438 | 106 of 20015 | 0 | 3 | 0 | 3.673 | 8.06 |
| ekf_int2, no re-join, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_AT_CLOSURE 5, STOPPED_EN_ROUTE 3 | 5 of 10 | - | 6.70 | 1.498 | -4.670 | 9 | 4.606 | 106 of 19833 | 0 | 1 | 0 | 2.165 | 5.91 |
| ekf_int2, gnss_ramp_r5 | 10 | CLOSURE_INCURSION 2, STOPPED_AT_CLOSURE 5, STOPPED_EN_ROUTE 3 | 5 of 10 | - | 6.70 | 1.500 | -4.670 | 9 | 4.606 | 106 of 20015 | 0 | 0 | 0 | 2.165 | 5.91 |
| ekf_kerb, no re-join, gnss_ramp_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 0.48 | 0.193 | 0.017 | 0 | 0.000 | 0 of 44840 | 0 | 0 | 0 | 0.122 | 2.12 |
| ekf_kerb, gnss_ramp_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 0.48 | 0.193 | 0.017 | 0 | 0.000 | 0 of 44840 | 0 | 0 | 0 | 0.122 | 2.12 |
| ekf_kerb2, no re-join, gnss_ramp_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.017 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.122 | 1.71 |
| ekf_kerb2, gnss_ramp_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.017 | 0 | 0.000 | 0 of 44829 | 0 | 0 | 0 | 0.122 | 1.71 |
| ekf, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int2, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_int2, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.393 | -0.178 | 1 | 0.114 | 0 of 18740 | 0 | 10 | 7 | 1.321 | 5.52 |
| ekf_kerb, no re-join, imu_fault_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.67 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44793 | 0 | 0 | 0 | 0.081 | 0.64 |
| ekf_kerb, imu_fault_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.67 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44793 | 0 | 0 | 0 | 0.081 | 0.64 |
| ekf_kerb2, no re-join, imu_fault_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.67 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44793 | 0 | 0 | 0 | 0.081 | 0.64 |
| ekf_kerb2, imu_fault_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.67 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44793 | 0 | 0 | 0 | 0.081 | 0.64 |
| ekf, no re-join, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 9, STOPPED_EN_ROUTE 1 | 9 of 10 | - | 8.24 | 1.475 | -0.429 | 2 | 0.368 | 0 of 24504 | 0 | 20 | 11 | 1.717 | 8.01 |
| ekf, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 8.27 | 0.576 | -0.178 | 2 | 0.114 | 0 of 24368 | 0 | 20 | 11 | 1.647 | 8.01 |
| ekf_int, no re-join, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 26.81 | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 20 | 11 | 0.273 | 0.80 |
| ekf_int, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 26.81 | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 20 | 11 | 0.273 | 0.80 |
| ekf_int2, no re-join, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 26.81 | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 20 | 11 | 0.273 | 0.80 |
| ekf_int2, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 26.81 | 0.576 | -0.178 | 1 | 0.114 | 0 of 42722 | 0 | 20 | 11 | 0.273 | 0.80 |
| ekf_kerb, no re-join, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 28.49 | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 20 | 17 | 0.146 | 0.64 |
| ekf_kerb, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 28.49 | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 20 | 17 | 0.146 | 0.64 |
| ekf_kerb2, no re-join, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 28.49 | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 20 | 17 | 0.146 | 0.64 |
| ekf_kerb2, kerb_faults_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 28.49 | 0.815 | 0.032 | 0 | 0.000 | 0 of 42737 | 0 | 20 | 17 | 0.146 | 0.64 |
| ekf, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int2, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_int2, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.395 | -0.178 | 1 | 0.114 | 0 of 34509 | 0 | 10 | 6 | 0.306 | 0.91 |
| ekf_kerb, no re-join, combined_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44827 | 0 | 0 | 0 | 0.102 | 0.64 |
| ekf_kerb, combined_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44827 | 0 | 0 | 0 | 0.102 | 0.64 |
| ekf_kerb2, no re-join, combined_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44827 | 0 | 0 | 0 | 0.102 | 0.64 |
| ekf_kerb2, combined_r5 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.033 | 0 | 0.000 | 0 of 44827 | 0 | 0 | 0 | 0.102 | 0.64 |

## R5-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 80.9 | - | 0.193 | 0.128 | 0 | 0.000 | 0 of 39410 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf_int, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf_int, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf_int2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf_int2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.7 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 39302 | 0 | 0 | 0 | 0.171 | 0.64 |
| ekf_kerb, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39243 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39243 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb2, no re-join, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39243 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb2, nominal_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39243 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf_int, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf_int, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf_int2, no re-join, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf_int2, gnss_outage_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.178 | 1 | 0.114 | 0 of 38218 | 0 | 11 | 4 | 0.318 | 0.92 |
| ekf_kerb, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39245 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39245 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, no re-join, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39245 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, gnss_outage_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39245 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.497 | -0.178 | 1 | 0.114 | 0 of 17443 | 0 | 11 | 4 | 3.153 | 12.44 |
| ekf, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.178 | 1 | 0.114 | 0 of 17369 | 0 | 11 | 4 | 3.153 | 12.44 |
| ekf_int, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.220 | -0.178 | 1 | 0.114 | 0 of 38711 | 0 | 10 | 4 | 0.236 | 0.90 |
| ekf_int, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.220 | -0.178 | 1 | 0.114 | 0 of 38711 | 0 | 10 | 4 | 0.236 | 0.90 |
| ekf_int2, no re-join, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.220 | -0.178 | 1 | 0.114 | 0 of 38711 | 0 | 10 | 4 | 0.236 | 0.90 |
| ekf_int2, multipath_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.220 | -0.178 | 1 | 0.114 | 0 of 38711 | 0 | 10 | 4 | 0.236 | 0.90 |
| ekf_kerb, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39225 | 0 | 0 | 0 | 0.087 | 0.64 |
| ekf_kerb, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39225 | 0 | 0 | 0 | 0.087 | 0.64 |
| ekf_kerb2, no re-join, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39236 | 0 | 0 | 0 | 0.087 | 0.64 |
| ekf_kerb2, multipath_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.4 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39236 | 0 | 0 | 0 | 0.087 | 0.64 |
| ekf, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.803 | 8 | 5.739 | 22 of 15497 | 0 | 10 | 0 | 3.053 | 7.84 |
| ekf, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -5.803 | 7 | 5.739 | 22 of 15347 | 0 | 10 | 0 | 3.053 | 7.84 |
| ekf_int, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.378 | 8 | 5.314 | 72 of 15119 | 0 | 4 | 0 | 2.738 | 7.84 |
| ekf_int, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.378 | 9 | 5.314 | 71 of 15917 | 0 | 4 | 0 | 2.739 | 7.84 |
| ekf_int2, no re-join, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.322 | 8 | 4.258 | 72 of 15119 | 0 | 1 | 0 | 2.065 | 5.81 |
| ekf_int2, gnss_ramp_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.322 | 9 | 4.258 | 71 of 15917 | 0 | 1 | 0 | 2.167 | 5.81 |
| ekf_kerb, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.078 | 0 | 0.000 | 0 of 39229 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.078 | 0 | 0.000 | 0 of 39229 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb2, no re-join, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.078 | 0 | 0.000 | 0 of 39229 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb2, gnss_ramp_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.078 | 0 | 0.000 | 0 of 39229 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf_int, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf_int, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf_int2, no re-join, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf_int2, imu_fault_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.178 | 1 | 0.114 | 0 of 15972 | 0 | 10 | 0 | 1.339 | 5.30 |
| ekf_kerb, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39249 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39249 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb2, no re-join, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39249 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf_kerb2, imu_fault_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.081 | 0 | 0.000 | 0 of 39249 | 0 | 0 | 0 | 0.085 | 0.64 |
| ekf, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.494 | -1.575 | 2 | 1.558 | 0 of 22211 | 0 | 11 | 4 | 1.639 | 8.63 |
| ekf, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.488 | -1.577 | 3 | 1.560 | 0 of 22950 | 0 | 12 | 4 | 1.639 | 8.63 |
| ekf_int, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.029 | -0.178 | 1 | 0.114 | 0 of 42493 | 0 | 10 | 4 | 0.287 | 0.64 |
| ekf_int, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.029 | -0.178 | 1 | 0.114 | 0 of 42493 | 0 | 10 | 4 | 0.287 | 0.64 |
| ekf_int2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.029 | -0.178 | 1 | 0.114 | 0 of 42493 | 0 | 10 | 4 | 0.287 | 0.64 |
| ekf_int2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.029 | -0.178 | 1 | 0.114 | 0 of 42493 | 0 | 10 | 4 | 0.287 | 0.64 |
| ekf_kerb, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | 0.177 | 0 | 0.000 | 0 of 40080 | 0 | 10 | 7 | 0.260 | 1.05 |
| ekf_kerb, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | 0.177 | 0 | 0.000 | 0 of 40080 | 0 | 10 | 7 | 0.260 | 1.05 |
| ekf_kerb2, no re-join, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | 0.177 | 0 | 0.000 | 0 of 42494 | 0 | 10 | 8 | 0.254 | 0.82 |
| ekf_kerb2, kerb_faults_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | 0.177 | 0 | 0.000 | 0 of 42494 | 0 | 10 | 8 | 0.254 | 0.82 |
| ekf, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf_int, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf_int, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf_int2, no re-join, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf_int2, combined_r5 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.227 | -0.178 | 1 | 0.114 | 0 of 34157 | 0 | 10 | 5 | 0.322 | 0.68 |
| ekf_kerb, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 39244 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 39244 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb2, no re-join, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 39244 | 0 | 0 | 0 | 0.110 | 0.64 |
| ekf_kerb2, combined_r5 | 10 | COMPLETED 10 | 10 of 10 | 80.5 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 39244 | 0 | 0 | 0 | 0.110 | 0.64 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
