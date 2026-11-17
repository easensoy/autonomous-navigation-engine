# Closed-loop driving on estimated state: held-out evaluation of the verified re-join after pose corrections (network R6)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. "no re-join" is the stack evaluated on R3 and R4. Every other row re-joins only corrections the estimator verified against the map (kerb registration), by the amount the map moved it; nothing jumps on simulator truth.

## R6-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 73.6 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 35830 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf_int, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf_int, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf_int2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf_int2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.522 | -0.202 | 2 | 0.138 | 0 of 35677 | 0 | 0 | 0 | 0.171 | 0.61 |
| ekf_kerb, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35680 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35680 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35680 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35680 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf_int, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf_int, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf_int2, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf_int2, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.269 | -0.202 | 2 | 0.138 | 0 of 39036 | 0 | 10 | 6 | 0.194 | 0.92 |
| ekf_kerb, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35697 | 0 | 0 | 0 | 0.071 | 0.61 |
| ekf_kerb, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35697 | 0 | 0 | 0 | 0.071 | 0.61 |
| ekf_kerb2, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35697 | 0 | 0 | 0 | 0.071 | 0.61 |
| ekf_kerb2, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35697 | 0 | 0 | 0 | 0.071 | 0.61 |
| ekf, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 16590 | 0 | 10 | 6 | 2.307 | 9.65 |
| ekf, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 16590 | 0 | 10 | 6 | 2.307 | 9.65 |
| ekf_int, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 37590 | 0 | 10 | 6 | 0.227 | 0.77 |
| ekf_int, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 37590 | 0 | 10 | 6 | 0.227 | 0.77 |
| ekf_int2, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 37590 | 0 | 10 | 6 | 0.227 | 0.77 |
| ekf_int2, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | -0.202 | 2 | 0.138 | 0 of 37590 | 0 | 10 | 6 | 0.227 | 0.77 |
| ekf_kerb, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35683 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35683 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf_kerb2, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.034 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.067 | 0.61 |
| ekf, no re-join, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.338 | 8 | 7.274 | 221 of 14770 | 0 | 11 | 0 | 3.298 | 8.15 |
| ekf, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -7.338 | 8 | 7.274 | 221 of 14770 | 0 | 11 | 0 | 3.298 | 8.15 |
| ekf_int, no re-join, gnss_ramp_r6 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.0 | - | 1.499 | -7.338 | 8 | 7.274 | 204 of 18411 | 0 | 12 | 0 | 3.746 | 8.15 |
| ekf_int, gnss_ramp_r6 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.0 | - | 1.499 | -7.338 | 8 | 7.274 | 204 of 18411 | 0 | 12 | 0 | 3.746 | 8.15 |
| ekf_int2, no re-join, gnss_ramp_r6 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.0 | - | 1.499 | -5.141 | 8 | 5.077 | 204 of 18392 | 0 | 7 | 0 | 2.290 | 5.65 |
| ekf_int2, gnss_ramp_r6 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 82.0 | - | 1.499 | -5.141 | 8 | 5.077 | 204 of 18392 | 0 | 7 | 0 | 2.290 | 5.65 |
| ekf_kerb, no re-join, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35687 | 0 | 0 | 0 | 0.169 | 1.21 |
| ekf_kerb, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35687 | 0 | 0 | 0 | 0.169 | 1.21 |
| ekf_kerb2, no re-join, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35689 | 0 | 0 | 0 | 0.169 | 1.08 |
| ekf_kerb2, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.027 | 0 | 0.000 | 0 of 35689 | 0 | 0 | 0 | 0.169 | 1.08 |
| ekf, no re-join, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 28654 | 0 | 3 | 0 | 1.438 | 5.58 |
| ekf, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 28654 | 0 | 3 | 0 | 1.438 | 5.58 |
| ekf_int, no re-join, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 27906 | 0 | 3 | 0 | 1.438 | 4.61 |
| ekf_int, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 27906 | 0 | 3 | 0 | 1.438 | 4.61 |
| ekf_int2, no re-join, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 27906 | 0 | 3 | 0 | 1.438 | 4.58 |
| ekf_int2, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.3 | - | 1.500 | -3.945 | 9 | 3.881 | 31 of 27906 | 0 | 3 | 0 | 1.438 | 4.58 |
| ekf_kerb, no re-join, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.3 | - | 1.500 | -3.899 | 9 | 3.835 | 0 of 30172 | 0 | 0 | 0 | 1.299 | 4.14 |
| ekf_kerb, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.2 | - | 1.500 | -3.899 | 8 | 3.835 | 0 of 30161 | 0 | 0 | 0 | 1.300 | 4.14 |
| ekf_kerb2, no re-join, kerb_return_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 74.0 | - | 1.500 | -3.899 | 7 | 3.835 | 0 of 20363 | 0 | 0 | 0 | 1.575 | 4.14 |
| ekf_kerb2, kerb_return_r6 | 10 | COMPLETED 4, STOPPED_EN_ROUTE 6 | 4 of 10 | 73.3 | - | 1.500 | -3.899 | 6 | 3.835 | 0 of 24294 | 0 | 0 | 0 | 1.495 | 4.14 |
| ekf, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.429 | 4.93 |
| ekf, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.429 | 4.93 |
| ekf_int, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.284 | 4.93 |
| ekf_int, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.284 | 4.93 |
| ekf_int2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.284 | 4.93 |
| ekf_int2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -2.534 | 10 | 2.471 | 0 of 13655 | 0 | 10 | 0 | 1.284 | 4.93 |
| ekf_kerb, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.297 | 10 | 6.241 | 0 of 13817 | 0 | 10 | 0 | 1.189 | 5.83 |
| ekf_kerb, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -11.274 | 10 | 11.210 | 0 of 13817 | 0 | 11 | 0 | 1.189 | 13.76 |
| ekf_kerb2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -6.297 | 10 | 6.241 | 0 of 13817 | 0 | 10 | 0 | 1.189 | 5.83 |
| ekf_kerb2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -3.782 | 10 | 3.718 | 0 of 13817 | 0 | 10 | 0 | 1.189 | 6.67 |
| ekf, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf_int, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf_int, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf_int2, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf_int2, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.161 | -0.202 | 2 | 0.138 | 0 of 33727 | 0 | 10 | 6 | 0.316 | 0.69 |
| ekf_kerb, no re-join, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 1.495 | -3.350 | 1 | 3.286 | 0 of 35032 | 0 | 0 | 0 | 0.104 | 3.75 |
| ekf_kerb, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 1.495 | -3.350 | 1 | 3.286 | 0 of 35031 | 0 | 0 | 0 | 0.104 | 3.75 |
| ekf_kerb2, no re-join, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.070 | 0 | 0.000 | 0 of 35635 | 0 | 0 | 0 | 0.104 | 0.62 |
| ekf_kerb2, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.070 | 0 | 0.000 | 0 of 35635 | 0 | 0 | 0 | 0.104 | 0.62 |

## R6-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 71.9 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34970 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf_int, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf_int, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf_int2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf_int2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.576 | -0.178 | 2 | 0.114 | 0 of 34769 | 0 | 0 | 0 | 0.194 | 0.64 |
| ekf_kerb, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34779 | 0 | 0 | 0 | 0.093 | 0.64 |
| ekf_kerb, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34779 | 0 | 0 | 0 | 0.093 | 0.64 |
| ekf_kerb2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34779 | 0 | 0 | 0 | 0.093 | 0.64 |
| ekf_kerb2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34779 | 0 | 0 | 0 | 0.093 | 0.64 |
| ekf, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int2, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int2, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_kerb, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34778 | 0 | 0 | 0 | 0.084 | 0.64 |
| ekf_kerb, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34778 | 0 | 0 | 0 | 0.084 | 0.64 |
| ekf_kerb2, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34778 | 0 | 0 | 0 | 0.084 | 0.64 |
| ekf_kerb2, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34778 | 0 | 0 | 0 | 0.084 | 0.64 |
| ekf, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.481 | -0.437 | 2 | 0.373 | 0 of 17751 | 0 | 11 | 3 | 2.580 | 9.21 |
| ekf, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.481 | -0.437 | 2 | 0.373 | 0 of 17751 | 0 | 11 | 3 | 2.580 | 9.21 |
| ekf_int, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int2, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int2, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_kerb, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34797 | 0 | 0 | 0 | 0.091 | 0.64 |
| ekf_kerb, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34797 | 0 | 0 | 0 | 0.091 | 0.64 |
| ekf_kerb2, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34798 | 0 | 0 | 0 | 0.091 | 0.64 |
| ekf_kerb2, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34798 | 0 | 0 | 0 | 0.091 | 0.64 |
| ekf, no re-join, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 86.6 | - | 1.498 | -7.066 | 9 | 7.002 | 101 of 15657 | 0 | 12 | 0 | 3.533 | 8.10 |
| ekf, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 86.6 | - | 1.498 | -7.066 | 9 | 7.002 | 101 of 15657 | 0 | 12 | 0 | 3.533 | 8.10 |
| ekf_int, no re-join, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -7.066 | 9 | 7.002 | 91 of 15060 | 0 | 10 | 0 | 3.846 | 8.10 |
| ekf_int, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -7.066 | 9 | 7.002 | 91 of 15060 | 0 | 10 | 0 | 3.846 | 8.10 |
| ekf_int2, no re-join, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.124 | 9 | 5.060 | 91 of 15221 | 0 | 3 | 0 | 2.259 | 5.81 |
| ekf_int2, gnss_ramp_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -5.124 | 9 | 5.060 | 91 of 15221 | 0 | 3 | 0 | 2.259 | 5.81 |
| ekf_kerb, no re-join, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34746 | 0 | 0 | 0 | 0.131 | 0.87 |
| ekf_kerb, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34746 | 0 | 0 | 0 | 0.131 | 0.87 |
| ekf_kerb2, no re-join, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34746 | 0 | 0 | 0 | 0.131 | 0.87 |
| ekf_kerb2, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.5 | - | 0.193 | 0.046 | 0 | 0.000 | 0 of 34746 | 0 | 0 | 0 | 0.131 | 0.87 |
| ekf, no re-join, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 1 | 0 | 1.477 | 5.81 |
| ekf, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 1 | 0 | 1.477 | 5.81 |
| ekf_int, no re-join, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 0 | 0 | 1.477 | 4.42 |
| ekf_int, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 0 | 0 | 1.477 | 4.42 |
| ekf_int2, no re-join, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 0 | 0 | 1.477 | 4.42 |
| ekf_int2, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.5 | - | 1.500 | -3.876 | 10 | 3.812 | 154 of 27907 | 0 | 0 | 0 | 1.477 | 4.42 |
| ekf_kerb, no re-join, kerb_return_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 1.500 | -3.819 | 9 | 3.755 | 214 of 29828 | 0 | 0 | 0 | 1.391 | 4.22 |
| ekf_kerb, kerb_return_r6 | 10 | COMPLETED 10 | 10 of 10 | 71.6 | - | 1.500 | -3.819 | 8 | 3.755 | 214 of 29839 | 0 | 0 | 0 | 1.391 | 4.22 |
| ekf_kerb2, no re-join, kerb_return_r6 | 10 | COMPLETED 2, STOPPED_EN_ROUTE 8 | 2 of 10 | 72.2 | - | 1.499 | -3.819 | 8 | 3.755 | 192 of 20636 | 0 | 0 | 0 | 1.661 | 4.22 |
| ekf_kerb2, kerb_return_r6 | 10 | COMPLETED 6, STOPPED_EN_ROUTE 4 | 6 of 10 | 71.6 | - | 1.499 | -3.819 | 8 | 3.755 | 192 of 25134 | 0 | 0 | 0 | 1.420 | 4.22 |
| ekf, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_kerb, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 10 | 0 | 0.909 | 2.95 |
| ekf_kerb, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 10 | 0 | 0.909 | 2.95 |
| ekf_kerb2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 10 | 0 | 0.909 | 2.95 |
| ekf_kerb2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 10 | 0 | 0.909 | 2.95 |
| ekf, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int2, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int2, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_kerb, no re-join, combined_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 71.9 | - | 1.497 | -3.701 | 4 | 3.638 | 35 of 30959 | 0 | 0 | 0 | 0.197 | 6.27 |
| ekf_kerb, combined_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 71.9 | - | 1.497 | -9.007 | 4 | 8.944 | 35 of 33145 | 0 | 1 | 0 | 0.197 | 11.48 |
| ekf_kerb2, no re-join, combined_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.9 | - | 0.856 | -0.208 | 1 | 0.144 | 0 of 34034 | 0 | 0 | 0 | 0.181 | 1.66 |
| ekf_kerb2, combined_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 71.9 | - | 0.856 | -0.074 | 1 | 0.011 | 0 of 34034 | 0 | 0 | 0 | 0.181 | 1.66 |

## R6-B_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.86 | 0.193 | 0.029 | 0 | 0.000 | 0 of 39922 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf_int, no re-join, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf_int, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf_int2, no re-join, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf_int2, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.69 | 0.576 | -0.178 | 1 | 0.114 | 0 of 39739 | 0 | 0 | 0 | 0.146 | 0.64 |
| ekf_kerb, no re-join, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39760 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39760 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, no re-join, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39760 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf_kerb2, nominal_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39760 | 0 | 0 | 0 | 0.086 | 0.64 |
| ekf, no re-join, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int, no re-join, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int2, no re-join, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_int2, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 13.27 | 0.576 | -0.178 | 1 | 0.114 | 0 of 38449 | 0 | 10 | 7 | 0.214 | 0.64 |
| ekf_kerb, no re-join, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39774 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39774 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb2, no re-join, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39774 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb2, gnss_outage_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39774 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf, no re-join, multipath_r6 | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | 1 of 10 | - | 9.45 | 0.576 | -0.178 | 1 | 0.114 | 0 of 19151 | 0 | 10 | 3 | 2.580 | 9.21 |
| ekf, multipath_r6 | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | 1 of 10 | - | 9.45 | 0.576 | -0.178 | 1 | 0.114 | 0 of 19151 | 0 | 10 | 3 | 2.580 | 9.21 |
| ekf_int, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int2, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_int2, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 38070 | 0 | 10 | 3 | 0.301 | 0.86 |
| ekf_kerb, no re-join, multipath_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39764 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb, multipath_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39764 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb2, no re-join, multipath_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39764 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf_kerb2, multipath_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39764 | 0 | 0 | 0 | 0.075 | 0.64 |
| ekf, no re-join, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 8 | 1 of 10 | - | 6.37 | 1.498 | -7.066 | 9 | 7.002 | 101 of 15900 | 0 | 11 | 0 | 3.533 | 8.10 |
| ekf, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 8 | 1 of 10 | - | 6.37 | 1.498 | -7.066 | 9 | 7.002 | 101 of 15900 | 0 | 11 | 0 | 3.533 | 8.10 |
| ekf_int, no re-join, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 2, STOPPED_EN_ROUTE 8 | 0 of 10 | - | - | 1.498 | -7.066 | 9 | 7.002 | 91 of 15060 | 0 | 11 | 0 | 3.846 | 8.10 |
| ekf_int, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 2, STOPPED_EN_ROUTE 8 | 0 of 10 | - | - | 1.498 | -7.066 | 9 | 7.002 | 91 of 15060 | 0 | 11 | 0 | 3.846 | 8.10 |
| ekf_int2, no re-join, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 4, STOPPED_AT_CLOSURE 3, STOPPED_EN_ROUTE 3 | 3 of 10 | - | 1.06 | 1.500 | -5.124 | 9 | 5.060 | 91 of 15221 | 0 | 4 | 0 | 2.259 | 5.81 |
| ekf_int2, gnss_ramp_r6 | 10 | CLOSURE_INCURSION 4, STOPPED_AT_CLOSURE 3, STOPPED_EN_ROUTE 3 | 3 of 10 | - | 1.06 | 1.500 | -5.124 | 9 | 5.060 | 91 of 15221 | 0 | 4 | 0 | 2.259 | 5.81 |
| ekf_kerb, no re-join, gnss_ramp_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39737 | 0 | 0 | 0 | 0.130 | 0.87 |
| ekf_kerb, gnss_ramp_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39737 | 0 | 0 | 0 | 0.130 | 0.87 |
| ekf_kerb2, no re-join, gnss_ramp_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39737 | 0 | 0 | 0 | 0.130 | 0.87 |
| ekf_kerb2, gnss_ramp_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.82 | 0.193 | 0.046 | 0 | 0.000 | 0 of 39737 | 0 | 0 | 0 | 0.130 | 0.87 |
| ekf, no re-join, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 1.599 | 3.28 |
| ekf, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 1.599 | 3.28 |
| ekf_int, no re-join, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 2.312 | 3.28 |
| ekf_int, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 2.312 | 3.28 |
| ekf_int2, no re-join, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 2.312 | 3.28 |
| ekf_int2, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.56 | 1.499 | -2.706 | 5 | 2.642 | 0 of 22207 | 0 | 0 | 0 | 2.312 | 3.28 |
| ekf_kerb, no re-join, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.76 | 1.498 | -2.676 | 4 | 2.612 | 0 of 24566 | 0 | 0 | 0 | 2.250 | 3.14 |
| ekf_kerb, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.76 | 1.498 | -2.676 | 4 | 2.612 | 0 of 24566 | 0 | 0 | 0 | 2.250 | 3.14 |
| ekf_kerb2, no re-join, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.76 | 1.498 | -2.676 | 4 | 2.612 | 0 of 24566 | 0 | 0 | 0 | 2.250 | 3.14 |
| ekf_kerb2, kerb_return_r6 | 10 | CLOSURE_INCURSION 1, STOPPED_AT_CLOSURE 8, STOPPED_EN_ROUTE 1 | 8 of 10 | - | 0.76 | 1.498 | -2.676 | 4 | 2.612 | 0 of 24566 | 0 | 0 | 0 | 2.250 | 3.14 |
| ekf, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_int2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.498 | -0.575 | 10 | 0.511 | 0 of 13796 | 0 | 10 | 0 | 1.372 | 4.36 |
| ekf_kerb, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 11 | 0 | 0.909 | 2.95 |
| ekf_kerb, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 11 | 0 | 0.909 | 2.95 |
| ekf_kerb2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 11 | 0 | 0.909 | 2.95 |
| ekf_kerb2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -0.542 | 9 | 0.478 | 0 of 13734 | 0 | 11 | 0 | 0.909 | 2.95 |
| ekf, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int2, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_int2, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.993 | -0.178 | 1 | 0.114 | 0 of 31422 | 0 | 10 | 5 | 0.271 | 0.64 |
| ekf_kerb, no re-join, combined_r6 | 10 | STOPPED_AT_CLOSURE 9, STOPPED_EN_ROUTE 1 | 9 of 10 | - | 1.80 | 1.308 | -0.890 | 1 | 0.826 | 0 of 37053 | 0 | 0 | 0 | 0.085 | 1.41 |
| ekf_kerb, combined_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.80 | 1.308 | -0.890 | 1 | 0.826 | 0 of 39764 | 0 | 0 | 0 | 0.085 | 1.41 |
| ekf_kerb2, no re-join, combined_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.80 | 0.856 | -0.208 | 1 | 0.144 | 0 of 39764 | 0 | 0 | 0 | 0.085 | 1.19 |
| ekf_kerb2, combined_r6 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.80 | 0.856 | -0.074 | 1 | 0.011 | 0 of 39762 | 0 | 0 | 0 | 0.085 | 1.19 |

## R6-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 73.5 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 35780 | 0 | 0 | 0 | - | - |
| ekf, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf_int, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf_int, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf_int2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf_int2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.5 | - | 0.515 | -0.202 | 2 | 0.138 | 0 of 35704 | 0 | 0 | 0 | 0.168 | 0.61 |
| ekf_kerb, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.066 | 0 | 0.000 | 0 of 35700 | 0 | 0 | 0 | 0.078 | 0.61 |
| ekf_kerb, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.066 | 0 | 0.000 | 0 of 35700 | 0 | 0 | 0 | 0.078 | 0.61 |
| ekf_kerb2, no re-join, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.066 | 0 | 0.000 | 0 of 35700 | 0 | 0 | 0 | 0.078 | 0.61 |
| ekf_kerb2, nominal_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.066 | 0 | 0.000 | 0 of 35700 | 0 | 0 | 0 | 0.078 | 0.61 |
| ekf, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf_int, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf_int, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf_int2, no re-join, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf_int2, gnss_outage_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.780 | -0.202 | 2 | 0.138 | 0 of 36773 | 0 | 10 | 8 | 0.186 | 0.61 |
| ekf_kerb, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.068 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf_kerb, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.068 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf_kerb2, no re-join, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.068 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf_kerb2, gnss_outage_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.4 | - | 0.193 | 0.068 | 0 | 0.000 | 0 of 35695 | 0 | 0 | 0 | 0.077 | 0.61 |
| ekf, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 15209 | 0 | 10 | 9 | 2.737 | 9.83 |
| ekf, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 15209 | 0 | 10 | 9 | 2.737 | 9.83 |
| ekf_int, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 36718 | 0 | 10 | 9 | 0.205 | 0.61 |
| ekf_int, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 36718 | 0 | 10 | 9 | 0.205 | 0.61 |
| ekf_int2, no re-join, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 36718 | 0 | 10 | 9 | 0.205 | 0.61 |
| ekf_int2, multipath_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.856 | -0.202 | 2 | 0.138 | 0 of 36718 | 0 | 10 | 9 | 0.205 | 0.61 |
| ekf_kerb, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 35685 | 0 | 0 | 0 | 0.079 | 0.61 |
| ekf_kerb, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 35685 | 0 | 0 | 0 | 0.079 | 0.61 |
| ekf_kerb2, no re-join, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 35685 | 0 | 0 | 0 | 0.079 | 0.61 |
| ekf_kerb2, multipath_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 35685 | 0 | 0 | 0 | 0.079 | 0.61 |
| ekf, no re-join, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 82.1 | - | 1.499 | -6.977 | 10 | 6.913 | 35 of 15109 | 0 | 10 | 1 | 3.314 | 8.81 |
| ekf, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 82.1 | - | 1.499 | -6.977 | 10 | 6.913 | 35 of 15109 | 0 | 10 | 1 | 3.314 | 8.81 |
| ekf_int, no re-join, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 73.0 | - | 1.499 | -6.977 | 10 | 6.913 | 35 of 14355 | 0 | 7 | 1 | 2.720 | 8.81 |
| ekf_int, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 73.0 | - | 1.499 | -6.977 | 10 | 6.913 | 35 of 14355 | 0 | 7 | 1 | 2.720 | 8.81 |
| ekf_int2, no re-join, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 73.0 | - | 1.499 | -5.476 | 10 | 5.414 | 35 of 14350 | 0 | 4 | 1 | 2.713 | 6.57 |
| ekf_int2, gnss_ramp_r6 | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 73.0 | - | 1.499 | -5.476 | 10 | 5.414 | 35 of 14350 | 0 | 4 | 1 | 2.713 | 6.57 |
| ekf_kerb, no re-join, gnss_ramp_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 73.3 | - | 1.498 | -3.405 | 2 | 3.347 | 0 of 29852 | 0 | 1 | 0 | 0.169 | 5.11 |
| ekf_kerb, gnss_ramp_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.3 | - | 1.498 | -3.405 | 2 | 3.347 | 0 of 33777 | 0 | 1 | 0 | 0.169 | 5.11 |
| ekf_kerb2, no re-join, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.070 | 0 | 0.000 | 0 of 35648 | 0 | 0 | 0 | 0.169 | 1.10 |
| ekf_kerb2, gnss_ramp_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.3 | - | 0.193 | 0.070 | 0 | 0.000 | 0 of 35648 | 0 | 0 | 0 | 0.169 | 1.10 |
| ekf, no re-join, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.5 | - | 1.499 | -3.946 | 8 | 3.882 | 122 of 29158 | 0 | 3 | 0 | 1.375 | 4.93 |
| ekf, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.5 | - | 1.499 | -3.946 | 8 | 3.882 | 122 of 29158 | 0 | 3 | 0 | 1.375 | 4.93 |
| ekf_int, no re-join, kerb_return_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 73.3 | - | 1.499 | -3.711 | 8 | 3.647 | 122 of 27239 | 0 | 0 | 0 | 1.424 | 4.13 |
| ekf_int, kerb_return_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 73.3 | - | 1.499 | -3.711 | 8 | 3.647 | 122 of 27239 | 0 | 0 | 0 | 1.424 | 4.13 |
| ekf_int2, no re-join, kerb_return_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 73.3 | - | 1.499 | -3.711 | 8 | 3.647 | 122 of 27239 | 0 | 0 | 0 | 1.424 | 4.13 |
| ekf_int2, kerb_return_r6 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 73.3 | - | 1.499 | -3.711 | 8 | 3.647 | 122 of 27239 | 0 | 0 | 0 | 1.424 | 4.13 |
| ekf_kerb, no re-join, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.5 | - | 1.500 | -3.760 | 7 | 3.696 | 189 of 30150 | 0 | 0 | 0 | 1.357 | 4.07 |
| ekf_kerb, kerb_return_r6 | 10 | COMPLETED 9, STOPPED_EN_ROUTE 1 | 9 of 10 | 73.5 | - | 1.500 | -3.760 | 7 | 3.696 | 189 of 30159 | 0 | 0 | 0 | 1.357 | 4.07 |
| ekf_kerb2, no re-join, kerb_return_r6 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 73.4 | - | 1.500 | -3.760 | 7 | 3.696 | 161 of 24850 | 0 | 0 | 0 | 1.285 | 4.07 |
| ekf_kerb2, kerb_return_r6 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 73.4 | - | 1.500 | -3.760 | 7 | 3.696 | 161 of 28574 | 0 | 0 | 0 | 1.285 | 4.07 |
| ekf, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf_int, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf_int, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf_int2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf_int2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.142 | -0.202 | 2 | 0.138 | 0 of 15741 | 0 | 10 | 8 | 0.830 | 3.13 |
| ekf_kerb, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.786 | 0.178 | 0 | 0.000 | 0 of 15514 | 0 | 10 | 10 | 0.491 | 1.76 |
| ekf_kerb, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.786 | 0.178 | 0 | 0.000 | 0 of 15514 | 0 | 10 | 10 | 0.491 | 1.76 |
| ekf_kerb2, no re-join, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.786 | 0.178 | 0 | 0.000 | 0 of 15514 | 0 | 10 | 10 | 0.491 | 1.76 |
| ekf_kerb2, imu_fault_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.786 | 0.178 | 0 | 0.000 | 0 of 15514 | 0 | 10 | 10 | 0.491 | 1.76 |
| ekf, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf_int, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf_int, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf_int2, no re-join, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf_int2, combined_r6 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.001 | -0.202 | 2 | 0.138 | 0 of 30993 | 0 | 10 | 4 | 0.382 | 0.68 |
| ekf_kerb, no re-join, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 1.500 | -1.710 | 3 | 1.646 | 88 of 35136 | 0 | 0 | 0 | 0.592 | 3.57 |
| ekf_kerb, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 1.500 | -1.710 | 3 | 1.646 | 88 of 35120 | 0 | 0 | 0 | 0.593 | 3.57 |
| ekf_kerb2, no re-join, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 35602 | 0 | 0 | 0 | 0.159 | 1.29 |
| ekf_kerb2, combined_r6 | 10 | COMPLETED 10 | 10 of 10 | 73.2 | - | 0.193 | 0.074 | 0 | 0.000 | 0 of 35602 | 0 | 0 | 0 | 0.159 | 1.29 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
