# Closed-loop driving on estimated state: held-out evaluation of the integrity and kerb increment (network R3)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. "previous stack" is the stack evaluated on R2 in November 2026, without the clearance layer; every other row, and the reference, use the clearance layer.

## R3-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 70.8 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 34395 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.504 | -0.183 | 3 | 0.119 | 0 of 34198 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.5 | - | 0.566 | -0.202 | 2 | 0.138 | 0 of 34228 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf_int, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.5 | - | 0.566 | -0.202 | 2 | 0.138 | 0 of 34228 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf_kerb, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.193 | 0.042 | 0 | 0.000 | 0 of 34231 | 0 | 0 | 0 | 0.075 | 0.61 |
| ekf, previous stack, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.905 | -0.183 | 1 | 0.119 | 0 of 44520 | 10 | 0 | 4 | 0.335 | 0.97 |
| ekf, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.009 | -0.202 | 2 | 0.138 | 0 of 40332 | 0 | 10 | 5 | 0.205 | 0.78 |
| ekf_int, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.009 | -0.202 | 2 | 0.138 | 0 of 40332 | 0 | 10 | 5 | 0.205 | 0.78 |
| ekf_kerb, gnss_outage_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.193 | 0.048 | 0 | 0.000 | 0 of 34209 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf, previous stack, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.628 | -0.272 | 2 | 0.208 | 0 of 21585 | 0 | 0 | 0 | 2.459 | 10.20 |
| ekf, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.824 | -0.202 | 2 | 0.138 | 0 of 20771 | 0 | 10 | 6 | 1.944 | 10.19 |
| ekf_int, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.824 | -0.202 | 2 | 0.138 | 0 of 41771 | 0 | 10 | 6 | 0.189 | 0.71 |
| ekf_kerb, multipath_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.193 | 0.048 | 0 | 0.000 | 0 of 34196 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf, previous stack, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.168 | 10 | 4.104 | 266 of 24549 | 0 | 0 | 0 | 2.081 | 5.00 |
| ekf, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.174 | 10 | 4.110 | 248 of 24581 | 0 | 0 | 0 | 2.054 | 5.00 |
| ekf_int, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -4.174 | 10 | 4.110 | 248 of 24581 | 0 | 0 | 0 | 2.054 | 5.00 |
| ekf_kerb, gnss_ramp_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.6 | - | 0.193 | 0.048 | 0 | 0.000 | 0 of 34413 | 0 | 0 | 0 | 0.091 | 0.61 |
| ekf, previous stack, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.504 | -0.183 | 3 | 0.119 | 0 of 34198 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.5 | - | 0.566 | -0.202 | 2 | 0.138 | 0 of 34228 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf_int, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.5 | - | 0.566 | -0.202 | 2 | 0.138 | 0 of 34228 | 0 | 0 | 0 | 0.174 | 0.61 |
| ekf_kerb, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.5 | - | 0.347 | 0.044 | 0 | 0.000 | 0 of 34227 | 0 | 0 | 0 | 0.083 | 0.61 |
| ekf, previous stack, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -9.140 | 10 | 9.083 | 0 of 21824 | 0 | 0 | 0 | 2.640 | 15.31 |
| ekf, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.202 | 2 | 0.138 | 0 of 35281 | 0 | 10 | 6 | 0.220 | 0.69 |
| ekf_int, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.150 | -0.202 | 2 | 0.138 | 0 of 35281 | 0 | 10 | 6 | 0.220 | 0.69 |
| ekf_kerb, combined_r3 | 10 | COMPLETED 10 | 10 of 10 | 70.4 | - | 0.193 | 0.053 | 0 | 0.000 | 0 of 34221 | 0 | 0 | 0 | 0.080 | 0.61 |

## R3-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 72.3 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 35170 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.384 | -0.193 | 3 | 0.132 | 0 of 35035 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 71.8 | - | 0.656 | 0.001 | 0 | 0.000 | 0 of 34941 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf_int, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 71.8 | - | 0.656 | 0.001 | 0 | 0.000 | 0 of 34941 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf_kerb, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.193 | 0.033 | 0 | 0.000 | 0 of 35049 | 0 | 0 | 0 | 0.081 | 0.66 |
| ekf, previous stack, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.384 | 0.004 | 0 | 0.000 | 0 of 43361 | 10 | 0 | 8 | 0.212 | 0.77 |
| ekf, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.709 | 0.001 | 0 | 0.000 | 0 of 39179 | 0 | 10 | 8 | 0.184 | 0.66 |
| ekf_int, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.709 | 0.001 | 0 | 0.000 | 0 of 39179 | 0 | 10 | 8 | 0.184 | 0.66 |
| ekf_kerb, gnss_outage_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 35018 | 0 | 0 | 0 | 0.088 | 0.66 |
| ekf, previous stack, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -1.396 | 5 | 1.353 | 0 of 20342 | 0 | 0 | 0 | 2.655 | 10.46 |
| ekf, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.488 | -5.723 | 1 | 5.659 | 0 of 20330 | 0 | 11 | 9 | 2.655 | 10.46 |
| ekf_int, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.106 | 0.001 | 0 | 0.000 | 0 of 33460 | 0 | 10 | 9 | 0.258 | 0.77 |
| ekf_kerb, multipath_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.193 | 0.040 | 0 | 0.000 | 0 of 35038 | 0 | 0 | 0 | 0.091 | 0.66 |
| ekf, previous stack, gnss_ramp_r3 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 71.6 | - | 1.500 | -3.980 | 8 | 3.916 | 269 of 27260 | 0 | 0 | 0 | 2.066 | 4.80 |
| ekf, gnss_ramp_r3 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 71.4 | - | 1.500 | -3.979 | 8 | 3.915 | 267 of 27113 | 0 | 0 | 0 | 2.069 | 4.80 |
| ekf_int, gnss_ramp_r3 | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 71.4 | - | 1.500 | -3.979 | 8 | 3.915 | 267 of 27113 | 0 | 0 | 0 | 2.069 | 4.80 |
| ekf_kerb, gnss_ramp_r3 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 72.2 | - | 1.500 | -3.989 | 4 | 3.925 | 32 of 29526 | 0 | 2 | 0 | 0.711 | 4.74 |
| ekf, previous stack, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.384 | -0.193 | 3 | 0.132 | 0 of 35035 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 71.8 | - | 0.656 | 0.001 | 0 | 0.000 | 0 of 34941 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf_int, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 71.8 | - | 0.656 | 0.001 | 0 | 0.000 | 0 of 34941 | 0 | 0 | 0 | 0.202 | 0.66 |
| ekf_kerb, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.1 | - | 0.193 | 0.032 | 0 | 0.000 | 0 of 35052 | 0 | 0 | 0 | 0.089 | 0.66 |
| ekf, previous stack, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -11.100 | 9 | 11.037 | 0 of 23379 | 2 | 0 | 2 | 1.072 | 13.53 |
| ekf, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.751 | 0.001 | 0 | 0.000 | 0 of 33207 | 0 | 10 | 6 | 0.256 | 0.73 |
| ekf_int, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.751 | 0.001 | 0 | 0.000 | 0 of 33207 | 0 | 10 | 6 | 0.256 | 0.73 |
| ekf_kerb, combined_r3 | 10 | COMPLETED 10 | 10 of 10 | 72.0 | - | 0.216 | 0.112 | 0 | 0.000 | 0 of 34999 | 0 | 0 | 0 | 0.097 | 0.66 |

## R3-B_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.84 | 0.193 | 0.029 | 0 | 0.000 | 0 of 51719 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.384 | -0.193 | 3 | 0.132 | 0 of 51536 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf, nominal_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.656 | 0.001 | 0 | 0.000 | 0 of 51535 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf_int, nominal_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.656 | 0.001 | 0 | 0.000 | 0 of 51535 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf_kerb, nominal_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.76 | 0.193 | 0.033 | 0 | 0.000 | 0 of 51518 | 0 | 0 | 0 | 0.074 | 0.66 |
| ekf, previous stack, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.384 | 0.004 | 0 | 0.000 | 0 of 43361 | 10 | 0 | 8 | 0.212 | 0.77 |
| ekf, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.709 | 0.001 | 0 | 0.000 | 0 of 39179 | 0 | 10 | 8 | 0.184 | 0.66 |
| ekf_int, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.709 | 0.001 | 0 | 0.000 | 0 of 39179 | 0 | 10 | 8 | 0.184 | 0.66 |
| ekf_kerb, gnss_outage_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.83 | 0.193 | 0.040 | 0 | 0.000 | 0 of 51507 | 0 | 0 | 0 | 0.086 | 0.66 |
| ekf, previous stack, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -1.396 | 5 | 1.353 | 0 of 20342 | 0 | 0 | 0 | 2.655 | 10.46 |
| ekf, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.488 | -5.723 | 1 | 5.659 | 0 of 20330 | 0 | 11 | 9 | 2.655 | 10.46 |
| ekf_int, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.106 | 0.001 | 0 | 0.000 | 0 of 33460 | 0 | 10 | 9 | 0.258 | 0.77 |
| ekf_kerb, multipath_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.78 | 0.193 | 0.040 | 0 | 0.000 | 0 of 51499 | 0 | 0 | 0 | 0.089 | 0.66 |
| ekf, previous stack, gnss_ramp_r3 | 10 | CLOSURE_INCURSION 6, STOPPED_AT_CLOSURE 2, STOPPED_EN_ROUTE 2 | 2 of 10 | 71.4 | 5.12 | 1.500 | -3.980 | 8 | 3.916 | 269 of 29435 | 0 | 0 | 0 | 2.050 | 4.80 |
| ekf, gnss_ramp_r3 | 10 | CLOSURE_INCURSION 6, STOPPED_AT_CLOSURE 2, STOPPED_EN_ROUTE 2 | 2 of 10 | 71.2 | 5.14 | 1.500 | -3.979 | 8 | 3.915 | 267 of 29286 | 0 | 0 | 0 | 2.054 | 4.80 |
| ekf_int, gnss_ramp_r3 | 10 | CLOSURE_INCURSION 6, STOPPED_AT_CLOSURE 2, STOPPED_EN_ROUTE 2 | 2 of 10 | 71.2 | 5.14 | 1.500 | -3.979 | 8 | 3.915 | 267 of 29286 | 0 | 0 | 0 | 2.120 | 4.80 |
| ekf_kerb, gnss_ramp_r3 | 10 | CLOSURE_INCURSION 3, STOPPED_AT_CLOSURE 5, STOPPED_EN_ROUTE 2 | 5 of 10 | 72.2 | 1.89 | 1.500 | -3.989 | 4 | 3.925 | 32 of 37708 | 0 | 2 | 0 | 0.711 | 4.74 |
| ekf, previous stack, kerb_clutter_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.384 | -0.193 | 3 | 0.132 | 0 of 51536 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf, kerb_clutter_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.656 | 0.001 | 0 | 0.000 | 0 of 51535 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf_int, kerb_clutter_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.656 | 0.001 | 0 | 0.000 | 0 of 51535 | 0 | 0 | 0 | 0.175 | 0.66 |
| ekf_kerb, kerb_clutter_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.70 | 0.193 | 0.032 | 0 | 0.000 | 0 of 51530 | 0 | 0 | 0 | 0.099 | 0.66 |
| ekf, previous stack, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.499 | -11.100 | 9 | 11.037 | 0 of 23379 | 2 | 0 | 2 | 1.072 | 13.53 |
| ekf, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.751 | 0.001 | 0 | 0.000 | 0 of 33207 | 0 | 10 | 6 | 0.256 | 0.73 |
| ekf_int, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.751 | 0.001 | 0 | 0.000 | 0 of 33207 | 0 | 10 | 6 | 0.256 | 0.73 |
| ekf_kerb, combined_r3 | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.79 | 0.216 | 0.112 | 0 | 0.000 | 0 of 51509 | 0 | 0 | 0 | 0.090 | 0.66 |

## R3-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 45.3 | - | 0.193 | 0.178 | 0 | 0.000 | 0 of 21665 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.0 | - | 0.556 | -0.230 | 1 | 0.167 | 0 of 21509 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 21598 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf_int, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 21598 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf_kerb, nominal_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.1 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 21558 | 0 | 0 | 0 | 0.090 | 0.64 |
| ekf, previous stack, gnss_outage_r3 | 10 | COMPLETED 8, STOPPED_EN_ROUTE 2 | 8 of 10 | 48.2 | - | 1.345 | -0.410 | 3 | 0.348 | 0 of 22524 | 0 | 0 | 0 | 0.330 | 1.38 |
| ekf, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.953 | -0.178 | 1 | 0.114 | 0 of 38042 | 0 | 14 | 8 | 0.259 | 0.64 |
| ekf_int, gnss_outage_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.953 | -0.178 | 1 | 0.114 | 0 of 38042 | 0 | 14 | 8 | 0.259 | 0.64 |
| ekf_kerb, gnss_outage_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.193 | 0.170 | 0 | 0.000 | 0 of 21548 | 0 | 0 | 0 | 0.108 | 0.64 |
| ekf, previous stack, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.180 | -1.445 | 6 | 1.384 | 0 of 15215 | 0 | 0 | 0 | 3.139 | 10.23 |
| ekf, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.333 | -0.984 | 6 | 0.923 | 0 of 15203 | 0 | 0 | 0 | 3.132 | 10.22 |
| ekf_int, multipath_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.435 | -0.178 | 1 | 0.114 | 0 of 36761 | 0 | 10 | 3 | 0.305 | 1.05 |
| ekf_kerb, multipath_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.193 | 0.170 | 0 | 0.000 | 0 of 21560 | 0 | 0 | 0 | 0.093 | 0.64 |
| ekf, previous stack, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -3.797 | 9 | 3.733 | 72 of 15514 | 0 | 0 | 0 | 1.931 | 4.62 |
| ekf, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -3.809 | 9 | 3.745 | 72 of 15524 | 0 | 0 | 0 | 1.934 | 4.64 |
| ekf_int, gnss_ramp_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.500 | -3.809 | 9 | 3.745 | 72 of 15524 | 0 | 0 | 0 | 1.934 | 4.41 |
| ekf_kerb, gnss_ramp_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.1 | - | 0.193 | 0.155 | 0 | 0.000 | 0 of 21572 | 0 | 0 | 0 | 0.084 | 0.64 |
| ekf, previous stack, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.0 | - | 0.556 | -0.230 | 1 | 0.167 | 0 of 21509 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 21598 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf_int, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.576 | -0.178 | 1 | 0.114 | 0 of 21598 | 0 | 0 | 0 | 0.191 | 0.64 |
| ekf_kerb, kerb_clutter_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.2 | - | 0.201 | 0.169 | 0 | 0.000 | 0 of 21568 | 0 | 0 | 0 | 0.099 | 0.64 |
| ekf, previous stack, combined_r3 | 10 | COMPLETED 5, STOPPED_EN_ROUTE 5 | 5 of 10 | 47.7 | - | 1.077 | -0.230 | 3 | 0.167 | 0 of 22919 | 0 | 0 | 0 | 0.308 | 0.88 |
| ekf, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.033 | -0.178 | 1 | 0.114 | 0 of 30308 | 0 | 10 | 4 | 0.268 | 0.68 |
| ekf_int, combined_r3 | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.033 | -0.178 | 1 | 0.114 | 0 of 30308 | 0 | 10 | 4 | 0.268 | 0.68 |
| ekf_kerb, combined_r3 | 10 | COMPLETED 10 | 10 of 10 | 45.1 | - | 0.193 | 0.169 | 0 | 0.000 | 0 of 21555 | 0 | 0 | 0 | 0.096 | 0.64 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
