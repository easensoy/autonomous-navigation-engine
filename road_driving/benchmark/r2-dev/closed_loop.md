# Closed-loop driving on estimated state: development and regression data for the integrity and kerb increment (network R2)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: GCC 14.2.0. "previous stack" is the stack evaluated on R2 in November 2026, without the clearance layer; every other row, and the reference, use the clearance layer.

## R2-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 62.6 | - | 0.193 | 0.128 | 0 | 0.000 | 0 of 30315 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 62.3 | - | 0.463 | -0.183 | 1 | 0.119 | 0 of 30110 | 0 | 0 | 0 | 0.179 | 0.61 |
| ekf, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 62.3 | - | 0.457 | -0.202 | 2 | 0.138 | 0 of 30162 | 0 | 0 | 0 | 0.179 | 0.61 |
| ekf_int, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 62.3 | - | 0.457 | -0.202 | 2 | 0.138 | 0 of 30162 | 0 | 0 | 0 | 0.179 | 0.61 |
| ekf_kerb, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 62.2 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 30087 | 0 | 0 | 0 | 0.081 | 0.61 |
| ekf, previous stack, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.534 | -0.183 | 2 | 0.119 | 0 of 43651 | 10 | 0 | 2 | 0.297 | 0.71 |
| ekf, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.741 | -0.202 | 2 | 0.138 | 0 of 39456 | 0 | 10 | 7 | 0.189 | 0.64 |
| ekf_int, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.741 | -0.202 | 2 | 0.138 | 0 of 39456 | 0 | 10 | 7 | 0.189 | 0.64 |
| ekf_kerb, gnss_outage_h | 10 | COMPLETED 10 | 10 of 10 | 62.1 | - | 0.193 | 0.068 | 0 | 0.000 | 0 of 30074 | 0 | 0 | 0 | 0.083 | 0.61 |
| ekf, previous stack, multipath_h | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 64.4 | - | 0.638 | -0.374 | 5 | 0.310 | 0 of 21464 | 0 | 0 | 0 | 2.250 | 9.21 |
| ekf, multipath_h | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 1 of 10 | 64.4 | - | 1.297 | -0.202 | 2 | 0.138 | 0 of 21463 | 0 | 10 | 6 | 2.249 | 9.21 |
| ekf_int, multipath_h | 10 | COMPLETED 7, STOPPED_EN_ROUTE 3 | 7 of 10 | 64.8 | - | 1.335 | -0.202 | 2 | 0.138 | 0 of 28442 | 0 | 10 | 6 | 0.220 | 0.61 |
| ekf_kerb, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 62.2 | - | 0.193 | 0.066 | 0 | 0.000 | 0 of 30097 | 0 | 0 | 0 | 0.081 | 0.61 |
| ekf, previous stack, combined_h | 10 | COMPLETED 10 | 10 of 10 | 66.2 | - | 0.595 | -0.270 | 2 | 0.206 | 0 of 32111 | 0 | 0 | 0 | 0.207 | 0.65 |
| ekf, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.984 | -0.265 | 1 | 0.202 | 0 of 32941 | 0 | 10 | 4 | 0.194 | 0.71 |
| ekf_int, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.984 | -0.265 | 1 | 0.202 | 0 of 32941 | 0 | 10 | 4 | 0.194 | 0.71 |
| ekf_kerb, combined_h | 10 | COMPLETED 10 | 10 of 10 | 62.1 | - | 0.193 | 0.067 | 0 | 0.000 | 0 of 30087 | 0 | 0 | 0 | 0.101 | 0.61 |

## R2-A_late_closure

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | - | 1.86 | 0.193 | 0.128 | 0 | 0.000 | 0 of 51676 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.463 | -0.183 | 1 | 0.119 | 0 of 51517 | 0 | 0 | 0 | 0.156 | 0.61 |
| ekf, nominal_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.457 | -0.202 | 2 | 0.138 | 0 of 51569 | 0 | 0 | 0 | 0.156 | 0.61 |
| ekf_int, nominal_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.457 | -0.202 | 2 | 0.138 | 0 of 51569 | 0 | 0 | 0 | 0.156 | 0.61 |
| ekf_kerb, nominal_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.71 | 0.193 | 0.067 | 0 | 0.000 | 0 of 51530 | 0 | 0 | 0 | 0.073 | 0.61 |
| ekf, previous stack, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.534 | -0.183 | 2 | 0.119 | 0 of 43651 | 10 | 0 | 2 | 0.297 | 0.71 |
| ekf, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.741 | -0.202 | 2 | 0.138 | 0 of 39456 | 0 | 10 | 7 | 0.189 | 0.64 |
| ekf_int, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.741 | -0.202 | 2 | 0.138 | 0 of 39456 | 0 | 10 | 7 | 0.189 | 0.64 |
| ekf_kerb, gnss_outage_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.73 | 0.193 | 0.068 | 0 | 0.000 | 0 of 51505 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf, previous stack, multipath_h | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | 1 of 10 | - | 1.76 | 0.638 | -0.374 | 5 | 0.310 | 0 of 23601 | 0 | 0 | 0 | 2.250 | 9.21 |
| ekf, multipath_h | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | 1 of 10 | - | 1.72 | 1.297 | -0.202 | 2 | 0.138 | 0 of 23633 | 0 | 10 | 6 | 2.249 | 9.21 |
| ekf_int, multipath_h | 10 | STOPPED_AT_CLOSURE 7, STOPPED_EN_ROUTE 3 | 7 of 10 | - | 1.75 | 1.335 | -0.202 | 2 | 0.138 | 0 of 43411 | 0 | 10 | 6 | 0.185 | 0.61 |
| ekf_kerb, multipath_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.71 | 0.193 | 0.066 | 0 | 0.000 | 0 of 51498 | 0 | 0 | 0 | 0.074 | 0.61 |
| ekf, previous stack, combined_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.71 | 0.595 | -0.270 | 2 | 0.206 | 0 of 53478 | 0 | 0 | 0 | 0.185 | 0.65 |
| ekf, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.984 | -0.265 | 1 | 0.202 | 0 of 32941 | 0 | 10 | 4 | 0.194 | 0.71 |
| ekf_int, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.984 | -0.265 | 1 | 0.202 | 0 of 32941 | 0 | 10 | 4 | 0.194 | 0.71 |
| ekf_kerb, combined_h | 10 | STOPPED_AT_CLOSURE 10 | 10 of 10 | - | 1.72 | 0.193 | 0.067 | 0 | 0.000 | 0 of 51483 | 0 | 0 | 0 | 0.083 | 0.61 |

## R2-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 61.2 | - | 0.193 | 0.029 | 0 | 0.000 | 0 of 29645 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 61.0 | - | 0.411 | -0.240 | 4 | 0.178 | 0 of 29456 | 0 | 0 | 0 | 0.195 | 0.66 |
| ekf, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 60.8 | - | 0.636 | 0.001 | 0 | 0.000 | 0 of 29414 | 0 | 0 | 0 | 0.195 | 0.66 |
| ekf_int, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 60.8 | - | 0.636 | 0.001 | 0 | 0.000 | 0 of 29414 | 0 | 0 | 0 | 0.195 | 0.66 |
| ekf_kerb, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 60.9 | - | 0.193 | 0.028 | 0 | 0.000 | 0 of 29429 | 0 | 0 | 0 | 0.082 | 0.66 |
| ekf, previous stack, gnss_outage_h | 10 | COMPLETED 10 | 10 of 10 | 66.2 | - | 0.650 | -0.454 | 5 | 0.392 | 0 of 32052 | 0 | 0 | 0 | 0.224 | 0.79 |
| ekf, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.211 | 0.001 | 0 | 0.000 | 0 of 42980 | 0 | 10 | 7 | 0.248 | 0.87 |
| ekf_int, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.211 | 0.001 | 0 | 0.000 | 0 of 42980 | 0 | 10 | 7 | 0.248 | 0.87 |
| ekf_kerb, gnss_outage_h | 10 | COMPLETED 10 | 10 of 10 | 60.9 | - | 0.193 | 0.037 | 0 | 0.000 | 0 of 29446 | 0 | 0 | 0 | 0.081 | 0.66 |
| ekf, previous stack, multipath_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.476 | -4.360 | 1 | 4.296 | 0 of 21937 | 0 | 0 | 0 | 1.904 | 7.91 |
| ekf, multipath_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.493 | -4.278 | 3 | 4.214 | 0 of 21740 | 0 | 12 | 9 | 2.097 | 7.92 |
| ekf_int, multipath_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 1.184 | 0.001 | 0 | 0.000 | 0 of 34371 | 0 | 10 | 9 | 0.242 | 0.66 |
| ekf_kerb, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 60.9 | - | 0.193 | 0.037 | 0 | 0.000 | 0 of 29447 | 0 | 0 | 0 | 0.085 | 0.66 |
| ekf, previous stack, combined_h | 10 | COMPLETED 10 | 10 of 10 | 64.7 | - | 0.558 | -0.356 | 3 | 0.294 | 0 of 31352 | 0 | 0 | 0 | 0.226 | 0.75 |
| ekf, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.670 | -0.021 | 0 | 0.000 | 0 of 33681 | 0 | 10 | 5 | 0.211 | 0.67 |
| ekf_int, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.670 | -0.021 | 0 | 0.000 | 0 of 33681 | 0 | 10 | 5 | 0.211 | 0.67 |
| ekf_kerb, combined_h | 10 | COMPLETED 10 | 10 of 10 | 60.8 | - | 0.193 | 0.037 | 0 | 0.000 | 0 of 29409 | 0 | 0 | 0 | 0.100 | 0.66 |

## R2-C

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | - | 37.8 | - | 0.193 | 0.028 | 0 | 0.000 | 0 of 17900 | 0 | 0 | 0 | - | - |
| ekf, previous stack, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 37.5 | - | 0.463 | -0.229 | 7 | 0.167 | 0 of 17754 | 0 | 0 | 0 | 0.165 | 0.61 |
| ekf, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 37.4 | - | 0.604 | -0.202 | 2 | 0.138 | 0 of 17715 | 0 | 0 | 0 | 0.166 | 0.61 |
| ekf_int, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 37.4 | - | 0.604 | -0.202 | 2 | 0.138 | 0 of 17715 | 0 | 0 | 0 | 0.166 | 0.61 |
| ekf_kerb, nominal_h | 10 | COMPLETED 10 | 10 of 10 | 37.4 | - | 0.193 | 0.023 | 0 | 0.000 | 0 of 17718 | 0 | 0 | 0 | 0.076 | 0.61 |
| ekf, previous stack, gnss_outage_h | 10 | COMPLETED 10 | 10 of 10 | 37.6 | - | 0.567 | -0.242 | 8 | 0.180 | 0 of 17801 | 0 | 0 | 0 | 0.231 | 0.73 |
| ekf, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.692 | -0.202 | 2 | 0.138 | 0 of 37046 | 0 | 10 | 7 | 0.263 | 0.68 |
| ekf_int, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.692 | -0.202 | 2 | 0.138 | 0 of 37046 | 0 | 10 | 7 | 0.263 | 0.68 |
| ekf_kerb, gnss_outage_h | 10 | COMPLETED 10 | 10 of 10 | 37.5 | - | 0.193 | 0.023 | 0 | 0.000 | 0 of 17748 | 0 | 0 | 0 | 0.081 | 0.61 |
| ekf, previous stack, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 37.6 | - | 0.615 | -0.229 | 6 | 0.167 | 0 of 17778 | 0 | 0 | 0 | 0.218 | 0.63 |
| ekf, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 42.0 | - | 0.710 | -0.202 | 2 | 0.138 | 0 of 20026 | 0 | 10 | 6 | 0.203 | 0.62 |
| ekf_int, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 42.4 | - | 0.699 | -0.202 | 2 | 0.138 | 0 of 20198 | 0 | 10 | 6 | 0.207 | 0.63 |
| ekf_kerb, multipath_h | 10 | COMPLETED 10 | 10 of 10 | 37.5 | - | 0.193 | 0.023 | 0 | 0.000 | 0 of 17748 | 0 | 0 | 0 | 0.078 | 0.61 |
| ekf, previous stack, combined_h | 10 | COMPLETED 10 | 10 of 10 | 37.6 | - | 0.589 | -0.270 | 8 | 0.206 | 0 of 17759 | 0 | 0 | 0 | 0.205 | 0.78 |
| ekf, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.935 | -0.265 | 1 | 0.202 | 0 of 30288 | 0 | 10 | 8 | 0.222 | 0.73 |
| ekf_int, combined_h | 10 | STOPPED_EN_ROUTE 10 | 0 of 10 | - | - | 0.935 | -0.265 | 1 | 0.202 | 0 of 30288 | 0 | 10 | 8 | 0.222 | 0.73 |
| ekf_kerb, combined_h | 10 | COMPLETED 10 | 10 of 10 | 37.5 | - | 0.193 | 0.016 | 0 | 0.000 | 0 of 17771 | 0 | 0 | 0 | 0.084 | 0.61 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
