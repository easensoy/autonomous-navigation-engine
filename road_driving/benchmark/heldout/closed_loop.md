# Closed-loop driving on estimated state: held-out split (network R2)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on the EKF's estimate under each sensor condition, with matched seeds 1..10. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260.

## R2-A

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | 62.6 | - | 0.193 | 0.000 | 0 | 0 of 30315 | 0 | - | - |
| EKF, nominal_h | 10 | COMPLETED 10 | 62.3 | - | 0.463 | -0.183 | 1 | 0 of 30110 | 0 | 0.179 | 0.61 |
| EKF, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | - | - | 0.534 | -0.183 | 2 | 0 of 43651 | 10 | 0.297 | 0.71 |
| EKF, multipath_h | 10 | COMPLETED 1, STOPPED_EN_ROUTE 9 | 64.4 | - | 0.638 | -0.374 | 5 | 0 of 21464 | 0 | 2.250 | 9.21 |
| EKF, combined_h | 10 | COMPLETED 10 | 66.2 | - | 0.595 | -0.270 | 2 | 0 of 32111 | 0 | 0.207 | 0.65 |

## R2-A_late_closure

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | STOPPED_AT_CLOSURE 10 | - | 1.86 | 0.193 | 0.000 | 0 | 0 of 51676 | 0 | - | - |
| EKF, nominal_h | 10 | STOPPED_AT_CLOSURE 10 | - | 1.73 | 0.463 | -0.183 | 1 | 0 of 51517 | 0 | 0.156 | 0.61 |
| EKF, gnss_outage_h | 10 | STOPPED_EN_ROUTE 10 | - | - | 0.534 | -0.183 | 2 | 0 of 43651 | 10 | 0.297 | 0.71 |
| EKF, multipath_h | 10 | STOPPED_AT_CLOSURE 1, STOPPED_EN_ROUTE 9 | - | 1.76 | 0.638 | -0.374 | 5 | 0 of 23601 | 0 | 2.250 | 9.21 |
| EKF, combined_h | 10 | STOPPED_AT_CLOSURE 10 | - | 1.71 | 0.595 | -0.270 | 2 | 0 of 53478 | 0 | 0.185 | 0.65 |

## R2-B

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | 61.2 | - | 0.193 | 0.001 | 0 | 0 of 29655 | 0 | - | - |
| EKF, nominal_h | 10 | COMPLETED 10 | 61.0 | - | 0.411 | -0.240 | 4 | 0 of 29456 | 0 | 0.195 | 0.66 |
| EKF, gnss_outage_h | 10 | COMPLETED 10 | 66.2 | - | 0.650 | -0.454 | 5 | 0 of 32052 | 0 | 0.224 | 0.79 |
| EKF, multipath_h | 10 | STOPPED_EN_ROUTE 10 | - | - | 1.476 | -4.360 | 1 | 0 of 21937 | 0 | 1.904 | 7.91 |
| EKF, combined_h | 10 | COMPLETED 10 | 64.7 | - | 0.558 | -0.356 | 3 | 0 of 31352 | 0 | 0.226 | 0.75 |

## R2-C

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 10 | COMPLETED 10 | 37.8 | - | 0.193 | 0.000 | 0 | 0 of 17900 | 0 | - | - |
| EKF, nominal_h | 10 | COMPLETED 10 | 37.5 | - | 0.463 | -0.229 | 7 | 0 of 17754 | 0 | 0.165 | 0.61 |
| EKF, gnss_outage_h | 10 | COMPLETED 10 | 37.6 | - | 0.567 | -0.242 | 8 | 0 of 17801 | 0 | 0.231 | 0.73 |
| EKF, multipath_h | 10 | COMPLETED 10 | 37.6 | - | 0.615 | -0.229 | 6 | 0 of 17778 | 0 | 0.218 | 0.63 |
| EKF, combined_h | 10 | COMPLETED 10 | 37.6 | - | 0.589 | -0.270 | 8 | 0 of 17759 | 0 | 0.205 | 0.78 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to a carriageway edge; negative means part of the footprint crossed it. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it).
