# Closed-loop driving on estimated state: development split (network R1)

Each scenario is driven with the stack on simulator truth (the labelled reference) and on the EKF's estimate under each sensor condition, with matched seeds 1..5. Every quantity below is measured from truth by the evaluator. Toolchain: GCC 14.2.0.

## c0_nominal

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 5 | COMPLETED 5 | 52.4 | - | 0.161 | 0.000 | 0 | 0 of 12645 | 0 | - | - |
| EKF, nominal | 5 | COMPLETED 5 | 52.3 | - | 0.463 | -0.183 | 1 | 0 of 12593 | 0 | 0.183 | 0.61 |
| EKF, gnss_outage | 5 | COMPLETED 5 | 53.8 | - | 0.649 | -0.183 | 2 | 0 of 12955 | 0 | 0.192 | 0.67 |
| EKF, multipath | 5 | COMPLETED 5 | 52.7 | - | 0.594 | -0.183 | 1 | 0 of 12666 | 0 | 0.186 | 0.66 |

## c2_advance

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 5 | COMPLETED 5 | 108.7 | - | 0.161 | 0.000 | 0 | 0 of 26680 | 0 | - | - |
| EKF, nominal | 5 | COMPLETED 5 | 108.4 | - | 0.463 | -0.183 | 2 | 0 of 26627 | 0 | 0.198 | 0.61 |
| EKF, gnss_outage | 5 | COMPLETED 5 | 108.4 | - | 0.463 | -0.183 | 2 | 0 of 26627 | 0 | 0.198 | 0.61 |
| EKF, multipath | 5 | COMPLETED 5 | 108.4 | - | 0.463 | -0.183 | 2 | 0 of 26627 | 0 | 0.198 | 0.61 |

## c3_r30

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 5 | STOPPED_AT_CLOSURE 5 | - | 1.88 | 0.161 | 0.000 | 0 | 0 of 20893 | 0 | - | - |
| EKF, nominal | 5 | STOPPED_AT_CLOSURE 5 | - | 1.79 | 0.463 | -0.183 | 1 | 0 of 20829 | 0 | 0.170 | 0.61 |
| EKF, gnss_outage | 5 | STOPPED_AT_CLOSURE 5 | - | 1.79 | 0.649 | -0.183 | 2 | 0 of 21203 | 0 | 0.180 | 0.67 |
| EKF, multipath | 5 | STOPPED_AT_CLOSURE 5 | - | 1.79 | 0.594 | -0.183 | 1 | 0 of 20870 | 0 | 0.173 | 0.66 |

## c5_lift

| Pose source | Runs | Outcomes | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs off the road | Lane-association errors | Localisation stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 5 | COMPLETED 5 | 90.1 | - | 0.161 | 0.000 | 0 | 0 of 22055 | 0 | - | - |
| EKF, nominal | 5 | COMPLETED 5 | 89.4 | - | 0.463 | -0.183 | 1 | 0 of 21858 | 0 | 0.162 | 0.61 |
| EKF, gnss_outage | 5 | COMPLETED 5 | 91.1 | - | 0.649 | -0.183 | 2 | 0 of 22253 | 0 | 0.172 | 0.67 |
| EKF, multipath | 5 | COMPLETED 5 | 89.4 | - | 0.594 | -0.183 | 1 | 0 of 21894 | 0 | 0.165 | 0.66 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to a carriageway edge; negative means part of the footprint crossed it. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it).
