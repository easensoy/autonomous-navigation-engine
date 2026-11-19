# Closed-loop driving on estimated state: vehicle transfer: San Francisco (SoMa) with the I-PACE proxy

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..15. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. Vehicle: ipace_proxy_v1. Each estimator drives with the unique re-join, the default stack since R7.

## SF-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 15 | COMPLETED 15 | - | 76.4 | - | 0.193 | 5.280 | 0 | 0.000 | 0 of 55805 | 0 | 0 | 0 | - | - |
| ekf_kerb2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 76.1 | - | 0.193 | 5.280 | 0 | 0.000 | 0 of 55567 | 0 | 0 | 0 | 0.090 | 0.66 |
| ekf_kerb2, gnss_outage_sf | 15 | COMPLETED 15 | 15 of 15 | 76.0 | - | 0.193 | 5.280 | 0 | 0.000 | 0 of 55551 | 0 | 0 | 0 | 0.160 | 0.85 |
| ekf_kerb2, gnss_drift_sf | 15 | COMPLETED 11, STOPPED_EN_ROUTE 4 | 11 of 15 | 76.0 | - | 1.493 | 3.832 | 0 | 0.000 | 87 of 47009 | 0 | 0 | 0 | 0.428 | 4.11 |
| ekf_kerb2, kerb_map_error_sf | 15 | COMPLETED 9, STOPPED_EN_ROUTE 6 | 9 of 15 | 75.9 | - | 1.498 | 5.280 | 0 | 0.000 | 646 of 43654 | 0 | 0 | 0 | 0.825 | 2.53 |

## SF-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 15 | COMPLETED 15 | - | 87.7 | - | 0.193 | 4.151 | 0 | 0.000 | 0 of 64340 | 0 | 0 | 0 | - | - |
| ekf_kerb2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 87.3 | - | 0.193 | 4.111 | 0 | 0.000 | 0 of 64015 | 0 | 0 | 0 | 0.085 | 0.66 |
| ekf_kerb2, gnss_outage_sf | 15 | COMPLETED 15 | 15 of 15 | 87.3 | - | 0.193 | 4.111 | 0 | 0.000 | 0 of 64031 | 0 | 0 | 0 | 0.155 | 0.76 |
| ekf_kerb2, gnss_drift_sf | 15 | COMPLETED 9, STOPPED_EN_ROUTE 6 | 9 of 15 | 87.3 | - | 1.483 | 4.110 | 0 | 0.000 | 0 of 48615 | 0 | 0 | 0 | 0.385 | 4.11 |
| ekf_kerb2, kerb_map_error_sf | 15 | COMPLETED 13, STOPPED_EN_ROUTE 2 | 13 of 15 | 87.2 | - | 1.331 | 4.115 | 0 | 0.000 | 0 of 58967 | 0 | 0 | 0 | 0.598 | 2.41 |

Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
