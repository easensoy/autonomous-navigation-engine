# Closed-loop driving on estimated state: the unchanged filters on a new network: San Francisco (SoMa), Cybercab-inspired proxy

Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each sensor condition, with matched seeds 1..15. Every quantity below is measured from truth by the evaluator. Toolchain: MSVC 195136260. Vehicle: cybercab_proxy_v1. Each estimator drives with the unique re-join, the default stack since R7.

## SF-A

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 15 | COMPLETED 15 | - | 76.4 | - | 0.193 | 5.394 | 0 | 0.000 | 0 of 55800 | 0 | 0 | 0 | - | - |
| ekf_kerb2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 76.0 | - | 0.193 | 5.394 | 0 | 0.000 | 0 of 55560 | 0 | 0 | 0 | 0.089 | 0.65 |
| ekf_int2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 76.0 | - | 0.505 | 5.130 | 0 | 0.000 | 0 of 55568 | 0 | 0 | 0 | 0.196 | 0.78 |
| ekf_kerb2, gnss_outage_sf | 15 | COMPLETED 15 | 15 of 15 | 76.1 | - | 0.193 | 5.394 | 0 | 0.000 | 0 of 55574 | 0 | 0 | 0 | 0.160 | 0.88 |
| ekf_int2, gnss_outage_sf | 15 | STOPPED_EN_ROUTE 15 | 0 of 15 | - | - | 0.947 | 4.717 | 0 | 0.000 | 0 of 51274 | 15 | 0 | 2 | 0.510 | 1.05 |
| ekf_kerb2, gnss_drift_sf | 15 | COMPLETED 11, STOPPED_EN_ROUTE 4 | 11 of 15 | 76.1 | - | 1.494 | 3.945 | 0 | 0.000 | 86 of 47014 | 0 | 0 | 0 | 0.422 | 4.12 |
| ekf_int2, gnss_drift_sf | 15 | COMPLETED 14, STOPPED_EN_ROUTE 1 | 14 of 15 | 76.1 | - | 1.500 | 1.513 | 0 | 0.000 | 192 of 40351 | 0 | 0 | 0 | 2.068 | 4.67 |
| ekf_kerb2, kerb_map_error_sf | 15 | COMPLETED 13, STOPPED_EN_ROUTE 2 | 13 of 15 | 76.0 | - | 1.497 | 5.394 | 0 | 0.000 | 261 of 51280 | 0 | 0 | 0 | 0.675 | 2.45 |
| ekf_int2, kerb_map_error_sf | 15 | STOPPED_EN_ROUTE 15 | 0 of 15 | - | - | 0.947 | 4.717 | 0 | 0.000 | 0 of 51274 | 15 | 0 | 2 | 0.510 | 1.05 |

## SF-B

| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ground truth (reference) | 15 | COMPLETED 15 | - | 87.7 | - | 0.193 | 4.322 | 0 | 0.000 | 0 of 64340 | 0 | 0 | 0 | - | - |
| ekf_kerb2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 87.4 | - | 0.193 | 4.283 | 0 | 0.000 | 0 of 64026 | 0 | 0 | 0 | 0.086 | 0.65 |
| ekf_int2, nominal_sf | 15 | COMPLETED 15 | 15 of 15 | 87.4 | - | 0.526 | 4.151 | 0 | 0.000 | 0 of 64048 | 0 | 0 | 0 | 0.179 | 0.78 |
| ekf_kerb2, gnss_outage_sf | 15 | COMPLETED 15 | 15 of 15 | 87.4 | - | 0.193 | 4.283 | 0 | 0.000 | 0 of 64051 | 0 | 0 | 0 | 0.156 | 0.75 |
| ekf_int2, gnss_outage_sf | 15 | STOPPED_EN_ROUTE 15 | 0 of 15 | - | - | 0.949 | 5.065 | 0 | 0.000 | 0 of 51254 | 15 | 0 | 2 | 0.509 | 1.05 |
| ekf_kerb2, gnss_drift_sf | 15 | COMPLETED 8, STOPPED_EN_ROUTE 7 | 8 of 15 | 87.3 | - | 1.498 | 4.282 | 0 | 0.000 | 0 of 45707 | 0 | 0 | 0 | 0.385 | 4.09 |
| ekf_int2, gnss_drift_sf | 15 | COMPLETED 13, STOPPED_EN_ROUTE 2 | 13 of 15 | 87.4 | - | 1.499 | 1.859 | 0 | 0.000 | 269 of 46721 | 0 | 0 | 0 | 1.948 | 4.67 |
| ekf_kerb2, kerb_map_error_sf | 15 | COMPLETED 11, STOPPED_EN_ROUTE 4 | 11 of 15 | 87.2 | - | 1.358 | 4.287 | 0 | 0.000 | 0 of 54036 | 0 | 0 | 0 | 0.604 | 2.54 |
| ekf_int2, kerb_map_error_sf | 15 | STOPPED_EN_ROUTE 15 | 0 of 15 | - | - | 0.949 | 5.065 | 0 | 0.000 | 0 of 51254 | 15 | 0 | 2 | 0.509 | 1.05 |

Clearance is the smallest distance from the footprint with mirrors (4.200 m by 1.930 m) to the kerb; negative means part of the footprint crossed it. Runs with the body over a kerb count the body without mirrors (1.930 m wide), and Deepest is how far it went. Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). A stop is unwarranted when the true position error at the decision was below 0.3 m.
