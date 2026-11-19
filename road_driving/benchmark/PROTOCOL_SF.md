# San Francisco: the unchanged filters on a new network, predeclared protocol

This file was written and committed with the code and tools it evaluates, before any scored run on the evaluation seeds or on route SF-B. The results are added afterwards in [`sf/`](sf), produced once by that code unchanged. The earlier evaluations ([R2](PROTOCOL.md) to [R7](PROTOCOL_R7.md)) and the London scenes are unchanged.

## The question

How well do the existing filters localise, and the existing stack drive, on a new street network when GNSS becomes unreliable, and when do kerb observations help or mislead them?

Nothing in the estimators or the stack changed for this evaluation: the five filters, `locpolicy`, `clearpolicy` and the unique re-join are those frozen for R7 (`EstimatorConfig` defaults, no overrides). What is new is where they run and on what:

| New | What |
|---|---|
| Network `SF-SOMA` | San Francisco, South of Market (SoMa), by the Caltrain station at 4th and Townsend: the block bounded by Townsend, 5th, Brannan and 4th Streets, with Bluxome Street where it crosses 5th Street. **Right-hand traffic.** Built from a pinned OpenStreetMap extract ([provenance](../viewer/districts/PROVENANCE.md)) |
| Vehicle `cybercab_proxy_v1` | A Cybercab-inspired proxy: wheelbase 2.75 m, length 4.20 m, width 1.93 m, no exterior mirrors; GNSS antenna 1.30 m, IMU 1.10 m and kerb scanner 3.55 m ahead of the rear axle; actuator limits as the I-PACE proxy's. Tesla's public pages give its ground clearance and cabin, not these: they are research assumptions ([vehicle.hpp](../include/road_driving/vehicle.hpp)) |
| Four conditions | Placed in metres along the route, below |
| A map error kept apart from the world | The vehicle's map (estimator and stack) can differ from the world's kerb (scanner and evaluator) |
| Extended metrics | Along-road error and per-axis consistency, the fault window's timing, integrity delays and false alarms, kerb corrections |

The traffic side is a district setting: the modelled lane runs right of the centre line, the left turn crosses the opposing lane and carries the 4 s turn penalty, and the stack's clearance shift is bounded towards the centre line on the lane's left. London and R1 to R7 keep left-hand traffic and their results: the 192-episode closure suite is byte-identical, and the R7-A held-out closed loops (850 episodes) and the London showcase traces reproduce exactly.

## Network and routes

`SF-SOMA` ([`src/district.cpp`](../src/district.cpp)): 5 junctions and 10 patch edges, 27 lanes, 48 turning movements, 2 banned turns (OSM relations 10673966 and 10673575: no left turn from 4th Street north-east-bound onto Townsend or Brannan). 4th Street is one-way south-east, its light-rail trackway modelled as a 7.2 m central reservation. The local frame is the WGS84 tangent plane at 37.7770 N, 122.3950 W; it reproduces the geodesic lengths between the mapped junction nodes to within 4.2 mm over 826 m.

| Route | Start | Path | Length | Turns |
|---|---|---|---|---|
| `SF-A` | Townsend Street south-west-bound, 15 m past 4th Street's junction mouth | the 231 m straight past the Caltrain rail yard, right into 5th Street, ahead across Bluxome Street, right into Brannan Street, 100 m along it | 567 m | right, ahead, right (near side) |
| `SF-B` | Brannan Street south-west-bound, 15 m past 4th Street's junction mouth | the 232 m straight, left into 5th Street, ahead across Bluxome Street, left into Townsend Street, 200 m along it towards the station corner | 671 m | left, ahead, left (across the opposing lane) |

**SF-B is a held-back route within one district.** It shares its streets with SF-A, but drives them in the opposite direction, on the other side of each street, with left turns on wider arcs. Before the scored runs it was driven only on simulator truth (seed 0, the geometry and integration smoke test) and in the static checks; no estimator drove it. SF-A was used for development (below), with seeds 901 to 905 that the evaluation does not use.

Both vehicles' footprints, driven exactly along every movement and lane, clear the kerb by at least 2.0 m (the I-PACE proxy with mirrors) and 2.18 m (the Cybercab-inspired proxy): the streets the routes drive are 15.6 to 17.6 m between kerbs, with the modelled lanes 1.65 to 1.8 m from the centre line.

## Conditions (frozen)

Distances are metres along the route from its start. Both routes run about 231 m along their first street before their first turn, so each condition covers the same stretch of both. A zone is the set of map rectangles around the route over its span, as for every earlier condition; the actual timing comes from the simulator and is reported per run.

| Condition | Definition |
|---|---|
| `nominal_sf` | no fault |
| `gnss_outage_sf` | no fixes from 50 to 240 m: the first straight's last 180 m and the turn's entry |
| `gnss_drift_sf` | a simulated fault model: from 40 m the fixes drift away at 0.3 m/s with no jump, reach 4.0 m and stay at 4.0 m until 240 m, then return at 0.3 m/s. Its direction is drawn per seed (the kerb stream's draw, as for every earlier ramp) |
| `kerb_map_error_sf` | the outage of `gnss_outage_sf`, and a misplaced map segment: from 90 to 200 m both kerbs of the street are 0.6 m to the route's right in the vehicle's map, with a step at each end. The world keeps the true kerbs; no measurement changes |

The sensor models and latencies are unchanged: GNSS at 10 Hz with 0.10 ± 0.02 s latency, 0.25 m white noise per axis and a 0.15 m Gauss-Markov error (60 s), 2% dropped; IMU at 100 Hz, no latency; wheel speed and steering at 50 Hz, 0.02 s; kerb scanner at 10 Hz, 0.05 s, 37 beams over ±90°, 0.5 to 25 m, 0.03 m range noise. Each sensor is mounted where the vehicle profile puts it, and the estimators use the same positions. A GNSS bias is a prescribed simulated fault: no building geometry generates it.

## Splits, seeds and workload

| Split | Vehicle | Routes | Seeds | Open loop | Closed loop |
|---|---|---|---|---|---|
| `sf-dev` (development, done) | Cybercab-inspired | SF-A | 901 to 905 | 20 drives x 8 estimators | 45 episodes |
| `sf` (the evaluation) | Cybercab-inspired | SF-A, SF-B | 1 to 15 | 120 drives x 8 estimators | 270 episodes |
| `sf-ipace` (vehicle transfer) | I-PACE proxy | SF-A, SF-B | 1 to 15 | 120 drives x 8 estimators | 150 episodes |

- **Open loop.** Each drive is recorded once on simulator truth; `dr`, `gnss_hold`, `gnss_cv`, `ekf`, `ekf_int`, `ekf_int2`, `ekf_kerb` and `ekf_kerb2` each process the same timestamped sensor log in the same arrival order, with that vehicle's lever arms and the vehicle's map, and are scored in the fixed map frame.
- **Closed loop.** Each route on simulator truth (the reference), and under each condition on `ekf_kerb2` (the established stack) and `ekf_int2` (a filter without kerbs, for the decision comparison), both with the unique re-join; on the I-PACE proxy, `ekf_kerb2` only. 2 x (1 + 4 x 2) x 15 = 270 and 2 x (1 + 4) x 15 = 150 episodes. Matched seeds align the random streams; trajectories and the measurements they produce can then diverge.
- Each split runs once, with the MSVC 19.51 build of the commit that adds this file. Nothing is re-run after the results are seen.

## Metrics

Errors are rear-axle position errors at 50 Hz from each estimator's initialisation, in metres. **Along-road and lateral** are the error projected on the true heading at each sample, and across it: defined in junctions too, with no route association, so the along-road error a straight's kerbs cannot see shows as such and becomes lateral when the heading turns. **Per-axis consistency** is the mean of (error / the filter's own sigma on that axis)^2, 1 degree of freedom, and the share inside the 95% interval; samples without a covariance are left out.

The **fault window** comes from the world: the longest outage, else the samples where the fixes carry their full offset (the drift's sustained part). In it: the error at its end (and its along and lateral parts), the largest error inside it, the first fix after it (its time of validity and arrival), the **detection delay** (from onset to the first FAULTED or INCONSISTENT state; never: -1), the **trust delay** (from the end until TRUSTED is held for 1 s; never: 999) and the **reacquisition** (from the end until the error is held below 0.5 m for 1 s; never: 999, a censored value counted as not reacquired). **False alarms** are entries into FAULTED or INCONSISTENT outside the window and the 10 s after it. **Kerb corrections** are the registrations that moved the estimate more than 0.1 m, the unique ones, and the largest. Under the map error: the largest and final lateral error while the true car is beside the misplaced segment.

Closed loop, from truth: completion, the same outcome as the reference, the smallest clearance (with mirrors), runs with the body over a kerb and how deep, fallback stops and their reason (the monitor's message), localisation and clearance stops and unwarranted ones (true error below 0.3 m at the decision), lane-association errors, illegal moves, pose corrections re-joined. A fallback stop is not a completion.

The independent unit is the run (route x seed): repeated samples within a run are summarised per run before any comparison across runs. Every table states its denominator.

## Criteria

[`tools/sf_analysis.py`](../tools/sf_analysis.py), committed with this file, computes every verdict and writes `summary.json`.

| # | Criterion |
|---|---|
| P1 | **The primary transfer question.** `gnss_outage_sf`: `ekf_kerb2`'s error when GNSS returns is below dead reckoning's in at least 27 of the 30 pairs (route x seed), and its median is at most 0.50 m. The same comparison for `ekf`, `ekf_int2`, `ekf_kerb` and `gnss_cv` is reported |
| P2 | **The nominal trade-off.** `nominal_sf`: `ekf_kerb2`'s lateral 95th percentile, pooled over its runs, at most 0.10 m (R3's bound; the checker uses the largest run's 95th percentile, which the pooled one cannot exceed), and every run's 95th percentile position error at most 0.50 m; no EKF run fails (error above 5 m); the median NEES (3 dof) of `ekf`, `ekf_int2` and `ekf_kerb2` within 0.5 to 6 |
| P3 | **The closed-loop reference outcome.** `nominal_sf` and `gnss_outage_sf`: `ekf_kerb2` ends at least 29 of 30 episodes as the reference does, with no episode over a kerb and no fallback stop |
| P4 | **The ambiguous kerb.** `kerb_map_error_sf`: no `ekf_kerb2` episode with the body over a kerb. The comparison with `ekf_int2` (lateral error beside the segment, end-of-outage error, closed-loop outcome) is reported both ways, whichever is better |
| P5 | **The R7-A seed 7 mechanism**, reported with a predeclared judgement: SF reproduces the mechanism if some `ekf_kerb2` run enters the turn after the drift with more than 1.0 m of along-road error and less than 0.3 m lateral (open loop), and some closed-loop drift episode then ends in a fallback stop for lateral error or with the body over a kerb. It reproduces the R7 failure only if one is over a kerb |
| P6 | **Integrity without faults.** `nominal_sf`: no FAULTED or INCONSISTENT entry by `ekf_int`, `ekf_int2`, `ekf_kerb` or `ekf_kerb2` |
| P7 | **The vehicle transfer.** P1's test on the I-PACE proxy's own logs |

**Expectations, stated before the run:**
- P1 should pass: on R2 to R7 the kerb filters held the lateral error through every outage, and here both kerbs stay in range on the first straight (6.6 to 10.6 m away).
- P3 holds for the reference by design; for `ekf_kerb2` it needs no stop for clearance, which wide streets make likely. `ekf_int2` is expected to stop for localisation inside the outage (its lateral sigma passes 0.75 m), and to wait there: there is no fallback to truth. That is reported, not judged.
- P4: from development, `ekf_kerb2` follows the misplaced segment (its scans fit it uniquely: a lateral error near 0.6 m), and corrects at the segment's ends and at the turn. A first version of this condition misplaced only the near kerb; the registration's residual gate rejected those returns and fitted the correct far kerb, and nothing happened. That was replaced by a misplaced segment, which a scan cannot reject because both kerbs agree with each other.
- P5: in development (SF-A, seeds 901 to 905), the drift left `ekf_kerb2` with up to 1.9 m of along-road error on the straight while it reported an along-road 2-sigma near 0.6 m; at the turn a re-localisation and the conversion to lateral error ended four of the five closed-loop episodes in fallback stops, with 3.4 m or more of clearance. SF is expected to reproduce the mechanism, and not the crossing: its streets leave 3 to 5 m.
- P7 should hold: the lever arms differ by 0.30 m (GNSS and IMU) and 0.29 m (scanner), within what the filters model as installation data.

**Reported without a criterion:** every other cell of the open-loop and closed-loop tables, per condition and estimator, and per vehicle; paired differences of the vehicles on the same scenario and seed (their sensor logs differ, so these compare scenarios, not logs).

## What development did

Development used SF-A only, seeds 901 to 905 (`road_drive bench-loc --split sf-dev --seeds 5` and `bench-closed`), and single development drives. It settled the network's geometry (the patch edges continuing each street's line exactly, Bluxome Street's junction mouth at 11 m so its near-side turns fit a 5.5 m arc), the conditions' placement in metres, the map error's form (above), and the analysis tools. No estimator or stack setting was changed.

It also set P2's position bound. A first draft required every nominal run's 95th percentile position error to be at most 0.30 m; on SF-A `ekf_kerb2` reached 0.39 m, almost all of it along the road on the 231 m straight, where kerbs say nothing about along-road position, while its lateral error stayed at 0.047 m. The lateral bound is R3's (0.10 m) and the position bound became 0.50 m: the criterion tests the lateral accuracy the clearance depends on, and bounds the along-road error that SF's long straights leave.

## Commands

Open loop, then closed loop, from the repository root (MSVC build):

```bash
road_drive bench-loc --split sf --seeds 15 --out road_driving/benchmark/sf/sf
road_drive bench-closed --split sf --seeds 15 --out road_driving/benchmark/sf/sf
road_drive bench-loc --split sf-ipace --seeds 15 --out road_driving/benchmark/sf/sf-ipace
road_drive bench-closed --split sf-ipace --seeds 15 --out road_driving/benchmark/sf/sf-ipace
python road_driving/tools/sf_analysis.py road_driving/benchmark/sf
```

The example traces (a typical run and the worst relevant one) are single drives written with `road_drive drive ... --out`, and their timelines with [`tools/sf_timeline.py`](../tools/sf_timeline.py).

## The showcase recording

The recording shows SF-A under `gnss_outage_sf` with the evaluation seed whose closed-loop `ekf_kerb2` episode has the median position RMSE of the 15 (ties: the lower seed): a typical run, chosen by this rule after the evaluation, not by watching runs.

## Analysis rules

- Every run counts, failed and incomplete runs included, and every table states its denominator.
- Nothing is excluded or re-run after the results are seen. If a result leads to a change, the original results are reported, and the cases used are marked as seen before any later campaign.
- A criterion that fails is reported as failed.
