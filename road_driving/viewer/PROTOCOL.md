# Live protocol (`road_drive live`, version 1)

The viewer starts `road_drive live` as a child process and speaks to it over its standard streams:

| Stream | Direction | Content |
|---|---|---|
| stdin | viewer to backend | commands, one JSON object per line, each with `"cmd"` |
| stdout | backend to viewer | messages, one JSON object per line, each with `"v": 1` and `"type"` |
| stderr | backend to a log | human-readable diagnostics, never parsed |

When stdin closes (the viewer exits or crashes) the backend stops. The implementation is `road_driving/src/live.cpp`; the viewer's side is `ane_viewer/protocol.py`.

## Time

The simulation advances in fixed 10 ms world ticks, exactly as a headless run (`runEpisode`) does. The backend calls two hooks per tick (`src/live_hooks.hpp`): one before the sensors step, which applies operator commands, and one after the stack's control step, which reports.

Every simulation message carries `tick` (the world tick, from 0) and `t` (simulated seconds, `tick` × 0.01).

Pacing only waits between ticks:
- `speed` is simulated seconds per wall second; 0 means as fast as possible.
- If the machine falls more than 0.25 s behind, the backend counts a lag event in `status` and paces on from there, instead of racing to catch up.

The rendering rate, the observation rate, pausing and pacing change no computation. `tests/test_live.cpp` checks this trace for trace.

## Commands

| `cmd` | Fields | Effect |
|---|---|---|
| `reset` | `config` | start a new live episode (see Configuration) |
| `replay` | `session` | replay a session file (REPLAY mode) |
| `pause`, `resume` | | freeze or release simulated time; the interface stays live |
| `step` | `ticks` | run exactly that many ticks, then stay paused |
| `speed` | `value` | pacing |
| `observe` | `every` | send a `tick` message every N ticks (measurements and events are always sent) |
| `gnss_outage` | `on` | no fixes while on (the noise is still drawn, so the other sensors are unchanged) |
| `gnss_bias` | `on`, `metres`, `bearing_deg` | a constant offset on every fix, `metres` at `bearing_deg` from the car's true heading when applied; fixed in the map frame from then on |
| `kerb_blackout` | `on` | no kerb scans while on |
| `closure` | `action`: `ahead` (`distance`, `works`) or `lift` | place a roadworks closure on the stack's route ahead, or lift it; the oracle notifies the stack at once |
| `save_session` | `path` | write the session file |
| `quit` | | stop |

The world commands are `gnss_outage`, `gnss_bias`, `kerb_blackout` and `closure`:
- Each takes effect at the start of the next tick the backend runs, or at `at_tick` if given.
- An `applied` message reports the tick and the resolved command, for example `gnss_bias` with the map-frame `dx` and `dy`, or `closure_add` with the lane and barrier position.
- In REPLAY mode world commands are refused; they come from the session file.

## Messages

| `type` | When | Content |
|---|---|---|
| `hello` | once | protocol name, backend commit and toolchain, command list |
| `session` | each episode | `mode` (`live` or `replay`), the full configuration, labels, `dt` |
| `scene` | each episode | the map (junctions, streets, lanes, kerb lines and corner arcs), vehicle dimensions, sensor mounts, condition zones, obstacles the kerb scanner sees. A district network's map (`KEN`, `KNB`) adds three fields, still version 1: `district` (its id, name, title, origin, source and licence), `street_info` (per street: its real name, the kerb's distance left and right of the centreline, the central reservation's width, the lane's offset) and `kerb_boundary` (the indices of the kerb lines that are patch edges: they close the carriageway but are not kerbs). The grids' maps are unchanged. A right-hand-traffic district (`SF-SOMA`) adds `city`, `frame` and `"traffic":"right"` to `district`; a left-hand one omits `traffic`. The vehicle carries its profile: `id`, `description`, and `height`, `wheel_radius` and `track` for drawing; an older message without them is drawn as the I-PACE proxy. `map_errors` lists each misplaced kerb of the vehicle's map: the kerb line, its offset, and its ends as the map has them (`map`) and where the world's kerb is (`true`). |
| `tick` | every tick (or every N) | the true state (for drawing and diagnostics only), the command, the estimate with its covariance and integrity state, the stack's mode and offsets, the truth diagnostics (`diag`: along-road, lateral and heading error in the true heading frame; `clearance`; `crossing`), the sensors' availability and the injected faults |
| `gnss` | each fix released | the fix (map frame) and its reported sigma |
| `scan` | each kerb scan released | the scanner's true pose at the scan, the beam geometry, the 37 ranges (-1: no return), and the filter's decision on that scan (`MATCHED`, `REJECTED`, `-`) |
| `event` | as they happen | world and stack events (`CLOSURE_NOTIFIED`, `POSE_CORRECTION`, `MONITOR_TRIGGER`, ...) and the estimator's: `KERB_CORRECTION` (`unique` true or false, `dx`, `dy`), `KERB_RELOCALISATION` |
| `route` | each route adopted | the whole route path |
| `path` | 10 Hz | the reference the controller follows for the next 80 m, with its clearance shift and re-join |
| `closure` | on each change | barrier pose, works end, lane width, present, notified, lifted |
| `status` | 4 Hz and on change | `running`, `paused`, `finished`; speed, measured real-time factor, `lagging`, lag events |
| `applied` | each world command | the tick and the resolved command |
| `end` | episode end | outcome, stack mode, trace hash, toolchain, key metrics; in replay, whether the recorded trace was reproduced |
| `saved`, `error`, `bye` | | |

## Configuration

```json
{"scenario": "R7-A", "condition": "kerb_faults_r7", "mode": "estimated", "estimator": "ekf_kerb2", "seed": 7, "rejoin": "unique"}
```

`vehicle` (`ipace_proxy_v1`, the default, or `cybercab_proxy_v1`) chooses the vehicle profile; the session message echoes it.

- `scenario`: a benchmark route such as `R7-A`, a closure scenario such as `R7-B_late_closure`, or a catalogue scenario such as `c3_r30`.
- `mode`: `estimated` or `ground_truth`.
- The defaults are those of `road_drive drive`: condition `nominal`, estimator `ekf`, seed 1, re-join `unique`, a 2 s alignment before departure.
- A configuration and seed give the episode `road_drive drive` gives, trace for trace.

## Session files

A session file holds everything a replay needs:

```json
{"format": "ane-session", "version": 1, "backend": {"commit": "...", "toolchain": "MSVC 195136260"},
 "config": {...},
 "commands": [{"tick": 1204, "cmd": "gnss_bias", "dx": -0.0013, "dy": 5.0}, {"tick": 3305, "cmd": "closure_add", "lane": "J21>J22", "barrier_s": 5.638, "works": 15}],
 "result": {"t": 108.93, "outcome": "COMPLETED", "trace_hash": "0xeaa75f7426e71a53", "toolchain": "MSVC 195136260"}}
```

`road_drive session-replay FILE` runs it headlessly: the same configuration, with each command at its tick.
- On the recording toolchain the trace must be bit-identical.
- On another toolchain the outcome must match, and the trace hash is reported.

`examples/sandbox_session.json` was recorded through the viewer. ctest replays it as `road_driving_session_replay`.
