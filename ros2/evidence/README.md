# Simulation evidence

Everything here was produced by running the trials in Gazebo (`ros2/tools/run_trials.py`) and analysing
their recordings (`ros2/ane_sim/scripts/analyze_trial.py`). It is simulation evidence: no physical
robot was involved.

| Path | Contents |
|---|---|
| `summary.json`, `SUMMARY.md` | the published evaluation campaign, aggregated per scenario (`ros2/tools/summarize_trials.py`) |
| `trials/<campaign>/campaign.json` | engine commit, integration-code hash, container image and package versions, host, timing settings, clock checks |
| `trials/<campaign>/index.json` | every attempted trial with its status, in the order run |
| `trials/<campaign>/<scenario>/trial_<k>/manifest.json` | the trial's inputs: scenario file and its SHA-256, seed, initial pose, mode (headless or capture) |
| `.../runner_result.json` | what the scenario runner did and when (simulation, monotonic and system time) |
| `.../metrics.json` | every metric, the acceptance verdicts, the consistency checks and the offline route re-plans |
| `.../trajectory.json` | ground truth at 10 Hz, routes, blockages, faults, the full ordered event log, and the navigator's own status (`ane/status`) at 10 Hz |
| `.../camera_log.csv` | captured trials only: every pose the Gazebo GUI camera was moved to, with the simulation time and the view it served |
| `.../*.sdf` | the obstacles the runner created in Gazebo |
| `figures/` | plots of the evaluation campaign's trials (`ros2/tools/render_figures.py`) |
| `media/` | the recorded runs: previews (GIF), poster frame, full recordings (MP4), each with a provenance record (`.json`) (`ros2/tools/make_video.py`); the software-chain diagram (`ros2/tools/render_chain.py`) |
| `recordings.json` | sizes and SHA-256 of the rosbag2 recordings kept out of Git, and whether they are published |

**Campaigns.**

- `2026-11-09-eval` is the evaluation campaign: 30 headless trials, five per scenario, run on the
  integration code committed with it. Its `campaign.json` holds the hash that `check_sim_evidence.py`
  recomputes from the checkout; `summary.json` and the README's numbers come from it.
- `2026-11-09-capture` holds the two recorded runs (scenario B, the full mission; scenario C, the failure
  case) with the Gazebo GUI and RViz2 rendered in software on a virtual display. They are separate
  trials, analysed like any other, but slower (rendering lowers the real-time factor) and not part of
  the evaluation's statistics; the media show only their own recorded values.
- Earlier campaigns, kept as recorded: `2026-11-08` (development), `2026-11-08-eval` and
  `2026-11-08-capture`, run on an earlier, uncommitted revision of this integration (their
  `campaign.json` records its hash). The navigation code was the same; the recording and visualisation
  code and the world's materials and lighting changed afterwards. All 30 trials of each of the two
  30-trial campaigns met every criterion.

**Engine commit.** The two 2026-11-09 campaigns, and the media made from the capture, record engine
commit `fbe3278`, the identifier of that commit before the published history was condensed. The
published commit with the same tree is `8f786e3`; the earlier campaigns record `5f0fc91`, published as
`3a6630b`. The integration-tree hash is computed from file contents, so it is the same for both
identifiers ([commit identifiers](../../docs/EVIDENCE.md#commit-identifiers)).

**Development runs not kept here.** Before these campaigns, 11 smoke and probe runs, 2 pipeline checks
and 4 test captures were used to find two defects in the experiment set-up, both fixed before the
campaigns: a Gazebo server that survived the end of a trial and fed a second clock and robot into the
next trial in the same container (each trial now runs in a fresh container and kills the server at the
end), and bags whose receive stamps came from a Docker Desktop system clock that is stepped back about
1.3 s every 30 s (bags are now stamped with simulation time). The same clock steps also stopped the
screen recorder in some captures; screen frames are now stamped by their index. Those runs, and the
test captures used to frame the recordings, measured nothing that is reported.

**Recordings.** The rosbag2 files are not in Git. `recordings.json` lists every archive and every bag
with its size and SHA-256, and states whether they have been published; at the time of writing they have
not, and exist only on the machine that ran the trials.

**Check it.**

```bash
python ros2/tools/check_sim_evidence.py --route-check <path to ane_route_check>
```

replays every engine call recorded in the published campaign through this checkout's engine, re-derives
every acceptance verdict from the stored metrics and the committed criteria, and checks the hashes and
the README's numbers. CI runs it on every push.
