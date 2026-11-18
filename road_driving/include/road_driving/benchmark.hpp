#pragma once
// Localisation benchmark. Open loop: a drive is recorded once (ground-truth driving, simulated sensors),
// then every estimator processes the same sensor log in the same arrival order and is scored against
// the same truth in the fixed map frame (no per-trajectory alignment). Closed loop: the stack drives on
// an estimator's output and the evaluator measures the consequences from truth.
#include "road_driving/episode.hpp"
#include "road_driving/localisation.hpp"

#include <string>
#include <utility>
#include <vector>

namespace road {

struct LocMetrics {
    std::string estimator;
    bool failed = false;            // never initialised, or position error above kFailError after initialising
    double initTime = -1.0;         // s, first valid estimate
    int samples = 0;                // 50 Hz samples scored (after initialisation)
    double rmse = 0.0, median = 0.0, p95 = 0.0, max = 0.0;  // m, rear-axle position
    double headingRmseDeg = 0.0, headingMaxDeg = 0.0, speedRmse = 0.0;
    double rpe5 = 0.0, rpe20 = 0.0;  // m, RMS error of the displacement over 5 s and 20 s windows
    // The longest GNSS outage: error when GNSS returns, largest error inside it, distance driven inside
    // it, and seconds from its end until the error stays below 0.5 m for 1 s (-1: no outage; 999: never).
    double outageEnd = -1.0, outageMax = -1.0, outageDistance = -1.0, recovery = -1.0;
    int neesDof = 0;                // 3 (x, y, heading) for the EKF; 2 (x, y) for the GNSS-only filter
    double neesMean = -1.0, neesInside = -1.0;  // mean NEES; share of samples inside the 95% chi-square interval
    double nisGnssMean = -1.0, nisGnssInside = -1.0, nisWheelMean = -1.0, nisWheelInside = -1.0;
    int gnssUsed = 0, gnssRejected = 0, wheelRejected = 0, resets = 0, lateDropped = 0;
    double runtimeMs = 0.0, latencyMedianUs = 0.0, latencyP99Us = 0.0, latencyMaxUs = 0.0;  // wall clock
    // Lateral error: the position error across the true heading (what keeps the car off the kerb).
    double lateralRmse = 0.0, lateralP95 = 0.0, lateralMax = 0.0;
    int stepAlarms = 0, faultExits = 0, gnssExcluded = 0, reanchors = 0, kerbUsed = 0, kerbRejected = 0;
    int faultRecoveries = 0, headingReanchors = 0, kerbRelocalisations = 0;  // second versions
    double nisKerbMean = -1.0;
    std::vector<double> errors;     // per-sample position errors (pooled statistics; not serialised)
    std::vector<double> lateral;    // per-sample absolute lateral errors (not serialised)
};
constexpr double kFailError = 5.0;  // m

// Every estimator in `names` (default: estimatorNames()) processes the same sensor log.
std::vector<LocMetrics> evaluateLocalisation(const std::vector<TruthSample>& truth, const std::vector<Measurement>& sensors,
                                             const std::vector<std::pair<double, double>>& outages, const LaneNetwork& net,
                                             const EstimatorConfig& cfg, std::vector<std::vector<StateEstimate>>* traces = nullptr,
                                             const std::vector<std::string>* names = nullptr);
std::string locMetricsJson(const LocMetrics& m);

struct RouteSpec {
    std::string name, network, startLane;
    double startS;
    std::string goalLane;
    double goalS;
};
std::vector<RouteSpec> benchmarkRoutes(bool heldOut);  // three development routes on R1, three held-out on R2
std::vector<RouteSpec> benchmarkRoutesR3();             // three routes on R3
std::vector<RouteSpec> benchmarkRoutesR4();             // three routes on R4
std::vector<RouteSpec> benchmarkRoutesR5();             // three routes on R5
std::vector<RouteSpec> benchmarkRoutesR6();             // three routes on R6
std::vector<RouteSpec> benchmarkRoutesR7();             // three routes on R7
std::vector<RouteSpec> districtRoutes();                // KEN-A (Kensington) and KNB-A (Knightsbridge): district.hpp
Scenario benchmarkScenario(const RouteSpec& route, const std::string& condition, std::uint64_t seed, const std::string& mode);
// Closed-loop scenarios: development uses R1's closure scenarios; held out uses the R2 routes and one
// R2 closure scenario; R3 and R4 their three routes and one closure scenario each.
std::vector<Scenario> closedLoopScenarios(bool heldOut);
std::vector<Scenario> closedLoopScenariosR3();
std::vector<Scenario> closedLoopScenariosR4();
std::vector<Scenario> closedLoopScenariosR5();
std::vector<Scenario> closedLoopScenariosR6();
std::vector<Scenario> closedLoopScenariosR7();
// A catalogue scenario, a benchmark route (R1 to R7), a district route or a closed-loop scenario, by name.
bool findNamedScenario(const std::string& name, Scenario& sc);

// A benchmark split. "dev" (R1) and "heldout" (R2) are the November 2026 evaluation exactly as recorded:
// estimator "ekf" against the baselines, and the stack without the clearance layer. "r2" reuses R2 as
// development and regression data for the integrity and kerb increment; "r3" is its held-out evaluation.
// "r3dev" reuses R3 as development data for the second versions (ekf_int2, ekf_kerb2); "r4" is their
// held-out evaluation. Every one of those splits drives the stack it was evaluated with, so the stack's
// later re-join after pose corrections is off in them. "r5" is the re-join's held-out evaluation: every
// filter drives each episode with and without it. "r6" does the same for the verified re-join, and "r7" for
// the unique re-join, with the kerb filters also driven with the R6 version.
struct PoseSource {
    std::string label, mode, estimator;
    bool feasibility = true;
    Rejoin rejoin = Rejoin::Unique;
};
struct SplitSpec {
    std::string name, title;
    std::vector<RouteSpec> routes;
    std::vector<std::string> conditions;        // open loop
    std::vector<std::string> estimators;        // open loop
    std::vector<Scenario> scenarios;            // closed loop
    std::vector<std::string> closedConditions;  // closed loop
    std::vector<PoseSource> sources;            // closed loop, after the ground-truth reference
    bool referenceFeasibility = false;
    Rejoin referenceRejoin = Rejoin::Verified;  // recorded for the reference; nothing jumps on simulator truth
};
SplitSpec splitSpec(const std::string& name);  // throws for an unknown split

struct CampaignOptions {
    std::string split = "dev";
    int seeds = 5;
    std::string outDir;
    std::string config;  // EstimatorConfig overrides for development tuning; empty for the frozen configuration
    // Closed loop only: run just these scenarios (comma-separated; empty: all). Episodes are independent,
    // so one process per scenario, merged in the split's scenario order by tools/merge_closed_loop.py,
    // gives the same files as one process running them all.
    std::string scenarios;
    // Closed loop only: "off", "on", "verified" or "unique" sets the re-join for every pose source; empty keeps
    // the split's own.
    std::string rejoin;
};
// Open loop. Writes runs.jsonl, summary.md, campaign.txt and a few sensor logs; returns summary.md.
std::string runLocalisationCampaign(const CampaignOptions& opt);
// Closed loop. Writes closed_loop.jsonl and closed_loop.md; returns the latter.
std::string runClosedLoopCampaign(const CampaignOptions& opt);

bool parseTruthCsv(const std::string& text, std::vector<TruthSample>& out);
// Comma-separated "t0:t1" pairs, as stored in a manifest's outage_windows.
std::vector<std::pair<double, double>> parseWindows(const std::string& s);

}  // namespace road
