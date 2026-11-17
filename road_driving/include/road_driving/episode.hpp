#pragma once
// One closed-loop episode on network R1: the world (vehicle truth, the closure and its oracle
// notification), the driving stack (engine routing, speed and steering control, the closure executive)
// and an evaluator that reads only the truth log and the scenario.
//
// Labels that apply to every episode: SYNTHETIC_ROADS (network R1, left-hand traffic, no real street),
// GROUND_TRUTH_POSE (the stack reads simulator truth), ORACLE_CLOSURE (the scenario tells the stack
// about the closure; nothing is sensed). There is no other traffic.
#include "road_driving/lane_network.hpp"
#include "road_driving/localisation.hpp"
#include "road_driving/vehicle.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace road {

// Stack constants that tests and the evaluator also need.
namespace stackparams {
constexpr double kLatency = 0.40;       // s: command delay + longitudinal lag, the stack's latency budget
constexpr double kLatPlan = 1.9;        // m/s^2: lateral acceleration behind curvature speed caps
constexpr double kCapTolerance = 0.2;   // m/s: speed above a cap treated as tracking ripple, not infeasibility
constexpr double kStopMargin = 2.0;     // m: planned gap between front bumper and a barrier
constexpr double kEngage = 1.5;         // m/s^2: a planned stop starts once it needs this deceleration
constexpr double kFirmHold = 3.85;      // m/s^2: deceleration ceiling while holding a firm-class stop
constexpr double kEmergencyEvent = 5.0; // m/s^2: an emergency-braking event (UN R157 threshold)
}  // namespace stackparams

enum class Variant { B0, B1 };  // B0 plain replanning; B1 commitment-aware replanning
const char* variantName(Variant v);
bool parseVariant(const std::string& s, Variant& v);

enum class Reveal { None, AtStart, BeforeJunction, BeforeBarrier };
const char* revealName(Reveal r);
bool parseReveal(const std::string& s, Reveal& r);

struct Scenario {
    std::string name = "c0_nominal";
    std::string scenarioClass = "C0";
    Variant variant = Variant::B1;
    std::uint64_t seed = 0;            // 0: nominal start; otherwise seeded jitter of start and notification
    double maxSegment = 20.0;          // R1 lane segmentation, m
    std::string startLane = "J00>J10";
    double startLaneS = 7.5;           // rear axle 20 m east of J00
    std::string goalLane = "J21>J22";
    double goalLaneS = 47.5;           // 60 m north of J21
    bool closure = false;              // closure X1, kind ROADWORKS, reported by the oracle
    std::string closureLane = "J20>J21";
    double barrierLaneS = 27.5;        // barrier line 40 m north of J20
    double worksLength = 15.0;
    Reveal reveal = Reveal::None;
    double revealDistance = 0.0;       // m: front bumper to the junction stop line or to the barrier
    std::string revealJunction = "J10";
    double liftAfterStop = -1.0;       // s after the car comes to rest with the closure in force; < 0 never
    double timeBudget = 300.0;         // s
    // Localisation. With a sensor condition the world simulates GNSS, IMU, wheel and steering sensors
    // and logs them. Mode "ground_truth" drives on simulator truth (the reference); "estimated" drives
    // on the named estimator's output, which sees only the sensor measurements.
    std::string network = "R1";
    std::string condition;              // empty: no sensors
    std::string mode = "ground_truth";  // or "estimated"
    std::string estimator = "ekf";
    double departDelay = 0.0;           // s at rest before the mission starts (initial alignment)
    // The stack's clearance feasibility layer (sideways shift and explicit stop). Off reproduces the stack
    // evaluated on R2 in November 2026; manifests written before it existed replay with it off.
    bool feasibility = true;
    // The stack's re-join after a pose correction (rejoinpolicy). Off reproduces the stack evaluated on R3
    // and R4; manifests written before it existed replay with it off.
    bool rejoin = true;
};

std::vector<Scenario> catalogue();
bool findScenario(const std::string& name, Scenario& out);

struct TruthSample {
    double t, x, y, yaw, v, steer, accel, cmdSteer, cmdAccel;
    int closure;  // 0 none, 1 present and not yet notified, 2 present and notified, 3 lifted
};

struct LogEvent {
    double t;
    std::string source;  // WORLD or STACK
    std::string type;
    std::string detail;
};

struct RouteRecord {
    double t;
    std::string reason, engineState;
    std::vector<int> elements;
};

struct Metrics {
    bool completed = false;
    double timeToGoal = -1.0;
    bool incursion = false;
    double incursionDepth = 0.0;     // m beyond the barrier line, front corners within the closed lane
    double stopMargin = -1.0;        // m from front bumper to barrier at final rest before it; -1 if n/a
    double maxCrossTrackLink = 0.0, maxCrossTrackConnector = 0.0;
    int illegalTransitions = 0;      // moves onto an element that is not a legal successor
    int offLaneSamples = 0;          // no legal element within 1.5 m of the rear axle
    double maxSpeedExcess = 0.0;     // m/s above the 20 mph limit (negative when always below)
    double maxSteer = 0.0, steerRateLimitedShare = 0.0, maxLatAccel = 0.0;
    double maxDecel = 0.0, maxAccel = 0.0, maxJerk = 0.0;
    int emergencyEvents = 0;         // actual deceleration rising above 5.0 m/s^2
    std::vector<int> visited;        // elements associated from truth, in order
    std::vector<int> truthElement;   // the element associated from truth at every truth sample (-1: none)
    double minClearance = 0.0;       // m, smallest footprint (with mirrors) to road-edge distance; < 0 is off the road
    int clearanceViolations = 0;     // truth samples with part of the body (no mirrors) off the carriageway
    double maxCrossing = 0.0;        // m, deepest the body went over the kerb (0: never)
    // Stack decisions, counted from its event log (never used for the outcome)
    int replans = 0, masks = 0, fallbacks = 0, monitorTriggers = 0;
    std::string stopClass;
    // Estimated-state driving: the stack's own element against the truth association, and the
    // driving estimator's error against truth (computed after the run, never fed back).
    int associationSamples = 0, associationErrors = 0;
    int localisationStops = 0, localisationDegraded = 0;
    double estRmse = -1.0, estP95 = -1.0, estMax = -1.0;
    // Stops the stack decided because of its estimate (localisation unreliable, or no clear continuation),
    // judged afterwards against truth: unwarranted when the true position error at the decision was below
    // kUnwarrantedError, i.e. the stop came from reported uncertainty rather than actual error.
    int clearanceStops = 0, unwarrantedStops = 0;
};
constexpr double kUnwarrantedError = 0.3;  // m

struct EpisodeResult {
    Scenario scenario;
    double jitterAlong = 0.0, jitterLateral = 0.0, jitterHeading = 0.0, jitterReveal = 0.0;
    std::string outcome;         // from the evaluator: COMPLETED, STOPPED_AT_CLOSURE, STOPPED_EN_ROUTE, CLOSURE_INCURSION, TIMEOUT
    std::string stackFinalMode;  // the stack's own view, stored and never read by the evaluator
    Metrics metrics;
    std::vector<TruthSample> truth;
    std::vector<LogEvent> events;
    std::vector<RouteRecord> routes;
    std::string truthCsv, eventsCsv, routesCsv;
    std::uint64_t traceHash = 0;  // FNV-1a over truth.csv
    std::uint64_t routeHash = 0;  // FNV-1a over the route decisions, without times
    double simSeconds = 0.0;
    double tReveal = -1.0, tLift = -1.0;
    double engineMs = 0.0;        // wall clock in engine calls; excluded from every hash
    std::uint64_t engineSilencedBytes = 0;
    std::size_t engineNodes = 0, engineEdges = 0;
    // Localisation logs (only with a sensor condition).
    std::vector<Measurement> sensors;                    // arrival order, as the estimators saw them
    std::vector<StateEstimate> estimates;                // estimated mode: the driving estimate at every truth sample
    std::vector<std::pair<double, double>> outages;      // GNSS outage intervals [t0, t1] (world record)
    SensorZones zones;
    std::vector<std::pair<double, int>> belief;          // estimated mode: the stack's element at each control tick
    std::string sensorsCsv, estimatesCsv;
};

EpisodeResult runEpisode(const Scenario& sc);

// Evaluator: reads the truth samples, the scenario and the network only.
Metrics evaluate(const LaneNetwork& net, const VehicleParams& vp, const Scenario& sc,
                 const std::vector<TruthSample>& truth, double tLift, std::string& outcome);

std::uint64_t fnv1a(const std::string& text);
std::string hex64(std::uint64_t v);
std::string toolchain();
std::string engineCommit();
bool engineSourcesModified();

std::string manifestText(const EpisodeResult& r);
bool writeEpisode(const EpisodeResult& r, const std::string& dir, std::string& error);
bool readKeyValues(const std::string& path, std::map<std::string, std::string>& kv, std::string& error);
bool scenarioFromManifest(const std::map<std::string, std::string>& kv, Scenario& sc, std::string& error);
bool readText(const std::string& path, std::string& text);

}  // namespace road
