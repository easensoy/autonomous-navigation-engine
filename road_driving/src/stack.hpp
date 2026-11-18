#pragma once
// The driving stack: route from the engine, closure executive, speed planning, steering. It reads the
// vehicle pose (GROUND_TRUTH) and closure reports (ORACLE) and nothing else from the world.
#include "road_driving/drive_path.hpp"
#include "road_driving/engine_router.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/lane_network.hpp"
#include "road_driving/vehicle.hpp"

#include <string>
#include <vector>

namespace road {

struct ClosureReport {
    std::string id, kind;
    int lane = -1;
    double barrierLaneS = 0.0, worksLength = 0.0;
    std::vector<int> closed;  // lane segments overlapping the works
    bool active = true;       // false once the oracle reports the lift
};

// The estimator's map-verified corrections (StateEstimate::verifiedCorrections, or uniqueCorrections for the
// unique re-join): a count, and the position change of the latest. Zero for simulator truth and for estimators
// without a map reference.
struct VerifiedCorrection {
    int count = 0;
    double dx = 0.0, dy = 0.0;
};

struct Observation {
    double t = 0.0;
    VehicleState pose;                    // GROUND_TRUTH, or the estimator's output when `estimated`
    std::vector<ClosureReport> closures;  // ORACLE: every closure notified so far
    // Estimated-state driving: the estimate's quality, as the estimator reports it (no truth).
    bool estimated = false;
    bool estimateValid = true;
    double estimateAge = 0.0;   // s since the estimate's time of validity
    double sigmaAlong = 0.0;    // m, position 1-sigma along the estimated heading
    double sigmaAcross = 0.0;   // m, position 1-sigma across it
    double sigmaHeading = 0.0;  // rad, heading 1-sigma (0 when the estimator reports none)
    VerifiedCorrection verified;
};

// Clearance feasibility. The body swept along the path ahead must keep from every kerb a margin of
// kSigma times the footprint's lateral 1-sigma plus a tracking allowance. That sigma combines the
// estimate's lateral sigma with its heading sigma acting at the front bumper; on simulator truth it is
// zero. Where the path itself passes too close, the stack shifts it sideways: on turns within the
// carriageway, on lanes only towards the centre line and short of it by the tracking allowance. Where no
// shift is enough it stops before that point and waits until the margin returns.
namespace clearpolicy {
constexpr double kSigma = 2.0;          // lateral 1-sigmas of margin
constexpr double kTrack = 0.10;         // m: tracking allowance (ground-truth driving stays within it)
constexpr double kShiftMax = 0.5;       // m: largest sideways shift on a turn
constexpr double kShiftSlope = 0.03;    // m of shift per m of path (then smoothed)
constexpr double kStep = 0.5;           // m between checked path samples
constexpr double kHorizon = 60.0;       // m checked ahead of the rear axle
constexpr double kResume = 0.05;        // m of spare margin, held for 2 s, to resume after a stop
constexpr double kShiftRate = 0.05;     // m: largest change of the shift at any point per planning tick
}  // namespace clearpolicy

// Pose corrections. An estimate can jump: a kerb re-localisation, a GNSS re-anchor, GNSS returning after
// dead reckoning. The car itself did not move, so following the old reference would turn the correction
// into a steering step, which the steering-rate limit can grow into an oscillation. When the cross-track
// error changes between two control ticks by more than the car can move, the controller's reference moves
// by that change and returns to the path smoothly (a raised cosine). The monitor does not use the re-join:
// a jump may also be a corruption (a filter captured by a biased receiver), so more than 1 m from the path
// still stops the car. Larger corrections than kMax are not re-joined.
namespace rejoinpolicy {
constexpr double kJump = 0.15;    // m of cross-track change in one control tick (20 ms) that is a correction
constexpr double kSlope = 0.05;   // m of re-join per m of path
constexpr double kMinLength = 10.0;  // m
constexpr double kMax = 1.5;      // m: largest re-join
}  // namespace rejoinpolicy

// When the estimate is degraded the stack slows down; when it is unreliable (invalid, stale or too
// uncertain across the lane) the stack stops in lane and waits for it to recover. It never switches to
// another pose source.
namespace locpolicy {
// Chosen on development data: while dead reckoning, the EKF's lateral 1-sigma overstated its actual
// error about threefold, so a 0.75 m sigma corresponds to roughly 0.25 m of actual lateral error.
constexpr double kSlowAcross = 0.35, kSlowAlong = 1.5;  // m: above either, cap the speed
constexpr double kStopAcross = 0.75, kStopAlong = 3.0;  // m: above either, stop
constexpr double kResumeAcross = 0.30;                  // m: resume after 2 s below this
constexpr double kMaxAge = 0.25;                        // s
constexpr double kDegradedSpeed = 5.0;                  // m/s
constexpr double kStopDecel = 2.5;                      // m/s^2
}  // namespace locpolicy

class Stack {
public:
    enum class Mode { Drive, StopForClosure, Fallback, Hold, HoldFallback, Done, NoRoute, LocalisationStop, ClearanceStop };
    int believedElement() const { return path_.empty() ? -1 : path_.elements()[idx_]; }
    double pathShiftAt(double s) const;  // m, current sideways shift of the path (positive left)
    double currentShift() const { return pathShiftAt(s_); }
    double rejoinAt(double s) const;     // m, sideways offset of the reference after a pose correction
    double monitorOffset() const;        // m, sideways offset of the monitor's reference at the car

    // rejoin: the stack's response to a pose correction (Rejoin in episode.hpp).
    Stack(const LaneNetwork& net, const VehicleParams& vp, Variant variant, int goalElement, double goalLaneS, bool feasibility = true,
          Rejoin rejoin = Rejoin::Unique);
    void initialise(const Observation& obs);  // first planning tick: closures known at departure, initial route
    void plan(const Observation& obs);        // every planning tick (10 Hz)
    Command control(const VehicleState& pose, double t, const VerifiedCorrection& verified = {});  // every control tick (50 Hz)

    Mode mode() const { return mode_; }
    // Read-only views for live display (road_drive live); nothing reads them back into the stack.
    const DrivePath& path() const { return path_; }
    double pathS() const { return s_; }
    const char* modeName() const;
    double engineMs() const { return router_.engineMilliseconds(); }
    std::uint64_t silencedBytes() const { return router_.silencedBytes(); }
    std::size_t engineNodes() const { return router_.nodeCount(); }
    std::size_t engineEdges() const { return router_.edgeCount(); }

    std::vector<LogEvent> events;
    std::vector<RouteRecord> routes;

private:
    struct Mask {
        int connector;
        int approachLane;
    };

    void event(const std::string& type, const std::string& detail);
    void track(const VehicleState& pose, const VerifiedCorrection& verified);
    void adopt(const DrivePath& path, double s, const std::string& reason);
    void replan(const std::string& reason, double v);
    void recover(const std::string& reason);
    void planClosureStop(const std::string& reason, double v);
    void fallback(const std::string& reason);
    void computeEnvelope();
    double envelopeAt(double s) const;
    double barrierS(const DrivePath& path, const ClosureReport& c) const;  // path s of the barrier line, -1 if absent
    double nearestBarrierAhead(const DrivePath& path, double sRear) const;
    bool entersKnownClosure(const DrivePath& path, double sRear) const;
    int currentElement() const;

    const LaneNetwork& net_;
    VehicleParams vp_;
    Variant variant_;
    int goal_;
    double goalLaneS_;
    EngineRouter router_;

    DrivePath path_;
    std::vector<double> caps_;
    double s_ = 0.0;
    double lateral_ = 0.0;
    size_t idx_ = 0;
    double v_ = 0.0;
    double a_ = 0.0;  // actual longitudinal acceleration (ground truth)
    double t_ = 0.0;
    Mode mode_ = Mode::Drive;
    bool engineStale_ = false;

    bool goalOnPath_ = false;
    double goalS_ = 0.0;
    bool goalLatched_ = false;

    bool stopActive_ = false;
    double stopS_ = 0.0;
    double stopLimit_ = 0.0;
    bool stopLatched_ = false;
    std::string stopClass_;
    std::string stopClosure_;

    std::vector<ClosureReport> known_;
    std::vector<Mask> masks_;

    double aEnv_ = 1.8;
    std::vector<double> env_;
    size_t envFrom_ = 0;

    void localisationPolicy(const Observation& obs);
    double speedCap_ = 1e9;
    bool degraded_ = false;
    Mode beforeLocStop_ = Mode::Drive;
    double goodSince_ = -1.0;

    // Clearance feasibility (clearpolicy).
    void computeClearance();               // per-sample side clearances of the adopted path
    void clearancePolicy(const Observation& obs);
    bool feasibility_ = true;
    std::vector<double> clrLeft_, clrRight_;  // body-to-kerb clearance of each side, rear axle on the path
    std::vector<char> clrTurn_;               // sample lies on a turning movement
    std::vector<double> shift_;               // sideways shift per sample (positive left)
    double shiftS_ = -1.0;                    // path s at the previous clearance tick (-1: none since adoption)
    Mode beforeClearStop_ = Mode::Drive;
    double clearStopS_ = 0.0, clearSince_ = -1.0;
    bool clearLatched_ = false;

    // Pose corrections (rejoinpolicy).
    Rejoin rejoin_ = Rejoin::Unique;
    int lastVerified_ = 0;  // the estimator's verified-correction count at the previous tick
    double rejoinS0_ = 0.0, rejoinD0_ = 0.0, rejoinLength_ = 0.0;
    double lastErr_ = 0.0;     // cross-track error from the reference at the previous tick
    bool haveLastErr_ = false;
};

// Speed caps of a path (20 mph limit and curvature), and the deceleration needed from (s, v) to meet
// every cap within the horizon, after the latency budget. Infinite when a cap is already unreachable.
std::vector<double> speedCaps(const DrivePath& path);
double requiredDecel(const DrivePath& path, const std::vector<double>& caps, double s, double v);

}  // namespace road
