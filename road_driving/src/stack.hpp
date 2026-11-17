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
};

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
    enum class Mode { Drive, StopForClosure, Fallback, Hold, HoldFallback, Done, NoRoute, LocalisationStop };
    int believedElement() const { return path_.empty() ? -1 : path_.elements()[idx_]; }

    Stack(const LaneNetwork& net, const VehicleParams& vp, Variant variant, int goalElement, double goalLaneS);
    void initialise(const Observation& obs);  // first planning tick: closures known at departure, initial route
    void plan(const Observation& obs);        // every planning tick (10 Hz)
    Command control(const VehicleState& pose, double t);  // every control tick (50 Hz)

    Mode mode() const { return mode_; }
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
    void track(const VehicleState& pose);
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
};

// Speed caps of a path (20 mph limit and curvature), and the deceleration needed from (s, v) to meet
// every cap within the horizon, after the latency budget. Infinite when a cap is already unreachable.
std::vector<double> speedCaps(const DrivePath& path);
double requiredDecel(const DrivePath& path, const std::vector<double>& caps, double s, double v);

}  // namespace road
