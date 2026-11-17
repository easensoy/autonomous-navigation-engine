#pragma once
// Simulated sensors and state estimation for road_driving.
//
// Frames and units. The map frame is the network's (x east, y north, metres). The body frame has its
// origin at the rear-axle centre, x forward and y left. Angles are radians, anticlockwise from map x.
// Time is seconds since the start of the episode. Every measurement carries the time it is valid for
// (tMeas) and the time it reaches the estimator (tArrival >= tMeas).
//
// Separation. The world turns its ground truth into measurements (SensorSimulator). Estimators receive
// only those measurements, in arrival order, and the lane map (used once, for the initial heading of a
// car parked in a lane). Nothing derived from truth reaches an estimator any other way.
#include "road_driving/lane_network.hpp"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace road {

// ---- Measurements -------------------------------------------------------------------------------------
struct Measurement {
    enum Kind { Gnss = 0, Imu = 1, Wheel = 2, Steer = 3 };
    Kind kind = Imu;
    double tMeas = 0.0;
    double tArrival = 0.0;
    // Gnss: antenna x, y (map frame, m) and the receiver's reported 1-sigma (m).
    // Imu: specific force along body x and y (m/s^2) and yaw rate (rad/s) at the IMU.
    // Wheel: rear-axle speed from the wheel encoders (m/s). Steer: road-wheel angle (rad).
    double a = 0.0, b = 0.0, c = 0.0;
};
const char* kindName(Measurement::Kind k);

// ---- Sensor configuration and conditions --------------------------------------------------------------
struct GnssConfig {
    double rate = 10.0;           // Hz
    double latency = 0.10;        // s from time of validity to arrival
    double latencyJitter = 0.02;  // s, uniform +-
    double sigma = 0.25;          // m per axis, white
    double corrSigma = 0.15;      // m per axis, first-order Gauss-Markov
    double corrTau = 60.0;        // s
    double dropProb = 0.02;       // fixes lost at random
    double antennaX = 1.60;       // m, body frame
    double antennaY = 0.0;
    double reportedSigma = 0.30;  // m, what the receiver reports (constant; it does not see multipath)
};
struct ImuConfig {
    double rate = 100.0;
    double leverX = 1.40, leverY = 0.0;  // m, IMU position in the body frame (axes aligned)
    double gyroNoise = 0.0015;           // rad/s, 1-sigma per sample
    double gyroBias = 0.0025;            // rad/s, initial magnitude (sign drawn per seed)
    double gyroBiasWalk = 2e-5;          // rad/s per sqrt(s)
    double accelNoise = 0.03;            // m/s^2, 1-sigma per sample
    double accelBias = 0.04;             // m/s^2, initial magnitude (sign drawn per seed)
    double accelBiasWalk = 1e-4;         // m/s^2 per sqrt(s)
};
struct WheelConfig {
    double rate = 50.0;
    double latency = 0.02;
    double noise = 0.02;        // m/s
    double scaleError = 0.01;   // tyre radius error, magnitude (sign drawn per seed)
    double quantum = 0.005;     // m/s, encoder resolution; reads exactly 0 at rest
};
struct SteerConfig {
    double rate = 50.0;
    double latency = 0.02;
    double noise = 0.002;   // rad
    double offset = 0.004;  // rad, magnitude (sign drawn per seed)
};
// A stretch of the route, as fractions of its length; the world turns it into map-frame rectangles.
struct Span {
    double from = 0.0, to = 0.0;
    bool active() const { return to > from; }
};
struct SensorConditions {
    std::string name = "nominal";
    GnssConfig gnss;
    ImuConfig imu;
    WheelConfig wheel;
    SteerConfig steer;
    Span outage;                    // no GNSS fixes (urban canyon, underpass)
    Span multipath;                 // fixes shifted by a constant offset of this magnitude, extra noise
    double multipathBias = 0.0;     // m (direction drawn per seed)
    double multipathSigma = 0.0;    // m, extra white noise
    Span slip;                      // wheel speed reads (1 + slipRatio) times the truth
    double slipRatio = 0.0;
    double gyroBiasStepTime = -1.0; // s; a step change of the gyro bias at this time (< 0: none)
    double gyroBiasStep = 0.0;      // rad/s
};
// Named conditions. Development: nominal, gnss_outage, multipath, wheel_slip, imu_bias, delay_drop.
// Held out (different placements and magnitudes, plus a combination): the same names with suffix _h,
// and combined_h.
SensorConditions sensorConditions(const std::string& name);
std::vector<std::string> conditionNames(bool heldOut);

struct Zone {
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
    bool contains(double x, double y) const { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
};
struct SensorZones {
    std::vector<Zone> outage, multipath, slip;
};

// World side: truth in, measurements out. Deterministic for a seed.
class SensorSimulator {
public:
    SensorSimulator(const SensorConditions& cond, const SensorZones& zones, std::uint64_t seed, double wheelbase);
    // Called every world tick (dt) with the true rear-axle state at time t. Generates the measurements
    // valid at t and returns, through `out`, every measurement whose arrival time is <= t.
    void step(double t, double dt, double x, double y, double yaw, double v, double steer, double accel, std::vector<Measurement>& out);
    bool gnssAvailable() const { return gnssAvailable_; }  // false inside an outage zone
    bool inMultipath() const { return inMultipath_; }
    bool inSlip() const { return inSlip_; }
    const std::vector<Measurement>& log() const { return log_; }  // every measurement, in arrival order

private:
    struct Rng {
        std::uint64_t state;
        std::uint64_t next();
        double uniform(double a, double b);
        double normal();
    };
    double cadence(double rate) const;
    SensorConditions c_;
    SensorZones zones_;
    Rng rng_;
    double wheelbase_;
    double gyroBias_, accelBias_, wheelScale_, steerOffset_;
    double corrX_ = 0.0, corrY_ = 0.0, mpX_ = 0.0, mpY_ = 0.0;
    double nextGnss_ = 0.0, nextWheel_ = 0.0, nextSteer_ = 0.0, nextImu_ = 0.0;
    double lastYawRate_ = 0.0;
    bool gnssAvailable_ = true, inMultipath_ = false, inSlip_ = false, stepped_ = false;
    std::vector<Measurement> pending_;
    std::vector<Measurement> log_;
};

std::string sensorLogCsv(const std::vector<Measurement>& log);
bool parseSensorLog(const std::string& text, std::vector<Measurement>& out, std::string& error);

// ---- Estimates and estimators ------------------------------------------------------------------------
struct StateEstimate {
    double t = 0.0;
    bool valid = false;
    double x = 0.0, y = 0.0, yaw = 0.0, v = 0.0;
    double accel = 0.0, yawRate = 0.0, steer = 0.0;
    bool hasCov = false;
    double pxx = 0.0, pxy = 0.0, pyy = 0.0, pxyaw = 0.0, pyyaw = 0.0, pyawyaw = 0.0, pvv = 0.0;
    const char* status = "INIT";
    double lastGnssUsed = -1.0;  // time of validity of the last GNSS fix the estimator used
    double sigmaPos() const;     // sqrt of the largest eigenvalue of the position covariance
    double sigmaAlong(double heading) const;  // position 1-sigma along / across a direction
    double sigmaAcross(double heading) const;
};

struct EstimatorStats {
    int gnssUsed = 0, gnssRejected = 0, wheelUsed = 0, wheelRejected = 0, resets = 0, lateDropped = 0, zeroUpdates = 0;
    std::vector<double> nisGnss, nisWheel;  // normalised innovation squared of every candidate update
    std::vector<double> latencyUs;          // wall-clock processing time of each measurement (not deterministic)
};

// Tuned on the development split only, then frozen (LOCALISATION.md).
struct EstimatorConfig {
    double gnssSigma = 0.28;       // m, white measurement noise assumed per axis
    double gnssCorrSigma = 0.15;   // m, the receiver's slowly varying error, estimated as two states
    double gnssCorrTau = 60.0;     // s, its correlation time
    double wheelSigma = 0.03;      // m/s
    double gyroSigma = 0.0015;     // rad/s per sample
    double accelSigma = 0.03;      // m/s^2 per sample
    double posProcess = 0.0006;    // m^2 per metre travelled, unmodelled motion
    double gyroBiasWalk = 1e-4;    // rad/s per sqrt(s)
    double accelBiasWalk = 2e-4;   // m/s^2 per sqrt(s)
    double scaleWalk = 1e-5;       // per sqrt(s)
    double initPosSigma = 1.0;     // m
    double initYawSigma = 0.05;    // rad
    double initGyroBiasSigma = 0.01;   // rad/s
    double initAccelBiasSigma = 0.15;  // m/s^2
    double initScaleSigma = 0.02;
    double gateGnss = 11.0;        // chi-square, 2 dof, about 0.996
    double gateWheel = 10.83;      // chi-square, 1 dof, 0.999
    // Robust GNSS update: a fix whose NIS exceeds this (but passes the gate) has its noise inflated by
    // NIS / robustNis before the update, so near-gate outliers pull the estimate less. <= 0 disables.
    double robustNis = 3.0;        // chi-square, 2 dof, about 0.78
    int resetAfterRejected = 100;  // consecutive rejected fixes (10 s) before GNSS is trusted again
    double bufferSeconds = 1.0;    // delayed measurements older than this are dropped
    double cvAccelSigma = 1.5;     // m/s^2, GNSS-only constant-velocity filter
    // Antenna and IMU positions are vehicle installation data, known to every estimator.
    double antennaX = 1.60, antennaY = 0.0, imuLeverX = 1.40;
};

class Estimator {
public:
    virtual ~Estimator() = default;
    virtual const char* name() const = 0;
    virtual void process(const Measurement& m) = 0;  // in arrival order
    virtual StateEstimate estimateAt(double t) const = 0;
    EstimatorStats stats;
};

// "ekf"      fused GNSS + IMU + wheel extended Kalman filter (9 states, delayed-measurement replay)
// "dr"       wheel + gyro dead reckoning
// "gnss_hold" the latest GNSS fix, held
// "gnss_cv"  constant-velocity Kalman filter on GNSS fixes alone
std::unique_ptr<Estimator> makeEstimator(const std::string& name, const EstimatorConfig& cfg, const LaneNetwork& map);
std::vector<std::string> estimatorNames();
// "key=value,key=value" overrides of EstimatorConfig fields (used only for development tuning runs).
bool applyConfigOverrides(const std::string& text, EstimatorConfig& cfg, std::string& error);
std::string configText(const EstimatorConfig& cfg);

}  // namespace road
