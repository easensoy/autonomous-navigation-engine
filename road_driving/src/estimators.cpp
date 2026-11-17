// Estimators. Each receives measurements in arrival order and the lane map (for one thing: the initial
// heading of a car parked in a lane). See LOCALISATION.md for the model, its assumptions and safeguards.
#include "road_driving/localisation.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <sstream>
#include <string>

namespace road {

double StateEstimate::sigmaPos() const {
    if (!hasCov) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double tr = 0.5 * (pxx + pyy), det = pxx * pyy - pxy * pxy;
    return std::sqrt(std::max(0.0, tr + std::sqrt(std::max(0.0, tr * tr - det))));
}

double StateEstimate::sigmaAlong(double h) const {
    const double c = std::cos(h), s = std::sin(h);
    return std::sqrt(std::max(0.0, c * c * pxx + 2.0 * c * s * pxy + s * s * pyy));
}

double StateEstimate::sigmaAcross(double h) const { return sigmaAlong(h + 0.5 * kPi); }

namespace {

constexpr int N = 9;  // x, y, yaw, v, gyro bias, accel bias, wheel scale error, slow GNSS error (x, y)
enum { PX = 0, PY = 1, YAW = 2, VEL = 3, BG = 4, BA = 5, SW = 6, GX = 7, GY = 8 };
using Vec = std::array<double, N>;
using Mat = std::array<std::array<double, N>, N>;

Mat zeros() {
    Mat m{};
    return m;
}

bool finite(const Vec& x, const Mat& P) {
    for (int i = 0; i < N; ++i) {
        if (!std::isfinite(x[i]) || !std::isfinite(P[i][i]) || P[i][i] < 0.0) {
            return false;
        }
    }
    return true;
}

void symmetrise(Mat& P) {
    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            const double m = 0.5 * (P[i][j] + P[j][i]);
            P[i][j] = P[j][i] = m;
        }
        P[i][i] = std::max(P[i][i], 1e-12);
    }
}

// Kalman update for a measurement of dimension M (1 or 2), Joseph form. Returns the NIS (normalised
// innovation squared); applies the update only when nis <= gate (or the gate is negative).
template <int M>
double kalmanUpdate(Vec& x, Mat& P, const double (&H)[M][N], const double (&y)[M], const double (&R)[M][M], double gate, bool& applied) {
    applied = false;
    double PHt[N][M];
    for (int i = 0; i < N; ++i) {
        for (int k = 0; k < M; ++k) {
            double s = 0.0;
            for (int j = 0; j < N; ++j) {
                s += P[i][j] * H[k][j];
            }
            PHt[i][k] = s;
        }
    }
    double S[M][M];
    for (int a = 0; a < M; ++a) {
        for (int b = 0; b < M; ++b) {
            double s = R[a][b];
            for (int j = 0; j < N; ++j) {
                s += H[a][j] * PHt[j][b];
            }
            S[a][b] = s;
        }
    }
    // Inverse of S by its closed form, after a positive-definiteness check (Cholesky criterion).
    double Si[M][M];
    if (M == 1) {
        if (!(S[0][0] > 1e-15)) return std::numeric_limits<double>::infinity();
        Si[0][0] = 1.0 / S[0][0];
    } else {
        const double det = S[0][0] * S[M - 1][M - 1] - S[0][M - 1] * S[M - 1][0];
        if (!(S[0][0] > 1e-15) || !(det > 1e-24)) return std::numeric_limits<double>::infinity();
        Si[0][0] = S[M - 1][M - 1] / det;
        Si[M - 1][M - 1] = S[0][0] / det;
        Si[0][M - 1] = -S[0][M - 1] / det;
        Si[M - 1][0] = -S[M - 1][0] / det;
    }
    double nis = 0.0;
    for (int a = 0; a < M; ++a) {
        for (int b = 0; b < M; ++b) {
            nis += y[a] * Si[a][b] * y[b];
        }
    }
    if (gate >= 0.0 && nis > gate) {
        return nis;
    }
    double K[N][M];
    for (int i = 0; i < N; ++i) {
        for (int b = 0; b < M; ++b) {
            double s = 0.0;
            for (int a = 0; a < M; ++a) {
                s += PHt[i][a] * Si[a][b];
            }
            K[i][b] = s;
        }
    }
    for (int i = 0; i < N; ++i) {
        for (int a = 0; a < M; ++a) {
            x[i] += K[i][a] * y[a];
        }
    }
    // Joseph form: P = (I - KH) P (I - KH)^T + K R K^T, which stays symmetric positive semi-definite.
    Mat A = zeros();
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            double s = (i == j) ? 1.0 : 0.0;
            for (int a = 0; a < M; ++a) {
                s -= K[i][a] * H[a][j];
            }
            A[i][j] = s;
        }
    }
    Mat AP = zeros(), Pn = zeros();
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            double s = 0.0;
            for (int k = 0; k < N; ++k) s += A[i][k] * P[k][j];
            AP[i][j] = s;
        }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j) {
            double s = 0.0;
            for (int k = 0; k < N; ++k) s += AP[i][k] * A[j][k];
            for (int a = 0; a < M; ++a)
                for (int b = 0; b < M; ++b) s += K[i][a] * R[a][b] * K[j][b];
            Pn[i][j] = s;
        }
    P = Pn;
    symmetrise(P);
    applied = true;
    return nis;
}

// The heading of the lane nearest to a point: the car starts parked in a lane, facing along it.
double laneHeadingAt(const LaneNetwork& map, double x, double y) {
    double best = std::numeric_limits<double>::infinity(), yaw = 0.0;
    for (const Element& e : map.elements) {
        if (e.isConnector()) {
            continue;
        }
        const double d = distanceToElement(e, x, y);
        if (d < best) {
            best = d;
            yaw = e.pts.front().yaw;
        }
    }
    return yaw;
}

struct InitialPose {
    double x, y, yaw;
};
// Every estimator starts from the same information: its first GNSS fix, the antenna lever arm, and
// the heading of the lane the fix lies in.
InitialPose initialPose(const LaneNetwork& map, const EstimatorConfig& cfg, const Measurement& fix) {
    const double yaw = laneHeadingAt(map, fix.a, fix.b);
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {fix.a - (c * cfg.antennaX - s * cfg.antennaY), fix.b - (s * cfg.antennaX + c * cfg.antennaY), yaw};
}

class Timer {
public:
    explicit Timer(EstimatorStats& st) : st_(st), t0_(std::chrono::steady_clock::now()) {}
    ~Timer() { st_.latencyUs.push_back(std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0_).count()); }

private:
    EstimatorStats& st_;
    std::chrono::steady_clock::time_point t0_;
};

// ---- Extended Kalman filter ---------------------------------------------------------------------------
// State: map-frame rear-axle position (x, y), heading, speed, gyro bias, accelerometer bias, wheel-speed
// scale error, and the receiver's slowly varying error (first-order Gauss-Markov, x and y). The IMU drives the prediction (yaw rate and longitudinal specific force, corrected for
// the IMU lever arm); wheel speed and GNSS position (at the antenna) are measurements. Measurements are
// applied at their time of validity: a late one rewinds the filter to that time and replays everything
// after it.
class Ekf : public Estimator {
public:
    Ekf(const EstimatorConfig& cfg, const LaneNetwork& map) : cfg_(cfg), map_(map) {}
    const char* name() const override { return "ekf"; }

    void process(const Measurement& m) override {
        Timer timer(stats);
        if (!init_) {
            if (m.kind == Measurement::Gnss) {
                initialise(m);
            }
            return;
        }
        if (m.tMeas < cur_.t - cfg_.bufferSeconds) {
            ++stats.lateDropped;
            return;
        }
        if (m.tMeas >= cur_.t) {
            buf_.push_back({m, cur_});
            apply(cur_, m);
        } else {
            auto it = std::upper_bound(buf_.begin(), buf_.end(), m.tMeas, [](double t, const Entry& e) { return t < e.m.tMeas; });
            if (it == buf_.begin() && (buf_.empty() || buf_.front().before.t > m.tMeas)) {
                ++stats.lateDropped;  // older than anything we can rewind to
                return;
            }
            std::vector<Measurement> tail{m};
            for (auto j = it; j != buf_.end(); ++j) {
                tail.push_back(j->m);
            }
            cur_ = (it == buf_.end()) ? cur_ : it->before;
            buf_.erase(it, buf_.end());
            for (const Measurement& r : tail) {
                buf_.push_back({r, cur_});
                apply(cur_, r);
            }
        }
        while (!buf_.empty() && buf_.front().m.tMeas < cur_.t - cfg_.bufferSeconds) {
            buf_.pop_front();
        }
        if (!finite(cur_.x, cur_.P)) {  // numerical failure: start again from the next fix
            ++stats.resets;
            init_ = false;
            buf_.clear();
        }
    }

    StateEstimate estimateAt(double t) const override {
        StateEstimate e;
        e.t = t;
        if (!init_) {
            return e;
        }
        Core c = cur_;
        predictTo(c, t);
        const Vec& x = c.x;
        e.valid = true;
        e.x = x[PX];
        e.y = x[PY];
        e.yaw = x[YAW];
        e.v = x[VEL];
        e.yawRate = c.wz - x[BG];
        e.accel = c.ax + e.yawRate * e.yawRate * cfg_.imuLeverX - x[BA];
        e.steer = c.steer;
        e.hasCov = true;
        e.pxx = c.P[PX][PX];
        e.pxy = c.P[PX][PY];
        e.pyy = c.P[PY][PY];
        e.pxyaw = c.P[PX][YAW];
        e.pyyaw = c.P[PY][YAW];
        e.pyawyaw = c.P[YAW][YAW];
        e.pvv = c.P[VEL][VEL];
        e.lastGnssUsed = c.lastGnss;
        const double sp = e.sigmaPos();
        e.status = sp > 2.0 ? "LOST" : (sp > 0.5 ? "DEGRADED" : (t - c.lastGnss > 2.0 ? "DEAD_RECKONING" : "OK"));
        return e;
    }

private:
    struct Core {
        double t = 0.0;
        Vec x{};
        Mat P{};
        double wz = 0.0, ax = 0.0, steer = 0.0;
        bool haveImu = false;
        double wheel = -1.0, stillSince = -1.0, lastGnss = -1.0;
        int rejectRun = 0;
    };
    struct Entry {
        Measurement m;
        Core before;  // the filter state just before m was applied
    };

    void initialise(const Measurement& fix) {
        const InitialPose p = initialPose(map_, cfg_, fix);
        cur_ = Core{};
        cur_.t = fix.tMeas;
        cur_.x = {p.x, p.y, p.yaw, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        const double d[N] = {cfg_.initPosSigma, cfg_.initPosSigma, cfg_.initYawSigma, 0.1, cfg_.initGyroBiasSigma, cfg_.initAccelBiasSigma,
                             cfg_.initScaleSigma, cfg_.gnssCorrSigma, cfg_.gnssCorrSigma};
        for (int i = 0; i < N; ++i) {
            cur_.P[i][i] = d[i] * d[i];
        }
        cur_.lastGnss = fix.tMeas;
        buf_.clear();
        init_ = true;
    }

    // Propagate with the held IMU sample (zero-order hold between IMU samples).
    void predictTo(Core& c, double t) const {
        const double dt = t - c.t;
        if (dt <= 0.0) {
            return;
        }
        Vec& x = c.x;
        const double l = cfg_.imuLeverX;
        const double w = c.haveImu ? c.wz - x[BG] : 0.0;
        const double a = c.haveImu ? c.ax + w * w * l - x[BA] : 0.0;
        const double ym = x[YAW] + 0.5 * w * dt;
        const double cs = std::cos(ym), sn = std::sin(ym), v = x[VEL];
        Mat F = zeros();
        for (int i = 0; i < N; ++i) F[i][i] = 1.0;
        F[PX][YAW] = -v * sn * dt;
        F[PX][VEL] = cs * dt;
        F[PX][BG] = 0.5 * v * sn * dt * dt;
        F[PY][YAW] = v * cs * dt;
        F[PY][VEL] = sn * dt;
        F[PY][BG] = -0.5 * v * cs * dt * dt;
        if (c.haveImu) {
            F[YAW][BG] = -dt;
            F[VEL][BG] = -2.0 * w * l * dt;
            F[VEL][BA] = -dt;
        }
        x[PX] += v * cs * dt;
        x[PY] += v * sn * dt;
        x[YAW] = wrapAngle(x[YAW] + w * dt);
        x[VEL] = std::max(0.0, x[VEL] + a * dt);  // no reverse
        const double phi = std::exp(-dt / cfg_.gnssCorrTau);
        F[GX][GX] = F[GY][GY] = phi;
        x[GX] *= phi;
        x[GY] *= phi;
        // P = F P F^T + Q
        Mat FP = zeros(), Pn = zeros();
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) {
                double s = 0.0;
                for (int k = 0; k < N; ++k) s += F[i][k] * c.P[k][j];
                FP[i][j] = s;
            }
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j) {
                double s = 0.0;
                for (int k = 0; k < N; ++k) s += FP[i][k] * F[j][k];
                Pn[i][j] = s;
            }
        // White IMU noise integrates to a random walk: per-sample sigma^2 x sample period x dt.
        const double imuPeriod = 0.01;
        // Unmodelled motion (lateral slip, path effects) accrues with distance travelled, not with time:
        // a car at rest gains no position uncertainty.
        Pn[PX][PX] += cfg_.posProcess * std::fabs(v) * dt;
        Pn[PY][PY] += cfg_.posProcess * std::fabs(v) * dt;
        Pn[YAW][YAW] += cfg_.gyroSigma * cfg_.gyroSigma * imuPeriod * dt;
        Pn[VEL][VEL] += cfg_.accelSigma * cfg_.accelSigma * imuPeriod * dt;
        Pn[BG][BG] += cfg_.gyroBiasWalk * cfg_.gyroBiasWalk * dt;
        Pn[BA][BA] += cfg_.accelBiasWalk * cfg_.accelBiasWalk * dt;
        Pn[SW][SW] += cfg_.scaleWalk * cfg_.scaleWalk * dt;
        Pn[GX][GX] += cfg_.gnssCorrSigma * cfg_.gnssCorrSigma * (1.0 - phi * phi);
        Pn[GY][GY] += cfg_.gnssCorrSigma * cfg_.gnssCorrSigma * (1.0 - phi * phi);
        c.P = Pn;
        symmetrise(c.P);
        c.t = t;
    }

    void apply(Core& c, const Measurement& m) {
        predictTo(c, m.tMeas);
        bool applied = false;
        switch (m.kind) {
            case Measurement::Imu: {
                c.wz = m.c;
                c.ax = m.a;
                c.haveImu = true;
                // Standstill: encoders silent for 0.2 s and the gyro quiet. Then the speed is zero, the gyro
                // reads its bias and the accelerometer its bias (flat ground): zero-velocity updates.
                if (c.wheel == 0.0 && c.stillSince >= 0.0 && m.tMeas - c.stillSince >= 0.2 && std::fabs(m.c) < 0.03) {
                    const double Hv[1][N] = {{0, 0, 0, 1, 0, 0, 0, 0, 0}}, Hg[1][N] = {{0, 0, 0, 0, 1, 0, 0, 0, 0}},
                                 Ha[1][N] = {{0, 0, 0, 0, 0, 1, 0, 0, 0}};
                    const double yv[1] = {-c.x[VEL]}, yg[1] = {m.c - c.x[BG]}, ya[1] = {m.a - c.x[BA]};
                    const double Rv[1][1] = {{1e-4}}, Rg[1][1] = {{cfg_.gyroSigma * cfg_.gyroSigma}},
                                 Ra[1][1] = {{cfg_.accelSigma * cfg_.accelSigma}};
                    kalmanUpdate<1>(c.x, c.P, Hv, yv, Rv, -1.0, applied);
                    kalmanUpdate<1>(c.x, c.P, Hg, yg, Rg, -1.0, applied);
                    kalmanUpdate<1>(c.x, c.P, Ha, ya, Ra, -1.0, applied);
                    ++stats.zeroUpdates;
                }
                break;
            }
            case Measurement::Wheel: {
                if (m.a == 0.0) {
                    if (c.wheel != 0.0) c.stillSince = m.tMeas;
                } else {
                    c.stillSince = -1.0;
                }
                c.wheel = m.a;
                const double H[1][N] = {{0, 0, 0, 1.0 + c.x[SW], 0, 0, c.x[VEL], 0, 0}};
                const double y[1] = {m.a - c.x[VEL] * (1.0 + c.x[SW])};
                const double q = 0.005;  // encoder resolution
                const double R[1][1] = {{cfg_.wheelSigma * cfg_.wheelSigma + q * q / 12.0}};
                const double nis = kalmanUpdate<1>(c.x, c.P, H, y, R, cfg_.gateWheel, applied);
                stats.nisWheel.push_back(nis);
                ++(applied ? stats.wheelUsed : stats.wheelRejected);
                break;
            }
            case Measurement::Gnss: {
                const double cy = std::cos(c.x[YAW]), sy = std::sin(c.x[YAW]);
                const double lx = cfg_.antennaX, ly = cfg_.antennaY;
                const double H[2][N] = {{1, 0, -sy * lx - cy * ly, 0, 0, 0, 0, 1, 0}, {0, 1, cy * lx - sy * ly, 0, 0, 0, 0, 0, 1}};
                const double y[2] = {m.a - (c.x[PX] + cy * lx - sy * ly + c.x[GX]), m.b - (c.x[PY] + sy * lx + cy * ly + c.x[GY])};
                const double r2 = cfg_.gnssSigma * cfg_.gnssSigma;
                const double R[2][2] = {{r2, 0.0}, {0.0, r2}};
                Vec xs = c.x;
                Mat Ps = c.P;
                bool probe = false;
                const double nis = kalmanUpdate<2>(xs, Ps, H, y, R, 0.0, probe);  // NIS only (copies)
                stats.nisGnss.push_back(nis);
                if (nis <= cfg_.gateGnss) {
                    // Robust update: in the outer tail, inflate the noise in proportion to the NIS.
                    const double scale = (cfg_.robustNis > 0.0 && nis > cfg_.robustNis) ? nis / cfg_.robustNis : 1.0;
                    const double Rr[2][2] = {{r2 * scale, 0.0}, {0.0, r2 * scale}};
                    kalmanUpdate<2>(c.x, c.P, H, y, Rr, -1.0, applied);
                }
                if (!applied && ++c.rejectRun >= cfg_.resetAfterRejected) {
                    // GNSS has disagreed for longer than odometry can be trusted: reopen the position
                    // uncertainty to cover the disagreement and accept the fix.
                    const double e2 = y[0] * y[0] + y[1] * y[1];
                    for (int i = 0; i < N; ++i) {
                        if (i != PX && i != PY) {
                            c.P[PX][i] = c.P[i][PX] = 0.0;
                            c.P[PY][i] = c.P[i][PY] = 0.0;
                        }
                    }
                    c.P[PX][PY] = c.P[PY][PX] = 0.0;
                    c.P[PX][PX] = c.P[PY][PY] = e2 + r2;
                    kalmanUpdate<2>(c.x, c.P, H, y, R, -1.0, applied);
                    ++stats.resets;
                }
                if (applied) {
                    c.rejectRun = 0;
                    c.lastGnss = m.tMeas;
                    ++stats.gnssUsed;
                } else {
                    ++stats.gnssRejected;
                }
                break;
            }
            case Measurement::Steer:
                c.steer = m.a;
                break;
        }
        c.x[YAW] = wrapAngle(c.x[YAW]);
        c.x[VEL] = std::max(0.0, c.x[VEL]);
    }

    EstimatorConfig cfg_;
    const LaneNetwork& map_;
    bool init_ = false;
    Core cur_;
    std::deque<Entry> buf_;
};

// ---- Dead reckoning: wheel speed and gyro, no GNSS after the first fix ---------------------------------
class DeadReckoning : public Estimator {
public:
    DeadReckoning(const EstimatorConfig& cfg, const LaneNetwork& map) : cfg_(cfg), map_(map) {}
    const char* name() const override { return "dr"; }

    void process(const Measurement& m) override {
        Timer timer(stats);
        State& s = s_;
        if (!s.init) {
            if (m.kind == Measurement::Gnss) {
                const InitialPose p = initialPose(map_, cfg_, m);
                s.x = p.x;
                s.y = p.y;
                s.yaw = p.yaw;
                s.t = m.tMeas;
                s.init = true;
            }
            return;
        }
        switch (m.kind) {
            case Measurement::Imu:
                advance(s, m.tMeas);
                s.wz = m.c;
                s.ax = m.a;
                // Gyro bias calibration from standstill (same rule as the EKF's).
                if (s.wheel == 0.0 && s.stillSince >= 0.0 && m.tMeas - s.stillSince >= 0.2 && std::fabs(m.c) < 0.03) {
                    s.biasSum += m.c;
                    ++s.biasN;
                    s.bg = s.biasSum / static_cast<double>(s.biasN);
                }
                s.haveImu = true;
                break;
            case Measurement::Wheel: {
                advance(s, std::max(s.t, m.tMeas));
                // The speed applies from its time of validity: correct the distance already integrated.
                const double late = std::max(0.0, s.t - m.tMeas);
                const double prev = std::max(0.0, s.wheel);
                s.x += (m.a - prev) * late * std::cos(s.yaw);
                s.y += (m.a - prev) * late * std::sin(s.yaw);
                if (m.a == 0.0) {
                    if (s.wheel != 0.0) s.stillSince = m.tMeas;
                } else {
                    s.stillSince = -1.0;
                }
                s.wheel = m.a;
                break;
            }
            case Measurement::Steer:
                s.steer = m.a;
                break;
            case Measurement::Gnss:
                break;
        }
    }

    StateEstimate estimateAt(double t) const override {
        StateEstimate e;
        e.t = t;
        if (!s_.init) {
            return e;
        }
        State c = s_;
        advance(c, t);
        e.valid = true;
        e.x = c.x;
        e.y = c.y;
        e.yaw = c.yaw;
        e.v = c.wheel > 0.0 ? c.wheel : 0.0;
        e.yawRate = c.wz - c.bg;
        e.accel = c.ax;
        e.steer = c.steer;
        e.status = "DEAD_RECKONING";
        return e;
    }

private:
    struct State {
        bool init = false, haveImu = false;
        double t = 0.0, x = 0.0, y = 0.0, yaw = 0.0, wz = 0.0, ax = 0.0, steer = 0.0, bg = 0.0;
        double wheel = -1.0;  // unknown until the first encoder reading
        double stillSince = -1.0, biasSum = 0.0;
        long biasN = 0;
    };
    static void advance(State& s, double t) {
        const double dt = t - s.t;
        if (dt <= 0.0) {
            return;
        }
        const double w = s.haveImu ? s.wz - s.bg : 0.0;
        const double ym = s.yaw + 0.5 * w * dt;
        const double v = std::max(0.0, s.wheel);
        s.x += v * std::cos(ym) * dt;
        s.y += v * std::sin(ym) * dt;
        s.yaw = wrapAngle(s.yaw + w * dt);
        s.t = t;
    }
    EstimatorConfig cfg_;
    const LaneNetwork& map_;
    State s_;
};

// ---- GNSS only: the latest fix, held ------------------------------------------------------------------
class GnssHold : public Estimator {
public:
    GnssHold(const EstimatorConfig& cfg, const LaneNetwork& map) : cfg_(cfg), map_(map) {}
    const char* name() const override { return "gnss_hold"; }
    void process(const Measurement& m) override {
        Timer timer(stats);
        if (m.kind != Measurement::Gnss) {
            return;
        }
        if (!init_) {
            yaw_ = initialPose(map_, cfg_, m).yaw;
            init_ = true;
        } else if (m.tMeas > tFix_) {
            const double dx = m.a - fx_, dy = m.b - fy_, d = std::hypot(dx, dy);
            if (d > 1.0) {  // heading from fix-to-fix motion, once it exceeds the fix noise
                yaw_ = std::atan2(dy, dx);
            }
            v_ = d / (m.tMeas - tFix_);
        } else {
            ++stats.lateDropped;
            return;
        }
        fx_ = m.a;
        fy_ = m.b;
        tFix_ = m.tMeas;
        ++stats.gnssUsed;
    }
    StateEstimate estimateAt(double t) const override {
        StateEstimate e;
        e.t = t;
        if (!init_) {
            return e;
        }
        const double c = std::cos(yaw_), s = std::sin(yaw_);
        e.valid = true;
        e.x = fx_ - (c * cfg_.antennaX - s * cfg_.antennaY);
        e.y = fy_ - (s * cfg_.antennaX + c * cfg_.antennaY);
        e.yaw = yaw_;
        e.v = v_;
        e.lastGnssUsed = tFix_;
        e.status = "GNSS_HOLD";
        return e;
    }

private:
    EstimatorConfig cfg_;
    const LaneNetwork& map_;
    bool init_ = false;
    double fx_ = 0.0, fy_ = 0.0, tFix_ = -1.0, yaw_ = 0.0, v_ = 0.0;
};

// ---- GNSS only: constant-velocity Kalman filter on the antenna position --------------------------------
class GnssCv : public Estimator {
public:
    GnssCv(const EstimatorConfig& cfg, const LaneNetwork& map) : cfg_(cfg), map_(map) {}
    const char* name() const override { return "gnss_cv"; }
    void process(const Measurement& m) override {
        Timer timer(stats);
        if (m.kind != Measurement::Gnss) {
            return;
        }
        if (!init_) {
            yaw_ = initialPose(map_, cfg_, m).yaw;
            s_ = {m.a, m.b, 0.0, 0.0};
            P_ = {};
            P_[0][0] = P_[1][1] = cfg_.gnssSigma * cfg_.gnssSigma;
            P_[2][2] = P_[3][3] = 0.25;
            t_ = m.tMeas;
            init_ = true;
            return;
        }
        if (m.tMeas < t_) {
            ++stats.lateDropped;
            return;
        }
        predict(s_, P_, m.tMeas - t_);
        t_ = m.tMeas;
        // Update x and y separately (independent axes, R diagonal), with a joint 2-dof gate.
        const double r2 = cfg_.gnssSigma * cfg_.gnssSigma;
        const double yx = m.a - s_[0], yy = m.b - s_[1];
        const double Sx = P_[0][0] + r2, Sy = P_[1][1] + r2, Sxy = P_[0][1];
        const double det = Sx * Sy - Sxy * Sxy;
        const double nis = (Sy * yx * yx - 2.0 * Sxy * yx * yy + Sx * yy * yy) / det;
        stats.nisGnss.push_back(nis);
        bool accept = nis <= cfg_.gateGnss;
        if (!accept && ++rejectRun_ >= cfg_.resetAfterRejected) {
            P_[0][0] = P_[1][1] = yx * yx + yy * yy + r2;
            P_[0][1] = P_[1][0] = 0.0;
            accept = true;
            ++stats.resets;
        }
        if (!accept) {
            ++stats.gnssRejected;
            return;
        }
        rejectRun_ = 0;
        // The same robust weighting as the EKF's: a fix in the outer tail counts for less.
        const double rr = r2 * ((cfg_.robustNis > 0.0 && nis > cfg_.robustNis && nis <= cfg_.gateGnss) ? nis / cfg_.robustNis : 1.0);
        for (size_t p = 0; p < 2; ++p) {
            const double S = P_[p][p] + rr, innov = p == 0 ? m.a - s_[0] : m.b - s_[1];
            double K[4];
            for (size_t i = 0; i < 4; ++i) K[i] = P_[i][p] / S;
            for (size_t i = 0; i < 4; ++i) s_[i] += K[i] * innov;
            std::array<std::array<double, 4>, 4> Pn = P_;
            for (size_t i = 0; i < 4; ++i)
                for (size_t j = 0; j < 4; ++j) Pn[i][j] = P_[i][j] - K[i] * P_[p][j];
            P_ = Pn;
        }
        lastFix_ = m.tMeas;
        ++stats.gnssUsed;
    }
    StateEstimate estimateAt(double t) const override {
        StateEstimate e;
        e.t = t;
        if (!init_) {
            return e;
        }
        std::array<double, 4> s = s_;
        auto P = P_;
        predict(s, P, std::max(0.0, t - t_));
        const double speed = std::hypot(s[2], s[3]);
        const double yaw = speed > 0.5 ? std::atan2(s[3], s[2]) : yaw_;
        if (speed > 0.5) {
            yaw_ = yaw;
        }
        const double c = std::cos(yaw), sn = std::sin(yaw);
        e.valid = true;
        e.x = s[0] - (c * cfg_.antennaX - sn * cfg_.antennaY);
        e.y = s[1] - (sn * cfg_.antennaX + c * cfg_.antennaY);
        e.yaw = yaw;
        e.v = speed;
        e.hasCov = true;
        e.pxx = P[0][0];
        e.pxy = P[0][1];
        e.pyy = P[1][1];
        e.pyawyaw = std::numeric_limits<double>::quiet_NaN();  // no heading covariance in this model
        e.lastGnssUsed = lastFix_;
        e.status = "GNSS_CV";
        return e;
    }

private:
    void predict(std::array<double, 4>& s, std::array<std::array<double, 4>, 4>& P, double dt) const {
        if (dt <= 0.0) return;
        s[0] += s[2] * dt;
        s[1] += s[3] * dt;
        const double q = cfg_.cvAccelSigma * cfg_.cvAccelSigma;
        for (int axis = 0; axis < 2; ++axis) {
            const size_t p = static_cast<size_t>(axis), v = static_cast<size_t>(axis + 2);
            // P = F P F^T + Q for one axis, with F = [1 dt; 0 1] (cross-axis terms advance alike)
            for (size_t j = 0; j < 4; ++j) P[p][j] += dt * P[v][j];
            for (size_t i = 0; i < 4; ++i) P[i][p] += dt * P[i][v];
            P[p][p] += q * dt * dt * dt / 3.0;
            P[p][v] += q * dt * dt / 2.0;
            P[v][p] += q * dt * dt / 2.0;
            P[v][v] += q * dt;
        }
    }
    EstimatorConfig cfg_;
    const LaneNetwork& map_;
    bool init_ = false;
    std::array<double, 4> s_{};
    std::array<std::array<double, 4>, 4> P_{};
    double t_ = 0.0, lastFix_ = -1.0;
    mutable double yaw_ = 0.0;
    int rejectRun_ = 0;
};

}  // namespace

std::vector<std::string> estimatorNames() { return {"ekf", "dr", "gnss_cv", "gnss_hold"}; }

std::unique_ptr<Estimator> makeEstimator(const std::string& name, const EstimatorConfig& cfg, const LaneNetwork& map) {
    if (name == "ekf") return std::unique_ptr<Estimator>(new Ekf(cfg, map));
    if (name == "dr") return std::unique_ptr<Estimator>(new DeadReckoning(cfg, map));
    if (name == "gnss_hold") return std::unique_ptr<Estimator>(new GnssHold(cfg, map));
    if (name == "gnss_cv") return std::unique_ptr<Estimator>(new GnssCv(cfg, map));
    return nullptr;
}

}  // namespace road

namespace road {

bool applyConfigOverrides(const std::string& text, EstimatorConfig& cfg, std::string& error) {
    std::map<std::string, double*> fields = {
        {"gnss_sigma", &cfg.gnssSigma},          {"wheel_sigma", &cfg.wheelSigma},     {"gyro_sigma", &cfg.gyroSigma},
        {"accel_sigma", &cfg.accelSigma},        {"pos_process", &cfg.posProcess},     {"gyro_bias_walk", &cfg.gyroBiasWalk},
        {"accel_bias_walk", &cfg.accelBiasWalk}, {"scale_walk", &cfg.scaleWalk},       {"gate_gnss", &cfg.gateGnss},
        {"gate_wheel", &cfg.gateWheel},          {"robust_nis", &cfg.robustNis},       {"cv_accel_sigma", &cfg.cvAccelSigma},
        {"init_pos_sigma", &cfg.initPosSigma},   {"init_yaw_sigma", &cfg.initYawSigma}, {"init_gyro_bias_sigma", &cfg.initGyroBiasSigma},
        {"gnss_corr_sigma", &cfg.gnssCorrSigma}, {"gnss_corr_tau", &cfg.gnssCorrTau}};
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t end = text.find(',', start);
        const std::string item = text.substr(start, end == std::string::npos ? std::string::npos : end - start);
        const std::size_t eq = item.find('=');
        if (eq == std::string::npos) {
            error = "expected key=value in '" + item + "'";
            return false;
        }
        const std::string key = item.substr(0, eq);
        if (key == "reset_after_rejected") {
            cfg.resetAfterRejected = std::stoi(item.substr(eq + 1));
        } else if (fields.count(key)) {
            *fields[key] = std::stod(item.substr(eq + 1));
        } else {
            error = "unknown estimator setting " + key;
            return false;
        }
        if (end == std::string::npos) break;
        start = end + 1;
    }
    return true;
}

std::string configText(const EstimatorConfig& c) {
    std::ostringstream o;
    o << "gnss_sigma=" << c.gnssSigma << " gnss_corr_sigma=" << c.gnssCorrSigma << " gnss_corr_tau=" << c.gnssCorrTau << " wheel_sigma=" << c.wheelSigma << " gyro_sigma=" << c.gyroSigma << " accel_sigma=" << c.accelSigma
      << " pos_process=" << c.posProcess << " gyro_bias_walk=" << c.gyroBiasWalk << " accel_bias_walk=" << c.accelBiasWalk
      << " scale_walk=" << c.scaleWalk << " gate_gnss=" << c.gateGnss << " gate_wheel=" << c.gateWheel << " robust_nis=" << c.robustNis
      << " reset_after_rejected=" << c.resetAfterRejected << " cv_accel_sigma=" << c.cvAccelSigma << " init_pos_sigma=" << c.initPosSigma
      << " init_yaw_sigma=" << c.initYawSigma << " init_gyro_bias_sigma=" << c.initGyroBiasSigma;
    return o.str();
}

}  // namespace road
