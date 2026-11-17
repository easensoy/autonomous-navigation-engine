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
    Ekf(const EstimatorConfig& cfg, const LaneNetwork& map, const char* name) : cfg_(cfg), map_(map), name_(name) {}
    const char* name() const override { return name_; }

    void process(const Measurement& m) override {
        if (m.kind == Measurement::Kerb && !cfg_.kerb) {
            return;  // a filter without kerb ranging never sees the scans
        }
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
        if (cfg_.integrity) {
            e.gnss = c.lastFixSeen >= 0.0 && t - c.lastFixSeen > 2.0 ? "OUTAGE"
                                                                       : (c.gnssMode == 1 ? "FAULTED" : (c.gnssMode == 2 ? "INCONSISTENT" : "TRUSTED"));
        }
        if (cfg_.kerb) {
            e.kerb = c.lastKerbSeen < 0.0 || t - c.lastKerbSeen > 0.5 ? "NONE" : (c.lastKerbOk ? "MATCHED" : "REJECTED");
        }
        return e;
    }

private:
    static constexpr int kWindow = 32;
    struct Core {
        double t = 0.0;
        Vec x{};
        Mat P{};
        double wz = 0.0, ax = 0.0, steer = 0.0;
        bool haveImu = false;
        double wheel = -1.0, stillSince = -1.0, lastGnss = -1.0;
        int rejectRun = 0;
        // GNSS integrity: an odometry-only track (never corrected by GNSS or kerbs) for the jump test.
        double odoX = 0.0, odoY = 0.0, odoYaw = 0.0;
        double prevFixT = -1.0, prevFixX = 0.0, prevFixY = 0.0, prevOdoX = 0.0, prevOdoY = 0.0, lastFixSeen = -1.0;
        int gnssMode = 0;  // 0 trusted, 1 faulted (jump detected), 2 inconsistent (persistent disagreement)
        double faultX = 0.0, faultY = 0.0;  // the innovation when the jump was detected
        int exitRun = 0, agreeRun = 0;
        double winT[kWindow] = {}, winX[kWindow] = {}, winY[kWindow] = {};
        bool winStepOk[kWindow] = {};
        int winN = 0, winHead = 0;
        // Kerb ranging.
        double lastKerbSeen = -1.0, lastKerbGood = -1.0, distSinceKerb = 0.0;
        bool lastKerbOk = false;
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
        cur_.odoX = p.x;
        cur_.odoY = p.y;
        cur_.odoYaw = p.yaw;
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
        if (cfg_.integrity) {
            const double om = c.odoYaw + 0.5 * w * dt;
            c.odoX += v * std::cos(om) * dt;
            c.odoY += v * std::sin(om) * dt;
            c.odoYaw = wrapAngle(c.odoYaw + w * dt);
        }
        if (cfg_.kerb) {
            c.distSinceKerb += std::fabs(v) * dt;
        }
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
                if (cfg_.integrity) {
                    gnssWithIntegrity(c, m, H, y, nis);
                    break;
                }
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
            case Measurement::Kerb:
                kerbUpdate(c, m);
                break;
        }
        c.x[YAW] = wrapAngle(c.x[YAW]);
        c.x[VEL] = std::max(0.0, c.x[VEL]);
    }

    // GNSS with integrity monitoring. Three tests, none of them a timer:
    //  - jump test: a fix-to-fix displacement that the odometry track cannot explain marks the receiver
    //    FAULTED, remembering the offset it jumped to; fixes are then excluded until several in a row fit
    //    the filter and clearly not that offset (the fault has ended);
    //  - gate: a fix outside the filter's chi-square gate is not used; after many in a row with no jump,
    //    the filter and the receiver are INCONSISTENT;
    //  - re-anchoring, only when INCONSISTENT: if the recent fixes agree with each other and with
    //    odometry (no jumps, small scatter) and their offset is plausible for a filter up to 3 times
    //    overconfident, the filter admits the disagreement as uncertainty and uses them. A kerb match
    //    that supports the filter blocks this.
    void gnssWithIntegrity(Core& c, const Measurement& m, const double (&H)[2][N], const double (&y)[2], double nis) {
        const double cyo = std::cos(c.odoYaw), syo = std::sin(c.odoYaw);
        const double antX = c.odoX + cyo * cfg_.antennaX - syo * cfg_.antennaY, antY = c.odoY + syo * cfg_.antennaX + cyo * cfg_.antennaY;
        bool jump = false;
        if (c.prevFixT >= 0.0 && m.tMeas - c.prevFixT <= cfg_.stepMaxGap && m.tMeas > c.prevFixT) {
            const double rx = (m.a - c.prevFixX) - (antX - c.prevOdoX), ry = (m.b - c.prevFixY) - (antY - c.prevOdoY);
            const double moved = std::hypot(antX - c.prevOdoX, antY - c.prevOdoY);
            const double var = 2.0 * cfg_.gnssSigma * cfg_.gnssSigma + 0.0004 * moved * moved;  // fix noise, odometry 2%
            jump = (rx * rx + ry * ry) / var > cfg_.stepGate;
        }
        // Re-synchronise the odometry heading with the filter, then remember this fix and the track position.
        c.odoYaw = c.x[YAW];
        c.prevFixT = m.tMeas;
        c.prevFixX = m.a;
        c.prevFixY = m.b;
        c.prevOdoX = c.odoX + std::cos(c.odoYaw) * cfg_.antennaX - std::sin(c.odoYaw) * cfg_.antennaY;
        c.prevOdoY = c.odoY + std::sin(c.odoYaw) * cfg_.antennaX + std::cos(c.odoYaw) * cfg_.antennaY;
        c.lastFixSeen = m.tMeas;
        if (jump) {
            ++stats.stepAlarms;
        }
        // Innovation covariance S = H P H^T + R, for the fault hypothesis test.
        const double r2 = cfg_.gnssSigma * cfg_.gnssSigma;
        double S[2][2];
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b) {
                double s = a == b ? r2 : 0.0;
                for (int i = 0; i < N; ++i)
                    for (int j = 0; j < N; ++j) s += H[a][i] * c.P[i][j] * H[b][j];
                S[a][b] = s;
            }
        const double det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        auto nisOf = [&](double ex, double ey) { return det > 1e-24 ? (S[1][1] * ex * ex - 2.0 * S[0][1] * ex * ey + S[0][0] * ey * ey) / det : 1e18; };
        bool use = false;
        // Fixes the filter does not use are remembered (innovation, and whether they passed the jump test)
        // for the re-anchoring test; any used fix clears the record.
        auto remember = [&]() {
            c.winT[c.winHead] = m.tMeas;
            c.winX[c.winHead] = y[0];
            c.winY[c.winHead] = y[1];
            c.winStepOk[c.winHead] = !jump;
            c.winHead = (c.winHead + 1) % kWindow;
            c.winN = std::min(c.winN + 1, kWindow);
        };
        if (c.gnssMode == 0) {
            if (jump) {
                c.gnssMode = 1;
                c.faultX = y[0];
                c.faultY = y[1];
                c.exitRun = 0;
                c.winN = 0;
            } else if (nis <= cfg_.gateGnss) {
                use = true;
                c.winN = 0;
            } else {
                remember();
                if (++c.rejectRun >= cfg_.inconsistentAfter) {
                    c.gnssMode = 2;
                    c.agreeRun = 0;
                }
            }
        } else if (c.gnssMode == 1) {
            const double nis1 = nisOf(y[0] - c.faultX, y[1] - c.faultY);
            c.exitRun = (nis <= cfg_.gateGnss && nis + 2.0 < nis1) ? c.exitRun + 1 : 0;
            if (c.exitRun >= cfg_.faultExitFixes) {
                c.gnssMode = 0;
                c.winN = 0;
                c.rejectRun = 0;
                ++stats.faultExits;
                use = true;
            }
        } else {
            remember();
            c.agreeRun = nis <= cfg_.gateGnss ? c.agreeRun + 1 : 0;
            if (c.agreeRun >= 3) {  // the filter and the receiver agree again
                c.gnssMode = 0;
                c.winN = 0;
                c.rejectRun = 0;
                use = true;
            } else if (c.winN >= cfg_.reanchorWindow) {
                // The window's innovations: a drifting filter makes them grow steadily, so their scatter is
                // taken about a straight-line trend in time; what remains is the receiver's own noise.
                const int n = cfg_.reanchorWindow;
                bool stepsOk = true;
                double st = 0.0, sxv = 0.0, syv = 0.0;
                for (int k = 0; k < n; ++k) {
                    const int i = (c.winHead - 1 - k + 2 * kWindow) % kWindow;
                    st += c.winT[i] / n;
                    sxv += c.winX[i] / n;
                    syv += c.winY[i] / n;
                    stepsOk = stepsOk && c.winStepOk[i];
                }
                double stt = 0.0, stx = 0.0, sty = 0.0;
                for (int k = 0; k < n; ++k) {
                    const int i = (c.winHead - 1 - k + 2 * kWindow) % kWindow;
                    stt += (c.winT[i] - st) * (c.winT[i] - st);
                    stx += (c.winT[i] - st) * (c.winX[i] - sxv);
                    sty += (c.winT[i] - st) * (c.winY[i] - syv);
                }
                const double gx = stt > 1e-9 ? stx / stt : 0.0, gy = stt > 1e-9 ? sty / stt : 0.0;  // m/s
                double scatter = 0.0;
                for (int k = 0; k < n; ++k) {
                    const int i = (c.winHead - 1 - k + 2 * kWindow) % kWindow;
                    const double ex = c.winX[i] - sxv - gx * (c.winT[i] - st), ey = c.winY[i] - syv - gy * (c.winT[i] - st);
                    scatter += (ex * ex + ey * ey) / n;
                }
                // Plausible for an overconfident filter: test the latest offset against an inflated S.
                const double mx = y[0], my = y[1];
                const double k = cfg_.reanchorInflation;
                const double a = k * (S[0][0] - r2) + r2, b = k * S[0][1], d = k * (S[1][1] - r2) + r2, dd = a * d - b * b;
                const double plaus = dd > 1e-24 ? (d * mx * mx - 2.0 * b * mx * my + a * my * my) / dd : 1e18;
                const bool kerbSupports = cfg_.kerb && c.lastKerbGood >= 0.0 && m.tMeas - c.lastKerbGood < 1.0;
                if (stepsOk && std::sqrt(scatter) <= cfg_.reanchorScatter && plaus <= cfg_.gateGnss && !kerbSupports) {
                    // Admit the disagreement as uncertainty, then use the fix normally: the position by the
                    // offset itself; the heading by the drift rate it implies; the gyro bias with it.
                    const double drift = std::hypot(gx, gy) / std::max(1.0, c.x[VEL]);
                    c.P[PX][PX] += mx * mx + r2;
                    c.P[PY][PY] += my * my + r2;
                    c.P[PX][PY] += mx * my;
                    c.P[PY][PX] += mx * my;
                    c.P[YAW][YAW] += 4.0 * drift * drift + 0.02 * 0.02;
                    c.P[BG][BG] += 0.01 * 0.01;
                    symmetrise(c.P);
                    c.gnssMode = 0;
                    c.winN = 0;
                    c.rejectRun = 0;
                    ++stats.reanchors;
                    use = true;
                }
            }
        }
        bool applied = false;
        if (use) {
            const double sc = (cfg_.robustNis > 0.0 && nis > cfg_.robustNis && nis <= cfg_.gateGnss) ? nis / cfg_.robustNis : 1.0;
            const double Rr[2][2] = {{r2 * sc, 0.0}, {0.0, r2 * sc}};
            // The gate has been applied above (or deliberately waived after re-anchoring, whose inflated
            // covariance then sets the fix's weight).
            kalmanUpdate<2>(c.x, c.P, H, y, Rr, -1.0, applied);
        }
        if (applied) {
            c.rejectRun = 0;
            c.lastGnss = m.tMeas;
            ++stats.gnssUsed;
        } else {
            ++stats.gnssRejected;
            if (c.gnssMode == 1) ++stats.gnssExcluded;
        }
    }

    // Scan-to-map registration of a kerb scan, folded into the filter. Each return's end point should lie
    // on a mapped kerb: its signed distance to the nearest kerb is the residual. A robust Gauss-Newton
    // search over (x, y, heading), anchored by the filter's own prediction, gives the pose that fits the
    // scan; its information matrix says which directions the scan constrains (along a straight kerb only
    // lateral position and heading; at a corner all three). Each constrained direction becomes a scalar
    // measurement of the filter. The kerb deviation is correlated along the scan, so the information is
    // scaled to kerbEffective independent returns. A scan that disagrees with the filter beyond the gate
    // is rejected, and counts as no support for the filter.
    void kerbUpdate(Core& c, const Measurement& m) {
        c.lastKerbSeen = m.tMeas;
        c.lastKerbOk = false;
        const double p0[3] = {c.x[PX], c.x[PY], c.x[YAW]};
        double P0[3][3];
        const int idx[3] = {PX, PY, YAW};
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) P0[a][b] = c.P[idx[a]][idx[b]];
        double P0i[3][3];
        if (!invert3(P0, P0i)) {
            ++stats.kerbRejected;
            return;
        }
        const double sig2 = cfg_.kerbSigma * cfg_.kerbSigma;
        // Lateral prior uncertainty sets how far from a kerb a return may start out.
        const double latSig = std::sqrt(std::max(P0[0][0], P0[1][1]));
        double p[3] = {p0[0], p0[1], p0[2]};
        struct Pt {
            double d, J[3];
        };
        auto residuals = [&](const double (&q)[3], double gate, std::vector<Pt>& out) {
            out.clear();
            const double sx = q[0] + m.c * std::cos(q[2]), sy = q[1] + m.c * std::sin(q[2]);
            for (size_t i = 0; i < m.ranges.size(); ++i) {
                const double r = m.ranges[i];
                if (r <= 0.0) continue;
                const double th = q[2] + m.a + static_cast<double>(i) * m.b;
                const double ex = sx + r * std::cos(th), ey = sy + r * std::sin(th);
                double nx = 0.0, ny = 0.0;
                const double d = map_.kerbDistance(ex, ey, nx, ny);
                if (std::fabs(d) > gate) continue;
                out.push_back({d, {nx, ny, nx * -(ey - q[1]) + ny * (ex - q[0])}});
            }
        };
        std::vector<Pt> pts;
        double gate = std::min(1.5, std::max(0.4, 3.0 * latSig));
        for (int it = 0; it < 6; ++it) {
            residuals(p, gate, pts);
            if (static_cast<int>(pts.size()) < cfg_.kerbMinReturns) break;
            const double kappa = std::min(1.0, cfg_.kerbEffective / static_cast<double>(pts.size()));
            double A[3][3], g[3];
            for (int a = 0; a < 3; ++a) {
                g[a] = 0.0;
                for (int b = 0; b < 3; ++b) {
                    A[a][b] = P0i[a][b];
                    g[a] -= P0i[a][b] * (p[b] - p0[b]);
                }
            }
            for (const Pt& q : pts) {
                const double u = std::fabs(q.d) / cfg_.kerbSigma;
                const double w = kappa * (u <= 2.0 ? 1.0 : 2.0 / u) / sig2;  // Huber weight
                for (int a = 0; a < 3; ++a) {
                    g[a] -= w * q.J[a] * q.d;
                    for (int b = 0; b < 3; ++b) A[a][b] += w * q.J[a] * q.J[b];
                }
            }
            double Ai[3][3];
            if (!invert3(A, Ai)) break;
            double step = 0.0;
            for (int a = 0; a < 3; ++a) {
                double dlt = 0.0;
                for (int b = 0; b < 3; ++b) dlt += Ai[a][b] * g[b];
                p[a] += dlt;
                step += dlt * dlt;
            }
            gate = std::max(3.0 * cfg_.kerbSigma, 0.5 * gate);
            if (step < 1e-8 && it >= 1) break;
        }
        residuals(p, 3.0 * cfg_.kerbSigma, pts);
        if (static_cast<int>(pts.size()) < cfg_.kerbMinReturns) {
            ++stats.kerbRejected;
            return;
        }
        const double kappa = std::min(1.0, cfg_.kerbEffective / static_cast<double>(pts.size()));
        double L[3][3] = {}, gr[3] = {};
        for (const Pt& q : pts) {
            for (int a = 0; a < 3; ++a) {
                gr[a] += kappa * q.J[a] * q.d / sig2;
                for (int b = 0; b < 3; ++b) L[a][b] += kappa * q.J[a] * q.J[b] / sig2;
            }
        }
        double ev[3], V[3][3];
        eigenSym3(L, ev, V);
        const double emax = std::max(ev[0], std::max(ev[1], ev[2]));
        struct Dir {
            double z, r, h[3];
        };
        std::vector<Dir> dirs;
        double nisTot = 0.0;
        for (int j = 0; j < 3; ++j) {
            if (ev[j] < 1.0 || ev[j] < 1e-4 * emax) continue;  // a direction the scan does not constrain
            const double v[3] = {V[0][j], V[1][j], V[2][j]};
            double vp = 0.0, vg = 0.0, vp0 = 0.0, s = 0.0;
            for (int a = 0; a < 3; ++a) {
                vp += v[a] * p[a];
                vg += v[a] * gr[a];
                vp0 += v[a] * p0[a];
                for (int b = 0; b < 3; ++b) s += v[a] * P0[a][b] * v[b];
            }
            // The measurement alone along v: the converged pose minus the prior's pull on it.
            double z = vp - vg / ev[j];
            if (std::fabs(v[2]) > 0.5) {  // keep heading differences wrapped
                z = vp0 + wrapAngle(z - vp0);
            }
            // White noise of the returns, plus the kerb-deviation floor counted once per correlation length.
            const double floor = (v[0] * v[0] + v[1] * v[1]) * cfg_.kerbFloorPos * cfg_.kerbFloorPos + v[2] * v[2] * cfg_.kerbFloorYaw * cfg_.kerbFloorYaw;
            const double r = 1.0 / ev[j] + floor * std::max(1.0, cfg_.kerbCorrLength / std::max(0.5, c.distSinceKerb));
            nisTot += (z - vp0) * (z - vp0) / (s + r);
            dirs.push_back({z, r, {v[0], v[1], v[2]}});
        }
        stats.nisKerb.push_back(nisTot);
        if (dirs.empty() || nisTot > cfg_.kerbGate) {
            ++stats.kerbRejected;
            return;
        }
        const double scale = nisTot > cfg_.kerbRobust ? nisTot / cfg_.kerbRobust : 1.0;
        for (const Dir& d : dirs) {
            double Hd[1][N] = {};
            Hd[0][PX] = d.h[0];
            Hd[0][PY] = d.h[1];
            Hd[0][YAW] = d.h[2];
            const double pred = d.h[0] * c.x[PX] + d.h[1] * c.x[PY] + d.h[2] * c.x[YAW];
            const double yv[1] = {d.z - pred};
            const double Rv[1][1] = {{d.r * scale}};
            bool applied = false;
            kalmanUpdate<1>(c.x, c.P, Hd, yv, Rv, -1.0, applied);
        }
        ++stats.kerbUsed;
        c.lastKerbOk = true;
        c.distSinceKerb = 0.0;
        if (nisTot <= cfg_.kerbRobust) {
            c.lastKerbGood = m.tMeas;
        }
    }

    static bool invert3(const double (&A)[3][3], double (&B)[3][3]) {
        const double a = A[0][0], b = A[0][1], cc = A[0][2], d = A[1][0], e = A[1][1], f = A[1][2], g = A[2][0], h = A[2][1], i = A[2][2];
        const double C00 = e * i - f * h, C01 = -(d * i - f * g), C02 = d * h - e * g;
        const double det = a * C00 + b * C01 + cc * C02;
        if (!(std::fabs(det) > 1e-30)) return false;
        B[0][0] = C00 / det;
        B[1][0] = C01 / det;
        B[2][0] = C02 / det;
        B[0][1] = -(b * i - cc * h) / det;
        B[1][1] = (a * i - cc * g) / det;
        B[2][1] = -(a * h - b * g) / det;
        B[0][2] = (b * f - cc * e) / det;
        B[1][2] = -(a * f - cc * d) / det;
        B[2][2] = (a * e - b * d) / det;
        return true;
    }

    // Eigen-decomposition of a symmetric 3x3 matrix by Jacobi rotations; columns of V are eigenvectors.
    static void eigenSym3(const double (&Ain)[3][3], double (&ev)[3], double (&V)[3][3]) {
        double A[3][3];
        for (int a = 0; a < 3; ++a)
            for (int b = 0; b < 3; ++b) {
                A[a][b] = 0.5 * (Ain[a][b] + Ain[b][a]);
                V[a][b] = a == b ? 1.0 : 0.0;
            }
        for (int sweep = 0; sweep < 30; ++sweep) {
            const double off = A[0][1] * A[0][1] + A[0][2] * A[0][2] + A[1][2] * A[1][2];
            if (off < 1e-24) break;
            for (int pq = 0; pq < 3; ++pq) {
                const int pI = pq == 2 ? 1 : 0, qI = pq == 0 ? 1 : 2;
                if (std::fabs(A[pI][qI]) < 1e-300) continue;
                const double theta = 0.5 * std::atan2(2.0 * A[pI][qI], A[qI][qI] - A[pI][pI]);
                const double cs = std::cos(theta), sn = std::sin(theta);
                for (int k = 0; k < 3; ++k) {  // A = J^T A J
                    const double akp = A[k][pI], akq = A[k][qI];
                    A[k][pI] = cs * akp - sn * akq;
                    A[k][qI] = sn * akp + cs * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = A[pI][k], aqk = A[qI][k];
                    A[pI][k] = cs * apk - sn * aqk;
                    A[qI][k] = sn * apk + cs * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = V[k][pI], vkq = V[k][qI];
                    V[k][pI] = cs * vkp - sn * vkq;
                    V[k][qI] = sn * vkp + cs * vkq;
                }
            }
        }
        for (int a = 0; a < 3; ++a) ev[a] = A[a][a];
    }

    EstimatorConfig cfg_;
    const LaneNetwork& map_;
    const char* name_;
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
            case Measurement::Kerb:
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

std::vector<std::string> estimatorNamesExtended() { return {"ekf", "ekf_int", "ekf_kerb", "dr", "gnss_cv", "gnss_hold"}; }

std::unique_ptr<Estimator> makeEstimator(const std::string& name, const EstimatorConfig& cfg, const LaneNetwork& map) {
    if (name == "ekf" || name == "ekf_int" || name == "ekf_kerb") {
        EstimatorConfig c = cfg;
        c.integrity = name != "ekf";
        c.kerb = name == "ekf_kerb";
        if (c.kerb) {
            c.gyroBiasWalk = c.kerbGyroBiasWalk;
        }
        const char* label = name == "ekf" ? "ekf" : (name == "ekf_int" ? "ekf_int" : "ekf_kerb");
        return std::unique_ptr<Estimator>(new Ekf(c, map, label));
    }
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
        {"gnss_corr_sigma", &cfg.gnssCorrSigma}, {"gnss_corr_tau", &cfg.gnssCorrTau},   {"step_gate", &cfg.stepGate},
        {"reanchor_scatter", &cfg.reanchorScatter}, {"reanchor_inflation", &cfg.reanchorInflation}, {"kerb_sigma", &cfg.kerbSigma},
        {"kerb_effective", &cfg.kerbEffective}, {"kerb_floor_pos", &cfg.kerbFloorPos}, {"kerb_floor_yaw", &cfg.kerbFloorYaw},
        {"kerb_corr_length", &cfg.kerbCorrLength}, {"kerb_gate", &cfg.kerbGate}, {"kerb_robust", &cfg.kerbRobust},
        {"kerb_gyro_bias_walk", &cfg.kerbGyroBiasWalk}};
    std::map<std::string, int*> ints = {{"fault_exit_fixes", &cfg.faultExitFixes}, {"inconsistent_after", &cfg.inconsistentAfter},
                                        {"reanchor_window", &cfg.reanchorWindow}, {"kerb_min_returns", &cfg.kerbMinReturns}};
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
        } else if (ints.count(key)) {
            *ints[key] = std::stoi(item.substr(eq + 1));
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
      << " init_yaw_sigma=" << c.initYawSigma << " init_gyro_bias_sigma=" << c.initGyroBiasSigma << " step_gate=" << c.stepGate
      << " fault_exit_fixes=" << c.faultExitFixes << " inconsistent_after=" << c.inconsistentAfter << " reanchor_window=" << c.reanchorWindow
      << " reanchor_scatter=" << c.reanchorScatter << " reanchor_inflation=" << c.reanchorInflation << " kerb_sigma=" << c.kerbSigma
      << " kerb_effective=" << c.kerbEffective << " kerb_floor_pos=" << c.kerbFloorPos << " kerb_floor_yaw=" << c.kerbFloorYaw
      << " kerb_corr_length=" << c.kerbCorrLength << " kerb_gyro_bias_walk=" << c.kerbGyroBiasWalk << " kerb_min_returns=" << c.kerbMinReturns << " kerb_gate=" << c.kerbGate
      << " kerb_robust=" << c.kerbRobust;
    return o.str();
}

}  // namespace road
