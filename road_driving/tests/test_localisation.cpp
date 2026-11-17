// Localisation tests: sensors, estimators, replay from logs, and driving on the estimate. Each check states
// a property that could be false. Returns non-zero on any failure.
#include "road_driving/benchmark.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/localisation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

using namespace road;

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string& what, const std::string& detail = "") {
    if (ok) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("FAIL  %s%s%s\n", what.c_str(), detail.empty() ? "" : "  -- ", detail.c_str());
    }
}

std::string f3(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.3f", v);
    return b;
}

RouteSpec route(const std::string& name) {
    for (bool h : {false, true}) {
        for (const RouteSpec& r : benchmarkRoutes(h)) {
            if (r.name == name) return r;
        }
    }
    for (const RouteSpec& r : benchmarkRoutesR3()) {
        if (r.name == name) return r;
    }
    for (const RouteSpec& r : benchmarkRoutesR4()) {
        if (r.name == name) return r;
    }
    for (const RouteSpec& r : benchmarkRoutesR5()) {
        if (r.name == name) return r;
    }
    for (const RouteSpec& r : benchmarkRoutesR6()) {
        if (r.name == name) return r;
    }
    return {};
}

EpisodeResult record(const std::string& routeName, const std::string& condition, std::uint64_t seed) {
    return runEpisode(benchmarkScenario(route(routeName), condition, seed, "ground_truth"));
}

LocMetrics find(const std::vector<LocMetrics>& ms, const std::string& name) {
    for (const LocMetrics& m : ms) {
        if (m.estimator == name) return m;
    }
    return ms.front();
}

// ---- L1 sensors ---------------------------------------------------------------------------------------
void testSensors() {
    const EpisodeResult r = record("R1-A", "nominal", 1);
    const double T = r.simSeconds;
    int n[5] = {0, 0, 0, 0, 0};
    bool sorted = true, latencyOk = true, restOk = true;
    double sx = 0.0, sy = 0.0, sxx = 0.0, syy = 0.0;
    int ng = 0;
    const GnssConfig g;
    for (size_t i = 0; i < r.sensors.size(); ++i) {
        const Measurement& m = r.sensors[i];
        ++n[m.kind];
        if (i > 0 && m.tArrival < r.sensors[i - 1].tArrival) sorted = false;
        const double lat = m.tArrival - m.tMeas;
        if (m.kind == Measurement::Imu && std::fabs(lat) > 1e-6) latencyOk = false;
        if (m.kind == Measurement::Wheel && std::fabs(lat - 0.02) > 1e-6) latencyOk = false;
        if (m.kind == Measurement::Gnss && (lat < 0.08 - 1e-6 || lat > 0.12 + 1e-6)) latencyOk = false;
        if (m.tMeas < 1.9) {  // parked for the 2 s alignment: encoders silent, no acceleration sensed
            if (m.kind == Measurement::Wheel && m.a != 0.0) restOk = false;
            if (m.kind == Measurement::Imu && std::fabs(m.a) > 0.3) restOk = false;
        }
        if (m.kind == Measurement::Gnss) {
            const size_t k = static_cast<size_t>(std::lround(m.tMeas / 0.02));
            if (k < r.truth.size()) {
                const TruthSample& t = r.truth[k];
                const double ex = m.a - (t.x + std::cos(t.yaw) * g.antennaX), ey = m.b - (t.y + std::sin(t.yaw) * g.antennaX);
                sx += ex; sy += ey; sxx += ex * ex; syy += ey * ey; ++ng;
            }
        }
    }
    check(std::fabs(n[Measurement::Imu] / T - 100.0) < 2.0, "L1 IMU at 100 Hz", f3(n[Measurement::Imu] / T));
    check(std::fabs(n[Measurement::Wheel] / T - 50.0) < 1.5 && std::fabs(n[Measurement::Steer] / T - 50.0) < 1.5, "L1 wheel and steering at 50 Hz");
    check(n[Measurement::Gnss] / T > 9.0 && n[Measurement::Gnss] / T < 10.1, "L1 GNSS at 10 Hz less 2% dropped fixes", f3(n[Measurement::Gnss] / T));
    check(sorted, "L1 the log is in arrival order");
    check(latencyOk, "L1 latencies: IMU 0, wheel 0.02 s, GNSS 0.10 +- 0.02 s");
    check(restOk, "L1 at rest the encoders read 0 and the accelerometer senses no brake-hold 'acceleration'");
    const double stdx = std::sqrt(sxx / ng - (sx / ng) * (sx / ng)), stdy = std::sqrt(syy / ng - (sy / ng) * (sy / ng));
    check(stdx > 0.18 && stdx < 0.45 && stdy > 0.18 && stdy < 0.45, "L1 GNSS error spread matches the configured noise (white 0.25 m, correlated 0.15 m)",
          f3(stdx) + " / " + f3(stdy));

    const EpisodeResult o = record("R1-A", "gnss_outage", 1);
    bool silent = !o.outages.empty();
    for (const Measurement& m : o.sensors) {
        for (const auto& w : o.outages) {
            if (m.kind == Measurement::Gnss && m.tMeas > w.first + 1e-6 && m.tMeas < w.second - 1e-6) silent = false;
        }
    }
    check(silent, "L1 no GNSS fix is produced inside an outage zone");
}

// ---- L2 replay exactness, L3 delayed measurements -----------------------------------------------------
void testReplayAndDelays() {
    const EpisodeResult r = record("R1-A", "delay_drop", 2);
    const LaneNetwork net = buildR1();
    const EstimatorConfig cfg;
    std::vector<std::vector<StateEstimate>> memTraces, fileTraces;
    const auto inMemory = evaluateLocalisation(r.truth, r.sensors, r.outages, net, cfg, &memTraces);
    const std::string dir = (std::filesystem::temp_directory_path() / "road_driving_loc_test").string();
    std::string error;
    writeEpisode(r, dir, error);
    std::string truthText, sensorText;
    std::vector<TruthSample> truth;
    std::vector<Measurement> sensors;
    const bool read = readText(dir + "/truth.csv", truthText) && readText(dir + "/sensors.csv", sensorText) && parseTruthCsv(truthText, truth) &&
                      parseSensorLog(sensorText, sensors, error);
    check(read && truth.size() == r.truth.size() && sensors.size() == r.sensors.size(), "L2 truth.csv and sensors.csv read back completely", error);
    const auto fromFiles = evaluateLocalisation(truth, sensors, r.outages, net, cfg, &fileTraces);
    // The estimators' inputs are bit-identical (measurements are rounded to the log's format at the
    // source), so every estimate must match exactly; truth.csv keeps 6 decimals, so the scores against
    // it agree to that precision.
    bool sameTraces = fileTraces.size() == memTraces.size(), sameScores = fromFiles.size() == inMemory.size();
    for (size_t k = 0; sameTraces && k < memTraces.size(); ++k) {
        for (size_t i = 0; sameTraces && i < memTraces[k].size(); ++i) {
            const StateEstimate &x = memTraces[k][i], &y = fileTraces[k][i];
            // Query times differ in the last bit (k x 0.01 in memory, 2-decimal text in truth.csv).
            sameTraces = x.valid == y.valid && std::fabs(x.x - y.x) < 1e-9 && std::fabs(x.y - y.y) < 1e-9 && std::fabs(x.yaw - y.yaw) < 1e-9 &&
                         std::fabs(x.pxx - y.pxx) < 1e-12;
        }
    }
    for (size_t i = 0; sameScores && i < inMemory.size(); ++i) {
        sameScores = std::fabs(fromFiles[i].rmse - inMemory[i].rmse) < 1e-5 && std::fabs(fromFiles[i].max - inMemory[i].max) < 1e-5;
    }
    check(sameTraces, "L2 every estimator replayed from sensors.csv reproduces its estimates to 1e-9 m (no world simulator)");
    check(sameScores, "L2 and the same scores against truth.csv, to its 6-decimal precision");
    std::filesystem::remove_all(dir);

    // The same measurements with no delays, in validity order (ties: IMU, wheel, steering, GNSS, the order
    // in which a delayed arrival is re-inserted). Rewinding for late measurements must give the same filter.
    std::vector<Measurement> inOrder = r.sensors;
    auto rank = [](const Measurement& m) { return m.kind == Measurement::Imu ? 0 : m.kind == Measurement::Wheel ? 1 : m.kind == Measurement::Steer ? 2 : 3; };
    std::stable_sort(inOrder.begin(), inOrder.end(), [&](const Measurement& a, const Measurement& b) {
        return a.tMeas != b.tMeas ? a.tMeas < b.tMeas : rank(a) < rank(b);
    });
    for (Measurement& m : inOrder) m.tArrival = m.tMeas;
    // The delayed filter starts when its first fix arrives and never sees what arrived before that: give
    // the in-order filter exactly the same measurements.
    double tInit = 1e18;
    for (const Measurement& m : r.sensors) {
        if (m.kind == Measurement::Gnss) {
            tInit = m.tArrival;
            break;
        }
    }
    std::vector<Measurement> usable;
    for (const Measurement& m : r.sensors) {
        if (m.tArrival >= tInit) usable.push_back(m);
    }
    std::vector<Measurement> filtered;
    for (const Measurement& m : inOrder) {
        for (const Measurement& u : usable) {
            if (u.tMeas == m.tMeas && u.kind == m.kind && u.a == m.a) {
                filtered.push_back(m);
                break;
            }
        }
    }
    inOrder.swap(filtered);
    auto delayedAll = makeEstimator("ekf", cfg, net), orderedAll = makeEstimator("ekf", cfg, net);
    for (const Measurement& m : r.sensors) delayedAll->process(m);
    for (const Measurement& m : inOrder) orderedAll->process(m);
    const double tEnd = r.sensors.back().tArrival;
    const StateEstimate da = delayedAll->estimateAt(tEnd), oa = orderedAll->estimateAt(tEnd);
    const double diff = std::max({std::fabs(da.x - oa.x), std::fabs(da.y - oa.y), std::fabs(da.yaw - oa.yaw), std::fabs(da.pxx - oa.pxx)});
    check(diff < 1e-9, "L3 delayed and out-of-order GNSS (latency 0.4 s, jitter 0.1 s) gives the same EKF as in-order processing", f3(diff * 1e9) + " nm");
    check(da.valid && delayedAll->stats.lateDropped == 0, "L3 no measurement arrived too late to be replayed");
}

// ---- L4-L8 estimator behaviour --------------------------------------------------------------------------
void testEstimators() {
    const LaneNetwork net1 = buildR1();
    const EstimatorConfig cfg;
    int wins = 0, runs = 0;
    for (std::uint64_t seed = 1; seed <= 3; ++seed) {
        const EpisodeResult r = record("R1-A", "nominal", seed);
        std::vector<std::vector<StateEstimate>> traces;
        const auto ms = evaluateLocalisation(r.truth, r.sensors, r.outages, net1, cfg, &traces);
        const LocMetrics e = find(ms, "ekf");
        const std::string tag = "seed " + std::to_string(seed);
        check(!e.failed && e.rmse < 0.3 && e.p95 < 0.5, "L4 EKF nominal accuracy (RMSE < 0.3 m, p95 < 0.5 m), " + tag, f3(e.rmse) + " / " + f3(e.p95));
        check(e.neesMean > 0.5 && e.neesMean < 6.0, "L4 EKF nominal NEES mean (3 dof) within a factor of two of 3 (0.5 to 6), " + tag, f3(e.neesMean));
        ++runs;
        wins += (e.rmse < find(ms, "dr").rmse && e.rmse < find(ms, "gnss_cv").rmse && e.rmse < find(ms, "gnss_hold").rmse) ? 1 : 0;
        bool pd = true;
        for (const StateEstimate& s : traces[0]) {
            if (s.valid && !(s.pxx > 0.0 && s.pyy > 0.0 && s.pxx * s.pyy - s.pxy * s.pxy > 0.0 && s.pyawyaw > 0.0)) pd = false;
        }
        check(pd, "L7 the EKF position and heading covariance stays positive definite, " + tag);
    }
    check(wins == runs, "L4 the EKF beats dead reckoning and both GNSS-only baselines on every nominal run", std::to_string(wins) + "/" + std::to_string(runs));

    const EpisodeResult mp = record("R1-A", "multipath", 1);
    const auto mm = evaluateLocalisation(mp.truth, mp.sensors, mp.outages, net1, cfg);
    check(find(mm, "ekf").gnssRejected > 0 && find(mm, "ekf").max < 4.0, "L5 the EKF rejects biased fixes and stays inside the 4 m multipath offset",
          std::to_string(find(mm, "ekf").gnssRejected) + " rejected, max " + f3(find(mm, "ekf").max));
    check(find(mm, "ekf").max < find(mm, "gnss_cv").max, "L5 under multipath the EKF's worst error is below the GNSS-only filter's");

    const EpisodeResult wb = record("R1-B", "nominal", 1);  // ends westbound: heading near +-pi
    const auto mw = evaluateLocalisation(wb.truth, wb.sensors, wb.outages, net1, cfg);
    check(find(mw, "ekf").headingMaxDeg < 5.0, "L6 heading wraps cleanly through +-180 degrees (EKF max heading error < 5 deg)", f3(find(mw, "ekf").headingMaxDeg));

    const EpisodeResult ou = record("R1-A", "gnss_outage", 1);
    const auto mo = evaluateLocalisation(ou.truth, ou.sensors, ou.outages, net1, cfg);
    check(find(mo, "ekf").outageEnd >= 0.0 && find(mo, "ekf").outageEnd < find(mo, "dr").outageEnd,
          "L8 through a GNSS outage the EKF drifts less than dead reckoning", f3(find(mo, "ekf").outageEnd) + " vs " + f3(find(mo, "dr").outageEnd));
    check(find(mo, "ekf").recovery >= 0.0 && find(mo, "ekf").recovery < 30.0, "L8 the EKF error is back below 0.5 m after the outage", f3(find(mo, "ekf").recovery));

    const auto again = evaluateLocalisation(ou.truth, ou.sensors, ou.outages, net1, cfg);
    check(find(again, "ekf").rmse == find(mo, "ekf").rmse, "L12 evaluating the same log twice gives identical results");
}

// ---- L9, L10 driving on the estimate ------------------------------------------------------------------
void testClosedLoop() {
    Scenario s;
    findScenario("c0_nominal", s);
    s.departDelay = 2.0;
    s.seed = 1;
    s.condition = "nominal";
    Scenario gt = s;
    gt.mode = "ground_truth";
    Scenario est = s;
    est.mode = "estimated";
    const EpisodeResult a = runEpisode(gt), b = runEpisode(est);
    check(b.outcome == "COMPLETED" && b.metrics.localisationStops == 0, "L9 driving on the EKF completes the nominal mission", b.outcome);
    check(b.traceHash != a.traceHash && b.metrics.estRmse > 0.0 && b.metrics.estRmse < 0.4,
          "L9 the stack consumes the estimate (a different drive) and the estimate is scored against truth", f3(b.metrics.estRmse));
    check(b.metrics.associationErrors == 0 && b.metrics.illegalTransitions == 0, "L9 no lane-association error and no illegal move on the estimate");

    Scenario dr = est;
    dr.estimator = "dr";  // no covariance: its quality cannot be judged
    const EpisodeResult c = runEpisode(dr);
    double vmax = 0.0;
    for (const TruthSample& t : c.truth) vmax = std::max(vmax, t.v);
    bool flagged = false;
    for (const LogEvent& e : c.events) flagged = flagged || e.type == "LOCALISATION_UNRELIABLE";
    check(flagged && vmax < 0.5, "L10 an estimate without a covariance is unusable: the stack stops and never drives on it",
          "max speed " + f3(vmax));
}

// ---- L11 held-out network ------------------------------------------------------------------------------
void testHeldOutNetwork() {
    const LaneNetwork r2 = buildR2();
    const VehicleParams vp;
    check(r2.validate().empty(), "L11 R2 passes every map invariant");
    double worst = 1e9;
    int tight = 0;
    for (const Element& e : r2.elements) {
        if (!e.isConnector()) continue;
        tight += (e.turn == Turn::Left && e.speedLimit == r2.spec.tightSpeed) ? 1 : 0;
        for (size_t i = 0; i < e.pts.size(); i += 5) {
            worst = std::min(worst, footprintClearance(r2, e.pts[i].x, e.pts[i].y, e.pts[i].yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
        }
    }
    check(worst >= 0.0, "L11 every R2 movement, driven exactly, keeps the footprint with mirrors on the carriageway", f3(worst));
    check(tight >= 4, "L11 R2's tight junctions have slow left turns", std::to_string(tight));
    for (bool held : {false, true}) {
        for (const RouteSpec& rs : benchmarkRoutes(held)) {
            const LaneNetwork n = buildNetwork(rs.network);
            const int s = n.segmentAt(n.lane(rs.startLane), rs.startS), g = n.segmentAt(n.lane(rs.goalLane), rs.goalS);
            check(referenceRoute(n, s, g).found, "L11 benchmark route " + rs.name + " is legal and reachable");
        }
    }
}

// ---- L13 kerb geometry ---------------------------------------------------------------------------------
void testKerbGeometry() {
    for (const char* name : {"R1", "R2", "R3", "R4", "R5", "R6"}) {
        const LaneNetwork net = buildNetwork(name);
        std::uint64_t st = 12345;
        auto uni = [&](double a, double b) {
            st = st * 6364136223846793005ULL + 1442695040888963407ULL;
            return a + (b - a) * static_cast<double>(st >> 11) / 9007199254740992.0;
        };
        double x0 = 1e9, x1 = -1e9, y0 = 1e9, y1 = -1e9;
        for (const Junction& j : net.junctions) {
            x0 = std::min(x0, j.x); x1 = std::max(x1, j.x); y0 = std::min(y0, j.y); y1 = std::max(y1, j.y);
        }
        int signs = 0, lower = 0, inside = 0;
        for (int i = 0; i < 100000; ++i) {
            const double x = uni(x0 - 20, x1 + 70), y = uni(y0 - 20, y1 + 20);
            const double c = net.coarseClearance(x, y), e = net.roadClearance(x, y);
            if ((c >= 0.0) != (e >= 0.0) && std::fabs(c) > 1e-9) ++signs;
            if (c > 0.0) {
                ++inside;
                if (e < c - 1e-9) ++lower;
            }
        }
        check(signs == 0, std::string("L13 ") + name + ": the exact kerb distance has the conservative one's sign everywhere", std::to_string(signs));
        check(inside > 1000 && lower == 0, std::string("L13 ") + name + ": inside the carriageway it is never smaller than the conservative one");
        double off = 0.0;
        for (const KerbLine& k : net.kerbLines) {
            for (int s = 0; s <= 20; ++s) off = std::max(off, std::fabs(net.coarseClearance(k.ax + s / 20.0 * (k.bx - k.ax), k.ay + s / 20.0 * (k.by - k.ay))));
        }
        for (const KerbArc& a : net.kerbArcs) {
            for (int s = 0; s <= 20; ++s) {
                const double th = a.mid - a.half + 2.0 * a.half * s / 20.0;
                off = std::max(off, std::fabs(net.coarseClearance(a.cx + a.r * std::cos(th), a.cy + a.r * std::sin(th))));
            }
        }
        check(off < 1e-9, std::string("L13 ") + name + ": every kerb line and arc lies on the edge of the carriageway", f3(off));
        // Rays from inside the carriageway stop exactly where the conservative function changes sign.
        double worst = 0.0;
        int rays = 0;
        for (int i = 0; i < 3000 && rays < 300; ++i) {
            const double x = uni(x0 - 10, x1 + 10), y = uni(y0 - 10, y1 + 10);
            if (net.coarseClearance(x, y) < 0.05) continue;
            const double th = uni(-kPi, kPi), dx = std::cos(th), dy = std::sin(th);
            const double r = net.rayToKerb(x, y, dx, dy, 200.0);
            if (r < 0.0) continue;
            ++rays;
            worst = std::max({worst, std::fabs(net.roadClearance(x + r * dx, y + r * dy)), net.coarseClearance(x + (r - 0.01) * dx, y + (r - 0.01) * dy) < 0.0 ? 1.0 : 0.0});
        }
        check(rays >= 100 && worst < 1e-6, std::string("L13 ") + name + ": a ray stops at the first kerb it meets", f3(worst));
    }
    // The figure that was wrong: R2's ordinary left turns, driven exactly, clear the kerb by about 0.24 m with
    // mirrors (the conservative function said 0.000 because it ignored the rounded corner).
    const LaneNetwork r2 = buildR2();
    const VehicleParams vp;
    double leftExact = 1e9, leftCoarse = 1e9;
    for (const Element& e : r2.elements) {
        if (!e.isConnector() || e.turn != Turn::Left || e.speedLimit < r2.spec.speedLimit) continue;
        for (const PathPoint& p : e.pts) {
            leftExact = std::min(leftExact, footprintClearance(r2, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
            const double c = std::cos(p.yaw), s = std::sin(p.yaw);
            for (int i = 0; i <= 8; ++i) {
                const double u = -vp.rearOverhang + (vp.frontOffset() + vp.rearOverhang) * i / 8.0, w = 0.5 * vp.widthMirrors;
                leftCoarse = std::min(leftCoarse, r2.coarseClearance(p.x + c * u - s * w, p.y + s * u + c * w));
            }
        }
    }
    check(leftExact > 0.2 && leftCoarse < 0.01, "L13 R2 ordinary left turns: 0.24 m exact clearance, not the conservative 0.00", f3(leftExact) + " vs " + f3(leftCoarse));
}

// ---- L14 kerb scanner ----------------------------------------------------------------------------------
void testKerbScanner() {
    const LaneNetwork r2 = buildR2();
    const EpisodeResult r = record("R2-A", "nominal_h", 1);
    int scans = 0, returns = 0, onKerb = 0;
    for (const Measurement& m : r.sensors) {
        if (m.kind != Measurement::Kerb) continue;
        ++scans;
        const size_t k = static_cast<size_t>(std::lround(m.tMeas / 0.02));
        if (k >= r.truth.size()) continue;
        const TruthSample& t = r.truth[k];
        const double sx = t.x + m.c * std::cos(t.yaw), sy = t.y + m.c * std::sin(t.yaw);
        for (size_t i = 0; i < m.ranges.size(); ++i) {
            if (m.ranges[i] <= 0.0) continue;
            ++returns;
            const double th = t.yaw + m.a + static_cast<double>(i) * m.b;
            onKerb += std::fabs(r2.roadClearance(sx + m.ranges[i] * std::cos(th), sy + m.ranges[i] * std::sin(th))) < 0.25 ? 1 : 0;
        }
    }
    const double T = r.simSeconds;
    check(std::fabs(scans / T - 10.0) < 0.5, "L14 kerb scans at 10 Hz", f3(scans / T));
    check(returns > 0 && onKerb > 0.98 * returns, "L14 returns end on a mapped kerb from the true pose (range noise and kerb deviation within 0.25 m)",
          std::to_string(onKerb) + "/" + std::to_string(returns));
    // Kerb scans replay bit-identically through sensors.csv.
    std::vector<Measurement> back;
    std::string error;
    parseSensorLog(sensorLogCsv(r.sensors), back, error);
    bool same = back.size() == r.sensors.size();
    for (size_t i = 0; same && i < back.size(); ++i) same = back[i].ranges == r.sensors[i].ranges && back[i].kind == r.sensors[i].kind;
    check(same, "L14 kerb ranges survive the log format exactly", error);
    // Adding the scanner changed nothing else: the other sensors draw from their own random stream.
    const EpisodeResult c = record("R2-A", "kerb_clutter_r3", 1);
    bool othersSame = true;
    size_t a = 0, b = 0;
    while (othersSame && a < r.sensors.size() && b < c.sensors.size()) {
        if (r.sensors[a].kind == Measurement::Kerb) { ++a; continue; }
        if (c.sensors[b].kind == Measurement::Kerb) { ++b; continue; }
        othersSame = r.sensors[a].a == c.sensors[b].a && r.sensors[a].tMeas == c.sensors[b].tMeas;
        ++a;
        ++b;
    }
    check(othersSame, "L14 kerb faults leave every other measurement unchanged (common random numbers)");
    int parked = 0, blackout = 0;
    for (const Measurement& m : c.sensors) {
        if (m.kind != Measurement::Kerb) continue;
        for (double x : m.ranges) parked += (x > 0.0 && x < 3.0) ? 1 : 0;
    }
    for (const Zone& z : c.zones.kerbBlackout) {
        for (const Measurement& m : c.sensors) {
            const size_t k = static_cast<size_t>(std::lround(m.tMeas / 0.02));
            if (m.kind == Measurement::Kerb && k < c.truth.size() && z.contains(c.truth[k].x, c.truth[k].y)) ++blackout;
        }
    }
    check(!c.zones.obstacles.empty() && parked > 50, "L14 parked cars and clutter produce short, unmapped returns", std::to_string(parked));
    check(!c.zones.kerbBlackout.empty() && blackout == 0, "L14 no kerb scan inside a blackout zone");

    // GNSS fault generators (mechanics only; no estimator runs on these conditions here). The offset of each
    // fix from the true antenna position, minus the receiver's ordinary error, follows the configured shape.
    auto offsets = [&](const EpisodeResult& e) {
        std::vector<std::pair<double, double>> out;  // (time, offset magnitude)
        const GnssConfig g;
        for (const Measurement& m : e.sensors) {
            if (m.kind != Measurement::Gnss) continue;
            const size_t k = static_cast<size_t>(std::lround(m.tMeas / 0.02));
            if (k >= e.truth.size()) continue;
            const TruthSample& t = e.truth[k];
            out.push_back({m.tMeas, std::hypot(m.a - (t.x + std::cos(t.yaw) * g.antennaX), m.b - (t.y + std::sin(t.yaw) * g.antennaX))});
        }
        return out;
    };
    const EpisodeResult ramp = record("R2-A", "gnss_ramp_r3", 1);
    double early = 0.0, late = 0.0;
    int ne = 0, nl = 0;
    for (const auto& o : offsets(ramp)) {
        const double frac = o.first / ramp.simSeconds;
        if (frac < 0.3) { early += o.second; ++ne; }
        if (frac > 0.70 && frac < 0.76) { late += o.second; ++nl; }
    }
    check(ne > 0 && nl > 0 && early / ne < 1.0 && late / nl > 3.0, "L14 the GNSS ramp grows smoothly to metres with no step",
          f3(early / std::max(1, ne)) + " -> " + f3(late / std::max(1, nl)));
    double biggestStep = 0.0;
    const auto ro = offsets(ramp);
    for (size_t i = 1; i < ro.size(); ++i) biggestStep = std::max(biggestStep, std::fabs(ro[i].second - ro[i - 1].second));
    check(biggestStep < 1.6, "L14 the ramp has no fix-to-fix jump bigger than the receiver noise allows", f3(biggestStep));
    const EpisodeResult jumps = record("R2-A", "gnss_jumps_r3", 1);
    int switches = 0;
    bool on = false;
    for (const auto& o : offsets(jumps)) {
        const bool now = o.second > 2.0;
        switches += now != on ? 1 : 0;
        on = now;
    }
    check(switches >= 4, "L14 intermittent multipath switches on and off repeatedly", std::to_string(switches));
}

// ---- L15 GNSS integrity --------------------------------------------------------------------------------
void testIntegrity() {
    const LaneNetwork r1 = buildR1();
    const EstimatorConfig cfg;
    const std::vector<std::string> names = {"ekf", "ekf_int"};
    // A clean 6 m offset lasting most of the route: the timed reset adopts it; integrity never does.
    const EpisodeResult pb = record("R1-A", "test_persistent_bias", 1);
    const auto mp = evaluateLocalisation(pb.truth, pb.sensors, pb.outages, r1, cfg, nullptr, &names);
    check(find(mp, "ekf").max > 4.0, "L15 persistent bias: the timed reset (ekf) is captured by a 6 m offset", f3(find(mp, "ekf").max));
    check(find(mp, "ekf_int").max < 1.5 && find(mp, "ekf_int").reanchors == 0 && find(mp, "ekf_int").stepAlarms > 0,
          "L15 persistent bias: integrity detects the jump and never trusts the offset however long it lasts",
          f3(find(mp, "ekf_int").max) + " m, re-anchors " + std::to_string(find(mp, "ekf_int").reanchors));
    // Legitimate recovery: an unmodelled gyro-bias step inside an outage leaves the filter metres off and
    // overconfident. GNSS returning clean must be taken back without any timer.
    const EpisodeResult dr = record("R1-A", "test_drift_recovery", 1);
    const auto md = evaluateLocalisation(dr.truth, dr.sensors, dr.outages, r1, cfg, nullptr, &names);
    const LocMetrics& i = find(md, "ekf_int");
    check(i.outageEnd > 1.0, "L15 drift: the filter is more than 1 m off when GNSS returns", f3(i.outageEnd));
    check(i.reanchors >= 1 && i.recovery >= 0.0 && i.recovery < 5.0, "L15 drift: integrity re-anchors to the self-consistent fixes and recovers within 5 s",
          std::to_string(i.reanchors) + " re-anchors, recovery " + f3(i.recovery) + " s");
    check(i.resets == 0, "L15 drift: no timed reset is involved");
}

// ---- L16 kerb-aided filter -----------------------------------------------------------------------------
void testKerbFilter() {
    const LaneNetwork r2 = buildR2();
    const EstimatorConfig cfg;
    const std::vector<std::string> names = {"ekf", "ekf_kerb"};
    for (std::uint64_t seed : {1, 2}) {
        const EpisodeResult r = record("R2-B", "nominal_h", seed);
        const auto ms = evaluateLocalisation(r.truth, r.sensors, r.outages, r2, cfg, nullptr, &names);
        const LocMetrics& k = find(ms, "ekf_kerb");
        const std::string tag = ", seed " + std::to_string(seed);
        check(k.lateralP95 < 0.10 && k.lateralP95 < 0.5 * find(ms, "ekf").lateralP95, "L16 kerb registration: lateral p95 below 0.10 m and half the GNSS EKF's" + tag,
              f3(k.lateralP95) + " vs " + f3(find(ms, "ekf").lateralP95));
        check(k.neesMean > 0.5 && k.neesMean < 6.0, "L16 kerb registration: NEES within 0.5 to 6" + tag, f3(k.neesMean));
        check(k.kerbUsed > 0.9 * (k.kerbUsed + k.kerbRejected), "L16 kerb registration: nominal scans are used", std::to_string(k.kerbUsed));
    }
    // In an outage the kerbs hold the lateral position on their own.
    const EpisodeResult o = record("R2-A", "gnss_outage_h", 1);
    const auto mo = evaluateLocalisation(o.truth, o.sensors, o.outages, r2, cfg, nullptr, &names);
    check(find(mo, "ekf_kerb").lateralP95 < 0.10 && find(mo, "ekf_kerb").outageEnd < find(mo, "ekf").outageEnd,
          "L16 through R2-A's 127 m outage: kerb lateral p95 below 0.10 m, and less error than the GNSS EKF when GNSS returns",
          f3(find(mo, "ekf_kerb").lateralP95) + ", " + f3(find(mo, "ekf_kerb").outageEnd) + " vs " + f3(find(mo, "ekf").outageEnd));
}

// ---- L17 clearance feasibility -------------------------------------------------------------------------
void testClearance() {
    // The previous stack crosses the kerb at R2-B's right-turn exit on the GNSS EKF; the layer shifts the
    // path away from that kerb.
    Scenario s = benchmarkScenario(route("R2-B"), "nominal_h", 1, "estimated");
    s.estimator = "ekf";
    s.feasibility = false;
    const EpisodeResult before = runEpisode(s);
    s.feasibility = true;
    const EpisodeResult after = runEpisode(s);
    check(before.metrics.clearanceViolations > 0, "L17 R2-B seed 1 on the GNSS EKF: the previous stack puts the body over the kerb",
          std::to_string(before.metrics.clearanceViolations));
    check(after.metrics.clearanceViolations == 0 && after.outcome == "COMPLETED", "L17 with the clearance layer the same drive completes with the body on the road",
          after.outcome + ", " + std::to_string(after.metrics.clearanceViolations));
    // Where no shift gives the margin, the stack stops before that point and says so.
    Scenario o = benchmarkScenario(route("R2-A"), "gnss_outage_h", 1, "estimated");
    o.estimator = "ekf";
    const EpisodeResult stopped = runEpisode(o);
    bool explicitStop = false;
    for (const LogEvent& e : stopped.events) explicitStop = explicitStop || e.type == "NO_CLEAR_CONTINUATION";
    check(explicitStop && stopped.metrics.clearanceStops >= 1, "L17 an outage on a straight: the GNSS EKF's margin runs out and the stack stops explicitly");
    // On simulator truth the layer changes nothing on R2's routes (the reference keeps its path).
    for (const char* rn : {"R2-A", "R2-B", "R2-C"}) {
        Scenario g = benchmarkScenario(route(rn), "nominal_h", 2, "ground_truth");
        const EpisodeResult on = runEpisode(g);
        check(on.outcome == "COMPLETED" && on.metrics.clearanceStops == 0 && on.metrics.clearanceViolations == 0,
              std::string("L17 ground truth on ") + rn + ": completes, no clearance stop, body always on the road");
    }
    // The kerb filter keeps the margin through the same outage and completes.
    o.estimator = "ekf_kerb";
    const EpisodeResult kerb = runEpisode(o);
    check(kerb.outcome == "COMPLETED" && kerb.metrics.clearanceStops == 0 && kerb.metrics.clearanceViolations == 0,
          "L17 the kerb filter drives R2-A's outage without stopping and without touching a kerb", kerb.outcome);
}

// ---- L18 network R3 ------------------------------------------------------------------------------------
void testR3() {
    const LaneNetwork r3 = buildR3();
    const VehicleParams vp;
    check(r3.validate().empty(), "L18 R3 passes every map invariant");
    double worst = 1e9;
    for (const Element& e : r3.elements) {
        if (!e.isConnector()) continue;
        for (const PathPoint& p : e.pts) {
            worst = std::min(worst, footprintClearance(r3, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
        }
    }
    check(worst >= 0.0, "L18 every R3 movement, driven exactly, keeps the footprint with mirrors on the carriageway (exact kerb)", f3(worst));
    for (const RouteSpec& rs : benchmarkRoutesR3()) {
        const int s = r3.segmentAt(r3.lane(rs.startLane), rs.startS), g = r3.segmentAt(r3.lane(rs.goalLane), rs.goalS);
        check(referenceRoute(r3, s, g).found, "L18 R3 benchmark route " + rs.name + " is legal and reachable");
    }
    for (const std::string& c : conditionNamesR3()) {
        bool ok = true;
        try {
            sensorConditions(c);
        } catch (const std::exception&) {
            ok = false;
        }
        check(ok, "L18 R3 condition " + c + " is defined");
    }
}

// ---- L19 network R4 ------------------------------------------------------------------------------------
// Static checks only: R4 is held out, so no estimator runs on it here.
void testR4() {
    const LaneNetwork r4 = buildR4();
    const VehicleParams vp;
    check(r4.validate().empty(), "L19 R4 passes every map invariant");
    double worst = 1e9;
    for (const Element& e : r4.elements) {
        if (!e.isConnector()) continue;
        for (const PathPoint& p : e.pts) {
            worst = std::min(worst, footprintClearance(r4, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
        }
    }
    check(worst >= 0.0, "L19 every R4 movement, driven exactly, keeps the footprint with mirrors on the carriageway (exact kerb)", f3(worst));
    for (const RouteSpec& rs : benchmarkRoutesR4()) {
        const int s = r4.segmentAt(r4.lane(rs.startLane), rs.startS), g = r4.segmentAt(r4.lane(rs.goalLane), rs.goalS);
        const RefRoute rr = referenceRoute(r4, s, g);
        check(rr.found && routeLegal(r4, rr.nodes), "L19 R4 benchmark route " + rs.name + " is legal and reachable", rr.found ? routeText(r4, rr.nodes) : "");
    }
    // The closure lane lies on R4-B's route.
    const Scenario closure = closedLoopScenariosR4().back();
    const RouteSpec rb = route("R4-B");
    const RefRoute rr = referenceRoute(r4, r4.segmentAt(r4.lane(rb.startLane), rb.startS), r4.segmentAt(r4.lane(rb.goalLane), rb.goalS));
    bool onRoute = false;
    for (int n : rr.nodes) onRoute = onRoute || r4.elements[n].lane == r4.lane(closure.closureLane);
    check(closure.closure && onRoute, "L19 the R4 closure lane " + closure.closureLane + " lies on R4-B's route");
    for (const std::string& c : conditionNamesR4()) {
        bool ok = true;
        try {
            sensorConditions(c);
        } catch (const std::exception&) {
            ok = false;
        }
        check(ok, "L19 R4 condition " + c + " is defined");
    }
    // The kerb map error's mechanics, on R1: the same drive and seed with and without it. Returns change
    // only inside the zone, only on the scanner's left, and only outward by the shift over the incidence.
    const EpisodeResult a = record("R1-A", "nominal", 1);
    const EpisodeResult b = record("R1-A", "test_kerb_shift", 1);
    int shifted = 0, wrongSide = 0, wrongSize = 0, scans = 0;
    for (size_t i = 0, j = 0; i < a.sensors.size() && j < b.sensors.size(); ++i, ++j) {
        const Measurement& ma = a.sensors[i];
        const Measurement& mb = b.sensors[j];
        if (ma.kind != Measurement::Kind::Kerb || mb.kind != Measurement::Kind::Kerb || ma.ranges.size() != mb.ranges.size()) continue;
        ++scans;
        for (size_t k = 0; k < ma.ranges.size(); ++k) {
            if (ma.ranges[k] <= 0.0 || mb.ranges[k] <= 0.0) continue;
            const double d = mb.ranges[k] - ma.ranges[k];
            if (std::fabs(d) < 1e-9) continue;
            ++shifted;
            wrongSide += ma.a + static_cast<double>(k) * ma.b > 0.0 ? 0 : 1;
            wrongSize += d >= 0.5 - 1e-9 ? 0 : 1;
        }
    }
    check(a.sensors.size() == b.sensors.size() && scans > 100 && shifted > 100 && wrongSide == 0 && wrongSize == 0,
          "L19 kerb map error: returns from the left kerb inside the zone, and only those, read at least 0.5 m longer",
          std::to_string(shifted) + " shifted returns in " + std::to_string(scans) + " scans; wrong side " + std::to_string(wrongSide) +
              ", too small " + std::to_string(wrongSize));
}

// ---- L20 second versions (ekf_int2, ekf_kerb2) ----------------------------------------------------------
bool sameTrace(const std::vector<StateEstimate>& a, const std::vector<StateEstimate>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].valid != b[i].valid || a[i].x != b[i].x || a[i].y != b[i].y || a[i].yaw != b[i].yaw || a[i].pxx != b[i].pxx || a[i].pyy != b[i].pyy) {
            return false;
        }
    }
    return true;
}

void testSecondVersions() {
    const EstimatorConfig cfg;
    // Where no fault ends and no scan is lost, each second version is its first version exactly.
    {
        const LaneNetwork r2 = buildR2();
        const std::vector<std::string> names = {"ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"};
        const EpisodeResult r = record("R2-B", "nominal_h", 1);
        std::vector<std::vector<StateEstimate>> tr;
        const auto ms = evaluateLocalisation(r.truth, r.sensors, r.outages, r2, cfg, &tr, &names);
        check(tr.size() == 4 && sameTrace(tr[0], tr[1]), "L20 nominal: ekf_int2 reproduces ekf_int's trace exactly");
        check(tr.size() == 4 && sameTrace(tr[2], tr[3]) && find(ms, "ekf_kerb2").kerbRelocalisations == 0,
              "L20 nominal: ekf_kerb2 reproduces ekf_kerb's trace exactly and never re-localises");
    }
    // L15's two cases: a persistent clean offset is never trusted; clean fixes after drift are re-anchored.
    {
        const LaneNetwork r1 = buildR1();
        const std::vector<std::string> names = {"ekf_int2"};
        const EpisodeResult pb = record("R1-A", "test_persistent_bias", 1);
        const LocMetrics p = evaluateLocalisation(pb.truth, pb.sensors, pb.outages, r1, cfg, nullptr, &names).front();
        check(p.max < 1.5 && p.reanchors == 0, "L20 persistent bias: ekf_int2 never trusts the offset (it leaves the fault only when the offset ends)",
              f3(p.max) + " m, re-anchors " + std::to_string(p.reanchors) + ", fault recoveries " + std::to_string(p.faultRecoveries));
        const EpisodeResult dr = record("R1-A", "test_drift_recovery", 1);
        const LocMetrics d = evaluateLocalisation(dr.truth, dr.sensors, dr.outages, r1, cfg, nullptr, &names).front();
        check(d.reanchors >= 1 && d.recovery >= 0.0 && d.recovery < 5.0 && d.resets == 0, "L20 drift: ekf_int2 re-anchors without a timer and recovers within 5 s",
              std::to_string(d.reanchors) + " re-anchors, recovery " + f3(d.recovery) + " s");
    }
    // R3's first failure (now development data): combined faults leave ekf_int faulted for the rest of the
    // drive; ekf_int2 leaves the fault when the receiver jumps back and re-anchors from the track angle.
    {
        const LaneNetwork r3 = buildR3();
        const std::vector<std::string> names = {"ekf_int", "ekf_int2"};
        const EpisodeResult c = record("R3-A", "combined_r3", 1);
        std::vector<std::vector<StateEstimate>> tr;
        const auto ms = evaluateLocalisation(c.truth, c.sensors, c.outages, r3, cfg, &tr, &names);
        const LocMetrics& a = find(ms, "ekf_int");
        const LocMetrics& b = find(ms, "ekf_int2");
        const TruthSample& end = c.truth.back();
        const StateEstimate& eb = tr[1].back();
        const double endErr = std::hypot(eb.x - end.x, eb.y - end.y);
        check(a.max > 20.0, "L20 R3-A combined, seed 1: ekf_int never leaves the fault and ends far off", f3(a.max));
        check(b.faultRecoveries >= 1 && b.headingReanchors >= 1 && endErr < 1.0,
              "L20 the same drive: ekf_int2 leaves the fault, re-anchors on the track angle and ends within 1 m",
              f3(b.max) + " m max, " + f3(endErr) + " m at the end");
    }
    // R3's second failure: a slow GNSS excursion drags ekf_kerb off its kerbs, after which every scan is
    // rejected; ekf_kerb2 finds the pose the scans fit and returns to them.
    {
        const LaneNetwork r3 = buildR3();
        const std::vector<std::string> names = {"ekf_kerb", "ekf_kerb2"};
        const EpisodeResult rp = record("R3-B", "gnss_ramp_r3", 2);
        const auto ms = evaluateLocalisation(rp.truth, rp.sensors, rp.outages, r3, cfg, nullptr, &names);
        const LocMetrics& a = find(ms, "ekf_kerb");
        const LocMetrics& b = find(ms, "ekf_kerb2");
        check(a.lateralP95 > 1.0, "L20 R3-B ramp, seed 2: ekf_kerb follows the excursion off its kerbs", f3(a.lateralP95));
        check(b.kerbRelocalisations >= 1 && b.lateralP95 < 0.2 && b.max < 2.0, "L20 the same drive: ekf_kerb2 re-localises on the kerbs and holds them",
              std::to_string(b.kerbRelocalisations) + " re-localisations, lateral p95 " + f3(b.lateralP95) + ", max " + f3(b.max));
    }
}

// ---- L21 pose corrections in the stack ------------------------------------------------------------------
void testPoseCorrections() {
    auto drive = [](const std::string& routeName, const std::string& condition, std::uint64_t seed, int& corrections, int& fallbacks,
                    Rejoin rejoin = Rejoin::Verified) {
        Scenario s = benchmarkScenario(route(routeName), condition, seed, "estimated");
        s.estimator = "ekf_kerb2";
        s.rejoin = rejoin;
        const EpisodeResult r = runEpisode(s);
        corrections = fallbacks = 0;
        for (const LogEvent& e : r.events) {
            corrections += e.type == "POSE_CORRECTION" ? 1 : 0;
            fallbacks += e.type == "MONITOR_TRIGGER" ? 1 : 0;
        }
        return r;
    };
    int corrections = 0, fallbacks = 0;
    // R3-B under the slow excursion, seed 9: ekf_kerb2 re-localises by about 1.2 m in the middle of a turn.
    // Chasing the old reference turned that into a steering oscillation and a fallback stop over the kerb.
    const EpisodeResult a = drive("R3-B", "gnss_ramp_r3", 9, corrections, fallbacks);
    check(corrections >= 1 && fallbacks == 0 && a.outcome == "COMPLETED" && a.metrics.clearanceViolations == 0,
          "L21 R3-B slow excursion, seed 9: the verified re-join follows the mid-turn kerb re-localisation and completes on the road",
          a.outcome + ", corrections " + std::to_string(corrections) + ", fallbacks " + std::to_string(fallbacks));
    // R4-C under kerb faults, seed 1: GNSS returns and corrects the estimate where the clearance layer has
    // shifted the path. Re-joining every jump (the stack evaluated on R5) ends it without a fallback stop; the
    // verified re-join leaves that unverified GNSS correction to the monitor, as the R3 and R4 stack did.
    const EpisodeResult b = drive("R4-C", "kerb_faults_r4", 1, corrections, fallbacks, Rejoin::All);
    check(corrections >= 1 && fallbacks == 0 && b.outcome == "COMPLETED" && b.metrics.clearanceViolations == 0,
          "L21 R4-C kerb faults, seed 1: re-joining every jump, GNSS corrections near a shifted path end in no fallback stop",
          b.outcome + ", corrections " + std::to_string(corrections) + ", fallbacks " + std::to_string(fallbacks));
    const EpisodeResult c = drive("R4-C", "kerb_faults_r4", 1, corrections, fallbacks, Rejoin::Verified);
    check(corrections == 0 && fallbacks == 1 && c.metrics.clearanceViolations == 0,
          "L21 the same drive with the verified re-join: no re-join for an unverified GNSS correction, the monitor stops it",
          c.outcome + ", corrections " + std::to_string(corrections) + ", fallbacks " + std::to_string(fallbacks));
}

// ---- L22 network R5 ------------------------------------------------------------------------------------
// Static checks only: R5 is held out, so nothing drives on it here.
void testR5() {
    const LaneNetwork r5 = buildR5();
    const VehicleParams vp;
    check(r5.validate().empty(), "L22 R5 passes every map invariant");
    double worst = 1e9;
    for (const Element& e : r5.elements) {
        if (!e.isConnector()) continue;
        for (const PathPoint& p : e.pts) {
            worst = std::min(worst, footprintClearance(r5, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
        }
    }
    check(worst >= 0.0, "L22 every R5 movement, driven exactly, keeps the footprint with mirrors on the carriageway (exact kerb)", f3(worst));
    for (const RouteSpec& rs : benchmarkRoutesR5()) {
        const int s = r5.segmentAt(r5.lane(rs.startLane), rs.startS), g = r5.segmentAt(r5.lane(rs.goalLane), rs.goalS);
        const RefRoute rr = referenceRoute(r5, s, g);
        check(rr.found && routeLegal(r5, rr.nodes), "L22 R5 benchmark route " + rs.name + " is legal and reachable", rr.found ? routeText(r5, rr.nodes) : "");
    }
    const Scenario closure = closedLoopScenariosR5().back();
    const RouteSpec rb = route("R5-B");
    const RefRoute rr = referenceRoute(r5, r5.segmentAt(r5.lane(rb.startLane), rb.startS), r5.segmentAt(r5.lane(rb.goalLane), rb.goalS));
    bool onRoute = false;
    for (int n : rr.nodes) onRoute = onRoute || r5.elements[n].lane == r5.lane(closure.closureLane);
    check(closure.closure && onRoute, "L22 the R5 closure lane " + closure.closureLane + " lies on R5-B's route");
    for (const std::string& c : conditionNamesR5()) {
        bool ok = true;
        try {
            sensorConditions(c);
        } catch (const std::exception&) {
            ok = false;
        }
        check(ok, "L22 R5 condition " + c + " is defined");
    }
}

// ---- L23 the re-join switch -----------------------------------------------------------------------------
// Off is the stack evaluated on R3 and R4: the same drive as L21's first case fails as it did there.
void testRejoinSwitch() {
    Scenario s = benchmarkScenario(route("R3-B"), "gnss_ramp_r3", 9, "estimated");
    s.estimator = "ekf_kerb2";
    s.rejoin = Rejoin::Off;
    const EpisodeResult off = runEpisode(s);
    int corrections = 0, fallbacks = 0;
    for (const LogEvent& e : off.events) {
        corrections += e.type == "POSE_CORRECTION" ? 1 : 0;
        fallbacks += e.type == "MONITOR_TRIGGER" ? 1 : 0;
    }
    check(corrections == 0 && fallbacks == 1 && off.outcome == "STOPPED_EN_ROUTE" && off.metrics.clearanceViolations > 0,
          "L23 with the re-join off, R3-B seed 9 ends in the recorded fallback stop over the kerb",
          off.outcome + ", fallbacks " + std::to_string(fallbacks) + ", crossing samples " + std::to_string(off.metrics.clearanceViolations));
    // On simulator truth nothing jumps: the switch changes nothing.
    Scenario g = benchmarkScenario(route("R3-B"), "gnss_ramp_r3", 9, "ground_truth");
    g.rejoin = Rejoin::Off;
    const EpisodeResult a = runEpisode(g);
    g.rejoin = Rejoin::All;
    const EpisodeResult b = runEpisode(g);
    g.rejoin = Rejoin::Verified;
    const EpisodeResult c = runEpisode(g);
    check(a.traceHash == b.traceHash && a.traceHash == c.traceHash, "L23 on simulator truth every re-join setting leaves the trace unchanged");
    // A GNSS-only filter reports no map-verified correction, so the verified re-join drives it exactly as the
    // R3 and R4 stack does, even where the estimate jumps (R3-B, slow excursion, seed 9).
    Scenario q = benchmarkScenario(route("R3-B"), "gnss_ramp_r3", 9, "estimated");
    q.estimator = "ekf_int2";
    q.rejoin = Rejoin::Off;
    const EpisodeResult qOff = runEpisode(q);
    q.rejoin = Rejoin::Verified;
    const EpisodeResult qVer = runEpisode(q);
    check(qOff.traceHash == qVer.traceHash, "L23 a GNSS-only filter drives identically with the verified re-join and without any");
}

// ---- L24 network R6 ------------------------------------------------------------------------------------
// Static checks only: R6 is held out, so nothing drives on it here.
void testR6() {
    const LaneNetwork r6 = buildR6();
    const VehicleParams vp;
    check(r6.validate().empty(), "L24 R6 passes every map invariant");
    double worst = 1e9;
    for (const Element& e : r6.elements) {
        if (!e.isConnector()) continue;
        for (const PathPoint& p : e.pts) {
            worst = std::min(worst, footprintClearance(r6, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors));
        }
    }
    check(worst >= 0.0, "L24 every R6 movement, driven exactly, keeps the footprint with mirrors on the carriageway (exact kerb)", f3(worst));
    for (const RouteSpec& rs : benchmarkRoutesR6()) {
        const int s = r6.segmentAt(r6.lane(rs.startLane), rs.startS), g = r6.segmentAt(r6.lane(rs.goalLane), rs.goalS);
        const RefRoute rr = referenceRoute(r6, s, g);
        check(rr.found && routeLegal(r6, rr.nodes), "L24 R6 benchmark route " + rs.name + " is legal and reachable", rr.found ? routeText(r6, rr.nodes) : "");
    }
    const Scenario closure = closedLoopScenariosR6().back();
    const RouteSpec rb = route("R6-B");
    const RefRoute rr = referenceRoute(r6, r6.segmentAt(r6.lane(rb.startLane), rb.startS), r6.segmentAt(r6.lane(rb.goalLane), rb.goalS));
    bool onRoute = false;
    for (int n : rr.nodes) onRoute = onRoute || r6.elements[n].lane == r6.lane(closure.closureLane);
    check(closure.closure && onRoute, "L24 the R6 closure lane " + closure.closureLane + " lies on R6-B's route");
    for (const std::string& c : conditionNamesR6()) {
        bool ok = true;
        try {
            sensorConditions(c);
        } catch (const std::exception&) {
            ok = false;
        }
        check(ok, "L24 R6 condition " + c + " is defined");
    }
}

}  // namespace

int main() {
    testSensors();
    testReplayAndDelays();
    testEstimators();
    testClosedLoop();
    testHeldOutNetwork();
    testKerbGeometry();
    testKerbScanner();
    testIntegrity();
    testKerbFilter();
    testClearance();
    testR3();
    testR4();
    testSecondVersions();
    testPoseCorrections();
    testR5();
    testRejoinSwitch();
    testR6();
    std::printf("localisation tests: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
