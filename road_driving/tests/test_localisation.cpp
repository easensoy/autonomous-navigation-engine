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
    int n[4] = {0, 0, 0, 0};
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

}  // namespace

int main() {
    testSensors();
    testReplayAndDelays();
    testEstimators();
    testClosedLoop();
    testHeldOutNetwork();
    std::printf("localisation tests: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
