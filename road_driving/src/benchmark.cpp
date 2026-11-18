#include "road_driving/benchmark.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace road {

namespace {

// Two-sided 95% chi-square intervals.
constexpr double kChi2Lo[4] = {0.0, 0.000982, 0.0506, 0.2158};
constexpr double kChi2Hi[4] = {0.0, 5.0239, 7.3778, 9.3484};

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return -1.0;
    std::sort(v.begin(), v.end());
    const size_t i = std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())));
    return v[i];
}

double median(std::vector<double> v) { return percentile(std::move(v), 0.5); }

std::string num(double v, int prec = 3) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

void writeFile(const std::filesystem::path& p, const std::string& text) {
    std::ofstream out(p, std::ios::binary);
    out << text;
    if (!out) throw std::runtime_error("cannot write " + p.string());
}

}  // namespace

std::vector<LocMetrics> evaluateLocalisation(const std::vector<TruthSample>& truth, const std::vector<Measurement>& sensors,
                                             const std::vector<std::pair<double, double>>& outages, const LaneNetwork& net,
                                             const EstimatorConfig& cfg, std::vector<std::vector<StateEstimate>>* traces,
                                             const std::vector<std::string>* names) {
    std::vector<LocMetrics> all;
    // The longest outage, for drift and recovery.
    std::pair<double, double> longest{-1.0, -1.0};
    for (const auto& w : outages) {
        if (w.second - w.first > longest.second - longest.first) longest = w;
    }
    const std::vector<std::string> list = names ? *names : estimatorNames();
    for (const std::string& name : list) {
        std::unique_ptr<Estimator> est = makeEstimator(name, cfg, net);
        std::vector<StateEstimate> trace;
        trace.reserve(truth.size());
        size_t mi = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (const TruthSample& ts : truth) {
            while (mi < sensors.size() && sensors[mi].tArrival <= ts.t + 1e-9) {
                est->process(sensors[mi++]);
            }
            trace.push_back(est->estimateAt(ts.t));
        }
        LocMetrics m;
        m.estimator = name;
        m.runtimeMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        std::vector<double> head, speed, nees;
        double s2 = 0.0, h2 = 0.0, v2 = 0.0;
        int neesIn = 0;
        for (size_t i = 0; i < truth.size(); ++i) {
            const StateEstimate& e = trace[i];
            const TruthSample& ts = truth[i];
            if (!e.valid) {
                if (m.initTime >= 0.0) m.failed = true;  // lost after initialising
                continue;
            }
            if (m.initTime < 0.0) m.initTime = ts.t;
            const double dx = e.x - ts.x, dy = e.y - ts.y, dyaw = wrapAngle(e.yaw - ts.yaw);
            const double err = std::hypot(dx, dy);
            m.errors.push_back(err);
            m.lateral.push_back(std::fabs(-std::sin(ts.yaw) * dx + std::cos(ts.yaw) * dy));
            s2 += err * err;
            h2 += dyaw * dyaw;
            v2 += (e.v - ts.v) * (e.v - ts.v);
            m.headingMaxDeg = std::max(m.headingMaxDeg, std::fabs(dyaw) * 180.0 / kPi);
            m.failed = m.failed || err > kFailError;
            if (e.hasCov) {
                double n = 0.0;
                int dof = 2;
                if (std::isfinite(e.pyawyaw)) {
                    // Inverse of the 3x3 (x, y, heading) covariance by cofactors.
                    const double a = e.pxx, b = e.pxy, c = e.pxyaw, d = e.pyy, f = e.pyyaw, g = e.pyawyaw;
                    const double A = d * g - f * f, B = c * f - b * g, C = b * f - c * d, D = a * g - c * c, F = b * c - a * f, G = a * d - b * b;
                    const double det = a * A + b * B + c * C;
                    if (det > 1e-30) {
                        n = (A * dx * dx + D * dy * dy + G * dyaw * dyaw + 2.0 * (B * dx * dy + C * dx * dyaw + F * dy * dyaw)) / det;
                        dof = 3;
                    } else {
                        continue;
                    }
                } else {
                    const double det = e.pxx * e.pyy - e.pxy * e.pxy;
                    if (det <= 1e-30) continue;
                    n = (e.pyy * dx * dx - 2.0 * e.pxy * dx * dy + e.pxx * dy * dy) / det;
                }
                m.neesDof = dof;
                nees.push_back(n);
                neesIn += (n >= kChi2Lo[dof] && n <= kChi2Hi[dof]) ? 1 : 0;
            }
        }
        m.samples = static_cast<int>(m.errors.size());
        if (m.samples == 0) {
            m.failed = true;
        } else {
            m.rmse = std::sqrt(s2 / m.samples);
            m.headingRmseDeg = std::sqrt(h2 / m.samples) * 180.0 / kPi;
            m.speedRmse = std::sqrt(v2 / m.samples);
            m.median = percentile(m.errors, 0.5);
            m.p95 = percentile(m.errors, 0.95);
            m.max = *std::max_element(m.errors.begin(), m.errors.end());
            double l2 = 0.0;
            for (double l : m.lateral) l2 += l * l;
            m.lateralRmse = std::sqrt(l2 / static_cast<double>(m.lateral.size()));
            m.lateralP95 = percentile(m.lateral, 0.95);
            m.lateralMax = *std::max_element(m.lateral.begin(), m.lateral.end());
        }
        if (!nees.empty()) {
            double sum = 0.0;
            for (double n : nees) sum += n;
            m.neesMean = sum / static_cast<double>(nees.size());
            m.neesInside = static_cast<double>(neesIn) / static_cast<double>(nees.size());
        }
        // Relative position error over fixed time windows (50 Hz samples).
        for (int w : {5, 20}) {
            const size_t n = static_cast<size_t>(w * 50);
            double sum = 0.0;
            int cnt = 0;
            for (size_t i = 0; i + n < truth.size(); i += 5) {
                if (!trace[i].valid || !trace[i + n].valid) continue;
                const double ex = (trace[i + n].x - trace[i].x) - (truth[i + n].x - truth[i].x);
                const double ey = (trace[i + n].y - trace[i].y) - (truth[i + n].y - truth[i].y);
                sum += ex * ex + ey * ey;
                ++cnt;
            }
            (w == 5 ? m.rpe5 : m.rpe20) = cnt > 0 ? std::sqrt(sum / cnt) : -1.0;
        }
        // Drift through the longest outage and the recovery after it.
        if (longest.second > longest.first) {
            double dist = 0.0, emax = 0.0, eend = -1.0;
            for (size_t i = 0; i < truth.size(); ++i) {
                const double t = truth[i].t;
                if (t < longest.first || t > longest.second || !trace[i].valid) continue;
                const double e = std::hypot(trace[i].x - truth[i].x, trace[i].y - truth[i].y);
                emax = std::max(emax, e);
                eend = e;
                if (i > 0 && truth[i - 1].t >= longest.first) dist += std::hypot(truth[i].x - truth[i - 1].x, truth[i].y - truth[i - 1].y);
            }
            m.outageMax = emax;
            m.outageEnd = eend;
            m.outageDistance = dist;
            m.recovery = 999.0;
            double goodSince = -1.0;
            for (size_t i = 0; i < truth.size(); ++i) {
                if (truth[i].t < longest.second || !trace[i].valid) continue;
                const double e = std::hypot(trace[i].x - truth[i].x, trace[i].y - truth[i].y);
                if (e < 0.5) {
                    if (goodSince < 0.0) goodSince = truth[i].t;
                    if (truth[i].t - goodSince >= 1.0) {
                        m.recovery = goodSince - longest.second;
                        break;
                    }
                } else {
                    goodSince = -1.0;
                }
            }
        }
        const EstimatorStats& st = est->stats;
        auto nisSummary = [](const std::vector<double>& v, int dof, double& mean, double& inside) {
            if (v.empty()) return;
            double sum = 0.0;
            int in = 0;
            for (double x : v) {
                sum += std::min(x, 1e6);
                in += (x >= kChi2Lo[dof] && x <= kChi2Hi[dof]) ? 1 : 0;
            }
            mean = sum / static_cast<double>(v.size());
            inside = static_cast<double>(in) / static_cast<double>(v.size());
        };
        nisSummary(st.nisGnss, 2, m.nisGnssMean, m.nisGnssInside);
        nisSummary(st.nisWheel, 1, m.nisWheelMean, m.nisWheelInside);
        m.gnssUsed = st.gnssUsed;
        m.gnssRejected = st.gnssRejected;
        m.wheelRejected = st.wheelRejected;
        m.resets = st.resets;
        m.lateDropped = st.lateDropped;
        m.latencyMedianUs = percentile(st.latencyUs, 0.5);
        m.latencyP99Us = percentile(st.latencyUs, 0.99);
        m.latencyMaxUs = st.latencyUs.empty() ? 0.0 : *std::max_element(st.latencyUs.begin(), st.latencyUs.end());
        m.stepAlarms = st.stepAlarms;
        m.faultExits = st.faultExits;
        m.gnssExcluded = st.gnssExcluded;
        m.reanchors = st.reanchors;
        m.kerbUsed = st.kerbUsed;
        m.faultRecoveries = st.faultRecoveries;
        m.headingReanchors = st.headingReanchors;
        m.kerbRelocalisations = st.kerbRelocalisations;
        m.kerbRejected = st.kerbRejected;
        if (!st.nisKerb.empty()) {
            double s = 0.0;
            for (double x : st.nisKerb) s += std::min(x, 1e6);
            m.nisKerbMean = s / static_cast<double>(st.nisKerb.size());
        }
        if (traces) traces->push_back(std::move(trace));
        all.push_back(std::move(m));
    }
    return all;
}

std::string locMetricsJson(const LocMetrics& m) {
    std::ostringstream o;
    o << "\"estimator\":\"" << m.estimator << "\",\"failed\":" << (m.failed ? "true" : "false") << ",\"init_time\":" << num(m.initTime, 2)
      << ",\"samples\":" << m.samples << ",\"rmse\":" << num(m.rmse, 4) << ",\"median\":" << num(m.median, 4) << ",\"p95\":" << num(m.p95, 4)
      << ",\"max\":" << num(m.max, 4) << ",\"heading_rmse_deg\":" << num(m.headingRmseDeg, 3) << ",\"heading_max_deg\":" << num(m.headingMaxDeg, 3)
      << ",\"speed_rmse\":" << num(m.speedRmse, 4) << ",\"rpe5\":" << num(m.rpe5, 4) << ",\"rpe20\":" << num(m.rpe20, 4)
      << ",\"outage_end\":" << num(m.outageEnd, 4) << ",\"outage_max\":" << num(m.outageMax, 4) << ",\"outage_distance\":" << num(m.outageDistance, 1)
      << ",\"recovery\":" << num(m.recovery, 2) << ",\"nees_dof\":" << m.neesDof << ",\"nees_mean\":" << num(m.neesMean, 3)
      << ",\"nees_inside\":" << num(m.neesInside, 3) << ",\"nis_gnss_mean\":" << num(m.nisGnssMean, 3) << ",\"nis_gnss_inside\":" << num(m.nisGnssInside, 3)
      << ",\"nis_wheel_mean\":" << num(m.nisWheelMean, 3) << ",\"nis_wheel_inside\":" << num(m.nisWheelInside, 3) << ",\"gnss_used\":" << m.gnssUsed
      << ",\"gnss_rejected\":" << m.gnssRejected << ",\"wheel_rejected\":" << m.wheelRejected << ",\"resets\":" << m.resets
      << ",\"late_dropped\":" << m.lateDropped << ",\"runtime_ms_wall\":" << num(m.runtimeMs, 2) << ",\"latency_median_us_wall\":" << num(m.latencyMedianUs, 2)
      << ",\"latency_p99_us_wall\":" << num(m.latencyP99Us, 2) << ",\"latency_max_us_wall\":" << num(m.latencyMaxUs, 1)
      << ",\"lateral_rmse\":" << num(m.lateralRmse, 4) << ",\"lateral_p95\":" << num(m.lateralP95, 4) << ",\"lateral_max\":" << num(m.lateralMax, 4)
      << ",\"step_alarms\":" << m.stepAlarms << ",\"fault_exits\":" << m.faultExits << ",\"gnss_excluded\":" << m.gnssExcluded
      << ",\"reanchors\":" << m.reanchors << ",\"kerb_used\":" << m.kerbUsed << ",\"kerb_rejected\":" << m.kerbRejected
      << ",\"nis_kerb_mean\":" << num(m.nisKerbMean, 3) << ",\"fault_recoveries\":" << m.faultRecoveries << ",\"heading_reanchors\":" << m.headingReanchors
      << ",\"kerb_relocalisations\":" << m.kerbRelocalisations;
    return o.str();
}

std::vector<RouteSpec> benchmarkRoutes(bool heldOut) {
    if (!heldOut) {
        return {{"R1-A", "R1", "J00>J10", 7.5, "J21>J22", 47.5}, {"R1-B", "R1", "J02>J12", 10.0, "J20>J10", 40.0},
                {"R1-C", "R1", "J20>J21", 10.0, "J01>J00", 30.0}};
    }
    return {{"R2-A", "R2", "J01>J11", 10.0, "J31>J32", 40.0}, {"R2-B", "R2", "J30>J20", 10.0, "J01>J02", 30.0},
            {"R2-C", "R2", "J02>J12", 10.0, "J11>J01", 30.0}};
}

Scenario benchmarkScenario(const RouteSpec& route, const std::string& condition, std::uint64_t seed, const std::string& mode) {
    Scenario sc;
    sc.name = route.name;
    sc.scenarioClass = "LOC";
    sc.network = route.network;
    sc.startLane = route.startLane;
    sc.startLaneS = route.startS;
    sc.goalLane = route.goalLane;
    sc.goalLaneS = route.goalS;
    sc.condition = condition;
    sc.mode = mode;
    sc.seed = seed;
    sc.departDelay = 2.0;
    return sc;
}

std::vector<Scenario> closedLoopScenarios(bool heldOut) {
    std::vector<Scenario> out;
    if (!heldOut) {
        for (const char* n : {"c0_nominal", "c2_advance", "c3_r30", "c5_lift"}) {
            Scenario s;
            findScenario(n, s);
            s.departDelay = 2.0;
            out.push_back(s);
        }
        return out;
    }
    for (const RouteSpec& r : benchmarkRoutes(true)) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutes(true)[0], "", 0, "ground_truth");
    c.name = "R2-A_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J31>J32";
    c.barrierLaneS = 30.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

std::vector<RouteSpec> benchmarkRoutesR3() {
    return {{"R3-A", "R3", "J00>J10", 10.0, "J23>J33", 40.0}, {"R3-B", "R3", "J30>J20", 10.0, "J02>J03", 30.0},
            {"R3-C", "R3", "J13>J12", 10.0, "J21>J31", 30.0}};
}

std::vector<Scenario> closedLoopScenariosR3() {
    std::vector<Scenario> out;
    for (const RouteSpec& r : benchmarkRoutesR3()) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutesR3()[1], "", 0, "ground_truth");
    c.name = "R3-B_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J01>J02";
    c.barrierLaneS = 35.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

std::vector<RouteSpec> benchmarkRoutesR4() {
    return {{"R4-A", "R4", "J00>J10", 10.0, "J41>J42", 30.0}, {"R4-B", "R4", "J41>J31", 10.0, "J10>J00", 30.0},
            {"R4-C", "R4", "J02>J01", 10.0, "J22>J32", 30.0}};
}

std::vector<Scenario> closedLoopScenariosR4() {
    std::vector<Scenario> out;
    for (const RouteSpec& r : benchmarkRoutesR4()) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutesR4()[1], "", 0, "ground_truth");
    c.name = "R4-B_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J20>J10";
    c.barrierLaneS = 35.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

std::vector<RouteSpec> benchmarkRoutesR5() {
    return {{"R5-A", "R5", "J00>J10", 10.0, "J14>J24", 30.0}, {"R5-B", "R5", "J24>J23", 10.0, "J01>J00", 30.0},
            {"R5-C", "R5", "J04>J03", 10.0, "J23>J13", 30.0}};
}

std::vector<Scenario> closedLoopScenariosR5() {
    std::vector<Scenario> out;
    for (const RouteSpec& r : benchmarkRoutesR5()) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutesR5()[1], "", 0, "ground_truth");
    c.name = "R5-B_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J12>J02";
    c.barrierLaneS = 35.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

std::vector<RouteSpec> benchmarkRoutesR6() {
    return {{"R6-A", "R6", "J00>J10", 10.0, "J23>J33", 30.0}, {"R6-B", "R6", "J33>J32", 10.0, "J01>J00", 30.0},
            {"R6-C", "R6", "J03>J13", 10.0, "J20>J30", 30.0}};
}

std::vector<Scenario> closedLoopScenariosR6() {
    std::vector<Scenario> out;
    for (const RouteSpec& r : benchmarkRoutesR6()) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutesR6()[1], "", 0, "ground_truth");
    c.name = "R6-B_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J22>J12";
    c.barrierLaneS = 35.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

std::vector<RouteSpec> benchmarkRoutesR7() {
    return {{"R7-A", "R7", "J00>J10", 10.0, "J33>J43", 30.0}, {"R7-B", "R7", "J43>J33", 10.0, "J01>J00", 30.0},
            {"R7-C", "R7", "J03>J13", 10.0, "J40>J30", 30.0}};
}

std::vector<Scenario> closedLoopScenariosR7() {
    std::vector<Scenario> out;
    for (const RouteSpec& r : benchmarkRoutesR7()) {
        out.push_back(benchmarkScenario(r, "", 0, "ground_truth"));
    }
    Scenario c = benchmarkScenario(benchmarkRoutesR7()[1], "", 0, "ground_truth");
    c.name = "R7-B_late_closure";
    c.scenarioClass = "C3";
    c.closure = true;
    c.closureLane = "J23>J13";
    c.barrierLaneS = 35.0;
    c.worksLength = 15.0;
    c.reveal = Reveal::BeforeBarrier;
    c.revealDistance = 30.0;
    out.push_back(c);
    return out;
}

bool findNamedScenario(const std::string& name, Scenario& sc) {
    if (findScenario(name, sc)) return true;
    for (bool held : {false, true}) {
        for (const RouteSpec& r : benchmarkRoutes(held)) {
            if (r.name == name) {
                sc = benchmarkScenario(r, "", 0, "ground_truth");
                return true;
            }
        }
        for (const Scenario& s : closedLoopScenarios(held)) {
            if (s.name == name) {
                sc = s;
                return true;
            }
        }
    }
    for (int n : {3, 4, 5, 6, 7}) {
        for (const RouteSpec& r : n == 3   ? benchmarkRoutesR3()
                                  : n == 4 ? benchmarkRoutesR4()
                                  : n == 5 ? benchmarkRoutesR5()
                                  : n == 6 ? benchmarkRoutesR6()
                                           : benchmarkRoutesR7()) {
            if (r.name == name) {
                sc = benchmarkScenario(r, "", 0, "ground_truth");
                return true;
            }
        }
        for (const Scenario& s : n == 3   ? closedLoopScenariosR3()
                                 : n == 4 ? closedLoopScenariosR4()
                                 : n == 5 ? closedLoopScenariosR5()
                                 : n == 6 ? closedLoopScenariosR6()
                                          : closedLoopScenariosR7()) {
            if (s.name == name) {
                sc = s;
                return true;
            }
        }
    }
    return false;
}

SplitSpec splitSpec(const std::string& name) {
    SplitSpec s;
    s.name = name;
    if (name == "r7") {
        s.title = "held-out evaluation of the unique re-join after pose corrections (network R7)";
        s.routes = benchmarkRoutesR7();
        s.conditions = conditionNamesR7();
        s.estimators = estimatorNamesV2();
        s.scenarios = closedLoopScenariosR7();
        s.closedConditions = conditionNamesR7();
        for (const char* e : {"ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"}) {
            s.sources.push_back({std::string(e) + ", no re-join", "estimated", e, true, Rejoin::Off});
            s.sources.push_back({e, "estimated", e, true, Rejoin::Unique});
        }
        for (const char* e : {"ekf_kerb", "ekf_kerb2"}) {  // the R6 subject, for comparison
            s.sources.push_back({std::string(e) + ", R6 re-join", "estimated", e, true, Rejoin::Verified});
        }
        s.referenceFeasibility = true;
        s.referenceRejoin = Rejoin::Unique;
        return s;
    }
    if (name == "r6") {
        s.title = "held-out evaluation of the verified re-join after pose corrections (network R6)";
        s.routes = benchmarkRoutesR6();
        s.conditions = conditionNamesR6();
        s.estimators = estimatorNamesV2();
        s.scenarios = closedLoopScenariosR6();
        s.closedConditions = conditionNamesR6();
        for (const char* e : {"ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"}) {
            s.sources.push_back({std::string(e) + ", no re-join", "estimated", e, true, Rejoin::Off});
            s.sources.push_back({e, "estimated", e, true, Rejoin::Verified});
        }
        s.referenceFeasibility = true;
        return s;
    }
    if (name == "r5") {
        s.title = "held-out evaluation of the re-join after pose corrections (network R5)";
        s.routes = benchmarkRoutesR5();
        s.conditions = conditionNamesR5();
        s.estimators = estimatorNamesV2();
        s.scenarios = closedLoopScenariosR5();
        s.closedConditions = conditionNamesR5();
        for (const char* e : {"ekf", "ekf_int", "ekf_int2", "ekf_kerb", "ekf_kerb2"}) {
            s.sources.push_back({std::string(e) + ", no re-join", "estimated", e, true, Rejoin::Off});
            s.sources.push_back({e, "estimated", e, true, Rejoin::All});
        }
        s.referenceFeasibility = true;
        return s;
    }
    if (name == "r3dev" || name == "r4") {
        const bool r4 = name == "r4";
        s.title = r4 ? "held-out evaluation of the second versions (network R4)"
                     : "development and regression data for the second versions (network R3)";
        s.routes = r4 ? benchmarkRoutesR4() : benchmarkRoutesR3();
        s.conditions = r4 ? conditionNamesR4() : conditionNamesR3();
        s.estimators = estimatorNamesV2();
        s.scenarios = r4 ? closedLoopScenariosR4() : closedLoopScenariosR3();
        s.closedConditions = r4 ? conditionNamesR4()
                                : std::vector<std::string>{"nominal_r3", "gnss_outage_r3", "multipath_r3", "gnss_ramp_r3", "kerb_clutter_r3", "combined_r3"};
        s.sources = {{"ekf", "estimated", "ekf", true, Rejoin::Off},
                     {"ekf_int", "estimated", "ekf_int", true, Rejoin::Off},
                     {"ekf_int2", "estimated", "ekf_int2", true, Rejoin::Off},
                     {"ekf_kerb", "estimated", "ekf_kerb", true, Rejoin::Off},
                     {"ekf_kerb2", "estimated", "ekf_kerb2", true, Rejoin::Off}};
        s.referenceFeasibility = true;
        return s;
    }
    if (name == "dev" || name == "heldout") {
        const bool h = name == "heldout";
        s.title = h ? "held-out split (network R2)" : "development split (network R1)";
        s.routes = benchmarkRoutes(h);
        s.conditions = conditionNames(h);
        s.estimators = estimatorNames();
        s.scenarios = closedLoopScenarios(h);
        s.closedConditions = h ? std::vector<std::string>{"nominal_h", "gnss_outage_h", "multipath_h", "combined_h"}
                               : std::vector<std::string>{"nominal", "gnss_outage", "multipath"};
        s.sources = {{"EKF", "estimated", "ekf", false, Rejoin::Off}};
        s.referenceFeasibility = false;
        return s;
    }
    if (name == "r2" || name == "r3") {
        const bool r3 = name == "r3";
        s.title = r3 ? "held-out evaluation of the integrity and kerb increment (network R3)"
                     : "development and regression data for the integrity and kerb increment (network R2)";
        s.routes = r3 ? benchmarkRoutesR3() : benchmarkRoutes(true);
        s.conditions = r3 ? conditionNamesR3() : conditionNames(true);
        s.estimators = estimatorNamesExtended();
        s.scenarios = r3 ? closedLoopScenariosR3() : closedLoopScenarios(true);
        s.closedConditions = r3 ? std::vector<std::string>{"nominal_r3", "gnss_outage_r3", "multipath_r3", "gnss_ramp_r3", "kerb_clutter_r3",
                                                            "combined_r3"}
                                : std::vector<std::string>{"nominal_h", "gnss_outage_h", "multipath_h", "combined_h"};
        s.sources = {{"ekf, previous stack", "estimated", "ekf", false, Rejoin::Off},
                     {"ekf", "estimated", "ekf", true, Rejoin::Off},
                     {"ekf_int", "estimated", "ekf_int", true, Rejoin::Off},
                     {"ekf_kerb", "estimated", "ekf_kerb", true, Rejoin::Off}};
        s.referenceFeasibility = true;
        return s;
    }
    throw std::invalid_argument("unknown split " + name);
}

std::string runLocalisationCampaign(const CampaignOptions& opt) {
    namespace fs = std::filesystem;
    const SplitSpec sp = splitSpec(opt.split);
    const std::vector<RouteSpec>& routes = sp.routes;
    const std::vector<std::string>& conds = sp.conditions;
    const bool extended = sp.estimators.size() > estimatorNames().size();
    const bool v2 = std::find(sp.estimators.begin(), sp.estimators.end(), "ekf_int2") != sp.estimators.end();
    EstimatorConfig cfg;
    std::string cfgError;
    if (!applyConfigOverrides(opt.config, cfg, cfgError)) {
        throw std::runtime_error(cfgError);
    }
    fs::create_directories(fs::path(opt.outDir) / "logs");
    std::ofstream jl(fs::path(opt.outDir) / "runs.jsonl", std::ios::binary);
    struct Group {
        std::vector<LocMetrics> runs;
        std::vector<double> pooled, lateral;
    };
    std::map<std::string, std::map<std::string, Group>> groups;  // condition -> estimator -> runs
    // condition -> "a vs b" -> (a's RMSE lower, runs)
    std::map<std::string, std::vector<std::pair<std::string, std::pair<int, int>>>> wins;
    std::vector<std::string> failures;
    int drives = 0, incomplete = 0;
    for (const RouteSpec& route : routes) {
        const LaneNetwork net = buildNetwork(route.network);
        for (const std::string& cond : conds) {
            for (int seed = 1; seed <= opt.seeds; ++seed) {
                const Scenario sc = benchmarkScenario(route, cond, static_cast<std::uint64_t>(seed), "ground_truth");
                const EpisodeResult r = runEpisode(sc);
                ++drives;
                incomplete += r.outcome == "COMPLETED" ? 0 : 1;
                const std::vector<LocMetrics> ms = evaluateLocalisation(r.truth, r.sensors, r.outages, net, cfg, nullptr, &sp.estimators);
                std::map<std::string, const LocMetrics*> by;
                for (const LocMetrics& m : ms) {
                    jl << "{\"split\":\"" << sp.name << "\",\"route\":\"" << route.name << "\",\"condition\":\"" << cond << "\",\"seed\":" << seed
                       << ",\"drive_outcome\":\"" << r.outcome << "\",\"trace_hash\":\"" << hex64(r.traceHash) << "\"," << locMetricsJson(m) << "}\n";
                    Group& g = groups[cond][m.estimator];
                    g.runs.push_back(m);
                    g.pooled.insert(g.pooled.end(), m.errors.begin(), m.errors.end());
                    g.lateral.insert(g.lateral.end(), m.lateral.begin(), m.lateral.end());
                    by[m.estimator] = &m;
                    if (m.failed) {
                        failures.push_back(route.name + " " + cond + " seed " + std::to_string(seed) + ": " + m.estimator + " failed (max error " + num(m.max, 2) + " m)");
                    }
                }
                std::vector<std::pair<std::string, std::string>> pairs;
                if (v2) {
                    pairs = {{"ekf_int2", "ekf_int"}, {"ekf_kerb2", "ekf_kerb"}, {"ekf_int2", "ekf"}, {"ekf_kerb2", "ekf_int2"}};
                } else if (extended) {
                    pairs = {{"ekf_int", "ekf"}, {"ekf_kerb", "ekf"}, {"ekf_kerb", "ekf_int"}};
                } else {
                    for (const std::string& n : sp.estimators) {
                        if (n != "ekf") pairs.push_back({"ekf", n});
                    }
                }
                for (const auto& pr : pairs) {
                    const std::string key = pr.first + " vs " + pr.second;
                    auto& v = wins[cond];
                    auto it = std::find_if(v.begin(), v.end(), [&](const std::pair<std::string, std::pair<int, int>>& e) { return e.first == key; });
                    if (it == v.end()) {
                        v.push_back({key, {0, 0}});
                        it = v.end() - 1;
                    }
                    it->second.first += by[pr.first]->rmse < by[pr.second]->rmse ? 1 : 0;
                    ++it->second.second;
                }
                // Raw logs for replay without the simulator: the first route, seed 1, a few conditions (every
                // other run is reproducible from its route, condition and seed).
                bool keepLog = false;  // development splits (r2, r3dev) keep none
                if (sp.name == "r7") {
                    keepLog = cond == "kerb_return_r7" || cond == "imu_fault_r7" || cond == "kerb_faults_r7";
                } else if (sp.name == "r6") {
                    keepLog = cond == "gnss_ramp_r6" || cond == "kerb_return_r6" || cond == "imu_fault_r6";
                } else if (sp.name == "r5") {
                    keepLog = cond == "gnss_ramp_r5" || cond == "kerb_faults_r5" || cond == "combined_r5";
                } else if (sp.name == "r4") {
                    keepLog = cond == "multipath_r4" || cond == "gnss_ramp_r4" || cond == "combined_r4";
                } else if (sp.name == "r3") {
                    keepLog = cond == "multipath_r3" || cond == "gnss_ramp_r3" || cond == "kerb_clutter_r3";
                } else if (!extended) {
                    keepLog = cond.rfind("gnss_outage", 0) == 0 || cond.rfind("multipath", 0) == 0 || cond == "combined_h";
                }
                if (route.name == routes.front().name && seed == 1 && keepLog) {
                    std::string error;
                    writeEpisode(r, (fs::path(opt.outDir) / "logs" / (route.name + "_" + cond + "_s1")).string(), error);
                }
            }
        }
    }
    // Summary.
    std::ostringstream md;
    md << "# Localisation benchmark: " << sp.title << "\n\n";
    md << "Open loop: each drive is recorded once with ground-truth driving; every estimator then processes the same sensor log in the "
          "same arrival order and is scored against the same truth in the fixed map frame, with no per-trajectory alignment. Errors "
          "are rear-axle positions sampled at 50 Hz from each estimator's initialisation; lateral error is the part across the true "
          "heading. A run fails when the position error ever exceeds " << num(kFailError, 1) << " m after initialising (failed runs stay in every statistic).\n\n";
    md << "Drives: " << drives << " (" << routes.size() << " routes x " << conds.size() << " conditions x " << opt.seeds
       << " seeds); drives not completing their route: " << incomplete << ". Toolchain: " << toolchain() << ". Engine commit at configure: "
       << engineCommit() << ".\n\n";
    for (const std::string& cond : conds) {
        md << "## " << cond << "\n\n";
        md << "| Estimator | Runs | Failed | RMSE, median run (m) | RMSE, pooled (m) | p95, pooled (m) | Max (m) | Lateral p95, pooled (m) | "
              "Heading RMSE, median (deg) | Outage drift at end, median (m) | Recovery, median (s) | NEES mean (dof) | NEES in 95% | "
              "GNSS NIS mean | GNSS rejected | Jumps detected | Re-anchors | Kerb scans used | "
           << (v2 ? "Fault recoveries | Heading re-anchors | Kerb re-localisations | " : "") << "p99 latency (us) |\n";
        md << "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|" << (v2 ? "---|---|---|" : "") << "\n";
        for (const std::string& name : sp.estimators) {
            const Group& g = groups[cond][name];
            if (g.runs.empty()) continue;
            std::vector<double> rmses, heads, ends, recs, nees, neesIn, nis, lat;
            int failed = 0, rejected = 0, jumps = 0, reanch = 0, kUsed = 0, kAll = 0, recov = 0, hReanch = 0, reloc = 0;
            double mx = 0.0, s2 = 0.0;
            for (const LocMetrics& m : g.runs) {
                rmses.push_back(m.rmse);
                heads.push_back(m.headingRmseDeg);
                if (m.outageEnd >= 0.0) ends.push_back(m.outageEnd);
                if (m.recovery >= 0.0) recs.push_back(m.recovery);
                if (m.neesMean >= 0.0) {
                    nees.push_back(m.neesMean);
                    neesIn.push_back(m.neesInside);
                }
                if (m.nisGnssMean >= 0.0) nis.push_back(m.nisGnssMean);
                lat.push_back(m.latencyP99Us);
                failed += m.failed ? 1 : 0;
                rejected += m.gnssRejected;
                jumps += m.stepAlarms;
                reanch += m.reanchors;
                kUsed += m.kerbUsed;
                kAll += m.kerbUsed + m.kerbRejected;
                recov += m.faultRecoveries;
                hReanch += m.headingReanchors;
                reloc += m.kerbRelocalisations;
                mx = std::max(mx, m.max);
            }
            for (double e : g.pooled) s2 += e * e;
            auto mean = [](const std::vector<double>& v) {
                double s = 0.0;
                for (double x : v) s += x;
                return v.empty() ? -1.0 : s / static_cast<double>(v.size());
            };
            const bool integrityEst = name == "ekf_int" || name == "ekf_kerb" || name == "ekf_int2" || name == "ekf_kerb2";
            const bool kerbEst = name == "ekf_kerb" || name == "ekf_kerb2";
            const bool secondEst = name == "ekf_int2" || name == "ekf_kerb2";
            const std::string neesText = nees.empty() ? "-" : num(mean(nees), 2) + " (" + std::to_string(g.runs.front().neesDof) + ")";
            md << "| " << name << " | " << g.runs.size() << " | " << failed << " | " << num(median(rmses), 3) << " | "
               << num(std::sqrt(s2 / static_cast<double>(std::max<size_t>(1, g.pooled.size()))), 3) << " | " << num(percentile(g.pooled, 0.95), 3) << " | "
               << num(mx, 2) << " | " << num(percentile(g.lateral, 0.95), 3) << " | " << num(median(heads), 2) << " | "
               << (ends.empty() ? "-" : num(median(ends), 2)) << " | " << (recs.empty() ? "-" : num(median(recs), 1)) << " | " << neesText << " | "
               << (neesIn.empty() ? "-" : num(mean(neesIn), 2)) << " | " << (nis.empty() ? "-" : num(mean(nis), 2)) << " | " << rejected << " | "
               << (integrityEst ? std::to_string(jumps) : std::string("-")) << " | " << (integrityEst ? std::to_string(reanch) : std::string("-")) << " | "
               << (kerbEst && kAll > 0 ? num(100.0 * kUsed / kAll, 1) + "%" : std::string("-")) << " | ";
            if (v2) {
                md << (secondEst ? std::to_string(recov) : std::string("-")) << " | " << (secondEst ? std::to_string(hReanch) : std::string("-")) << " | "
                   << (name == "ekf_kerb2" ? std::to_string(reloc) : std::string("-")) << " | ";
            }
            md << num(median(lat), 1) << " |\n";
        }
        md << "\nLower RMSE, paired by drive: ";
        bool first = true;
        for (const auto& kv : wins[cond]) {
            md << (first ? "" : "; ") << kv.first << " in " << kv.second.first << " of " << kv.second.second << " runs";
            first = false;
        }
        md << ".\n\n";
    }
    md << "## Failed runs\n\n";
    if (failures.empty()) md << "None.\n";
    for (const std::string& f : failures) md << "- " << f << "\n";
    md << "\nRecovery 999 means the error did not stay below 0.5 m for 1 s before the drive ended. Latency and runtime are wall-clock "
          "times on the machine that ran the campaign and are not deterministic.\n";
    writeFile(fs::path(opt.outDir) / "summary.md", md.str());

    std::ostringstream ct;
    ct << "split=" << sp.name << "\nseeds=1.." << opt.seeds << "\nroutes=";
    for (const RouteSpec& r : routes) ct << r.name << ":" << r.network << ":" << r.startLane << "@" << r.startS << "->" << r.goalLane << "@" << r.goalS << " ";
    ct << "\nconditions=";
    for (const std::string& c : conds) ct << c << " ";
    ct << "\nestimators=";
    for (const std::string& e : sp.estimators) ct << e << " ";
    ct << "\nestimator_config=" << configText(cfg) << "\ntoolchain=" << toolchain() << "\nengine_commit_at_configure=" << engineCommit()
       << "\nconfig.gnss_sigma=" << cfg.gnssSigma << "\nconfig.wheel_sigma=" << cfg.wheelSigma << "\nconfig.gyro_sigma=" << cfg.gyroSigma
       << "\nconfig.accel_sigma=" << cfg.accelSigma << "\nconfig.pos_process=" << cfg.posProcess << "\nconfig.gyro_bias_walk=" << cfg.gyroBiasWalk
       << "\nconfig.accel_bias_walk=" << cfg.accelBiasWalk << "\nconfig.scale_walk=" << cfg.scaleWalk << "\nconfig.gate_gnss=" << cfg.gateGnss
       << "\nconfig.gate_wheel=" << cfg.gateWheel << "\nconfig.reset_after_rejected=" << cfg.resetAfterRejected << "\nconfig.cv_accel_sigma=" << cfg.cvAccelSigma
       << "\n";
    writeFile(fs::path(opt.outDir) / "campaign.txt", ct.str());
    return md.str();
}

std::string runClosedLoopCampaign(const CampaignOptions& opt) {
    namespace fs = std::filesystem;
    const SplitSpec sp = splitSpec(opt.split);
    const bool previousStack = std::any_of(sp.sources.begin(), sp.sources.end(), [](const PoseSource& p) { return !p.feasibility; });
    fs::create_directories(opt.outDir);
    std::ofstream jl(fs::path(opt.outDir) / "closed_loop.jsonl", std::ios::binary);
    struct Row {
        int runs = 0, completed = 0, sameAsRef = 0, refCompared = 0, violations = 0, assocErr = 0, assocN = 0, locStops = 0, clearStops = 0,
            unwarranted = 0;
        std::vector<double> times, xtrack, clear, estRmse, estMax, margins, crossing;
        std::map<std::string, int> outcomes;
    };
    std::map<std::string, std::map<std::string, Row>> rows;  // scenario -> pose source label -> row
    std::vector<std::string> modeOrder;
    for (const Scenario& base : sp.scenarios) {
        if (!opt.scenarios.empty() && ("," + opt.scenarios + ",").find("," + base.name + ",") == std::string::npos) {
            continue;
        }
        std::vector<std::pair<std::string, Scenario>> variants;
        Scenario gt = base;
        gt.mode = "ground_truth";
        gt.condition = sp.closedConditions.front();
        gt.feasibility = sp.referenceFeasibility;
        if (opt.rejoin.empty() || !parseRejoin(opt.rejoin, gt.rejoin)) {  // nothing jumps on simulator truth
            gt.rejoin = sp.referenceRejoin;
        }
        variants.push_back({"ground truth (reference)", gt});
        for (const std::string& c : sp.closedConditions) {
            for (const PoseSource& src : sp.sources) {
                Scenario e = base;
                e.mode = src.mode;
                e.estimator = src.estimator;
                e.feasibility = src.feasibility;
                e.rejoin = src.rejoin;
                if (!opt.rejoin.empty() && !parseRejoin(opt.rejoin, e.rejoin)) {
                    throw std::invalid_argument("unknown rejoin setting " + opt.rejoin);
                }
                e.condition = c;
                variants.push_back({src.label + ", " + c, e});
            }
        }
        std::map<int, std::string> refOutcome;
        for (auto& v : variants) {
            if (std::find(modeOrder.begin(), modeOrder.end(), v.first) == modeOrder.end()) modeOrder.push_back(v.first);
            for (int seed = 1; seed <= opt.seeds; ++seed) {
                Scenario sc = v.second;
                sc.seed = static_cast<std::uint64_t>(seed);
                const EpisodeResult r = runEpisode(sc);
                const Metrics& m = r.metrics;
                const bool isRef = sc.mode == "ground_truth";
                if (isRef) refOutcome[seed] = r.outcome;
                jl << "{\"split\":\"" << sp.name << "\",\"scenario\":\"" << base.name << "\",\"mode\":\"" << sc.mode << "\",\"estimator\":\""
                   << (isRef ? "-" : sc.estimator) << "\",\"feasibility\":" << (sc.feasibility ? "true" : "false")
                   << ",\"rejoin\":\"" << rejoinName(sc.rejoin) << "\",\"condition\":\"" << sc.condition
                   << "\",\"seed\":" << seed << ",\"outcome\":\"" << r.outcome << "\",\"stack_mode\":\"" << r.stackFinalMode
                   << "\",\"time\":" << num(r.simSeconds, 2) << ",\"stop_margin\":" << num(m.stopMargin, 3) << ",\"incursion_depth\":" << num(m.incursionDepth, 3)
                   << ",\"xtrack_link\":" << num(m.maxCrossTrackLink, 3) << ",\"xtrack_connector\":" << num(m.maxCrossTrackConnector, 3)
                   << ",\"min_clearance\":" << num(m.minClearance, 3) << ",\"clearance_violations\":" << m.clearanceViolations
                   << ",\"max_crossing\":" << num(m.maxCrossing, 3) << ",\"association_errors\":" << m.associationErrors
                   << ",\"association_samples\":" << m.associationSamples << ",\"localisation_stops\":" << m.localisationStops
                   << ",\"localisation_degraded\":" << m.localisationDegraded << ",\"clearance_stops\":" << m.clearanceStops
                   << ",\"unwarranted_stops\":" << m.unwarrantedStops << ",\"est_rmse\":" << num(m.estRmse, 4) << ",\"est_p95\":" << num(m.estP95, 4)
                   << ",\"est_max\":" << num(m.estMax, 4) << ",\"illegal\":" << m.illegalTransitions << ",\"off_lane\":" << m.offLaneSamples
                   << ",\"same_as_reference\":" << (!isRef && refOutcome.count(seed) && refOutcome[seed] == r.outcome ? "true" : "false")
                   << ",\"pose_corrections\":"
                   << std::count_if(r.events.begin(), r.events.end(), [](const LogEvent& e) { return e.type == "POSE_CORRECTION"; })
                   << ",\"trace_hash\":\"" << hex64(r.traceHash) << "\"}\n";
                Row& row = rows[base.name][v.first];
                ++row.runs;
                ++row.outcomes[r.outcome];
                row.completed += m.completed ? 1 : 0;
                if (!isRef && refOutcome.count(seed)) {
                    ++row.refCompared;
                    row.sameAsRef += refOutcome[seed] == r.outcome ? 1 : 0;
                }
                row.violations += m.clearanceViolations > 0 ? 1 : 0;
                row.crossing.push_back(m.maxCrossing);
                row.assocErr += m.associationErrors;
                row.assocN += m.associationSamples;
                row.locStops += m.localisationStops;
                row.clearStops += m.clearanceStops;
                row.unwarranted += m.unwarrantedStops;
                if (m.completed) row.times.push_back(m.timeToGoal);
                row.xtrack.push_back(std::max(m.maxCrossTrackLink, m.maxCrossTrackConnector));
                row.clear.push_back(m.minClearance);
                if (m.stopMargin >= 0.0) row.margins.push_back(m.stopMargin);
                if (m.estRmse >= 0.0) {
                    row.estRmse.push_back(m.estRmse);
                    row.estMax.push_back(m.estMax);
                }
            }
        }
    }
    std::ostringstream md;
    md << "# Closed-loop driving on estimated state: " << sp.title << "\n\n";
    md << "Each scenario is driven with the stack on simulator truth (the labelled reference) and on each estimator's output under each "
          "sensor condition, with matched seeds 1.." << opt.seeds << ". Every quantity below is measured from truth by the evaluator. Toolchain: "
       << toolchain() << ".";
    if (previousStack && sp.sources.size() > 1) {
        md << " \"previous stack\" is the stack evaluated on R2 in November 2026, without the clearance layer; every other row, and the "
              "reference, use the clearance layer.";
    }
    auto anyRejoin = [&](Rejoin r) { return std::any_of(sp.sources.begin(), sp.sources.end(), [&](const PoseSource& p) { return p.rejoin == r; }); };
    if (anyRejoin(Rejoin::Off) && anyRejoin(Rejoin::Unique) && opt.rejoin.empty()) {
        md << " \"no re-join\" is the stack evaluated on R3 and R4. \"R6 re-join\" re-joins every correction the estimator verified "
              "against the map (kerb registration), as evaluated on R6. Every other row re-joins only those whose kerb scan fits the map "
              "uniquely, as tested at the registered pose or by the re-localisation search; nothing jumps on simulator truth.";
    } else if (anyRejoin(Rejoin::Off) && anyRejoin(Rejoin::Verified) && opt.rejoin.empty()) {
        md << " \"no re-join\" is the stack evaluated on R3 and R4. Every other row re-joins only corrections the estimator verified "
              "against the map (kerb registration), by the amount the map moved it; nothing jumps on simulator truth.";
    }
    if (anyRejoin(Rejoin::Off) && anyRejoin(Rejoin::All) && opt.rejoin.empty()) {
        md << " \"no re-join\" is the stack evaluated on R3 and R4: the clearance layer without the re-join after pose corrections, its "
              "monitor measuring from the unshifted path. Every other row uses the re-join; nothing jumps on simulator truth.";
    }
    md << "\n\n";
    for (const auto& sv : rows) {
        md << "## " << sv.first << "\n\n";
        md << "| Pose source | Runs | Outcomes | Same outcome as reference | Time to goal, median (s) | Stop margin, min (m) | Max cross-track (m) | "
              "Min clearance (m) | Runs with the body over a kerb | Deepest (m) | Lane-association errors | Localisation stops | Clearance stops | "
              "Unwarranted stops | Estimate RMSE, median (m) | Estimate max (m) |\n";
        md << "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
        for (const std::string& label : modeOrder) {
            const auto it = sv.second.find(label);
            if (it == sv.second.end()) continue;
            const Row& row = it->second;
            std::string outcomes;
            for (const auto& o : row.outcomes) outcomes += (outcomes.empty() ? "" : ", ") + o.first + " " + std::to_string(o.second);
            const double xt = row.xtrack.empty() ? 0.0 : *std::max_element(row.xtrack.begin(), row.xtrack.end());
            const double cl = row.clear.empty() ? 0.0 : *std::min_element(row.clear.begin(), row.clear.end());
            const double cr = row.crossing.empty() ? 0.0 : *std::max_element(row.crossing.begin(), row.crossing.end());
            md << "| " << label << " | " << row.runs << " | " << outcomes << " | "
               << (row.refCompared ? std::to_string(row.sameAsRef) + " of " + std::to_string(row.refCompared) : std::string("-")) << " | "
               << (row.times.empty() ? "-" : num(median(row.times), 1)) << " | "
               << (row.margins.empty() ? "-" : num(*std::min_element(row.margins.begin(), row.margins.end()), 2)) << " | " << num(xt, 3) << " | "
               << num(cl, 3) << " | " << row.violations << " | " << num(cr, 3) << " | " << row.assocErr << " of " << row.assocN << " | " << row.locStops
               << " | " << row.clearStops << " | " << row.unwarranted << " | " << (row.estRmse.empty() ? "-" : num(median(row.estRmse), 3)) << " | "
               << (row.estMax.empty() ? "-" : num(*std::max_element(row.estMax.begin(), row.estMax.end()), 2)) << " |\n";
        }
        md << "\n";
    }
    md << "Clearance is the smallest distance from the footprint with mirrors (4.682 m by 2.139 m) to the kerb; negative means part of the "
          "footprint crossed it. Runs with the body over a kerb count the body without mirrors (2.011 m wide), and Deepest is how far it went. "
          "Lane-association errors count control ticks where the stack's element is not the evaluator's lane (or a movement joining it). "
          "A stop is unwarranted when the true position error at the decision was below "
       << num(kUnwarrantedError, 1) << " m.\n";
    writeFile(fs::path(opt.outDir) / "closed_loop.md", md.str());
    return md.str();
}

bool parseTruthCsv(const std::string& text, std::vector<TruthSample>& out) {
    std::istringstream in(text);
    std::string line;
    std::getline(in, line);
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        TruthSample s{};
        double* fields[9] = {&s.t, &s.x, &s.y, &s.yaw, &s.v, &s.steer, &s.accel, &s.cmdSteer, &s.cmdAccel};
        const char* p = line.c_str();
        for (double* f : fields) {
            char* end = nullptr;
            *f = std::strtod(p, &end);
            if (end == p || *end != ',') return false;
            p = end + 1;
        }
        char* end = nullptr;
        s.closure = static_cast<int>(std::strtol(p, &end, 10));
        if (end == p) return false;
        out.push_back(s);
    }
    return true;
}

std::vector<std::pair<double, double>> parseWindows(const std::string& s) {
    std::vector<std::pair<double, double>> w;
    std::istringstream in(s);
    std::string item;
    while (std::getline(in, item, ',')) {
        const size_t c = item.find(':');
        if (c != std::string::npos) w.push_back({std::stod(item.substr(0, c)), std::stod(item.substr(c + 1))});
    }
    return w;
}

}  // namespace road
