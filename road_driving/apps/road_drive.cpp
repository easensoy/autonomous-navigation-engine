// road_drive: run, list and replay closed-loop episodes on the synthetic networks R1 to R7, and the
// localisation benchmark.
//
//   road_drive list
//   road_drive run <scenario> [--variant B0|B1] [--seed N] [--out DIR]
//   road_drive replay <episode dir>
//   road_drive suite [--variant B0|B1] [--seeds N]
//   road_drive drive <scenario|route> --condition C [--mode ground_truth|estimated] [--estimator NAME] [--feasibility on|off]
//                    [--rejoin off|on|verified|unique] [--seed N] [--out DIR]
//   road_drive localise <episode dir> [--estimators a,b] [--config k=v,...]   (stored sensor log; no world simulation)
//   road_drive bench-loc --split dev|heldout|r2|r3|r3dev|r4|r5|r6|r7 [--seeds N] --out DIR
//   road_drive bench-closed --split dev|heldout|r2|r3|r3dev|r4|r5|r6|r7 [--seeds N] [--scenario A,B] [--rejoin off|on|verified|unique] --out DIR
//   road_drive map R1|R2|R3|R4|R5|R6|R7 [--json FILE]
//
// Every episode uses synthetic roads and an oracle closure report. Pose is simulator truth unless the
// mode is "estimated", when the stack drives on the estimator's output.
#include "road_driving/benchmark.hpp"
#include "road_driving/episode.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <map>
#include <sstream>
#include <string>

using namespace road;

namespace {

void printSummary(const EpisodeResult& r) {
    const Metrics& m = r.metrics;
    std::printf("%-11s %-2s seed=%-3llu %-18s mode=%-19s t=%6.2f s  stop_class=%-11s margin=%6.2f m  incursion=%5.2f m  "
                "replans=%d masks=%d fallbacks=%d  xtrack link/conn=%.3f/%.3f m  max_decel=%.2f  max_lat=%.2f  illegal=%d off_lane=%d "
                "over_limit=%.2f  trace=%s\n",
                r.scenario.name.c_str(), variantName(r.scenario.variant), static_cast<unsigned long long>(r.scenario.seed), r.outcome.c_str(),
                r.stackFinalMode.c_str(), r.simSeconds, m.stopClass.empty() ? "-" : m.stopClass.c_str(), m.stopMargin, m.incursionDepth, m.replans,
                m.masks, m.fallbacks, m.maxCrossTrackLink, m.maxCrossTrackConnector, m.maxDecel, m.maxLatAccel, m.illegalTransitions,
                m.offLaneSamples, m.maxSpeedExcess, hex64(r.traceHash).c_str());
}

int usage() {
    std::fprintf(stderr,
                 "usage:\n  road_drive list\n  road_drive run <scenario> [--variant B0|B1] [--seed N] [--out DIR]\n"
                 "  road_drive replay <episode dir>\n  road_drive suite [--variant B0|B1] [--seeds N]\n"
                 "  road_drive drive <scenario|route> [--condition C] [--mode ground_truth|estimated] [--estimator NAME]\n"
                 "                   [--feasibility on|off] [--rejoin off|on|verified|unique] [--seed N] [--out DIR]\n"
                 "  road_drive localise <episode dir> [--estimators a,b] [--config k=v,...]\n"
                 "  road_drive bench-loc --split dev|heldout|r2|r3|r3dev|r4|r5|r6|r7 [--seeds N] --out DIR\n"
                 "  road_drive bench-closed --split dev|heldout|r2|r3|r3dev|r4|r5|r6|r7 [--seeds N] [--scenario A,B] [--rejoin off|on|verified|unique] --out DIR\n"
                 "  road_drive map R1|R2|R3|R4|R5|R6|R7 [--json FILE]\n");
    return 2;
}

int firstDifferentLine(const std::string& a, const std::string& b) {
    std::istringstream ia(a), ib(b);
    std::string la, lb;
    int n = 1;
    while (true) {
        const bool ga = static_cast<bool>(std::getline(ia, la));
        const bool gb = static_cast<bool>(std::getline(ib, lb));
        for (std::string* line : {&la, &lb}) {
            if (!line->empty() && line->back() == '\r') {
                line->pop_back();  // a CRLF checkout of the stored log is the same log
            }
        }
        if (!ga && !gb) {
            return 0;
        }
        if (ga != gb || la != lb) {
            return n;
        }
        ++n;
    }
}

int replay(const std::string& dir) {
    std::map<std::string, std::string> kv;
    std::string error;
    Scenario sc;
    if (!readKeyValues(dir + "/manifest.txt", kv, error) || !scenarioFromManifest(kv, sc, error)) {
        std::fprintf(stderr, "replay: %s\n", error.c_str());
        return 2;
    }
    const EpisodeResult r = runEpisode(sc);
    const bool sameToolchain = kv["toolchain"] == toolchain();
    const bool outcomeOk = kv["outcome"] == r.outcome;
    const bool routesOk = kv["route_hash"] == hex64(r.routeHash);
    const bool traceOk = kv["trace_hash"] == hex64(r.traceHash);
    std::printf("replay of %s (%s, %s, seed %llu)\n", dir.c_str(), sc.name.c_str(), variantName(sc.variant), static_cast<unsigned long long>(sc.seed));
    std::printf("  recorded with %s; replayed with %s\n", kv["toolchain"].c_str(), toolchain().c_str());
    std::printf("  outcome    recorded %-20s replayed %-20s %s\n", kv["outcome"].c_str(), r.outcome.c_str(), outcomeOk ? "MATCH" : "DIFFERENT");
    std::printf("  routes     recorded %-20s replayed %-20s %s\n", kv["route_hash"].c_str(), hex64(r.routeHash).c_str(), routesOk ? "MATCH" : "DIFFERENT");
    std::printf("  trace      recorded %-20s replayed %-20s %s\n", kv["trace_hash"].c_str(), hex64(r.traceHash).c_str(), traceOk ? "MATCH" : "DIFFERENT");
    std::string stored;
    if (!traceOk && readText(dir + "/truth.csv", stored)) {
        std::printf("  first differing truth.csv line: %d\n", firstDifferentLine(stored, r.truthCsv));
    }
    if (sameToolchain) {
        return (outcomeOk && routesOk && traceOk) ? 0 : 1;
    }
    // Bit-identical floating point is not expected across compilers: decisions must still agree.
    std::printf("  different toolchain: the trace hash is reported, not required; outcome and routes must match\n");
    for (const char* key : {"stop_margin", "incursion_depth", "max_decel", "max_cross_track_link", "max_cross_track_connector"}) {
        const auto it = kv.find(key);
        if (it == kv.end()) {
            continue;
        }
        double now = 0.0;
        const Metrics& m = r.metrics;
        if (std::strcmp(key, "stop_margin") == 0) now = m.stopMargin;
        if (std::strcmp(key, "incursion_depth") == 0) now = m.incursionDepth;
        if (std::strcmp(key, "max_decel") == 0) now = m.maxDecel;
        if (std::strcmp(key, "max_cross_track_link") == 0) now = m.maxCrossTrackLink;
        if (std::strcmp(key, "max_cross_track_connector") == 0) now = m.maxCrossTrackConnector;
        std::printf("  %-26s recorded %s replayed %.3f\n", key, it->second.c_str(), now);
    }
    return (outcomeOk && routesOk) ? 0 : 1;
}

bool scenarioByName(const std::string& name, Scenario& sc) {
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

int drive(const std::string& name, std::map<std::string, std::string>& opt) {
    Scenario sc;
    if (!scenarioByName(name, sc)) {
        std::fprintf(stderr, "unknown scenario or route '%s'\n", name.c_str());
        return 2;
    }
    sc.condition = opt.count("--condition") ? opt["--condition"] : "nominal";
    sc.mode = opt.count("--mode") ? opt["--mode"] : "estimated";
    sc.seed = opt.count("--seed") ? std::stoull(opt["--seed"]) : 1;
    sc.departDelay = opt.count("--delay") ? std::stod(opt["--delay"]) : 2.0;
    sc.estimator = opt.count("--estimator") ? opt["--estimator"] : "ekf";
    sc.feasibility = !opt.count("--feasibility") || opt["--feasibility"] != "off";
    if (opt.count("--rejoin") && !parseRejoin(opt["--rejoin"], sc.rejoin)) return usage();
    if (opt.count("--variant") && !parseVariant(opt["--variant"], sc.variant)) return usage();
    const EpisodeResult r = runEpisode(sc);
    printSummary(r);
    const Metrics& m = r.metrics;
    std::printf("  mode=%s estimator=%s feasibility=%s rejoin=%s condition=%s: min clearance %.3f m, body over a kerb in %d samples (deepest %.3f m), "
                "lane-association errors %d of %d, localisation stops %d, clearance stops %d (unwarranted %d), estimate RMSE %.3f m (p95 %.3f, "
                "max %.3f), GNSS outages %zu\n",
                sc.mode.c_str(), sc.estimator.c_str(), sc.feasibility ? "on" : "off", rejoinName(sc.rejoin), sc.condition.c_str(), m.minClearance,
                m.clearanceViolations,
                m.maxCrossing, m.associationErrors, m.associationSamples, m.localisationStops, m.clearanceStops, m.unwarrantedStops, m.estRmse, m.estP95,
                m.estMax, r.outages.size());
    if (opt.count("--out")) {
        std::string error;
        if (!writeEpisode(r, opt["--out"], error)) {
            std::fprintf(stderr, "%s\n", error.c_str());
            return 1;
        }
        std::printf("wrote %s\n", opt["--out"].c_str());
    }
    return 0;
}

int localise(const std::string& dir, std::map<std::string, std::string>& opt) {
    std::map<std::string, std::string> kv;
    std::string error, truthText, sensorText;
    if (!readKeyValues(dir + "/manifest.txt", kv, error) || !readText(dir + "/truth.csv", truthText) || !readText(dir + "/sensors.csv", sensorText)) {
        std::fprintf(stderr, "localise: needs manifest.txt, truth.csv and sensors.csv in %s\n", dir.c_str());
        return 2;
    }
    std::vector<TruthSample> truth;
    std::vector<Measurement> sensors;
    if (!parseTruthCsv(truthText, truth) || !parseSensorLog(sensorText, sensors, error)) {
        std::fprintf(stderr, "localise: %s\n", error.empty() ? "bad truth.csv" : error.c_str());
        return 2;
    }
    const LaneNetwork net = buildNetwork(kv.count("network_name") ? kv["network_name"] : "R1");
    std::vector<std::vector<StateEstimate>> traces;
    // Development options: --estimators a,b,c and --config key=value,... (defaults: all estimators, the frozen configuration).
    std::vector<std::string> names = estimatorNamesExtended();
    if (opt.count("--estimators")) {
        names.clear();
        std::istringstream in(opt["--estimators"]);
        for (std::string n; std::getline(in, n, ',');) names.push_back(n);
    }
    EstimatorConfig cfg;
    if (opt.count("--config") && !applyConfigOverrides(opt["--config"], cfg, error)) {
        std::fprintf(stderr, "localise: %s\n", error.c_str());
        return 2;
    }
    const auto ms = evaluateLocalisation(truth, sensors, parseWindows(kv["outage_windows"]), net, cfg, &traces, &names);
    // Every estimator's trace, for plots and the demonstration (offline products, not inputs).
    for (size_t k = 0; k < ms.size() && k < traces.size(); ++k) {
        std::ofstream tr(dir + "/trace_" + ms[k].estimator + ".csv", std::ios::binary);
        tr << "t,valid,x,y,yaw,v,pxx,pxy,pyy,sigma_pos,status,pyawyaw,pxyaw,pyyaw,gnss,kerb\n";
        char line[320];
        for (const StateEstimate& e : traces[k]) {
            std::snprintf(line, sizeof line, "%.2f,%d,%.4f,%.4f,%.5f,%.4f,%.6g,%.6g,%.6g,%.4f,%s,%.6g,%.6g,%.6g,%s,%s\n", e.t, e.valid ? 1 : 0, e.x, e.y,
                          e.yaw, e.v, e.pxx, e.pxy, e.pyy, e.hasCov ? e.sigmaPos() : -1.0, e.status, e.pyawyaw, e.pxyaw, e.pyyaw, e.gnss, e.kerb);
            tr << line;
        }
    }
    std::printf("%s: %zu truth samples, %zu measurements (condition %s), replayed without the world simulator\n", dir.c_str(), truth.size(),
                sensors.size(), kv["condition"].c_str());
    std::printf("%-10s %6s %8s %8s %8s %8s %9s %10s %9s %8s %8s\n", "estimator", "failed", "rmse", "p95", "max", "head_deg", "rpe20", "outage_end",
                "recovery", "nees", "rej_gnss");
    std::ofstream out(dir + "/localisation.jsonl", std::ios::binary);
    for (const LocMetrics& m : ms) {
        std::printf("%-10s %6s %8.3f %8.3f %8.3f %8.2f %9.3f %10.3f %9.1f %8.2f %8d\n", m.estimator.c_str(), m.failed ? "yes" : "no", m.rmse, m.p95,
                    m.max, m.headingRmseDeg, m.rpe20, m.outageEnd, m.recovery, m.neesMean, m.gnssRejected);
        out << "{" << locMetricsJson(m) << "}\n";
    }
    return 0;
}

int mapCmd(const std::string& name, std::map<std::string, std::string>& opt) {
    const LaneNetwork net = buildNetwork(name);
    const VehicleParams vp;
    const auto errs = net.validate();
    int segments = 0, connectors = 0;
    double minClear = 1e9;
    std::string worst;
    std::map<std::string, std::pair<double, std::string>> byKind;  // worst clearance per kind of movement
    for (const Element& e : net.elements) {
        (e.isConnector() ? connectors : segments) += 1;
        if (!e.isConnector()) continue;
        const std::string kind = std::string(turnName(e.turn)) + (e.speedLimit < net.spec.speedLimit ? " (tight)" : "");
        for (size_t i = 0; i < e.pts.size(); i += 5) {
            const PathPoint& p = e.pts[i];
            const double c = footprintClearance(net, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors);
            if (c < minClear) {
                minClear = c;
                worst = e.name + " (" + turnName(e.turn) + ")";
            }
            const auto it = byKind.find(kind);
            if (it == byKind.end() || c < it->second.first) byKind[kind] = {c, e.name};
        }
    }
    std::printf("network %s: %zu junctions, %zu lanes, %d lane segments, %d turning movements, %zu banned turns; validation %s\n", name.c_str(),
                net.junctions.size(), net.lanes.size(), segments, connectors, net.banned.size(), errs.empty() ? "passes" : errs.front().c_str());
    std::printf("footprint with mirrors driven exactly along every movement: smallest clearance to a carriageway edge %.3f m at %s\n", minClear,
                worst.c_str());
    for (const auto& k : byKind) {
        std::printf("  %-14s smallest clearance %7.3f m (%s)\n", k.first.c_str(), k.second.first, k.second.second.c_str());
    }
    // The benchmark routes on this network as the independent reference planner finds them: movements only.
    std::vector<RouteSpec> routes = benchmarkRoutes(false);
    for (const auto& more : {benchmarkRoutes(true), benchmarkRoutesR3(), benchmarkRoutesR4(), benchmarkRoutesR5(), benchmarkRoutesR6(), benchmarkRoutesR7()}) {
        routes.insert(routes.end(), more.begin(), more.end());
    }
    for (const RouteSpec& rs : routes) {
        if (rs.network != name) continue;
        const RefRoute rr = referenceRoute(net, net.segmentAt(net.lane(rs.startLane), rs.startS), net.segmentAt(net.lane(rs.goalLane), rs.goalS));
        std::string moves;
        double length = 0.0;
        for (int n : rr.nodes) {
            const Element& e = net.elements[static_cast<size_t>(n)];
            length += e.length;
            if (e.isConnector()) moves += (moves.empty() ? "" : ", ") + e.name + " " + turnName(e.turn) + (e.speedLimit < net.spec.speedLimit ? " (tight)" : "");
        }
        std::printf("route %s: %s, %.0f m; %s\n", rs.name.c_str(), rr.found ? "reachable" : "UNREACHABLE", length, moves.c_str());
    }
    if (opt.count("--json")) {
        std::ofstream o(opt["--json"], std::ios::binary);
        o << "{\"name\":\"" << name << "\",\"lane_width\":" << net.spec.laneWidth << ",\"junctions\":[";
        for (size_t j = 0; j < net.junctions.size(); ++j) {
            o << (j ? "," : "") << "{\"name\":\"" << net.junctions[j].name << "\",\"x\":" << net.junctions[j].x << ",\"y\":" << net.junctions[j].y
              << ",\"kerb\":" << net.kerbRadius[j] << "}";
        }
        o << "],\"streets\":[";
        for (size_t i = 0; i < net.streets.size(); ++i) {
            const Street& s = net.streets[i];
            o << (i ? "," : "") << "[" << s.a << "," << s.b << "," << s.ax << "," << s.ay << "," << s.bx << "," << s.by << "]";
        }
        o << "],\"lanes\":[";
        for (size_t i = 0; i < net.lanes.size(); ++i) {
            const Lane& l = net.lanes[i];
            o << (i ? "," : "") << "{\"name\":\"" << l.name << "\",\"x0\":" << l.x0 << ",\"y0\":" << l.y0 << ",\"x1\":" << l.x1 << ",\"y1\":" << l.y1 << "}";
        }
        o << "],\"connectors\":[";
        bool first = true;
        for (const Element& e : net.elements) {
            if (!e.isConnector()) continue;
            o << (first ? "" : ",") << "{\"name\":\"" << e.name << "\",\"turn\":\"" << turnName(e.turn) << "\",\"pts\":[";
            first = false;
            for (size_t i = 0; i < e.pts.size(); i += 5) {
                o << (i ? "," : "") << "[" << e.pts[i].x << "," << e.pts[i].y << "]";
            }
            o << ",[" << e.pts.back().x << "," << e.pts.back().y << "]]}";
        }
        o << "],\"banned\":[";
        for (size_t i = 0; i < net.banned.size(); ++i) {
            o << (i ? "," : "") << "[\"" << net.banned[i].junction << "\",\"" << net.banned[i].fromLane << "\",\"" << net.banned[i].toLane << "\"]";
        }
        o << "]}\n";
        std::printf("wrote %s\n", opt["--json"].c_str());
    }
    return errs.empty() ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        return usage();
    }
    const std::string cmd = argv[1];
    std::map<std::string, std::string> opt;
    std::string positional;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--", 0) == 0 && i + 1 < argc) {
            opt[a] = argv[++i];
        } else {
            positional = a;
        }
    }
    try {
        if (cmd == "list") {
            for (const Scenario& s : catalogue()) {
                std::printf("%-11s %s reveal=%s distance=%g lift_after_stop=%g\n", s.name.c_str(), s.scenarioClass.c_str(), revealName(s.reveal),
                            s.revealDistance, s.liftAfterStop);
            }
            return 0;
        }
        if (cmd == "run") {
            Scenario sc;
            if (!findScenario(positional, sc)) {
                std::fprintf(stderr, "unknown scenario '%s' (see road_drive list)\n", positional.c_str());
                return 2;
            }
            if (opt.count("--variant") && !parseVariant(opt["--variant"], sc.variant)) {
                return usage();
            }
            if (opt.count("--seed")) {
                sc.seed = std::stoull(opt["--seed"]);
            }
            const EpisodeResult r = runEpisode(sc);
            printSummary(r);
            if (opt.count("--out")) {
                std::string error;
                if (!writeEpisode(r, opt["--out"], error)) {
                    std::fprintf(stderr, "%s\n", error.c_str());
                    return 1;
                }
                std::printf("wrote %s\n", opt["--out"].c_str());
            }
            return 0;
        }
        if (cmd == "replay") {
            if (positional.empty()) {
                return usage();
            }
            return replay(positional);
        }
        if (cmd == "drive" && !positional.empty()) {
            return drive(positional, opt);
        }
        if (cmd == "localise" && !positional.empty()) {
            return localise(positional, opt);
        }
        if (cmd == "map" && !positional.empty()) {
            return mapCmd(positional, opt);
        }
        if ((cmd == "bench-loc" || cmd == "bench-closed") && opt.count("--out")) {
            CampaignOptions co;
            co.split = opt.count("--split") ? opt["--split"] : "dev";
            co.seeds = opt.count("--seeds") ? std::stoi(opt["--seeds"]) : (co.split == "dev" ? 5 : 10);
            co.outDir = opt["--out"];
            co.config = opt.count("--config") ? opt["--config"] : "";
            co.scenarios = opt.count("--scenario") ? opt["--scenario"] : "";
            co.rejoin = opt.count("--rejoin") ? opt["--rejoin"] : "";
            Rejoin check;
            if (!co.rejoin.empty() && !parseRejoin(co.rejoin, check)) {
                return usage();
            }
            std::fputs((cmd == "bench-loc" ? runLocalisationCampaign(co) : runClosedLoopCampaign(co)).c_str(), stdout);
            return 0;
        }
        if (cmd == "suite") {
            const int seeds = opt.count("--seeds") ? std::stoi(opt["--seeds"]) : 1;
            for (const Scenario& base : catalogue()) {
                for (int seed = 0; seed < seeds; ++seed) {
                    Scenario sc = base;
                    if (opt.count("--variant") && !parseVariant(opt["--variant"], sc.variant)) {
                        return usage();
                    }
                    sc.seed = static_cast<unsigned long long>(seed);
                    printSummary(runEpisode(sc));
                }
            }
            return 0;
        }
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "error: %s\n", ex.what());
        return 1;
    }
    return usage();
}
