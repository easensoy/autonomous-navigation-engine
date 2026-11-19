// The San Francisco network (SF-SOMA, right-hand traffic), the vehicle profiles and the San Francisco conditions:
// lane side and turn decisions, the recorded rules, swept clearance for both vehicles, the local frame, the routes,
// the profiles reaching every consumer (sensor mounts, lever arms, manifests, live messages and sessions), and the
// map error that the vehicle's map has and the world does not. No graphics needed.
#include "road_driving/benchmark.hpp"
#include "road_driving/district.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/live.hpp"
#include "road_driving/vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace road;

namespace {

int g_pass = 0, g_fail = 0;

void check(bool ok, const std::string& what, const std::string& detail = "") {
    (ok ? g_pass : g_fail)++;
    std::printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", what.c_str(), detail.empty() ? "" : ": ", detail.c_str());
}

std::string num(double v, int prec = 3) {
    char b[64];
    std::snprintf(b, sizeof b, "%.*f", prec, v);
    return b;
}

Scenario sfScenario(const std::string& route, const std::string& condition, std::uint64_t seed, const std::string& mode, const std::string& estimator,
                    const std::string& vehicle) {
    Scenario sc;
    findNamedScenario(route, sc);
    sc.condition = condition;
    sc.mode = mode;
    sc.seed = seed;
    sc.departDelay = 2.0;
    sc.estimator = estimator;
    sc.vehicle = vehicle;
    return sc;
}

int connectorOf(const LaneNetwork& n, const std::string& a, const std::string& b) {
    const int la = n.lane(a), lb = n.lane(b);
    return la >= 0 && lb >= 0 ? n.connector(la, lb) : -1;
}

void trafficSide() {
    const LaneNetwork sf = buildNetwork("SF-SOMA");
    const std::vector<std::string> errs = sf.validate();
    check(sf.general && sf.spec.rightHand && errs.empty(), "SF-SOMA builds as a right-hand-traffic district network and validates",
          errs.empty() ? "" : errs.front());
    bool right = true;
    for (const Lane& l : sf.lanes) right = right && l.dirX * (l.y0 - l.centreY0) - l.dirY * (l.x0 - l.centreX0) < 0.0;
    check(right, "SF-SOMA: every lane keeps right of its street's centreline");
    const std::vector<std::string> merr = mirrored(sf).validate();
    check(!merr.empty() && merr.front().find("right-hand") != std::string::npos, "a mirrored copy of SF-SOMA (lanes on the left) is rejected",
          merr.empty() ? "accepted" : merr.front());
    const LaneNetwork ken = buildNetwork("KEN");
    check(!ken.spec.rightHand && ken.validate().empty(), "the London districts keep left-hand traffic");
    NetworkSpec rhs;
    rhs.rightHand = true;
    bool threw = false;
    try {
        buildGrid(gridR1(), rhs);
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "the grids refuse right-hand traffic");
    check(sf.spec.penaltyLeft > sf.spec.penaltyRight && ken.spec.penaltyRight > ken.spec.penaltyLeft,
          "the turn across the opposing lane costs the larger penalty: left in San Francisco, right in London",
          "SF left " + num(sf.spec.penaltyLeft, 1) + " s, right " + num(sf.spec.penaltyRight, 1) + " s");
    // At 5th and Townsend, from Townsend south-west-bound: into 5th north-west-bound is a right turn, towards King
    // Street a left turn; the right turn keeps to the near side on a tighter arc than the left turn.
    const int right5 = connectorOf(sf, "T4>T5", "T5>X5"), left5 = connectorOf(sf, "T4>T5", "T5>S5");
    check(right5 >= 0 && left5 >= 0 && sf.elements[static_cast<size_t>(right5)].turn == Turn::Right &&
              sf.elements[static_cast<size_t>(left5)].turn == Turn::Left,
          "T5: from Townsend south-west-bound, 5th Street north-west-bound is a right turn and south-east-bound a left turn");
    auto maxCurv = [&](int e) {
        double k = 0.0;
        for (const PathPoint& p : sf.elements[static_cast<size_t>(e)].pts) k = std::max(k, std::fabs(p.kappa));
        return k;
    };
    check(right5 >= 0 && left5 >= 0 && maxCurv(right5) > maxCurv(left5), "T5: the near-side right turn runs on a tighter arc than the left turn",
          "radii " + num(1.0 / maxCurv(right5), 1) + " and " + num(1.0 / maxCurv(left5), 1) + " m");
    // The turns on the routes, and each lane's position relative to its own direction of travel.
    int uturns = 0;
    for (const Element& e : sf.elements) {
        if (e.isConnector() && std::fabs(wrapAngle(e.pts.back().yaw - e.pts.front().yaw)) > 0.75 * kPi) ++uturns;
    }
    check(uturns == 0, "SF-SOMA builds no U-turn");
}

void rules() {
    const LaneNetwork sf = buildNetwork("SF-SOMA");
    check(sf.lane("B4>T4") >= 0 && sf.lane("T4>B4") < 0 && sf.lane("N4>B4") >= 0 && sf.lane("B4>N4") < 0 && sf.lane("T4>S4") >= 0 && sf.lane("S4>T4") < 0,
          "4th Street is one-way south-east, as OpenStreetMap records it");
    check(connectorOf(sf, "B4>T4", "T4>TNE") < 0 && connectorOf(sf, "N4>B4", "B4>BNE") < 0,
          "no left turn from 4th Street onto Townsend or Brannan north-east-bound (OSM relations 10673966, 10673575)");
    check(connectorOf(sf, "B4>T4", "T4>T5") >= 0 && connectorOf(sf, "N4>B4", "B4>B5") >= 0 && connectorOf(sf, "B4>T4", "T4>S4") >= 0,
          "the right turns and the ahead movement from 4th Street exist");
    check(sf.banned.size() == 2, "SF-SOMA lists its two banned turns");
    bool median = false;
    for (const Street& s : sf.streets) median = median || (s.name == "4th Street" && std::fabs(s.median - 7.2) < 1e-9);
    check(median, "4th Street has the light-rail trackway as a 7.2 m central reservation");
    // The local frame: 5th Street between Townsend and Brannan is 192.390 m between OSM nodes 65325061 and 65315481
    // (WGS84 geodesic); the junctions sit on those nodes, rounded to 1 cm.
    const int t5 = sf.junction("T5"), b5 = sf.junction("B5");
    const double d = std::hypot(sf.junctions[static_cast<size_t>(t5)].x - sf.junctions[static_cast<size_t>(b5)].x,
                                sf.junctions[static_cast<size_t>(t5)].y - sf.junctions[static_cast<size_t>(b5)].y);
    check(std::fabs(d - 192.390) < 0.02, "the local frame reproduces 5th Street's geodesic length between Townsend and Brannan", num(d) + " m");
}

void geometry() {
    const LaneNetwork net = buildNetwork("SF-SOMA");
    for (const std::string& id : vehicleProfileNames()) {
        const VehicleParams vp = vehicleProfile(id);
        double worst = 1e9;
        std::string where;
        for (const Element& e : net.elements) {
            for (size_t i = 0; i < e.pts.size(); i += 3) {
                const PathPoint& p = e.pts[i];
                const double c = footprintClearance(net, p.x, p.y, p.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors);
                if (c < worst) {
                    worst = c;
                    where = e.name;
                }
            }
        }
        check(worst > 0.10, "SF-SOMA: the " + id + " footprint, driven exactly along every movement and lane, clears the kerb by more than 0.10 m",
              "smallest " + num(worst) + " m at " + where);
    }
    const auto loops = carriagewayLoops(net, 0.005);
    double x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
    for (const auto& lp : loops) {
        for (const auto& p : lp) {
            x0 = std::min(x0, p.first);
            y0 = std::min(y0, p.second);
            x1 = std::max(x1, p.first);
            y1 = std::max(y1, p.second);
        }
    }
    int samples = 0, disagree = 0;
    for (double x = x0 - 5.0; x <= x1 + 5.0; x += 1.7) {
        for (double y = y0 - 5.0; y <= y1 + 5.0; y += 1.7) {
            const double dd = net.roadClearance(x, y);
            if (std::fabs(dd) < 0.01) continue;
            ++samples;
            disagree += (dd > 0.0) != insideCarriageway(loops, x, y) ? 1 : 0;
        }
    }
    check(samples > 1000 && disagree == 0, "SF-SOMA: the kerb distance's sign matches a point-in-polygon test",
          std::to_string(disagree) + " of " + std::to_string(samples) + " points disagree");
    int edges = 0, seen = 0;
    for (const KerbLine& k : net.kerbLines) {
        if (!k.boundary) continue;
        ++edges;
        const double sx = 0.5 * (k.ax + k.bx) + 6.0 * k.nx, sy = 0.5 * (k.ay + k.by) + 6.0 * k.ny;
        const double r = net.rayToKerb(sx, sy, -k.nx, -k.ny, 25.0);
        seen += (r > 0.0 && std::fabs(r - 6.0) < 0.5) ? 1 : 0;
    }
    check(edges == 10 && seen == 0, "SF-SOMA: its 10 patch edges give the scanner no return");
}

void routes() {
    const LaneNetwork net = buildNetwork("SF-SOMA");
    const char* expect[][3] = {{"SF-A", "T5:T4>X5", "B5:X5>B4"}, {"SF-B", "B5:B4>X5", "T5:X5>T4"}};
    const Turn turn[] = {Turn::Right, Turn::Left};
    int k = 0;
    for (const RouteSpec& rs : benchmarkRoutesSF()) {
        const RefRoute rr = referenceRoute(net, net.segmentAt(net.lane(rs.startLane), rs.startS), net.segmentAt(net.lane(rs.goalLane), rs.goalS));
        double length = 0.0;
        std::string moves;
        bool turnsOk = true;
        for (int n : rr.nodes) {
            const Element& e = net.elements[static_cast<size_t>(n)];
            length += e.length;
            if (e.isConnector()) {
                moves += (moves.empty() ? "" : ", ") + e.name + " " + turnName(e.turn);
                turnsOk = turnsOk && (e.turn == Turn::Ahead || e.turn == turn[k]);
            }
        }
        const std::string text = routeText(net, rr.nodes);
        check(rr.found && routeLegal(net, rr.nodes) && rs.name == expect[k][0] && text.find(expect[k][1]) != std::string::npos &&
                  text.find(expect[k][2]) != std::string::npos && turnsOk && length > 500.0 && length < 900.0,
              rs.name + ": reachable, legal, " + (k == 0 ? "right" : "left") + " turns only, 500 to 900 m", num(length, 0) + " m; " + moves);
        ++k;
    }
}

void profiles() {
    VehicleParams def, ip;
    check(findVehicleProfile("ipace_proxy_v1", ip) && vehicleProfileText(ip) == vehicleProfileText(def) && std::string(ip.id) == def.id,
          "ipace_proxy_v1 is the default VehicleParams, field for field");
    const VehicleParams cy = vehicleProfile("cybercab_proxy_v1");
    check(std::fabs(cy.wheelbase + cy.frontOverhang + cy.rearOverhang - cy.length) < 1e-9 && std::fabs(cy.kerbScannerX - cy.frontOffset()) < 1e-9 &&
              cy.widthMirrors >= cy.width && cy.maxSteer == ip.maxSteer && cy.maxSteerRate == ip.maxSteerRate && cy.commandDelay == ip.commandDelay,
          "cybercab_proxy_v1 is coherent (length, scanner at the bumper) and keeps the I-PACE proxy's actuator limits",
          vehicleProfileText(cy));
    bool threw = false;
    try {
        vehicleProfile("model_y");
    } catch (const std::exception&) {
        threw = true;
    }
    check(threw, "an unknown vehicle profile is refused");
    const SensorConditions a = sensorConditionsFor("nominal_sf", ip), b = sensorConditionsFor("nominal_sf", cy);
    const SensorConditions plain = sensorConditions("nominal_sf");
    check(a.gnss.antennaX == plain.gnss.antennaX && a.imu.leverX == plain.imu.leverX && a.kerb.mountX == plain.kerb.mountX,
          "the I-PACE proxy's sensor mounts are the sensor defaults");
    check(b.gnss.antennaX == 1.30 && b.imu.leverX == 1.10 && b.kerb.mountX == 3.55 && estimatorConfigFor(cy).antennaX == 1.30 &&
              estimatorConfigFor(cy).imuLeverX == 1.10,
          "the Cybercab proxy's mounts reach the sensors and the estimators' lever arms");
}

void propagation() {
    // Same route, condition and seed; only the vehicle differs. At rest during the departure delay both cars stand at
    // the same pose and draw the same noise, so every fix differs by exactly the antenna lever arms' difference.
    const EpisodeResult ri = runEpisode(sfScenario("SF-A", "nominal_sf", 3, "ground_truth", "ekf", "ipace_proxy_v1"));
    const EpisodeResult rc = runEpisode(sfScenario("SF-A", "nominal_sf", 3, "ground_truth", "ekf", "cybercab_proxy_v1"));
    const double yaw = ri.truth.front().yaw;
    const double ex = std::cos(yaw) * (1.30 - 1.60), ey = std::sin(yaw) * (1.30 - 1.60);
    std::map<double, std::pair<double, double>> fixI;
    for (const Measurement& m : ri.sensors) {
        if (m.kind == Measurement::Gnss && m.tMeas < 1.9) fixI[m.tMeas] = {m.a, m.b};
    }
    int compared = 0;
    double worst = 0.0;
    for (const Measurement& m : rc.sensors) {
        if (m.kind != Measurement::Gnss || m.tMeas >= 1.9 || !fixI.count(m.tMeas)) continue;
        worst = std::max(worst, std::hypot(m.a - fixI[m.tMeas].first - ex, m.b - fixI[m.tMeas].second - ey));
        ++compared;
    }
    check(compared >= 15 && worst < 2e-6, "the GNSS fixes come from the profile's antenna: 1.30 m against 1.60 m ahead of the rear axle",
          std::to_string(compared) + " fixes, largest residual " + num(worst * 1e6, 2) + " um");
    bool scanner = false;
    for (const Measurement& m : rc.sensors) {
        if (m.kind == Measurement::Kerb) {
            scanner = std::fabs(m.c - 3.55) < 1e-9;
            break;
        }
    }
    check(scanner, "the kerb scans come from the profile's scanner, 3.55 m ahead of the rear axle");
    // Manifests carry the profile; an older manifest without one is the I-PACE proxy; an unknown one is refused.
    std::map<std::string, std::string> kv;
    std::istringstream in(manifestText(rc));
    for (std::string line; std::getline(in, line);) {
        const size_t eq = line.find('=');
        if (eq != std::string::npos) kv[line.substr(0, eq)] = line.substr(eq + 1);
    }
    Scenario back;
    std::string error;
    check(kv["vehicle_profile"] == "cybercab_proxy_v1" && kv["vehicle_params"].find("wheelbase=2.750") != std::string::npos &&
              scenarioFromManifest(kv, back, error) && back.vehicle == "cybercab_proxy_v1",
          "the manifest records the profile and its parameters, and a replay reads them back");
    const EpisodeResult again = runEpisode(back);
    check(again.traceHash == rc.traceHash, "the episode replays from its manifest to the same trace", hex64(rc.traceHash));
    kv.erase("vehicle_profile");
    Scenario old;
    check(scenarioFromManifest(kv, old, error) && old.vehicle == "ipace_proxy_v1", "a manifest written before the profiles replays with the I-PACE proxy");
    kv["vehicle_profile"] = "model_y";
    check(!scenarioFromManifest(kv, old, error), "a manifest naming an unknown profile is refused", error);
}

struct LiveRun {
    std::string out, hash;
    int ends = 0;
};

LiveRun live(const std::string& config, const std::string& script, const std::string& sessionOut = "") {
    LiveOptions opt;
    opt.speed = 0.0;
    opt.script = true;
    opt.initialConfig = config;
    opt.sessionOut = sessionOut;
    std::istringstream in(script);
    std::ostringstream out, diag;
    runLiveServer(in, out, diag, opt);
    LiveRun r;
    r.out = out.str();
    std::istringstream lines(r.out);
    for (std::string line; std::getline(lines, line);) {
        if (line.find("\"type\":\"end\"") == std::string::npos) continue;
        const std::string k = "\"trace_hash\":\"";
        const size_t a = line.find(k);
        r.hash = a == std::string::npos ? "" : line.substr(a + k.size(), 18);
        ++r.ends;
    }
    return r;
}

void liveAndSessions() {
    // Map to live to replay, San Francisco: a live run without commands is the headless run; a session with operator
    // commands replays headlessly to the live trace and keeps its vehicle.
    const char* sfConfig = R"({"scenario":"SF-A","condition":"kerb_map_error_sf","estimator":"ekf_kerb2","seed":904,"rejoin":"unique","vehicle":"cybercab_proxy_v1"})";
    Scenario sc;
    std::string error;
    check(scenarioFromConfig(sfConfig, sc, error) && sc.vehicle == "cybercab_proxy_v1" && sc.network == "SF-SOMA", "a live configuration names its vehicle",
          error);
    const LiveRun a = live(sfConfig, "");
    const EpisodeResult h = runEpisode(sc);
    check(a.ends == 1 && a.hash == hex64(h.traceHash), "SF live run without commands equals the headless run", a.hash + " vs " + hex64(h.traceHash));
    check(a.out.find("\"vehicle\":{\"id\":\"cybercab_proxy_v1\"") != std::string::npos && a.out.find("\"mount_x\":3.550") != std::string::npos &&
              a.out.find("\"antenna_x\":1.300") != std::string::npos && a.out.find("\"traffic\":\"right\"") != std::string::npos,
          "the scene message carries the profile, its sensor mounts and the traffic side");
    check(a.out.find("\"map_errors\":[{\"line\"") != std::string::npos && a.out.find("\"offset\":0.600") != std::string::npos,
          "the scene message carries the map error: the misplaced kerb as the vehicle's map has it and where the true kerb is");
    const std::string path = "test_sf_session.json";
    const LiveRun b = live(R"({"scenario":"SF-B","condition":"nominal_sf","estimator":"ekf_kerb2","seed":905,"vehicle":"cybercab_proxy_v1"})",
                           R"({"cmd":"gnss_outage","on":true,"at_tick":1500}
{"cmd":"gnss_outage","on":false,"at_tick":3000}
{"cmd":"kerb_blackout","on":true,"at_tick":3200}
{"cmd":"kerb_blackout","on":false,"at_tick":3800}
)",
                           path);
    std::ostringstream text;
    {
        std::ifstream f(path, std::ios::binary);  // closed before the file is removed (Windows keeps an open file)
        text << f.rdbuf();
    }
    SessionReplay rep;
    check(replaySession(text.str(), rep, error) && rep.commands == 4 && hex64(rep.result.traceHash) == b.hash &&
              rep.result.scenario.vehicle == "cybercab_proxy_v1",
          "a Cybercab session with four operator commands replays headlessly to the live trace", b.hash);
    std::remove(path.c_str());
    // London still runs live: a smoke check on KEN-A.
    const char* kenConfig = R"({"scenario":"KEN-A","condition":"gnss_outage_ldn","estimator":"ekf_kerb2","seed":1,"rejoin":"unique"})";
    Scenario ks;
    scenarioFromConfig(kenConfig, ks, error);
    const LiveRun k = live(kenConfig, "");
    check(k.ends == 1 && k.hash == hex64(runEpisode(ks).traceHash) && ks.vehicle == "ipace_proxy_v1",
          "London smoke: KEN-A live equals headless, with the I-PACE proxy by default", k.hash);
}

void conditions() {
    // The outage: no fix is measured inside it, and it covers the first street's last 180 m.
    const EpisodeResult o = runEpisode(sfScenario("SF-A", "gnss_outage_sf", 7, "ground_truth", "ekf", "cybercab_proxy_v1"));
    bool none = !o.outages.empty();
    for (const Measurement& m : o.sensors) {
        for (const auto& w : o.outages) none = none && !(m.kind == Measurement::Gnss && m.tMeas > w.first + 1e-6 && m.tMeas < w.second - 1e-6);
    }
    const double dur = o.outages.empty() ? 0.0 : o.outages.front().second - o.outages.front().first;
    check(none && o.outages.size() == 1 && dur > 15.0 && dur < 35.0, "gnss_outage_sf: one outage, no fix inside it",
          o.outages.empty() ? "none" : num(o.outages.front().first, 2) + " to " + num(o.outages.front().second, 2) + " s");
    // The drift: the fixes' offset from the true antenna grows without a jump and is held at 4 m.
    const EpisodeResult d = runEpisode(sfScenario("SF-A", "gnss_drift_sf", 7, "ground_truth", "ekf", "cybercab_proxy_v1"));
    double peak = 0.0, steepest = 0.0, prev = -1.0;
    for (float f : d.gnssFaultTrace) {
        peak = std::max(peak, static_cast<double>(f));
        if (prev >= 0.0) steepest = std::max(steepest, std::fabs(f - prev));
        prev = f;
    }
    int held = 0;
    for (float f : d.gnssFaultTrace) held += f >= 3.999f ? 1 : 0;
    check(std::fabs(peak - 4.0) < 1e-6 && steepest < 0.01 && held > 50, "gnss_drift_sf: the offset grows at 0.3 m/s with no step and is held at 4 m",
          std::to_string(held) + " samples at 4 m; largest step " + num(steepest, 4) + " m per 20 ms");
    // The map error changes no measurement: the world keeps the true kerb.
    const EpisodeResult m = runEpisode(sfScenario("SF-A", "kerb_map_error_sf", 7, "ground_truth", "ekf", "cybercab_proxy_v1"));
    check(m.sensorsCsv == o.sensorsCsv && !m.zones.mapErrors.empty(), "kerb_map_error_sf changes no measurement against gnss_outage_sf: the scanner sees the true kerb");
    const LaneNetwork net = buildNetwork("SF-SOMA");
    const LaneNetwork map = withMapErrors(net, m.zones.mapErrors);
    bool shifted = m.zones.mapErrors.size() == 2;
    for (const MapKerbError& e : m.zones.mapErrors) {
        double ax, ay, bx, by;
        mapErrorSegment(net, e, ax, ay, bx, by);
        const KerbLine& k = net.kerbLines[static_cast<size_t>(e.line)];
        const double off = (ax - k.ax) * k.nx + (ay - k.ay) * k.ny;
        shifted = shifted && std::fabs(std::fabs(off) - 0.6) < 1e-9 && std::fabs(std::fabs(e.offset) - 0.6) < 1e-12 && e.t1 - e.t0 > 0.3;
    }
    bool closes = true;
    for (const auto& lp : carriagewayLoops(map)) closes = closes && lp.size() >= 3;
    check(shifted && closes && map.kerbLines.size() == net.kerbLines.size() + 8,
          "the vehicle's map has both kerbs of a 110 m stretch 0.6 m to the side, and its kerb still closes");
    // The same estimated drive twice gives the same trace.
    const Scenario e = sfScenario("SF-A", "gnss_outage_sf", 7, "estimated", "ekf_kerb2", "cybercab_proxy_v1");
    const EpisodeResult e1 = runEpisode(e), e2 = runEpisode(e);
    check(e1.traceHash == e2.traceHash && e1.outcome == "COMPLETED", "SF-A on ekf_kerb2 through the outage completes, and deterministically",
          e1.outcome + ", " + hex64(e1.traceHash));
    std::printf("INFO SF-A gnss_outage_sf seed 7 ekf_kerb2: RMSE %.3f m, max %.3f m, smallest clearance %.3f m\n", e1.metrics.estRmse, e1.metrics.estMax,
                e1.metrics.minClearance);
}

}  // namespace

int main() {
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    trafficSide();
    rules();
    geometry();
    routes();
    profiles();
    propagation();
    liveAndSessions();
    conditions();
    std::printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 && g_pass > 0 ? 0 : 1;
}
