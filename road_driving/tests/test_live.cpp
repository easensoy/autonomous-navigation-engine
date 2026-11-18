// Live operation: the line-protocol server and recorded sessions (src/live.cpp). No graphics needed.
#include "road_driving/benchmark.hpp"
#include "road_driving/episode.hpp"
#include "road_driving/live.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
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

struct Run {
    std::string out, diag;
    std::string hash;  // from the "end" message
    int ends = 0;
};

std::string field(const std::string& line, const std::string& key) {
    const std::string k = "\"" + key + "\":\"";
    const size_t a = line.find(k);
    if (a == std::string::npos) return "";
    const size_t b = line.find('"', a + k.size());
    return line.substr(a + k.size(), b - a - k.size());
}

Run live(const std::string& config, const std::string& script, int observeEvery = 1, double speed = 0.0, const std::string& sessionOut = "") {
    LiveOptions opt;
    opt.speed = speed;
    opt.observeEvery = observeEvery;
    opt.script = true;
    opt.initialConfig = config;
    opt.sessionOut = sessionOut;
    std::istringstream in(script);
    std::ostringstream out, diag;
    runLiveServer(in, out, diag, opt);
    Run r;
    r.out = out.str();
    r.diag = diag.str();
    std::istringstream lines(r.out);
    std::string line;
    while (std::getline(lines, line)) {
        if (line.find("\"type\":\"end\"") != std::string::npos) {
            r.hash = field(line, "trace_hash");
            ++r.ends;
        }
    }
    return r;
}

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream s;
    s << f.rdbuf();
    return s.str();
}

int count(const std::string& text, const std::string& needle) {
    int n = 0;
    for (size_t p = text.find(needle); p != std::string::npos; p = text.find(needle, p + needle.size())) ++n;
    return n;
}

const char* kDemoC = R"({"scenario":"R7-A","condition":"kerb_faults_r7","estimator":"ekf_kerb2","seed":7,"rejoin":"unique"})";
const char* kNominal = R"({"scenario":"R7-A","condition":"nominal_r7","estimator":"ekf_kerb2","seed":3,"rejoin":"unique"})";
const char* kWorldScript =
    R"({"cmd":"gnss_outage","on":true,"at_tick":1500}
{"cmd":"gnss_outage","on":false,"at_tick":2500}
{"cmd":"kerb_blackout","on":true,"at_tick":3000}
{"cmd":"kerb_blackout","on":false,"at_tick":3600}
{"cmd":"gnss_bias","on":true,"metres":4,"bearing_deg":90,"at_tick":4000}
{"cmd":"gnss_bias","on":false,"at_tick":5000}
{"cmd":"closure","action":"ahead","distance":40,"at_tick":5200}
{"cmd":"closure","action":"lift","at_tick":6500}
)";

Scenario scenario(const char* config) {
    Scenario sc;
    std::string error;
    if (!scenarioFromConfig(config, sc, error)) std::printf("config error: %s\n", error.c_str());
    return sc;
}

// A hooked run that only observes is the headless run.
void testObservationChangesNothing() {
    for (const char* config : {kDemoC, kNominal, R"({"scenario":"R3-B","condition":"gnss_ramp_r3","estimator":"ekf_kerb2","seed":5})",
                               R"({"scenario":"R7-B_late_closure","condition":"nominal_r7","mode":"ground_truth","seed":2})"}) {
        const Run r = live(config, "");
        const EpisodeResult h = runEpisode(scenario(config));
        check(r.ends == 1 && r.hash == hex64(h.traceHash), std::string("live run without commands equals the headless run: ") + config,
              r.hash + " vs " + hex64(h.traceHash));
    }
}

// Commands at recorded ticks replay headlessly to the same trace; observation rate and pacing change nothing.
void testSessionsReplay() {
    const std::string path = "test_live_session.json";
    const Run a = live(kNominal, kWorldScript, 1, 0.0, path);
    const std::string session = readFile(path);
    SessionReplay rep;
    std::string error;
    const bool ok = replaySession(session, rep, error);
    check(ok && rep.commands == 8, "the session records every world command with its tick", ok ? std::to_string(rep.commands) : error);
    check(ok && hex64(rep.result.traceHash) == a.hash && rep.recordedHash == a.hash, "headless replay of the session reproduces the live trace",
          (ok ? hex64(rep.result.traceHash) : error) + " vs " + a.hash);
    check(count(a.out, "\"type\":\"applied\"") == 8 && count(a.out, "\"type\":\"error\"") == 0, "all eight commands applied, none refused");
    check(count(a.out, "CLOSURE_NOTIFIED") == 1 && count(a.out, "CLOSURE_LIFTED") == 1, "the operator's closure is notified and lifted");
    const Run b = live(kNominal, kWorldScript, 7, 0.0);
    check(b.hash == a.hash, "observing every 7th tick instead of every tick changes nothing", b.hash);
    const Run c = live(kNominal, kWorldScript, 3, 40.0);
    check(c.hash == a.hash, "real-time pacing (40x) changes nothing", c.hash);
    const std::string paused = std::string(R"({"cmd":"pause"})") + "\n" + R"({"cmd":"step","ticks":25})" + "\n" + R"({"cmd":"resume"})" + "\n" + kWorldScript;
    const Run d = live(kNominal, paused, 1, 0.0);
    check(d.hash == a.hash, "pausing, single-stepping and resuming change nothing", d.hash);
    // Without commands the same configuration drives differently: the commands did act.
    const Run e = live(kNominal, "");
    check(e.hash != a.hash, "the world commands change the episode", e.hash);

    // Ticks arrive in order, one message per tick, every line versioned.
    std::istringstream lines(a.out);
    std::string line;
    long last = -1;
    bool ordered = true, versioned = true;
    while (std::getline(lines, line)) {
        versioned = versioned && line.rfind("{\"v\":1,\"type\":\"", 0) == 0;
        if (line.find("\"type\":\"tick\"") != std::string::npos) {
            const long tick = std::stol(line.substr(line.find("\"tick\":") + 7));
            ordered = ordered && tick == last + 1;
            last = tick;
        }
    }
    check(versioned, "every message is one JSON object with a version and a type");
    check(ordered && last > 5000, "tick messages carry consecutive tick IDs", std::to_string(last));
}

// The injected faults do what they say, and draw no extra random numbers.
void testFaults() {
    std::string error;
    const std::string base = std::string(R"({"format":"ane-session","version":1,"config":)") + kNominal + R"(,"commands":[)";
    SessionReplay none, faults, bias;
    replaySession(base + "]}", none, error);
    replaySession(base +
                      R"({"tick":1500,"cmd":"gnss_outage","on":true},{"tick":2500,"cmd":"gnss_outage","on":false},)"
                      R"({"tick":3000,"cmd":"kerb_blackout","on":true},{"tick":3600,"cmd":"kerb_blackout","on":false}]})",
                  faults, error);
    replaySession(base + R"({"tick":2000,"cmd":"gnss_bias","dx":3.25,"dy":-1.5}]})", bias, error);
    int gnssIn = 0, gnssAround = 0, kerbIn = 0, kerbAround = 0;
    for (const Measurement& m : faults.result.sensors) {
        if (m.kind == Measurement::Gnss) {
            (m.tMeas >= 15.0 - 1e-9 && m.tMeas < 25.0 - 1e-9 ? gnssIn : gnssAround)++;
        } else if (m.kind == Measurement::Kerb) {
            (m.tMeas >= 30.0 - 1e-9 && m.tMeas < 36.0 - 1e-9 ? kerbIn : kerbAround)++;
        }
    }
    check(gnssIn == 0 && gnssAround > 300, "an injected GNSS outage leaves no fix inside it", std::to_string(gnssIn) + " inside, " + std::to_string(gnssAround) + " outside");
    check(kerbIn == 0 && kerbAround > 300, "an injected kerb blackout leaves no scan inside it", std::to_string(kerbIn) + " inside, " + std::to_string(kerbAround) + " outside");
    // Before the outage both runs are identical, measurement for measurement.
    size_t same = 0;
    while (same < none.result.sensors.size() && same < faults.result.sensors.size() && none.result.sensors[same].tArrival < 15.0) {
        const Measurement& a = none.result.sensors[same];
        const Measurement& b = faults.result.sensors[same];
        if (a.kind != b.kind || a.a != b.a || a.b != b.b || a.c != b.c || a.ranges != b.ranges) break;
        ++same;
    }
    check(same > 1000 && none.result.sensors[same].tArrival >= 15.0, "up to the injected outage the sensors are unchanged", std::to_string(same));
    // The first fix valid at or after the offset's tick is the clean fix moved by exactly the offset.
    const Measurement* clean = nullptr;
    const Measurement* moved = nullptr;
    for (const Measurement& m : none.result.sensors) {
        if (m.kind == Measurement::Gnss && m.tMeas >= 20.0 - 1e-9) {
            clean = &m;
            break;
        }
    }
    for (const Measurement& m : bias.result.sensors) {
        if (m.kind == Measurement::Gnss && m.tMeas >= 20.0 - 1e-9) {
            moved = &m;
            break;
        }
    }
    const bool shifted = clean && moved && clean->tMeas == moved->tMeas && std::fabs(moved->a - clean->a - 3.25) < 1e-5 && std::fabs(moved->b - clean->b + 1.5) < 1e-5;
    check(shifted, "an injected GNSS offset moves the fix by exactly the offset, with the same noise",
          clean && moved ? std::to_string(moved->a - clean->a) + ", " + std::to_string(moved->b - clean->b) : "no fix");
}

// A reset starts the new episode from tick 0 with nothing carried over.
void testReset() {
    const std::string script = std::string(R"({"cmd":"reset","config":)") + kDemoC + "}\n";
    const Run r = live(kNominal, script);
    const EpisodeResult h = runEpisode(scenario(kDemoC));
    check(r.ends == 1 && r.hash == hex64(h.traceHash), "a reset at the start runs the new configuration as a fresh episode", r.hash);
    check(count(r.out, "\"type\":\"session\"") == 2, "each episode announces its session");
}

// Closing the input ends an interactive server promptly.
void testEndOfInput() {
    LiveOptions opt;
    opt.speed = 1.0;
    opt.initialConfig = kNominal;
    std::istringstream in("");
    std::ostringstream out, diag;
    const auto t0 = std::chrono::steady_clock::now();
    const int code = runLiveServer(in, out, diag, opt);
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    check(code == 0 && s < 5.0 && out.str().find("\"type\":\"bye\"") != std::string::npos, "the server stops when its input closes", std::to_string(s) + " s");
}

// A replay refuses new world commands.
void testReplayRefusesCommands() {
    const std::string path = "test_live_replay.json";
    live(kNominal, kWorldScript, 50, 0.0, path);
    LiveOptions opt;
    opt.speed = 0.0;
    opt.script = true;
    opt.replayPath = path;
    std::istringstream in(R"({"cmd":"gnss_outage","on":true})");
    std::ostringstream out, diag;
    runLiveServer(in, out, diag, opt);
    check(out.str().find("\"mode\":\"replay\"") != std::string::npos && out.str().find("replay: world commands come from the session file") != std::string::npos,
          "a replay is marked as one and refuses world commands");
    check(out.str().find("\"hash_equal\":true") != std::string::npos, "the replay reports that it reproduced the recorded trace");
}

}  // namespace

int main() {
    testObservationChangesNothing();
    testSessionsReplay();
    testFaults();
    testReset();
    testEndOfInput();
    testReplayRefusesCommands();
    std::printf("live tests: %d passed, %d failed\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
