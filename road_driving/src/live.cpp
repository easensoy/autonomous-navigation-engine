// Live operation: the line-protocol server behind the 3D viewer, and session replay.
// Protocol: viewer/PROTOCOL.md. Every message is one JSON object on one line, with "v" and "type".
#include "road_driving/live.hpp"

#include "road_driving/district.hpp"

#include "road_driving/benchmark.hpp"
#include "road_driving/localisation.hpp"
#include "live_hooks.hpp"
#include "stack.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <istream>
#include <mutex>
#include <ostream>
#include <sstream>
#include <thread>

namespace road {

namespace {

constexpr int kProtocolVersion = 1;

// ---- A minimal JSON reader (objects, arrays, strings, numbers, true, false, null) ---------------------
struct JVal {
    enum Kind { Null, Bool, Num, Str, Arr, Obj };
    Kind kind = Null;
    bool b = false;
    double n = 0.0;
    std::string s;
    std::vector<JVal> a;
    std::vector<std::pair<std::string, JVal>> o;
    const JVal* get(const char* key) const {
        if (kind != Obj) return nullptr;
        for (const auto& kv : o) {
            if (kv.first == key) return &kv.second;
        }
        return nullptr;
    }
    std::string str(const char* key, const std::string& d = "") const {
        const JVal* v = get(key);
        return v && v->kind == Str ? v->s : d;
    }
    double num(const char* key, double d) const {
        const JVal* v = get(key);
        return v && v->kind == Num ? v->n : d;
    }
    bool flag(const char* key, bool d) const {
        const JVal* v = get(key);
        return v && v->kind == Bool ? v->b : d;
    }
};

struct JParser {
    const std::string& t;
    size_t i = 0;
    std::string error;
    explicit JParser(const std::string& text) : t(text) {}
    bool fail(const std::string& why) {
        if (error.empty()) error = why + " at offset " + std::to_string(i);
        return false;
    }
    void ws() {
        while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\n' || t[i] == '\r')) ++i;
    }
    bool string(std::string& out) {
        ++i;  // opening quote
        while (i < t.size()) {
            const char c = t[i++];
            if (c == '"') return true;
            if (c != '\\') {
                out += c;
                continue;
            }
            if (i >= t.size()) break;
            const char e = t[i++];
            switch (e) {
                case 'n': out += '\n'; break;
                case 't': out += '\t'; break;
                case 'r': out += '\r'; break;
                case 'b': out += '\b'; break;
                case 'f': out += '\f'; break;
                case 'u': {
                    if (i + 4 > t.size()) return fail("bad escape");
                    const unsigned code = static_cast<unsigned>(std::strtoul(t.substr(i, 4).c_str(), nullptr, 16));
                    i += 4;
                    if (code < 0x80) {
                        out += static_cast<char>(code);
                    } else if (code < 0x800) {
                        out += static_cast<char>(0xC0 | (code >> 6));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    } else {
                        out += static_cast<char>(0xE0 | (code >> 12));
                        out += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
                        out += static_cast<char>(0x80 | (code & 0x3F));
                    }
                    break;
                }
                default: out += e; break;
            }
        }
        return fail("unterminated string");
    }
    bool value(JVal& v, int depth = 0) {
        if (depth > 32) return fail("too deep");
        ws();
        if (i >= t.size()) return fail("unexpected end");
        const char c = t[i];
        if (c == '{') {
            ++i;
            v.kind = JVal::Obj;
            ws();
            if (i < t.size() && t[i] == '}') {
                ++i;
                return true;
            }
            for (;;) {
                ws();
                if (i >= t.size() || t[i] != '"') return fail("expected a key");
                std::string key;
                if (!string(key)) return false;
                ws();
                if (i >= t.size() || t[i] != ':') return fail("expected ':'");
                ++i;
                JVal item;
                if (!value(item, depth + 1)) return false;
                v.o.emplace_back(key, std::move(item));
                ws();
                if (i < t.size() && t[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < t.size() && t[i] == '}') {
                    ++i;
                    return true;
                }
                return fail("expected ',' or '}'");
            }
        }
        if (c == '[') {
            ++i;
            v.kind = JVal::Arr;
            ws();
            if (i < t.size() && t[i] == ']') {
                ++i;
                return true;
            }
            for (;;) {
                JVal item;
                if (!value(item, depth + 1)) return false;
                v.a.push_back(std::move(item));
                ws();
                if (i < t.size() && t[i] == ',') {
                    ++i;
                    continue;
                }
                if (i < t.size() && t[i] == ']') {
                    ++i;
                    return true;
                }
                return fail("expected ',' or ']'");
            }
        }
        if (c == '"') {
            v.kind = JVal::Str;
            return string(v.s);
        }
        if (t.compare(i, 4, "true") == 0) {
            i += 4;
            v.kind = JVal::Bool;
            v.b = true;
            return true;
        }
        if (t.compare(i, 5, "false") == 0) {
            i += 5;
            v.kind = JVal::Bool;
            return true;
        }
        if (t.compare(i, 4, "null") == 0) {
            i += 4;
            return true;
        }
        const char* begin = t.c_str() + i;
        char* end = nullptr;
        const double d = std::strtod(begin, &end);
        if (end == begin) return fail("unexpected character");
        i += static_cast<size_t>(end - begin);
        v.kind = JVal::Num;
        v.n = d;
        return true;
    }
};

bool parseJson(const std::string& text, JVal& out, std::string& error) {
    JParser p(text);
    if (!p.value(out)) {
        error = p.error;
        return false;
    }
    p.ws();
    if (p.i != text.size()) {
        error = "trailing characters at offset " + std::to_string(p.i);
        return false;
    }
    return true;
}

// ---- Writing ------------------------------------------------------------------------------------------
std::string jstr(const std::string& s) {
    std::string out = "\"";
    for (const char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", static_cast<unsigned>(static_cast<unsigned char>(c)));
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out + "\"";
}

// Fixed decimals for display values; "null" for a value that is not finite.
std::string jnum(double v, int decimals) {
    if (!std::isfinite(v)) return "null";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

// Round-trip precision, for values a replay must reproduce exactly.
std::string jexact(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

std::string jg(double v) {
    if (!std::isfinite(v)) return "null";
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.6g", v);
    return buf;
}

std::string dump(const JVal& v) {
    switch (v.kind) {
        case JVal::Null: return "null";
        case JVal::Bool: return v.b ? "true" : "false";
        case JVal::Num: return jexact(v.n);
        case JVal::Str: return jstr(v.s);
        case JVal::Arr: {
            std::string s = "[";
            for (size_t i = 0; i < v.a.size(); ++i) s += (i ? "," : "") + dump(v.a[i]);
            return s + "]";
        }
        case JVal::Obj: {
            std::string s = "{";
            for (size_t i = 0; i < v.o.size(); ++i) s += (i ? "," : "") + jstr(v.o[i].first) + ":" + dump(v.o[i].second);
            return s + "}";
        }
    }
    return "null";
}

std::string fixed3(double v) { return jnum(v, 3); }

// ---- Configuration ------------------------------------------------------------------------------------
// The defaults are those of `road_drive drive`, so a configuration naming only a route and a condition
// gives the episode a drive command gives.
bool scenarioFromJVal(const JVal& c, Scenario& sc, std::string& error) {
    if (c.kind != JVal::Obj) {
        error = "configuration is not an object";
        return false;
    }
    const std::string name = c.str("scenario", "R7-A");
    if (!findNamedScenario(name, sc)) {
        error = "unknown scenario or route '" + name + "'";
        return false;
    }
    sc.condition = c.str("condition", "nominal");
    sc.mode = c.str("mode", "estimated");
    sc.seed = static_cast<std::uint64_t>(std::llround(c.num("seed", 1.0)));
    sc.departDelay = c.num("delay", 2.0);
    sc.estimator = c.str("estimator", "ekf");
    sc.feasibility = c.flag("feasibility", true);
    if (c.get("rejoin") && !parseRejoin(c.str("rejoin"), sc.rejoin)) {
        error = "unknown rejoin setting '" + c.str("rejoin") + "'";
        return false;
    }
    if (c.get("variant") && !parseVariant(c.str("variant"), sc.variant)) {
        error = "unknown variant '" + c.str("variant") + "'";
        return false;
    }
    sc.vehicle = c.str("vehicle", "ipace_proxy_v1");  // sessions recorded before profiles existed: the I-PACE proxy
    VehicleParams profile;
    if (!findVehicleProfile(sc.vehicle, profile)) {
        error = "unknown vehicle profile '" + sc.vehicle + "'";
        return false;
    }
    if (sc.mode != "estimated" && sc.mode != "ground_truth") {
        error = "unknown mode '" + sc.mode + "'";
        return false;
    }
    try {
        sensorConditions(sc.condition);
    } catch (const std::exception& e) {
        error = e.what();
        return false;
    }
    if (sc.mode == "estimated") {
        const LaneNetwork probe = buildNetwork(sc.network);
        if (!makeEstimator(sc.estimator, EstimatorConfig{}, probe)) {
            error = "unknown estimator '" + sc.estimator + "'";
            return false;
        }
    }
    return true;
}

// The configuration as it is recorded: every field explicit.
std::string configJson(const Scenario& sc, const std::string& name) {
    return "{\"scenario\":" + jstr(name) + ",\"network\":" + jstr(sc.network) + ",\"condition\":" + jstr(sc.condition) +
           ",\"mode\":" + jstr(sc.mode) + ",\"estimator\":" + jstr(sc.estimator) + ",\"seed\":" + std::to_string(sc.seed) +
           ",\"rejoin\":" + jstr(rejoinName(sc.rejoin)) + ",\"variant\":" + jstr(variantName(sc.variant)) +
           ",\"delay\":" + jexact(sc.departDelay) + ",\"feasibility\":" + (sc.feasibility ? "true" : "false") + ",\"vehicle\":" + jstr(sc.vehicle) + "}";
}

// ---- World commands -----------------------------------------------------------------------------------
// A world command in its resolved form: what a session records and a replay applies.
struct WorldCommand {
    long tick = 0;
    std::string cmd;  // gnss_outage, kerb_blackout, gnss_bias, closure_add, closure_lift
    bool on = false;
    double dx = 0.0, dy = 0.0;
    std::string lane;
    double barrierS = 0.0, works = 15.0;
};

std::string commandJson(const WorldCommand& c) {
    std::string s = "{\"tick\":" + std::to_string(c.tick) + ",\"cmd\":" + jstr(c.cmd);
    if (c.cmd == "gnss_outage" || c.cmd == "kerb_blackout") {
        s += std::string(",\"on\":") + (c.on ? "true" : "false");
    } else if (c.cmd == "gnss_bias") {
        s += ",\"dx\":" + jexact(c.dx) + ",\"dy\":" + jexact(c.dy);
    } else if (c.cmd == "closure_add") {
        s += ",\"lane\":" + jstr(c.lane) + ",\"barrier_s\":" + jexact(c.barrierS) + ",\"works\":" + jexact(c.works);
    }
    return s + "}";
}

bool commandFromJVal(const JVal& v, WorldCommand& c, std::string& error) {
    c.tick = std::lround(v.num("tick", -1.0));
    c.cmd = v.str("cmd");
    c.on = v.flag("on", false);
    c.dx = v.num("dx", 0.0);
    c.dy = v.num("dy", 0.0);
    c.lane = v.str("lane");
    c.barrierS = v.num("barrier_s", 0.0);
    c.works = v.num("works", 15.0);
    if (c.tick < 0) {
        error = "command without a tick";
        return false;
    }
    if (c.cmd != "gnss_outage" && c.cmd != "kerb_blackout" && c.cmd != "gnss_bias" && c.cmd != "closure_add" && c.cmd != "closure_lift") {
        error = "unknown world command '" + c.cmd + "'";
        return false;
    }
    return true;
}

// Applies a resolved world command to this tick's control. Returns false (with a reason) if it cannot apply.
bool applyCommand(const WorldCommand& c, const TickView& v, TickControl& ctl, std::string& why) {
    if (c.cmd == "gnss_outage") {
        ctl.faults.gnssOutage = c.on;
    } else if (c.cmd == "kerb_blackout") {
        ctl.faults.kerbBlackout = c.on;
    } else if (c.cmd == "gnss_bias") {
        ctl.faults.gnssBiasX = c.dx;
        ctl.faults.gnssBiasY = c.dy;
    } else if (c.cmd == "closure_add") {
        if (v.closure->on) {
            why = "this episode already has a closure";
            return false;
        }
        const int lane = v.net->lane(c.lane);
        if (lane < 0) {
            why = "unknown lane " + c.lane;
            return false;
        }
        ctl.addClosure = true;
        ctl.closureLane = lane;
        ctl.barrierLaneS = c.barrierS;
        ctl.worksLength = c.works;
    } else if (c.cmd == "closure_lift") {
        if (!v.closure->on || !v.closure->present) {
            why = "no closure stands";
            return false;
        }
        ctl.liftClosure = true;
    }
    return true;
}

// A closure on the route ahead: the first lane position at least `distance` beyond the front bumper with
// room for the barrier and the works, on the stack's current route.
bool closureAhead(const TickView& v, double distance, double works, WorldCommand& c, std::string& why) {
    if (!v.stackStarted || v.stack->path().empty()) {
        why = "the stack has no route yet";
        return false;
    }
    const DrivePath& p = v.stack->path();
    for (double s = v.stack->pathS() + v.vehicle->frontOffset() + std::max(10.0, distance); s < p.length(); s += 1.0) {
        const size_t i = p.elementIndexAt(s);
        const Element& e = v.net->elements[static_cast<size_t>(p.elements()[i])];
        if (e.isConnector()) continue;
        const double laneS = e.laneS0 + (s - p.elementStart(i));
        const Lane& lane = v.net->lanes[static_cast<size_t>(e.lane)];
        if (laneS < 5.0 || laneS + works + 5.0 > lane.length) continue;
        c.cmd = "closure_add";
        c.lane = lane.name;
        c.barrierS = laneS;
        c.works = works;
        return true;
    }
    why = "no lane ahead on the route has room for a closure";
    return false;
}

std::string sessionJson(const std::string& config, const std::vector<WorldCommand>& cmds, const EpisodeResult* r) {
    std::string s = "{\"format\":\"ane-session\",\"version\":1,\"backend\":{\"commit\":" + jstr(engineCommit()) +
                    ",\"toolchain\":" + jstr(toolchain()) + "},\"config\":" + config + ",\"commands\":[";
    for (size_t i = 0; i < cmds.size(); ++i) s += (i ? ",\n  " : "\n  ") + commandJson(cmds[i]);
    s += cmds.empty() ? "]" : "\n]";
    if (r) {
        s += ",\"result\":{\"t\":" + jnum(r->simSeconds, 2) + ",\"outcome\":" + jstr(r->outcome) + ",\"stack_mode\":" + jstr(r->stackFinalMode) +
             ",\"trace_hash\":" + jstr(hex64(r->traceHash)) + ",\"toolchain\":" + jstr(toolchain()) +
             ",\"max_crossing\":" + jnum(r->metrics.maxCrossing, 3) + ",\"min_clearance\":" + jnum(r->metrics.minClearance, 3) + "}";
    } else {
        s += ",\"result\":null";
    }
    return s + "}\n";
}

bool parseSession(const std::string& text, JVal& root, Scenario& sc, std::string& config, std::vector<WorldCommand>& cmds, std::string& error) {
    if (!parseJson(text, root, error)) return false;
    if (root.str("format") != "ane-session") {
        error = "not a session file";
        return false;
    }
    if (root.num("version", 0.0) != 1.0) {
        error = "unsupported session version";
        return false;
    }
    const JVal* c = root.get("config");
    if (!c || !scenarioFromJVal(*c, sc, error)) {
        if (error.empty()) error = "session without a configuration";
        return false;
    }
    config = dump(*c);
    cmds.clear();
    if (const JVal* list = root.get("commands")) {
        for (const JVal& item : list->a) {
            WorldCommand w;
            if (!commandFromJVal(item, w, error)) return false;
            cmds.push_back(w);
        }
    }
    std::stable_sort(cmds.begin(), cmds.end(), [](const WorldCommand& a, const WorldCommand& b) { return a.tick < b.tick; });
    return true;
}

// Applies recorded commands at their ticks; nothing else (headless replay).
class ReplayHooks : public EpisodeHooks {
public:
    explicit ReplayHooks(const std::vector<WorldCommand>& cmds) : cmds_(cmds) {}
    bool beforeTick(const TickView& v, TickControl& ctl) override {
        while (next_ < cmds_.size() && cmds_[next_].tick <= v.tick) {
            std::string why;
            applyCommand(cmds_[next_], v, ctl, why);
            ++next_;
        }
        return true;
    }
    void afterTick(const TickView&) override {}

private:
    const std::vector<WorldCommand>& cmds_;
    size_t next_ = 0;
};

bool inside(const std::vector<Zone>& zs, double x, double y) {
    for (const Zone& z : zs) {
        if (z.contains(x, y)) return true;
    }
    return false;
}

// ---- The server ---------------------------------------------------------------------------------------
class Server : public EpisodeHooks {
public:
    Server(std::istream& in, std::ostream& out, std::ostream& diag, const LiveOptions& opt) : in_(in), out_(out), diag_(diag), opt_(opt) {
        speed_ = std::max(0.0, opt.speed);
        paused_ = opt.startPaused;
        observeEvery_ = std::max(1, opt.observeEvery);
    }

    int run();
    bool beforeTick(const TickView& v, TickControl& ctl) override;
    void afterTick(const TickView& v) override;

private:
    using Clock = std::chrono::steady_clock;
    enum class Pending { None, Live, Replay, Quit };

    void emit(const std::string& json) { out_ << json << '\n'; }
    void flush() { out_.flush(); }
    std::string head(const char* type) { return "{\"v\":" + std::to_string(kProtocolVersion) + ",\"type\":\"" + type + "\""; }
    std::string headTick(const char* type, long tick, double t) { return head(type) + ",\"tick\":" + std::to_string(tick) + ",\"t\":" + jnum(t, 2); }
    void error(const std::string& message) {
        emit(head("error") + ",\"message\":" + jstr(message) + "}");
        diag_ << "live: " << message << "\n";
    }

    // Input.
    void reader();
    bool popLine(std::string& line, int waitMs);
    void handleInput(const TickView* v);  // drains commands; stops at one that ends the episode
    bool handleLine(const std::string& line, const TickView* v);  // false: stop draining
    bool endEpisode() const { return pending_ != Pending::None; }

    // Episodes.
    void startLive(const std::string& config);
    void startReplay(const std::string& path);
    void runEpisodeNow();
    void sendScene(const TickView& v);
    void sendClosure(const TickView& v);
    void sendStatus(const TickView* v, bool force);
    void pace(double tNext);
    void saveSession(const std::string& path);

    std::istream& in_;
    std::ostream& out_;
    std::ostream& diag_;
    LiveOptions opt_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<std::string> lines_;
    bool eof_ = false;

    Pending pending_ = Pending::None;
    std::string pendingConfig_, pendingReplay_;
    bool quit_ = false;

    // The current episode.
    std::string mode_ = "idle";  // live, replay, idle
    std::string state_ = "idle";  // running, paused, finished, idle
    Scenario sc_;
    std::string config_, scenarioName_;
    std::vector<WorldCommand> scheduled_;  // replay: from the file; live: requested, not yet applied
    size_t nextScheduled_ = 0;
    std::vector<WorldCommand> applied_;
    std::vector<std::pair<std::string, long>> liveRequests_;  // live: raw requests waiting for this tick
    bool haveResult_ = false;
    EpisodeResult last_;
    std::string recordedHash_, recordedToolchain_, recordedOutcome_;

    // Pacing and status.
    double speed_ = 1.0;
    bool paused_ = false;
    long stepBudget_ = 0;
    int observeEvery_ = 1;
    Clock::time_point epochWall_;
    double epochSim_ = 0.0;
    bool epochSet_ = false;
    Clock::time_point lastStatus_;
    Clock::time_point rtfWall_;
    double rtfSim_ = 0.0, rtf_ = 0.0;
    int lagEvents_ = 0;
    bool lagging_ = false;

    // Observation bookkeeping.
    bool sceneSent_ = false;
    size_t worldSent_ = 0, stackSent_ = 0, routesSent_ = 0;
    int verifiedSeen_ = 0, uniqueSeen_ = 0, relocSeen_ = 0, kerbUsedSeen_ = 0, kerbRejSeen_ = 0;
    std::string closureSig_;
    std::vector<VehicleState> ring_ = std::vector<VehicleState>(1024);
    long lastTick_ = -1;
};

void Server::reader() {
    std::string line;
    while (std::getline(in_, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        std::lock_guard<std::mutex> lock(mu_);
        lines_.push_back(line);
        cv_.notify_all();
    }
    std::lock_guard<std::mutex> lock(mu_);
    eof_ = true;
    cv_.notify_all();
}

bool Server::popLine(std::string& line, int waitMs) {
    std::unique_lock<std::mutex> lock(mu_);
    if (lines_.empty() && waitMs > 0 && !eof_) {
        cv_.wait_for(lock, std::chrono::milliseconds(waitMs), [&] { return !lines_.empty() || eof_; });
    }
    if (lines_.empty()) return false;
    line = lines_.front();
    lines_.pop_front();
    return true;
}

void Server::handleInput(const TickView* v) {
    std::string line;
    while (!endEpisode() && popLine(line, 0)) {
        if (!handleLine(line, v)) break;
    }
    bool eof = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        eof = eof_ && lines_.empty();
    }
    if (eof && !opt_.script && !endEpisode()) {
        diag_ << "live: input closed, stopping\n";
        quit_ = true;
        pending_ = Pending::Quit;
    }
}

bool Server::handleLine(const std::string& line, const TickView* v) {
    JVal msg;
    std::string err;
    if (!parseJson(line, msg, err) || msg.kind != JVal::Obj) {
        error("bad command (" + (err.empty() ? std::string("not an object") : err) + "): " + line.substr(0, 120));
        return true;
    }
    const std::string cmd = msg.str("cmd");
    if (cmd == "quit") {
        quit_ = true;
        pending_ = Pending::Quit;
        return false;
    }
    if (cmd == "reset") {
        const JVal* c = msg.get("config");
        pendingConfig_ = c ? dump(*c) : config_;
        if (pendingConfig_.empty()) {
            error("reset needs a configuration");
            return true;
        }
        pending_ = Pending::Live;
        return false;
    }
    if (cmd == "replay") {
        pendingReplay_ = msg.str("session");
        pending_ = Pending::Replay;
        return false;
    }
    if (cmd == "pause") {
        paused_ = true;
        stepBudget_ = 0;
        sendStatus(v, true);
        return true;
    }
    if (cmd == "resume") {
        paused_ = false;
        stepBudget_ = 0;
        epochSet_ = false;  // pacing restarts from here: no catching up the paused time
        sendStatus(v, true);
        return true;
    }
    if (cmd == "step") {
        paused_ = true;
        stepBudget_ += std::max(1L, std::lround(msg.num("ticks", 1.0)));
        return true;
    }
    if (cmd == "speed") {
        speed_ = std::max(0.0, msg.num("value", 1.0));
        epochSet_ = false;
        sendStatus(v, true);
        return true;
    }
    if (cmd == "observe") {
        observeEvery_ = std::max(1, static_cast<int>(std::lround(msg.num("every", 1.0))));
        return true;
    }
    if (cmd == "save_session") {
        saveSession(msg.str("path"));
        return true;
    }
    if (cmd == "gnss_outage" || cmd == "kerb_blackout" || cmd == "gnss_bias" || cmd == "closure") {
        if (mode_ != "live" || state_ == "finished" || state_ == "idle") {
            error(mode_ == "replay" ? "replay: world commands come from the session file" : "no live episode is running");
            return true;
        }
        const long at = msg.get("at_tick") ? std::lround(msg.num("at_tick", 0.0)) : -1;
        liveRequests_.push_back({line, at});
        return true;
    }
    error("unknown command '" + cmd + "'");
    return true;
}

void Server::startLive(const std::string& config) {
    JVal c;
    std::string err;
    Scenario sc;
    if (!parseJson(config, c, err) || !scenarioFromJVal(c, sc, err)) {
        error("cannot start: " + err);
        state_ = haveResult_ ? "finished" : "idle";
        return;
    }
    sc_ = sc;
    scenarioName_ = c.str("scenario", "R7-A");
    config_ = configJson(sc_, scenarioName_);
    mode_ = "live";
    scheduled_.clear();
    recordedHash_.clear();
    recordedToolchain_.clear();
    recordedOutcome_.clear();
    runEpisodeNow();
}

void Server::startReplay(const std::string& path) {
    std::string text, err;
    JVal root;
    Scenario sc;
    std::string config;
    std::vector<WorldCommand> cmds;
    if (!readText(path, text)) {
        error("cannot read session " + path);
        return;
    }
    if (!parseSession(text, root, sc, config, cmds, err)) {
        error("session " + path + ": " + err);
        return;
    }
    sc_ = sc;
    JVal c;
    parseJson(config, c, err);
    scenarioName_ = c.str("scenario", "R7-A");
    config_ = configJson(sc_, scenarioName_);
    mode_ = "replay";
    scheduled_ = cmds;
    recordedHash_.clear();
    recordedToolchain_.clear();
    recordedOutcome_.clear();
    if (const JVal* r = root.get("result")) {
        recordedHash_ = r->str("trace_hash");
        recordedToolchain_ = r->str("toolchain");
        recordedOutcome_ = r->str("outcome");
    }
    runEpisodeNow();
}

void Server::runEpisodeNow() {
    nextScheduled_ = 0;
    applied_.clear();
    liveRequests_.clear();
    haveResult_ = false;
    sceneSent_ = false;
    worldSent_ = stackSent_ = routesSent_ = 0;
    verifiedSeen_ = uniqueSeen_ = relocSeen_ = kerbUsedSeen_ = kerbRejSeen_ = 0;
    closureSig_.clear();
    lastTick_ = -1;
    epochSet_ = false;
    lagEvents_ = 0;
    lagging_ = false;
    rtf_ = 0.0;
    stepBudget_ = 0;
    state_ = paused_ ? "paused" : "running";
    diag_ << "live: " << mode_ << " episode " << config_ << "\n";
    EpisodeResult r = runEpisodeHooked(sc_, this);
    if (r.outcome == "ABORTED") {
        diag_ << "live: episode ended early at t=" << jnum(r.simSeconds, 2) << " s\n";
        state_ = "idle";
        return;
    }
    last_ = r;
    haveResult_ = true;
    state_ = "finished";
    const Metrics& m = r.metrics;
    std::string match = "null";
    if (!recordedHash_.empty()) {
        const bool same = recordedToolchain_ == toolchain();
        match = std::string("{\"hash_equal\":") + (hex64(r.traceHash) == recordedHash_ ? "true" : "false") + ",\"same_toolchain\":" + (same ? "true" : "false") +
                ",\"outcome_equal\":" + (r.outcome == recordedOutcome_ ? "true" : "false") + ",\"recorded_hash\":" + jstr(recordedHash_) + "}";
    }
    emit(headTick("end", lastTick_, r.simSeconds) + ",\"mode\":" + jstr(mode_) + ",\"outcome\":" + jstr(r.outcome) +
         ",\"stack_mode\":" + jstr(r.stackFinalMode) + ",\"trace_hash\":" + jstr(hex64(r.traceHash)) + ",\"toolchain\":" + jstr(toolchain()) +
         ",\"replay_check\":" + match + ",\"metrics\":{\"completed\":" + (m.completed ? "true" : "false") + ",\"time_to_goal\":" + jnum(m.timeToGoal, 2) +
         ",\"max_crossing\":" + jnum(m.maxCrossing, 3) + ",\"min_clearance\":" + jnum(m.minClearance, 3) +
         ",\"clearance_violations\":" + std::to_string(m.clearanceViolations) + ",\"incursion_depth\":" + jnum(m.incursionDepth, 3) +
         ",\"est_rmse\":" + jnum(m.estRmse, 3) + ",\"est_max\":" + jnum(m.estMax, 3) + ",\"fallbacks\":" + std::to_string(m.fallbacks) +
         "},\"commands\":" + std::to_string(applied_.size()) + "}");
    sendStatus(nullptr, true);
    flush();
    diag_ << "live: " << r.outcome << " at t=" << jnum(r.simSeconds, 2) << " s, trace " << hex64(r.traceHash) << "\n";
    if (!opt_.sessionOut.empty()) saveSession(opt_.sessionOut);
}

void Server::saveSession(const std::string& path) {
    if (config_.empty()) {
        error("no episode to save");
        return;
    }
    const std::string target = path.empty() ? std::string("session.json") : path;
    std::ofstream f(target, std::ios::binary);
    f << sessionJson(config_, mode_ == "replay" ? scheduled_ : applied_, haveResult_ ? &last_ : nullptr);
    if (!f) {
        error("cannot write " + target);
        return;
    }
    emit(head("saved") + ",\"path\":" + jstr(target) + ",\"commands\":" + std::to_string(mode_ == "replay" ? scheduled_.size() : applied_.size()) +
         ",\"complete\":" + (haveResult_ ? "true" : "false") + "}");
    flush();
    diag_ << "live: session written to " << target << "\n";
}

void Server::sendScene(const TickView& v) {
    const VehicleParams& vp = *v.vehicle;
    // The sensors as mounted on this vehicle (with no condition, where they would be).
    const SensorConditions cond = v.scenario->condition.empty() ? sensorConditionsFor("nominal", vp) : sensorConditionsFor(v.scenario->condition, vp);
    std::string zones;
    auto zoneList = [&](const char* name, const std::vector<Zone>& zs) {
        zones += std::string(zones.empty() ? "" : ",") + "\"" + name + "\":[";
        for (size_t i = 0; i < zs.size(); ++i) {
            zones += (i ? "," : "") + std::string("[") + fixed3(zs[i].x0) + "," + fixed3(zs[i].y0) + "," + fixed3(zs[i].x1) + "," + fixed3(zs[i].y1) + "]";
        }
        zones += "]";
    };
    zoneList("outage", v.zones->outage);
    zoneList("multipath", v.zones->multipath);
    zoneList("slip", v.zones->slip);
    zoneList("ramp", v.zones->ramp);
    zoneList("kerb_blackout", v.zones->kerbBlackout);
    zoneList("kerb_shift", v.zones->kerbShift);
    std::string obstacles = "[";
    for (size_t i = 0; i < v.zones->obstacles.size(); ++i) {
        const Obstacle& o = v.zones->obstacles[i];
        obstacles += (i ? "," : "") + std::string("[") + fixed3(o.cx) + "," + fixed3(o.cy) + "," + jnum(o.yaw, 5) + "," + fixed3(o.length) + "," +
                     fixed3(o.width) + "]";
    }
    obstacles += "]";
    // A map error: each misplaced kerb part as the vehicle's map has it, and where the world keeps the true kerb
    // (absent when none).
    std::string mapErrors;
    if (!v.zones->mapErrors.empty()) {
        mapErrors = ",\"map_errors\":[";
        for (size_t i = 0; i < v.zones->mapErrors.size(); ++i) {
            const MapKerbError& me = v.zones->mapErrors[i];
            double ax, ay, bx, by;
            mapErrorSegment(*v.net, me, ax, ay, bx, by);
            const KerbLine& k = v.net->kerbLines[static_cast<size_t>(me.line)];
            mapErrors += (i ? "," : "") + std::string("{\"line\":") + std::to_string(me.line) + ",\"offset\":" + fixed3(me.offset) + ",\"map\":[" +
                         jnum(ax, 4) + "," + jnum(ay, 4) + "," + jnum(bx, 4) + "," + jnum(by, 4) + "],\"true\":[" + jnum(ax - k.nx * me.offset, 4) + "," +
                         jnum(ay - k.ny * me.offset, 4) + "," + jnum(bx - k.nx * me.offset, 4) + "," + jnum(by - k.ny * me.offset, 4) + "]}";
        }
        mapErrors += "]";
    }
    emit(headTick("session", v.tick, v.t) + ",\"mode\":" + jstr(mode_) + ",\"config\":" + config_ + ",\"labels\":" +
         jstr(std::string(v.net->general ? "SOURCE_INFORMED_ROADS" : "SYNTHETIC_ROADS") +
              (v.scenario->mode == "estimated" ? ",ESTIMATED_POSE(" + v.scenario->estimator + "),SIMULATED_SENSORS,ORACLE_CLOSURE"
                                               : ",GROUND_TRUTH_POSE,ORACLE_CLOSURE")) +
         ",\"dt\":" + jnum(v.dt, 3) + ",\"time_budget\":" + jnum(v.scenario->timeBudget, 1) + ",\"backend\":{\"commit\":" + jstr(engineCommit()) +
         ",\"toolchain\":" + jstr(toolchain()) + "}}");
    emit(head("scene") + ",\"tick\":" + std::to_string(v.tick) + ",\"map\":" + sceneJson(*v.net) + ",\"vehicle\":{\"id\":" + jstr(vp.id) +
         ",\"wheelbase\":" + fixed3(vp.wheelbase) + ",\"length\":" + fixed3(vp.length) + ",\"width\":" + fixed3(vp.width) +
         ",\"width_mirrors\":" + fixed3(vp.widthMirrors) + ",\"front_overhang\":" + fixed3(vp.frontOverhang) + ",\"rear_overhang\":" + fixed3(vp.rearOverhang) +
         ",\"max_steer\":" + jnum(vp.maxSteer, 3) + ",\"description\":" + jstr(vp.description) + ",\"height\":" + fixed3(vp.height) +
         ",\"wheel_radius\":" + fixed3(vp.wheelRadius) + ",\"track\":" + fixed3(vp.track) + "},\"sensors\":{\"kerb\":{\"mount_x\":" + fixed3(cond.kerb.mountX) +
         ",\"first_bearing\":" + jnum(cond.kerb.firstBearing, 5) + ",\"bearing_step\":" + jnum(cond.kerb.bearingStep, 5) + ",\"beams\":" +
         std::to_string(cond.kerb.beams) + ",\"min_range\":" + fixed3(cond.kerb.minRange) + ",\"max_range\":" + fixed3(cond.kerb.maxRange) +
         ",\"rate\":" + jnum(cond.kerb.rate, 1) + "},\"gnss\":{\"antenna_x\":" + fixed3(cond.gnss.antennaX) + ",\"antenna_y\":" + fixed3(cond.gnss.antennaY) +
         ",\"rate\":" + jnum(cond.gnss.rate, 1) + "},\"imu\":{\"lever_x\":" + fixed3(cond.imu.leverX) + "}},\"zones\":{" + zones + "},\"obstacles\":" + obstacles +
         mapErrors + "}");
    sceneSent_ = true;
}

void Server::sendClosure(const TickView& v) {
    const LiveClosure& c = *v.closure;
    if (!c.on) return;
    const std::string sig = std::to_string(c.lane) + (c.present ? "P" : "-") + (c.revealed ? "R" : "-") + (c.lifted ? "L" : "-");
    if (sig == closureSig_) return;
    closureSig_ = sig;
    const PathPoint a = v.net->lanePoint(c.lane, c.barrierLaneS);
    const PathPoint b = v.net->lanePoint(c.lane, c.barrierLaneS + c.worksLength);
    emit(headTick("closure", v.tick, v.t) + ",\"id\":\"X1\",\"lane\":" + jstr(v.net->lanes[static_cast<size_t>(c.lane)].name) +
         ",\"present\":" + (c.present ? "true" : "false") + ",\"notified\":" + (c.revealed ? "true" : "false") + ",\"lifted\":" + (c.lifted ? "true" : "false") +
         ",\"barrier\":[" + fixed3(a.x) + "," + fixed3(a.y) + "," + jnum(a.yaw, 5) + "],\"works_end\":[" + fixed3(b.x) + "," + fixed3(b.y) + "," + jnum(b.yaw, 5) +
         "],\"barrier_lane_s\":" + fixed3(c.barrierLaneS) + ",\"works_length\":" + fixed3(c.worksLength) + ",\"lane_width\":" + fixed3(v.net->spec.laneWidth) + "}");
}

void Server::sendStatus(const TickView* v, bool force) {
    const Clock::time_point now = Clock::now();
    if (!force && now - lastStatus_ < std::chrono::milliseconds(250)) return;
    lastStatus_ = now;
    const std::string st = state_ == "finished" || state_ == "idle" ? state_ : (paused_ ? "paused" : "running");
    emit(head("status") + ",\"state\":" + jstr(st) + ",\"mode\":" + jstr(mode_) + ",\"tick\":" + std::to_string(v ? v->tick : lastTick_) +
         ",\"t\":" + jnum(v ? v->t : (lastTick_ < 0 ? 0.0 : lastTick_ * 0.01), 2) + ",\"speed\":" + jnum(speed_, 3) + ",\"rtf\":" + jnum(rtf_, 3) +
         ",\"lagging\":" + (lagging_ ? "true" : "false") + ",\"lag_events\":" + std::to_string(lagEvents_) + ",\"observe_every\":" + std::to_string(observeEvery_) +
         ",\"commands\":" + std::to_string(mode_ == "replay" ? scheduled_.size() : applied_.size()) + "}");
}

bool Server::beforeTick(const TickView& v, TickControl& ctl) {
    if (!sceneSent_) {
        sendScene(v);
        flush();
    }
    handleInput(&v);
    // Pause freezes simulated time: no tick runs, so no sensor progresses. The interface stays live.
    while (paused_ && stepBudget_ == 0 && !endEpisode()) {
        state_ = "paused";
        sendStatus(&v, false);
        flush();
        std::string line;
        if (popLine(line, 50)) {
            handleLine(line, &v);
        }
        handleInput(&v);
    }
    if (endEpisode()) return false;
    state_ = paused_ ? "paused" : "running";
    if (paused_ && stepBudget_ > 0) --stepBudget_;

    // Live requests take effect now (or at their at_tick); a replay's commands at their recorded ticks.
    if (mode_ == "live") {
        std::vector<std::pair<std::string, long>> keep;
        for (const auto& req : liveRequests_) {
            if (req.second > v.tick) {
                keep.push_back(req);
                continue;
            }
            JVal msg;
            std::string err;
            parseJson(req.first, msg, err);
            const std::string cmd = msg.str("cmd");
            WorldCommand w;
            w.tick = v.tick;
            std::string why;
            bool ok = true;
            if (cmd == "gnss_outage" || cmd == "kerb_blackout") {
                w.cmd = cmd;
                w.on = msg.flag("on", true);
            } else if (cmd == "gnss_bias") {
                // Metres at a bearing relative to the car's true heading now, fixed in the map frame from here on.
                w.cmd = "gnss_bias";
                if (msg.flag("on", true)) {
                    const double m = msg.num("metres", 5.0), b = msg.num("bearing_deg", 90.0) * kPi / 180.0;
                    w.dx = m * std::cos(v.truth->yaw + b);
                    w.dy = m * std::sin(v.truth->yaw + b);
                }
            } else if (cmd == "closure") {
                const std::string action = msg.str("action", "ahead");
                if (action == "lift") {
                    w.cmd = "closure_lift";
                } else {
                    ok = closureAhead(v, msg.num("distance", 40.0), msg.num("works", 15.0), w, why);
                }
            }
            if (ok) ok = applyCommand(w, v, ctl, why);
            if (!ok) {
                error(cmd + " not applied at tick " + std::to_string(v.tick) + ": " + why);
                continue;
            }
            applied_.push_back(w);
            emit(headTick("applied", v.tick, v.t) + ",\"command\":" + commandJson(w) + (req.second >= 0 && req.second < v.tick ? ",\"late\":true" : "") + "}");
        }
        liveRequests_.swap(keep);
    } else if (mode_ == "replay") {
        while (nextScheduled_ < scheduled_.size() && scheduled_[nextScheduled_].tick <= v.tick) {
            const WorldCommand& w = scheduled_[nextScheduled_++];
            std::string why;
            if (applyCommand(w, v, ctl, why)) {
                emit(headTick("applied", v.tick, v.t) + ",\"command\":" + commandJson(w) + ",\"replay\":true}");
            } else {
                error("replayed " + w.cmd + " not applied: " + why);
            }
        }
    }
    return true;
}

void Server::afterTick(const TickView& v) {
    lastTick_ = v.tick;
    const VehicleState& x = *v.truth;
    ring_[static_cast<size_t>(v.tick) % ring_.size()] = x;
    const StateEstimate* e = v.estimate;

    // Measurements released this tick, as the estimator received them.
    if (v.arrived) {
        for (const Measurement& m : *v.arrived) {
            if (m.kind == Measurement::Gnss) {
                emit(headTick("gnss", v.tick, v.t) + ",\"t_meas\":" + jnum(m.tMeas, 2) + ",\"x\":" + fixed3(m.a) + ",\"y\":" + fixed3(m.b) +
                     ",\"sigma\":" + fixed3(m.c) + "}");
            } else if (m.kind == Measurement::Kerb) {
                const long k = std::lround(m.tMeas / v.dt);
                const VehicleState& p = (v.tick - k >= 0 && v.tick - k < static_cast<long>(ring_.size())) ? ring_[static_cast<size_t>(k) % ring_.size()] : x;
                std::string ranges = "[";
                for (size_t i = 0; i < m.ranges.size(); ++i) ranges += (i ? "," : "") + (m.ranges[i] > 0.0 ? jnum(m.ranges[i], 3) : std::string("-1"));
                ranges += "]";
                emit(headTick("scan", v.tick, v.t) + ",\"t_meas\":" + jnum(m.tMeas, 2) + ",\"true_pose\":[" + fixed3(p.x) + "," + fixed3(p.y) + "," +
                     jnum(p.yaw, 5) + "],\"first_bearing\":" + jnum(m.a, 5) + ",\"bearing_step\":" + jnum(m.b, 5) + ",\"mount_x\":" + fixed3(m.c) +
                     ",\"ranges\":" + ranges + ",\"match\":" + jstr(e ? e->kerb : "-") + "}");
            }
        }
    }
    // The estimator's own events: kerb corrections it verified (and whether the match was unique), and
    // re-localisations.
    if (e && v.estimator) {
        if (e->verifiedCorrections > verifiedSeen_) {
            const bool unique = e->uniqueCorrections > uniqueSeen_;
            emit(headTick("event", v.tick, v.t) + ",\"source\":\"ESTIMATOR\",\"event\":\"KERB_CORRECTION\",\"detail\":" +
                 jstr(std::string(unique ? "unique" : "not unique") + ";dx=" + jnum(e->verifiedJumpX, 3) + ";dy=" + jnum(e->verifiedJumpY, 3)) +
                 ",\"unique\":" + (unique ? "true" : "false") + ",\"dx\":" + fixed3(e->verifiedJumpX) + ",\"dy\":" + fixed3(e->verifiedJumpY) + "}");
            verifiedSeen_ = e->verifiedCorrections;
            uniqueSeen_ = e->uniqueCorrections;
        }
        if (v.estimator->stats.kerbRelocalisations > relocSeen_) {
            relocSeen_ = v.estimator->stats.kerbRelocalisations;
            emit(headTick("event", v.tick, v.t) + ",\"source\":\"ESTIMATOR\",\"event\":\"KERB_RELOCALISATION\",\"detail\":" +
                 jstr("count=" + std::to_string(relocSeen_)) + "}");
        }
    }
    // World and stack events.
    while (worldSent_ < v.worldEvents->size()) {
        const LogEvent& ev = (*v.worldEvents)[worldSent_++];
        emit(headTick("event", v.tick, ev.t) + ",\"source\":" + jstr(ev.source) + ",\"event\":" + jstr(ev.type) + ",\"detail\":" + jstr(ev.detail) + "}");
    }
    while (stackSent_ < v.stack->events.size()) {
        const LogEvent& ev = v.stack->events[stackSent_++];
        emit(headTick("event", v.tick, ev.t) + ",\"source\":" + jstr(ev.source) + ",\"event\":" + jstr(ev.type) + ",\"detail\":" + jstr(ev.detail) + "}");
    }
    sendClosure(v);
    // The route the stack adopted, and the reference it steers to (path with its clearance shift and re-join).
    const DrivePath& path = v.stack->path();
    const bool newRoute = v.stack->routes.size() != routesSent_;
    if (newRoute && !path.empty()) {
        routesSent_ = v.stack->routes.size();
        std::string pts = "[";
        bool first = true;
        double lastS = -1e9;
        for (const PathSample& p : path.samples()) {
            if (p.s - lastS < 2.0 && &p != &path.samples().back()) continue;
            lastS = p.s;
            pts += (first ? "" : ",") + std::string("[") + fixed3(p.x) + "," + fixed3(p.y) + "]";
            first = false;
        }
        pts += "]";
        const RouteRecord& rr = v.stack->routes.back();
        emit(headTick("route", v.tick, v.t) + ",\"reason\":" + jstr(rr.reason) + ",\"route\":" + jstr(routeText(*v.net, rr.elements)) + ",\"pts\":" + pts + "}");
    }
    if (v.stackStarted && !path.empty() && (newRoute || v.tick % 10 == 0)) {
        std::string pts = "[";
        const double s0 = std::max(0.0, v.stack->pathS() - 5.0), s1 = std::min(path.length(), v.stack->pathS() + 80.0);
        bool first = true;
        for (double s = s0; s <= s1; s += 1.0) {
            const PathSample p = path.at(s);
            const double d = v.stack->pathShiftAt(s) + v.stack->rejoinAt(s);
            pts += (first ? "" : ",") + std::string("[") + fixed3(p.x - std::sin(p.yaw) * d) + "," + fixed3(p.y + std::cos(p.yaw) * d) + "]";
            first = false;
        }
        pts += "]";
        emit(headTick("path", v.tick, v.t) + ",\"s\":" + fixed3(v.stack->pathS()) + ",\"pts\":" + pts + "}");
    }
    // The tick: truth (for rendering and diagnostics only), the command, the estimate, the stack, diagnostics.
    if (v.tick % observeEvery_ == 0) {
        const VehicleParams& vp = *v.vehicle;
        const double clearance = footprintClearance(*v.net, x.x, x.y, x.yaw, vp.rearOverhang, vp.frontOffset(), vp.widthMirrors);
        const double body = footprintClearance(*v.net, x.x, x.y, x.yaw, vp.rearOverhang, vp.frontOffset(), vp.width);
        std::string est = "null", diag = "null";
        if (e) {
            est = std::string("{\"valid\":") + (e->valid ? "true" : "false") + ",\"x\":" + fixed3(e->x) + ",\"y\":" + fixed3(e->y) + ",\"yaw\":" +
                  jnum(e->yaw, 5) + ",\"v\":" + fixed3(e->v) + ",\"pxx\":" + jg(e->pxx) + ",\"pxy\":" + jg(e->pxy) + ",\"pyy\":" + jg(e->pyy) +
                  ",\"pyawyaw\":" + jg(e->pyawyaw) + ",\"sigma\":" + jnum(e->valid ? e->sigmaPos() : -1.0, 3) + ",\"sigma_along\":" +
                  jnum(e->valid ? e->sigmaAlong(e->yaw) : -1.0, 3) + ",\"sigma_across\":" + jnum(e->valid ? e->sigmaAcross(e->yaw) : -1.0, 3) +
                  ",\"status\":" + jstr(e->status) + ",\"gnss\":" + jstr(e->gnss) + ",\"kerb\":" + jstr(e->kerb) +
                  ",\"verified\":" + std::to_string(e->verifiedCorrections) + ",\"unique\":" + std::to_string(e->uniqueCorrections) +
                  ",\"relocalisations\":" + std::to_string(v.estimator ? v.estimator->stats.kerbRelocalisations : 0) +
                  ",\"kerb_used\":" + std::to_string(v.estimator ? v.estimator->stats.kerbUsed : 0) +
                  ",\"kerb_rejected\":" + std::to_string(v.estimator ? v.estimator->stats.kerbRejected : 0) + "}";
            if (e->valid) {
                const double dx = e->x - x.x, dy = e->y - x.y, c = std::cos(x.yaw), s = std::sin(x.yaw);
                diag = "{\"along\":" + fixed3(dx * c + dy * s) + ",\"lateral\":" + fixed3(-dx * s + dy * c) + ",\"heading_deg\":" +
                       jnum(wrapAngle(e->yaw - x.yaw) * 180.0 / kPi, 3) + ",\"position\":" + fixed3(std::hypot(dx, dy)) + "}";
            }
        }
        std::string sensors = "null";
        if (v.sensors) {
            const LiveFaults& f = v.sensors->liveFaults();
            const bool blackout = f.kerbBlackout || inside(v.zones->kerbBlackout, x.x, x.y);
            sensors = std::string("{\"gnss_available\":") + (v.sensors->gnssAvailable() ? "true" : "false") + ",\"gnss_fault_m\":" +
                      fixed3(v.sensors->gnssFault()) + ",\"multipath\":" + (v.sensors->inMultipath() ? "true" : "false") + ",\"wheel_slip\":" +
                      (v.sensors->inSlip() ? "true" : "false") + ",\"kerb_scanning\":" + (blackout ? "false" : "true") +
                      ",\"kerb_map_error\":" + (inside(v.zones->kerbShift, x.x, x.y) ? "true" : "false") + ",\"injected\":{\"gnss_outage\":" +
                      (f.gnssOutage ? "true" : "false") + ",\"kerb_blackout\":" + (f.kerbBlackout ? "true" : "false") + ",\"gnss_bias\":[" +
                      fixed3(f.gnssBiasX) + "," + fixed3(f.gnssBiasY) + "]}}";
        }
        const Stack& st = *v.stack;
        const int elem = v.stackStarted ? st.believedElement() : -1;
        emit(headTick("tick", v.tick, v.t) + ",\"truth\":{\"x\":" + fixed3(x.x) + ",\"y\":" + fixed3(x.y) + ",\"yaw\":" + jnum(x.yaw, 5) + ",\"v\":" +
             fixed3(x.v) + ",\"steer\":" + jnum(x.steer, 5) + ",\"accel\":" + fixed3(x.accel) + "},\"cmd\":{\"steer\":" + jnum(v.command->steer, 5) +
             ",\"accel\":" + fixed3(v.command->accel) + "},\"est\":" + est + ",\"stack\":{\"started\":" + (v.stackStarted ? "true" : "false") +
             ",\"mode\":" + jstr(st.modeName()) + ",\"element\":" + jstr(elem >= 0 ? v.net->elements[static_cast<size_t>(elem)].name : "-") +
             ",\"s\":" + fixed3(st.pathS()) + ",\"shift\":" + fixed3(v.stackStarted ? st.currentShift() : 0.0) + ",\"rejoin\":" +
             fixed3(v.stackStarted ? st.rejoinAt(st.pathS()) : 0.0) + "},\"diag\":" + diag + ",\"clearance\":" + fixed3(clearance) +
             ",\"crossing\":" + fixed3(std::max(0.0, -body)) + ",\"sensors\":" + sensors + "}");
    }
    sendStatus(&v, false);
    flush();
    pace(v.t + v.dt);
}

void Server::pace(double tNext) {
    const Clock::time_point now = Clock::now();
    if (!epochSet_) {
        epochWall_ = now;
        epochSim_ = tNext;
        epochSet_ = true;
        rtfWall_ = now;
        rtfSim_ = tNext;
        return;
    }
    const double wallSince = std::chrono::duration<double>(now - rtfWall_).count();
    if (wallSince >= 1.0) {
        rtf_ = (tNext - rtfSim_) / wallSince;
        rtfWall_ = now;
        rtfSim_ = tNext;
        lagging_ = speed_ > 0.0 && !paused_ && rtf_ < 0.95 * speed_;
    }
    if (speed_ <= 0.0 || (paused_ && stepBudget_ > 0)) return;  // unpaced, or stepping
    const Clock::time_point target = epochWall_ + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((tNext - epochSim_) / speed_));
    if (now < target) {
        std::this_thread::sleep_until(target);
    } else if (now - target > std::chrono::milliseconds(250)) {
        // The machine cannot keep up: report it and pace on from here rather than racing to catch up.
        ++lagEvents_;
        epochWall_ = now;
        epochSim_ = tNext;
    }
}

int Server::run() {
    emit(head("hello") + ",\"protocol\":\"ane-live\",\"backend\":{\"commit\":" + jstr(engineCommit()) + ",\"toolchain\":" + jstr(toolchain()) +
         "},\"commands\":[\"reset\",\"replay\",\"pause\",\"resume\",\"step\",\"speed\",\"observe\",\"gnss_outage\",\"gnss_bias\",\"kerb_blackout\","
         "\"closure\",\"save_session\",\"quit\"]}");
    flush();
    std::thread readerThread;
    if (opt_.script) {
        this->reader();  // the whole script, now: its commands carry ticks
    } else {
        readerThread = std::thread(&Server::reader, this);
    }
    if (!opt_.replayPath.empty()) {
        pending_ = Pending::Replay;
        pendingReplay_ = opt_.replayPath;
    } else if (!opt_.initialConfig.empty()) {
        pending_ = Pending::Live;
        pendingConfig_ = opt_.initialConfig;
    }
    while (!quit_) {
        if (pending_ == Pending::Live) {
            pending_ = Pending::None;
            startLive(pendingConfig_);
        } else if (pending_ == Pending::Replay) {
            pending_ = Pending::None;
            startReplay(pendingReplay_);
        } else if (pending_ == Pending::Quit) {
            break;
        }
        if (quit_ || pending_ != Pending::None) continue;
        if (opt_.script && (state_ == "finished" || state_ == "idle")) {
            // A script runs its episodes, then ends; a reset it contains has already started the next one.
            std::lock_guard<std::mutex> lock(mu_);
            if (lines_.empty()) break;
        }
        std::string line;
        if (popLine(line, 100)) {
            handleLine(line, nullptr);
        } else {
            std::lock_guard<std::mutex> lock(mu_);
            if (eof_ && lines_.empty() && !opt_.script) {
                diag_ << "live: input closed, stopping\n";
                break;
            }
        }
        sendStatus(nullptr, false);
        flush();
    }
    emit(head("bye") + "}");
    flush();
    if (readerThread.joinable()) {
        bool done = false;
        {
            std::lock_guard<std::mutex> lock(mu_);
            done = eof_;
        }
        if (done) {
            readerThread.join();
        } else {
            readerThread.detach();  // blocked on an open input that the process will close on exit
        }
    }
    return 0;
}

}  // namespace

bool scenarioFromConfig(const std::string& configJson, Scenario& sc, std::string& error) {
    JVal c;
    return parseJson(configJson, c, error) && scenarioFromJVal(c, sc, error);
}

bool replaySession(const std::string& sessionJson, SessionReplay& out, std::string& error) {
    JVal root;
    Scenario sc;
    std::string config;
    std::vector<WorldCommand> cmds;
    if (!parseSession(sessionJson, root, sc, config, cmds, error)) return false;
    ReplayHooks hooks(cmds);
    out.result = runEpisodeHooked(sc, &hooks);
    out.commands = static_cast<int>(cmds.size());
    if (const JVal* r = root.get("result")) {
        out.recordedHash = r->str("trace_hash");
        out.recordedToolchain = r->str("toolchain");
        out.recordedOutcome = r->str("outcome");
    }
    return true;
}

int runLiveServer(std::istream& in, std::ostream& out, std::ostream& diag, const LiveOptions& opt) {
    Server server(in, out, diag, opt);
    return server.run();
}

std::string sceneJson(const LaneNetwork& net) {
    std::string s = "{\"name\":" + jstr(net.spec.name) + ",\"lane_width\":" + fixed3(net.spec.laneWidth) + ",\"mouth\":" + fixed3(net.spec.mouth) +
                    ",\"junctions\":[";
    for (size_t j = 0; j < net.junctions.size(); ++j) {
        s += (j ? "," : "") + std::string("{\"name\":") + jstr(net.junctions[j].name) + ",\"x\":" + fixed3(net.junctions[j].x) + ",\"y\":" +
             fixed3(net.junctions[j].y) + ",\"kerb\":" + fixed3(net.kerbRadius[j]) + "}";
    }
    s += "],\"streets\":[";
    for (size_t i = 0; i < net.streets.size(); ++i) {
        const Street& st = net.streets[i];
        s += (i ? "," : "") + std::string("[") + std::to_string(st.a) + "," + std::to_string(st.b) + "," + fixed3(st.ax) + "," + fixed3(st.ay) + "," +
             fixed3(st.bx) + "," + fixed3(st.by) + "]";
    }
    s += "],\"lanes\":[";
    for (size_t i = 0; i < net.lanes.size(); ++i) {
        const Lane& l = net.lanes[i];
        s += (i ? "," : "") + std::string("{\"name\":") + jstr(l.name) + ",\"from\":" + std::to_string(l.from) + ",\"to\":" + std::to_string(l.to) +
             ",\"x0\":" + fixed3(l.x0) + ",\"y0\":" + fixed3(l.y0) + ",\"x1\":" + fixed3(l.x1) + ",\"y1\":" + fixed3(l.y1) + ",\"length\":" + fixed3(l.length) + "}";
    }
    s += "],\"kerb_lines\":[";
    for (size_t i = 0; i < net.kerbLines.size(); ++i) {
        const KerbLine& k = net.kerbLines[i];
        s += (i ? "," : "") + std::string("[") + jnum(k.ax, 4) + "," + jnum(k.ay, 4) + "," + jnum(k.bx, 4) + "," + jnum(k.by, 4) + "," + jnum(k.nx, 5) + "," +
             jnum(k.ny, 5) + "]";
    }
    s += "],\"kerb_arcs\":[";
    for (size_t i = 0; i < net.kerbArcs.size(); ++i) {
        const KerbArc& a = net.kerbArcs[i];
        s += (i ? "," : "") + std::string("[") + jnum(a.cx, 4) + "," + jnum(a.cy, 4) + "," + jnum(a.r, 4) + "," + jnum(a.mid, 6) + "," + jnum(a.half, 6) + "]";
    }
    s += "],\"banned\":[";
    for (size_t i = 0; i < net.banned.size(); ++i) {
        s += (i ? "," : "") + std::string("[") + jstr(net.banned[i].junction) + "," + jstr(net.banned[i].fromLane) + "," + jstr(net.banned[i].toLane) + "]";
    }
    s += "]";
    DistrictSpec d;
    if (net.general && findDistrict(net.spec.name, d)) {
        // A district network: its provenance; per street its name, widths and lane offset; and which kerb lines
        // are patch edges (not kerbs). The grids' scene messages are unchanged.
        // The city, the frame and the traffic side only where a district sets them (the London scene messages are
        // unchanged: left-hand traffic, their own frame).
        const std::string more = (d.city.empty() ? std::string() : ",\"city\":" + jstr(d.city)) + (d.frame.empty() ? std::string() : ",\"frame\":" + jstr(d.frame)) +
                                 (d.rightHand ? std::string(",\"traffic\":\"right\"") : std::string());
        s += ",\"district\":{\"id\":" + jstr(d.id) + ",\"district\":" + jstr(d.district) + ",\"title\":" + jstr(d.title) + ",\"origin\":[" +
             jnum(d.lat0, 6) + "," + jnum(d.lon0, 6) + "],\"source\":" + jstr(d.source) + ",\"licence\":" + jstr(d.licence) + more + "}";
        s += ",\"street_info\":[";
        for (size_t i = 0; i < net.streets.size(); ++i) {
            const Street& st = net.streets[i];
            s += (i ? "," : "") + std::string("{\"name\":") + jstr(st.name) + ",\"half_width\":" + fixed3(st.halfWidth) +
                 ",\"half_width_right\":" + fixed3(st.halfWidthRight) + ",\"median\":" + fixed3(st.median) + ",\"lane_offset\":" +
                 fixed3(st.laneOffset) + "}";
        }
        s += "],\"kerb_boundary\":[";
        bool first = true;
        for (size_t i = 0; i < net.kerbLines.size(); ++i) {
            if (!net.kerbLines[i].boundary) continue;
            s += (first ? "" : ",") + std::to_string(i);
            first = false;
        }
        s += "]";
    }
    return s + "}";
}

}  // namespace road
