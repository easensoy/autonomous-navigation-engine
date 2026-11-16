// road_drive: run, list and replay closed-loop episodes on the synthetic network R1.
//
//   road_drive list
//   road_drive run <scenario> [--variant B0|B1] [--seed N] [--out DIR]
//   road_drive replay <episode dir>
//   road_drive suite [--variant B0|B1] [--seeds N]
//
// Every episode uses synthetic roads, ground-truth pose and an oracle closure report.
#include "road_driving/episode.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
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
                 "  road_drive replay <episode dir>\n  road_drive suite [--variant B0|B1] [--seeds N]\n");
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
