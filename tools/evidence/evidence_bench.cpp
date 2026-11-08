// evidence_bench: times the library on saved workloads. Measurement mode: no search observer is
// attached anywhere in this program, std::cout (where the library logs) is silenced for the whole
// run, and only the planner call itself sits between the two clock reads.
//
//   evidence_bench queries <workload-dir> <name> <out-dir> [repeats] [warmup]
//   evidence_bench replans <workload-dir> <name> <out-dir> [repeats]
//   evidence_bench hero    <scenario.txt> <out-dir> [repeats]
//
// Raw samples go to CSV (nanoseconds, steady_clock); a JSON run record describes the run.
// Exit status 0 when every route checked is valid, 1 otherwise, 2 on bad usage or input.

#include "evidence_common.hpp"

#include "autonomous_navigator/NavigationCore.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <thread>

#if defined(_WIN32)
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#elif defined(__linux__)
#include <sys/resource.h>
#include <unistd.h>
#endif

namespace {

using namespace evidence;
using Clock = std::chrono::steady_clock;

long long nanosecondsBetween(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count();
}

// CPU time that other processes used while this benchmark command ran, as a fraction of the machine's
// whole CPU capacity (every logical processor for the whole wall time). Recorded in each run record so
// that a run made under background load can be recognised; it is not subtracted from any timing.
class BackgroundLoad {
public:
    BackgroundLoad() : start(sample()) {}

    // In [0, 1]; -1 when the platform does not report system CPU times.
    double fraction() const {
        const Sample end = sample();
        const double wall = end.wallSeconds - start.wallSeconds;
        if (!start.valid || !end.valid || wall <= 0.0 || logicalProcessors() <= 0) {
            return -1.0;
        }
        const double others = (end.systemBusySeconds - start.systemBusySeconds) - (end.ownSeconds - start.ownSeconds);
        return std::clamp(others / (wall * logicalProcessors()), 0.0, 1.0);
    }

    static int logicalProcessors() {
#if defined(_WIN32)
        return static_cast<int>(GetActiveProcessorCount(ALL_PROCESSOR_GROUPS));
#else
        return static_cast<int>(std::thread::hardware_concurrency());
#endif
    }

private:
    struct Sample {
        bool valid = false;
        double wallSeconds = 0.0;
        double systemBusySeconds = 0.0;  // all processes, summed over logical processors
        double ownSeconds = 0.0;         // this process
    };

    static Sample sample() {
        Sample s;
        s.wallSeconds = std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
#if defined(_WIN32)
        FILETIME idle, kernel, user, created, exited, ownKernel, ownUser;
        auto seconds = [](const FILETIME& t) {
            return static_cast<double>((static_cast<unsigned long long>(t.dwHighDateTime) << 32) | t.dwLowDateTime) * 1e-7;
        };
        if (GetSystemTimes(&idle, &kernel, &user) &&
            GetProcessTimes(GetCurrentProcess(), &created, &exited, &ownKernel, &ownUser)) {
            s.systemBusySeconds = seconds(kernel) + seconds(user) - seconds(idle);  // kernel time includes idle
            s.ownSeconds = seconds(ownKernel) + seconds(ownUser);
            s.valid = true;
        }
#elif defined(__linux__)
        std::ifstream stat("/proc/stat");
        std::string cpu;
        unsigned long long user = 0, nice = 0, system = 0, idle = 0, iowait = 0, irq = 0, softirq = 0, steal = 0;
        rusage own{};
        if (stat >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal && cpu == "cpu" &&
            getrusage(RUSAGE_SELF, &own) == 0) {
            const double tick = 1.0 / static_cast<double>(sysconf(_SC_CLK_TCK));
            s.systemBusySeconds = static_cast<double>(user + nice + system + irq + softirq + steal) * tick;
            s.ownSeconds = static_cast<double>(own.ru_utime.tv_sec + own.ru_stime.tv_sec) +
                           static_cast<double>(own.ru_utime.tv_usec + own.ru_stime.tv_usec) * 1e-6;
            s.valid = true;
        }
#endif
        return s;
    }

    Sample start;
};

const BackgroundLoad& commandLoad() {
    static const BackgroundLoad load;  // first used at the start of main
    return load;
}

// Smallest step of the clock as seen by this process, in nanoseconds.
long long clockStepNs() {
    long long best = std::numeric_limits<long long>::max();
    for (int k = 0; k < 200; ++k) {
        const auto a = Clock::now();
        auto b = Clock::now();
        while (b == a) {
            b = Clock::now();
        }
        best = std::min(best, nanosecondsBetween(a, b));
    }
    return best;
}

// Pins this thread to one logical processor of the fastest core class (Windows hybrid CPUs mix
// performance and efficiency cores) and raises its priority. Returns a description for the record.
std::string pinToFastCore() {
#if defined(_WIN32)
    DWORD length = 0;
    GetLogicalProcessorInformationEx(RelationProcessorCore, nullptr, &length);
    std::vector<unsigned char> buffer(length);
    auto* info = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data());
    if (length == 0 || !GetLogicalProcessorInformationEx(RelationProcessorCore, info, &length)) {
        return "not pinned (GetLogicalProcessorInformationEx failed)";
    }
    int bestClass = -1, bestProcessor = -1, coreClasses = 0;
    for (DWORD offset = 0; offset < length;) {
        auto* entry = reinterpret_cast<SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX*>(buffer.data() + offset);
        const PROCESSOR_RELATIONSHIP& core = entry->Processor;
        const KAFFINITY mask = core.GroupMask[0].Mask;
        if (core.GroupMask[0].Group == 0 && mask != 0) {
            int first = 0;
            while (((mask >> first) & 1) == 0) {
                ++first;
            }
            if (core.EfficiencyClass > bestClass) {
                bestClass = core.EfficiencyClass;
                bestProcessor = first;
            }
            coreClasses = std::max(coreClasses, int(core.EfficiencyClass) + 1);
        }
        offset += entry->Size;
    }
    if (bestProcessor < 0) {
        return "not pinned (no core found)";
    }
    const bool pinned = SetThreadAffinityMask(GetCurrentThread(), KAFFINITY(1) << bestProcessor) != 0;
    const bool process = SetPriorityClass(GetCurrentProcess(), ABOVE_NORMAL_PRIORITY_CLASS) != 0;
    const bool thread = SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST) != 0;
    std::ostringstream text;
    text << (pinned ? "pinned to logical processor " : "pinning failed for logical processor ") << bestProcessor
         << " (efficiency class " << bestClass << " of " << coreClasses << " classes; higher is faster)"
         << "; process priority " << (process ? "above normal" : "unchanged") << ", thread priority "
         << (thread ? "highest" : "unchanged");
    return text.str();
#else
    return "not pinned (only implemented on Windows)";
#endif
}

std::ofstream openOutput(const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot write " + path);
    }
    return out;
}

struct Query { int start; int goal; };
struct ReplanEvent { int position; int goal; Rect block; };

std::vector<std::string> dataLines(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("cannot open " + path);
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        if (!line.empty() && line[0] != '#') {
            lines.push_back(line);
        }
    }
    return lines;
}

std::vector<Query> loadQueries(const std::string& path) {
    std::vector<Query> queries;
    for (const std::string& line : dataLines(path)) {
        std::istringstream in(line);
        Query q{};
        if (!(in >> q.start >> q.goal)) {
            throw std::runtime_error(path + ": bad query line '" + line + "'");
        }
        queries.push_back(q);
    }
    return queries;
}

std::vector<ReplanEvent> loadReplans(const std::string& path) {
    std::vector<ReplanEvent> events;
    for (const std::string& line : dataLines(path)) {
        std::istringstream in(line);
        ReplanEvent e{};
        if (!(in >> e.position >> e.goal >> e.block.x0 >> e.block.y0 >> e.block.x1 >> e.block.y1)) {
            throw std::runtime_error(path + ": bad replan line '" + line + "'");
        }
        events.push_back(e);
    }
    return events;
}

bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) <= 1e-9 * std::max({1.0, std::fabs(a), std::fabs(b)});
}

// Builds the graph with the library's console logging silenced and times the construction.
struct Loaded {
    Scenario scenario;
    std::unique_ptr<Graph> graph;
    long long buildNs;
};

Loaded loadWorkload(const std::string& scenarioPath) {
    Loaded loaded{loadScenario(scenarioPath), std::make_unique<Graph>(), 0};
    const auto t0 = Clock::now();
    buildGraph(loaded.scenario, *loaded.graph);
    loaded.buildNs = nanosecondsBetween(t0, Clock::now());
    return loaded;
}

void writeRunRecord(const std::string& path, const std::string& command, const std::string& workload,
                    const Graph& graph, long long buildNs, int repeats, int warmup, const std::string& pinning,
                    long long clockStep, size_t samples, size_t invalid,
                    const std::function<void(JsonWriter&)>& extra) {
    std::ofstream out = openOutput(path);
    JsonWriter j(out);
    j.beginObject();
    j.key("schema").value("ane-evidence/bench-run/1");
    j.key("command").value(command).key("workload").value(workload);
    j.key("compiler").value(compilerDescription());
    j.key("timer").value("std::chrono::steady_clock, nanoseconds; only the planner call is between the two reads");
    j.key("clock_step_ns").value(clockStep);
    j.key("logging").value("std::cout has no stream buffer for the whole run, so library prints return after a "
                           "state check; BFS::findPath still executes its per-node print statements");
    j.key("observers").value("none attached (measurement mode)");
    j.key("pinning").value(pinning);
    j.key("repeats").value(repeats).key("warmup").value(warmup);
    j.key("graph_nodes").value(graph.getNodeCount()).key("graph_edges").value(graph.getEdgeCount());
    j.key("graph_fingerprint").value(graphFingerprint(graph));
    j.key("graph_build_ns").value(buildNs);
    j.key("samples").value(samples).key("invalid_routes").value(invalid);
    j.key("logical_processors").value(BackgroundLoad::logicalProcessors());
    j.key("background_cpu_load").value(commandLoad().fraction());
    j.key("background_cpu_load_definition")
        .value("CPU time of all other processes during this command / (wall time x logical processors); -1 if "
               "the platform does not report it");
    extra(j);
    j.endObject();
}

// ---------------------------------------------------------------------------------------------
// queries: four planners on every saved start/goal pair, interleaved per query.

int runQueries(const std::string& dir, const std::string& name, const std::string& outDir, int repeats, int warmup) {
    const std::string pinning = pinToFastCore();
    const long long step = clockStepNs();
    CoutSilencer quiet;
    Loaded w = loadWorkload(dir + "/" + name + ".txt");
    const Graph& g = *w.graph;
    const std::vector<Query> queries = loadQueries(dir + "/" + name + ".queries.txt");

    // A*'s first query on a new object includes one pass over every edge to find the heuristic's
    // scale (cached per graph revision afterwards). Measured on fresh objects, apart from the queries.
    std::vector<long long> firstQueryNs, cachedQueryNs;
    for (int k = 0; k < 21; ++k) {
        AStar fresh(&g);
        const auto t0 = Clock::now();
        const std::vector<int> a = fresh.findPath(queries[0].start, queries[0].goal);
        const auto t1 = Clock::now();
        const std::vector<int> b = fresh.findPath(queries[0].start, queries[0].goal);
        const auto t2 = Clock::now();
        firstQueryNs.push_back(nanosecondsBetween(t0, t1));
        cachedQueryNs.push_back(nanosecondsBetween(t1, t2));
    }

    AStar astar(&g);
    AStar astarZero(&g);
    astarZero.setHeuristicFunction([](int, int) { return 0.0; });
    Dijkstra dijkstra(&g);
    BFS bfs(&g);
    astar.findPath(queries[0].start, queries[0].goal);      // pay the scale pass before timing
    astarZero.findPath(queries[0].start, queries[0].goal);  // (h = 0 skips it, but keep calls symmetric)

    struct Variant { const char* key; std::function<std::vector<int>(int, int)> run; };
    const std::vector<Variant> variants = {
        {"astar", [&](int s, int t) { return astar.findPath(s, t); }},
        {"astar_zero", [&](int s, int t) { return astarZero.findPath(s, t); }},
        {"dijkstra", [&](int s, int t) { return dijkstra.findShortestPath(s, t); }},
        {"bfs", [&](int s, int t) { return bfs.findPath(s, t); }},
    };

    std::ofstream times = openOutput(outDir + "/" + name + ".query_times.csv");
    times << "query,variant,rep,ns\n";
    std::ofstream results = openOutput(outDir + "/" + name + ".query_results.csv");
    results << "query,start,goal,variant,found,valid,cost,hops,length,astar_expansions,cost_ratio_to_dijkstra\n";

    size_t samples = 0, invalid = 0;
    size_t astarMisses[2] = {0, 0};
    const size_t n = variants.size();
    for (size_t q = 0; q < queries.size(); ++q) {
        const Query query = queries[q];
        std::vector<std::vector<int>> routes(n);
        std::vector<size_t> expansions(n, 0);
        for (int rep = -warmup; rep < repeats; ++rep) {
            for (size_t k = 0; k < n; ++k) {
                const size_t v = (k + q + static_cast<size_t>(rep + warmup)) % n;  // rotate the order
                const auto t0 = Clock::now();
                std::vector<int> route = variants[v].run(query.start, query.goal);
                const auto t1 = Clock::now();
                if (rep >= 0) {
                    times << q << "," << variants[v].key << "," << rep << "," << nanosecondsBetween(t0, t1) << "\n";
                    ++samples;
                }
                if (rep == -warmup) {
                    routes[v] = std::move(route);
                    if (v == 0) expansions[v] = astar.getLastSearchNodesExplored();
                    if (v == 1) expansions[v] = astarZero.getLastSearchNodesExplored();
                }
            }
        }
        const double optimum = routes[2].empty() ? std::numeric_limits<double>::infinity() : routeCost(g, routes[2]);
        for (size_t v = 0; v < n; ++v) {
            const std::vector<int>& r = routes[v];
            const bool found = !r.empty();
            const bool valid = found && r.front() == query.start && r.back() == query.goal && routeFollowsEdges(g, r);
            if (found && !valid) {
                ++invalid;
            }
            const double cost = found ? routeCost(g, r) : 0.0;
            results << q << "," << query.start << "," << query.goal << "," << variants[v].key << "," << found << ","
                    << valid << "," << formatDouble(cost) << "," << (found ? r.size() - 1 : 0) << ","
                    << formatDouble(found ? routeLength(g, r) : 0.0) << ",";
            if (v <= 1) {
                results << expansions[v];
            }
            results << "," << (found && std::isfinite(optimum) && optimum > 0 ? formatDouble(cost / optimum) : "") << "\n";
            // A* should match Dijkstra's optimum. Misses are counted here, left visible in the results
            // file, and must be zero for check_outputs.py to pass (docs/TECHNICAL_NOTES.md, tie bug).
            if (v <= 1 && std::isfinite(optimum) && (!found || !nearlyEqual(cost, optimum))) {
                ++astarMisses[v];
            }
        }
        if (q % 100 == 0) {
            std::cerr << name << ": query " << q << "/" << queries.size() << "\n";
        }
    }

    writeRunRecord(outDir + "/" + name + ".query_run.json", "queries", name, g, w.buildNs, repeats, warmup, pinning,
                   step, samples, invalid, [&](JsonWriter& j) {
                       j.key("queries").value(queries.size());
                       j.key("astar_not_optimal").value(astarMisses[0]);
                       j.key("astar_zero_not_optimal").value(astarMisses[1]);
                       j.key("variants").beginArray(true);
                       for (const Variant& v : variants) j.value(v.key);
                       j.endArray();
                       j.key("order").value("per query and repetition the four variants run back to back, in an "
                                            "order rotated by query and repetition; one solver object per variant "
                                            "is reused for every query");
                       j.key("astar_first_query_ns").beginArray(true);
                       for (long long t : firstQueryNs) j.value(t);
                       j.endArray();
                       j.key("astar_cached_query_ns").beginArray(true);
                       for (long long t : cachedQueryNs) j.value(t);
                       j.endArray();
                       j.key("astar_first_query_note").value("fresh AStar objects, query 0: the first call includes "
                                                             "the O(E) heuristic-scale pass, the second reuses it");
                   });
    std::cerr << name << ": " << samples << " timed calls, " << invalid << " invalid routes, A* not optimal on "
              << astarMisses[0] << " (h = 0: " << astarMisses[1] << ") of " << queries.size() << " queries\n";
    return invalid == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// replans: NavigationCore::handleObstacles end to end, plus the operations it performs, timed apart.

int runReplans(const std::string& dir, const std::string& name, const std::string& outDir, int repeats) {
    const std::string pinning = pinToFastCore();
    const long long step = clockStepNs();
    CoutSilencer quiet;
    Loaded w = loadWorkload(dir + "/" + name + ".txt");
    const Graph& g = *w.graph;
    const std::vector<ReplanEvent> events = loadReplans(dir + "/" + name + ".replans.txt");

    std::ofstream times = openOutput(outDir + "/" + name + ".replan_times.csv");
    times << "event,phase,rep,ns\n";
    std::ofstream results = openOutput(outDir + "/" + name + ".replan_results.csv");
    results << "event,position,goal,blocked_nodes,on_route,replanned,found,valid,avoids_blocked,new_cost,optimum,"
               "copy_nodes,copy_edges\n";

    size_t samples = 0, invalid = 0, notOptimal = 0;
    for (size_t e = 0; e < events.size(); ++e) {
        const ReplanEvent& ev = events[e];
        std::vector<int> blocked = nodesInRect(g, ev.block);
        blocked.erase(std::remove_if(blocked.begin(), blocked.end(),
                                     [&](int id) { return id == ev.position || id == ev.goal; }),
                      blocked.end());

        // End to end: a navigator that has just planned position -> goal is told about the blockage.
        std::vector<int> before, after;
        for (int rep = -1; rep < repeats; ++rep) {
            NavigationCore nav(&g);
            nav.startNavigation(ev.position, ev.goal);
            before = nav.getCurrentPath();
            const auto t0 = Clock::now();
            nav.handleObstacles(blocked);
            const auto t1 = Clock::now();
            after = nav.getCurrentPath();
            if (rep >= 0) {
                times << e << ",end_to_end," << rep << "," << nanosecondsBetween(t0, t1) << "\n";
                ++samples;
            }
        }

        // The same work, one operation at a time: copy the graph, remove the blocked nodes, then a
        // first A* query on the copy (constructor, heuristic-scale pass, search) and a second one
        // (search only, scale cached).
        size_t copyNodes = 0, copyEdges = 0;
        std::vector<int> direct;
        for (int rep = -1; rep < repeats; ++rep) {
            const auto t0 = Clock::now();
            Graph reduced(g);
            const auto t1 = Clock::now();
            for (int id : blocked) {
                reduced.removeNode(id);
            }
            const auto t2 = Clock::now();
            AStar search(&reduced);
            direct = search.findPath(ev.position, ev.goal);
            const auto t3 = Clock::now();
            const std::vector<int> again = search.findPath(ev.position, ev.goal);
            const auto t4 = Clock::now();
            copyNodes = reduced.getNodeCount();
            copyEdges = reduced.getEdgeCount();
            if (rep >= 0) {
                times << e << ",copy," << rep << "," << nanosecondsBetween(t0, t1) << "\n";
                times << e << ",remove," << rep << "," << nanosecondsBetween(t1, t2) << "\n";
                times << e << ",search_first," << rep << "," << nanosecondsBetween(t2, t3) << "\n";
                times << e << ",search_cached," << rep << "," << nanosecondsBetween(t3, t4) << "\n";
                samples += 4;
            }
        }

        const bool onRoute = !routeAvoids(before, blocked, 1);
        const bool replanned = after != before;
        const bool found = !after.empty();
        const bool valid = found && after.front() == ev.position && after.back() == ev.goal && routeFollowsEdges(g, after);
        const bool avoids = routeAvoids(after, blocked);
        const Graph reduced = withoutNodes(g, blocked);
        double optimum = std::numeric_limits<double>::infinity();
        {
            Dijkstra d(&reduced);
            optimum = d.getShortestDistance(ev.position, ev.goal);
        }
        const double newCost = found ? routeCost(g, after) : 0.0;
        if (found && (!valid || !avoids)) {
            ++invalid;  // a route that leaves the graph or enters a blocked node
        }
        if (onRoute && std::isfinite(optimum) && (!found || !nearlyEqual(newCost, optimum))) {
            ++notOptimal;  // counted, not fatal here; check_outputs.py requires zero
        }
        if (found != !direct.empty() || (found && !nearlyEqual(routeCost(g, direct), newCost))) {
            ++invalid;  // the operation-by-operation sequence must reproduce the navigator's result
        }
        results << e << "," << ev.position << "," << ev.goal << "," << blocked.size() << "," << onRoute << ","
                << replanned << "," << found << "," << valid << "," << avoids << "," << formatDouble(newCost) << ","
                << formatDouble(optimum) << "," << copyNodes << "," << copyEdges << "\n";
        if (e % 50 == 0) {
            std::cerr << name << ": replan event " << e << "/" << events.size() << "\n";
        }
    }
    writeRunRecord(outDir + "/" + name + ".replan_run.json", "replans", name, g, w.buildNs, repeats, 1, pinning, step,
                   samples, invalid, [&](JsonWriter& j) {
                       j.key("events").value(events.size());
                       j.key("replans_not_optimal").value(notOptimal);
                       j.key("phases").value("end_to_end = NavigationCore::handleObstacles; copy = Graph copy "
                                             "constructor; remove = Graph::removeNode per blocked node; "
                                             "search_first = AStar constructor + first findPath on the copy; "
                                             "search_cached = second findPath on the same AStar");
                   });
    std::cerr << name << ": " << samples << " timed operations, " << invalid << " invalid, " << notOptimal
              << " replans not optimal of " << events.size() << "\n";
    return invalid == 0 ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// hero: the README scenario's initial plan and its replan, repeated on fresh navigators.

int runHero(const std::string& scenarioPath, const std::string& outDir, int repeats) {
    const std::string pinning = pinToFastCore();
    const long long step = clockStepNs();
    CoutSilencer quiet;
    Loaded w = loadWorkload(scenarioPath);
    const Scenario& s = w.scenario;
    const Graph& g = *w.graph;
    const int start = s.latticeId(s.startX, s.startY), goal = s.latticeId(s.goalX, s.goalY);
    std::vector<int> blocked;
    for (const Block& b : s.blocks) {
        for (int id : nodesInRect(g, b.rect)) blocked.push_back(id);
    }

    std::ofstream times = openOutput(outDir + "/hero.times.csv");
    times << "phase,rep,ns\n";
    size_t samples = 0, invalid = 0;
    std::vector<int> lastRoute;
    for (int rep = -5; rep < repeats; ++rep) {
        NavigationCore nav(&g);
        const auto t0 = Clock::now();
        nav.startNavigation(start, goal);
        const auto t1 = Clock::now();
        for (int k = 0; k < s.blockStep; ++k) {
            nav.executeNavigationStep();
        }
        const int position = nav.getCurrentPosition();
        const auto t2 = Clock::now();
        nav.handleObstacles(blocked);
        const auto t3 = Clock::now();
        lastRoute = nav.getCurrentPath();

        // The replan's operations on their own, from the same node.
        std::vector<int> removed = blocked;
        removed.erase(std::remove(removed.begin(), removed.end(), position), removed.end());
        const auto t4 = Clock::now();
        Graph reduced(g);
        const auto t5 = Clock::now();
        for (int id : removed) reduced.removeNode(id);
        const auto t6 = Clock::now();
        AStar search(&reduced);
        const std::vector<int> direct = search.findPath(position, goal);
        const auto t7 = Clock::now();
        search.findPath(position, goal);
        const auto t8 = Clock::now();
        if (rep >= 0) {
            times << "plan," << rep << "," << nanosecondsBetween(t0, t1) << "\n";
            times << "replan," << rep << "," << nanosecondsBetween(t2, t3) << "\n";
            times << "copy," << rep << "," << nanosecondsBetween(t4, t5) << "\n";
            times << "remove," << rep << "," << nanosecondsBetween(t5, t6) << "\n";
            times << "search_first," << rep << "," << nanosecondsBetween(t6, t7) << "\n";
            times << "search_cached," << rep << "," << nanosecondsBetween(t7, t8) << "\n";
            samples += 6;
            const std::vector<int> remainder(lastRoute.begin() + s.blockStep, lastRoute.end());
            if (remainder != direct || !routeAvoids(lastRoute, blocked, static_cast<size_t>(s.blockStep) + 1)) {
                ++invalid;
            }
        }
    }
    writeRunRecord(outDir + "/hero.run.json", "hero", s.name, g, w.buildNs, repeats, 5, pinning, step, samples,
                   invalid, [&](JsonWriter& j) {
                       j.key("phases").value("plan = NavigationCore::startNavigation on a new navigator; replan = "
                                             "NavigationCore::handleObstacles after block_step steps; copy, remove, "
                                             "search_first, search_cached = the replan's operations timed apart");
                       j.key("blocked_nodes").value(blocked.size());
                       j.key("final_route_hops").value(lastRoute.empty() ? 0 : lastRoute.size() - 1);
                   });
    std::cerr << "hero: " << samples << " timed operations, " << invalid << " invalid\n";
    return invalid == 0 ? 0 : 1;
}

int usage() {
    std::cerr << "usage: evidence_bench queries <workload-dir> <name> <out-dir> [repeats=5] [warmup=1]\n"
                 "       evidence_bench replans <workload-dir> <name> <out-dir> [repeats=5]\n"
                 "       evidence_bench hero <scenario.txt> <out-dir> [repeats=1000]\n";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        return usage();
    }
    const std::string command = argv[1];
    commandLoad();  // starts the background-load measurement for this command
    try {
        if (command == "queries" && argc >= 5) {
            return runQueries(argv[2], argv[3], argv[4], argc > 5 ? std::atoi(argv[5]) : 5, argc > 6 ? std::atoi(argv[6]) : 1);
        }
        if (command == "replans" && argc >= 5) {
            return runReplans(argv[2], argv[3], argv[4], argc > 5 ? std::atoi(argv[5]) : 5);
        }
        if (command == "hero") {
            return runHero(argv[2], argv[3], argc > 4 ? std::atoi(argv[4]) : 1000);
        }
    } catch (const std::exception& e) {
        std::cerr << "evidence_bench: " << e.what() << "\n";
        return 2;
    }
    return usage();
}
