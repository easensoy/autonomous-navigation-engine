// evidence_export: runs the library on the saved scenarios and writes what it did as JSON for the
// README figures, checking every result on the way. This is trace mode: search observers are
// attached here and nothing is timed (timing lives in evidence_bench, which attaches none).
//
//   evidence_export hero      <scenario.txt> <out.json>   navigator story with a mid-route blockage
//   evidence_export search    <scenario.txt> <out.json>   Dijkstra, A* (h = 0), A*, BFS on the replan query
//   evidence_export failure   <scenario.txt> <out.json>   a blockage that leaves no route
//   evidence_export workloads <out-dir>                   benchmark graphs, queries and replan events
//
// Exit status 0 when every check passes, 1 when any fails, 2 on bad usage or input.

#include "evidence_common.hpp"

#include "autonomous_navigator/NavigationCore.hpp"
#include "navigation_strategies/DynamicReplanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/SearchObserver.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace evidence;

struct Check {
    std::string name;
    bool pass;
    std::string detail;
};

class Checks {
public:
    bool add(const std::string& name, bool pass, const std::string& detail = "") {
        items.push_back({name, pass, detail});
        return pass;
    }
    bool allPassed() const {
        return std::all_of(items.begin(), items.end(), [](const Check& c) { return c.pass; });
    }
    void write(JsonWriter& j) const {
        j.key("checks").beginArray();
        for (const Check& c : items) {
            j.beginObject().key("name").value(c.name).key("pass").value(c.pass);
            if (!c.detail.empty()) {
                j.key("detail").value(c.detail);
            }
            j.endObject();
        }
        j.endArray();
    }
    void report(const std::string& what) const {
        size_t passed = 0;
        for (const Check& c : items) {
            if (c.pass) {
                ++passed;
            } else {
                std::cerr << "  FAIL " << c.name << (c.detail.empty() ? "" : ": " + c.detail) << "\n";
            }
        }
        std::cerr << what << ": " << passed << "/" << items.size() << " checks passed\n";
    }
private:
    std::vector<Check> items;
};

// Records a search's events in the order the algorithm executes them.
class Recorder : public SearchObserver {
public:
    struct Expand { int node; double cost; double priority; };
    struct Push { int node; int parent; double cost; double priority; };
    std::vector<Expand> expands;
    std::vector<Push> pushes;

    void onExpand(int nodeId, double costSoFar, double priority) override {
        expands.push_back({nodeId, costSoFar, priority});
    }
    void onFrontierPush(int nodeId, int parentId, double costSoFar, double priority) override {
        pushes.push_back({nodeId, parentId, costSoFar, priority});
    }
};

bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) <= 1e-9 * std::max({1.0, std::fabs(a), std::fabs(b)});
}

std::string describe(double value) { return formatDouble(value); }

size_t hops(const std::vector<int>& route) { return route.empty() ? 0 : route.size() - 1; }

int nodeAt(const Scenario& s, const Graph& g, int x, int y, const std::string& what) {
    const int id = s.latticeId(x, y);
    if (!s.inBounds(x, y) || !g.hasNode(id)) {
        throw std::runtime_error(what + " (" + std::to_string(x) + ", " + std::to_string(y) + ") is not a graph node");
    }
    return id;
}

std::vector<int> blockedNodes(const Scenario& s, const Graph& g) {
    std::vector<int> blocked;
    for (const Block& block : s.blocks) {
        for (int id : nodesInRect(g, block.rect)) {
            blocked.push_back(id);
        }
    }
    std::sort(blocked.begin(), blocked.end());
    blocked.erase(std::unique(blocked.begin(), blocked.end()), blocked.end());
    return blocked;
}

std::vector<int> without(std::vector<int> ids, int keep) {
    ids.erase(std::remove(ids.begin(), ids.end(), keep), ids.end());
    return ids;
}

Graph buildScenarioGraph(const Scenario& s) {
    Graph g;
    CoutSilencer quiet;
    buildGraph(s, g);
    return g;
}

// The graph with every edge weight set to 1: Dijkstra on it gives the fewest-edge route length,
// an independent check on BFS.
Graph unitWeightCopy(const Graph& g) {
    Graph unit;
    CoutSilencer quiet;
    for (int id : sortedNodeIds(g)) {
        const Node& n = g.getNode(id);
        unit.addNode(id, n.getName(), n.getX(), n.getY());
    }
    for (const EdgeRecord& e : undirectedEdges(g)) {
        unit.addEdge(e.from, e.to, 1.0, true);
    }
    return unit;
}

// Optimal cost by the library's Dijkstra, or infinity when unreachable.
double dijkstraCost(const Graph& g, int from, int to) {
    if (!g.hasNode(from) || !g.hasNode(to)) {
        return std::numeric_limits<double>::infinity();
    }
    Dijkstra d(&g);
    return d.getShortestDistance(from, to);
}

void writeScene(JsonWriter& j, const Scenario& s, const Graph& g) {
    j.key("scenario").beginObject();
    j.key("name").value(s.name).key("title").value(s.title);
    j.key("lattice").beginArray(true).value(s.xmin).value(s.ymin).value(s.xmax).value(s.ymax).endArray();
    j.key("connectivity").value(s.connectivity);
    j.key("shelves").beginArray();
    for (const Shelf& shelf : s.shelves) {
        j.beginArray(true).value(shelf.rect.x0).value(shelf.rect.y0).value(shelf.rect.x1).value(shelf.rect.y1)
            .value(shelf.label).endArray();
    }
    j.endArray();
    j.key("zones").beginArray();
    for (const Zone& zone : s.zones) {
        j.beginArray(true).value(zone.rect.x0).value(zone.rect.y0).value(zone.rect.x1).value(zone.rect.y1)
            .value(zone.multiplier).value(zone.label).endArray();
    }
    j.endArray();
    j.key("blocks").beginArray();
    for (const Block& block : s.blocks) {
        j.beginArray(true).value(block.rect.x0).value(block.rect.y0).value(block.rect.x1).value(block.rect.y1)
            .value(block.label).endArray();
    }
    j.endArray();
    j.endObject();

    j.key("graph").beginObject();
    j.key("node_count").value(g.getNodeCount()).key("edge_count").value(g.getEdgeCount());
    j.key("fingerprint").value(graphFingerprint(g));
    j.key("nodes").beginArray();
    for (int id : sortedNodeIds(g)) {
        const Node& n = g.getNode(id);
        j.beginArray(true).value(id).value(n.getX()).value(n.getY()).endArray();
    }
    j.endArray();
    j.key("edges").beginArray();
    for (const EdgeRecord& e : undirectedEdges(g)) {
        j.beginArray(true).value(e.from).value(e.to).value(e.weight).endArray();
    }
    j.endArray();
    j.endObject();
}

void writeRoute(JsonWriter& j, const Graph& g, const std::vector<int>& route) {
    j.beginObject();
    j.key("nodes").ints(route);
    j.key("cost").value(route.empty() ? 0.0 : routeCost(g, route));
    j.key("length").value(route.empty() ? 0.0 : routeLength(g, route));
    j.key("hops").value(hops(route));
    j.endObject();
}

void writeHeader(JsonWriter& j, const std::string& schema, const std::string& scenarioPath) {
    j.key("schema").value(schema);
    j.key("scenario_file").value(scenarioPath);
    j.key("compiler").value(compilerDescription());
    j.key("units").beginObject()
        .key("coordinates").value("raw coordinate units (Node x/y); Node::euclideanDistance is not used")
        .key("cost").value("edge-weight units: straight-line length x zone multiplier")
        .key("steps").value("simulated navigation steps: one graph node per NavigationCore::executeNavigationStep")
        .endObject();
}

std::ofstream openOutput(const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot write " + path);
    }
    return out;
}

// ---------------------------------------------------------------------------------------------
// hero: plan, step, blockage, replan, arrive.

int runHero(const std::string& scenarioPath, const std::string& outPath) {
    const Scenario s = loadScenario(scenarioPath);
    const Graph g = buildScenarioGraph(s);
    Checks checks;
    const int start = nodeAt(s, g, s.startX, s.startY, "start");
    const int goal = nodeAt(s, g, s.goalX, s.goalY, "goal");
    const std::vector<int> blocked = blockedNodes(s, g);
    checks.add("blockage covers at least one node", !blocked.empty(), std::to_string(blocked.size()) + " nodes");
    checks.add("start and goal are not blocked",
               std::find(blocked.begin(), blocked.end(), start) == blocked.end() &&
                   std::find(blocked.begin(), blocked.end(), goal) == blocked.end());

    NavigationCore nav(&g);
    bool started;
    {
        CoutSilencer quiet;
        started = nav.startNavigation(start, goal);
    }
    checks.add("navigation starts", started);
    const std::vector<int> initial = nav.getCurrentPath();
    const double initialOptimum = dijkstraCost(g, start, goal);
    checks.add("initial route runs start -> goal", !initial.empty() && initial.front() == start && initial.back() == goal);
    checks.add("initial route follows graph edges", routeFollowsEdges(g, initial));
    checks.add("initial route cost equals Dijkstra's optimum", nearlyEqual(routeCost(g, initial), initialOptimum),
               describe(routeCost(g, initial)) + " vs " + describe(initialOptimum));
    const std::string statusAtStart = nav.getNavigationStatus();

    struct StepRecord {
        int step;
        int node;
        int routeVersion;
        double remainingReported;  // NavigationCore::getRemainingDistance(), which sums edge weights
        double remainingComputed;
    };
    std::vector<StepRecord> timeline;
    std::vector<std::vector<int>> routes{initial};
    std::vector<int> plannedAtStep{0};
    size_t index = 0;
    bool indexConsistent = true, remainingConsistent = true;
    auto record = [&](int step) {
        const std::vector<int> path = nav.getCurrentPath();
        indexConsistent = indexConsistent && index < path.size() && path[index] == nav.getCurrentPosition();
        const std::vector<int> remaining(path.begin() + static_cast<long>(std::min(index, path.size())), path.end());
        const double computed = routeCost(g, remaining);
        const double reported = nav.getRemainingDistance();
        remainingConsistent = remainingConsistent && nearlyEqual(computed, reported);
        timeline.push_back({step, nav.getCurrentPosition(), static_cast<int>(routes.size()) - 1, reported, computed});
    };
    record(0);

    int step = 0;
    int blockPosition = -1;
    size_t blockIndex = 0;
    std::vector<int> routeBefore, routeAfter;
    std::string statusAtBlock;
    while (step <= 100000) {
        if (step == s.blockStep && !blocked.empty() && blockPosition < 0) {
            routeBefore = nav.getCurrentPath();
            blockPosition = nav.getCurrentPosition();
            blockIndex = index;
            {
                CoutSilencer quiet;
                nav.handleObstacles(blocked);
            }
            routeAfter = nav.getCurrentPath();
            statusAtBlock = nav.getNavigationStatus();
            if (routeAfter != routeBefore) {
                routes.push_back(routeAfter);
                plannedAtStep.push_back(step);
            }
        }
        bool moved;
        {
            CoutSilencer quiet;
            moved = nav.executeNavigationStep();
        }
        if (!moved) {
            break;
        }
        ++step;
        ++index;
        record(step);
    }
    const std::string statusAtEnd = nav.getNavigationStatus();

    std::vector<int> traversed;
    for (const StepRecord& r : timeline) {
        traversed.push_back(r.node);
    }
    checks.add("the blockage was reported at the scenario's step", blockPosition >= 0,
               "block_step " + std::to_string(s.blockStep));
    checks.add("the blockage lay on the route ahead", !routeAvoids(routeBefore, blocked, blockIndex + 1));
    checks.add("navigator replanned (route changed)", routes.size() == 2);
    checks.add("progress index matches the current position at every step", indexConsistent);
    checks.add("getRemainingDistance equals the summed edge weights ahead, every step", remainingConsistent);
    checks.add("navigation completed at the goal", nav.isNavigationComplete() && nav.getCurrentPosition() == goal);
    checks.add("traversed route follows graph edges", routeFollowsEdges(g, traversed));
    checks.add("no blocked node entered after the blockage", routeAvoids(traversed, blocked, blockIndex + 1));
    checks.add("travelled part kept unchanged by the replan",
               routeAfter.size() > blockIndex &&
                   std::equal(routeBefore.begin(), routeBefore.begin() + static_cast<long>(blockIndex) + 1,
                              routeAfter.begin()));
    const std::vector<int> replanned(routeAfter.begin() + static_cast<long>(std::min(blockIndex, routeAfter.size())),
                                     routeAfter.end());
    const Graph reduced = withoutNodes(g, without(blocked, blockPosition));
    const double replanOptimum = dijkstraCost(reduced, blockPosition, goal);
    checks.add("replanned remainder starts at the current node", !replanned.empty() && replanned.front() == blockPosition);
    checks.add("replanned remainder avoids every blocked node", routeAvoids(replanned, blocked));
    checks.add("replanned remainder follows edges of the reduced graph", routeFollowsEdges(reduced, replanned));
    checks.add("replanned remainder cost equals Dijkstra's optimum on the reduced graph",
               nearlyEqual(routeCost(g, replanned), replanOptimum),
               describe(routeCost(g, replanned)) + " vs " + describe(replanOptimum));
    checks.add("traversed route equals the final active route", traversed == nav.getCurrentPath());

    std::vector<int> abandoned(routeBefore.begin() + static_cast<long>(std::min(blockIndex, routeBefore.size())),
                               routeBefore.end());

    std::ofstream out = openOutput(outPath);
    JsonWriter j(out);
    j.beginObject();
    writeHeader(j, "ane-evidence/hero/1", scenarioPath);
    j.key("engine_path").value(
        "NavigationCore::startNavigation -> DynamicReplanner::updateAndReplan -> GlobalPathPlanner (auto: A*); "
        "NavigationCore::handleObstacles -> DynamicReplanner::updateAndReplan -> copy graph, remove blocked nodes, "
        "AStar::findPath from the current node");
    writeScene(j, s, g);
    j.key("start").value(start).key("goal").value(goal);
    j.key("blocked_nodes").ints(blocked);
    j.key("block_step").value(s.blockStep);
    j.key("routes").beginArray();
    for (size_t v = 0; v < routes.size(); ++v) {
        j.beginObject().key("version").value(static_cast<int>(v)).key("planned_at_step").value(plannedAtStep[v]);
        j.key("route");
        writeRoute(j, g, routes[v]);
        j.endObject();
    }
    j.endArray();
    j.key("replan").beginObject();
    j.key("step").value(s.blockStep).key("position").value(blockPosition).key("route_index").value(blockIndex);
    j.key("abandoned_remainder");
    writeRoute(j, g, abandoned);
    j.key("new_remainder");
    writeRoute(j, g, replanned);
    j.key("dijkstra_optimum_on_reduced_graph").value(replanOptimum);
    j.key("status_after").value(statusAtBlock);
    j.endObject();
    j.key("timeline").beginArray();
    for (const StepRecord& r : timeline) {
        j.beginArray(true).value(r.step).value(r.node).value(r.routeVersion).value(r.remainingReported).endArray();
    }
    j.endArray();
    j.key("timeline_columns").beginArray(true).value("step").value("node").value("route_version")
        .value("remaining_cost").endArray();
    j.key("traversed");
    writeRoute(j, g, traversed);
    j.key("initial_optimum").value(initialOptimum);
    j.key("status").beginObject().key("start").value(statusAtStart).key("end").value(statusAtEnd).endObject();
    checks.write(j);
    j.endObject();

    checks.report("hero " + s.name);
    return checks.allPassed() ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// search: four planners on the replanner's own query.

struct VariantResult {
    std::string key;
    std::string label;
    std::vector<int> route;
    Recorder events;
    bool sameAsUntraced = false;
    size_t ownCounter = 0;  // A*'s getLastSearchNodesExplored(), when available
    bool hasOwnCounter = false;
};

bool nondecreasing(const std::vector<Recorder::Expand>& expands) {
    for (size_t i = 1; i < expands.size(); ++i) {
        if (expands[i].priority < expands[i - 1].priority - 1e-9 * std::max(1.0, std::fabs(expands[i - 1].priority))) {
            return false;
        }
    }
    return true;
}

// Nodes whose cheapest frontier entry had priority below the optimum but that were never expanded.
// A best-first search with no heuristic (Dijkstra, A* with h = 0) or a consistent one (A*'s default)
// expands every such node before it reaches the goal, so the list must be empty. Before the open-list
// fix, A* dropped a node whose stale and live entries tied on priority (docs/TECHNICAL_NOTES.md).
// Not meaningful for BFS, whose priority is depth.
std::vector<int> neverExpandedBelow(const Recorder& events, double optimum) {
    std::vector<Recorder::Push> sorted = events.pushes;
    std::stable_sort(sorted.begin(), sorted.end(), [](const Recorder::Push& a, const Recorder::Push& b) {
        return a.node != b.node ? a.node < b.node : a.cost < b.cost;
    });
    std::vector<int> expanded;
    for (const auto& e : events.expands) expanded.push_back(e.node);
    std::sort(expanded.begin(), expanded.end());
    std::vector<int> dropped;
    for (size_t k = 0; k < sorted.size(); ++k) {
        if (k > 0 && sorted[k].node == sorted[k - 1].node) continue;  // first = lowest cost
        if (sorted[k].priority < optimum - 1e-9 * std::max(1.0, optimum) &&
            !std::binary_search(expanded.begin(), expanded.end(), sorted[k].node)) {
            dropped.push_back(sorted[k].node);
        }
    }
    return dropped;
}

int runSearch(const std::string& scenarioPath, const std::string& outPath) {
    const Scenario s = loadScenario(scenarioPath);
    const Graph g = buildScenarioGraph(s);
    Checks checks;
    const int start = nodeAt(s, g, s.startX, s.startY, "start");
    const int goal = nodeAt(s, g, s.goalX, s.goalY, "goal");
    const std::vector<int> blocked = blockedNodes(s, g);

    // Drive the navigator to the step where the blockage is reported, exactly as the hero does.
    NavigationCore nav(&g);
    {
        CoutSilencer quiet;
        nav.startNavigation(start, goal);
        for (int k = 0; k < s.blockStep; ++k) {
            nav.executeNavigationStep();
        }
    }
    const int position = nav.getCurrentPosition();
    const std::vector<int> removed = without(blocked, position);
    const Graph reduced = withoutNodes(g, removed);
    checks.add("query start is the navigator's position at the block step", reduced.hasNode(position));

    std::vector<VariantResult> results(4);
    results[0].key = "dijkstra";
    results[0].label = "Dijkstra";
    results[1].key = "astar_zero";
    results[1].label = "A*, h = 0";
    results[2].key = "astar";
    results[2].label = "A*, default heuristic";
    results[3].key = "bfs";
    results[3].label = "BFS";
    {
        Dijkstra plain(&reduced), traced(&reduced);
        traced.setSearchObserver(&results[0].events);
        const std::vector<int> untraced = plain.findShortestPath(position, goal);
        results[0].route = traced.findShortestPath(position, goal);
        results[0].sameAsUntraced = untraced == results[0].route;
    }
    {
        auto zero = [](int, int) { return 0.0; };
        AStar plain(&reduced), traced(&reduced);
        plain.setHeuristicFunction(zero);
        traced.setHeuristicFunction(zero);
        traced.setSearchObserver(&results[1].events);
        const std::vector<int> untraced = plain.findPath(position, goal);
        results[1].route = traced.findPath(position, goal);
        results[1].sameAsUntraced = untraced == results[1].route;
        results[1].ownCounter = traced.getLastSearchNodesExplored();
        results[1].hasOwnCounter = true;
    }
    {
        AStar plain(&reduced), traced(&reduced);
        traced.setSearchObserver(&results[2].events);
        const std::vector<int> untraced = plain.findPath(position, goal);
        results[2].route = traced.findPath(position, goal);
        results[2].sameAsUntraced = untraced == results[2].route;
        results[2].ownCounter = traced.getLastSearchNodesExplored();
        results[2].hasOwnCounter = true;
    }
    {
        CoutSilencer quiet;  // BFS prints every step to std::cout
        BFS plain(&reduced), traced(&reduced);
        traced.setSearchObserver(&results[3].events);
        const std::vector<int> untraced = plain.findPath(position, goal);
        results[3].route = traced.findPath(position, goal);
        results[3].sameAsUntraced = untraced == results[3].route;
    }

    // The replanner's own route, for comparison: NavigationCore::handleObstacles on the same state.
    {
        CoutSilencer quiet;
        nav.handleObstacles(blocked);
    }
    const std::vector<int> navPath = nav.getCurrentPath();
    const std::vector<int> navRemainder(navPath.begin() + s.blockStep, navPath.end());

    const double optimum = dijkstraCost(reduced, position, goal);
    const Graph unit = unitWeightCopy(reduced);
    const double fewestHops = dijkstraCost(unit, position, goal);

    // Reference: the shortest route by distance, ignoring zone costs (the library's Dijkstra on a copy
    // whose edge weights are the segment lengths). Not a planner variant; it names the detour the
    // cost model rejects.
    std::vector<int> shortestByLength;
    {
        Graph lengths;
        CoutSilencer quiet;
        for (int id : sortedNodeIds(reduced)) {
            const Node& n = reduced.getNode(id);
            lengths.addNode(id, n.getName(), n.getX(), n.getY());
        }
        for (const EdgeRecord& e : undirectedEdges(reduced)) {
            const Node& a = reduced.getNode(e.from);
            const Node& b = reduced.getNode(e.to);
            lengths.addEdge(e.from, e.to, std::hypot(b.getX() - a.getX(), b.getY() - a.getY()), true);
        }
        Dijkstra d(&lengths);
        shortestByLength = d.findShortestPath(position, goal);
    }
    checks.add("shortest-by-length reference route follows edges", routeFollowsEdges(reduced, shortestByLength));
    for (const VariantResult& r : results) {
        checks.add(r.label + ": route runs position -> goal",
                   !r.route.empty() && r.route.front() == position && r.route.back() == goal);
        checks.add(r.label + ": route follows edges and avoids blocked nodes",
                   routeFollowsEdges(reduced, r.route) && routeAvoids(r.route, blocked));
        checks.add(r.label + ": attaching the observer leaves the route unchanged", r.sameAsUntraced);
        checks.add(r.label + ": expansions arrive in frontier order (nondecreasing priority)", nondecreasing(r.events.expands));
        if (r.hasOwnCounter) {
            checks.add(r.label + ": observer expansions equal getLastSearchNodesExplored()",
                       r.ownCounter == r.events.expands.size(),
                       std::to_string(r.events.expands.size()) + " vs " + std::to_string(r.ownCounter));
        }
    }
    for (int k = 0; k < 3; ++k) {
        checks.add(results[k].label + ": cost equals Dijkstra's optimum", nearlyEqual(routeCost(reduced, results[k].route), optimum),
                   describe(routeCost(reduced, results[k].route)) + " vs " + describe(optimum));
        const std::vector<int> dropped = neverExpandedBelow(results[k].events, optimum);
        checks.add(results[k].label + ": every node pushed below the optimum is expanded", dropped.empty(),
                   std::to_string(dropped.size()) + " never expanded");
    }
    checks.add("BFS: hop count equals the fewest edges (Dijkstra on unit weights)",
               nearlyEqual(static_cast<double>(hops(results[3].route)), fewestHops),
               std::to_string(hops(results[3].route)) + " vs " + describe(fewestHops));
    checks.add("A* route equals the navigator's replanned remainder", results[2].route == navRemainder);

    std::ofstream out = openOutput(outPath);
    JsonWriter j(out);
    j.beginObject();
    writeHeader(j, "ane-evidence/search/1", scenarioPath);
    j.key("query").value(
        "the replanner's query: from the navigator's node at block_step to the goal, on the graph with the blocked "
        "nodes removed");
    writeScene(j, s, g);
    j.key("start").value(position).key("goal").value(goal).key("block_step").value(s.blockStep);
    j.key("blocked_nodes").ints(blocked);
    j.key("dijkstra_optimum").value(optimum).key("fewest_hops").value(fewestHops);
    j.key("reference_shortest_length");
    writeRoute(j, reduced, shortestByLength);
    j.key("definitions").beginObject()
        .key("expansion").value("a node taken off the frontier and expanded: A* and Dijkstra when a node is settled "
                                "(A* counts the goal, then stops), BFS when a node is dequeued (BFS stops when it "
                                "first sees the goal, so the goal is never dequeued)")
        .key("push").value("a node put on the frontier with a new best cost (A*, Dijkstra) or first discovered (BFS)")
        .key("cost").value("sum of edge weights along the returned route")
        .key("length").value("sum of straight-line segment lengths, raw coordinate units")
        .key("hops").value("number of edges in the returned route")
        .endObject();
    j.key("variants").beginArray();
    for (const VariantResult& r : results) {
        j.beginObject();
        j.key("key").value(r.key).key("label").value(r.label);
        j.key("route");
        writeRoute(j, reduced, r.route);
        j.key("expansions").value(r.events.expands.size());
        j.key("pushes").value(r.events.pushes.size());
        j.key("never_expanded_below_optimum")
            .ints(r.key == "bfs" ? std::vector<int>{} : neverExpandedBelow(r.events, optimum));
        j.key("expand_events").beginArray();
        for (const auto& e : r.events.expands) {
            j.beginArray(true).value(e.node).value(e.cost).value(e.priority).endArray();
        }
        j.endArray();
        j.key("expand_columns").beginArray(true).value("node").value("cost").value("priority").endArray();
        j.key("push_events").beginArray();
        for (const auto& p : r.events.pushes) {
            j.beginArray(true).value(p.node).value(p.parent).value(p.cost).value(p.priority).endArray();
        }
        j.endArray();
        j.key("push_columns").beginArray(true).value("node").value("parent").value("cost").value("priority").endArray();
        j.endObject();
    }
    j.endArray();
    checks.write(j);
    j.endObject();

    checks.report("search " + s.name);
    return checks.allPassed() ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// failure: a blockage that leaves no route.

int runFailure(const std::string& scenarioPath, const std::string& outPath) {
    const Scenario s = loadScenario(scenarioPath);
    const Graph g = buildScenarioGraph(s);
    Checks checks;
    const int start = nodeAt(s, g, s.startX, s.startY, "start");
    const int goal = nodeAt(s, g, s.goalX, s.goalY, "goal");
    const std::vector<int> blocked = blockedNodes(s, g);
    const bool goalBlocked = std::find(blocked.begin(), blocked.end(), goal) != blocked.end();

    NavigationCore nav(&g);
    {
        CoutSilencer quiet;
        nav.startNavigation(start, goal);
        for (int k = 0; k < s.blockStep; ++k) {
            nav.executeNavigationStep();
        }
    }
    const int position = nav.getCurrentPosition();
    const std::vector<int> routeBefore = nav.getCurrentPath();
    const std::string statusBefore = nav.getNavigationStatus();
    const std::vector<int> remainingBefore(routeBefore.begin() + s.blockStep, routeBefore.end());
    checks.add("the blockage lies on the route ahead", !routeAvoids(remainingBefore, blocked, 1));

    // What the replanner itself returns for this state.
    std::vector<int> replannerResult;
    {
        CoutSilencer quiet;
        DynamicReplanner replanner(&g);
        replanner.setInitialPath(remainingBefore);
        replannerResult = replanner.updateAndReplan(position, goal, blocked);
    }

    // What the navigator does with it; its console message is captured, not discarded.
    std::ostringstream console;
    std::streambuf* saved = std::cout.rdbuf(console.rdbuf());
    nav.handleObstacles(blocked);
    std::cout.rdbuf(saved);
    std::string message = console.str();
    while (!message.empty() && (message.back() == '\n' || message.back() == '\r')) {
        message.pop_back();
    }
    const std::string statusAfter = nav.getNavigationStatus();
    const NavigationState stateAfter = nav.getNavigationState();
    const int positionAfter = nav.getCurrentPosition();
    const std::vector<int> routeAfter = nav.getCurrentPath();
    const std::vector<int> invalidated = nav.getInvalidatedRoute();
    const double remainingAfter = nav.getRemainingDistance();
    bool stepped;
    bool retried;
    {
        CoutSilencer quiet;
        stepped = nav.executeNavigationStep();
        retried = nav.retryNavigation();  // the blockage still stands, so this must fail
    }
    const int positionAfterStep = nav.getCurrentPosition();
    const NavigationState stateAfterRetry = nav.getNavigationState();
    const std::vector<int> travelledBefore(routeBefore.begin(), routeBefore.begin() + s.blockStep + 1);

    // Independent evidence that no route exists: BFS reachability and A* on the reduced graph.
    const Graph reduced = withoutNodes(g, without(blocked, position));
    bool reachable;
    {
        CoutSilencer quiet;
        BFS bfs(&reduced);
        reachable = bfs.isReachable(position, goal);
    }
    Recorder astarEvents;
    AStar astar(&reduced);
    astar.setSearchObserver(&astarEvents);
    const std::vector<int> astarRoute = astar.findPath(position, goal);
    size_t component = 0;
    std::vector<int> unreachable;  // nodes of the reduced graph that no route from the navigator reaches
    {
        Dijkstra d(&reduced);
        for (const auto& entry : d.findShortestDistances(position)) {
            if (std::isfinite(entry.second)) {
                ++component;
            } else {
                unreachable.push_back(entry.first);
            }
        }
        std::sort(unreachable.begin(), unreachable.end());
    }

    checks.add("DynamicReplanner returns no route", replannerResult.empty());
    checks.add("no route exists (BFS reachability on the reduced graph)", !reachable);
    checks.add("A* on the reduced graph returns no route", astarRoute.empty());
    std::vector<int> pushedNodes, expandedNodes;
    for (const auto& p : astarEvents.pushes) pushedNodes.push_back(p.node);
    for (const auto& e : astarEvents.expands) expandedNodes.push_back(e.node);
    std::sort(pushedNodes.begin(), pushedNodes.end());
    pushedNodes.erase(std::unique(pushedNodes.begin(), pushedNodes.end()), pushedNodes.end());
    std::sort(expandedNodes.begin(), expandedNodes.end());
    expandedNodes.erase(std::unique(expandedNodes.begin(), expandedNodes.end()), expandedNodes.end());
    std::vector<int> neverExpanded;
    std::set_difference(pushedNodes.begin(), pushedNodes.end(), expandedNodes.begin(), expandedNodes.end(),
                        std::back_inserter(neverExpanded));
    if (goalBlocked) {
        checks.add("goal is not in the reduced graph, so A* expands nothing", astarEvents.expands.empty());
    } else {
        // With no route, A* has to exhaust the component before it gives up. Before the open-list
        // fix it stopped short, leaving pushed nodes unexpanded (docs/TECHNICAL_NOTES.md).
        checks.add("A* expanded every node of the reachable component",
                   expandedNodes.size() == component && astarEvents.expands.size() == component &&
                       neverExpanded.empty(),
                   std::to_string(astarEvents.expands.size()) + " expansions, " + std::to_string(component) +
                       " reachable, " + std::to_string(neverExpanded.size()) + " pushed but never expanded");
    }
    checks.add("the navigator enters BLOCKED", stateAfter == NavigationState::BLOCKED &&
                                                   statusAfter.rfind("BLOCKED", 0) == 0, statusAfter);
    checks.add("the navigator stays on its node", positionAfter == position && positionAfterStep == position);
    checks.add("executeNavigationStep refuses to move", !stepped);
    checks.add("getCurrentPath returns only the travelled part, ending at the current node",
               routeAfter == travelledBefore && !routeAfter.empty() && routeAfter.back() == position);
    checks.add("getInvalidatedRoute returns the abandoned route ahead",
               invalidated == remainingBefore && !routeAvoids(invalidated, blocked));
    checks.add("getRemainingDistance is infinite", std::isinf(remainingAfter));
    checks.add("retryNavigation fails while the blockage stands",
               !retried && stateAfterRetry == NavigationState::BLOCKED);

    std::ofstream out = openOutput(outPath);
    JsonWriter j(out);
    j.beginObject();
    writeHeader(j, "ane-evidence/failure/1", scenarioPath);
    writeScene(j, s, g);
    j.key("start").value(start).key("goal").value(goal).key("block_step").value(s.blockStep);
    j.key("blocked_nodes").ints(blocked);
    j.key("goal_blocked").value(goalBlocked);
    j.key("position").value(position);
    j.key("route_before");
    writeRoute(j, g, routeBefore);
    j.key("status_before").value(statusBefore);
    j.key("replanner_route").ints(replannerResult);
    j.key("console_message").value(message);
    j.key("status_after").value(statusAfter);
    j.key("state_after").value(std::string(navigationStateName(stateAfter)));
    j.key("position_after").value(positionAfter);
    j.key("current_path_after").ints(routeAfter);
    j.key("invalidated_route").ints(invalidated);
    j.key("invalidated_route_blocked_nodes").value(invalidated.size() - static_cast<size_t>(std::count_if(
        invalidated.begin(), invalidated.end(),
        [&](int n) { return std::find(blocked.begin(), blocked.end(), n) == blocked.end(); })));
    j.key("remaining_after_is_infinite").value(std::isinf(remainingAfter));
    j.key("execute_step_after").value(stepped);
    j.key("retry_after").value(retried);
    j.key("state_after_retry").value(std::string(navigationStateName(stateAfterRetry)));
    j.key("reachable").value(reachable);
    j.key("astar_route").ints(astarRoute);
    j.key("astar_expansions").value(astarEvents.expands.size());
    j.key("astar_pushed_nodes").value(pushedNodes.size());
    j.key("astar_pushed_never_expanded").ints(neverExpanded);
    j.key("reachable_component_nodes").value(component);
    j.key("unreachable_nodes").ints(unreachable);
    j.key("reduced_graph_nodes").value(reduced.getNodeCount());
    checks.write(j);
    j.endObject();

    checks.report("failure " + s.name);
    return checks.allPassed() ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------
// workloads: benchmark graphs, query sets and replan events, all from fixed seeds.

struct WorkloadSpec {
    WarehouseParams params;
    int queries;
    int replans;
    std::uint64_t querySeed;
    std::uint64_t replanSeed;
};

int runWorkloads(const std::string& outDir) {
    std::vector<WorkloadSpec> specs;
    {
        WorkloadSpec small{{}, 1000, 300, 0x5EED0001ULL, 0x5EED0101ULL};
        small.params.name = "warehouse_s";
        small.params.blocksX = 4;
        small.params.blocksY = 5;
        small.params.seed = 11;
        specs.push_back(small);
        WorkloadSpec medium{{}, 1000, 300, 0x5EED0002ULL, 0x5EED0102ULL};
        medium.params.name = "warehouse_m";
        medium.params.blocksX = 12;
        medium.params.blocksY = 17;
        medium.params.seed = 12;
        specs.push_back(medium);
        WorkloadSpec large{{}, 500, 200, 0x5EED0003ULL, 0x5EED0103ULL};
        large.params.name = "warehouse_l";
        large.params.blocksX = 24;
        large.params.blocksY = 34;
        large.params.seed = 13;
        specs.push_back(large);
    }
    Checks checks;
    std::ofstream summary = openOutput(outDir + "/workloads.json");
    JsonWriter j(summary);
    j.beginObject();
    j.key("schema").value("ane-evidence/workloads/1");
    j.key("generator").value("evidence_export workloads (tools/evidence/evidence_export.cpp), splitmix64 seeds");
    j.key("workloads").beginArray();
    for (const WorkloadSpec& spec : specs) {
        const Scenario s = makeWarehouse(spec.params);
        saveScenario(s, outDir + "/" + s.name + ".txt");
        const Graph g = buildScenarioGraph(s);
        const std::vector<int> ids = sortedNodeIds(g);

        // Queries: distinct node pairs drawn uniformly.
        SplitMix64 qrng(spec.querySeed);
        std::ofstream q = openOutput(outDir + "/" + s.name + ".queries.txt");
        q << "# " << s.name << " queries: start_id goal_id (seed " << spec.querySeed << ", " << spec.queries << " pairs)\n";
        for (int k = 0; k < spec.queries; ++k) {
            int a = ids[qrng.below(ids.size())], b;
            do {
                b = ids[qrng.below(ids.size())];
            } while (b == a);
            q << a << " " << b << "\n";
        }

        // Replan events: a trip of at least a quarter of the warehouse diagonal; a 3 x 3 lattice square
        // is blocked around the middle node of A*'s route, and the navigator replans from the start.
        SplitMix64 rrng(spec.replanSeed);
        std::ofstream r = openOutput(outDir + "/" + s.name + ".replans.txt");
        r << "# " << s.name << " replan events: position_id goal_id block_x0 block_y0 block_x1 block_y1 (seed "
          << spec.replanSeed << ")\n";
        const double minimum = 0.25 * std::hypot(double(s.xmax - s.xmin), double(s.ymax - s.ymin));
        AStar planner(&g);
        int written = 0, attempts = 0;
        while (written < spec.replans && attempts < spec.replans * 50) {
            ++attempts;
            const int a = ids[rrng.below(ids.size())], b = ids[rrng.below(ids.size())];
            const Node& na = g.getNode(a);
            const Node& nb = g.getNode(b);
            if (std::hypot(nb.getX() - na.getX(), nb.getY() - na.getY()) < minimum) {
                continue;
            }
            const std::vector<int> route = planner.findPath(a, b);
            if (route.size() < 8) {
                continue;
            }
            const Node& mid = g.getNode(route[route.size() / 2]);
            const Rect block{mid.getX() - 1.5, mid.getY() - 1.5, mid.getX() + 1.5, mid.getY() + 1.5};
            if (block.contains(na.getX(), na.getY()) || block.contains(nb.getX(), nb.getY())) {
                continue;
            }
            r << a << " " << b << " " << formatDouble(block.x0) << " " << formatDouble(block.y0) << " "
              << formatDouble(block.x1) << " " << formatDouble(block.y1) << "\n";
            ++written;
        }
        checks.add(s.name + ": all replan events generated", written == spec.replans,
                   std::to_string(written) + " of " + std::to_string(spec.replans));

        j.beginObject();
        j.key("name").value(s.name).key("title").value(s.title);
        j.key("scenario_file").value(s.name + ".txt");
        j.key("queries_file").value(s.name + ".queries.txt");
        j.key("replans_file").value(s.name + ".replans.txt");
        j.key("lattice").beginArray(true).value(s.xmin).value(s.ymin).value(s.xmax).value(s.ymax).endArray();
        j.key("nodes").value(g.getNodeCount()).key("edges").value(g.getEdgeCount());
        j.key("picking_zones").value(s.zones.size()).key("racks").value(s.shelves.size());
        j.key("fingerprint").value(graphFingerprint(g));
        j.key("query_count").value(spec.queries).key("query_seed").value(static_cast<unsigned long long>(spec.querySeed));
        j.key("replan_count").value(written).key("replan_seed").value(static_cast<unsigned long long>(spec.replanSeed));
        j.key("layout_seed").value(static_cast<unsigned long long>(spec.params.seed));
        j.endObject();
        std::cerr << s.name << ": " << g.getNodeCount() << " nodes, " << g.getEdgeCount() << " edges, "
                  << s.zones.size() << " picking zones\n";
    }
    j.endArray();
    checks.write(j);
    j.endObject();
    checks.report("workloads");
    return checks.allPassed() ? 0 : 1;
}

int usage() {
    std::cerr << "usage: evidence_export hero|search|failure <scenario.txt> <out.json>\n"
                 "       evidence_export workloads <out-dir>\n";
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        return usage();
    }
    const std::string command = argv[1];
    try {
        if (command == "workloads") {
            return runWorkloads(argv[2]);
        }
        if (argc < 4) {
            return usage();
        }
        if (command == "hero") {
            return runHero(argv[2], argv[3]);
        }
        if (command == "search") {
            return runSearch(argv[2], argv[3]);
        }
        if (command == "failure") {
            return runFailure(argv[2], argv[3]);
        }
    } catch (const std::exception& e) {
        std::cerr << "evidence_export: " << e.what() << "\n";
        return 2;
    }
    return usage();
}
