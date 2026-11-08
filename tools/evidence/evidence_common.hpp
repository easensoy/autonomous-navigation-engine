#pragma once
// Shared code for the evidence tools (evidence_trace and evidence_bench): scenario files, building
// lattice graphs through the public Graph API, route checks, a small JSON writer and helpers.
// Nothing here plans a route; every route comes from the library's planners.

#include "core/Graph.hpp"
#include <cstdint>
#include <ostream>
#include <streambuf>
#include <string>
#include <type_traits>
#include <vector>

namespace evidence {

// Axis-aligned rectangle in raw coordinate units (the x/y stored on each Node).
struct Rect {
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    bool contains(double x, double y) const { return x >= x0 && x <= x1 && y >= y0 && y <= y1; }
};

struct Shelf { Rect rect; std::string label; };
struct Zone { Rect rect; double multiplier = 1.0; std::string label; };
struct Block { Rect rect; std::string label; };
struct NamedPoint { std::string name; int x = 0, y = 0; };

// A scenario file describes a lattice warehouse: one node per free lattice point (unit spacing),
// shelves that remove nodes and edges, cost zones, and optionally a start, goal and blockage.
// See tools/evidence/scenarios/README.md for the format.
struct Scenario {
    std::string name;
    std::string title;
    int xmin = 0, ymin = 0, xmax = 0, ymax = 0;  // inclusive lattice bounds
    int connectivity = 8;                         // 4 or 8
    std::vector<Shelf> shelves;
    std::vector<Zone> zones;
    std::vector<Block> blocks;
    std::vector<NamedPoint> points;
    int blockStep = -1;
    bool hasStart = false, hasGoal = false;
    int startX = 0, startY = 0, goalX = 0, goalY = 0;

    int columns() const { return xmax - xmin + 1; }
    int latticeId(int x, int y) const { return (y - ymin) * columns() + (x - xmin); }
    bool inBounds(int x, int y) const { return x >= xmin && x <= xmax && y >= ymin && y <= ymax; }
    bool insideShelf(double x, double y) const;
    double nodeMultiplier(int x, int y) const;  // largest zone multiplier at the node, else 1
    const NamedPoint* point(const std::string& pointName) const;
};

Scenario loadScenario(const std::string& path);
void saveScenario(const Scenario& scenario, const std::string& path);

// Benchmark warehouses: blocksX x blocksY racks with aisles between rack rows, cross aisles between
// rack columns and a perimeter aisle. Each aisle stretch between two cross aisles becomes a picking
// zone with probability congestedFraction (drawn from seed).
struct WarehouseParams {
    std::string name;
    int blocksX = 4, blocksY = 5;
    int rackWidth = 12, rackDepth = 3, aisleWidth = 2, crossWidth = 3, border = 3;
    double congestedFraction = 0.25;
    double congestedMultiplier = 3.0;
    std::uint64_t seed = 1;
};
Scenario makeWarehouse(const WarehouseParams& params);

// True if the closed segment (ax,ay)-(bx,by) touches the closed rectangle.
bool segmentTouchesRect(double ax, double ay, double bx, double by, const Rect& rect);

// Adds one node per free lattice point and one two-way edge per pair of lattice neighbours whose
// segment touches no shelf. 8-connected lattices add diagonals, which therefore never cut a shelf
// corner. Edge weight = straight-line length x mean of the two endpoint zone multipliers.
// The library logs every addNode/addEdge to std::cout; callers silence it (see CoutSilencer).
void buildGraph(const Scenario& scenario, Graph& graph);

// Node ids inside a rectangle, ascending.
std::vector<int> nodesInRect(const Graph& graph, const Rect& rect);

// Route properties, computed from the graph the route was planned on.
bool routeFollowsEdges(const Graph& graph, const std::vector<int>& route);
double routeCost(const Graph& graph, const std::vector<int>& route);    // sum of edge weights
double routeLength(const Graph& graph, const std::vector<int>& route);  // raw coordinate units
bool routeAvoids(const std::vector<int>& route, const std::vector<int>& blocked, size_t fromIndex = 0);

// FNV-1a hash of the node coordinates and edge weights, in id order, as 16 hex digits. Lets the
// benchmark confirm it measured the same graph the workload files describe.
std::string graphFingerprint(const Graph& graph);

// Edges as (from, to, weight) with from < to, sorted: each two-way edge once.
struct EdgeRecord { int from; int to; double weight; };
std::vector<EdgeRecord> undirectedEdges(const Graph& graph);
std::vector<int> sortedNodeIds(const Graph& graph);

// A copy of the graph without the given nodes: the same operation DynamicReplanner performs
// before it searches around obstacles (Graph copy, then Graph::removeNode per blocked node).
Graph withoutNodes(const Graph& graph, const std::vector<int>& removed);

// Sends std::cout nowhere while alive. With no stream buffer, every << on std::cout returns after
// a state check, so the library's console logging does almost no work.
class CoutSilencer {
public:
    CoutSilencer();
    ~CoutSilencer();
    CoutSilencer(const CoutSilencer&) = delete;
    CoutSilencer& operator=(const CoutSilencer&) = delete;
private:
    std::streambuf* saved;
};

// splitmix64: fully specified, so workloads are identical on every compiler and standard library
// (std::uniform_int_distribution is not).
class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) : state(seed) {}
    std::uint64_t next();
    std::uint64_t below(std::uint64_t bound);  // uniform in [0, bound), rejection sampling
    double unit();                             // uniform in [0, 1)
private:
    std::uint64_t state;
};

// Minimal streaming JSON writer. Arrays opened with compact = true stay on one line.
class JsonWriter {
public:
    explicit JsonWriter(std::ostream& out);
    JsonWriter& beginObject();
    JsonWriter& endObject();
    JsonWriter& beginArray(bool compact = false);
    JsonWriter& endArray();
    JsonWriter& key(const std::string& name);
    JsonWriter& value(const std::string& text);
    JsonWriter& value(const char* text);
    JsonWriter& value(double number);
    JsonWriter& value(bool flag);
    template <typename Integer,
              typename = std::enable_if_t<std::is_integral_v<Integer> && !std::is_same_v<Integer, bool>>>
    JsonWriter& value(Integer number) {
        separate();
        out << +number;
        return *this;
    }
    JsonWriter& null();
    JsonWriter& ints(const std::vector<int>& numbers);  // compact array
private:
    struct Level { bool array; bool compact; bool empty; };
    std::ostream& out;
    std::vector<Level> stack;
    bool afterKey = false;
    void separate();
    void newline();
};

std::string formatDouble(double number);  // shortest round-trip form, "." decimal point
std::string escapeJson(const std::string& text);

// Compiler and standard library identification baked in at build time.
std::string compilerDescription();

}  // namespace evidence
