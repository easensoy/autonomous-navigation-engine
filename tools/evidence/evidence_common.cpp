#include "evidence_common.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace evidence {

// ---------------------------------------------------------------------------------------------
// Scenario files

bool Scenario::insideShelf(double x, double y) const {
    return std::any_of(shelves.begin(), shelves.end(), [x, y](const Shelf& s) { return s.rect.contains(x, y); });
}

double Scenario::nodeMultiplier(int x, int y) const {
    double multiplier = 1.0;
    bool inZone = false;
    for (const Zone& zone : zones) {
        if (zone.rect.contains(x, y)) {
            multiplier = inZone ? std::max(multiplier, zone.multiplier) : zone.multiplier;
            inZone = true;
        }
    }
    return multiplier;
}

const NamedPoint* Scenario::point(const std::string& pointName) const {
    for (const NamedPoint& p : points) {
        if (p.name == pointName) {
            return &p;
        }
    }
    return nullptr;
}

namespace {

std::string restOfLine(std::istringstream& in) {
    std::string rest;
    std::getline(in, rest);
    const auto first = rest.find_first_not_of(" \t");
    return first == std::string::npos ? std::string() : rest.substr(first);
}

Rect readRect(std::istringstream& in, const std::string& where) {
    Rect r;
    if (!(in >> r.x0 >> r.y0 >> r.x1 >> r.y1)) {
        throw std::runtime_error(where + ": expected x0 y0 x1 y1");
    }
    if (r.x1 < r.x0 || r.y1 < r.y0) {
        throw std::runtime_error(where + ": rectangle has x1 < x0 or y1 < y0");
    }
    return r;
}

std::string directoryOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    return slash == std::string::npos ? std::string() : path.substr(0, slash + 1);
}

void parseScenarioFile(Scenario& s, const std::string& path, bool& haveLattice, int depth) {
    if (depth > 4) {
        throw std::runtime_error(path + ": includes nested too deeply");
    }
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("cannot open scenario " + path);
    }
    std::string line;
    int lineNumber = 0;
    while (std::getline(file, line)) {
        ++lineNumber;
        const std::string where = path + ":" + std::to_string(lineNumber);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        std::string body = line;
        if (const auto hash = body.find('#'); hash != std::string::npos) {
            body = body.substr(0, hash);
        }
        std::istringstream in(body);
        std::string keyword;
        if (!(in >> keyword)) {
            continue;
        }
        if (keyword == "include") {
            std::string included;
            if (!(in >> included)) {
                throw std::runtime_error(where + ": expected include <file>");
            }
            parseScenarioFile(s, directoryOf(path) + included, haveLattice, depth + 1);
        } else if (keyword == "name") {
            in >> s.name;
        } else if (keyword == "title") {
            s.title = restOfLine(in);
        } else if (keyword == "lattice") {
            if (!(in >> s.xmin >> s.ymin >> s.xmax >> s.ymax) || s.xmax < s.xmin || s.ymax < s.ymin) {
                throw std::runtime_error(where + ": expected lattice xmin ymin xmax ymax");
            }
            haveLattice = true;
        } else if (keyword == "connectivity") {
            if (!(in >> s.connectivity) || (s.connectivity != 4 && s.connectivity != 8)) {
                throw std::runtime_error(where + ": connectivity must be 4 or 8");
            }
        } else if (keyword == "shelf") {
            Shelf shelf;
            shelf.rect = readRect(in, where);
            shelf.label = restOfLine(in);
            s.shelves.push_back(shelf);
        } else if (keyword == "zone") {
            Zone zone;
            zone.rect = readRect(in, where);
            if (!(in >> zone.multiplier) || !(zone.multiplier > 0.0)) {
                throw std::runtime_error(where + ": zone needs a positive cost multiplier");
            }
            zone.label = restOfLine(in);
            s.zones.push_back(zone);
        } else if (keyword == "block") {
            Block block;
            block.rect = readRect(in, where);
            block.label = restOfLine(in);
            s.blocks.push_back(block);
        } else if (keyword == "block_step") {
            in >> s.blockStep;
        } else if (keyword == "start") {
            if (!(in >> s.startX >> s.startY)) {
                throw std::runtime_error(where + ": expected start x y");
            }
            s.hasStart = true;
        } else if (keyword == "goal") {
            if (!(in >> s.goalX >> s.goalY)) {
                throw std::runtime_error(where + ": expected goal x y");
            }
            s.hasGoal = true;
        } else if (keyword == "point") {
            NamedPoint p;
            if (!(in >> p.name >> p.x >> p.y)) {
                throw std::runtime_error(where + ": expected point name x y");
            }
            s.points.push_back(p);
        } else {
            throw std::runtime_error(where + ": unknown keyword '" + keyword + "'");
        }
    }
}

}  // namespace

Scenario loadScenario(const std::string& path) {
    Scenario s;
    bool haveLattice = false;
    parseScenarioFile(s, path, haveLattice, 0);
    if (!haveLattice) {
        throw std::runtime_error(path + ": no lattice line");
    }
    return s;
}

void saveScenario(const Scenario& s, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("cannot write scenario " + path);
    }
    auto rect = [](const Rect& r) {
        return formatDouble(r.x0) + " " + formatDouble(r.y0) + " " + formatDouble(r.x1) + " " + formatDouble(r.y1);
    };
    out << "# Generated by evidence_trace workloads; regenerate rather than edit.\n";
    out << "name " << s.name << "\n";
    if (!s.title.empty()) {
        out << "title " << s.title << "\n";
    }
    out << "lattice " << s.xmin << " " << s.ymin << " " << s.xmax << " " << s.ymax << "\n";
    out << "connectivity " << s.connectivity << "\n";
    for (const Shelf& shelf : s.shelves) {
        out << "shelf " << rect(shelf.rect) << (shelf.label.empty() ? "" : " " + shelf.label) << "\n";
    }
    for (const Zone& zone : s.zones) {
        out << "zone " << rect(zone.rect) << " " << formatDouble(zone.multiplier)
            << (zone.label.empty() ? "" : " " + zone.label) << "\n";
    }
    for (const Block& block : s.blocks) {
        out << "block " << rect(block.rect) << (block.label.empty() ? "" : " " + block.label) << "\n";
    }
    if (s.blockStep >= 0) {
        out << "block_step " << s.blockStep << "\n";
    }
    if (s.hasStart) {
        out << "start " << s.startX << " " << s.startY << "\n";
    }
    if (s.hasGoal) {
        out << "goal " << s.goalX << " " << s.goalY << "\n";
    }
    for (const NamedPoint& p : s.points) {
        out << "point " << p.name << " " << p.x << " " << p.y << "\n";
    }
}

Scenario makeWarehouse(const WarehouseParams& p) {
    Scenario s;
    s.name = p.name;
    s.title = std::to_string(p.blocksX) + " x " + std::to_string(p.blocksY) + " racks, picking zones with p=" +
              formatDouble(p.congestedFraction) + " at x" + formatDouble(p.congestedMultiplier) + ", seed " +
              std::to_string(p.seed);
    s.connectivity = 8;
    s.xmin = 0;
    s.ymin = 0;
    s.xmax = 2 * p.border + p.blocksX * p.rackWidth + (p.blocksX - 1) * p.crossWidth - 1;
    s.ymax = 2 * p.border + p.blocksY * p.rackDepth + (p.blocksY - 1) * p.aisleWidth - 1;
    for (int by = 0; by < p.blocksY; ++by) {
        for (int bx = 0; bx < p.blocksX; ++bx) {
            const double x0 = p.border + bx * (p.rackWidth + p.crossWidth);
            const double y0 = p.border + by * (p.rackDepth + p.aisleWidth);
            s.shelves.push_back({{x0 - 0.5, y0 - 0.5, x0 + p.rackWidth - 0.5, y0 + p.rackDepth - 0.5}, "rack"});
        }
    }
    SplitMix64 rng(p.seed);
    for (int by = 0; by + 1 < p.blocksY; ++by) {
        for (int bx = 0; bx < p.blocksX; ++bx) {
            if (rng.unit() >= p.congestedFraction) {
                continue;
            }
            const double x0 = p.border + bx * (p.rackWidth + p.crossWidth);
            const double y0 = p.border + by * (p.rackDepth + p.aisleWidth) + p.rackDepth;
            s.zones.push_back({{x0 - 0.5, y0 - 0.5, x0 + p.rackWidth - 0.5, y0 + p.aisleWidth - 0.5},
                               p.congestedMultiplier, "picking zone"});
        }
    }
    return s;
}

// ---------------------------------------------------------------------------------------------
// Geometry and graph construction

bool segmentTouchesRect(double ax, double ay, double bx, double by, const Rect& rect) {
    // Liang-Barsky clipping against the closed rectangle: touching counts as intersecting.
    double t0 = 0.0, t1 = 1.0;
    const double dx = bx - ax, dy = by - ay;
    auto clip = [&t0, &t1](double p, double q) {
        if (p == 0.0) {
            return q >= 0.0;
        }
        const double t = q / p;
        if (p < 0.0) {
            if (t > t1) return false;
            t0 = std::max(t0, t);
        } else {
            if (t < t0) return false;
            t1 = std::min(t1, t);
        }
        return true;
    };
    return clip(-dx, ax - rect.x0) && clip(dx, rect.x1 - ax) && clip(-dy, ay - rect.y0) && clip(dy, rect.y1 - ay);
}

void buildGraph(const Scenario& s, Graph& graph) {
    for (int y = s.ymin; y <= s.ymax; ++y) {
        for (int x = s.xmin; x <= s.xmax; ++x) {
            if (!s.insideShelf(x, y)) {
                graph.addNode(s.latticeId(x, y), "n" + std::to_string(s.latticeId(x, y)), x, y);
            }
        }
    }
    static const int offsets8[4][2] = {{1, 0}, {0, 1}, {1, 1}, {-1, 1}};
    const int offsetCount = s.connectivity == 8 ? 4 : 2;
    for (int y = s.ymin; y <= s.ymax; ++y) {
        for (int x = s.xmin; x <= s.xmax; ++x) {
            const int from = s.latticeId(x, y);
            if (!graph.hasNode(from)) {
                continue;
            }
            for (int k = 0; k < offsetCount; ++k) {
                const int nx = x + offsets8[k][0], ny = y + offsets8[k][1];
                if (!s.inBounds(nx, ny) || !graph.hasNode(s.latticeId(nx, ny))) {
                    continue;
                }
                const bool blocked = std::any_of(s.shelves.begin(), s.shelves.end(), [&](const Shelf& shelf) {
                    return segmentTouchesRect(x, y, nx, ny, shelf.rect);
                });
                if (blocked) {
                    continue;
                }
                const double length = std::hypot(double(nx - x), double(ny - y));
                const double multiplier = 0.5 * (s.nodeMultiplier(x, y) + s.nodeMultiplier(nx, ny));
                graph.addEdge(from, s.latticeId(nx, ny), length * multiplier, true);
            }
        }
    }
}

std::vector<int> nodesInRect(const Graph& graph, const Rect& rect) {
    std::vector<int> ids;
    for (int id : graph.getAllNodeIds()) {
        const Node& node = graph.getNode(id);
        if (rect.contains(node.getX(), node.getY())) {
            ids.push_back(id);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

bool routeFollowsEdges(const Graph& graph, const std::vector<int>& route) {
    if (route.empty() || !graph.hasNode(route.front())) {
        return false;
    }
    for (size_t i = 1; i < route.size(); ++i) {
        if (!graph.hasEdge(route[i - 1], route[i])) {
            return false;
        }
    }
    return true;
}

double routeCost(const Graph& graph, const std::vector<int>& route) {
    double cost = 0.0;
    for (size_t i = 1; i < route.size(); ++i) {
        cost += graph.getEdgeWeight(route[i - 1], route[i]);
    }
    return cost;
}

double routeLength(const Graph& graph, const std::vector<int>& route) {
    double length = 0.0;
    for (size_t i = 1; i < route.size(); ++i) {
        const Node& a = graph.getNode(route[i - 1]);
        const Node& b = graph.getNode(route[i]);
        length += std::hypot(b.getX() - a.getX(), b.getY() - a.getY());
    }
    return length;
}

bool routeAvoids(const std::vector<int>& route, const std::vector<int>& blocked, size_t fromIndex) {
    for (size_t i = fromIndex; i < route.size(); ++i) {
        if (std::find(blocked.begin(), blocked.end(), route[i]) != blocked.end()) {
            return false;
        }
    }
    return true;
}

std::vector<int> sortedNodeIds(const Graph& graph) {
    std::vector<int> ids = graph.getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    return ids;
}

std::vector<EdgeRecord> undirectedEdges(const Graph& graph) {
    std::vector<EdgeRecord> edges;
    for (int id : sortedNodeIds(graph)) {
        for (const Edge& edge : graph.getEdgesFrom(id)) {
            if (!edge.isBidirectional() || id < edge.getToNode()) {
                edges.push_back({id, edge.getToNode(), edge.getWeight()});
            }
        }
    }
    std::sort(edges.begin(), edges.end(), [](const EdgeRecord& a, const EdgeRecord& b) {
        return a.from != b.from ? a.from < b.from : a.to < b.to;
    });
    return edges;
}

std::string graphFingerprint(const Graph& graph) {
    std::uint64_t hash = 1469598103934665603ULL;
    auto mix = [&hash](const std::string& text) {
        for (unsigned char c : text) {
            hash ^= c;
            hash *= 1099511628211ULL;
        }
    };
    for (int id : sortedNodeIds(graph)) {
        const Node& node = graph.getNode(id);
        mix("n" + std::to_string(id) + ":" + formatDouble(node.getX()) + "," + formatDouble(node.getY()) + ";");
    }
    for (const EdgeRecord& edge : undirectedEdges(graph)) {
        mix("e" + std::to_string(edge.from) + "-" + std::to_string(edge.to) + ":" + formatDouble(edge.weight) + ";");
    }
    char buffer[17];
    std::snprintf(buffer, sizeof buffer, "%016llx", static_cast<unsigned long long>(hash));
    return buffer;
}

Graph withoutNodes(const Graph& graph, const std::vector<int>& removed) {
    Graph reduced(graph);
    for (int id : removed) {
        reduced.removeNode(id);
    }
    return reduced;
}

// ---------------------------------------------------------------------------------------------
// Helpers

CoutSilencer::CoutSilencer() : saved(std::cout.rdbuf(nullptr)) {}

CoutSilencer::~CoutSilencer() {
    std::cout.rdbuf(saved);
    std::cout.clear();
}

std::uint64_t SplitMix64::next() {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
}

std::uint64_t SplitMix64::below(std::uint64_t bound) {
    if (bound == 0) {
        throw std::invalid_argument("SplitMix64::below(0)");
    }
    const std::uint64_t limit = UINT64_MAX - UINT64_MAX % bound;
    std::uint64_t draw;
    do {
        draw = next();
    } while (draw >= limit);
    return draw % bound;
}

double SplitMix64::unit() {
    return double(next() >> 11) * (1.0 / 9007199254740992.0);
}

std::string formatDouble(double number) {
    if (!std::isfinite(number)) {
        return "null";
    }
    char buffer[40];
    for (int precision = 15; precision <= 17; ++precision) {
        std::snprintf(buffer, sizeof buffer, "%.*g", precision, number);
        if (std::strtod(buffer, nullptr) == number) {
            break;
        }
    }
    return buffer;
}

std::string escapeJson(const std::string& text) {
    std::string out;
    out.reserve(text.size() + 2);
    for (unsigned char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buffer[8];
                    std::snprintf(buffer, sizeof buffer, "\\u%04x", c);
                    out += buffer;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

JsonWriter::JsonWriter(std::ostream& stream) : out(stream) {}

void JsonWriter::newline() {
    out << "\n" << std::string(2 * stack.size(), ' ');
}

void JsonWriter::separate() {
    if (afterKey) {
        afterKey = false;
        return;
    }
    if (stack.empty()) {
        return;
    }
    Level& level = stack.back();
    if (!level.empty) {
        out << ",";
        if (level.compact) {
            out << " ";
        }
    }
    if (!level.compact) {
        newline();
    }
    level.empty = false;
}

JsonWriter& JsonWriter::beginObject() {
    separate();
    out << "{";
    stack.push_back({false, false, true});
    return *this;
}

JsonWriter& JsonWriter::endObject() {
    const bool empty = stack.back().empty;
    stack.pop_back();
    if (!empty) {
        newline();
    }
    out << "}";
    if (stack.empty()) {
        out << "\n";
    }
    return *this;
}

JsonWriter& JsonWriter::beginArray(bool compact) {
    const bool parentCompact = !stack.empty() && stack.back().compact;
    separate();
    out << "[";
    stack.push_back({true, compact || parentCompact, true});
    return *this;
}

JsonWriter& JsonWriter::endArray() {
    const Level level = stack.back();
    stack.pop_back();
    if (!level.empty && !level.compact) {
        newline();
    }
    out << "]";
    return *this;
}

JsonWriter& JsonWriter::key(const std::string& name) {
    separate();
    out << "\"" << escapeJson(name) << "\": ";
    afterKey = true;
    return *this;
}

JsonWriter& JsonWriter::value(const std::string& text) {
    separate();
    out << "\"" << escapeJson(text) << "\"";
    return *this;
}

JsonWriter& JsonWriter::value(const char* text) { return value(std::string(text)); }

JsonWriter& JsonWriter::value(double number) {
    separate();
    out << formatDouble(number);
    return *this;
}

JsonWriter& JsonWriter::value(bool flag) {
    separate();
    out << (flag ? "true" : "false");
    return *this;
}

JsonWriter& JsonWriter::null() {
    separate();
    out << "null";
    return *this;
}

JsonWriter& JsonWriter::ints(const std::vector<int>& numbers) {
    beginArray(true);
    for (int n : numbers) {
        value(n);
    }
    return endArray();
}

std::string compilerDescription() {
    std::string text;
#if defined(__clang__)
    text = std::string("Clang ") + __clang_version__;
#elif defined(_MSC_VER)
    const long long v = _MSC_FULL_VER;
    text = "MSVC " + std::to_string(v / 10000000) + "." + std::to_string((v / 100000) % 100) + "." +
           std::to_string(v % 100000);
#elif defined(__GNUC__)
    text = std::string("GCC ") + __VERSION__;
#else
    text = "unknown compiler";
#endif
#if defined(_MSVC_STL_VERSION)
    text += ", MSVC STL " + std::to_string(_MSVC_STL_VERSION);
#elif defined(_LIBCPP_VERSION)
    text += ", libc++ " + std::to_string(_LIBCPP_VERSION);
#elif defined(__GLIBCXX__)
    text += ", libstdc++ " + std::to_string(__GLIBCXX__);
#endif
    return text;
}

}  // namespace evidence
