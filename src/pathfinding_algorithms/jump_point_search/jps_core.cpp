#include "pathfinding_algorithms/JumpPointSearch.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <queue>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Jump Point Search without corner cutting on a uniform-cost grid graph. The supported domain, the
// guarantee and the search rules are documented in JumpPointSearch.hpp.

namespace {

constexpr double kRelativeTolerance = 1e-9;
const double kSqrt2 = std::sqrt(2.0);

constexpr int kDirections[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

bool nearlyEqual(double a, double b) {
    return std::fabs(a - b) <= kRelativeTolerance * std::max({1.0, std::fabs(a), std::fabs(b)});
}

int sign(int value) {
    return (value > 0) - (value < 0);
}

std::string number(double value) {
    std::ostringstream out;
    out.precision(17);
    out << value;
    return out.str();
}

std::string cellName(int x, int y) {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

std::string edgeName(int from, int to) {
    return "edge " + std::to_string(from) + " -> " + std::to_string(to);
}

struct FrontierEntry {
    double f;
    double g;
    int cell;
};

// Orders the frontier: lowest f first and, on equal f, the larger g.
struct ComesLater {
    bool operator()(const FrontierEntry& a, const FrontierEntry& b) const {
        if (a.f != b.f) {
            return a.f > b.f;
        }
        return a.g < b.g;
    }
};

}  // namespace

JumpPointSearch::JumpPointSearch(const Graph* environment, int width, int height)
    : graph(environment), gridWidth(width), gridHeight(height) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
}

JumpPointSearch::~JumpPointSearch() = default;

std::uint64_t JumpPointSearch::packCell(int x, int y) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(y);
}

const JumpPointSearch::GridModel& JumpPointSearch::currentModel() const {
    if (!model.built || model.revision != graph->getRevision() || model.width != gridWidth ||
        model.height != gridHeight || model.diagonal != diagonalMovement) {
        buildModel();
    }
    return model;
}

// Lays the graph onto the grid and records the first violation of the supported domain, if any.
void JumpPointSearch::buildModel() const {
    GridModel m;
    m.built = true;
    m.revision = graph->getRevision();
    m.width = gridWidth;
    m.height = gridHeight;
    m.diagonal = diagonalMovement;
    m.cellNode.assign(static_cast<std::size_t>(gridWidth) * static_cast<std::size_t>(gridHeight), -1);

    std::vector<int> ids = graph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());  // the same first violation on every platform

    auto store = [this, &m](std::string problem) {
        m.problem = std::move(problem);
        model = std::move(m);
    };
    auto nodeAt = [&m](int x, int y) {
        if (x < 0 || y < 0 || x >= m.width || y >= m.height) {
            return -1;
        }
        return m.cellNode[static_cast<std::size_t>(y) * m.width + x];
    };
    auto cellOf = [this](int id) {
        const Node& node = graph->getNode(id);
        return std::make_pair(static_cast<int>(std::lround(node.getX())), static_cast<int>(std::lround(node.getY())));
    };

    // 1. Every node on its own integer cell inside the grid.
    for (int id : ids) {
        const Node& node = graph->getNode(id);
        const double x = node.getX();
        const double y = node.getY();
        const double rx = std::round(x);
        const double ry = std::round(y);
        if (!std::isfinite(x) || !std::isfinite(y) || std::fabs(x - rx) > 1e-9 || std::fabs(y - ry) > 1e-9) {
            return store("node " + std::to_string(id) + " at (" + number(x) + ", " + number(y) +
                         ") is not on an integer grid point");
        }
        if (rx < 0 || ry < 0 || rx >= gridWidth || ry >= gridHeight) {
            return store("node " + std::to_string(id) + " at " + cellName(static_cast<int>(rx), static_cast<int>(ry)) +
                         " lies outside the " + std::to_string(gridWidth) + " x " + std::to_string(gridHeight) + " grid");
        }
        int& slot = m.cellNode[static_cast<std::size_t>(ry) * gridWidth + static_cast<std::size_t>(rx)];
        if (slot >= 0) {
            return store("nodes " + std::to_string(slot) + " and " + std::to_string(id) + " share the grid cell " +
                         cellName(static_cast<int>(rx), static_cast<int>(ry)));
        }
        slot = id;
    }

    // 2. Every edge is a grid move: neighbouring cells, no diagonal when diagonals are disabled, no
    //    corner cutting, and one weight s for all orthogonal edges.
    bool haveStep = false;
    double step = 1.0;
    for (int id : ids) {
        const auto [cx, cy] = cellOf(id);
        for (const Edge& edge : graph->getEdgesFrom(id)) {
            const int to = edge.getToNode();
            if (!graph->hasNode(to)) {
                return store(edgeName(id, to) + " leads to a node the graph does not have");
            }
            const auto [tx, ty] = cellOf(to);
            const int dx = tx - cx;
            const int dy = ty - cy;
            if (std::max(std::abs(dx), std::abs(dy)) != 1) {
                return store(edgeName(id, to) + " joins the cells " + cellName(cx, cy) + " and " + cellName(tx, ty) +
                             ", which are not neighbours");
            }
            if (dx != 0 && dy != 0) {
                if (!diagonalMovement) {
                    return store(edgeName(id, to) + " is diagonal, but diagonal movement is disabled");
                }
                if (nodeAt(cx + dx, cy) < 0 || nodeAt(cx, cy + dy) < 0) {
                    const bool firstEmpty = nodeAt(cx + dx, cy) < 0;
                    return store(edgeName(id, to) + " cuts the corner of the empty cell " +
                                 (firstEmpty ? cellName(cx + dx, cy) : cellName(cx, cy + dy)));
                }
                continue;  // its weight is checked once s is known
            }
            const double weight = edge.getWeight();
            if (!(weight > 0.0) || !std::isfinite(weight)) {
                return store(edgeName(id, to) + " has weight " + number(weight) +
                             "; orthogonal weights must be positive and finite");
            }
            if (!haveStep) {
                step = weight;
                haveStep = true;
            } else if (!nearlyEqual(weight, step)) {
                return store(edgeName(id, to) + " has weight " + number(weight) + ", but another orthogonal edge has weight " +
                             number(step));
            }
        }
    }

    // 3. Every grid move between two cells that hold nodes is an edge.
    const int directionCount = diagonalMovement ? 8 : 4;
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            const int from = nodeAt(x, y);
            if (from < 0) {
                continue;
            }
            for (int k = 0; k < directionCount; ++k) {
                const int dx = kDirections[k][0];
                const int dy = kDirections[k][1];
                const int to = nodeAt(x + dx, y + dy);
                if (to < 0 || (dx != 0 && dy != 0 && (nodeAt(x + dx, y) < 0 || nodeAt(x, y + dy) < 0))) {
                    continue;
                }
                if (!graph->hasEdge(from, to)) {
                    return store("the cells " + cellName(x, y) + " and " + cellName(x + dx, y + dy) + " hold nodes " +
                                 std::to_string(from) + " and " + std::to_string(to) + ", but the graph has no " +
                                 edgeName(from, to));
                }
            }
        }
    }

    // 4. Diagonal weights are s * sqrt(2), and every edge has a reverse edge of the same weight.
    for (int id : ids) {
        const auto [cx, cy] = cellOf(id);
        for (const Edge& edge : graph->getEdgesFrom(id)) {
            const int to = edge.getToNode();
            const auto [tx, ty] = cellOf(to);
            const double weight = edge.getWeight();
            if (tx != cx && ty != cy && !nearlyEqual(weight, step * kSqrt2)) {
                return store(edgeName(id, to) + " has weight " + number(weight) + ", but a diagonal must weigh s * sqrt(2) = " +
                             number(step * kSqrt2));
            }
            if (!graph->hasEdge(to, id)) {
                return store(edgeName(id, to) + " is one-way: the graph has no " + edgeName(to, id));
            }
            if (!nearlyEqual(graph->getEdgeWeight(to, id), weight)) {
                return store(edgeName(id, to) + " and its reverse have different weights");
            }
        }
    }

    m.step = step;
    model = std::move(m);
}

void JumpPointSearch::requireSupportedGraph() const {
    const GridModel& m = currentModel();
    if (!m.problem.empty()) {
        throw std::invalid_argument("JumpPointSearch: unsupported graph: " + m.problem);
    }
}

bool JumpPointSearch::supportsGraph(std::string* reason) const {
    const GridModel& m = currentModel();
    if (m.problem.empty()) {
        return true;
    }
    if (reason) {
        *reason = m.problem;
    }
    return false;
}

void JumpPointSearch::refreshOpenCells() {
    openCells.assign(model.cellNode.size(), 0);
    for (std::size_t i = 0; i < model.cellNode.size(); ++i) {
        openCells[i] = model.cellNode[i] >= 0 ? 1 : 0;
    }
    for (std::uint64_t key : markedCells) {
        const int x = static_cast<int>(static_cast<std::uint32_t>(key >> 32));
        const int y = static_cast<int>(static_cast<std::uint32_t>(key & 0xffffffffu));
        if (x >= 0 && y >= 0 && x < model.width && y < model.height) {
            openCells[static_cast<std::size_t>(y) * model.width + x] = 0;
        }
    }
}

bool JumpPointSearch::isOpen(int x, int y) const {
    if (x < 0 || y < 0 || x >= model.width || y >= model.height) {
        return false;
    }
    return openCells[static_cast<std::size_t>(y) * model.width + x] != 0;
}

// A move to a neighbouring cell; a diagonal also needs both cells beside it open (no corner cutting).
bool JumpPointSearch::canMove(int x, int y, int dx, int dy) const {
    if (!isOpen(x + dx, y + dy)) {
        return false;
    }
    if (dx != 0 && dy != 0) {
        return model.diagonal && isOpen(x + dx, y) && isOpen(x, y + dy);
    }
    return true;
}

// For a straight move into (x, y): a side neighbour is open while the cell behind it is closed, so
// the side neighbour can only be reached optimally through (x, y).
bool JumpPointSearch::hasForcedNeighbour(int x, int y, int dx, int dy) const {
    if (dx != 0) {
        return (isOpen(x, y + 1) && !isOpen(x - dx, y + 1)) || (isOpen(x, y - 1) && !isOpen(x - dx, y - 1));
    }
    return (isOpen(x + 1, y) && !isOpen(x + 1, y - dy)) || (isOpen(x - 1, y) && !isOpen(x - 1, y - dy));
}

bool JumpPointSearch::straightJumpFinds(int x, int y, int dx, int dy, int goalX, int goalY) const {
    while (canMove(x, y, dx, dy)) {
        x += dx;
        y += dy;
        if ((x == goalX && y == goalY) || hasForcedNeighbour(x, y, dx, dy)) {
            return true;
        }
    }
    return false;
}

// Moves from (x, y) in direction (dx, dy) to the next jump point; false if the run ends at a closed
// cell or the edge of the grid first.
bool JumpPointSearch::jump(int x, int y, int dx, int dy, int goalX, int goalY, int& jumpX, int& jumpY) const {
    while (canMove(x, y, dx, dy)) {
        x += dx;
        y += dy;
        bool stop = x == goalX && y == goalY;
        if (!stop) {
            if (dx != 0 && dy != 0) {
                stop = straightJumpFinds(x, y, dx, 0, goalX, goalY) || straightJumpFinds(x, y, 0, dy, goalX, goalY);
            } else {
                stop = hasForcedNeighbour(x, y, dx, dy);
                if (!stop && !model.diagonal && dy != 0) {
                    stop = straightJumpFinds(x, y, 1, 0, goalX, goalY) || straightJumpFinds(x, y, -1, 0, goalX, goalY);
                }
            }
        }
        if (stop) {
            jumpX = x;
            jumpY = y;
            return true;
        }
    }
    return false;
}

// Directions to jump in when expanding (x, y), given the jump point it was reached from (none for
// the start, passed as a negative parentX).
void JumpPointSearch::successorDirections(int x, int y, int parentX, int parentY,
                                          std::vector<std::pair<int, int>>& directions) const {
    directions.clear();
    if (parentX < 0) {
        const int count = model.diagonal ? 8 : 4;
        for (int k = 0; k < count; ++k) {
            directions.emplace_back(kDirections[k][0], kDirections[k][1]);
        }
        return;
    }
    const int dx = sign(x - parentX);
    const int dy = sign(y - parentY);
    if (dx != 0 && dy != 0) {
        directions = {{dx, 0}, {0, dy}, {dx, dy}};  // a diagonal has no forced neighbours
        return;
    }
    if (!model.diagonal) {
        if (dx != 0) {
            directions = {{dx, 0}, {0, 1}, {0, -1}};
        } else {
            directions = {{0, dy}, {1, 0}, {-1, 0}};
        }
        return;
    }
    if (dx != 0) {
        directions.emplace_back(dx, 0);
        for (int side : {1, -1}) {
            if (isOpen(x, y + side) && !isOpen(x - dx, y + side)) {
                directions.emplace_back(0, side);
                directions.emplace_back(dx, side);
            }
        }
    } else {
        directions.emplace_back(0, dy);
        for (int side : {1, -1}) {
            if (isOpen(x + side, y) && !isOpen(x + side, y - dy)) {
                directions.emplace_back(side, 0);
                directions.emplace_back(side, dy);
            }
        }
    }
}

double JumpPointSearch::heuristic(int x1, int y1, int x2, int y2) const {
    const double dx = std::abs(x1 - x2);
    const double dy = std::abs(y1 - y2);
    if (model.diagonal) {
        return model.step * (std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy));
    }
    return model.step * (dx + dy);
}

// Cost of the straight or diagonal run between two jump points.
double JumpPointSearch::segmentCost(int x1, int y1, int x2, int y2) const {
    const int dx = std::abs(x2 - x1);
    const int dy = std::abs(y2 - y1);
    if (dx != 0 && dy != 0) {
        return model.step * kSqrt2 * dx;
    }
    return model.step * (dx + dy);
}

std::vector<int> JumpPointSearch::findPath(int startId, int goalId) {
    lastExpansions = 0;
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        return {};  // unknown start or goal: no route, as with the other searches
    }
    requireSupportedGraph();
    refreshOpenCells();

    const Node& startNode = graph->getNode(startId);
    const Node& goalNode = graph->getNode(goalId);
    const int startX = static_cast<int>(std::lround(startNode.getX()));
    const int startY = static_cast<int>(std::lround(startNode.getY()));
    const int goalX = static_cast<int>(std::lround(goalNode.getX()));
    const int goalY = static_cast<int>(std::lround(goalNode.getY()));
    if (!isOpen(startX, startY) || !isOpen(goalX, goalY)) {
        return {};
    }
    if (startId == goalId) {
        return {startId};
    }

    const int width = model.width;
    const std::size_t cellCount = model.cellNode.size();
    std::vector<double> g(cellCount, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cellCount, -1);
    std::vector<char> closed(cellCount, 0);
    std::priority_queue<FrontierEntry, std::vector<FrontierEntry>, ComesLater> frontier;

    const int startCell = startY * width + startX;
    const int goalCell = goalY * width + goalX;
    g[startCell] = 0.0;
    frontier.push({heuristic(startX, startY, goalX, goalY), 0.0, startCell});

    std::vector<std::pair<int, int>> directions;
    while (!frontier.empty()) {
        const FrontierEntry current = frontier.top();
        frontier.pop();
        if (current.g > g[current.cell] || closed[current.cell]) {
            continue;  // made stale by a cheaper route, or already expanded at this cost
        }
        closed[current.cell] = 1;
        ++lastExpansions;
        if (current.cell == goalCell) {
            break;
        }

        const int x = current.cell % width;
        const int y = current.cell / width;
        const int from = parent[current.cell];
        successorDirections(x, y, from < 0 ? -1 : from % width, from < 0 ? -1 : from / width, directions);
        for (const auto& [dx, dy] : directions) {
            int jumpX = 0;
            int jumpY = 0;
            if (!jump(x, y, dx, dy, goalX, goalY, jumpX, jumpY)) {
                continue;
            }
            const int jumpCell = jumpY * width + jumpX;
            const double tentative = current.g + segmentCost(x, y, jumpX, jumpY);
            if (tentative < g[jumpCell]) {
                g[jumpCell] = tentative;
                parent[jumpCell] = current.cell;
                closed[jumpCell] = 0;  // reopens a closed cell; only rounding can make this happen
                frontier.push({tentative + heuristic(jumpX, jumpY, goalX, goalY), tentative, jumpCell});
            }
        }
    }

    if (!closed[goalCell]) {
        return {};
    }
    std::vector<int> jumpCells;
    for (int cell = goalCell; cell != -1; cell = parent[cell]) {
        jumpCells.push_back(cell);
    }
    std::reverse(jumpCells.begin(), jumpCells.end());
    return expandToRoute(jumpCells);
}

// Walks the straight or diagonal run between consecutive jump points cell by cell. The domain check
// guarantees that every step is an edge; a failure here is an internal error.
std::vector<int> JumpPointSearch::expandToRoute(const std::vector<int>& jumpCells) const {
    const int width = model.width;
    std::vector<int> route{model.cellNode[jumpCells.front()]};
    for (std::size_t i = 1; i < jumpCells.size(); ++i) {
        int x = jumpCells[i - 1] % width;
        int y = jumpCells[i - 1] / width;
        const int toX = jumpCells[i] % width;
        const int toY = jumpCells[i] / width;
        if (x != toX && y != toY && std::abs(toX - x) != std::abs(toY - y)) {
            throw std::logic_error("JumpPointSearch: jump points " + cellName(x, y) + " and " + cellName(toX, toY) +
                                   " are not on one straight or diagonal line");
        }
        const int dx = sign(toX - x);
        const int dy = sign(toY - y);
        while (x != toX || y != toY) {
            if (!canMove(x, y, dx, dy)) {
                throw std::logic_error("JumpPointSearch: illegal move from " + cellName(x, y));
            }
            x += dx;
            y += dy;
            const int node = model.cellNode[static_cast<std::size_t>(y) * width + x];
            if (!graph->hasEdge(route.back(), node)) {
                throw std::logic_error("JumpPointSearch: the route steps from node " + std::to_string(route.back()) +
                                       " to node " + std::to_string(node) + ", which share no edge");
            }
            route.push_back(node);
        }
    }
    return route;
}

std::vector<std::pair<int, int>> JumpPointSearch::getJumpPoints(int startX, int startY, int goalX, int goalY) {
    requireSupportedGraph();
    refreshOpenCells();
    std::vector<std::pair<int, int>> jumpPoints;
    if (!isOpen(startX, startY)) {
        return jumpPoints;
    }
    std::vector<std::pair<int, int>> directions;
    successorDirections(startX, startY, -1, -1, directions);
    for (const auto& [dx, dy] : directions) {
        int jumpX = 0;
        int jumpY = 0;
        if (jump(startX, startY, dx, dy, goalX, goalY, jumpX, jumpY)) {
            jumpPoints.emplace_back(jumpX, jumpY);
        }
    }
    return jumpPoints;
}

void JumpPointSearch::setGridDimensions(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
    gridWidth = width;
    gridHeight = height;
}

void JumpPointSearch::addObstacle(int x, int y) {
    if (x < 0 || y < 0 || x >= gridWidth || y >= gridHeight) {
        throw std::invalid_argument("JumpPointSearch::addObstacle: cell " + cellName(x, y) + " is outside the " +
                                    std::to_string(gridWidth) + " x " + std::to_string(gridHeight) + " grid");
    }
    markedCells.insert(packCell(x, y));
}

void JumpPointSearch::removeObstacle(int x, int y) {
    markedCells.erase(packCell(x, y));
}

void JumpPointSearch::enableDiagonalMovement(bool enable) {
    diagonalMovement = enable;
}

double JumpPointSearch::getPathCost(const std::vector<int>& path) const {
    double cost = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (!graph->hasEdge(path[i - 1], path[i])) {
            throw std::invalid_argument("JumpPointSearch::getPathCost: nodes " + std::to_string(path[i - 1]) + " and " +
                                        std::to_string(path[i]) + " share no edge");
        }
        cost += graph->getEdgeWeight(path[i - 1], path[i]);
    }
    return cost;
}

double JumpPointSearch::getPathLength(const std::vector<int>& path) {
    double totalLength = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        totalLength += graph->getNode(path[i - 1]).euclideanDistance(graph->getNode(path[i]));
    }
    return totalLength;
}

std::size_t JumpPointSearch::getLastSearchExpansions() const {
    return lastExpansions;
}
