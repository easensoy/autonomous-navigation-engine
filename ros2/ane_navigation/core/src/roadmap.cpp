#include "ane_core/roadmap.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ane {

GraphFrame::GraphFrame(double scale, double yaw, Point2 origin)
    : scale_(scale), yaw_(yaw), origin_(origin), cos_(std::cos(yaw)), sin_(std::sin(yaw)) {
    if (!(scale > 0.0) || !std::isfinite(scale) || !std::isfinite(yaw) || !std::isfinite(origin.x) ||
        !std::isfinite(origin.y)) {
        throw std::invalid_argument("graph frame needs a positive, finite scale and a finite yaw and origin");
    }
}

Point2 GraphFrame::toMap(const Point2& graph) const {
    const double gx = scale_ * graph.x;
    const double gy = scale_ * graph.y;
    return {cos_ * gx - sin_ * gy + origin_.x, sin_ * gx + cos_ * gy + origin_.y};
}

Point2 GraphFrame::toGraph(const Point2& map) const {
    const double dx = map.x - origin_.x;
    const double dy = map.y - origin_.y;
    return {(cos_ * dx + sin_ * dy) / scale_, (-sin_ * dx + cos_ * dy) / scale_};
}

namespace {

double number(const std::string& token, const std::string& where) {
    std::size_t used = 0;
    double value = 0.0;
    try {
        value = std::stod(token, &used);
    } catch (const std::exception&) {
        used = 0;
    }
    if (used != token.size() || !std::isfinite(value)) {
        throw std::runtime_error(where + ": '" + token + "' is not a finite number");
    }
    return value;
}

int integer(const std::string& token, const std::string& where) {
    const double value = number(token, where);
    if (value != std::floor(value) || std::abs(value) > 1e9) {
        throw std::runtime_error(where + ": '" + token + "' is not an integer id");
    }
    return static_cast<int>(value);
}

void need(const std::vector<std::string>& args, std::size_t count, const std::string& where) {
    if (args.size() < count) {
        throw std::runtime_error(where + ": expected " + std::to_string(count) + " values");
    }
}

Rect rectFrom(const std::vector<std::string>& args, std::size_t first, const std::string& where) {
    Rect r{number(args[first], where), number(args[first + 1], where), number(args[first + 2], where),
           number(args[first + 3], where)};
    if (!(r.min_x < r.max_x) || !(r.min_y < r.max_y)) {
        throw std::runtime_error(where + ": rectangle has no area");
    }
    return r;
}

}  // namespace

Roadmap Roadmap::load(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("cannot open roadmap file " + path);
    }
    std::ostringstream text;
    text << in.rdbuf();
    return parse(text.str(), path);
}

Roadmap Roadmap::parse(const std::string& text, const std::string& sourceName) {
    Roadmap map;
    std::istringstream lines(text);
    std::string raw;
    int lineNumber = 0;
    bool header = false;
    bool haveFrame = false;
    bool haveHall = false;
    std::vector<std::pair<std::string, std::pair<int, std::string>>> stationLines;  // name, (node, where)
    struct PendingEdge { int from; int to; double weight; std::string where; };
    std::vector<PendingEdge> pendingEdges;
    std::map<int, Point2> graphPoints;
    std::map<int, std::string> nodeWhere;

    while (std::getline(lines, raw)) {
        ++lineNumber;
        const std::string where = sourceName + ":" + std::to_string(lineNumber);
        const std::size_t hash = raw.find('#');
        std::istringstream fields(hash == std::string::npos ? raw : raw.substr(0, hash));
        std::string key;
        if (!(fields >> key)) {
            continue;
        }
        std::vector<std::string> args;
        for (std::string token; fields >> token;) {
            args.push_back(token);
        }
        if (!header) {
            if (key != "ane-roadmap" || args.size() != 1 || args[0] != "1") {
                throw std::runtime_error(where + ": expected the header 'ane-roadmap 1'");
            }
            header = true;
            continue;
        }
        if (key == "name") {
            need(args, 1, where);
            map.name_ = args[0];
        } else if (key == "frame") {
            need(args, 4, where);
            try {
                map.frame_ = GraphFrame(number(args[0], where), number(args[1], where),
                                        {number(args[2], where), number(args[3], where)});
            } catch (const std::invalid_argument& error) {
                throw std::runtime_error(where + ": " + error.what());
            }
            haveFrame = true;
        } else if (key == "hall") {
            need(args, 4, where);
            map.hall_ = rectFrom(args, 0, where);
            haveHall = true;
        } else if (key == "obstacle") {
            need(args, 7, where);
            StaticObstacle obstacle{args[0], rectFrom(args, 1, where), number(args[5], where), args[6]};
            if (!(obstacle.height > 0.0)) {
                throw std::runtime_error(where + ": obstacle height must be positive");
            }
            map.obstacles_.push_back(obstacle);
        } else if (key == "zone") {
            need(args, 6, where);
            CostZone zone{rectFrom(args, 0, where), number(args[4], where), args[5]};
            if (!(zone.multiplier > 0.0)) {
                throw std::runtime_error(where + ": zone multiplier must be positive");
            }
            map.zones_.push_back(zone);
        } else if (key == "station") {
            need(args, 2, where);
            stationLines.push_back({args[0], {integer(args[1], where), where}});
        } else if (key == "node") {
            need(args, 3, where);
            const int id = integer(args[0], where);
            if (graphPoints.count(id)) {
                throw std::runtime_error(where + ": duplicate node " + std::to_string(id));
            }
            graphPoints[id] = {number(args[1], where), number(args[2], where)};
            nodeWhere[id] = where;
        } else if (key == "edge") {
            need(args, 3, where);
            pendingEdges.push_back({integer(args[0], where), integer(args[1], where), number(args[2], where), where});
        } else {
            throw std::runtime_error(where + ": unknown keyword '" + key + "'");
        }
    }
    if (!header) {
        throw std::runtime_error(sourceName + ": empty roadmap (no 'ane-roadmap 1' header)");
    }
    if (!haveFrame || !haveHall) {
        throw std::runtime_error(sourceName + ": a roadmap needs 'frame' and 'hall' lines");
    }

    // Graph::addNode and addEdge print a line each; keep the roadmap load quiet.
    struct QuietStdout {
        std::ostringstream sink;
        std::streambuf* saved = std::cout.rdbuf(sink.rdbuf());
        ~QuietStdout() { std::cout.rdbuf(saved); }
    } quiet;

    std::map<int, std::string> names;
    for (const auto& entry : stationLines) {
        const int node = entry.second.first;
        if (!graphPoints.count(node)) {
            throw std::runtime_error(entry.second.second + ": station " + entry.first + " is on unknown node " +
                                     std::to_string(node));
        }
        map.stations_[entry.first] = node;
        names[node] = entry.first;
    }
    for (const auto& [id, point] : graphPoints) {
        const auto named = names.find(id);
        map.graph_.addNode(id, named != names.end() ? named->second : "n" + std::to_string(id), point.x, point.y);
        map.positions_[id] = map.frame_.toMap(point);
    }
    std::set<std::pair<int, int>> seen;
    for (const PendingEdge& edge : pendingEdges) {
        if (!graphPoints.count(edge.from) || !graphPoints.count(edge.to) || edge.from == edge.to) {
            throw std::runtime_error(edge.where + ": edge joins an unknown node or a node to itself");
        }
        if (!(edge.weight > 0.0)) {
            throw std::runtime_error(edge.where + ": edge weight must be positive");
        }
        const std::pair<int, int> key{std::min(edge.from, edge.to), std::max(edge.from, edge.to)};
        if (!seen.insert(key).second) {
            throw std::runtime_error(edge.where + ": duplicate edge");
        }
        map.graph_.addEdge(edge.from, edge.to, edge.weight, true);
        map.edges_.push_back({edge.from, edge.to, edge.weight});
    }
    return map;
}

int Roadmap::station(const std::string& name) const {
    const auto found = stations_.find(name);
    return found == stations_.end() ? -1 : found->second;
}

std::vector<int> Roadmap::nodeIds() const {
    std::vector<int> ids;
    ids.reserve(positions_.size());
    for (const auto& entry : positions_) {
        ids.push_back(entry.first);
    }
    return ids;
}

bool Roadmap::hasNode(int id) const {
    return positions_.count(id) != 0;
}

Point2 Roadmap::nodePosition(int id) const {
    return positions_.at(id);
}

int Roadmap::nearestNode(const Point2& map, double maxDistance, double* distanceOut) const {
    int best = -1;
    double bestDistance = std::numeric_limits<double>::infinity();
    for (const auto& [id, position] : positions_) {
        const double d = distance(map, position);
        if (d < bestDistance) {
            bestDistance = d;
            best = id;
        }
    }
    if (distanceOut) {
        *distanceOut = bestDistance;
    }
    return bestDistance <= maxDistance ? best : -1;
}

std::vector<int> Roadmap::nodesInRect(const Rect& map) const {
    std::vector<int> inside;
    for (const auto& [id, position] : positions_) {
        if (map.contains(position)) {
            inside.push_back(id);
        }
    }
    return inside;
}

bool Roadmap::routeFollowsEdges(const std::vector<int>& route) const {
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        if (!graph_.hasNode(route[i]) || !graph_.hasEdge(route[i], route[i + 1])) {
            return false;
        }
    }
    return route.empty() || graph_.hasNode(route.back());
}

double Roadmap::routeCost(const std::vector<int>& route) const {
    if (!routeFollowsEdges(route)) {
        return std::numeric_limits<double>::infinity();
    }
    double cost = 0.0;
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        cost += graph_.getEdgeWeight(route[i], route[i + 1]);
    }
    return cost;
}

double Roadmap::routeLength(const std::vector<int>& route) const {
    double length = 0.0;
    for (std::size_t i = 0; i + 1 < route.size(); ++i) {
        length += distance(nodePosition(route[i]), nodePosition(route[i + 1]));
    }
    return length;
}

double Roadmap::clearance(const Point2& p) const {
    double best = std::numeric_limits<double>::infinity();
    for (const StaticObstacle& obstacle : obstacles_) {
        best = std::min(best, distanceToRect(p, obstacle.rect));
    }
    return best;
}

double Roadmap::clearance(const Point2& a, const Point2& b) const {
    double best = std::numeric_limits<double>::infinity();
    for (const StaticObstacle& obstacle : obstacles_) {
        best = std::min(best, segmentRectDistance(a, b, obstacle.rect));
    }
    return best;
}

double Footprint::circumscribedRadius() const {
    return std::hypot(length / 2.0, width / 2.0);
}

ClearanceRequirement clearanceRequirement(const Footprint& footprint, double maxCrossTrack, double margin) {
    const double radius = footprint.circumscribedRadius();
    return {radius + margin, radius + maxCrossTrack + margin};
}

ClearanceReport checkClearance(const Roadmap& roadmap, const ClearanceRequirement& required) {
    ClearanceReport report;
    report.min_node_clearance = std::numeric_limits<double>::infinity();
    report.min_edge_clearance = std::numeric_limits<double>::infinity();
    for (int id : roadmap.nodeIds()) {
        const double c = roadmap.clearance(roadmap.nodePosition(id));
        if (c < report.min_node_clearance) {
            report.min_node_clearance = c;
            report.worst_node = id;
        }
        if (c < required.node) {
            std::ostringstream message;
            message << "node " << id << " has clearance " << c << " m < " << required.node << " m";
            report.violations.push_back(message.str());
        }
    }
    for (const RoadmapEdge& edge : roadmap.edges()) {
        const double c = roadmap.clearance(roadmap.nodePosition(edge.from), roadmap.nodePosition(edge.to));
        if (c < report.min_edge_clearance) {
            report.min_edge_clearance = c;
            report.worst_edge_from = edge.from;
            report.worst_edge_to = edge.to;
        }
        if (c < required.edge) {
            std::ostringstream message;
            message << "edge " << edge.from << "-" << edge.to << " has clearance " << c << " m < " << required.edge << " m";
            report.violations.push_back(message.str());
        }
    }
    return report;
}

}  // namespace ane
