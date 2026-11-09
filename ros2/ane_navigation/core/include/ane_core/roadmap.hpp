#pragma once
// The warehouse roadmap: the engine Graph built from an "ane-roadmap 1" file (generated from the
// warehouse layout by ane_sim/scripts/generate_world.py), the frame that maps graph coordinates to the
// metric map frame, the static obstacles, the cost zones and the named stations.
#include "ane_core/geometry.hpp"
#include "core/Graph.hpp"

#include <map>
#include <string>
#include <vector>

namespace ane {

// map = R(yaw) * (scale * graph) + origin. Graph units are those of the roadmap file (decimetres in
// the warehouse), map units are metres. The engine stores graph units on its Nodes; edge weights are
// metres times the cost-zone multiplier, so they do not depend on the graph units.
class GraphFrame {
public:
    GraphFrame() = default;
    GraphFrame(double scale, double yaw, Point2 origin);  // throws std::invalid_argument unless scale > 0
    Point2 toMap(const Point2& graph) const;
    Point2 toGraph(const Point2& map) const;
    double scale() const { return scale_; }
    double yaw() const { return yaw_; }
    Point2 origin() const { return origin_; }

private:
    double scale_ = 1.0;
    double yaw_ = 0.0;
    Point2 origin_{};
    double cos_ = 1.0;
    double sin_ = 0.0;
};

struct StaticObstacle {
    std::string kind;  // wall, rack or table
    Rect rect;         // map frame
    double height = 0.0;
    std::string label;
};

struct CostZone {
    Rect rect;
    double multiplier = 1.0;
    std::string label;
};

struct RoadmapEdge {
    int from = 0;
    int to = 0;
    double weight = 0.0;
};

class Roadmap {
public:
    // Reads a roadmap file. Throws std::runtime_error naming the source and line for an unknown
    // keyword, a malformed number, a duplicate node, an edge to an unknown node, a non-positive or
    // non-finite weight, a station on an unknown node, or a missing header or frame.
    static Roadmap load(const std::string& path);
    static Roadmap parse(const std::string& text, const std::string& sourceName);

    const std::string& name() const { return name_; }
    const Graph& graph() const { return graph_; }
    const GraphFrame& frame() const { return frame_; }
    const Rect& hall() const { return hall_; }
    const std::vector<StaticObstacle>& obstacles() const { return obstacles_; }
    const std::vector<CostZone>& zones() const { return zones_; }
    const std::vector<RoadmapEdge>& edges() const { return edges_; }
    const std::map<std::string, int>& stations() const { return stations_; }
    int station(const std::string& name) const;  // -1 when unknown
    std::vector<int> nodeIds() const;            // ascending

    bool hasNode(int id) const;
    Point2 nodePosition(int id) const;  // map frame; throws std::out_of_range for an unknown node
    // Nearest node to a map point within maxDistance, or -1. Ties go to the lower id.
    int nearestNode(const Point2& map, double maxDistance, double* distanceOut = nullptr) const;
    std::vector<int> nodesInRect(const Rect& map) const;  // ascending

    // Route measures. routeCost is infinite when a step is not an edge.
    double routeCost(const std::vector<int>& route) const;    // sum of edge weights
    double routeLength(const std::vector<int>& route) const;  // map-frame polyline length, metres
    bool routeFollowsEdges(const std::vector<int>& route) const;

    // Distance from a map point or segment to the nearest static obstacle (walls included).
    double clearance(const Point2& p) const;
    double clearance(const Point2& a, const Point2& b) const;

private:
    std::string name_;
    Graph graph_;
    GraphFrame frame_;
    Rect hall_{};
    std::vector<StaticObstacle> obstacles_;
    std::vector<CostZone> zones_;
    std::vector<RoadmapEdge> edges_;
    std::map<std::string, int> stations_;
    std::map<int, Point2> positions_;  // map frame, cached
};

// Robot footprint used for the clearance contract: a rectangle centred on the rotation axis.
struct Footprint {
    double length = 0.40;  // m, along x
    double width = 0.37;   // m, along y, wheels included
    double circumscribedRadius() const;
};

// Static clearance a roadmap must provide:
//   node >= circumscribed radius + margin                     (the robot may rotate in place at any node)
//   edge >= circumscribed radius + max cross-track + margin   (driving an edge with the allowed tracking error)
struct ClearanceRequirement {
    double node = 0.0;
    double edge = 0.0;
};
ClearanceRequirement clearanceRequirement(const Footprint& footprint, double maxCrossTrack, double margin);

struct ClearanceReport {
    double min_node_clearance = 0.0;
    int worst_node = -1;
    double min_edge_clearance = 0.0;
    int worst_edge_from = -1;
    int worst_edge_to = -1;
    std::vector<std::string> violations;  // empty when every node and edge meets the requirement
};
ClearanceReport checkClearance(const Roadmap& roadmap, const ClearanceRequirement& required);

}  // namespace ane
