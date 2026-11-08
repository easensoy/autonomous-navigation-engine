#pragma once
#include "core/Graph.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// Graph shapes buildGraph creates from numeric parameters.
enum class GraphType {
    GRID,   // width, height (positive integers); spacing (positive, default 1)
    RANDOM  // nodeCount (positive integer); connectionProbability in [0, 1]; seed (optional, 0 .. 2^32 - 1)
};

// Builds or extends a graph in place. Distances and edge weights here are straight-line coordinate
// distances (the convention NodeManager uses), not Node::euclideanDistance, which is 0.1 x that.
class GraphBuilder {
private:
    Graph* targetGraph;

    void buildGridGraph(int width, int height, double spacing);
    void buildRandomGraph(int nodeCount, double connectionProbability, std::uint32_t seed);
    void buildDelaunayGraph(const std::vector<std::pair<double, double>>& points);
    void buildVisibilityGraph(const std::vector<std::pair<double, double>>& points,
                              const std::vector<std::vector<std::pair<double, double>>>& obstacles);

public:
    // Throws std::invalid_argument for a null graph.
    explicit GraphBuilder(Graph* graph);

    // Populates an empty graph. Throws std::invalid_argument if the graph has nodes, or for a missing,
    // unknown or out-of-range parameter.
    //   GRID: nodes y * width + x at (x * spacing, y * spacing), joined to their orthogonal neighbours
    //         by two-way edges of weight spacing.
    //   RANDOM: Erdos-Renyi G(n, p). Nodes 0 .. nodeCount - 1 placed uniformly in [0, 100) x [0, 100);
    //         each pair joined with probability connectionProbability by a two-way edge weighted by
    //         its distance. A given seed reproduces the graph with the same standard library; without
    //         one the generator is seeded from std::random_device.
    void buildGraph(GraphType type, const std::unordered_map<std::string, double>& parameters);

    // Adds one node per coordinate pair, with ids counting up from the largest existing id + 1 (0 for
    // an empty graph) and names "node_<id>". Throws std::invalid_argument, adding nothing, if a
    // coordinate is not finite.
    void addNodesFromCoordinates(const std::vector<std::pair<double, double>>& coordinates);

    // Joins each node to its k nearest other nodes (ties broken by the smaller id) by two-way edges
    // weighted by their distance. Pairs already joined in either direction are left unchanged.
    // Throws std::invalid_argument for k < 1.
    void connectNearestNeighbors(int k);

    // Joins every pair of nodes at distance <= radius by a two-way edge weighted by the distance.
    // Pairs already joined in either direction are left unchanged. Throws std::invalid_argument for a
    // negative or non-finite radius.
    void connectWithinRadius(double radius);

    // Populates an empty graph with a width x height grid (spacing 1) reduced to a spanning tree by a
    // randomised depth-first search (std::rand): a maze with exactly one route between any two cells.
    // Throws std::invalid_argument if the graph has nodes or a dimension is not positive.
    void generateMazeLayout(int width, int height);
};
