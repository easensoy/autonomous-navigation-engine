#include "graph_operations/GraphBuilder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// GraphBuilder's public interface; the grid, random, Delaunay and visibility builders live in their
// own sources. The contracts are in GraphBuilder.hpp.

void generateMaze(Graph* graph, int width, int height, double spacing);  // grid_graph_generator.cpp

namespace {

double distanceBetween(const Node& a, const Node& b) {
    return std::hypot(a.getX() - b.getX(), a.getY() - b.getY());
}

bool joined(const Graph& graph, int a, int b) {
    return graph.hasEdge(a, b) || graph.hasEdge(b, a);
}

std::string parameterError(const std::string& name, const std::string& requirement) {
    return "GraphBuilder::buildGraph: parameter '" + name + "' must be " + requirement;
}

// The value of a parameter that must be a whole number in [low, high].
long long wholeParameter(const std::unordered_map<std::string, double>& parameters, const std::string& name,
                         long long low, long long high) {
    const auto found = parameters.find(name);
    if (found == parameters.end()) {
        throw std::invalid_argument("GraphBuilder::buildGraph: missing parameter '" + name + "'");
    }
    const double value = found->second;
    if (!std::isfinite(value) || value != std::floor(value) || value < static_cast<double>(low) ||
        value > static_cast<double>(high)) {
        throw std::invalid_argument(parameterError(name, "a whole number from " + std::to_string(low) + " to " +
                                                             std::to_string(high)));
    }
    return static_cast<long long>(value);
}

void rejectUnknownParameters(const std::unordered_map<std::string, double>& parameters,
                             const std::vector<std::string>& known) {
    for (const auto& entry : parameters) {
        if (std::find(known.begin(), known.end(), entry.first) == known.end()) {
            throw std::invalid_argument("GraphBuilder::buildGraph: unknown parameter '" + entry.first + "'");
        }
    }
}

}  // namespace

GraphBuilder::GraphBuilder(Graph* graph) : targetGraph(graph) {
    if (!targetGraph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

void GraphBuilder::buildGraph(GraphType type, const std::unordered_map<std::string, double>& parameters) {
    if (!targetGraph->isEmpty()) {
        throw std::invalid_argument("GraphBuilder::buildGraph needs an empty graph");
    }
    constexpr long long kMaxSide = 1 << 15;
    switch (type) {
        case GraphType::GRID: {
            rejectUnknownParameters(parameters, {"width", "height", "spacing"});
            const long long width = wholeParameter(parameters, "width", 1, kMaxSide);
            const long long height = wholeParameter(parameters, "height", 1, kMaxSide);
            const auto spacingEntry = parameters.find("spacing");
            const double spacing = spacingEntry == parameters.end() ? 1.0 : spacingEntry->second;
            if (!std::isfinite(spacing) || spacing <= 0.0) {
                throw std::invalid_argument(parameterError("spacing", "positive and finite"));
            }
            buildGridGraph(static_cast<int>(width), static_cast<int>(height), spacing);
            return;
        }
        case GraphType::RANDOM: {
            rejectUnknownParameters(parameters, {"nodeCount", "connectionProbability", "seed"});
            const long long nodeCount = wholeParameter(parameters, "nodeCount", 1, kMaxSide);
            const auto probability = parameters.find("connectionProbability");
            if (probability == parameters.end()) {
                throw std::invalid_argument("GraphBuilder::buildGraph: missing parameter 'connectionProbability'");
            }
            if (!(probability->second >= 0.0 && probability->second <= 1.0)) {
                throw std::invalid_argument(parameterError("connectionProbability", "in [0, 1]"));
            }
            const std::uint32_t seed =
                parameters.count("seed")
                    ? static_cast<std::uint32_t>(wholeParameter(parameters, "seed", 0, std::numeric_limits<std::uint32_t>::max()))
                    : std::random_device{}();
            buildRandomGraph(static_cast<int>(nodeCount), probability->second, seed);
            return;
        }
    }
    throw std::invalid_argument("GraphBuilder::buildGraph: unknown graph type");
}

void GraphBuilder::addNodesFromCoordinates(const std::vector<std::pair<double, double>>& coordinates) {
    for (const auto& [x, y] : coordinates) {
        if (!std::isfinite(x) || !std::isfinite(y)) {
            throw std::invalid_argument("GraphBuilder::addNodesFromCoordinates: coordinates must be finite");
        }
    }
    const std::vector<int> ids = targetGraph->getAllNodeIds();
    int next = ids.empty() ? 0 : *std::max_element(ids.begin(), ids.end()) + 1;
    for (const auto& [x, y] : coordinates) {
        targetGraph->addNode(next, "node_" + std::to_string(next), x, y);
        ++next;
    }
}

void GraphBuilder::connectNearestNeighbors(int k) {
    if (k < 1) {
        throw std::invalid_argument("GraphBuilder::connectNearestNeighbors: k must be at least 1");
    }
    std::vector<int> ids = targetGraph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    // Choose every node's neighbours first, from the graph as given, then add the edges.
    std::vector<std::pair<int, int>> pairs;
    for (int id : ids) {
        const Node& node = targetGraph->getNode(id);
        std::vector<std::pair<double, int>> others;
        for (int other : ids) {
            if (other != id) {
                others.emplace_back(distanceBetween(node, targetGraph->getNode(other)), other);
            }
        }
        const std::size_t count = std::min<std::size_t>(static_cast<std::size_t>(k), others.size());
        std::partial_sort(others.begin(), others.begin() + static_cast<std::ptrdiff_t>(count), others.end());
        for (std::size_t i = 0; i < count; ++i) {
            pairs.emplace_back(std::min(id, others[i].second), std::max(id, others[i].second));
        }
    }
    std::sort(pairs.begin(), pairs.end());
    pairs.erase(std::unique(pairs.begin(), pairs.end()), pairs.end());
    for (const auto& [a, b] : pairs) {
        if (!joined(*targetGraph, a, b)) {
            targetGraph->addEdge(a, b, distanceBetween(targetGraph->getNode(a), targetGraph->getNode(b)), true);
        }
    }
}

void GraphBuilder::connectWithinRadius(double radius) {
    if (!std::isfinite(radius) || radius < 0.0) {
        throw std::invalid_argument("GraphBuilder::connectWithinRadius: radius must be finite and non-negative");
    }
    std::vector<int> ids = targetGraph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    for (std::size_t i = 0; i < ids.size(); ++i) {
        for (std::size_t j = i + 1; j < ids.size(); ++j) {
            const double distance = distanceBetween(targetGraph->getNode(ids[i]), targetGraph->getNode(ids[j]));
            if (distance <= radius && !joined(*targetGraph, ids[i], ids[j])) {
                targetGraph->addEdge(ids[i], ids[j], distance, true);
            }
        }
    }
}

void GraphBuilder::generateMazeLayout(int width, int height) {
    if (!targetGraph->isEmpty()) {
        throw std::invalid_argument("GraphBuilder::generateMazeLayout needs an empty graph");
    }
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("GraphBuilder::generateMazeLayout: width and height must be positive");
    }
    generateMaze(targetGraph, width, height, 1.0);
}
