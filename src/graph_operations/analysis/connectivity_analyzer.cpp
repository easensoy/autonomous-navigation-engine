#include "graph_operations/ConnectivityAnalyzer.hpp"
#include <algorithm>
#include <iostream>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

// ConnectivityAnalyzer treats the graph as undirected (an edge in either direction connects
// two nodes) and measures components, hop distances and cut vertices.

namespace {
std::unordered_map<int, std::vector<int>> undirectedAdjacency(const Graph& graph) {
    std::unordered_map<int, std::vector<int>> adjacency;
    for (int id : graph.getAllNodeIds()) {
        adjacency[id];
        for (const Edge& edge : graph.getEdgesFrom(id)) {
            adjacency[id].push_back(edge.getToNode());
            adjacency[edge.getToNode()].push_back(id);
        }
    }
    for (auto& entry : adjacency) {
        auto& list = entry.second;
        std::sort(list.begin(), list.end());
        list.erase(std::unique(list.begin(), list.end()), list.end());
    }
    return adjacency;
}

// Hop distances from start, skipping 'removed' (-1 means none).
std::unordered_map<int, int> hopDistances(const std::unordered_map<int, std::vector<int>>& adjacency, int start,
                                          int removed = -1) {
    std::unordered_map<int, int> distance{{start, 0}};
    std::queue<int> frontier;
    frontier.push(start);
    while (!frontier.empty()) {
        const int current = frontier.front();
        frontier.pop();
        for (int next : adjacency.at(current)) {
            if (next != removed && distance.find(next) == distance.end()) {
                distance[next] = distance[current] + 1;
                frontier.push(next);
            }
        }
    }
    return distance;
}
}  // namespace

ConnectivityAnalyzer::ConnectivityAnalyzer(const Graph* environment) : graph(environment) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

void ConnectivityAnalyzer::depthFirstSearch(int nodeId, std::unordered_set<int>& visited,
                                            std::vector<int>& component) const {
    const auto adjacency = undirectedAdjacency(*graph);
    std::vector<int> stack{nodeId};
    visited.insert(nodeId);
    while (!stack.empty()) {
        const int current = stack.back();
        stack.pop_back();
        component.push_back(current);
        for (int next : adjacency.at(current)) {
            if (visited.insert(next).second) {
                stack.push_back(next);
            }
        }
    }
}

int ConnectivityAnalyzer::calculateShortestPathLength(int startId, int endId) const {
    if (!graph->hasNode(startId) || !graph->hasNode(endId)) {
        return -1;
    }
    const auto distance = hopDistances(undirectedAdjacency(*graph), startId);
    auto it = distance.find(endId);
    return it == distance.end() ? -1 : it->second;
}

ConnectivityMetrics ConnectivityAnalyzer::analyzeConnectivity() const {
    ConnectivityMetrics metrics{};
    metrics.components = findConnectedComponents();
    metrics.componentCount = static_cast<int>(metrics.components.size());
    metrics.isConnected = metrics.componentCount <= 1;
    for (const auto& component : metrics.components) {
        metrics.largestComponentSize = std::max(metrics.largestComponentSize, static_cast<int>(component.size()));
    }
    metrics.averagePathLength = calculateAveragePathLength();
    metrics.diameter = calculateGraphDiameter();
    return metrics;
}

bool ConnectivityAnalyzer::isGraphConnected() const {
    return findConnectedComponents().size() <= 1;
}

std::vector<std::vector<int>> ConnectivityAnalyzer::findConnectedComponents() const {
    std::vector<int> ids = graph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    std::unordered_set<int> visited;
    std::vector<std::vector<int>> components;
    for (int id : ids) {
        if (visited.count(id)) continue;
        std::vector<int> component;
        depthFirstSearch(id, visited, component);
        std::sort(component.begin(), component.end());
        components.push_back(std::move(component));
    }
    std::sort(components.begin(), components.end(),
              [](const auto& a, const auto& b) { return a.size() > b.size(); });
    return components;
}

int ConnectivityAnalyzer::getLargestComponentSize() const {
    const auto components = findConnectedComponents();
    return components.empty() ? 0 : static_cast<int>(components.front().size());
}

std::vector<int> ConnectivityAnalyzer::getLargestComponent() const {
    const auto components = findConnectedComponents();
    return components.empty() ? std::vector<int>{} : components.front();
}

double ConnectivityAnalyzer::calculateAveragePathLength() const {
    const auto adjacency = undirectedAdjacency(*graph);
    double total = 0.0;
    size_t pairs = 0;
    for (const auto& entry : adjacency) {
        for (const auto& d : hopDistances(adjacency, entry.first)) {
            if (d.first != entry.first) {
                total += d.second;
                ++pairs;
            }
        }
    }
    return pairs == 0 ? 0.0 : total / static_cast<double>(pairs);
}

int ConnectivityAnalyzer::calculateGraphDiameter() const {
    const auto adjacency = undirectedAdjacency(*graph);
    int diameter = 0;
    for (const auto& entry : adjacency) {
        for (const auto& d : hopDistances(adjacency, entry.first)) {
            diameter = std::max(diameter, d.second);
        }
    }
    return diameter;
}

std::vector<int> ConnectivityAnalyzer::findCriticalNodes() const {
    std::vector<int> critical;
    std::vector<int> ids = graph->getAllNodeIds();
    std::sort(ids.begin(), ids.end());
    for (int id : ids) {
        if (wouldRemovalDisconnect(id)) {
            critical.push_back(id);
        }
    }
    return critical;
}

bool ConnectivityAnalyzer::wouldRemovalDisconnect(int nodeId) const {
    if (!graph->hasNode(nodeId)) {
        return false;
    }
    const auto adjacency = undirectedAdjacency(*graph);
    const auto& neighbors = adjacency.at(nodeId);
    if (neighbors.size() < 2) {
        return false;
    }
    // Removing the node disconnects its component if its neighbours can no longer reach each other.
    const auto reached = hopDistances(adjacency, neighbors.front(), nodeId);
    return std::any_of(neighbors.begin(), neighbors.end(),
                       [&reached](int n) { return reached.find(n) == reached.end(); });
}

void ConnectivityAnalyzer::generateConnectivityReport() const {
    const ConnectivityMetrics m = analyzeConnectivity();
    std::cout << "=== Connectivity Report ===" << std::endl;
    std::cout << "Connected: " << (m.isConnected ? "yes" : "no") << std::endl;
    std::cout << "Components: " << m.componentCount << " (largest " << m.largestComponentSize << " nodes)" << std::endl;
    std::cout << "Average shortest path: " << m.averagePathLength << " hops" << std::endl;
    std::cout << "Diameter: " << m.diameter << " hops" << std::endl;
}
