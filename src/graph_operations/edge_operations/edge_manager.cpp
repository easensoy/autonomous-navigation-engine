#include "graph_operations/EdgeManager.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <tuple>
#include <utility>

// EdgeManager validates and edits edges and reports simple edge statistics.

namespace {
// Each stored edge once: a two-way edge appears in both adjacency lists, keep the from < to copy.
std::vector<std::tuple<int, int, double>> uniqueEdges(const Graph& graph) {
    std::vector<std::tuple<int, int, double>> edges;
    for (int id : graph.getAllNodeIds()) {
        for (const Edge& edge : graph.getEdgesFrom(id)) {
            if (edge.isBidirectional() && edge.getToNode() < id) {
                continue;
            }
            edges.emplace_back(id, edge.getToNode(), edge.getWeight());
        }
    }
    std::sort(edges.begin(), edges.end());
    return edges;
}
}  // namespace

EdgeManager::EdgeManager(Graph* environment) : graph(environment) {
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

bool EdgeManager::validateEdgeAddition(int fromId, int toId) const {
    return fromId != toId && graph->hasNode(fromId) && graph->hasNode(toId) && !graph->hasEdge(fromId, toId);
}

void EdgeManager::updateConnectivity(int fromId, int toId, bool added) {
    std::cout << "[EDGE_MANAGER] Edge " << fromId << " -> " << toId << (added ? " added" : " removed") << std::endl;
}

bool EdgeManager::addEdge(int fromId, int toId, double weight, bool bidirectional) {
    if (!validateEdgeAddition(fromId, toId) || weight < 0.0 || !std::isfinite(weight)) {
        return false;
    }
    graph->addEdge(fromId, toId, weight, bidirectional);
    updateConnectivity(fromId, toId, true);
    return true;
}

bool EdgeManager::removeEdge(int fromId, int toId) {
    bool bidirectional = false;
    for (const Edge& edge : graph->getEdgesFrom(fromId)) {
        if (edge.getToNode() == toId) {
            bidirectional = edge.isBidirectional();
            break;
        }
    }
    const bool removed = graph->removeEdge(fromId, toId);
    if (removed && bidirectional) {
        graph->removeEdge(toId, fromId);
    }
    if (removed) {
        updateConnectivity(fromId, toId, false);
    }
    return removed;
}

bool EdgeManager::updateEdgeWeight(int fromId, int toId, double newWeight) {
    if (!graph->hasEdge(fromId, toId) || newWeight < 0.0 || !std::isfinite(newWeight)) {
        return false;
    }
    bool bidirectional = true;
    for (const Edge& edge : graph->getEdgesFrom(fromId)) {
        if (edge.getToNode() == toId) {
            bidirectional = edge.isBidirectional();
            break;
        }
    }
    graph->removeEdge(fromId, toId);
    if (bidirectional) {
        graph->removeEdge(toId, fromId);
    }
    graph->addEdge(fromId, toId, newWeight, bidirectional);
    return true;
}

std::vector<std::pair<int, int>> EdgeManager::findShortestEdges(int count) const {
    auto edges = uniqueEdges(*graph);
    std::stable_sort(edges.begin(), edges.end(),
                     [](const auto& a, const auto& b) { return std::get<2>(a) < std::get<2>(b); });
    std::vector<std::pair<int, int>> result;
    for (int i = 0; i < count && i < static_cast<int>(edges.size()); ++i) {
        result.emplace_back(std::get<0>(edges[i]), std::get<1>(edges[i]));
    }
    return result;
}

std::vector<std::pair<int, int>> EdgeManager::findLongestEdges(int count) const {
    auto edges = uniqueEdges(*graph);
    std::stable_sort(edges.begin(), edges.end(),
                     [](const auto& a, const auto& b) { return std::get<2>(a) > std::get<2>(b); });
    std::vector<std::pair<int, int>> result;
    for (int i = 0; i < count && i < static_cast<int>(edges.size()); ++i) {
        result.emplace_back(std::get<0>(edges[i]), std::get<1>(edges[i]));
    }
    return result;
}

double EdgeManager::getTotalEdgeWeight() const {
    double total = 0.0;
    for (const auto& edge : uniqueEdges(*graph)) {
        total += std::get<2>(edge);
    }
    return total;
}

double EdgeManager::getAverageEdgeWeight() const {
    const auto edges = uniqueEdges(*graph);
    return edges.empty() ? 0.0 : getTotalEdgeWeight() / static_cast<double>(edges.size());
}

std::pair<int, int> EdgeManager::getHeaviestEdge() const {
    std::pair<int, int> heaviest{-1, -1};
    double best = -1.0;
    for (const auto& edge : uniqueEdges(*graph)) {
        if (std::get<2>(edge) > best) {
            best = std::get<2>(edge);
            heaviest = {std::get<0>(edge), std::get<1>(edge)};
        }
    }
    return heaviest;
}

size_t EdgeManager::getEdgeCount() const {
    return uniqueEdges(*graph).size();
}

bool EdgeManager::validateEdgeConsistency() const {
    for (int id : graph->getAllNodeIds()) {
        for (const Edge& edge : graph->getEdgesFrom(id)) {
            if (edge.getFromNode() != id || !graph->hasNode(edge.getToNode()) || edge.getWeight() < 0.0) {
                return false;
            }
            if (edge.isBidirectional() && !graph->hasEdge(edge.getToNode(), id)) {
                return false;
            }
        }
    }
    return true;
}

std::vector<std::pair<int, int>> EdgeManager::getMinimumSpanningTree() const {
    std::map<std::pair<int, int>, double> candidates;  // undirected pair -> cheaper weight
    for (int id : graph->getAllNodeIds()) {
        for (const Edge& edge : graph->getEdgesFrom(id)) {
            const int to = edge.getToNode();
            if (to == id) {
                continue;  // a self-loop never joins two trees
            }
            const auto key = std::make_pair(std::min(id, to), std::max(id, to));
            const auto [entry, added] = candidates.emplace(key, edge.getWeight());
            if (!added) {
                entry->second = std::min(entry->second, edge.getWeight());
            }
        }
    }
    std::vector<std::tuple<double, int, int>> ordered;
    ordered.reserve(candidates.size());
    for (const auto& [pair, weight] : candidates) {
        ordered.emplace_back(weight, pair.first, pair.second);
    }
    std::sort(ordered.begin(), ordered.end());

    std::unordered_map<int, int> parent;  // union-find with path halving
    for (int id : graph->getAllNodeIds()) {
        parent[id] = id;
    }
    auto root = [&parent](int node) {
        while (parent[node] != node) {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        return node;
    };
    std::vector<std::pair<int, int>> forest;
    for (const auto& [weight, a, b] : ordered) {
        const int rootA = root(a);
        const int rootB = root(b);
        if (rootA != rootB) {
            parent[rootA] = rootB;
            forest.emplace_back(a, b);
        }
    }
    return forest;
}
