#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_set>
#include <utility>

struct ConnectivityMetrics {
    bool isConnected;
    int componentCount;
    int largestComponentSize;
    double averagePathLength;
    int diameter;
    std::vector<std::vector<int>> components;
};

class ConnectivityAnalyzer {
private:
    const Graph* graph;
    
    void depthFirstSearch(int nodeId, std::unordered_set<int>& visited, std::vector<int>& component) const;
    int calculateShortestPathLength(int startId, int endId) const;

public:
    explicit ConnectivityAnalyzer(const Graph* environment);
    
    ConnectivityMetrics analyzeConnectivity() const;
    bool isGraphConnected() const;
    std::vector<std::vector<int>> findConnectedComponents() const;
    
    int getLargestComponentSize() const;
    std::vector<int> getLargestComponent() const;
    // Edges whose removal disconnects their component, as (smaller id, larger id) pairs in ascending
    // order. Edges are treated as undirected: two nodes joined in either direction are one edge.
    std::vector<std::pair<int, int>> getBridgeEdges() const;
    
    double calculateAveragePathLength() const;
    int calculateGraphDiameter() const;
    std::vector<int> findCriticalNodes() const;
    
    bool wouldRemovalDisconnect(int nodeId) const;
    
    void generateConnectivityReport() const;
};