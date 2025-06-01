#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_set>

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
    std::vector<int> getBridgeEdges() const;
    
    double calculateAveragePathLength() const;
    int calculateGraphDiameter() const;
    std::vector<int> findCriticalNodes() const;
    
    bool wouldRemovalDisconnect(int nodeId) const;
    std::vector<std::pair<int, int>> findBottleneckEdges() const;
    double calculateConnectivityRobustness() const;
    
    void generateConnectivityReport() const;
    std::vector<int> suggestAdditionalConnections() const;
    void optimizeConnectivity();
};