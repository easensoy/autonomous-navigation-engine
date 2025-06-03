#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <chrono>

enum class UpdateType {
    NODE_ADDITION,
    NODE_REMOVAL,
    EDGE_ADDITION,
    EDGE_REMOVAL,
    WEIGHT_MODIFICATION,
    FULL_RECONSTRUCTION
};

struct MapUpdate {
    UpdateType type;
    std::vector<int> affectedNodes;
    std::vector<std::pair<int, int>> affectedEdges;
    std::chrono::steady_clock::time_point timestamp;
    
    MapUpdate(UpdateType t);
};

class MapUpdater {
private:
    Graph* environment;
    std::vector<MapUpdate> updateHistory;
    bool realTimeUpdates;
    std::chrono::steady_clock::time_point lastUpdateTime;
    
    void processNodeAddition(const MapUpdate& update);
    void processNodeRemoval(const MapUpdate& update);
    void processEdgeModification(const MapUpdate& update);

public:
    explicit MapUpdater(Graph* graph);
    
    void updateMap(const std::vector<MapUpdate>& updates);
    void addNode(int nodeId, const std::string& name, double x, double y);
    void removeNode(int nodeId);
    void updateEdgeWeight(int fromId, int toId, double newWeight);
    
    void processSensorData(const std::vector<int>& sensorReadings);
    void integrateExternalMap(const Graph& externalMap);
    void synchronizeWithDatabase();
    
    std::vector<MapUpdate> getRecentUpdates(std::chrono::duration<double> timeWindow) const;
    void rollbackUpdate(size_t updateIndex);
    void clearUpdateHistory();
    
    void enableRealTimeUpdates(bool enable);
    bool validateUpdateConsistency() const;
    void optimizeMapStructure();
    
    size_t getUpdateCount() const;
    std::chrono::steady_clock::time_point getLastUpdateTime() const;
};