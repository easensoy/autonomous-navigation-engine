#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_set>
#include <chrono>

enum class ObstacleType {
    STATIC,
    DYNAMIC,
    TEMPORARY,
    PERMANENT
};

struct Obstacle {
    int nodeId;
    ObstacleType type;
    double severity;
    std::chrono::steady_clock::time_point detectionTime;
    
    Obstacle(int id, ObstacleType t, double s);
};

class ObstacleDetector {
private:
    const Graph* environment;
    std::vector<Obstacle> detectedObstacles;
    std::unordered_set<int> blockedNodes;
    double detectionSensitivity;
    
    bool isNodeAccessible(int nodeId) const;
    void updateObstacleStatus();

public:
    explicit ObstacleDetector(const Graph* graph);
    
    std::vector<int> detectObstacles();
    std::vector<int> detectObstaclesInRegion(const std::vector<int>& regionNodes);
    void addStaticObstacle(int nodeId);
    void addDynamicObstacle(int nodeId, double severity);
    
    void removeObstacle(int nodeId);
    void clearAllObstacles();
    bool isNodeBlocked(int nodeId) const;
    
    std::vector<int> getBlockedNodes() const;
    std::vector<Obstacle> getAllObstacles() const;
    std::vector<int> getObstaclesNearPath(const std::vector<int>& path) const;
    
    void setDetectionSensitivity(double sensitivity);
    void enableContinuousDetection(bool enable);
    void updateObstacleMap(const std::vector<int>& sensorData);
    
    double getObstacleDensity() const;
    bool hasObstaclesChanged() const;
};