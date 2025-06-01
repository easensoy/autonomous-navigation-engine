#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_map>

enum class CostType {
    DISTANCE,
    TIME,
    ENERGY,
    RISK,
    MONETARY,
    COMPOSITE
};

struct CostBreakdown {
    double totalCost;
    double distanceCost;
    double timeCost;
    double energyCost;
    double riskCost;
    std::unordered_map<std::string, double> customCosts;
};

class CostCalculator {
private:
    const Graph* environment;
    std::unordered_map<CostType, double> costWeights;
    std::unordered_map<int, double> nodeCosts;
    std::unordered_map<std::pair<int, int>, double> edgeCosts;

public:
    explicit CostCalculator(const Graph* graph);
    
    double calculatePathCost(const std::vector<int>& path) const;
    double calculatePathCostWithType(const std::vector<int>& path, CostType costType) const;
    CostBreakdown calculateDetailedCost(const std::vector<int>& path) const;
    
    void setCostWeight(CostType costType, double weight);
    void setNodeCost(int nodeId, double cost);
    void setEdgeCost(int fromId, int toId, double cost);
    
    double getSegmentCost(int fromId, int toId) const;
    double getNodeTraversalCost(int nodeId) const;
    double estimateCostToGoal(int currentId, int goalId) const;
    
    void loadCostModel(const std::string& modelFile);
    void saveCostModel(const std::string& modelFile) const;
    void resetCostModel();
    
    std::vector<double> calculateCostProfile(const std::vector<int>& path) const;
    int findCostliestSegment(const std::vector<int>& path) const;
    double calculateCostVariance(const std::vector<int>& path) const;
};