#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_map>
#include <functional>
#include <string>
#include <utility>

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
    struct EdgeKeyHash {
        size_t operator()(const std::pair<int, int>& key) const noexcept {
            return std::hash<long long>()((static_cast<long long>(key.first) << 32) ^
                                          static_cast<unsigned int>(key.second));
        }
    };
    std::unordered_map<std::pair<int, int>, double, EdgeKeyHash> edgeCosts;

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
    
    // The model (cost weights, node costs, edge costs) as text: a header line "cost-model 1", then
    // one entry per line, "weight <TYPE> <value>", "node <id> <cost>" or "edge <from> <to> <cost>",
    // with values written to round-trip exactly. loadCostModel replaces the model: it starts from the
    // defaults (resetCostModel) and applies every entry. Both throw std::runtime_error on an I/O
    // error; loadCostModel also throws on a malformed line, naming it, and then leaves the model unchanged.
    void loadCostModel(const std::string& modelFile);
    void saveCostModel(const std::string& modelFile) const;
    void resetCostModel();
    
    std::vector<double> calculateCostProfile(const std::vector<int>& path) const;
    int findCostliestSegment(const std::vector<int>& path) const;
    double calculateCostVariance(const std::vector<int>& path) const;
};