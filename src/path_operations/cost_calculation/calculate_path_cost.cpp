#include "path_operations/CostCalculator.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <limits>

class PathCostCalculator {
private:
    const Graph* graph;
    
    struct CostComponents {
        double baseCost;
        double distanceCost;
        double timeCost;
        double energyCost;
        double riskCost;
        double congestionCost;
        double elevationCost;
        double weatherCost;
        
        CostComponents() : baseCost(0.0), distanceCost(0.0), timeCost(0.0), 
                          energyCost(0.0), riskCost(0.0), congestionCost(0.0),
                          elevationCost(0.0), weatherCost(0.0) {}
        
        double getTotalCost() const {
            return baseCost + distanceCost + timeCost + energyCost + 
                   riskCost + congestionCost + elevationCost + weatherCost;
        }
    };
    
    struct CostProfile {
        std::vector<double> segmentCosts;
        std::vector<CostComponents> detailedComponents;
        double totalCost;
        double averageCostPerUnit;
        double maxSegmentCost;
        double minSegmentCost;
        int mostExpensiveSegment;
        
        CostProfile() : totalCost(0.0), averageCostPerUnit(0.0), 
                       maxSegmentCost(0.0), minSegmentCost(std::numeric_limits<double>::infinity()),
                       mostExpensiveSegment(-1) {}
    };
    
    // Cost calculation parameters
    double distanceWeightFactor;
    double timeWeightFactor;
    double energyWeightFactor;
    double riskWeightFactor;
    bool enableDetailedAnalysis;
    bool cacheCalculations;
    
    // Cost modifiers and tables
    std::unordered_map<int, double> nodeCostModifiers;
    std::unordered_map<std::string, double> edgeCostModifiers;
    std::unordered_map<std::string, double> terrainCostFactors;
    std::unordered_map<int, double> elevationPenalties;
    
    // Cached results
    mutable std::unordered_map<std::string, double> costCache;
    
    std::string generateCacheKey(const std::vector<int>& path) const {
        std::string key;
        for (size_t i = 0; i < path.size(); ++i) {
            key += std::to_string(path[i]);
            if (i < path.size() - 1) key += "_";
        }
        return key;
    }
    
    double calculateBasicDistance(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalDistance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalDistance += from.euclideanDistance(to);
        }
        
        return totalDistance;
    }
    
    double calculateEdgeCost(int fromNodeId, int toNodeId) const {
        const Node& fromNode = graph->getNode(fromNodeId);
        const Node& toNode = graph->getNode(toNodeId);
        
        double baseDistance = fromNode.euclideanDistance(toNode);
        double edgeWeight = graph->getEdgeWeight(fromNodeId, toNodeId);
        
        // Use edge weight if available, otherwise use euclidean distance
        double baseCost = (edgeWeight > 0) ? edgeWeight : baseDistance;
        
        // Apply edge-specific modifiers
        std::string edgeKey = std::to_string(fromNodeId) + "_" + std::to_string(toNodeId);
        auto edgeModifier = edgeCostModifiers.find(edgeKey);
        if (edgeModifier != edgeCostModifiers.end()) {
            baseCost *= edgeModifier->second;
        }
        
        return baseCost;
    }
    
    CostComponents calculateDetailedSegmentCost(int fromNodeId, int toNodeId) const {
        CostComponents components;
        
        const Node& fromNode = graph->getNode(fromNodeId);
        const Node& toNode = graph->getNode(toNodeId);
        
        // Base distance cost
        components.distanceCost = fromNode.euclideanDistance(toNode) * distanceWeightFactor;
        
        // Time cost estimation
        double estimatedSpeed = 1.0; // Base speed units per time
        components.timeCost = (components.distanceCost / estimatedSpeed) * timeWeightFactor;
        
        // Energy cost based on distance and terrain
        components.energyCost = components.distanceCost * energyWeightFactor;
        
        // Apply terrain factors if available
        std::string terrainType = determineTerrainType(fromNodeId, toNodeId);
        auto terrainFactor = terrainCostFactors.find(terrainType);
        if (terrainFactor != terrainCostFactors.end()) {
            components.energyCost *= terrainFactor->second;
        }
        
        // Elevation change penalty
        components.elevationCost = calculateElevationPenalty(fromNodeId, toNodeId);
        
        // Risk assessment
        components.riskCost = calculateRiskFactor(fromNodeId, toNodeId) * riskWeightFactor;
        
        // Node-specific cost modifiers
        auto fromModifier = nodeCostModifiers.find(fromNodeId);
        auto toModifier = nodeCostModifiers.find(toNodeId);
        
        double nodeModifier = 1.0;
        if (fromModifier != nodeCostModifiers.end()) {
            nodeModifier *= fromModifier->second;
        }
        if (toModifier != nodeCostModifiers.end()) {
            nodeModifier *= toModifier->second;
        }
        
        // Apply node modifiers to all components
        components.distanceCost *= nodeModifier;
        components.timeCost *= nodeModifier;
        components.energyCost *= nodeModifier;
        components.riskCost *= nodeModifier;
        
        components.baseCost = components.getTotalCost();
        
        return components;
    }
    
    std::string determineTerrainType(int fromNodeId, int toNodeId) const {
        // Simplified terrain determination based on node positions
        const Node& fromNode = graph->getNode(fromNodeId);
        const Node& toNode = graph->getNode(toNodeId);
        
        double avgX = (fromNode.getX() + toNode.getX()) / 2.0;
        double avgY = (fromNode.getY() + toNode.getY()) / 2.0;
        
        // Simple heuristic terrain classification
        if (avgX < 10 && avgY < 10) return "urban";
        else if (avgX > 50 || avgY > 50) return "mountain";
        else if (std::abs(avgX - avgY) < 5) return "flat";
        else return "mixed";
    }
    
    double calculateElevationPenalty(int fromNodeId, int toNodeId) const {
        // Simplified elevation penalty calculation
        auto fromElevation = elevationPenalties.find(fromNodeId);
        auto toElevation = elevationPenalties.find(toNodeId);
        
        if (fromElevation == elevationPenalties.end() || toElevation == elevationPenalties.end()) {
            return 0.0; // No elevation data available
        }
        
        double elevationChange = std::abs(toElevation->second - fromElevation->second);
        return elevationChange * 0.1; // Penalty factor for elevation change
    }
    
    double calculateRiskFactor(int fromNodeId, int toNodeId) const {
        // Risk assessment based on node characteristics and path properties
        double riskScore = 0.0;
        
        // Higher risk for nodes with fewer connections (less alternative routes)
        std::vector<int> fromNeighbors = graph->getNeighbors(fromNodeId);
        std::vector<int> toNeighbors = graph->getNeighbors(toNodeId);
        
        if (fromNeighbors.size() < 3) riskScore += 0.5;
        if (toNeighbors.size() < 3) riskScore += 0.5;
        
        // Distance-based risk (longer segments may have higher uncertainty)
        const Node& fromNode = graph->getNode(fromNodeId);
        const Node& toNode = graph->getNode(toNodeId);
        double distance = fromNode.euclideanDistance(toNode);
        
        if (distance > 10.0) riskScore += 0.3;
        
        return riskScore;
    }
    
    CostProfile generateCostProfile(const std::vector<int>& path) const {
        CostProfile profile;
        
        if (path.size() < 2) return profile;
        
        profile.segmentCosts.reserve(path.size() - 1);
        profile.detailedComponents.reserve(path.size() - 1);
        
        for (size_t i = 1; i < path.size(); ++i) {
            CostComponents components = calculateDetailedSegmentCost(path[i-1], path[i]);
            double segmentCost = components.getTotalCost();
            
            profile.segmentCosts.push_back(segmentCost);
            profile.detailedComponents.push_back(components);
            profile.totalCost += segmentCost;
            
            // Update statistics
            if (segmentCost > profile.maxSegmentCost) {
                profile.maxSegmentCost = segmentCost;
                profile.mostExpensiveSegment = static_cast<int>(i - 1);
            }
            
            if (segmentCost < profile.minSegmentCost) {
                profile.minSegmentCost = segmentCost;
            }
        }
        
        if (!profile.segmentCosts.empty()) {
            profile.averageCostPerUnit = profile.totalCost / profile.segmentCosts.size();
        }
        
        return profile;
    }
    
    bool validatePath(const std::vector<int>& path) const {
        if (path.empty()) return false;
        
        for (int nodeId : path) {
            if (!graph->hasNode(nodeId)) {
                std::cout << "[COST_CALC] Invalid node " << nodeId << " in path" << std::endl;
                return false;
            }
        }
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(path[i-1], path[i])) {
                std::cout << "[COST_CALC] No edge between nodes " << path[i-1] 
                          << " and " << path[i] << std::endl;
                return false;
            }
        }
        
        return true;
    }
    
public:
    PathCostCalculator(const Graph* environment) 
        : graph(environment), distanceWeightFactor(1.0), timeWeightFactor(1.0),
          energyWeightFactor(1.0), riskWeightFactor(0.5), enableDetailedAnalysis(true),
          cacheCalculations(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        // Initialize default terrain cost factors
        terrainCostFactors["urban"] = 1.0;
        terrainCostFactors["flat"] = 0.9;
        terrainCostFactors["mixed"] = 1.1;
        terrainCostFactors["mountain"] = 1.5;
        
        std::cout << "[COST_CALC] Path cost calculator initialized" << std::endl;
    }
    
    double calculatePathCost(const std::vector<int>& path) const {
        if (!validatePath(path)) {
            std::cout << "[COST_CALC] Invalid path provided for cost calculation" << std::endl;
            return std::numeric_limits<double>::infinity();
        }
        
        if (path.size() < 2) return 0.0;
        
        // Check cache first
        if (cacheCalculations) {
            std::string cacheKey = generateCacheKey(path);
            auto cachedResult = costCache.find(cacheKey);
            if (cachedResult != costCache.end()) {
                std::cout << "[COST_CALC] Retrieved cached cost for path" << std::endl;
                return cachedResult->second;
            }
        }
        
        std::cout << "[COST_CALC] Calculating cost for path with " << path.size() << " nodes" << std::endl;
        
        double totalCost = 0.0;
        
        if (enableDetailedAnalysis) {
            CostProfile profile = generateCostProfile(path);
            totalCost = profile.totalCost;
            
            std::cout << "[COST_CALC] Detailed analysis complete - Total cost: " << totalCost 
                      << ", Average per segment: " << profile.averageCostPerUnit << std::endl;
        } else {
            // Simple cost calculation
            for (size_t i = 1; i < path.size(); ++i) {
                totalCost += calculateEdgeCost(path[i-1], path[i]);
            }
        }
        
        // Cache the result
        if (cacheCalculations) {
            std::string cacheKey = generateCacheKey(path);
            costCache[cacheKey] = totalCost;
        }
        
        return totalCost;
    }
    
    CostBreakdown calculateDetailedCost(const std::vector<int>& path) const {
        CostBreakdown breakdown;
        
        if (!validatePath(path) || path.size() < 2) {
            return breakdown;
        }
        
        std::cout << "[COST_CALC] Generating detailed cost breakdown" << std::endl;
        
        CostProfile profile = generateCostProfile(path);
        breakdown.totalCost = profile.totalCost;
        
        // Aggregate component costs
        for (const auto& component : profile.detailedComponents) {
            breakdown.distanceCost += component.distanceCost;
            breakdown.timeCost += component.timeCost;
            breakdown.energyCost += component.energyCost;
            breakdown.riskCost += component.riskCost;
        }
        
        // Add custom cost entries
        breakdown.customCosts["terrain_penalty"] = 0.0;
        breakdown.customCosts["elevation_penalty"] = 0.0;
        breakdown.customCosts["congestion_cost"] = 0.0;
        
        for (const auto& component : profile.detailedComponents) {
            breakdown.customCosts["elevation_penalty"] += component.elevationCost;
            breakdown.customCosts["congestion_cost"] += component.congestionCost;
        }
        
        std::cout << "[COST_CALC] Cost breakdown: Distance=" << breakdown.distanceCost 
                  << ", Time=" << breakdown.timeCost << ", Energy=" << breakdown.energyCost 
                  << ", Risk=" << breakdown.riskCost << std::endl;
        
        return breakdown;
    }
    
    double calculateSegmentCost(int fromNodeId, int toNodeId) const {
        if (!graph->hasNode(fromNodeId) || !graph->hasNode(toNodeId)) {
            return std::numeric_limits<double>::infinity();
        }
        
        if (enableDetailedAnalysis) {
            CostComponents components = calculateDetailedSegmentCost(fromNodeId, toNodeId);
            return components.getTotalCost();
        } else {
            return calculateEdgeCost(fromNodeId, toNodeId);
        }
    }
    
    std::vector<double> calculateCostProfile(const std::vector<int>& path) const {
        if (!validatePath(path)) {
            return {};
        }
        
        CostProfile profile = generateCostProfile(path);
        return profile.segmentCosts;
    }
    
    int findMostExpensiveSegment(const std::vector<int>& path) const {
        if (!validatePath(path) || path.size() < 2) {
            return -1;
        }
        
        CostProfile profile = generateCostProfile(path);
        return profile.mostExpensiveSegment;
    }
    
    void setNodeCostModifier(int nodeId, double modifier) {
        nodeCostModifiers[nodeId] = modifier;
        std::cout << "[COST_CALC] Set cost modifier " << modifier << " for node " << nodeId << std::endl;
        
        // Clear relevant cache entries
        if (cacheCalculations) {
            clearCacheForNode(nodeId);
        }
    }
    
    void setEdgeCostModifier(int fromNodeId, int toNodeId, double modifier) {
        std::string edgeKey = std::to_string(fromNodeId) + "_" + std::to_string(toNodeId);
        edgeCostModifiers[edgeKey] = modifier;
        std::cout << "[COST_CALC] Set cost modifier " << modifier << " for edge " 
                  << fromNodeId << "->" << toNodeId << std::endl;
        
        // Clear relevant cache entries
        if (cacheCalculations) {
            clearCacheForEdge(fromNodeId, toNodeId);
        }
    }
    
    void setTerrainCostFactor(const std::string& terrainType, double factor) {
        terrainCostFactors[terrainType] = factor;
        std::cout << "[COST_CALC] Set terrain cost factor " << factor 
                  << " for terrain type " << terrainType << std::endl;
    }
    
    void setCostWeights(double distance, double time, double energy, double risk) {
        distanceWeightFactor = distance;
        timeWeightFactor = time;
        energyWeightFactor = energy;
        riskWeightFactor = risk;
        
        std::cout << "[COST_CALC] Updated cost weights - Distance: " << distance 
                  << ", Time: " << time << ", Energy: " << energy << ", Risk: " << risk << std::endl;
        
        // Clear cache as weights have changed
        if (cacheCalculations) {
            costCache.clear();
        }
    }
    
    void enableDetailedAnalysis(bool enable) {
        enableDetailedAnalysis = enable;
        std::cout << "[COST_CALC] Detailed analysis " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableCaching(bool enable) {
        cacheCalculations = enable;
        if (!enable) {
            costCache.clear();
        }
        std::cout << "[COST_CALC] Cost caching " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void clearCostCache() {
        costCache.clear();
        std::cout << "[COST_CALC] Cost cache cleared" << std::endl;
    }
    
    void printCostStatistics() const {
        std::cout << "[COST_CALC] Cost Calculator Statistics:" << std::endl;
        std::cout << "[COST_CALC]   Node modifiers: " << nodeCostModifiers.size() << std::endl;
        std::cout << "[COST_CALC]   Edge modifiers: " << edgeCostModifiers.size() << std::endl;
        std::cout << "[COST_CALC]   Terrain factors: " << terrainCostFactors.size() << std::endl;
        std::cout << "[COST_CALC]   Cached calculations: " << costCache.size() << std::endl;
        std::cout << "[COST_CALC]   Detailed analysis: " << (enableDetailedAnalysis ? "On" : "Off") << std::endl;
    }
    
private:
    void clearCacheForNode(int nodeId) {
        auto it = costCache.begin();
        while (it != costCache.end()) {
            if (it->first.find(std::to_string(nodeId)) != std::string::npos) {
                it = costCache.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    void clearCacheForEdge(int fromNodeId, int toNodeId) {
        std::string edgePattern = std::to_string(fromNodeId) + "_" + std::to_string(toNodeId);
        auto it = costCache.begin();
        while (it != costCache.end()) {
            if (it->first.find(edgePattern) != std::string::npos) {
                it = costCache.erase(it);
            } else {
                ++it;
            }
        }
    }
};