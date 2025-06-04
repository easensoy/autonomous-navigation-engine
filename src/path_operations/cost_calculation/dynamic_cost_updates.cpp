#include "path_operations/CostCalculator.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <queue>
#include <unordered_set>
#include <mutex>
#include <thread>

class DynamicCostUpdater {
private:
    const Graph* graph;
    
    struct CostUpdate {
        enum Type { NODE_COST_CHANGE, EDGE_COST_CHANGE, TERRAIN_CHANGE, CONGESTION_UPDATE, WEATHER_UPDATE };
        
        Type updateType;
        int nodeId;
        int edgeFromId;
        int edgeToId;
        double oldCost;
        double newCost;
        std::string reason;
        std::chrono::steady_clock::time_point timestamp;
        double severity;
        bool isPermanent;
        
        CostUpdate(Type type, const std::string& updateReason) 
            : updateType(type), nodeId(-1), edgeFromId(-1), edgeToId(-1),
              oldCost(0.0), newCost(0.0), reason(updateReason),
              timestamp(std::chrono::steady_clock::now()), severity(1.0), isPermanent(false) {}
    };
    
    struct CongestionData {
        double congestionLevel;
        std::chrono::steady_clock::time_point lastUpdate;
        std::chrono::duration<double> duration;
        bool isActive;
        double impactRadius;
        
        CongestionData() : congestionLevel(0.0), lastUpdate(std::chrono::steady_clock::now()),
                          duration(std::chrono::minutes(30)), isActive(false), impactRadius(5.0) {}
    };
    
    struct DynamicCostState {
        std::unordered_map<int, double> dynamicNodeCosts;
        std::unordered_map<std::string, double> dynamicEdgeCosts;
        std::unordered_map<int, CongestionData> congestionMap;
        std::unordered_map<std::string, double> weatherFactors;
        std::chrono::steady_clock::time_point lastGlobalUpdate;
        
        DynamicCostState() : lastGlobalUpdate(std::chrono::steady_clock::now()) {}
    };
    
    DynamicCostState currentState;
    std::vector<CostUpdate> updateHistory;
    std::queue<CostUpdate> pendingUpdates;
    
    // Thread safety
    mutable std::mutex stateMutex;
    mutable std::mutex historyMutex;
    
    // Configuration
    bool enableRealTimeUpdates;
    bool enableCongestionModeling;
    bool enableWeatherEffects;
    double updateFrequency;
    size_t maxHistorySize;
    std::chrono::duration<double> congestionDecayTime;
    
    // Background processing
    std::atomic<bool> backgroundProcessingActive;
    std::thread backgroundProcessor;
    
    void processBackgroundUpdates() {
        std::cout << "[DYNAMIC_COST] Background cost update processing started" << std::endl;
        
        while (backgroundProcessingActive.load()) {
            {
                std::lock_guard<std::mutex> lock(stateMutex);
                processPendingUpdates();
                updateCongestionDecay();
                updateWeatherEffects();
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000 / updateFrequency)));
        }
        
        std::cout << "[DYNAMIC_COST] Background cost update processing stopped" << std::endl;
    }
    
    void processPendingUpdates() {
        while (!pendingUpdates.empty()) {
            CostUpdate update = pendingUpdates.front();
            pendingUpdates.pop();
            
            applyUpdate(update);
            
            {
                std::lock_guard<std::mutex> historyLock(historyMutex);
                updateHistory.push_back(update);
                
                if (updateHistory.size() > maxHistorySize) {
                    updateHistory.erase(updateHistory.begin());
                }
            }
        }
    }
    
    void applyUpdate(const CostUpdate& update) {
        switch (update.updateType) {
            case CostUpdate::NODE_COST_CHANGE:
                applyNodeCostUpdate(update);
                break;
            case CostUpdate::EDGE_COST_CHANGE:
                applyEdgeCostUpdate(update);
                break;
            case CostUpdate::CONGESTION_UPDATE:
                applyCongestionUpdate(update);
                break;
            case CostUpdate::WEATHER_UPDATE:
                applyWeatherUpdate(update);
                break;
            default:
                std::cout << "[DYNAMIC_COST] Unknown update type" << std::endl;
                break;
        }
    }
    
    void applyNodeCostUpdate(const CostUpdate& update) {
        if (update.nodeId != -1) {
            currentState.dynamicNodeCosts[update.nodeId] = update.newCost;
            std::cout << "[DYNAMIC_COST] Updated node " << update.nodeId 
                      << " cost to " << update.newCost << " (" << update.reason << ")" << std::endl;
        }
    }
    
    void applyEdgeCostUpdate(const CostUpdate& update) {
        if (update.edgeFromId != -1 && update.edgeToId != -1) {
            std::string edgeKey = std::to_string(update.edgeFromId) + "_" + std::to_string(update.edgeToId);
            currentState.dynamicEdgeCosts[edgeKey] = update.newCost;
            std::cout << "[DYNAMIC_COST] Updated edge " << update.edgeFromId 
                      << "->" << update.edgeToId << " cost to " << update.newCost 
                      << " (" << update.reason << ")" << std::endl;
        }
    }
    
    void applyCongestionUpdate(const CostUpdate& update) {
        if (update.nodeId != -1 && enableCongestionModeling) {
            CongestionData& congestion = currentState.congestionMap[update.nodeId];
            congestion.congestionLevel = update.newCost;
            congestion.lastUpdate = update.timestamp;
            congestion.isActive = (update.newCost > 0.1);
            congestion.severity = update.severity;
            
            std::cout << "[DYNAMIC_COST] Updated congestion at node " << update.nodeId 
                      << " to level " << update.newCost << std::endl;
            
            updateCongestionImpact(update.nodeId);
        }
    }
    
    void applyWeatherUpdate(const CostUpdate& update) {
        if (enableWeatherEffects) {
            currentState.weatherFactors[update.reason] = update.newCost;
            std::cout << "[DYNAMIC_COST] Updated weather factor " << update.reason 
                      << " to " << update.newCost << std::endl;
            
            updateGlobalWeatherImpact();
        }
    }
    
    void updateCongestionImpact(int centerNodeId) {
        const CongestionData& congestion = currentState.congestionMap[centerNodeId];
        
        if (!congestion.isActive) return;
        
        // Apply congestion impact to nearby nodes and edges
        std::vector<int> nearbyNodes = findNodesWithinRadius(centerNodeId, congestion.impactRadius);
        
        for (int nodeId : nearbyNodes) {
            if (nodeId == centerNodeId) continue;
            
            double distance = calculateNodeDistance(centerNodeId, nodeId);
            double impactFactor = std::max(0.0, 1.0 - (distance / congestion.impactRadius));
            double additionalCost = congestion.congestionLevel * impactFactor * congestion.severity;
            
            // Update node cost
            currentState.dynamicNodeCosts[nodeId] = getDynamicNodeCost(nodeId) + additionalCost;
            
            // Update edges involving this node
            std::vector<int> neighbors = graph->getNeighbors(nodeId);
            for (int neighbor : neighbors) {
                std::string edgeKey = std::to_string(nodeId) + "_" + std::to_string(neighbor);
                currentState.dynamicEdgeCosts[edgeKey] = getDynamicEdgeCost(nodeId, neighbor) + additionalCost * 0.5;
            }
        }
    }
    
    void updateCongestionDecay() {
        auto currentTime = std::chrono::steady_clock::now();
        
        for (auto& [nodeId, congestion] : currentState.congestionMap) {
            if (!congestion.isActive) continue;
            
            auto timeSinceUpdate = currentTime - congestion.lastUpdate;
            if (timeSinceUpdate > congestionDecayTime) {
                double decayFactor = std::exp(-std::chrono::duration<double>(timeSinceUpdate).count() / 3600.0);
                congestion.congestionLevel *= decayFactor;
                
                if (congestion.congestionLevel < 0.05) {
                    congestion.isActive = false;
                    congestion.congestionLevel = 0.0;
                    std::cout << "[DYNAMIC_COST] Congestion at node " << nodeId << " has decayed" << std::endl;
                }
            }
        }
    }
    
    void updateWeatherEffects() {
        // Apply weather-based cost modifications across the entire graph
        for (const auto& [weatherType, factor] : currentState.weatherFactors) {
            if (weatherType == "rain") {
                applyGlobalCostMultiplier(1.0 + factor * 0.2);
            } else if (weatherType == "snow") {
                applyGlobalCostMultiplier(1.0 + factor * 0.5);
            } else if (weatherType == "fog") {
                applyGlobalCostMultiplier(1.0 + factor * 0.3);
            }
        }
    }
    
    void updateGlobalWeatherImpact() {
        double totalWeatherImpact = 0.0;
        for (const auto& [weatherType, factor] : currentState.weatherFactors) {
            totalWeatherImpact += factor;
        }
        
        if (totalWeatherImpact > 0.1) {
            std::cout << "[DYNAMIC_COST] Global weather impact: " << totalWeatherImpact << std::endl;
        }
    }
    
    void applyGlobalCostMultiplier(double multiplier) {
        // Apply multiplier to all dynamic costs
        for (auto& [nodeId, cost] : currentState.dynamicNodeCosts) {
            cost *= multiplier;
        }
        
        for (auto& [edgeKey, cost] : currentState.dynamicEdgeCosts) {
            cost *= multiplier;
        }
    }
    
    std::vector<int> findNodesWithinRadius(int centerNodeId, double radius) const {
        std::vector<int> nearbyNodes;
        const Node& centerNode = graph->getNode(centerNodeId);
        
        for (int nodeId : graph->getAllNodeIds()) {
            if (nodeId == centerNodeId) continue;
            
            const Node& node = graph->getNode(nodeId);
            double distance = centerNode.euclideanDistance(node);
            
            if (distance <= radius) {
                nearbyNodes.push_back(nodeId);
            }
        }
        
        return nearbyNodes;
    }
    
    double calculateNodeDistance(int nodeId1, int nodeId2) const {
        const Node& node1 = graph->getNode(nodeId1);
        const Node& node2 = graph->getNode(nodeId2);
        return node1.euclideanDistance(node2);
    }
    
public:
    DynamicCostUpdater(const Graph* environment) 
        : graph(environment), enableRealTimeUpdates(true), enableCongestionModeling(true),
          enableWeatherEffects(true), updateFrequency(1.0), maxHistorySize(1000),
          congestionDecayTime(std::chrono::hours(1)), backgroundProcessingActive(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[DYNAMIC_COST] Dynamic cost updater initialized" << std::endl;
    }
    
    ~DynamicCostUpdater() {
        stopBackgroundProcessing();
    }
    
    void startBackgroundProcessing() {
        if (!backgroundProcessingActive.load()) {
            backgroundProcessingActive.store(true);
            backgroundProcessor = std::thread(&DynamicCostUpdater::processBackgroundUpdates, this);
            std::cout << "[DYNAMIC_COST] Background processing started" << std::endl;
        }
    }
    
    void stopBackgroundProcessing() {
        if (backgroundProcessingActive.load()) {
            backgroundProcessingActive.store(false);
            if (backgroundProcessor.joinable()) {
                backgroundProcessor.join();
            }
            std::cout << "[DYNAMIC_COST] Background processing stopped" << std::endl;
        }
    }
    
    void updateNodeCost(int nodeId, double newCost, const std::string& reason, bool isPermanent = false) {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        CostUpdate update(CostUpdate::NODE_COST_CHANGE, reason);
        update.nodeId = nodeId;
        update.oldCost = getDynamicNodeCost(nodeId);
        update.newCost = newCost;
        update.isPermanent = isPermanent;
        
        if (enableRealTimeUpdates) {
            applyUpdate(update);
            
            std::lock_guard<std::mutex> historyLock(historyMutex);
            updateHistory.push_back(update);
        } else {
            pendingUpdates.push(update);
        }
        
        std::cout << "[DYNAMIC_COST] Queued node cost update for node " << nodeId << std::endl;
    }
    
    void updateEdgeCost(int fromNodeId, int toNodeId, double newCost, const std::string& reason, bool isPermanent = false) {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        CostUpdate update(CostUpdate::EDGE_COST_CHANGE, reason);
        update.edgeFromId = fromNodeId;
        update.edgeToId = toNodeId;
        update.oldCost = getDynamicEdgeCost(fromNodeId, toNodeId);
        update.newCost = newCost;
        update.isPermanent = isPermanent;
        
        if (enableRealTimeUpdates) {
            applyUpdate(update);
            
            std::lock_guard<std::mutex> historyLock(historyMutex);
            updateHistory.push_back(update);
        } else {
            pendingUpdates.push(update);
        }
        
        std::cout << "[DYNAMIC_COST] Queued edge cost update for edge " 
                  << fromNodeId << "->" << toNodeId << std::endl;
    }
    
    void updateCongestion(int nodeId, double congestionLevel, double severity = 1.0) {
        if (!enableCongestionModeling) return;
        
        std::lock_guard<std::mutex> lock(stateMutex);
        
        CostUpdate update(CostUpdate::CONGESTION_UPDATE, "traffic_congestion");
        update.nodeId = nodeId;
        update.newCost = congestionLevel;
        update.severity = severity;
        
        if (enableRealTimeUpdates) {
            applyUpdate(update);
        } else {
            pendingUpdates.push(update);
        }
        
        std::cout << "[DYNAMIC_COST] Updated congestion level " << congestionLevel 
                  << " at node " << nodeId << std::endl;
    }
    
    void updateWeatherCondition(const std::string& weatherType, double intensity) {
        if (!enableWeatherEffects) return;
        
        std::lock_guard<std::mutex> lock(stateMutex);
        
        CostUpdate update(CostUpdate::WEATHER_UPDATE, weatherType);
        update.newCost = intensity;
        
        if (enableRealTimeUpdates) {
            applyUpdate(update);
        } else {
            pendingUpdates.push(update);
        }
        
        std::cout << "[DYNAMIC_COST] Updated weather condition " << weatherType 
                  << " with intensity " << intensity << std::endl;
    }
    
    double getDynamicNodeCost(int nodeId) const {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        auto it = currentState.dynamicNodeCosts.find(nodeId);
        return (it != currentState.dynamicNodeCosts.end()) ? it->second : 0.0;
    }
    
    double getDynamicEdgeCost(int fromNodeId, int toNodeId) const {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        std::string edgeKey = std::to_string(fromNodeId) + "_" + std::to_string(toNodeId);
        auto it = currentState.dynamicEdgeCosts.find(edgeKey);
        return (it != currentState.dynamicEdgeCosts.end()) ? it->second : 0.0;
    }
    
    std::vector<CostUpdate> getRecentUpdates(std::chrono::duration<double> timeWindow) const {
        std::lock_guard<std::mutex> lock(historyMutex);
        
        std::vector<CostUpdate> recentUpdates;
        auto cutoffTime = std::chrono::steady_clock::now() - timeWindow;
        
        for (const auto& update : updateHistory) {
            if (update.timestamp >= cutoffTime) {
                recentUpdates.push_back(update);
            }
        }
        
        return recentUpdates;
    }
    
    void clearTemporaryUpdates() {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        auto it = updateHistory.begin();
        while (it != updateHistory.end()) {
            if (!it->isPermanent) {
                // Reverse the update
                CostUpdate reverseUpdate = *it;
                reverseUpdate.newCost = reverseUpdate.oldCost;
                applyUpdate(reverseUpdate);
                
                it = updateHistory.erase(it);
            } else {
                ++it;
            }
        }
        
        std::cout << "[DYNAMIC_COST] Cleared all temporary cost updates" << std::endl;
    }
    
    void setUpdateFrequency(double frequency) {
        updateFrequency = frequency;
        std::cout << "[DYNAMIC_COST] Update frequency set to " << frequency << " Hz" << std::endl;
    }
    
    void enableRealTimeMode(bool enable) {
        enableRealTimeUpdates = enable;
        std::cout << "[DYNAMIC_COST] Real-time updates " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableCongestionModel(bool enable) {
        enableCongestionModeling = enable;
        if (!enable) {
            currentState.congestionMap.clear();
        }
        std::cout << "[DYNAMIC_COST] Congestion modeling " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableWeatherModel(bool enable) {
        enableWeatherEffects = enable;
        if (!enable) {
            currentState.weatherFactors.clear();
        }
        std::cout << "[DYNAMIC_COST] Weather effects " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void printDynamicState() const {
        std::lock_guard<std::mutex> lock(stateMutex);
        
        std::cout << "[DYNAMIC_COST] Dynamic Cost State:" << std::endl;
        std::cout << "[DYNAMIC_COST]   Dynamic node costs: " << currentState.dynamicNodeCosts.size() << std::endl;
        std::cout << "[DYNAMIC_COST]   Dynamic edge costs: " << currentState.dynamicEdgeCosts.size() << std::endl;
        std::cout << "[DYNAMIC_COST]   Active congestion points: " << currentState.congestionMap.size() << std::endl;
        std::cout << "[DYNAMIC_COST]   Weather factors: " << currentState.weatherFactors.size() << std::endl;
        std::cout << "[DYNAMIC_COST]   Update history entries: " << updateHistory.size() << std::endl;
        std::cout << "[DYNAMIC_COST]   Pending updates: " << pendingUpdates.size() << std::endl;
    }
};