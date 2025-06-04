#include "navigation_strategies/DynamicReplanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <chrono>
#include <algorithm>

class IncrementalSearch {
private:
    struct SearchNode {
        int nodeId;
        double gScore;
        double fScore;
        int parent;
        bool isValid;
        std::chrono::steady_clock::time_point lastUpdate;
        
        SearchNode(int id = -1) : nodeId(id), gScore(std::numeric_limits<double>::infinity()),
                                 fScore(std::numeric_limits<double>::infinity()), parent(-1), 
                                 isValid(true), lastUpdate(std::chrono::steady_clock::now()) {}
    };
    
    struct ChangeRecord {
        enum Type { NODE_BLOCKED, NODE_UNBLOCKED, EDGE_COST_CHANGED, EDGE_ADDED, EDGE_REMOVED };
        Type changeType;
        int nodeId;
        std::pair<int, int> edgeId;
        double oldValue;
        double newValue;
        std::chrono::steady_clock::time_point timestamp;
        
        ChangeRecord(Type type, int node) : changeType(type), nodeId(node), 
                                           timestamp(std::chrono::steady_clock::now()) {}
        ChangeRecord(Type type, int from, int to, double oldVal, double newVal) 
            : changeType(type), edgeId({from, to}), oldValue(oldVal), newValue(newVal),
              timestamp(std::chrono::steady_clock::now()) {}
    };
    
    const Graph* graph;
    std::unordered_map<int, SearchNode> searchNodes;
    std::unordered_map<int, std::vector<int>> parentTree;
    std::unordered_set<int> invalidatedNodes;
    std::vector<ChangeRecord> changeHistory;
    
    int lastStartNode;
    int lastGoalNode;
    std::vector<int> lastPath;
    bool hasValidPath;
    
    std::unique_ptr<AStar> aStar;
    std::unique_ptr<Dijkstra> dijkstra;
    
    double cacheValidityThreshold;
    size_t maxChangeHistorySize;
    
    double calculateHeuristic(int from, int to) const {
        if (!graph->hasNode(from) || !graph->hasNode(to)) {
            return std::numeric_limits<double>::infinity();
        }
        
        const Node& fromNode = graph->getNode(from);
        const Node& toNode = graph->getNode(to);
        return fromNode.euclideanDistance(toNode);
    }
    
    void invalidateAffectedNodes(const ChangeRecord& change) {
        std::cout << "[INCREMENTAL] Invalidating nodes affected by change" << std::endl;
        
        switch (change.changeType) {
            case ChangeRecord::NODE_BLOCKED:
            case ChangeRecord::NODE_UNBLOCKED:
                invalidatedNodes.insert(change.nodeId);
                // Invalidate all nodes that had this as parent
                if (parentTree.find(change.nodeId) != parentTree.end()) {
                    for (int child : parentTree[change.nodeId]) {
                        invalidatedNodes.insert(child);
                    }
                }
                break;
                
            case ChangeRecord::EDGE_COST_CHANGED:
            case ChangeRecord::EDGE_ADDED:
            case ChangeRecord::EDGE_REMOVED:
                invalidatedNodes.insert(change.edgeId.first);
                invalidatedNodes.insert(change.edgeId.second);
                break;
        }
        
        // Propagate invalidation up the tree
        propagateInvalidation();
    }
    
    void propagateInvalidation() {
        std::queue<int> propagationQueue;
        std::unordered_set<int> processed;
        
        for (int nodeId : invalidatedNodes) {
            propagationQueue.push(nodeId);
        }
        
        while (!propagationQueue.empty()) {
            int current = propagationQueue.front();
            propagationQueue.pop();
            
            if (processed.find(current) != processed.end()) {
                continue;
            }
            processed.insert(current);
            
            searchNodes[current].isValid = false;
            
            // Find all nodes that have this as parent and invalidate them
            for (auto& pair : searchNodes) {
                if (pair.second.parent == current && pair.second.isValid) {
                    invalidatedNodes.insert(pair.first);
                    propagationQueue.push(pair.first);
                }
            }
        }
        
        std::cout << "[INCREMENTAL] Invalidated " << processed.size() << " nodes" << std::endl;
    }
    
    bool isSearchResultValid() const {
        if (!hasValidPath || lastPath.empty()) {
            return false;
        }
        
        // Check if any nodes in the path are invalidated
        for (int nodeId : lastPath) {
            if (invalidatedNodes.find(nodeId) != invalidatedNodes.end() ||
                !searchNodes.at(nodeId).isValid) {
                return false;
            }
        }
        
        return true;
    }
    
    std::vector<int> performIncrementalRepair(int startId, int goalId) {
        std::cout << "[INCREMENTAL] Performing incremental repair from " 
                  << startId << " to " << goalId << std::endl;
        
        if (invalidatedNodes.empty() && startId == lastStartNode && goalId == lastGoalNode) {
            std::cout << "[INCREMENTAL] No changes detected, returning cached path" << std::endl;
            return lastPath;
        }
        
        // For now, use A* to recompute affected portions
        // In a full implementation, this would repair only the invalidated parts
        std::vector<int> repairedPath;
        
        if (invalidatedNodes.size() < graph->getNodeCount() * 0.3) {
            // Minor changes - attempt incremental repair
            repairedPath = repairPath(startId, goalId);
        } else {
            // Major changes - perform full recomputation
            std::cout << "[INCREMENTAL] Too many changes, performing full recomputation" << std::endl;
            repairedPath = performFullSearch(startId, goalId);
        }
        
        // Update cache
        lastStartNode = startId;
        lastGoalNode = goalId;
        lastPath = repairedPath;
        hasValidPath = !repairedPath.empty();
        
        // Clear invalidated nodes
        invalidatedNodes.clear();
        
        return repairedPath;
    }
    
    std::vector<int> repairPath(int startId, int goalId) {
        std::cout << "[INCREMENTAL] Attempting path repair" << std::endl;
        
        // Find the first invalidated node in the current path
        int firstInvalidIndex = -1;
        for (size_t i = 0; i < lastPath.size(); ++i) {
            if (invalidatedNodes.find(lastPath[i]) != invalidatedNodes.end()) {
                firstInvalidIndex = static_cast<int>(i);
                break;
            }
        }
        
        if (firstInvalidIndex == -1) {
            // No invalidated nodes in path, check if path is still valid
            if (validatePath(lastPath)) {
                return lastPath;
            }
        }
        
        // Find a valid segment before the invalidated region
        int repairStartIndex = std::max(0, firstInvalidIndex - 1);
        int repairStart = (repairStartIndex < static_cast<int>(lastPath.size())) ? 
                         lastPath[repairStartIndex] : startId;
        
        // Search for a new path segment from repair point to goal
        std::vector<int> repairSegment = aStar->findPath(repairStart, goalId);
        
        if (repairSegment.empty()) {
            std::cout << "[INCREMENTAL] Repair failed, performing full search" << std::endl;
            return performFullSearch(startId, goalId);
        }
        
        // Combine valid prefix with repaired segment
        std::vector<int> repairedPath(lastPath.begin(), lastPath.begin() + repairStartIndex);
        repairedPath.insert(repairedPath.end(), repairSegment.begin(), repairSegment.end());
        
        std::cout << "[INCREMENTAL] Path repaired: " << lastPath.size() 
                  << " -> " << repairedPath.size() << " nodes" << std::endl;
        
        return repairedPath;
    }
    
    std::vector<int> performFullSearch(int startId, int goalId) {
        std::cout << "[INCREMENTAL] Performing full search from " 
                  << startId << " to " << goalId << std::endl;
        
        auto startTime = std::chrono::steady_clock::now();
        std::vector<int> path = aStar->findPath(startId, goalId);
        auto endTime = std::chrono::steady_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "[INCREMENTAL] Full search completed in " << duration.count() << "ms" << std::endl;
        
        // Update search nodes cache
        updateSearchCache(path);
        
        return path;
    }
    
    void updateSearchCache(const std::vector<int>& path) {
        if (path.empty()) return;
        
        // Update parent tree
        parentTree.clear();
        for (size_t i = 1; i < path.size(); ++i) {
            int parent = path[i-1];
            int child = path[i];
            parentTree[parent].push_back(child);
            
            searchNodes[child].parent = parent;
            searchNodes[child].isValid = true;
            searchNodes[child].lastUpdate = std::chrono::steady_clock::now();
        }
        
        searchNodes[path[0]].parent = -1;
        searchNodes[path[0]].isValid = true;
    }
    
    bool validatePath(const std::vector<int>& path) const {
        if (path.empty()) return false;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(path[i-1], path[i])) {
                return false;
            }
        }
        
        return true;
    }
    
public:
    IncrementalSearch(const Graph* environment) 
        : graph(environment), lastStartNode(-1), lastGoalNode(-1), hasValidPath(false),
          cacheValidityThreshold(300.0), maxChangeHistorySize(1000) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStar = std::make_unique<AStar>(graph);
        dijkstra = std::make_unique<Dijkstra>(graph);
        
        std::cout << "[INCREMENTAL] Incremental search algorithm initialized" << std::endl;
    }
    
    std::vector<int> findPath(int startId, int goalId) {
        std::cout << "[INCREMENTAL] Finding path from " << startId << " to " << goalId << std::endl;
        
        if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
            throw std::invalid_argument("Start or goal node does not exist");
        }
        
        // Check if we can use incremental repair
        if (hasValidPath && isSearchResultValid() && 
            startId == lastStartNode && goalId == lastGoalNode) {
            std::cout << "[INCREMENTAL] Using cached result" << std::endl;
            return lastPath;
        }
        
        return performIncrementalRepair(startId, goalId);
    }
    
    void recordNodeBlockage(int nodeId, bool blocked) {
        std::cout << "[INCREMENTAL] Recording node " << nodeId 
                  << (blocked ? " blocked" : " unblocked") << std::endl;
        
        ChangeRecord change(blocked ? ChangeRecord::NODE_BLOCKED : ChangeRecord::NODE_UNBLOCKED, nodeId);
        changeHistory.push_back(change);
        
        if (changeHistory.size() > maxChangeHistorySize) {
            changeHistory.erase(changeHistory.begin());
        }
        
        invalidateAffectedNodes(change);
    }
    
    void recordEdgeCostChange(int fromId, int toId, double oldCost, double newCost) {
        std::cout << "[INCREMENTAL] Recording edge cost change (" << fromId << "," << toId 
                  << ") " << oldCost << " -> " << newCost << std::endl;
        
        ChangeRecord change(ChangeRecord::EDGE_COST_CHANGED, fromId, toId, oldCost, newCost);
        changeHistory.push_back(change);
        
        if (changeHistory.size() > maxChangeHistorySize) {
            changeHistory.erase(changeHistory.begin());
        }
        
        invalidateAffectedNodes(change);
    }
    
    void recordEdgeAddition(int fromId, int toId, double cost) {
        std::cout << "[INCREMENTAL] Recording edge addition (" << fromId << "," << toId 
                  << ") with cost " << cost << std::endl;
        
        ChangeRecord change(ChangeRecord::EDGE_ADDED, fromId, toId, 0.0, cost);
        changeHistory.push_back(change);
        
        if (changeHistory.size() > maxChangeHistorySize) {
            changeHistory.erase(changeHistory.begin());
        }
        
        invalidateAffectedNodes(change);
    }
    
    void recordEdgeRemoval(int fromId, int toId, double oldCost) {
        std::cout << "[INCREMENTAL] Recording edge removal (" << fromId << "," << toId 
                  << ") with old cost " << oldCost << std::endl;
        
        ChangeRecord change(ChangeRecord::EDGE_REMOVED, fromId, toId, oldCost, 0.0);
        changeHistory.push_back(change);
        
        if (changeHistory.size() > maxChangeHistorySize) {
            changeHistory.erase(changeHistory.begin());
        }
        
        invalidateAffectedNodes(change);
    }
    
    void clearCache() {
        std::cout << "[INCREMENTAL] Clearing search cache" << std::endl;
        
        searchNodes.clear();
        parentTree.clear();
        invalidatedNodes.clear();
        changeHistory.clear();
        lastPath.clear();
        hasValidPath = false;
        lastStartNode = -1;
        lastGoalNode = -1;
    }
    
    void setCacheValidityThreshold(double seconds) {
        cacheValidityThreshold = seconds;
        std::cout << "[INCREMENTAL] Cache validity threshold set to " << seconds << " seconds" << std::endl;
    }
    
    void printStatistics() const {
        std::cout << "[INCREMENTAL] Algorithm Statistics:" << std::endl;
        std::cout << "[INCREMENTAL]   Cached nodes: " << searchNodes.size() << std::endl;
        std::cout << "[INCREMENTAL]   Invalidated nodes: " << invalidatedNodes.size() << std::endl;
        std::cout << "[INCREMENTAL]   Change history size: " << changeHistory.size() << std::endl;
        std::cout << "[INCREMENTAL]   Has valid path: " << (hasValidPath ? "Yes" : "No") << std::endl;
        
        if (hasValidPath) {
            std::cout << "[INCREMENTAL]   Last path length: " << lastPath.size() << std::endl;
            std::cout << "[INCREMENTAL]   Last start: " << lastStartNode << std::endl;
            std::cout << "[INCREMENTAL]   Last goal: " << lastGoalNode << std::endl;
        }
        
        // Count change types
        std::unordered_map<ChangeRecord::Type, int> changeCounts;
        for (const auto& change : changeHistory) {
            changeCounts[change.changeType]++;
        }
        
        std::cout << "[INCREMENTAL]   Recent changes:" << std::endl;
        std::cout << "[INCREMENTAL]     Node blockages: " << changeCounts[ChangeRecord::NODE_BLOCKED] << std::endl;
        std::cout << "[INCREMENTAL]     Node unblockages: " << changeCounts[ChangeRecord::NODE_UNBLOCKED] << std::endl;
        std::cout << "[INCREMENTAL]     Edge cost changes: " << changeCounts[ChangeRecord::EDGE_COST_CHANGED] << std::endl;
        std::cout << "[INCREMENTAL]     Edge additions: " << changeCounts[ChangeRecord::EDGE_ADDED] << std::endl;
        std::cout << "[INCREMENTAL]     Edge removals: " << changeCounts[ChangeRecord::EDGE_REMOVED] << std::endl;
    }
    
    double getCacheHitRatio() const {
        return hasValidPath && isSearchResultValid() ? 1.0 : 0.0;
    }
    
    size_t getInvalidatedNodeCount() const {
        return invalidatedNodes.size();
    }
};