#include "graph_operations/GraphAnalysis.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>
#include <queue>
#include <numeric>

class NodeRemovalAnalyzer {
private:
    Graph* graph;
    mutable std::mutex operationMutex;
    
    struct NodeRemovalResult {
        bool successful;
        int removedNodeId;
        std::chrono::steady_clock::time_point operationTimestamp;
        double operationTime;
        std::string removalStatus;
        size_t nodeCountBefore;
        size_t nodeCountAfter;
        size_t edgeCountBefore;
        size_t edgeCountAfter;
        double graphDensityBefore;
        double graphDensityAfter;
        std::vector<int> connectedNodes;
        std::vector<std::pair<int, double>> removedEdges;
        bool preservedConnectivity;
        bool createdIsolatedNodes;
        size_t componentCountBefore;
        size_t componentCountAfter;
        double topologicalImpact;
        std::vector<std::string> analysisNotes;
    };
    
    struct RemovalAnalysisMetrics {
        size_t totalRemovals;
        size_t successfulRemovals;
        size_t failedRemovals;
        size_t connectivityPreservingRemovals;
        size_t connectivityBreakingRemovals;
        size_t isolatingRemovals;
        double averageOperationTime;
        double averageTopologicalImpact;
        size_t totalEdgesRemoved;
        size_t totalNodesRemoved;
        std::chrono::steady_clock::time_point lastOperation;
        double totalGraphSimplification;
        size_t criticalNodeRemovals;
        size_t redundantNodeRemovals;
        size_t highDegreeNodeRemovals;
        double averageRemovedNodeDegree;
    };
    
    struct ConnectivityAnalysis {
        bool wasConnectedBefore;
        bool isConnectedAfter;
        size_t componentCountBefore;
        size_t componentCountAfter;
        std::vector<int> affectedComponents;
        std::vector<int> newIsolatedNodes;
        double pathEfficiencyChange;
        bool createdBridges;
        std::vector<std::pair<int, int>> criticalPaths;
    };
    
    RemovalAnalysisMetrics metrics;
    bool preserveConnectivity;
    bool enableTopologicalAnalysis;
    bool preventIsolation;
    double criticalityThreshold;
    std::string removalStrategy;
    std::function<void(const NodeRemovalResult&)> removalCallback;
    std::function<bool(int)> removalValidator;
    
public:
    explicit NodeRemovalAnalyzer(Graph* environment) 
        : graph(environment), preserveConnectivity(false), enableTopologicalAnalysis(true),
          preventIsolation(false), criticalityThreshold(0.1), removalStrategy("standard") {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeAnalysisMetrics();
        
        std::cout << "[NODE_REMOVAL] Node removal analysis system initialized" << std::endl;
    }
    
    NodeRemovalResult removeNode(int nodeId) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        auto operationStart = std::chrono::steady_clock::now();
        std::cout << "[NODE_REMOVAL] Analyzing node removal: " << nodeId << std::endl;
        
        NodeRemovalResult result;
        result.removedNodeId = nodeId;
        result.successful = false;
        result.operationTimestamp = operationStart;
        result.preservedConnectivity = false;
        result.createdIsolatedNodes = false;
        result.topologicalImpact = 0.0;
        
        // Capture initial graph state
        result.nodeCountBefore = graph->getNodeCount();
        result.edgeCountBefore = graph->getEdgeCount();
        result.graphDensityBefore = calculateGraphDensity();
        
        // Validate removal feasibility
        std::string validationResult = validateNodeRemoval(nodeId);
        result.removalStatus = validationResult;
        
        if (validationResult != "valid") {
            std::cout << "[NODE_REMOVAL] Node removal validation failed: " 
                      << validationResult << std::endl;
            updateFailureMetrics(result, validationResult);
            return result;
        }
        
        // Analyze removal impact before execution
        ConnectivityAnalysis connectivityBefore = analyzeConnectivityImpact(nodeId);
        result.componentCountBefore = connectivityBefore.componentCountBefore;
        
        if (preserveConnectivity && !connectivityBefore.isConnectedAfter) {
            std::cout << "[NODE_REMOVAL] Removal would break connectivity - aborting" << std::endl;
            result.removalStatus = "connectivity_violation";
            updateFailureMetrics(result, result.removalStatus);
            return result;
        }
        
        // Collect information about connected nodes and edges before removal
        result.connectedNodes = graph->getNeighbors(nodeId);
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            result.removedEdges.emplace_back(edge.getToNode(), edge.getWeight());
        }
        
        // Check for isolation prevention
        if (preventIsolation && wouldCreateIsolatedNodes(nodeId)) {
            std::cout << "[NODE_REMOVAL] Removal would create isolated nodes - aborting" << std::endl;
            result.removalStatus = "isolation_violation";
            updateFailureMetrics(result, result.removalStatus);
            return result;
        }
        
        // Execute the removal
        try {
            bool removalSuccessful = graph->removeNode(nodeId);
            
            if (!removalSuccessful) {
                std::cout << "[NODE_REMOVAL] Graph removal operation failed" << std::endl;
                result.removalStatus = "graph_removal_failed";
                updateFailureMetrics(result, result.removalStatus);
                return result;
            }
            
            result.successful = true;
            std::cout << "[NODE_REMOVAL] Node successfully removed from graph structure" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[NODE_REMOVAL] Node removal failed with exception: " 
                      << e.what() << std::endl;
            result.removalStatus = std::string("exception: ") + e.what();
            updateFailureMetrics(result, result.removalStatus);
            return result;
        }
        
        // Analyze post-removal state
        result.nodeCountAfter = graph->getNodeCount();
        result.edgeCountAfter = graph->getEdgeCount();
        result.graphDensityAfter = calculateGraphDensity();
        
        ConnectivityAnalysis connectivityAfter = analyzePostRemovalConnectivity();
        result.componentCountAfter = connectivityAfter.componentCountAfter;
        result.preservedConnectivity = connectivityBefore.isConnectedAfter == connectivityAfter.isConnectedAfter;
        result.createdIsolatedNodes = !connectivityAfter.newIsolatedNodes.empty();
        
        if (enableTopologicalAnalysis) {
            analyzeTopologicalChanges(result, connectivityBefore, connectivityAfter);
        }
        
        // Complete timing analysis
        auto operationEnd = std::chrono::steady_clock::now();
        result.operationTime = std::chrono::duration<double>(operationEnd - operationStart).count();
        
        // Update metrics
        updateSuccessMetrics(result);
        
        // Trigger callback if configured
        if (removalCallback) {
            removalCallback(result);
        }
        
        std::cout << "[NODE_REMOVAL] Node removal analysis completed successfully (time: " 
                  << result.operationTime << "s, edges removed: " 
                  << result.removedEdges.size() << ")" << std::endl;
        
        return result;
    }
    
    NodeRemovalResult removeNodeWithStrategy(int nodeId, const std::string& strategy) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[NODE_REMOVAL] Performing strategic node removal with strategy: " 
                  << strategy << std::endl;
        
        // Temporarily apply strategy
        std::string originalStrategy = removalStrategy;
        removalStrategy = strategy;
        
        NodeRemovalResult result = removeNode(nodeId);
        
        if (result.successful) {
            result.analysisNotes.push_back("Removal strategy applied: " + strategy);
        }
        
        // Restore original strategy
        removalStrategy = originalStrategy;
        
        return result;
    }
    
    std::vector<NodeRemovalResult> removeMultipleNodes(const std::vector<int>& nodeIds) {
        std::cout << "[NODE_REMOVAL] Processing batch removal of " << nodeIds.size() 
                  << " nodes" << std::endl;
        
        std::vector<NodeRemovalResult> results;
        results.reserve(nodeIds.size());
        
        // Sort nodes by removal priority (e.g., by degree)
        std::vector<int> sortedNodes = nodeIds;
        std::sort(sortedNodes.begin(), sortedNodes.end(), 
                 [this](int a, int b) {
                     return getNodeRemovalPriority(a) > getNodeRemovalPriority(b);
                 });
        
        for (int nodeId : sortedNodes) {
            if (!graph->hasNode(nodeId)) {
                // Node may have been removed as a side effect
                continue;
            }
            
            NodeRemovalResult result = removeNode(nodeId);
            results.push_back(result);
            
            // Log progress for large batches
            if (results.size() % 50 == 0) {
                std::cout << "[NODE_REMOVAL] Batch progress: " << results.size() 
                          << "/" << sortedNodes.size() << " removals processed" << std::endl;
            }
            
            // Stop if connectivity is broken and preservation is required
            if (preserveConnectivity && !result.preservedConnectivity && result.successful) {
                std::cout << "[NODE_REMOVAL] Halting batch processing due to connectivity concern" 
                          << std::endl;
                break;
            }
        }
        
        std::cout << "[NODE_REMOVAL] Batch removal completed with " 
                  << std::count_if(results.begin(), results.end(), 
                                  [](const NodeRemovalResult& r) { return r.successful; })
                  << " successful removals" << std::endl;
        
        return results;
    }
    
    bool canRemoveNode(int nodeId) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::string validationResult = validateNodeRemoval(nodeId);
        bool canRemove = (validationResult == "valid");
        
        if (canRemove && preserveConnectivity) {
            ConnectivityAnalysis analysis = analyzeConnectivityImpact(nodeId);
            canRemove = analysis.isConnectedAfter || !analysis.wasConnectedBefore;
        }
        
        if (canRemove && preventIsolation) {
            canRemove = !wouldCreateIsolatedNodes(nodeId);
        }
        
        std::cout << "[NODE_REMOVAL] Node removal feasibility check: " << nodeId 
                  << " - " << (canRemove ? "feasible" : "not feasible") << std::endl;
        
        return canRemove;
    }
    
    std::vector<int> identifyRemovalCandidates() {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[NODE_REMOVAL] Identifying optimal node removal candidates" << std::endl;
        
        std::vector<int> candidates;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            if (canRemoveNode(nodeId)) {
                double benefit = evaluateRemovalBenefit(nodeId);
                
                if (benefit > 0.3) { // Threshold for beneficial removal
                    candidates.push_back(nodeId);
                }
            }
        }
        
        // Sort candidates by benefit
        std::sort(candidates.begin(), candidates.end(), 
                 [this](int a, int b) {
                     return evaluateRemovalBenefit(a) > evaluateRemovalBenefit(b);
                 });
        
        std::cout << "[NODE_REMOVAL] Identified " << candidates.size() 
                  << " node removal candidates" << std::endl;
        
        return candidates;
    }
    
    std::vector<int> identifyRedundantNodes() {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[NODE_REMOVAL] Identifying redundant nodes for removal" << std::endl;
        
        std::vector<int> redundantNodes;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            if (isNodeRedundant(nodeId)) {
                redundantNodes.push_back(nodeId);
            }
        }
        
        std::cout << "[NODE_REMOVAL] Found " << redundantNodes.size() 
                  << " redundant nodes" << std::endl;
        
        return redundantNodes;
    }
    
    double calculateSimplificationImpact(int nodeId) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        if (!canRemoveNode(nodeId)) {
            return 0.0;
        }
        
        int nodeDegree = graph->getNeighbors(nodeId).size();
        size_t totalNodes = graph->getNodeCount();
        
        // Higher impact for nodes with more connections
        return static_cast<double>(nodeDegree) / (totalNodes - 1);
    }
    
    void configureRemoval(bool connectivityPreservation, bool topologicalAnalysis, 
                         bool isolationPrevention, const std::string& strategy, 
                         double criticalThreshold) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        preserveConnectivity = connectivityPreservation;
        enableTopologicalAnalysis = topologicalAnalysis;
        preventIsolation = isolationPrevention;
        removalStrategy = strategy;
        criticalityThreshold = criticalThreshold;
        
        std::cout << "[NODE_REMOVAL] Configuration updated:" << std::endl;
        std::cout << "[NODE_REMOVAL]   Preserve connectivity: " << (preserveConnectivity ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Topological analysis: " << (enableTopologicalAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Prevent isolation: " << (preventIsolation ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Removal strategy: " << removalStrategy << std::endl;
        std::cout << "[NODE_REMOVAL]   Criticality threshold: " << criticalityThreshold << std::endl;
    }
    
    void setRemovalCallback(std::function<void(const NodeRemovalResult&)> callback) {
        removalCallback = callback;
        std::cout << "[NODE_REMOVAL] Removal callback configured" << std::endl;
    }
    
    void setRemovalValidator(std::function<bool(int)> validator) {
        removalValidator = validator;
        std::cout << "[NODE_REMOVAL] Custom removal validator configured" << std::endl;
    }
    
    RemovalAnalysisMetrics getAnalysisMetrics() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        return metrics;
    }
    
    void generateRemovalReport() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "\n[NODE_REMOVAL] === NODE REMOVAL ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[NODE_REMOVAL] System Configuration:" << std::endl;
        std::cout << "[NODE_REMOVAL]   Preserve connectivity: " << (preserveConnectivity ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Topological analysis: " << (enableTopologicalAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Prevent isolation: " << (preventIsolation ? "Yes" : "No") << std::endl;
        std::cout << "[NODE_REMOVAL]   Removal strategy: " << removalStrategy << std::endl;
        std::cout << "[NODE_REMOVAL]   Criticality threshold: " << criticalityThreshold << std::endl;
        
        std::cout << "[NODE_REMOVAL] Current Graph State:" << std::endl;
        std::cout << "[NODE_REMOVAL]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[NODE_REMOVAL]   Total edges: " << graph->getEdgeCount() << std::endl;
        std::cout << "[NODE_REMOVAL]   Graph density: " << calculateGraphDensity() << std::endl;
        
        std::cout << "[NODE_REMOVAL] Operation Metrics:" << std::endl;
        std::cout << "[NODE_REMOVAL]   Total removal attempts: " << metrics.totalRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Successful removals: " << metrics.successfulRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Failed removals: " << metrics.failedRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Connectivity-preserving removals: " << metrics.connectivityPreservingRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Connectivity-breaking removals: " << metrics.connectivityBreakingRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Isolating removals: " << metrics.isolatingRemovals << std::endl;
        
        if (metrics.totalRemovals > 0) {
            double successRate = static_cast<double>(metrics.successfulRemovals) / metrics.totalRemovals * 100.0;
            std::cout << "[NODE_REMOVAL]   Success rate: " << successRate << "%" << std::endl;
        }
        
        std::cout << "[NODE_REMOVAL] Impact Metrics:" << std::endl;
        std::cout << "[NODE_REMOVAL]   Total nodes removed: " << metrics.totalNodesRemoved << std::endl;
        std::cout << "[NODE_REMOVAL]   Total edges removed: " << metrics.totalEdgesRemoved << std::endl;
        std::cout << "[NODE_REMOVAL]   Average operation time: " << metrics.averageOperationTime << " seconds" << std::endl;
        std::cout << "[NODE_REMOVAL]   Average topological impact: " << metrics.averageTopologicalImpact << std::endl;
        std::cout << "[NODE_REMOVAL]   Average removed node degree: " << metrics.averageRemovedNodeDegree << std::endl;
        std::cout << "[NODE_REMOVAL]   Total graph simplification: " << metrics.totalGraphSimplification << std::endl;
        std::cout << "[NODE_REMOVAL]   Critical node removals: " << metrics.criticalNodeRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   Redundant node removals: " << metrics.redundantNodeRemovals << std::endl;
        std::cout << "[NODE_REMOVAL]   High-degree node removals: " << metrics.highDegreeNodeRemovals << std::endl;
        
        std::cout << "[NODE_REMOVAL] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeAnalysisMetrics() {
        metrics.totalRemovals = 0;
        metrics.successfulRemovals = 0;
        metrics.failedRemovals = 0;
        metrics.connectivityPreservingRemovals = 0;
        metrics.connectivityBreakingRemovals = 0;
        metrics.isolatingRemovals = 0;
        metrics.averageOperationTime = 0.0;
        metrics.averageTopologicalImpact = 0.0;
        metrics.totalEdgesRemoved = 0;
        metrics.totalNodesRemoved = 0;
        metrics.lastOperation = std::chrono::steady_clock::now();
        metrics.totalGraphSimplification = 0.0;
        metrics.criticalNodeRemovals = 0;
        metrics.redundantNodeRemovals = 0;
        metrics.highDegreeNodeRemovals = 0;
        metrics.averageRemovedNodeDegree = 0.0;
    }
    
    std::string validateNodeRemoval(int nodeId) {
        // Check if node exists
        if (!graph->hasNode(nodeId)) {
            return "node_not_found";
        }
        
        // Apply custom validation if configured
        if (removalValidator && !removalValidator(nodeId)) {
            return "custom_validation_failed";
        }
        
        return "valid";
    }
    
    double calculateGraphDensity() const {
        size_t nodeCount = graph->getNodeCount();
        if (nodeCount <= 1) {
            return 0.0;
        }
        
        size_t edgeCount = graph->getEdgeCount();
        size_t maxPossibleEdges = nodeCount * (nodeCount - 1) / 2;
        
        return static_cast<double>(edgeCount) / maxPossibleEdges;
    }
    
    ConnectivityAnalysis analyzeConnectivityImpact(int nodeId) {
        ConnectivityAnalysis analysis;
        analysis.wasConnectedBefore = graph->isConnected();
        analysis.componentCountBefore = countConnectedComponents();
        
        // Simulate removal to check impact
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        // For simplicity, assume removal might disconnect the graph
        // In a full implementation, this would temporarily remove the node and check
        analysis.isConnectedAfter = analysis.wasConnectedBefore;
        analysis.componentCountAfter = analysis.componentCountBefore;
        
        // Check if removing this node would disconnect any components
        if (isNodeCriticalForConnectivity(nodeId)) {
            analysis.isConnectedAfter = false;
            analysis.componentCountAfter++;
        }
        
        return analysis;
    }
    
    ConnectivityAnalysis analyzePostRemovalConnectivity() {
        ConnectivityAnalysis analysis;
        analysis.isConnectedAfter = graph->isConnected();
        analysis.componentCountAfter = countConnectedComponents();
        
        // Identify any new isolated nodes
        std::vector<int> nodeIds = graph->getAllNodeIds();
        for (int nodeId : nodeIds) {
            if (graph->getNeighbors(nodeId).empty()) {
                analysis.newIsolatedNodes.push_back(nodeId);
            }
        }
        
        return analysis;
    }
    
    bool wouldCreateIsolatedNodes(int nodeId) {
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        // Check if any neighbor would become isolated
        for (int neighbor : neighbors) {
            std::vector<int> neighborConnections = graph->getNeighbors(neighbor);
            if (neighborConnections.size() == 1 && neighborConnections[0] == nodeId) {
                return true;
            }
        }
        
        return false;
    }
    
    bool isNodeCriticalForConnectivity(int nodeId) {
        // Simplified criticality check
        // In full implementation, would use bridge detection or articulation point algorithms
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        return neighbors.size() > 2; // Simple heuristic
    }
    
    bool isNodeRedundant(int nodeId) {
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        if (neighbors.size() <= 1) {
            return true; // Leaf nodes are often redundant
        }
        
        // Check if all neighbors are already connected to each other
        for (size_t i = 0; i < neighbors.size(); ++i) {
            for (size_t j = i + 1; j < neighbors.size(); ++j) {
                if (!graph->hasEdge(neighbors[i], neighbors[j])) {
                    return false;
                }
            }
        }
        
        return true; // Node is in a clique and might be redundant
    }
    
    double evaluateRemovalBenefit(int nodeId) {
        double simplificationBenefit = 1.0 / graph->getNodeCount();
        double connectivityCost = isNodeCriticalForConnectivity(nodeId) ? 5.0 : 0.0;
        double redundancyBonus = isNodeRedundant(nodeId) ? 2.0 : 0.0;
        
        return simplificationBenefit + redundancyBonus - connectivityCost;
    }
    
    int getNodeRemovalPriority(int nodeId) {
        // Lower degree nodes have higher removal priority
        return -(static_cast<int>(graph->getNeighbors(nodeId).size()));
    }
    
    size_t countConnectedComponents() {
        std::unordered_set<int> visited;
        size_t componentCount = 0;
        
        for (int nodeId : graph->getAllNodeIds()) {
            if (visited.find(nodeId) == visited.end()) {
                exploreComponent(nodeId, visited);
                componentCount++;
            }
        }
        
        return componentCount;
    }
    
    void exploreComponent(int nodeId, std::unordered_set<int>& visited) {
        visited.insert(nodeId);
        
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        for (int neighbor : neighbors) {
            if (visited.find(neighbor) == visited.end()) {
                exploreComponent(neighbor, visited);
            }
        }
    }
    
    void analyzeTopologicalChanges(NodeRemovalResult& result, 
                                 const ConnectivityAnalysis& before, 
                                 const ConnectivityAnalysis& after) {
        result.topologicalImpact = static_cast<double>(result.removedEdges.size()) / result.edgeCountBefore;
        
        if (result.topologicalImpact > criticalityThreshold) {
            result.analysisNotes.push_back("High topological impact detected");
        }
        
        if (result.preservedConnectivity) {
            result.analysisNotes.push_back("Connectivity successfully preserved");
        } else {
            result.analysisNotes.push_back("Connectivity was modified");
        }
        
        if (result.createdIsolatedNodes) {
            result.analysisNotes.push_back("Created isolated nodes");
        }
        
        size_t nodeDegree = result.removedEdges.size();
        if (nodeDegree > graph->getNodeCount() * 0.1) {
            result.analysisNotes.push_back("Removed high-degree node");
        }
    }
    
    void updateSuccessMetrics(const NodeRemovalResult& result) {
        metrics.totalRemovals++;
        metrics.successfulRemovals++;
        metrics.lastOperation = result.operationTimestamp;
        
        if (result.preservedConnectivity) {
            metrics.connectivityPreservingRemovals++;
        } else {
            metrics.connectivityBreakingRemovals++;
        }
        
        if (result.createdIsolatedNodes) {
            metrics.isolatingRemovals++;
        }
        
        metrics.totalNodesRemoved++;
        metrics.totalEdgesRemoved += result.removedEdges.size();
        
        // Update averages
        if (metrics.averageOperationTime == 0.0) {
            metrics.averageOperationTime = result.operationTime;
        } else {
            metrics.averageOperationTime = (metrics.averageOperationTime + result.operationTime) / 2.0;
        }
        
        if (metrics.averageTopologicalImpact == 0.0) {
            metrics.averageTopologicalImpact = result.topologicalImpact;
        } else {
            metrics.averageTopologicalImpact = (metrics.averageTopologicalImpact + result.topologicalImpact) / 2.0;
        }
        
        double nodeDegree = static_cast<double>(result.removedEdges.size());
        if (metrics.averageRemovedNodeDegree == 0.0) {
            metrics.averageRemovedNodeDegree = nodeDegree;
        } else {
            metrics.averageRemovedNodeDegree = (metrics.averageRemovedNodeDegree + nodeDegree) / 2.0;
        }
        
        double simplificationAmount = 1.0 / result.nodeCountBefore;
        metrics.totalGraphSimplification += simplificationAmount;
        
        // Analyze removal type
        for (const std::string& note : result.analysisNotes) {
            if (note.find("critical") != std::string::npos || note.find("High topological") != std::string::npos) {
                metrics.criticalNodeRemovals++;
            } else if (note.find("redundant") != std::string::npos) {
                metrics.redundantNodeRemovals++;
            } else if (note.find("high-degree") != std::string::npos) {
                metrics.highDegreeNodeRemovals++;
            }
        }
    }
    
    void updateFailureMetrics(const NodeRemovalResult& result, const std::string& reason) {
        metrics.totalRemovals++;
        metrics.failedRemovals++;
        metrics.lastOperation = result.operationTimestamp;
    }
};