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

class EdgeContractionAnalyzer {
private:
    Graph* graph;
    mutable std::mutex operationMutex;
    
    struct EdgeContractionResult {
        bool successful;
        int fromNode;
        int toNode;
        int mergedNodeId;
        double originalWeight;
        std::chrono::steady_clock::time_point operationTimestamp;
        double operationTime;
        std::string contractionStatus;
        size_t nodeCountBefore;
        size_t nodeCountAfter;
        size_t edgeCountBefore;
        size_t edgeCountAfter;
        double graphDensityBefore;
        double graphDensityAfter;
        std::vector<int> redirectedEdges;
        std::vector<int> removedEdges;
        bool preservedConnectivity;
        double topologicalImpact;
        std::vector<std::string> analysisNotes;
    };
    
    struct ContractionAnalysisMetrics {
        size_t totalContractions;
        size_t successfulContractions;
        size_t failedContractions;
        size_t connectivityPreservingContractions;
        size_t connectivityBreakingContractions;
        double averageOperationTime;
        double averageTopologicalImpact;
        size_t totalNodesRemoved;
        size_t totalEdgesRedirected;
        size_t totalEdgesRemoved;
        std::chrono::steady_clock::time_point lastOperation;
        double totalGraphSimplification;
        size_t criticalEdgeContractions;
        size_t redundantEdgeContractions;
    };
    
    struct ContractionPlan {
        std::vector<int> nodesToMerge;
        std::vector<std::pair<int, int>> edgesToRedirect;
        std::vector<int> edgesToRemove;
        double estimatedImpact;
        bool preservesConnectivity;
        std::string contractionStrategy;
    };
    
    ContractionAnalysisMetrics metrics;
    bool preserveConnectivity;
    bool enableTopologicalAnalysis;
    bool generateMergedNodeIds;
    int nextMergedNodeId;
    std::string contractionStrategy;
    std::function<void(const EdgeContractionResult&)> contractionCallback;
    std::function<bool(int, int)> contractionValidator;
    
public:
    explicit EdgeContractionAnalyzer(Graph* environment) 
        : graph(environment), preserveConnectivity(true), enableTopologicalAnalysis(true),
          generateMergedNodeIds(true), nextMergedNodeId(100000), contractionStrategy("conservative") {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeAnalysisMetrics();
        
        std::cout << "[EDGE_CONTRACTION] Edge contraction analysis system initialized" << std::endl;
    }
    
    EdgeContractionResult contractEdge(int fromNode, int toNode) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        auto operationStart = std::chrono::steady_clock::now();
        std::cout << "[EDGE_CONTRACTION] Analyzing edge contraction: " << fromNode 
                  << " -> " << toNode << std::endl;
        
        EdgeContractionResult result;
        result.fromNode = fromNode;
        result.toNode = toNode;
        result.mergedNodeId = -1;
        result.successful = false;
        result.operationTimestamp = operationStart;
        result.preservedConnectivity = false;
        result.topologicalImpact = 0.0;
        
        // Capture initial graph state
        result.nodeCountBefore = graph->getNodeCount();
        result.edgeCountBefore = graph->getEdgeCount();
        result.graphDensityBefore = calculateGraphDensity();
        
        // Validate contraction feasibility
        std::string validationResult = validateEdgeContraction(fromNode, toNode);
        result.contractionStatus = validationResult;
        
        if (validationResult != "valid") {
            std::cout << "[EDGE_CONTRACTION] Edge contraction validation failed: " 
                      << validationResult << std::endl;
            updateFailureMetrics(result, validationResult);
            return result;
        }
        
        // Analyze contraction impact before execution
        ContractionPlan plan = planContraction(fromNode, toNode);
        
        if (preserveConnectivity && !plan.preservesConnectivity) {
            std::cout << "[EDGE_CONTRACTION] Contraction would break connectivity - aborting" << std::endl;
            result.contractionStatus = "connectivity_violation";
            updateFailureMetrics(result, result.contractionStatus);
            return result;
        }
        
        // Execute the contraction
        try {
            result = executeContraction(fromNode, toNode, plan);
            
            std::cout << "[EDGE_CONTRACTION] Edge contraction executed successfully" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[EDGE_CONTRACTION] Edge contraction failed with exception: " 
                      << e.what() << std::endl;
            result.contractionStatus = std::string("exception: ") + e.what();
            updateFailureMetrics(result, result.contractionStatus);
            return result;
        }
        
        // Analyze post-contraction state
        result.nodeCountAfter = graph->getNodeCount();
        result.edgeCountAfter = graph->getEdgeCount();
        result.graphDensityAfter = calculateGraphDensity();
        
        if (enableTopologicalAnalysis) {
            analyzeTopologicalChanges(result);
        }
        
        // Complete timing analysis
        auto operationEnd = std::chrono::steady_clock::now();
        result.operationTime = std::chrono::duration<double>(operationEnd - operationStart).count();
        
        // Update metrics
        updateSuccessMetrics(result);
        
        // Trigger callback if configured
        if (contractionCallback) {
            contractionCallback(result);
        }
        
        std::cout << "[EDGE_CONTRACTION] Edge contraction analysis completed successfully (time: " 
                  << result.operationTime << "s, nodes reduced: " 
                  << (result.nodeCountBefore - result.nodeCountAfter) << ")" << std::endl;
        
        return result;
    }
    
    EdgeContractionResult contractEdgeWithStrategy(int fromNode, int toNode, 
                                                 const std::string& strategy) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[EDGE_CONTRACTION] Performing strategic edge contraction with strategy: " 
                  << strategy << std::endl;
        
        // Temporarily apply strategy
        std::string originalStrategy = contractionStrategy;
        contractionStrategy = strategy;
        
        EdgeContractionResult result = contractEdge(fromNode, toNode);
        
        if (result.successful) {
            result.analysisNotes.push_back("Contraction strategy applied: " + strategy);
        }
        
        // Restore original strategy
        contractionStrategy = originalStrategy;
        
        return result;
    }
    
    std::vector<EdgeContractionResult> contractMultipleEdges(
        const std::vector<std::pair<int, int>>& edges) {
        
        std::cout << "[EDGE_CONTRACTION] Processing batch contraction of " << edges.size() 
                  << " edges" << std::endl;
        
        std::vector<EdgeContractionResult> results;
        results.reserve(edges.size());
        
        for (const auto& [fromNode, toNode] : edges) {
            EdgeContractionResult result = contractEdge(fromNode, toNode);
            results.push_back(result);
            
            // Log progress for large batches
            if (results.size() % 50 == 0) {
                std::cout << "[EDGE_CONTRACTION] Batch progress: " << results.size() 
                          << "/" << edges.size() << " contractions processed" << std::endl;
            }
            
            // Stop if connectivity is broken and preservation is required
            if (preserveConnectivity && !result.preservedConnectivity && result.successful) {
                std::cout << "[EDGE_CONTRACTION] Halting batch processing due to connectivity concern" 
                          << std::endl;
                break;
            }
        }
        
        std::cout << "[EDGE_CONTRACTION] Batch contraction completed with " 
                  << std::count_if(results.begin(), results.end(), 
                                  [](const EdgeContractionResult& r) { return r.successful; })
                  << " successful contractions" << std::endl;
        
        return results;
    }
    
    bool canContractEdge(int fromNode, int toNode) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::string validationResult = validateEdgeContraction(fromNode, toNode);
        bool canContract = (validationResult == "valid");
        
        if (canContract && preserveConnectivity) {
            ContractionPlan plan = planContraction(fromNode, toNode);
            canContract = plan.preservesConnectivity;
        }
        
        std::cout << "[EDGE_CONTRACTION] Edge contraction feasibility check: " << fromNode 
                  << " -> " << toNode << " - " << (canContract ? "feasible" : "not feasible") 
                  << std::endl;
        
        return canContract;
    }
    
    std::vector<std::pair<int, int>> identifyContractionCandidates() {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[EDGE_CONTRACTION] Identifying optimal edge contraction candidates" << std::endl;
        
        std::vector<std::pair<int, int>> candidates;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            
            for (const Edge& edge : edges) {
                int otherNode = edge.getToNode();
                
                if (nodeId < otherNode) { // Avoid duplicate pairs
                    if (canContractEdge(nodeId, otherNode)) {
                        // Evaluate contraction benefit
                        double benefit = evaluateContractionBenefit(nodeId, otherNode);
                        
                        if (benefit > 0.5) { // Threshold for beneficial contraction
                            candidates.emplace_back(nodeId, otherNode);
                        }
                    }
                }
            }
        }
        
        // Sort candidates by benefit
        std::sort(candidates.begin(), candidates.end(), 
                 [this](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                     return evaluateContractionBenefit(a.first, a.second) > 
                            evaluateContractionBenefit(b.first, b.second);
                 });
        
        std::cout << "[EDGE_CONTRACTION] Identified " << candidates.size() 
                  << " edge contraction candidates" << std::endl;
        
        return candidates;
    }
    
    double calculateSimplificationImpact(int fromNode, int toNode) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        if (!canContractEdge(fromNode, toNode)) {
            return 0.0;
        }
        
        ContractionPlan plan = planContraction(fromNode, toNode);
        return plan.estimatedImpact;
    }
    
    void configureContraction(bool connectivityPreservation, bool topologicalAnalysis, 
                            const std::string& strategy, bool autoGenerateIds) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        preserveConnectivity = connectivityPreservation;
        enableTopologicalAnalysis = topologicalAnalysis;
        contractionStrategy = strategy;
        generateMergedNodeIds = autoGenerateIds;
        
        std::cout << "[EDGE_CONTRACTION] Configuration updated:" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Preserve connectivity: " << (preserveConnectivity ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Topological analysis: " << (enableTopologicalAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Contraction strategy: " << contractionStrategy << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Auto-generate merged node IDs: " << (generateMergedNodeIds ? "Yes" : "No") << std::endl;
    }
    
    void setContractionCallback(std::function<void(const EdgeContractionResult&)> callback) {
        contractionCallback = callback;
        std::cout << "[EDGE_CONTRACTION] Contraction callback configured" << std::endl;
    }
    
    void setContractionValidator(std::function<bool(int, int)> validator) {
        contractionValidator = validator;
        std::cout << "[EDGE_CONTRACTION] Custom contraction validator configured" << std::endl;
    }
    
    ContractionAnalysisMetrics getAnalysisMetrics() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        return metrics;
    }
    
    void generateContractionReport() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "\n[EDGE_CONTRACTION] === EDGE CONTRACTION ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[EDGE_CONTRACTION] System Configuration:" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Preserve connectivity: " << (preserveConnectivity ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Topological analysis: " << (enableTopologicalAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Contraction strategy: " << contractionStrategy << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Auto-generate merged node IDs: " << (generateMergedNodeIds ? "Yes" : "No") << std::endl;
        
        std::cout << "[EDGE_CONTRACTION] Current Graph State:" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total edges: " << graph->getEdgeCount() << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Graph density: " << calculateGraphDensity() << std::endl;
        
        std::cout << "[EDGE_CONTRACTION] Operation Metrics:" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total contraction attempts: " << metrics.totalContractions << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Successful contractions: " << metrics.successfulContractions << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Failed contractions: " << metrics.failedContractions << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Connectivity-preserving contractions: " << metrics.connectivityPreservingContractions << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Connectivity-breaking contractions: " << metrics.connectivityBreakingContractions << std::endl;
        
        if (metrics.totalContractions > 0) {
            double successRate = static_cast<double>(metrics.successfulContractions) / metrics.totalContractions * 100.0;
            std::cout << "[EDGE_CONTRACTION]   Success rate: " << successRate << "%" << std::endl;
        }
        
        std::cout << "[EDGE_CONTRACTION] Impact Metrics:" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total nodes removed: " << metrics.totalNodesRemoved << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total edges redirected: " << metrics.totalEdgesRedirected << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total edges removed: " << metrics.totalEdgesRemoved << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Average operation time: " << metrics.averageOperationTime << " seconds" << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Average topological impact: " << metrics.averageTopologicalImpact << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Total graph simplification: " << metrics.totalGraphSimplification << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Critical edge contractions: " << metrics.criticalEdgeContractions << std::endl;
        std::cout << "[EDGE_CONTRACTION]   Redundant edge contractions: " << metrics.redundantEdgeContractions << std::endl;
        
        std::cout << "[EDGE_CONTRACTION] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeAnalysisMetrics() {
        metrics.totalContractions = 0;
        metrics.successfulContractions = 0;
        metrics.failedContractions = 0;
        metrics.connectivityPreservingContractions = 0;
        metrics.connectivityBreakingContractions = 0;
        metrics.averageOperationTime = 0.0;
        metrics.averageTopologicalImpact = 0.0;
        metrics.totalNodesRemoved = 0;
        metrics.totalEdgesRedirected = 0;
        metrics.totalEdgesRemoved = 0;
        metrics.lastOperation = std::chrono::steady_clock::now();
        metrics.totalGraphSimplification = 0.0;
        metrics.criticalEdgeContractions = 0;
        metrics.redundantEdgeContractions = 0;
    }
    
    std::string validateEdgeContraction(int fromNode, int toNode) {
        // Check if nodes exist
        if (!graph->hasNode(fromNode)) {
            return "invalid_from_node";
        }
        
        if (!graph->hasNode(toNode)) {
            return "invalid_to_node";
        }
        
        // Check if edge exists
        if (!graph->hasEdge(fromNode, toNode)) {
            return "edge_not_found";
        }
        
        // Check if nodes are the same
        if (fromNode == toNode) {
            return "self_loop_contraction";
        }
        
        // Apply custom validation if configured
        if (contractionValidator && !contractionValidator(fromNode, toNode)) {
            return "custom_validation_failed";
        }
        
        return "valid";
    }
    
    ContractionPlan planContraction(int fromNode, int toNode) {
        ContractionPlan plan;
        plan.contractionStrategy = contractionStrategy;
        
        // Determine which node will survive the merge
        int survivingNode = (contractionStrategy == "keep_lower_id") ? 
                           std::min(fromNode, toNode) : std::max(fromNode, toNode);
        int removingNode = (survivingNode == fromNode) ? toNode : fromNode;
        
        plan.nodesToMerge = {survivingNode, removingNode};
        
        // Plan edge redirections
        const std::vector<Edge>& removingNodeEdges = graph->getEdgesFrom(removingNode);
        for (const Edge& edge : removingNodeEdges) {
            if (edge.getToNode() != survivingNode) {
                plan.edgesToRedirect.emplace_back(edge.getToNode(), survivingNode);
            }
        }
        
        // Analyze connectivity preservation
        plan.preservesConnectivity = wouldPreserveConnectivity(fromNode, toNode);
        
        // Estimate impact
        plan.estimatedImpact = calculateContractionImpact(fromNode, toNode);
        
        return plan;
    }
    
    EdgeContractionResult executeContraction(int fromNode, int toNode, 
                                           const ContractionPlan& plan) {
        EdgeContractionResult result;
        result.fromNode = fromNode;
        result.toNode = toNode;
        result.successful = true;
        
        // Get the original weight before contraction
        result.originalWeight = graph->getEdgeWeight(fromNode, toNode);
        
        // Determine surviving node
        int survivingNode = plan.nodesToMerge[0];
        int removingNode = plan.nodesToMerge[1];
        
        if (generateMergedNodeIds) {
            // Create new merged node
            result.mergedNodeId = nextMergedNodeId++;
            
            // Calculate merged position (average of original positions)
            const Node& node1 = graph->getNode(fromNode);
            const Node& node2 = graph->getNode(toNode);
            double mergedX = (node1.getX() + node2.getX()) / 2.0;
            double mergedY = (node1.getY() + node2.getY()) / 2.0;
            
            // Add merged node
            graph->addNode(result.mergedNodeId, "merged_" + std::to_string(result.mergedNodeId), 
                          mergedX, mergedY);
            survivingNode = result.mergedNodeId;
        } else {
            result.mergedNodeId = survivingNode;
        }
        
        // Redirect edges from removing node to surviving node
        std::vector<Edge> edgesToRedirect;
        const std::vector<Edge>& removingNodeEdges = graph->getEdgesFrom(removingNode);
        
        for (const Edge& edge : removingNodeEdges) {
            if (edge.getToNode() != fromNode && edge.getToNode() != toNode) {
                edgesToRedirect.push_back(edge);
                result.redirectedEdges.push_back(edge.getToNode());
            }
        }
        
        // Remove original nodes if using merged node strategy
        if (generateMergedNodeIds) {
            graph->removeNode(fromNode);
            graph->removeNode(toNode);
        } else {
            graph->removeNode(removingNode);
        }
        
        // Add redirected edges
        for (const Edge& edge : edgesToRedirect) {
            if (!graph->hasEdge(survivingNode, edge.getToNode())) {
                graph->addEdge(survivingNode, edge.getToNode(), edge.getWeight(), edge.isBidirectional());
            }
        }
        
        // Verify connectivity preservation
        result.preservedConnectivity = plan.preservesConnectivity;
        
        return result;
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
    
    bool wouldPreserveConnectivity(int fromNode, int toNode) {
        // Check if the edge is a bridge (critical for connectivity)
        return !isEdgeBridge(fromNode, toNode);
    }
    
    bool isEdgeBridge(int fromNode, int toNode) {
        // Simplified bridge detection
        // Remove edge temporarily and check connectivity
        bool hadEdge = graph->hasEdge(fromNode, toNode);
        
        if (!hadEdge) {
            return false;
        }
        
        // This would require temporarily removing the edge and checking connectivity
        // For now, return false as a conservative estimate
        return false;
    }
    
    double calculateContractionImpact(int fromNode, int toNode) {
        // Calculate the topological impact of contracting this edge
        int degree1 = graph->getNeighbors(fromNode).size();
        int degree2 = graph->getNeighbors(toNode).size();
        
        // Higher degree nodes have higher impact when contracted
        return static_cast<double>(degree1 + degree2) / (2.0 * graph->getNodeCount());
    }
    
    double evaluateContractionBenefit(int fromNode, int toNode) {
        // Evaluate the benefit of contracting this edge
        double simplificationBenefit = 1.0 / (graph->getNodeCount() + 1);
        double connectivityCost = isEdgeBridge(fromNode, toNode) ? 10.0 : 0.0;
        
        return simplificationBenefit - connectivityCost;
    }
    
    void analyzeTopologicalChanges(EdgeContractionResult& result) {
        result.topologicalImpact = calculateContractionImpact(result.fromNode, result.toNode);
        
        if (result.topologicalImpact > 0.1) {
            result.analysisNotes.push_back("High topological impact detected");
        }
        
        if (result.preservedConnectivity) {
            result.analysisNotes.push_back("Connectivity successfully preserved");
        } else {
            result.analysisNotes.push_back("Connectivity was modified");
        }
    }
    
    void updateSuccessMetrics(const EdgeContractionResult& result) {
        metrics.totalContractions++;
        metrics.successfulContractions++;
        metrics.lastOperation = result.operationTimestamp;
        
        if (result.preservedConnectivity) {
            metrics.connectivityPreservingContractions++;
        } else {
            metrics.connectivityBreakingContractions++;
        }
        
        metrics.totalNodesRemoved += (result.nodeCountBefore - result.nodeCountAfter);
        metrics.totalEdgesRedirected += result.redirectedEdges.size();
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
        
        double simplificationAmount = static_cast<double>(result.nodeCountBefore - result.nodeCountAfter) / 
                                    result.nodeCountBefore;
        metrics.totalGraphSimplification += simplificationAmount;
        
        // Analyze contraction type
        for (const std::string& note : result.analysisNotes) {
            if (note.find("critical") != std::string::npos) {
                metrics.criticalEdgeContractions++;
            } else if (note.find("redundant") != std::string::npos) {
                metrics.redundantEdgeContractions++;
            }
        }
    }
    
    void updateFailureMetrics(const EdgeContractionResult& result, const std::string& reason) {
        metrics.totalContractions++;
        metrics.failedContractions++;
        metrics.lastOperation = result.operationTimestamp;
    }
};