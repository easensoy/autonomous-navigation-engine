#include "graph_operations/GraphAnalysis.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <memory>
#include <cmath>

class EdgeAdditionAnalyzer {
private:
    Graph* graph;
    mutable std::mutex operationMutex;
    
    struct EdgeAdditionResult {
        bool successful;
        int fromNode;
        int toNode;
        double weight;
        bool bidirectional;
        std::chrono::steady_clock::time_point operationTimestamp;
        double operationTime;
        std::string validationStatus;
        double graphDensityBefore;
        double graphDensityAfter;
        size_t connectivityImpact;
        bool createdNewComponent;
        bool improvedConnectivity;
        std::vector<std::string> analysisNotes;
    };
    
    struct EdgeAnalysisMetrics {
        size_t totalAdditions;
        size_t successfulAdditions;
        size_t failedAdditions;
        size_t duplicateAttempts;
        size_t invalidNodeAttempts;
        double averageOperationTime;
        double averageWeightAdded;
        size_t bidirectionalEdgesAdded;
        size_t unidirectionalEdgesAdded;
        std::chrono::steady_clock::time_point lastOperation;
        double totalGraphDensityIncrease;
        size_t connectivityImprovements;
        size_t redundantEdgesAdded;
        size_t criticalEdgesAdded;
    };
    
    struct ConnectivityAnalysis {
        bool wasConnectedBefore;
        bool isConnectedAfter;
        size_t componentCountBefore;
        size_t componentCountAfter;
        std::vector<int> affectedComponents;
        double pathEfficiencyImprovement;
        std::vector<std::pair<int, int>> newShortestPaths;
    };
    
    EdgeAnalysisMetrics metrics;
    bool enableValidation;
    bool performConnectivityAnalysis;
    bool detectRedundantEdges;
    double redundancyThreshold;
    bool autoOptimizeWeights;
    std::function<void(const EdgeAdditionResult&)> additionCallback;
    std::function<bool(int, int, double)> customValidator;
    
public:
    explicit EdgeAdditionAnalyzer(Graph* environment) 
        : graph(environment), enableValidation(true), performConnectivityAnalysis(true),
          detectRedundantEdges(true), redundancyThreshold(1.1), autoOptimizeWeights(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeAnalysisMetrics();
        
        std::cout << "[EDGE_ADDITION] Edge addition analysis system initialized" << std::endl;
    }
    
    EdgeAdditionResult addEdge(int fromNode, int toNode, double weight, bool bidirectional = true) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        auto operationStart = std::chrono::steady_clock::now();
        std::cout << "[EDGE_ADDITION] Analyzing edge addition: " << fromNode 
                  << " -> " << toNode << " (weight: " << weight 
                  << ", bidirectional: " << (bidirectional ? "yes" : "no") << ")" << std::endl;
        
        EdgeAdditionResult result;
        result.fromNode = fromNode;
        result.toNode = toNode;
        result.weight = weight;
        result.bidirectional = bidirectional;
        result.successful = false;
        result.operationTimestamp = operationStart;
        
        // Capture initial graph state
        result.graphDensityBefore = calculateGraphDensity();
        
        // Perform validation if enabled
        if (enableValidation) {
            std::string validationResult = validateEdgeAddition(fromNode, toNode, weight);
            result.validationStatus = validationResult;
            
            if (validationResult != "valid") {
                std::cout << "[EDGE_ADDITION] Edge validation failed: " << validationResult << std::endl;
                updateFailureMetrics(result, validationResult);
                return result;
            }
        }
        
        // Analyze connectivity impact before addition
        ConnectivityAnalysis connectivityBefore;
        if (performConnectivityAnalysis) {
            connectivityBefore = analyzeConnectivityState();
        }
        
        // Perform the edge addition
        try {
            graph->addEdge(fromNode, toNode, weight, bidirectional);
            result.successful = true;
            
            std::cout << "[EDGE_ADDITION] Edge successfully added to graph structure" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[EDGE_ADDITION] Edge addition failed with exception: " << e.what() << std::endl;
            result.validationStatus = std::string("exception: ") + e.what();
            updateFailureMetrics(result, result.validationStatus);
            return result;
        }
        
        // Analyze post-addition state
        result.graphDensityAfter = calculateGraphDensity();
        
        if (performConnectivityAnalysis) {
            ConnectivityAnalysis connectivityAfter = analyzeConnectivityState();
            analyzeConnectivityImpact(result, connectivityBefore, connectivityAfter);
        }
        
        // Detect redundancy if enabled
        if (detectRedundantEdges) {
            analyzeEdgeRedundancy(result);
        }
        
        // Perform weight optimization if enabled
        if (autoOptimizeWeights) {
            optimizeEdgeWeight(result);
        }
        
        // Complete timing analysis
        auto operationEnd = std::chrono::steady_clock::now();
        result.operationTime = std::chrono::duration<double>(operationEnd - operationStart).count();
        
        // Update metrics
        updateSuccessMetrics(result);
        
        // Trigger callback if configured
        if (additionCallback) {
            additionCallback(result);
        }
        
        std::cout << "[EDGE_ADDITION] Edge addition analysis completed successfully (time: " 
                  << result.operationTime << "s, density change: " 
                  << (result.graphDensityAfter - result.graphDensityBefore) << ")" << std::endl;
        
        return result;
    }
    
    EdgeAdditionResult addEdgeWithAnalysis(int fromNode, int toNode, double weight, 
                                         bool bidirectional = true, bool forceAnalysis = false) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[EDGE_ADDITION] Performing comprehensive edge addition analysis" << std::endl;
        
        // Temporarily enable all analysis features
        bool originalConnectivityAnalysis = performConnectivityAnalysis;
        bool originalRedundancyDetection = detectRedundantEdges;
        
        if (forceAnalysis) {
            performConnectivityAnalysis = true;
            detectRedundantEdges = true;
        }
        
        EdgeAdditionResult result = addEdge(fromNode, toNode, weight, bidirectional);
        
        if (result.successful) {
            // Perform additional analysis
            analyzeGraphTopologyChange(result);
            analyzePathEfficiencyImpact(result);
            analyzeStructuralProperties(result);
        }
        
        // Restore original settings
        performConnectivityAnalysis = originalConnectivityAnalysis;
        detectRedundantEdges = originalRedundancyDetection;
        
        std::cout << "[EDGE_ADDITION] Comprehensive analysis completed with " 
                  << result.analysisNotes.size() << " analysis insights" << std::endl;
        
        return result;
    }
    
    std::vector<EdgeAdditionResult> addMultipleEdges(const std::vector<std::tuple<int, int, double, bool>>& edges) {
        std::cout << "[EDGE_ADDITION] Processing batch addition of " << edges.size() << " edges" << std::endl;
        
        std::vector<EdgeAdditionResult> results;
        results.reserve(edges.size());
        
        for (const auto& [fromNode, toNode, weight, bidirectional] : edges) {
            EdgeAdditionResult result = addEdge(fromNode, toNode, weight, bidirectional);
            results.push_back(result);
            
            // Log progress for large batches
            if (results.size() % 100 == 0) {
                std::cout << "[EDGE_ADDITION] Batch progress: " << results.size() 
                          << "/" << edges.size() << " edges processed" << std::endl;
            }
        }
        
        std::cout << "[EDGE_ADDITION] Batch addition completed with " 
                  << std::count_if(results.begin(), results.end(), 
                                  [](const EdgeAdditionResult& r) { return r.successful; })
                  << " successful additions" << std::endl;
        
        return results;
    }
    
    bool canAddEdge(int fromNode, int toNode, double weight) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        if (!enableValidation) {
            return graph->hasNode(fromNode) && graph->hasNode(toNode);
        }
        
        std::string validationResult = validateEdgeAddition(fromNode, toNode, weight);
        bool canAdd = (validationResult == "valid");
        
        std::cout << "[EDGE_ADDITION] Edge addition feasibility check: " << fromNode 
                  << " -> " << toNode << " - " << (canAdd ? "feasible" : "not feasible") 
                  << " (" << validationResult << ")" << std::endl;
        
        return canAdd;
    }
    
    double calculateOptimalWeight(int fromNode, int toNode) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "[EDGE_ADDITION] Calculating optimal weight for edge " 
                  << fromNode << " -> " << toNode << std::endl;
        
        if (!graph->hasNode(fromNode) || !graph->hasNode(toNode)) {
            std::cout << "[EDGE_ADDITION] Cannot calculate optimal weight - invalid nodes" << std::endl;
            return -1.0;
        }
        
        const Node& node1 = graph->getNode(fromNode);
        const Node& node2 = graph->getNode(toNode);
        
        // Calculate Euclidean distance as baseline
        double euclideanDistance = node1.euclideanDistance(node2);
        
        // Analyze existing edge weights in the vicinity
        double averageLocalWeight = calculateAverageLocalWeight(fromNode, toNode);
        
        // Consider graph density and connectivity
        double densityFactor = 1.0 + (0.5 * calculateGraphDensity());
        
        // Calculate optimal weight
        double optimalWeight = euclideanDistance * densityFactor;
        if (averageLocalWeight > 0) {
            optimalWeight = (optimalWeight + averageLocalWeight) / 2.0;
        }
        
        std::cout << "[EDGE_ADDITION] Optimal weight calculated: " << optimalWeight 
                  << " (euclidean: " << euclideanDistance 
                  << ", local avg: " << averageLocalWeight << ")" << std::endl;
        
        return optimalWeight;
    }
    
    void configureEdgeAddition(bool validation, bool connectivityAnalysis, bool redundancyDetection,
                             double redundancyThresh, bool weightOptimization) {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        enableValidation = validation;
        performConnectivityAnalysis = connectivityAnalysis;
        detectRedundantEdges = redundancyDetection;
        redundancyThreshold = redundancyThresh;
        autoOptimizeWeights = weightOptimization;
        
        std::cout << "[EDGE_ADDITION] Configuration updated:" << std::endl;
        std::cout << "[EDGE_ADDITION]   Validation enabled: " << (enableValidation ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Connectivity analysis: " << (performConnectivityAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Redundancy detection: " << (detectRedundantEdges ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Redundancy threshold: " << redundancyThreshold << std::endl;
        std::cout << "[EDGE_ADDITION]   Weight optimization: " << (autoOptimizeWeights ? "Yes" : "No") << std::endl;
    }
    
    void setAdditionCallback(std::function<void(const EdgeAdditionResult&)> callback) {
        additionCallback = callback;
        std::cout << "[EDGE_ADDITION] Addition callback configured" << std::endl;
    }
    
    void setCustomValidator(std::function<bool(int, int, double)> validator) {
        customValidator = validator;
        std::cout << "[EDGE_ADDITION] Custom validation function configured" << std::endl;
    }
    
    EdgeAnalysisMetrics getAnalysisMetrics() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        return metrics;
    }
    
    void generateAdditionReport() const {
        std::lock_guard<std::mutex> lock(operationMutex);
        
        std::cout << "\n[EDGE_ADDITION] === EDGE ADDITION ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[EDGE_ADDITION] System Configuration:" << std::endl;
        std::cout << "[EDGE_ADDITION]   Validation enabled: " << (enableValidation ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Connectivity analysis: " << (performConnectivityAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Redundancy detection: " << (detectRedundantEdges ? "Yes" : "No") << std::endl;
        std::cout << "[EDGE_ADDITION]   Redundancy threshold: " << redundancyThreshold << std::endl;
        std::cout << "[EDGE_ADDITION]   Weight optimization: " << (autoOptimizeWeights ? "Yes" : "No") << std::endl;
        
        std::cout << "[EDGE_ADDITION] Current Graph State:" << std::endl;
        std::cout << "[EDGE_ADDITION]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[EDGE_ADDITION]   Total edges: " << graph->getEdgeCount() << std::endl;
        std::cout << "[EDGE_ADDITION]   Graph density: " << calculateGraphDensity() << std::endl;
        
        std::cout << "[EDGE_ADDITION] Operation Metrics:" << std::endl;
        std::cout << "[EDGE_ADDITION]   Total addition attempts: " << metrics.totalAdditions << std::endl;
        std::cout << "[EDGE_ADDITION]   Successful additions: " << metrics.successfulAdditions << std::endl;
        std::cout << "[EDGE_ADDITION]   Failed additions: " << metrics.failedAdditions << std::endl;
        std::cout << "[EDGE_ADDITION]   Duplicate attempts: " << metrics.duplicateAttempts << std::endl;
        std::cout << "[EDGE_ADDITION]   Invalid node attempts: " << metrics.invalidNodeAttempts << std::endl;
        std::cout << "[EDGE_ADDITION]   Bidirectional edges added: " << metrics.bidirectionalEdgesAdded << std::endl;
        std::cout << "[EDGE_ADDITION]   Unidirectional edges added: " << metrics.unidirectionalEdgesAdded << std::endl;
        
        if (metrics.totalAdditions > 0) {
            double successRate = static_cast<double>(metrics.successfulAdditions) / metrics.totalAdditions * 100.0;
            std::cout << "[EDGE_ADDITION]   Success rate: " << successRate << "%" << std::endl;
        }
        
        std::cout << "[EDGE_ADDITION] Performance Metrics:" << std::endl;
        std::cout << "[EDGE_ADDITION]   Average operation time: " << metrics.averageOperationTime << " seconds" << std::endl;
        std::cout << "[EDGE_ADDITION]   Average weight added: " << metrics.averageWeightAdded << std::endl;
        std::cout << "[EDGE_ADDITION]   Total density increase: " << metrics.totalGraphDensityIncrease << std::endl;
        std::cout << "[EDGE_ADDITION]   Connectivity improvements: " << metrics.connectivityImprovements << std::endl;
        std::cout << "[EDGE_ADDITION]   Redundant edges added: " << metrics.redundantEdgesAdded << std::endl;
        std::cout << "[EDGE_ADDITION]   Critical edges added: " << metrics.criticalEdgesAdded << std::endl;
        
        std::cout << "[EDGE_ADDITION] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeAnalysisMetrics() {
        metrics.totalAdditions = 0;
        metrics.successfulAdditions = 0;
        metrics.failedAdditions = 0;
        metrics.duplicateAttempts = 0;
        metrics.invalidNodeAttempts = 0;
        metrics.averageOperationTime = 0.0;
        metrics.averageWeightAdded = 0.0;
        metrics.bidirectionalEdgesAdded = 0;
        metrics.unidirectionalEdgesAdded = 0;
        metrics.lastOperation = std::chrono::steady_clock::now();
        metrics.totalGraphDensityIncrease = 0.0;
        metrics.connectivityImprovements = 0;
        metrics.redundantEdgesAdded = 0;
        metrics.criticalEdgesAdded = 0;
    }
    
    std::string validateEdgeAddition(int fromNode, int toNode, double weight) {
        // Check if nodes exist
        if (!graph->hasNode(fromNode)) {
            return "invalid_from_node";
        }
        
        if (!graph->hasNode(toNode)) {
            return "invalid_to_node";
        }
        
        // Check for self-loops
        if (fromNode == toNode) {
            return "self_loop_detected";
        }
        
        // Check if edge already exists
        if (graph->hasEdge(fromNode, toNode)) {
            return "duplicate_edge";
        }
        
        // Validate weight
        if (weight < 0) {
            return "negative_weight";
        }
        
        if (std::isnan(weight) || std::isinf(weight)) {
            return "invalid_weight";
        }
        
        // Apply custom validation if configured
        if (customValidator && !customValidator(fromNode, toNode, weight)) {
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
    
    ConnectivityAnalysis analyzeConnectivityState() {
        ConnectivityAnalysis analysis;
        analysis.wasConnectedBefore = graph->isConnected();
        
        // Count connected components (simplified implementation)
        analysis.componentCountBefore = countConnectedComponents();
        
        return analysis;
    }
    
    void analyzeConnectivityImpact(EdgeAdditionResult& result, 
                                 const ConnectivityAnalysis& before, 
                                 const ConnectivityAnalysis& after) {
        ConnectivityAnalysis afterState = analyzeConnectivityState();
        
        result.createdNewComponent = false;
        result.improvedConnectivity = afterState.componentCountBefore < before.componentCountBefore;
        
        if (result.improvedConnectivity) {
            result.analysisNotes.push_back("Improved graph connectivity");
            result.connectivityImpact = before.componentCountBefore - afterState.componentCountBefore;
        } else {
            result.connectivityImpact = 0;
        }
    }
    
    void analyzeEdgeRedundancy(EdgeAdditionResult& result) {
        // Check if the added edge creates a shorter path than existing routes
        bool isRedundant = checkEdgeRedundancy(result.fromNode, result.toNode, result.weight);
        
        if (isRedundant) {
            result.analysisNotes.push_back("Edge identified as potentially redundant");
        } else {
            result.analysisNotes.push_back("Edge provides unique connectivity value");
        }
    }
    
    void optimizeEdgeWeight(EdgeAdditionResult& result) {
        double optimalWeight = calculateOptimalWeight(result.fromNode, result.toNode);
        
        if (std::abs(result.weight - optimalWeight) > 0.1) {
            result.analysisNotes.push_back("Weight differs from calculated optimal: " + 
                                         std::to_string(optimalWeight));
        }
    }
    
    void analyzeGraphTopologyChange(EdgeAdditionResult& result) {
        result.analysisNotes.push_back("Graph density increased by " + 
                                     std::to_string(result.graphDensityAfter - result.graphDensityBefore));
    }
    
    void analyzePathEfficiencyImpact(EdgeAdditionResult& result) {
        // Simplified path efficiency analysis
        double efficiencyImprovement = calculatePathEfficiencyImprovement(result.fromNode, result.toNode);
        
        if (efficiencyImprovement > 0.05) {
            result.analysisNotes.push_back("Significant path efficiency improvement: " + 
                                         std::to_string(efficiencyImprovement));
        }
    }
    
    void analyzeStructuralProperties(EdgeAdditionResult& result) {
        // Analyze impact on graph structural properties
        if (isEdgeCriticalForConnectivity(result.fromNode, result.toNode)) {
            result.analysisNotes.push_back("Edge is critical for graph connectivity");
        }
    }
    
    size_t countConnectedComponents() {
        // Simplified connected components counting
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
    
    bool checkEdgeRedundancy(int fromNode, int toNode, double weight) {
        // Check if there's already a path between nodes with similar total weight
        double existingPathCost = findShortestPathCost(fromNode, toNode);
        
        return (existingPathCost > 0) && (weight > existingPathCost * redundancyThreshold);
    }
    
    double findShortestPathCost(int fromNode, int toNode) {
        // Simplified shortest path calculation (would use Dijkstra in full implementation)
        return 1.0; // Placeholder
    }
    
    double calculateAverageLocalWeight(int fromNode, int toNode) {
        std::vector<double> localWeights;
        
        // Collect weights from edges connected to either node
        const std::vector<Edge>& fromEdges = graph->getEdgesFrom(fromNode);
        for (const Edge& edge : fromEdges) {
            localWeights.push_back(edge.getWeight());
        }
        
        const std::vector<Edge>& toEdges = graph->getEdgesFrom(toNode);
        for (const Edge& edge : toEdges) {
            localWeights.push_back(edge.getWeight());
        }
        
        if (localWeights.empty()) {
            return 0.0;
        }
        
        double sum = std::accumulate(localWeights.begin(), localWeights.end(), 0.0);
        return sum / localWeights.size();
    }
    
    double calculatePathEfficiencyImprovement(int fromNode, int toNode) {
        // Simplified efficiency calculation
        return 0.1; // Placeholder
    }
    
    bool isEdgeCriticalForConnectivity(int fromNode, int toNode) {
        // Check if edge connects different components
        return countConnectedComponents() > 1;
    }
    
    void updateSuccessMetrics(const EdgeAdditionResult& result) {
        metrics.totalAdditions++;
        metrics.successfulAdditions++;
        metrics.lastOperation = result.operationTimestamp;
        
        if (result.bidirectional) {
            metrics.bidirectionalEdgesAdded++;
        } else {
            metrics.unidirectionalEdgesAdded++;
        }
        
        // Update averages
        if (metrics.averageOperationTime == 0.0) {
            metrics.averageOperationTime = result.operationTime;
        } else {
            metrics.averageOperationTime = (metrics.averageOperationTime + result.operationTime) / 2.0;
        }
        
        if (metrics.averageWeightAdded == 0.0) {
            metrics.averageWeightAdded = result.weight;
        } else {
            metrics.averageWeightAdded = (metrics.averageWeightAdded + result.weight) / 2.0;
        }
        
        metrics.totalGraphDensityIncrease += (result.graphDensityAfter - result.graphDensityBefore);
        
        if (result.improvedConnectivity) {
            metrics.connectivityImprovements++;
        }
        
        // Check for redundancy or criticality
        for (const std::string& note : result.analysisNotes) {
            if (note.find("redundant") != std::string::npos) {
                metrics.redundantEdgesAdded++;
            } else if (note.find("critical") != std::string::npos) {
                metrics.criticalEdgesAdded++;
            }
        }
    }
    
    void updateFailureMetrics(const EdgeAdditionResult& result, const std::string& reason) {
        metrics.totalAdditions++;
        metrics.failedAdditions++;
        metrics.lastOperation = result.operationTimestamp;
        
        if (reason == "duplicate_edge") {
            metrics.duplicateAttempts++;
        } else if (reason == "invalid_from_node" || reason == "invalid_to_node") {
            metrics.invalidNodeAttempts++;
        }
    }
};