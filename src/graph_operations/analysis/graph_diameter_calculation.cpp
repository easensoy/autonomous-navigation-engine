#include "graph_operations/GraphAnalysis.hpp"
#include <iostream>
#include <limits>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>

class GraphDiameterCalculator {
private:
    const Graph* graph;
    mutable std::mutex calculationMutex;
    
    struct DiameterResult {
        double diameter;
        int sourceNode;
        int targetNode;
        std::vector<int> longestPath;
        double calculationTime;
        std::chrono::steady_clock::time_point calculationTimestamp;
        bool isValid;
        std::string calculationMethod;
        size_t nodesProcessed;
        size_t pathsEvaluated;
    };
    
    struct CalculationMetrics {
        size_t totalCalculations;
        double averageCalculationTime;
        double lastDiameter;
        size_t cacheHits;
        size_t cacheMisses;
        std::chrono::steady_clock::time_point lastCalculation;
        double averageNodesProcessed;
        double averagePathsEvaluated;
        size_t approximateCalculations;
        size_t exactCalculations;
    };
    
    DiameterResult cachedResult;
    CalculationMetrics metrics;
    bool cacheEnabled;
    double cacheValidityDuration;
    bool approximateMode;
    double approximationAccuracy;
    size_t maxNodesForExactCalculation;
    std::function<void(const DiameterResult&)> resultCallback;
    
public:
    explicit GraphDiameterCalculator(const Graph* environment) 
        : graph(environment), cacheEnabled(true), cacheValidityDuration(300.0),
          approximateMode(false), approximationAccuracy(0.95), maxNodesForExactCalculation(1000) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeCalculationMetrics();
        invalidateCachedResult();
        
        std::cout << "[GRAPH_DIAMETER] Graph diameter calculation system initialized" << std::endl;
    }
    
    DiameterResult calculateDiameter() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        auto calculationStart = std::chrono::steady_clock::now();
        std::cout << "[GRAPH_DIAMETER] Initiating graph diameter calculation" << std::endl;
        
        // Check cache validity
        if (cacheEnabled && isCachedResultValid()) {
            std::cout << "[GRAPH_DIAMETER] Using cached diameter result" << std::endl;
            metrics.cacheHits++;
            return cachedResult;
        }
        
        metrics.cacheMisses++;
        
        // Determine calculation method based on graph size
        bool useApproximate = approximateMode || 
                             (graph->getNodeCount() > maxNodesForExactCalculation);
        
        DiameterResult result;
        if (useApproximate) {
            std::cout << "[GRAPH_DIAMETER] Using approximate diameter calculation" << std::endl;
            result = calculateApproximateDiameter();
            metrics.approximateCalculations++;
        } else {
            std::cout << "[GRAPH_DIAMETER] Using exact diameter calculation" << std::endl;
            result = calculateExactDiameter();
            metrics.exactCalculations++;
        }
        
        auto calculationEnd = std::chrono::steady_clock::now();
        result.calculationTime = std::chrono::duration<double>(calculationEnd - calculationStart).count();
        result.calculationTimestamp = calculationEnd;
        result.isValid = true;
        
        // Update metrics
        updateCalculationMetrics(result);
        
        // Cache the result
        if (cacheEnabled) {
            cachedResult = result;
        }
        
        // Trigger callback if configured
        if (resultCallback) {
            resultCallback(result);
        }
        
        std::cout << "[GRAPH_DIAMETER] Diameter calculation completed: " << result.diameter 
                  << " (method: " << result.calculationMethod << ", time: " 
                  << result.calculationTime << "s)" << std::endl;
        
        return result;
    }
    
    DiameterResult calculateExactDiameter() {
        std::cout << "[GRAPH_DIAMETER] Executing exact diameter calculation using all-pairs shortest paths" << std::endl;
        
        DiameterResult result;
        result.diameter = 0.0;
        result.sourceNode = -1;
        result.targetNode = -1;
        result.calculationMethod = "exact_all_pairs";
        result.nodesProcessed = 0;
        result.pathsEvaluated = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[GRAPH_DIAMETER] Empty graph - diameter is 0" << std::endl;
            result.isValid = false;
            return result;
        }
        
        if (nodeIds.size() == 1) {
            std::cout << "[GRAPH_DIAMETER] Single node graph - diameter is 0" << std::endl;
            result.diameter = 0.0;
            result.sourceNode = nodeIds[0];
            result.targetNode = nodeIds[0];
            result.longestPath = {nodeIds[0]};
            return result;
        }
        
        // Calculate shortest paths from each node to all other nodes
        for (int sourceNode : nodeIds) {
            auto distances = calculateShortestDistancesFrom(sourceNode);
            result.nodesProcessed++;
            
            for (const auto& [targetNode, distance] : distances) {
                if (distance != std::numeric_limits<double>::infinity() && distance > result.diameter) {
                    result.diameter = distance;
                    result.sourceNode = sourceNode;
                    result.targetNode = targetNode;
                    result.longestPath = reconstructPath(sourceNode, targetNode, distances);
                }
                result.pathsEvaluated++;
            }
            
            // Progress reporting for large graphs
            if (result.nodesProcessed % 100 == 0) {
                std::cout << "[GRAPH_DIAMETER] Processed " << result.nodesProcessed 
                          << "/" << nodeIds.size() << " nodes" << std::endl;
            }
        }
        
        std::cout << "[GRAPH_DIAMETER] Exact calculation completed - processed " 
                  << result.nodesProcessed << " nodes, evaluated " 
                  << result.pathsEvaluated << " paths" << std::endl;
        
        return result;
    }
    
    DiameterResult calculateApproximateDiameter() {
        std::cout << "[GRAPH_DIAMETER] Executing approximate diameter calculation using sampling" << std::endl;
        
        DiameterResult result;
        result.diameter = 0.0;
        result.sourceNode = -1;
        result.targetNode = -1;
        result.calculationMethod = "approximate_sampling";
        result.nodesProcessed = 0;
        result.pathsEvaluated = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.size() <= 1) {
            return calculateExactDiameter(); // Fall back to exact for small graphs
        }
        
        // Sample a subset of nodes for approximate calculation
        size_t sampleSize = std::min(static_cast<size_t>(std::sqrt(nodeIds.size()) * 10), nodeIds.size());
        std::vector<int> sampledNodes = sampleNodes(nodeIds, sampleSize);
        
        std::cout << "[GRAPH_DIAMETER] Sampling " << sampledNodes.size() 
                  << " nodes from " << nodeIds.size() << " total nodes" << std::endl;
        
        // Calculate diameter using sampled nodes
        for (int sourceNode : sampledNodes) {
            auto distances = calculateShortestDistancesFrom(sourceNode);
            result.nodesProcessed++;
            
            for (const auto& [targetNode, distance] : distances) {
                if (distance != std::numeric_limits<double>::infinity() && distance > result.diameter) {
                    result.diameter = distance;
                    result.sourceNode = sourceNode;
                    result.targetNode = targetNode;
                    result.longestPath = reconstructPath(sourceNode, targetNode, distances);
                }
                result.pathsEvaluated++;
            }
        }
        
        // Apply approximation factor based on sampling ratio
        double samplingRatio = static_cast<double>(sampleSize) / nodeIds.size();
        double approximationFactor = 1.0 / std::sqrt(samplingRatio);
        result.diameter *= approximationFactor;
        
        std::cout << "[GRAPH_DIAMETER] Approximate calculation completed - sampled " 
                  << result.nodesProcessed << " nodes, approximation factor: " 
                  << approximationFactor << std::endl;
        
        return result;
    }
    
    double getGraphRadius() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "[GRAPH_DIAMETER] Calculating graph radius (minimum eccentricity)" << std::endl;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            return 0.0;
        }
        
        double radius = std::numeric_limits<double>::infinity();
        
        for (int node : nodeIds) {
            double eccentricity = calculateNodeEccentricity(node);
            if (eccentricity < radius) {
                radius = eccentricity;
            }
        }
        
        std::cout << "[GRAPH_DIAMETER] Graph radius calculated: " << radius << std::endl;
        return radius;
    }
    
    double calculateNodeEccentricity(int nodeId) {
        if (!graph->hasNode(nodeId)) {
            std::cout << "[GRAPH_DIAMETER] Cannot calculate eccentricity - node not found: " << nodeId << std::endl;
            return std::numeric_limits<double>::infinity();
        }
        
        auto distances = calculateShortestDistancesFrom(nodeId);
        double maxDistance = 0.0;
        
        for (const auto& [targetNode, distance] : distances) {
            if (distance != std::numeric_limits<double>::infinity() && distance > maxDistance) {
                maxDistance = distance;
            }
        }
        
        return maxDistance;
    }
    
    std::vector<int> findPeripheralNodes() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "[GRAPH_DIAMETER] Finding peripheral nodes (nodes with maximum eccentricity)" << std::endl;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        std::vector<int> peripheralNodes;
        
        if (nodeIds.empty()) {
            return peripheralNodes;
        }
        
        DiameterResult diameterResult = calculateDiameter();
        double targetEccentricity = diameterResult.diameter;
        
        for (int nodeId : nodeIds) {
            double eccentricity = calculateNodeEccentricity(nodeId);
            if (std::abs(eccentricity - targetEccentricity) < 0.001) { // Tolerance for floating point comparison
                peripheralNodes.push_back(nodeId);
            }
        }
        
        std::cout << "[GRAPH_DIAMETER] Found " << peripheralNodes.size() << " peripheral nodes" << std::endl;
        return peripheralNodes;
    }
    
    std::vector<int> findCentralNodes() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "[GRAPH_DIAMETER] Finding central nodes (nodes with minimum eccentricity)" << std::endl;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        std::vector<int> centralNodes;
        
        if (nodeIds.empty()) {
            return centralNodes;
        }
        
        double radius = getGraphRadius();
        
        for (int nodeId : nodeIds) {
            double eccentricity = calculateNodeEccentricity(nodeId);
            if (std::abs(eccentricity - radius) < 0.001) { // Tolerance for floating point comparison
                centralNodes.push_back(nodeId);
            }
        }
        
        std::cout << "[GRAPH_DIAMETER] Found " << centralNodes.size() << " central nodes" << std::endl;
        return centralNodes;
    }
    
    void configureCalculation(bool enableCache, double cacheValiditySeconds, 
                            bool useApproximate, double accuracy, size_t maxExactNodes) {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        cacheEnabled = enableCache;
        cacheValidityDuration = cacheValiditySeconds;
        approximateMode = useApproximate;
        approximationAccuracy = accuracy;
        maxNodesForExactCalculation = maxExactNodes;
        
        std::cout << "[GRAPH_DIAMETER] Calculation parameters configured:" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Approximate mode: " << (approximateMode ? "Yes" : "No") << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Approximation accuracy: " << approximationAccuracy << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Max nodes for exact calculation: " << maxNodesForExactCalculation << std::endl;
    }
    
    void setResultCallback(std::function<void(const DiameterResult&)> callback) {
        resultCallback = callback;
        std::cout << "[GRAPH_DIAMETER] Result callback configured" << std::endl;
    }
    
    void invalidateCache() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        invalidateCachedResult();
        std::cout << "[GRAPH_DIAMETER] Diameter calculation cache invalidated" << std::endl;
    }
    
    CalculationMetrics getCalculationMetrics() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        return metrics;
    }
    
    void generateDiameterReport() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "\n[GRAPH_DIAMETER] === GRAPH DIAMETER ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[GRAPH_DIAMETER] System Configuration:" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Approximate mode: " << (approximateMode ? "Yes" : "No") << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Approximation accuracy: " << approximationAccuracy << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Max nodes for exact calculation: " << maxNodesForExactCalculation << std::endl;
        
        std::cout << "[GRAPH_DIAMETER] Graph Properties:" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Total edges: " << graph->getEdgeCount() << std::endl;
        
        if (cachedResult.isValid) {
            std::cout << "[GRAPH_DIAMETER] Last Calculation Results:" << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Diameter: " << cachedResult.diameter << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Source node: " << cachedResult.sourceNode << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Target node: " << cachedResult.targetNode << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Longest path length: " << cachedResult.longestPath.size() << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Calculation method: " << cachedResult.calculationMethod << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Calculation time: " << cachedResult.calculationTime << " seconds" << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Nodes processed: " << cachedResult.nodesProcessed << std::endl;
            std::cout << "[GRAPH_DIAMETER]   Paths evaluated: " << cachedResult.pathsEvaluated << std::endl;
        }
        
        std::cout << "[GRAPH_DIAMETER] Performance Metrics:" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Total calculations: " << metrics.totalCalculations << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Exact calculations: " << metrics.exactCalculations << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Approximate calculations: " << metrics.approximateCalculations << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache hits: " << metrics.cacheHits << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Cache misses: " << metrics.cacheMisses << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Average calculation time: " << metrics.averageCalculationTime << " seconds" << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Average nodes processed: " << metrics.averageNodesProcessed << std::endl;
        std::cout << "[GRAPH_DIAMETER]   Average paths evaluated: " << metrics.averagePathsEvaluated << std::endl;
        
        if (metrics.totalCalculations > 0) {
            double cacheHitRate = static_cast<double>(metrics.cacheHits) / 
                                 (metrics.cacheHits + metrics.cacheMisses) * 100.0;
            std::cout << "[GRAPH_DIAMETER]   Cache hit rate: " << cacheHitRate << "%" << std::endl;
            
            double approximateRatio = static_cast<double>(metrics.approximateCalculations) / 
                                    metrics.totalCalculations * 100.0;
            std::cout << "[GRAPH_DIAMETER]   Approximate calculation ratio: " << approximateRatio << "%" << std::endl;
        }
        
        std::cout << "[GRAPH_DIAMETER] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeCalculationMetrics() {
        metrics.totalCalculations = 0;
        metrics.averageCalculationTime = 0.0;
        metrics.lastDiameter = 0.0;
        metrics.cacheHits = 0;
        metrics.cacheMisses = 0;
        metrics.lastCalculation = std::chrono::steady_clock::now();
        metrics.averageNodesProcessed = 0.0;
        metrics.averagePathsEvaluated = 0.0;
        metrics.approximateCalculations = 0;
        metrics.exactCalculations = 0;
    }
    
    void invalidateCachedResult() {
        cachedResult.diameter = 0.0;
        cachedResult.sourceNode = -1;
        cachedResult.targetNode = -1;
        cachedResult.longestPath.clear();
        cachedResult.calculationTime = 0.0;
        cachedResult.calculationTimestamp = std::chrono::steady_clock::now() - std::chrono::hours(1);
        cachedResult.isValid = false;
        cachedResult.calculationMethod = "";
        cachedResult.nodesProcessed = 0;
        cachedResult.pathsEvaluated = 0;
    }
    
    bool isCachedResultValid() const {
        if (!cachedResult.isValid) {
            return false;
        }
        
        auto currentTime = std::chrono::steady_clock::now();
        auto timeSinceCalculation = std::chrono::duration<double>(
            currentTime - cachedResult.calculationTimestamp);
        
        return timeSinceCalculation.count() < cacheValidityDuration;
    }
    
    std::unordered_map<int, double> calculateShortestDistancesFrom(int sourceNode) {
        std::unordered_map<int, double> distances;
        std::unordered_map<int, int> predecessors;
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, 
                           std::greater<std::pair<double, int>>> pq;
        
        // Initialize distances
        for (int nodeId : graph->getAllNodeIds()) {
            distances[nodeId] = std::numeric_limits<double>::infinity();
        }
        distances[sourceNode] = 0.0;
        predecessors[sourceNode] = -1;
        
        pq.push({0.0, sourceNode});
        
        // Dijkstra's algorithm
        while (!pq.empty()) {
            auto [currentDistance, currentNode] = pq.top();
            pq.pop();
            
            if (currentDistance > distances[currentNode]) {
                continue;
            }
            
            const std::vector<Edge>& edges = graph->getEdgesFrom(currentNode);
            for (const Edge& edge : edges) {
                int neighbor = edge.getToNode();
                double newDistance = distances[currentNode] + edge.getWeight();
                
                if (newDistance < distances[neighbor]) {
                    distances[neighbor] = newDistance;
                    predecessors[neighbor] = currentNode;
                    pq.push({newDistance, neighbor});
                }
            }
        }
        
        return distances;
    }
    
    std::vector<int> reconstructPath(int sourceNode, int targetNode, 
                                   const std::unordered_map<int, double>& distances) {
        // For diameter calculation, we primarily need the distance, not the full path
        // This is a simplified path reconstruction
        std::vector<int> path;
        
        if (distances.find(targetNode) == distances.end() || 
            distances.at(targetNode) == std::numeric_limits<double>::infinity()) {
            return path;
        }
        
        // Return a simplified path representation
        path.push_back(sourceNode);
        path.push_back(targetNode);
        
        return path;
    }
    
    std::vector<int> sampleNodes(const std::vector<int>& allNodes, size_t sampleSize) {
        std::vector<int> sampledNodes;
        
        if (sampleSize >= allNodes.size()) {
            return allNodes;
        }
        
        // Simple sampling strategy - take every nth node plus some random selections
        size_t step = allNodes.size() / sampleSize;
        
        for (size_t i = 0; i < allNodes.size(); i += step) {
            if (sampledNodes.size() < sampleSize) {
                sampledNodes.push_back(allNodes[i]);
            }
        }
        
        // Fill remaining slots with random selections
        while (sampledNodes.size() < sampleSize) {
            int randomIndex = rand() % allNodes.size();
            if (std::find(sampledNodes.begin(), sampledNodes.end(), 
                         allNodes[randomIndex]) == sampledNodes.end()) {
                sampledNodes.push_back(allNodes[randomIndex]);
            }
        }
        
        return sampledNodes;
    }
    
    void updateCalculationMetrics(const DiameterResult& result) {
        metrics.totalCalculations++;
        metrics.lastDiameter = result.diameter;
        metrics.lastCalculation = result.calculationTimestamp;
        
        // Update averages
        if (metrics.averageCalculationTime == 0.0) {
            metrics.averageCalculationTime = result.calculationTime;
        } else {
            metrics.averageCalculationTime = (metrics.averageCalculationTime + result.calculationTime) / 2.0;
        }
        
        if (metrics.averageNodesProcessed == 0.0) {
            metrics.averageNodesProcessed = static_cast<double>(result.nodesProcessed);
        } else {
            metrics.averageNodesProcessed = (metrics.averageNodesProcessed + result.nodesProcessed) / 2.0;
        }
        
        if (metrics.averagePathsEvaluated == 0.0) {
            metrics.averagePathsEvaluated = static_cast<double>(result.pathsEvaluated);
        } else {
            metrics.averagePathsEvaluated = (metrics.averagePathsEvaluated + result.pathsEvaluated) / 2.0;
        }
    }
};