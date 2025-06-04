#include "core/Graph.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <functional>

class NodeClusteringAnalyzer {
private:
    const Graph* graph;
    mutable std::mutex clusteringMutex;
    
    struct ClusteringResult {
        std::vector<std::vector<int>> clusters;
        size_t clusterCount;
        std::vector<double> clusterSizes;
        double averageClusterSize;
        double largestClusterSize;
        double smallestClusterSize;
        std::chrono::steady_clock::time_point calculationTimestamp;
        double calculationTime;
        std::string algorithm;
        bool isValid;
        size_t nodesProcessed;
        double modularity;
        double silhouetteScore;
        std::unordered_map<int, int> nodeToCluster;
    };
    
    struct ClusteringMetrics {
        size_t totalClusterings;
        size_t distanceClusterings;
        size_t connectivityClusterings;
        size_t modularityClusterings;
        double averageCalculationTime;
        double averageClusterCount;
        double averageModularity;
        std::chrono::steady_clock::time_point lastClustering;
        size_t cacheHits;
        size_t cacheMisses;
        double averageNodesProcessed;
        double bestModularityAchieved;
    };
    
    ClusteringResult cachedResult;
    ClusteringMetrics metrics;
    bool cacheEnabled;
    double cacheValidityDuration;
    std::string defaultAlgorithm;
    double distanceThreshold;
    int minClusterSize;
    std::function<void(const ClusteringResult&)> resultCallback;
    
public:
    explicit NodeClusteringAnalyzer(const Graph* environment) 
        : graph(environment), cacheEnabled(true), cacheValidityDuration(300.0),
          defaultAlgorithm("distance_based"), distanceThreshold(10.0), minClusterSize(2) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeClusteringMetrics();
        invalidateCachedResult();
        
        std::cout << "[NODE_CLUSTERING] Node clustering analysis system initialized" << std::endl;
    }
    
    ClusteringResult clusterNodes() {
        std::lock_guard<std::mutex> lock(clusteringMutex);
        
        auto calculationStart = std::chrono::steady_clock::now();
        std::cout << "[NODE_CLUSTERING] Initiating node clustering analysis" << std::endl;
        
        // Check cache validity
        if (cacheEnabled && isCachedResultValid()) {
            std::cout << "[NODE_CLUSTERING] Using cached clustering result" << std::endl;
            metrics.cacheHits++;
            return cachedResult;
        }
        
        metrics.cacheMisses++;
        
        ClusteringResult result;
        if (defaultAlgorithm == "distance_based") {
            std::cout << "[NODE_CLUSTERING] Using distance-based clustering" << std::endl;
            result = clusterByDistance();
            metrics.distanceClusterings++;
        } else if (defaultAlgorithm == "connectivity") {
            std::cout << "[NODE_CLUSTERING] Using connectivity-based clustering" << std::endl;
            result = clusterByConnectivity();
            metrics.connectivityClusterings++;
        } else if (defaultAlgorithm == "modularity") {
            std::cout << "[NODE_CLUSTERING] Using modularity-based clustering" << std::endl;
            result = clusterByModularity();
            metrics.modularityClusterings++;
        } else {
            std::cout << "[NODE_CLUSTERING] Auto-selecting clustering algorithm" << std::endl;
            result = selectOptimalClustering();
        }
        
        auto calculationEnd = std::chrono::steady_clock::now();
        result.calculationTime = std::chrono::duration<double>(calculationEnd - calculationStart).count();
        result.calculationTimestamp = calculationEnd;
        result.isValid = true;
        
        // Calculate additional metrics
        calculateClusteringQuality(result);
        
        // Update metrics
        updateClusteringMetrics(result);
        
        // Cache the result
        if (cacheEnabled) {
            cachedResult = result;
        }
        
        // Trigger callback if configured
        if (resultCallback) {
            resultCallback(result);
        }
        
        std::cout << "[NODE_CLUSTERING] Clustering completed: " << result.clusterCount 
                  << " clusters found (algorithm: " << result.algorithm 
                  << ", time: " << result.calculationTime << "s)" << std::endl;
        
        return result;
    }
    
    ClusteringResult clusterByDistance() {
        std::cout << "[NODE_CLUSTERING] Executing distance-based clustering" << std::endl;
        
        ClusteringResult result;
        result.algorithm = "distance_based";
        result.clusterCount = 0;
        result.nodesProcessed = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[NODE_CLUSTERING] No nodes to cluster" << std::endl;
            result.isValid = false;
            return result;
        }
        
        std::unordered_set<int> unprocessed(nodeIds.begin(), nodeIds.end());
        
        while (!unprocessed.empty()) {
            std::vector<int> cluster;
            std::queue<int> toProcess;
            
            int startNode = *unprocessed.begin();
            toProcess.push(startNode);
            unprocessed.erase(startNode);
            
            while (!toProcess.empty()) {
                int currentNode = toProcess.front();
                toProcess.pop();
                cluster.push_back(currentNode);
                result.nodesProcessed++;
                
                const Node& current = graph->getNode(currentNode);
                
                // Find nearby nodes within distance threshold
                auto it = unprocessed.begin();
                while (it != unprocessed.end()) {
                    const Node& candidate = graph->getNode(*it);
                    double distance = current.euclideanDistance(candidate);
                    
                    if (distance <= distanceThreshold) {
                        toProcess.push(*it);
                        it = unprocessed.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            
            if (static_cast<int>(cluster.size()) >= minClusterSize) {
                result.clusters.push_back(cluster);
                result.clusterCount++;
                
                std::cout << "[NODE_CLUSTERING] Created cluster " << result.clusterCount 
                          << " with " << cluster.size() << " nodes" << std::endl;
            } else {
                std::cout << "[NODE_CLUSTERING] Discarded small cluster with " 
                          << cluster.size() << " nodes" << std::endl;
            }
        }
        
        calculateClusterSizes(result);
        buildNodeToClusterMapping(result);
        
        std::cout << "[NODE_CLUSTERING] Distance-based clustering completed - " 
                  << result.clusterCount << " clusters" << std::endl;
        
        return result;
    }
    
    ClusteringResult clusterByConnectivity() {
        std::cout << "[NODE_CLUSTERING] Executing connectivity-based clustering" << std::endl;
        
        ClusteringResult result;
        result.algorithm = "connectivity";
        result.clusterCount = 0;
        result.nodesProcessed = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            result.isValid = false;
            return result;
        }
        
        std::unordered_set<int> visited;
        
        for (int nodeId : nodeIds) {
            if (visited.find(nodeId) == visited.end()) {
                std::vector<int> cluster;
                std::queue<int> toVisit;
                
                toVisit.push(nodeId);
                visited.insert(nodeId);
                
                while (!toVisit.empty()) {
                    int current = toVisit.front();
                    toVisit.pop();
                    cluster.push_back(current);
                    result.nodesProcessed++;
                    
                    // Add all connected neighbors
                    std::vector<int> neighbors = graph->getNeighbors(current);
                    for (int neighbor : neighbors) {
                        if (visited.find(neighbor) == visited.end()) {
                            visited.insert(neighbor);
                            toVisit.push(neighbor);
                        }
                    }
                }
                
                if (static_cast<int>(cluster.size()) >= minClusterSize) {
                    result.clusters.push_back(cluster);
                    result.clusterCount++;
                    
                    std::cout << "[NODE_CLUSTERING] Created connected component cluster " 
                              << result.clusterCount << " with " << cluster.size() << " nodes" << std::endl;
                }
            }
        }
        
        calculateClusterSizes(result);
        buildNodeToClusterMapping(result);
        
        std::cout << "[NODE_CLUSTERING] Connectivity-based clustering completed - " 
                  << result.clusterCount << " clusters" << std::endl;
        
        return result;
    }
    
    ClusteringResult clusterByModularity() {
        std::cout << "[NODE_CLUSTERING] Executing modularity-based clustering (Louvain method)" << std::endl;
        
        ClusteringResult result;
        result.algorithm = "modularity_louvain";
        result.clusterCount = 0;
        result.nodesProcessed = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            result.isValid = false;
            return result;
        }
        
        // Initialize: each node in its own cluster
        std::unordered_map<int, int> nodeToCluster;
        for (size_t i = 0; i < nodeIds.size(); ++i) {
            nodeToCluster[nodeIds[i]] = static_cast<int>(i);
        }
        
        bool improved = true;
        int iteration = 0;
        double bestModularity = -1.0;
        
        while (improved && iteration < 100) {
            improved = false;
            iteration++;
            
            std::cout << "[NODE_CLUSTERING] Louvain iteration " << iteration << std::endl;
            
            for (int nodeId : nodeIds) {
                int originalCluster = nodeToCluster[nodeId];
                double bestGain = 0.0;
                int bestCluster = originalCluster;
                
                // Try moving node to neighbor clusters
                std::vector<int> neighbors = graph->getNeighbors(nodeId);
                std::unordered_set<int> neighborClusters;
                
                for (int neighbor : neighbors) {
                    neighborClusters.insert(nodeToCluster[neighbor]);
                }
                
                for (int candidateCluster : neighborClusters) {
                    if (candidateCluster != originalCluster) {
                        double gain = calculateModularityGain(nodeId, originalCluster, 
                                                             candidateCluster, nodeToCluster);
                        if (gain > bestGain) {
                            bestGain = gain;
                            bestCluster = candidateCluster;
                        }
                    }
                }
                
                if (bestCluster != originalCluster) {
                    nodeToCluster[nodeId] = bestCluster;
                    improved = true;
                    result.nodesProcessed++;
                }
            }
            
            double currentModularity = calculateModularity(nodeToCluster);
            if (currentModularity > bestModularity) {
                bestModularity = currentModularity;
            }
            
            std::cout << "[NODE_CLUSTERING] Iteration " << iteration 
                      << " modularity: " << currentModularity << std::endl;
        }
        
        // Convert cluster mapping to cluster groups
        std::unordered_map<int, std::vector<int>> clusterGroups;
        for (const auto& [nodeId, clusterId] : nodeToCluster) {
            clusterGroups[clusterId].push_back(nodeId);
        }
        
        for (const auto& [clusterId, nodes] : clusterGroups) {
            if (static_cast<int>(nodes.size()) >= minClusterSize) {
                result.clusters.push_back(nodes);
                result.clusterCount++;
            }
        }
        
        result.modularity = bestModularity;
        result.nodeToCluster = nodeToCluster;
        
        calculateClusterSizes(result);
        
        std::cout << "[NODE_CLUSTERING] Modularity-based clustering completed - " 
                  << result.clusterCount << " clusters, modularity: " << bestModularity << std::endl;
        
        return result;
    }
    
    std::vector<int> getClusterForNode(int nodeId) {
        ClusteringResult result = clusterNodes();
        
        for (const std::vector<int>& cluster : result.clusters) {
            if (std::find(cluster.begin(), cluster.end(), nodeId) != cluster.end()) {
                return cluster;
            }
        }
        
        return {};
    }
    
    std::vector<int> getLargestCluster() {
        ClusteringResult result = clusterNodes();
        
        if (result.clusters.empty()) {
            return {};
        }
        
        auto largestCluster = std::max_element(result.clusters.begin(), result.clusters.end(),
            [](const std::vector<int>& a, const std::vector<int>& b) {
                return a.size() < b.size();
            });
        
        return *largestCluster;
    }
    
    void configureClusterng(const std::string& algorithm, double threshold, 
                          int minSize, bool enableCache, double cacheValiditySeconds) {
        std::lock_guard<std::mutex> lock(clusteringMutex);
        
        defaultAlgorithm = algorithm;
        distanceThreshold = threshold;
        minClusterSize = minSize;
        cacheEnabled = enableCache;
        cacheValidityDuration = cacheValiditySeconds;
        
        std::cout << "[NODE_CLUSTERING] Configuration updated:" << std::endl;
        std::cout << "[NODE_CLUSTERING]   Algorithm: " << defaultAlgorithm << std::endl;
        std::cout << "[NODE_CLUSTERING]   Distance threshold: " << distanceThreshold << std::endl;
        std::cout << "[NODE_CLUSTERING]   Minimum cluster size: " << minClusterSize << std::endl;
        std::cout << "[NODE_CLUSTERING]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
    }
    
    void setResultCallback(std::function<void(const ClusteringResult&)> callback) {
        resultCallback = callback;
        std::cout << "[NODE_CLUSTERING] Result callback configured" << std::endl;
    }
    
    void invalidateCache() {
        std::lock_guard<std::mutex> lock(clusteringMutex);
        invalidateCachedResult();
        std::cout << "[NODE_CLUSTERING] Clustering cache invalidated" << std::endl;
    }
    
    ClusteringMetrics getClusteringMetrics() const {
        std::lock_guard<std::mutex> lock(clusteringMutex);
        return metrics;
    }
    
    void generateClusteringReport() const {
        std::lock_guard<std::mutex> lock(clusteringMutex);
        
        std::cout << "\n[NODE_CLUSTERING] === NODE CLUSTERING ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[NODE_CLUSTERING] System Configuration:" << std::endl;
        std::cout << "[NODE_CLUSTERING]   Default algorithm: " << defaultAlgorithm << std::endl;
        std::cout << "[NODE_CLUSTERING]   Distance threshold: " << distanceThreshold << std::endl;
        std::cout << "[NODE_CLUSTERING]   Minimum cluster size: " << minClusterSize << std::endl;
        std::cout << "[NODE_CLUSTERING]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        
        std::cout << "[NODE_CLUSTERING] Graph Properties:" << std::endl;
        std::cout << "[NODE_CLUSTERING]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[NODE_CLUSTERING]   Total edges: " << graph->getEdgeCount() << std::endl;
        
        if (cachedResult.isValid) {
            std::cout << "[NODE_CLUSTERING] Last Clustering Results:" << std::endl;
            std::cout << "[NODE_CLUSTERING]   Cluster count: " << cachedResult.clusterCount << std::endl;
            std::cout << "[NODE_CLUSTERING]   Average cluster size: " << cachedResult.averageClusterSize << std::endl;
            std::cout << "[NODE_CLUSTERING]   Largest cluster size: " << cachedResult.largestClusterSize << std::endl;
            std::cout << "[NODE_CLUSTERING]   Smallest cluster size: " << cachedResult.smallestClusterSize << std::endl;
            std::cout << "[NODE_CLUSTERING]   Algorithm used: " << cachedResult.algorithm << std::endl;
            std::cout << "[NODE_CLUSTERING]   Calculation time: " << cachedResult.calculationTime << " seconds" << std::endl;
            std::cout << "[NODE_CLUSTERING]   Modularity: " << cachedResult.modularity << std::endl;
            std::cout << "[NODE_CLUSTERING]   Silhouette score: " << cachedResult.silhouetteScore << std::endl;
        }
        
        std::cout << "[NODE_CLUSTERING] Performance Metrics:" << std::endl;
        std::cout << "[NODE_CLUSTERING]   Total clusterings: " << metrics.totalClusterings << std::endl;
        std::cout << "[NODE_CLUSTERING]   Distance-based clusterings: " << metrics.distanceClusterings << std::endl;
        std::cout << "[NODE_CLUSTERING]   Connectivity-based clusterings: " << metrics.connectivityClusterings << std::endl;
        std::cout << "[NODE_CLUSTERING]   Modularity-based clusterings: " << metrics.modularityClusterings << std::endl;
        std::cout << "[NODE_CLUSTERING]   Cache hits: " << metrics.cacheHits << std::endl;
        std::cout << "[NODE_CLUSTERING]   Cache misses: " << metrics.cacheMisses << std::endl;
        std::cout << "[NODE_CLUSTERING]   Average calculation time: " << metrics.averageCalculationTime << " seconds" << std::endl;
        std::cout << "[NODE_CLUSTERING]   Average cluster count: " << metrics.averageClusterCount << std::endl;
        std::cout << "[NODE_CLUSTERING]   Best modularity achieved: " << metrics.bestModularityAchieved << std::endl;
        
        std::cout << "[NODE_CLUSTERING] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeClusteringMetrics() {
        metrics.totalClusterings = 0;
        metrics.distanceClusterings = 0;
        metrics.connectivityClusterings = 0;
        metrics.modularityClusterings = 0;
        metrics.averageCalculationTime = 0.0;
        metrics.averageClusterCount = 0.0;
        metrics.averageModularity = 0.0;
        metrics.lastClustering = std::chrono::steady_clock::now();
        metrics.cacheHits = 0;
        metrics.cacheMisses = 0;
        metrics.averageNodesProcessed = 0.0;
        metrics.bestModularityAchieved = -1.0;
    }
    
    void invalidateCachedResult() {
        cachedResult.clusters.clear();
        cachedResult.clusterCount = 0;
        cachedResult.clusterSizes.clear();
        cachedResult.averageClusterSize = 0.0;
        cachedResult.largestClusterSize = 0.0;
        cachedResult.smallestClusterSize = 0.0;
        cachedResult.calculationTimestamp = std::chrono::steady_clock::now() - std::chrono::hours(1);
        cachedResult.calculationTime = 0.0;
        cachedResult.algorithm = "";
        cachedResult.isValid = false;
        cachedResult.nodesProcessed = 0;
        cachedResult.modularity = 0.0;
        cachedResult.silhouetteScore = 0.0;
        cachedResult.nodeToCluster.clear();
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
    
    ClusteringResult selectOptimalClustering() {
        // Try different algorithms and select the best one
        std::vector<ClusteringResult> results;
        
        results.push_back(clusterByDistance());
        results.push_back(clusterByConnectivity());
        results.push_back(clusterByModularity());
        
        // Select based on modularity or other quality metrics
        auto bestResult = std::max_element(results.begin(), results.end(),
            [](const ClusteringResult& a, const ClusteringResult& b) {
                return a.modularity < b.modularity;
            });
        
        bestResult->algorithm = "auto_selected_" + bestResult->algorithm;
        return *bestResult;
    }
    
    void calculateClusterSizes(ClusteringResult& result) {
        result.clusterSizes.clear();
        
        if (result.clusters.empty()) {
            result.averageClusterSize = 0.0;
            result.largestClusterSize = 0.0;
            result.smallestClusterSize = 0.0;
            return;
        }
        
        double totalSize = 0.0;
        result.largestClusterSize = result.clusters[0].size();
        result.smallestClusterSize = result.clusters[0].size();
        
        for (const std::vector<int>& cluster : result.clusters) {
            double size = static_cast<double>(cluster.size());
            result.clusterSizes.push_back(size);
            totalSize += size;
            
            result.largestClusterSize = std::max(result.largestClusterSize, size);
            result.smallestClusterSize = std::min(result.smallestClusterSize, size);
        }
        
        result.averageClusterSize = totalSize / result.clusters.size();
    }
    
    void buildNodeToClusterMapping(ClusteringResult& result) {
        result.nodeToCluster.clear();
        
        for (size_t i = 0; i < result.clusters.size(); ++i) {
            for (int nodeId : result.clusters[i]) {
                result.nodeToCluster[nodeId] = static_cast<int>(i);
            }
        }
    }
    
    void calculateClusteringQuality(ClusteringResult& result) {
        if (result.nodeToCluster.empty()) {
            buildNodeToClusterMapping(result);
        }
        
        // Calculate modularity if not already calculated
        if (result.modularity == 0.0 && result.algorithm != "modularity_louvain") {
            result.modularity = calculateModularity(result.nodeToCluster);
        }
        
        // Calculate silhouette score
        result.silhouetteScore = calculateSilhouetteScore(result);
    }
    
    double calculateModularity(const std::unordered_map<int, int>& nodeToCluster) {
        size_t totalEdges = graph->getEdgeCount();
        if (totalEdges == 0) return 0.0;
        
        double modularity = 0.0;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int i : nodeIds) {
            for (int j : nodeIds) {
                if (nodeToCluster.at(i) == nodeToCluster.at(j)) {
                    double aij = graph->hasEdge(i, j) ? 1.0 : 0.0;
                    double ki = graph->getNeighbors(i).size();
                    double kj = graph->getNeighbors(j).size();
                    double expected = (ki * kj) / (2.0 * totalEdges);
                    
                    modularity += aij - expected;
                }
            }
        }
        
        return modularity / (2.0 * totalEdges);
    }
    
    double calculateModularityGain(int nodeId, int fromCluster, int toCluster,
                                 const std::unordered_map<int, int>& nodeToCluster) {
        // Simplified modularity gain calculation
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        double gainFromRemoval = 0.0;
        double gainFromAddition = 0.0;
        
        for (int neighbor : neighbors) {
            auto it = nodeToCluster.find(neighbor);
            if (it != nodeToCluster.end()) {
                if (it->second == fromCluster) {
                    gainFromRemoval -= 1.0;
                }
                if (it->second == toCluster) {
                    gainFromAddition += 1.0;
                }
            }
        }
        
        return gainFromAddition - gainFromRemoval;
    }
    
    double calculateSilhouetteScore(const ClusteringResult& result) {
        if (result.clusters.size() <= 1) return 0.0;
        
        double totalScore = 0.0;
        size_t totalNodes = 0;
        
        for (size_t clusterIdx = 0; clusterIdx < result.clusters.size(); ++clusterIdx) {
            const std::vector<int>& cluster = result.clusters[clusterIdx];
            
            for (int nodeId : cluster) {
                double a = calculateIntraClusterDistance(nodeId, cluster);
                double b = calculateNearestClusterDistance(nodeId, result.clusters, clusterIdx);
                
                double silhouette = (b - a) / std::max(a, b);
                totalScore += silhouette;
                totalNodes++;
            }
        }
        
        return totalNodes > 0 ? totalScore / totalNodes : 0.0;
    }
    
    double calculateIntraClusterDistance(int nodeId, const std::vector<int>& cluster) {
        if (cluster.size() <= 1) return 0.0;
        
        const Node& node = graph->getNode(nodeId);
        double totalDistance = 0.0;
        size_t count = 0;
        
        for (int otherId : cluster) {
            if (otherId != nodeId) {
                const Node& other = graph->getNode(otherId);
                totalDistance += node.euclideanDistance(other);
                count++;
            }
        }
        
        return count > 0 ? totalDistance / count : 0.0;
    }
    
    double calculateNearestClusterDistance(int nodeId, const std::vector<std::vector<int>>& clusters,
                                         size_t excludeClusterIdx) {
        const Node& node = graph->getNode(nodeId);
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (i == excludeClusterIdx) continue;
            
            double totalDistance = 0.0;
            for (int otherId : clusters[i]) {
                const Node& other = graph->getNode(otherId);
                totalDistance += node.euclideanDistance(other);
            }
            
            double avgDistance = totalDistance / clusters[i].size();
            minDistance = std::min(minDistance, avgDistance);
        }
        
        return minDistance;
    }
    
    void updateClusteringMetrics(const ClusteringResult& result) {
        metrics.totalClusterings++;
        metrics.lastClustering = result.calculationTimestamp;
        
        // Update averages
        if (metrics.averageCalculationTime == 0.0) {
            metrics.averageCalculationTime = result.calculationTime;
        } else {
            metrics.averageCalculationTime = (metrics.averageCalculationTime + result.calculationTime) / 2.0;
        }
        
        if (metrics.averageClusterCount == 0.0) {
            metrics.averageClusterCount = static_cast<double>(result.clusterCount);
        } else {
            metrics.averageClusterCount = (metrics.averageClusterCount + result.clusterCount) / 2.0;
        }
        
        if (metrics.averageModularity == 0.0) {
            metrics.averageModularity = result.modularity;
        } else {
            metrics.averageModularity = (metrics.averageModularity + result.modularity) / 2.0;
        }
        
        metrics.bestModularityAchieved = std::max(metrics.bestModularityAchieved, result.modularity);
        
        if (metrics.averageNodesProcessed == 0.0) {
            metrics.averageNodesProcessed = static_cast<double>(result.nodesProcessed);
        } else {
            metrics.averageNodesProcessed = (metrics.averageNodesProcessed + result.nodesProcessed) / 2.0;
        }
    }
};