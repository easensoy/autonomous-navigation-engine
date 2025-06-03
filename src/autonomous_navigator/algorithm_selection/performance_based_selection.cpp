#include "autonomous_navigator/AlgorithmSelector.hpp"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cmath>

class PerformanceTracker {
private:
    struct PerformanceHistory {
        std::vector<AlgorithmPerformanceMetrics> metrics;
        double weightedAverage;
        size_t successCount;
        size_t failureCount;
        std::chrono::steady_clock::time_point lastUsed;
    };
    
    std::unordered_map<AlgorithmType, PerformanceHistory> algorithmHistory;
    std::string performancePriority;
    bool adaptiveLearningEnabled;
    double learningRate;
    size_t maxHistorySize;
    
public:
    PerformanceTracker() : performancePriority("balanced"), adaptiveLearningEnabled(true), 
                          learningRate(0.1), maxHistorySize(100) {}
    
    void updateMetrics(AlgorithmType algorithm, const AlgorithmPerformanceMetrics& metrics) {
        std::cout << "[PERFORMANCE_TRACKER] Updating metrics for algorithm " 
                  << static_cast<int>(algorithm) << std::endl;
        
        PerformanceHistory& history = algorithmHistory[algorithm];
        history.metrics.push_back(metrics);
        history.lastUsed = std::chrono::steady_clock::now();
        
        // Keep history size manageable
        if (history.metrics.size() > maxHistorySize) {
            history.metrics.erase(history.metrics.begin());
        }
        
        // Update success/failure counts
        if (metrics.pathLength > 0 && metrics.executionTime > 0) {
            history.successCount++;
        } else {
            history.failureCount++;
        }
        
        // Calculate weighted average performance score
        history.weightedAverage = calculateWeightedPerformanceScore(history.metrics);
        
        std::cout << "[PERFORMANCE_TRACKER] Algorithm performance updated:" << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Execution time: " << metrics.executionTime << "s" << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Nodes explored: " << metrics.nodesExplored << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Path length: " << metrics.pathLength << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Memory usage: " << metrics.memoryUsage << " bytes" << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Optimality: " << metrics.optimality << std::endl;
        std::cout << "[PERFORMANCE_TRACKER]   Weighted average: " << history.weightedAverage << std::endl;
    }
    
    AlgorithmType selectBestPerformingAlgorithm(int startId, int goalId) const {
        std::cout << "[PERFORMANCE_TRACKER] Selecting best performing algorithm for path " 
                  << startId << " -> " << goalId << std::endl;
        
        if (algorithmHistory.empty()) {
            std::cout << "[PERFORMANCE_TRACKER] No performance history available, defaulting to A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        AlgorithmType bestAlgorithm = AlgorithmType::ASTAR;
        double bestScore = -1.0;
        
        for (const auto& [algorithm, history] : algorithmHistory) {
            if (history.metrics.empty()) {
                continue;
            }
            
            double score = calculateAlgorithmScore(algorithm, history);
            
            std::cout << "[PERFORMANCE_TRACKER] Algorithm " << static_cast<int>(algorithm) 
                      << " score: " << score << std::endl;
            
            if (score > bestScore) {
                bestScore = score;
                bestAlgorithm = algorithm;
            }
        }
        
        std::cout << "[PERFORMANCE_TRACKER] Selected algorithm " << static_cast<int>(bestAlgorithm) 
                  << " with score " << bestScore << std::endl;
        
        return bestAlgorithm;
    }
    
    AlgorithmPerformanceMetrics getAlgorithmMetrics(AlgorithmType algorithm) const {
        auto it = algorithmHistory.find(algorithm);
        if (it == algorithmHistory.end() || it->second.metrics.empty()) {
            std::cout << "[PERFORMANCE_TRACKER] No metrics available for algorithm " 
                      << static_cast<int>(algorithm) << std::endl;
            return AlgorithmPerformanceMetrics{0.0, 0, 0.0, 0, 0.0};
        }
        
        return calculateAverageMetrics(it->second.metrics);
    }
    
    void setPerformancePriority(const std::string& priority) {
        performancePriority = priority;
        std::cout << "[PERFORMANCE_TRACKER] Performance priority set to: " << priority << std::endl;
    }
    
    void enableAdaptiveLearning(bool enable) {
        adaptiveLearningEnabled = enable;
        std::cout << "[PERFORMANCE_TRACKER] Adaptive learning " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void clearPerformanceHistory() {
        algorithmHistory.clear();
        std::cout << "[PERFORMANCE_TRACKER] Performance history cleared" << std::endl;
    }
    
    AlgorithmType getRecommendedAlgorithm() const {
        if (algorithmHistory.empty()) {
            return AlgorithmType::ASTAR;
        }
        
        // Find algorithm with best overall performance
        AlgorithmType recommended = AlgorithmType::ASTAR;
        double bestOverallScore = -1.0;
        
        for (const auto& [algorithm, history] : algorithmHistory) {
            if (history.metrics.empty()) {
                continue;
            }
            
            double reliabilityScore = calculateReliabilityScore(history);
            double performanceScore = history.weightedAverage;
            double recencyScore = calculateRecencyScore(history.lastUsed);
            
            double overallScore = (reliabilityScore * 0.4) + (performanceScore * 0.4) + (recencyScore * 0.2);
            
            if (overallScore > bestOverallScore) {
                bestOverallScore = overallScore;
                recommended = algorithm;
            }
        }
        
        std::cout << "[PERFORMANCE_TRACKER] Recommended algorithm: " << static_cast<int>(recommended) 
                  << " with overall score: " << bestOverallScore << std::endl;
        
        return recommended;
    }
    
    bool hasReliableMetrics(AlgorithmType algorithm) const {
        auto it = algorithmHistory.find(algorithm);
        if (it == algorithmHistory.end()) {
            return false;
        }
        
        const PerformanceHistory& history = it->second;
        return history.metrics.size() >= 3 && history.successCount > history.failureCount;
    }
    
    void benchmarkAlgorithms(const std::vector<AlgorithmType>& algorithms, int startId, int goalId) {
        std::cout << "[PERFORMANCE_TRACKER] Benchmarking " << algorithms.size() 
                  << " algorithms for path " << startId << " -> " << goalId << std::endl;
        
        for (AlgorithmType algorithm : algorithms) {
            std::cout << "[PERFORMANCE_TRACKER] Benchmarking algorithm " 
                      << static_cast<int>(algorithm) << std::endl;
            
            // Simulate algorithm execution and collect metrics
            AlgorithmPerformanceMetrics metrics = simulateAlgorithmExecution(algorithm, startId, goalId);
            updateMetrics(algorithm, metrics);
        }
    }
    
private:
    double calculateWeightedPerformanceScore(const std::vector<AlgorithmPerformanceMetrics>& metrics) const {
        if (metrics.empty()) {
            return 0.0;
        }
        
        double totalScore = 0.0;
        double totalWeight = 0.0;
        
        for (size_t i = 0; i < metrics.size(); ++i) {
            double weight = adaptiveLearningEnabled ? (1.0 + i * learningRate) : 1.0;
            double score = calculateMetricScore(metrics[i]);
            
            totalScore += score * weight;
            totalWeight += weight;
        }
        
        return totalWeight > 0 ? totalScore / totalWeight : 0.0;
    }
    
    double calculateMetricScore(const AlgorithmPerformanceMetrics& metrics) const {
        if (performancePriority == "speed") {
            return metrics.executionTime > 0 ? 1.0 / metrics.executionTime : 0.0;
        } else if (performancePriority == "memory") {
            return metrics.memoryUsage > 0 ? 1.0 / metrics.memoryUsage : 0.0;
        } else if (performancePriority == "optimality") {
            return metrics.optimality;
        } else if (performancePriority == "exploration") {
            return metrics.nodesExplored > 0 ? 1.0 / metrics.nodesExplored : 0.0;
        } else { // balanced
            double speedScore = metrics.executionTime > 0 ? 1.0 / metrics.executionTime : 0.0;
            double memoryScore = metrics.memoryUsage > 0 ? 1.0 / std::log(metrics.memoryUsage + 1) : 0.0;
            double optimalityScore = metrics.optimality;
            double explorationScore = metrics.nodesExplored > 0 ? 1.0 / std::log(metrics.nodesExplored + 1) : 0.0;
            
            return (speedScore * 0.3) + (memoryScore * 0.2) + (optimalityScore * 0.3) + (explorationScore * 0.2);
        }
    }
    
    double calculateAlgorithmScore(AlgorithmType algorithm, const PerformanceHistory& history) const {
        double performanceScore = history.weightedAverage;
        double reliabilityScore = calculateReliabilityScore(history);
        double recencyScore = calculateRecencyScore(history.lastUsed);
        
        // Penalize algorithms with insufficient data
        double dataConfidenceScore = std::min(1.0, static_cast<double>(history.metrics.size()) / 5.0);
        
        return performanceScore * reliabilityScore * recencyScore * dataConfidenceScore;
    }
    
    double calculateReliabilityScore(const PerformanceHistory& history) const {
        size_t totalAttempts = history.successCount + history.failureCount;
        if (totalAttempts == 0) {
            return 0.0;
        }
        
        double successRate = static_cast<double>(history.successCount) / totalAttempts;
        return successRate;
    }
    
    double calculateRecencyScore(const std::chrono::steady_clock::time_point& lastUsed) const {
        auto now = std::chrono::steady_clock::now();
        auto timeSinceLastUse = std::chrono::duration_cast<std::chrono::hours>(now - lastUsed).count();
        
        // Decay factor: more recent usage gets higher score
        return std::exp(-timeSinceLastUse / 168.0); // 168 hours = 1 week half-life
    }
    
    AlgorithmPerformanceMetrics calculateAverageMetrics(const std::vector<AlgorithmPerformanceMetrics>& metrics) const {
        if (metrics.empty()) {
            return AlgorithmPerformanceMetrics{0.0, 0, 0.0, 0, 0.0};
        }
        
        AlgorithmPerformanceMetrics average{0.0, 0, 0.0, 0, 0.0};
        
        for (const auto& metric : metrics) {
            average.executionTime += metric.executionTime;
            average.nodesExplored += metric.nodesExplored;
            average.pathLength += metric.pathLength;
            average.memoryUsage += metric.memoryUsage;
            average.optimality += metric.optimality;
        }
        
        size_t count = metrics.size();
        average.executionTime /= count;
        average.nodesExplored /= count;
        average.pathLength /= count;
        average.memoryUsage /= count;
        average.optimality /= count;
        
        return average;
    }
    
    AlgorithmPerformanceMetrics simulateAlgorithmExecution(AlgorithmType algorithm, int startId, int goalId) const {
        // This would be replaced with actual algorithm execution in production
        AlgorithmPerformanceMetrics metrics;
        
        // Simulate different performance characteristics for different algorithms
        switch (algorithm) {
            case AlgorithmType::ASTAR:
                metrics.executionTime = 0.05 + (rand() % 20) / 1000.0;
                metrics.nodesExplored = 50 + rand() % 100;
                metrics.pathLength = 10.0 + (rand() % 50) / 10.0;
                metrics.memoryUsage = 1024 + rand() % 2048;
                metrics.optimality = 0.85 + (rand() % 15) / 100.0;
                break;
                
            case AlgorithmType::DIJKSTRA:
                metrics.executionTime = 0.08 + (rand() % 30) / 1000.0;
                metrics.nodesExplored = 80 + rand() % 150;
                metrics.pathLength = 10.0 + (rand() % 40) / 10.0;
                metrics.memoryUsage = 2048 + rand() % 3072;
                metrics.optimality = 0.95 + (rand() % 5) / 100.0;
                break;
                
            case AlgorithmType::BFS:
                metrics.executionTime = 0.03 + (rand() % 15) / 1000.0;
                metrics.nodesExplored = 100 + rand() % 200;
                metrics.pathLength = 12.0 + (rand() % 60) / 10.0;
                metrics.memoryUsage = 512 + rand() % 1024;
                metrics.optimality = 0.70 + (rand() % 20) / 100.0;
                break;
                
            default:
                metrics.executionTime = 0.06 + (rand() % 25) / 1000.0;
                metrics.nodesExplored = 60 + rand() % 120;
                metrics.pathLength = 11.0 + (rand() % 55) / 10.0;
                metrics.memoryUsage = 1536 + rand() % 2560;
                metrics.optimality = 0.80 + (rand() % 15) / 100.0;
                break;
        }
        
        return metrics;
    }
};

// Global performance tracker instance
static PerformanceTracker g_performanceTracker;

// Implementation of performance-based selection methods for AlgorithmSelector
AlgorithmType AlgorithmSelector::selectBasedOnPerformance(int startId, int goalId) const {
    std::cout << "[ALGORITHM_SELECTOR] Selecting algorithm based on performance history" << std::endl;
    return g_performanceTracker.selectBestPerformingAlgorithm(startId, goalId);
}

void AlgorithmSelector::updatePerformanceMetrics(AlgorithmType algorithm, const AlgorithmPerformanceMetrics& metrics) {
    std::cout << "[ALGORITHM_SELECTOR] Updating performance metrics for algorithm" << std::endl;
    g_performanceTracker.updateMetrics(algorithm, metrics);
}

void AlgorithmSelector::setPerformancePriority(const std::string& priority) {
    std::cout << "[ALGORITHM_SELECTOR] Setting performance priority: " << priority << std::endl;
    g_performanceTracker.setPerformancePriority(priority);
}

AlgorithmPerformanceMetrics AlgorithmSelector::getAlgorithmMetrics(AlgorithmType algorithm) const {
    return g_performanceTracker.getAlgorithmMetrics(algorithm);
}

void AlgorithmSelector::clearPerformanceHistory() {
    std::cout << "[ALGORITHM_SELECTOR] Clearing performance history" << std::endl;
    g_performanceTracker.clearPerformanceHistory();
}

void AlgorithmSelector::enableAdaptiveLearning(bool enable) {
    std::cout << "[ALGORITHM_SELECTOR] " << (enable ? "Enabling" : "Disabling") 
              << " adaptive learning" << std::endl;
    g_performanceTracker.enableAdaptiveLearning(enable);
}

AlgorithmType AlgorithmSelector::getRecommendedAlgorithm() const {
    return g_performanceTracker.getRecommendedAlgorithm();
}

void AlgorithmSelector::benchmarkAlgorithms(int startId, int goalId) {
    std::cout << "[ALGORITHM_SELECTOR] Benchmarking algorithms for path " 
              << startId << " -> " << goalId << std::endl;
    
    std::vector<AlgorithmType> algorithmsToTest = {
        AlgorithmType::ASTAR,
        AlgorithmType::DIJKSTRA,
        AlgorithmType::BFS
    };
    
    // Add Bellman-Ford if the graph might have negative weights
    if (environment) {
        // Check for negative weights (simplified check)
        bool hasNegativeWeights = false;
        for (int nodeId : environment->getAllNodeIds()) {
            const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                if (edge.getWeight() < 0) {
                    hasNegativeWeights = true;
                    break;
                }
            }
            if (hasNegativeWeights) break;
        }
        
        if (hasNegativeWeights) {
            algorithmsToTest.push_back(AlgorithmType::BELLMAN_FORD);
        }
    }
    
    g_performanceTracker.benchmarkAlgorithms(algorithmsToTest, startId, goalId);
}