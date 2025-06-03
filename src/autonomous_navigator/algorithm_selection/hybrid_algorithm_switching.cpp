#include "autonomous_navigator/AlgorithmSelector.hpp"
#include <iostream>
#include <chrono>
#include <queue>
#include <cmath>
#include <algorithm>

class HybridAlgorithmManager {
private:
    struct AlgorithmEvaluation {
        AlgorithmType algorithm;
        double environmentScore;
        double performanceScore;
        double combinedScore;
        double confidence;
        bool isReliable;
    };
    
    struct ExecutionContext {
        int startId;
        int goalId;
        std::chrono::steady_clock::time_point startTime;
        AlgorithmType currentAlgorithm;
        AlgorithmType backupAlgorithm;
        double maxExecutionTime;
        bool allowSwitching;
        size_t switchCount;
    };
    
    AlgorithmSelector* selector;
    ExecutionContext currentContext;
    std::vector<AlgorithmType> algorithmPriority;
    double environmentWeight;
    double performanceWeight;
    double switchingThreshold;
    bool dynamicSwitchingEnabled;
    
public:
    explicit HybridAlgorithmManager(AlgorithmSelector* algorithmSelector) 
        : selector(algorithmSelector), environmentWeight(0.6), performanceWeight(0.4),
          switchingThreshold(0.2), dynamicSwitchingEnabled(true) {
        
        // Default algorithm priority based on general effectiveness
        algorithmPriority = {
            AlgorithmType::ASTAR,
            AlgorithmType::DIJKSTRA,
            AlgorithmType::JUMP_POINT_SEARCH,
            AlgorithmType::BFS,
            AlgorithmType::BELLMAN_FORD
        };
    }
    
    AlgorithmType selectOptimalAlgorithmHybrid(int startId, int goalId) {
        std::cout << "[HYBRID_MANAGER] Selecting optimal algorithm using hybrid approach" << std::endl;
        
        currentContext.startId = startId;
        currentContext.goalId = goalId;
        currentContext.startTime = std::chrono::steady_clock::now();
        currentContext.switchCount = 0;
        currentContext.allowSwitching = dynamicSwitchingEnabled;
        
        std::vector<AlgorithmEvaluation> evaluations = evaluateAllAlgorithms(startId, goalId);
        
        if (evaluations.empty()) {
            std::cout << "[HYBRID_MANAGER] No algorithms evaluated, defaulting to A*" << std::endl;
            currentContext.currentAlgorithm = AlgorithmType::ASTAR;
            currentContext.backupAlgorithm = AlgorithmType::DIJKSTRA;
            return AlgorithmType::ASTAR;
        }
        
        // Sort algorithms by combined score
        std::sort(evaluations.begin(), evaluations.end(),
                 [](const AlgorithmEvaluation& a, const AlgorithmEvaluation& b) {
                     return a.combinedScore > b.combinedScore;
                 });
        
        // Select primary and backup algorithms
        currentContext.currentAlgorithm = evaluations[0].algorithm;
        currentContext.backupAlgorithm = evaluations.size() > 1 ? evaluations[1].algorithm : AlgorithmType::DIJKSTRA;
        
        std::cout << "[HYBRID_MANAGER] Selected algorithm: " << static_cast<int>(currentContext.currentAlgorithm)
                  << " (score: " << evaluations[0].combinedScore << ")" << std::endl;
        std::cout << "[HYBRID_MANAGER] Backup algorithm: " << static_cast<int>(currentContext.backupAlgorithm) << std::endl;
        
        return currentContext.currentAlgorithm;
    }
    
    AlgorithmType checkForSwitching(double currentExecutionTime, size_t nodesExplored, bool hasFoundPath) {
        if (!currentContext.allowSwitching || currentContext.switchCount >= 2) {
            return currentContext.currentAlgorithm; // Prevent infinite switching
        }
        
        std::cout << "[HYBRID_MANAGER] Checking for algorithm switching..." << std::endl;
        std::cout << "[HYBRID_MANAGER] Current execution time: " << currentExecutionTime << "s" << std::endl;
        std::cout << "[HYBRID_MANAGER] Nodes explored: " << nodesExplored << std::endl;
        
        // Check if current algorithm is performing poorly
        bool shouldSwitch = false;
        std::string switchReason;
        
        // Time-based switching
        if (currentContext.maxExecutionTime > 0 && currentExecutionTime > currentContext.maxExecutionTime * 0.8) {
            shouldSwitch = true;
            switchReason = "time limit approaching";
        }
        
        // Exploration efficiency switching
        auto elapsed = std::chrono::steady_clock::now() - currentContext.startTime;
        double elapsedSeconds = std::chrono::duration<double>(elapsed).count();
        
        if (elapsedSeconds > 0) {
            double explorationRate = nodesExplored / elapsedSeconds;
            
            // If exploration rate is too low, consider switching
            if (explorationRate < 50 && elapsedSeconds > 0.1) { // Less than 50 nodes/second after 100ms
                shouldSwitch = true;
                switchReason = "low exploration efficiency";
            }
        }
        
        // Performance degradation switching
        if (!hasFoundPath && elapsedSeconds > 0.5) { // No path found after 500ms
            AlgorithmPerformanceMetrics currentMetrics = selector->getAlgorithmMetrics(currentContext.currentAlgorithm);
            AlgorithmPerformanceMetrics backupMetrics = selector->getAlgorithmMetrics(currentContext.backupAlgorithm);
            
            if (backupMetrics.executionTime > 0 && 
                backupMetrics.executionTime < currentMetrics.executionTime * 0.7) {
                shouldSwitch = true;
                switchReason = "backup algorithm shows better historical performance";
            }
        }
        
        if (shouldSwitch) {
            std::cout << "[HYBRID_MANAGER] Switching algorithm due to: " << switchReason << std::endl;
            
            AlgorithmType newAlgorithm = selectFallbackAlgorithm(currentExecutionTime, nodesExplored);
            currentContext.currentAlgorithm = newAlgorithm;
            currentContext.switchCount++;
            
            std::cout << "[HYBRID_MANAGER] Switched to algorithm: " << static_cast<int>(newAlgorithm) << std::endl;
            return newAlgorithm;
        }
        
        return currentContext.currentAlgorithm;
    }
    
    AlgorithmType selectFallbackAlgorithm(double currentExecutionTime, size_t nodesExplored) {
        std::cout << "[HYBRID_MANAGER] Selecting fallback algorithm..." << std::endl;
        
        // Analyze current situation to select best fallback
        if (currentExecutionTime > 0.1 && nodesExplored < 100) {
            // Slow start, try a faster algorithm
            std::cout << "[HYBRID_MANAGER] Slow start detected, selecting fast algorithm" << std::endl;
            return AlgorithmType::BFS;
        }
        
        if (nodesExplored > 1000 && currentExecutionTime > 0.05) {
            // Too much exploration, try a more guided algorithm
            std::cout << "[HYBRID_MANAGER] Excessive exploration, selecting guided algorithm" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        // Use backup algorithm if available
        if (currentContext.backupAlgorithm != currentContext.currentAlgorithm) {
            std::cout << "[HYBRID_MANAGER] Using predetermined backup algorithm" << std::endl;
            return currentContext.backupAlgorithm;
        }
        
        // Select based on algorithm priority
        for (AlgorithmType algorithm : algorithmPriority) {
            if (algorithm != currentContext.currentAlgorithm) {
                std::cout << "[HYBRID_MANAGER] Selected from priority list: " << static_cast<int>(algorithm) << std::endl;
                return algorithm;
            }
        }
        
        std::cout << "[HYBRID_MANAGER] Fallback to Dijkstra as last resort" << std::endl;
        return AlgorithmType::DIJKSTRA;
    }
    
    void updateAlgorithmWeights(double envWeight, double perfWeight) {
        environmentWeight = envWeight;
        performanceWeight = perfWeight;
        
        // Normalize weights
        double total = environmentWeight + performanceWeight;
        if (total > 0) {
            environmentWeight /= total;
            performanceWeight /= total;
        }
        
        std::cout << "[HYBRID_MANAGER] Updated weights - Environment: " << environmentWeight 
                  << ", Performance: " << performanceWeight << std::endl;
    }
    
    void setDynamicSwitching(bool enabled, double threshold = 0.2) {
        dynamicSwitchingEnabled = enabled;
        switchingThreshold = threshold;
        
        std::cout << "[HYBRID_MANAGER] Dynamic switching " << (enabled ? "enabled" : "disabled")
                  << " with threshold: " << threshold << std::endl;
    }
    
    void setExecutionConstraints(double maxTime) {
        currentContext.maxExecutionTime = maxTime;
        std::cout << "[HYBRID_MANAGER] Set execution time constraint: " << maxTime << "s" << std::endl;
    }
    
    AlgorithmType adaptiveSelection(int startId, int goalId, const std::vector<double>& environmentFactors) {
        std::cout << "[HYBRID_MANAGER] Performing adaptive algorithm selection" << std::endl;
        
        // Use environmental factors to adjust selection
        double complexityFactor = environmentFactors.size() > 0 ? environmentFactors[0] : 1.0;
        double densityFactor = environmentFactors.size() > 1 ? environmentFactors[1] : 1.0;
        double connectivityFactor = environmentFactors.size() > 2 ? environmentFactors[2] : 1.0;
        
        std::cout << "[HYBRID_MANAGER] Environment factors - Complexity: " << complexityFactor
                  << ", Density: " << densityFactor << ", Connectivity: " << connectivityFactor << std::endl;
        
        // Adjust algorithm selection based on factors
        if (complexityFactor > 2.0 && densityFactor < 0.3) {
            std::cout << "[HYBRID_MANAGER] High complexity, low density - selecting A*" << std::endl;
            return AlgorithmType::ASTAR;
        }
        
        if (densityFactor > 0.7) {
            std::cout << "[HYBRID_MANAGER] High density - selecting Dijkstra" << std::endl;
            return AlgorithmType::DIJKSTRA;
        }
        
        if (connectivityFactor < 0.5) {
            std::cout << "[HYBRID_MANAGER] Low connectivity - selecting BFS for exploration" << std::endl;
            return AlgorithmType::BFS;
        }
        
        // Default to standard hybrid selection
        return selectOptimalAlgorithmHybrid(startId, goalId);
    }
    
    void recordExecutionResult(AlgorithmType algorithm, const AlgorithmPerformanceMetrics& metrics, bool successful) {
        std::cout << "[HYBRID_MANAGER] Recording execution result for algorithm " 
                  << static_cast<int>(algorithm) << std::endl;
        
        // Adjust weights based on success/failure
        if (successful) {
            std::cout << "[HYBRID_MANAGER] Successful execution recorded" << std::endl;
            
            // If the recommended algorithm worked well, increase performance weight slightly
            AlgorithmType recommended = selector->getRecommendedAlgorithm();
            if (algorithm == recommended) {
                performanceWeight = std::min(0.8, performanceWeight * 1.05);
                environmentWeight = 1.0 - performanceWeight;
            }
        } else {
            std::cout << "[HYBRID_MANAGER] Failed execution recorded" << std::endl;
            
            // If the algorithm failed, reduce confidence in performance-based selection
            environmentWeight = std::min(0.8, environmentWeight * 1.05);
            performanceWeight = 1.0 - environmentWeight;
        }
        
        std::cout << "[HYBRID_MANAGER] Adjusted weights - Environment: " << environmentWeight 
                  << ", Performance: " << performanceWeight << std::endl;
    }
    
private:
    std::vector<AlgorithmEvaluation> evaluateAllAlgorithms(int startId, int goalId) {
        std::vector<AlgorithmEvaluation> evaluations;
        
        for (AlgorithmType algorithm : algorithmPriority) {
            AlgorithmEvaluation eval;
            eval.algorithm = algorithm;
            
            // Get environment-based score using public methods
            eval.environmentScore = calculateEnvironmentScore(algorithm, startId, goalId);
            
            // Get performance-based score using public methods
            eval.performanceScore = calculatePerformanceScore(algorithm);
            
            // Calculate combined score
            eval.combinedScore = (eval.environmentScore * environmentWeight) + 
                               (eval.performanceScore * performanceWeight);
            
            // Determine reliability
            AlgorithmPerformanceMetrics metrics = selector->getAlgorithmMetrics(algorithm);
            eval.isReliable = metrics.executionTime > 0 && metrics.optimality > 0.5;
            
            // Calculate confidence based on data availability
            eval.confidence = eval.isReliable ? 0.8 : 0.3;
            
            evaluations.push_back(eval);
            
            std::cout << "[HYBRID_MANAGER] Algorithm " << static_cast<int>(algorithm) 
                      << " - Env: " << eval.environmentScore 
                      << ", Perf: " << eval.performanceScore
                      << ", Combined: " << eval.combinedScore
                      << ", Reliable: " << (eval.isReliable ? "Yes" : "No") << std::endl;
        }
        
        return evaluations;
    }
    
    double calculateEnvironmentScore(AlgorithmType algorithm, int startId, int goalId) {
        // Use public methods to determine environment-based score
        AlgorithmType sizeBasedRecommendation = selector->selectForEnvironmentSize(100, 200); // Use reasonable defaults
        AlgorithmType timeBasedRecommendation = selector->selectForRealTimeConstraints(0.1); // 100ms constraint
        AlgorithmType generalRecommendation = selector->getRecommendedAlgorithm();
        
        double score = 0.5; // Base score
        
        // Give higher scores to algorithms that match public method recommendations
        if (algorithm == sizeBasedRecommendation) {
            score += 0.2;
        }
        if (algorithm == timeBasedRecommendation) {
            score += 0.2;
        }
        if (algorithm == generalRecommendation) {
            score += 0.3;
        }
        
        // Give partial scores based on algorithm characteristics
        switch (algorithm) {
            case AlgorithmType::ASTAR:
                score += 0.3; // Generally good for most environments
                break;
            case AlgorithmType::DIJKSTRA:
                score += 0.2; // Good for dense graphs
                break;
            case AlgorithmType::BFS:
                score += 0.1; // Good for unweighted graphs
                break;
            case AlgorithmType::JUMP_POINT_SEARCH:
                score += 0.4; // Excellent for grid-like environments
                break;
            case AlgorithmType::BELLMAN_FORD:
                score += 0.1; // Specialized for negative weights
                break;
            default:
                score += 0.1;
                break;
        }
        
        return std::min(1.0, score); // Cap at 1.0
    }
    
    double calculatePerformanceScore(AlgorithmType algorithm) {
        AlgorithmPerformanceMetrics metrics = selector->getAlgorithmMetrics(algorithm);
        
        if (metrics.executionTime <= 0) {
            return 0.3; // Low score for algorithms without performance data
        }
        
        // Normalize metrics to 0-1 range
        double timeScore = std::min(1.0, 0.1 / metrics.executionTime); // Better for faster algorithms
        double optimalityScore = metrics.optimality;
        double efficiencyScore = metrics.nodesExplored > 0 ? std::min(1.0, 100.0 / metrics.nodesExplored) : 0.5;
        
        return (timeScore * 0.4) + (optimalityScore * 0.4) + (efficiencyScore * 0.2);
    }
};

// Global hybrid manager instance
static std::unique_ptr<HybridAlgorithmManager> g_hybridManager;

// Implementation of hybrid selection methods for AlgorithmSelector
AlgorithmType AlgorithmSelector::selectOptimalAlgorithm(int startId, int goalId) {
    std::cout << "[ALGORITHM_SELECTOR] Selecting optimal algorithm using hybrid approach" << std::endl;
    
    if (!g_hybridManager) {
        g_hybridManager = std::make_unique<HybridAlgorithmManager>(this);
    }
    
    return g_hybridManager->selectOptimalAlgorithmHybrid(startId, goalId);
}

std::vector<int> AlgorithmSelector::executeSelectedAlgorithm(AlgorithmType algorithm, int startId, int goalId) {
    std::cout << "[ALGORITHM_SELECTOR] Executing algorithm " << static_cast<int>(algorithm) 
              << " for path " << startId << " -> " << goalId << std::endl;
    
    auto startTime = std::chrono::steady_clock::now();
    std::vector<int> path;
    
    try {
        // This would execute the actual algorithm
        // For now, we'll simulate execution and return an empty path
        std::cout << "[ALGORITHM_SELECTOR] Algorithm execution completed" << std::endl;
        
        auto endTime = std::chrono::steady_clock::now();
        double executionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        // Record performance metrics
        AlgorithmPerformanceMetrics metrics;
        metrics.executionTime = executionTime;
        metrics.nodesExplored = 100; // Simulated value
        metrics.pathLength = path.empty() ? 0.0 : static_cast<double>(path.size());
        metrics.memoryUsage = 1024; // Simulated value
        metrics.optimality = path.empty() ? 0.0 : 0.9; // Simulated value
        
        updatePerformanceMetrics(algorithm, metrics);
        
        if (g_hybridManager) {
            g_hybridManager->recordExecutionResult(algorithm, metrics, !path.empty());
        }
        
    } catch (const std::exception& e) {
        std::cout << "[ALGORITHM_SELECTOR] Algorithm execution failed: " << e.what() << std::endl;
        
        // Record failure
        AlgorithmPerformanceMetrics failureMetrics{0.0, 0, 0.0, 0, 0.0};
        updatePerformanceMetrics(algorithm, failureMetrics);
        
        if (g_hybridManager) {
            g_hybridManager->recordExecutionResult(algorithm, failureMetrics, false);
        }
    }
    
    return path;
}