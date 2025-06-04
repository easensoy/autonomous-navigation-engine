#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include <iostream>
#include <chrono>
#include <algorithm>
#include <unordered_set>

class CompletePathPlanner {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStarPlanner;
    std::unique_ptr<Dijkstra> dijkstraPlanner;
    std::unique_ptr<BellmanFord> bellmanFordPlanner;
    std::unique_ptr<JumpPointSearch> jpsPlanner;
    
    struct PlanningResult {
        std::vector<int> path;
        double cost;
        double executionTime;
        PlanningAlgorithm algorithm;
        bool successful;
        std::string errorMessage;
        
        PlanningResult() : cost(std::numeric_limits<double>::infinity()), 
                          executionTime(0.0), algorithm(PlanningAlgorithm::AUTO_SELECT), 
                          successful(false) {}
    };
    
    std::vector<PlanningResult> planningHistory;
    bool enableFallbackStrategies;
    bool enableMultiAlgorithmComparison;
    double maxPlanningTime;
    
    PlanningResult executePlanningWithAlgorithm(int startId, int goalId, PlanningAlgorithm algorithm) {
        PlanningResult result;
        result.algorithm = algorithm;
        
        auto startTime = std::chrono::steady_clock::now();
        
        try {
            std::cout << "[COMPLETE_PLANNER] Executing " << algorithmToString(algorithm) 
                      << " from " << startId << " to " << goalId << std::endl;
            
            switch (algorithm) {
                case PlanningAlgorithm::ASTAR:
                    result.path = aStarPlanner->findPath(startId, goalId);
                    break;
                    
                case PlanningAlgorithm::DIJKSTRA:
                    result.path = dijkstraPlanner->findShortestPath(startId, goalId);
                    break;
                    
                case PlanningAlgorithm::BELLMAN_FORD:
                    result.path = bellmanFordPlanner->findShortestPath(startId, goalId);
                    break;
                    
                default:
                    result.path = aStarPlanner->findPath(startId, goalId);
                    result.algorithm = PlanningAlgorithm::ASTAR;
                    break;
            }
            
            auto endTime = std::chrono::steady_clock::now();
            result.executionTime = std::chrono::duration<double>(endTime - startTime).count();
            
            if (!result.path.empty()) {
                result.cost = calculatePathCost(result.path);
                result.successful = true;
                
                std::cout << "[COMPLETE_PLANNER] " << algorithmToString(algorithm) 
                          << " found path with " << result.path.size() << " nodes, cost: " 
                          << result.cost << ", time: " << result.executionTime << "s" << std::endl;
            } else {
                result.successful = false;
                result.errorMessage = "No path found";
                std::cout << "[COMPLETE_PLANNER] " << algorithmToString(algorithm) 
                          << " failed to find path" << std::endl;
            }
            
        } catch (const std::exception& e) {
            auto endTime = std::chrono::steady_clock::now();
            result.executionTime = std::chrono::duration<double>(endTime - startTime).count();
            result.successful = false;
            result.errorMessage = e.what();
            
            std::cout << "[COMPLETE_PLANNER] " << algorithmToString(algorithm) 
                      << " threw exception: " << e.what() << std::endl;
        }
        
        return result;
    }
    
    std::vector<int> executeFallbackStrategy(int startId, int goalId) {
        std::cout << "[COMPLETE_PLANNER] Executing fallback strategy" << std::endl;
        
        // Try algorithms in order of robustness
        std::vector<PlanningAlgorithm> fallbackOrder = {
            PlanningAlgorithm::DIJKSTRA,
            PlanningAlgorithm::BELLMAN_FORD,
            PlanningAlgorithm::ASTAR
        };
        
        for (PlanningAlgorithm algorithm : fallbackOrder) {
            PlanningResult result = executePlanningWithAlgorithm(startId, goalId, algorithm);
            
            if (result.successful) {
                std::cout << "[COMPLETE_PLANNER] Fallback successful with " 
                          << algorithmToString(algorithm) << std::endl;
                return result.path;
            }
        }
        
        // Last resort: try BFS for connectivity check
        std::cout << "[COMPLETE_PLANNER] All algorithms failed, checking basic connectivity" << std::endl;
        return findSimplePath(startId, goalId);
    }
    
    std::vector<int> findSimplePath(int startId, int goalId) {
        // Basic BFS implementation as last resort
        std::queue<int> frontier;
        std::unordered_map<int, int> parent;
        std::unordered_set<int> visited;
        
        frontier.push(startId);
        visited.insert(startId);
        parent[startId] = -1;
        
        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();
            
            if (current == goalId) {
                return reconstructSimplePath(parent, startId, goalId);
            }
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    parent[neighbor] = current;
                    frontier.push(neighbor);
                }
            }
        }
        
        return {};
    }
    
    std::vector<int> reconstructSimplePath(const std::unordered_map<int, int>& parent, 
                                          int startId, int goalId) {
        std::vector<int> path;
        int current = goalId;
        
        while (current != -1) {
            path.push_back(current);
            if (current == startId) break;
            current = parent.at(current);
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    PlanningResult selectBestResult(const std::vector<PlanningResult>& results) {
        if (results.empty()) {
            return PlanningResult();
        }
        
        // Filter successful results
        std::vector<PlanningResult> successfulResults;
        for (const auto& result : results) {
            if (result.successful) {
                successfulResults.push_back(result);
            }
        }
        
        if (successfulResults.empty()) {
            return results[0]; // Return first (failed) result
        }
        
        // Find result with best cost-time trade-off
        auto bestResult = std::min_element(successfulResults.begin(), successfulResults.end(),
            [](const PlanningResult& a, const PlanningResult& b) {
                // Prioritize cost, but consider execution time
                double scoreA = a.cost + (a.executionTime * 10.0); // Weight time less than cost
                double scoreB = b.cost + (b.executionTime * 10.0);
                return scoreA < scoreB;
            });
        
        std::cout << "[COMPLETE_PLANNER] Selected " << algorithmToString(bestResult->algorithm) 
                  << " as best result (cost: " << bestResult->cost 
                  << ", time: " << bestResult->executionTime << "s)" << std::endl;
        
        return *bestResult;
    }
    
    double calculatePathCost(const std::vector<int>& path) {
        if (path.size() < 2) return 0.0;
        
        double totalCost = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            double edgeWeight = graph->getEdgeWeight(path[i-1], path[i]);
            if (edgeWeight < 0) {
                // Edge doesn't exist - calculate euclidean distance
                const Node& from = graph->getNode(path[i-1]);
                const Node& to = graph->getNode(path[i]);
                totalCost += from.euclideanDistance(to);
            } else {
                totalCost += edgeWeight;
            }
        }
        
        return totalCost;
    }
    
    std::string algorithmToString(PlanningAlgorithm algorithm) const {
        switch (algorithm) {
            case PlanningAlgorithm::ASTAR: return "A*";
            case PlanningAlgorithm::DIJKSTRA: return "Dijkstra";
            case PlanningAlgorithm::BELLMAN_FORD: return "Bellman-Ford";
            case PlanningAlgorithm::AUTO_SELECT: return "Auto-Select";
            default: return "Unknown";
        }
    }
    
    bool validatePath(const std::vector<int>& path) {
        if (path.empty()) return false;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                return false;
            }
            
            if (!graph->hasEdge(path[i-1], path[i])) {
                std::cout << "[COMPLETE_PLANNER] Warning: No edge between " 
                          << path[i-1] << " and " << path[i] << std::endl;
            }
        }
        
        return true;
    }
    
public:
    CompletePathPlanner(const Graph* environment) 
        : graph(environment), enableFallbackStrategies(true), 
          enableMultiAlgorithmComparison(false), maxPlanningTime(10.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStarPlanner = std::make_unique<AStar>(graph);
        dijkstraPlanner = std::make_unique<Dijkstra>(graph);
        bellmanFordPlanner = std::make_unique<BellmanFord>(graph);
        
        std::cout << "[COMPLETE_PLANNER] Complete path planner initialized" << std::endl;
    }
    
    std::vector<int> planCompletePath(int startId, int goalId) {
        std::cout << "[COMPLETE_PLANNER] Planning complete path from " 
                  << startId << " to " << goalId << std::endl;
        
        if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
            throw std::invalid_argument("Start or goal node does not exist");
        }
        
        if (startId == goalId) {
            return {startId};
        }
        
        std::vector<PlanningResult> results;
        
        if (enableMultiAlgorithmComparison) {
            // Test multiple algorithms and select best
            std::vector<PlanningAlgorithm> algorithms = {
                PlanningAlgorithm::ASTAR,
                PlanningAlgorithm::DIJKSTRA
            };
            
            for (PlanningAlgorithm algorithm : algorithms) {
                PlanningResult result = executePlanningWithAlgorithm(startId, goalId, algorithm);
                results.push_back(result);
            }
            
            PlanningResult bestResult = selectBestResult(results);
            planningHistory.push_back(bestResult);
            
            if (bestResult.successful) {
                return bestResult.path;
            }
        } else {
            // Use A* as primary algorithm
            PlanningResult result = executePlanningWithAlgorithm(startId, goalId, PlanningAlgorithm::ASTAR);
            planningHistory.push_back(result);
            
            if (result.successful) {
                return result.path;
            }
        }
        
        // Execute fallback if enabled and primary planning failed
        if (enableFallbackStrategies) {
            return executeFallbackStrategy(startId, goalId);
        }
        
        std::cout << "[COMPLETE_PLANNER] All planning attempts failed" << std::endl;
        return {};
    }
    
    std::vector<int> planPathWithConstraints(int startId, int goalId, 
                                           const std::vector<int>& avoidNodes,
                                           double maxCost = std::numeric_limits<double>::infinity()) {
        std::cout << "[COMPLETE_PLANNER] Planning path with " << avoidNodes.size() 
                  << " nodes to avoid, max cost: " << maxCost << std::endl;
        
        // Temporarily modify graph to avoid specified nodes
        // This is a simplified implementation - in practice, you'd use a more sophisticated approach
        std::vector<int> path = planCompletePath(startId, goalId);
        
        if (path.empty()) {
            return {};
        }
        
        // Check if path violates constraints
        std::unordered_set<int> avoidSet(avoidNodes.begin(), avoidNodes.end());
        bool hasViolation = false;
        
        for (int nodeId : path) {
            if (avoidSet.find(nodeId) != avoidSet.end()) {
                hasViolation = true;
                break;
            }
        }
        
        double pathCost = calculatePathCost(path);
        if (pathCost > maxCost) {
            hasViolation = true;
        }
        
        if (!hasViolation) {
            std::cout << "[COMPLETE_PLANNER] Path satisfies all constraints" << std::endl;
            return path;
        }
        
        std::cout << "[COMPLETE_PLANNER] Path violates constraints, attempting constrained planning" << std::endl;
        
        // For now, return empty path if constraints violated
        // A full implementation would modify the graph or use constrained algorithms
        return {};
    }
    
    std::vector<std::vector<int>> planMultiplePaths(int startId, int goalId, int numPaths) {
        std::cout << "[COMPLETE_PLANNER] Planning " << numPaths 
                  << " alternative paths from " << startId << " to " << goalId << std::endl;
        
        std::vector<std::vector<int>> paths;
        
        // First path using standard planning
        std::vector<int> primaryPath = planCompletePath(startId, goalId);
        if (!primaryPath.empty()) {
            paths.push_back(primaryPath);
        }
        
        // Generate alternative paths by avoiding nodes from previous paths
        std::unordered_set<int> usedNodes;
        for (int nodeId : primaryPath) {
            usedNodes.insert(nodeId);
        }
        
        for (int i = 1; i < numPaths; ++i) {
            std::vector<int> avoidNodes(usedNodes.begin(), usedNodes.end());
            std::vector<int> altPath = planPathWithConstraints(startId, goalId, avoidNodes);
            
            if (!altPath.empty()) {
                paths.push_back(altPath);
                
                // Add nodes from this path to avoid set
                for (int nodeId : altPath) {
                    usedNodes.insert(nodeId);
                }
            }
        }
        
        std::cout << "[COMPLETE_PLANNER] Generated " << paths.size() 
                  << " alternative paths" << std::endl;
        
        return paths;
    }
    
    void enableFallbackPlanning(bool enable) {
        enableFallbackStrategies = enable;
        std::cout << "[COMPLETE_PLANNER] Fallback strategies " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableAlgorithmComparison(bool enable) {
        enableMultiAlgorithmComparison = enable;
        std::cout << "[COMPLETE_PLANNER] Multi-algorithm comparison " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void setMaxPlanningTime(double seconds) {
        maxPlanningTime = seconds;
        std::cout << "[COMPLETE_PLANNER] Max planning time set to " << seconds << "s" << std::endl;
    }
    
    void printPlanningHistory() const {
        std::cout << "[COMPLETE_PLANNER] Planning History (" << planningHistory.size() << " entries):" << std::endl;
        
        for (size_t i = 0; i < planningHistory.size(); ++i) {
            const auto& result = planningHistory[i];
            std::cout << "[COMPLETE_PLANNER]   " << i << ": " << algorithmToString(result.algorithm) 
                      << " - " << (result.successful ? "SUCCESS" : "FAILED");
            
            if (result.successful) {
                std::cout << " (nodes: " << result.path.size() 
                          << ", cost: " << result.cost 
                          << ", time: " << result.executionTime << "s)";
            } else {
                std::cout << " (" << result.errorMessage << ")";
            }
            std::cout << std::endl;
        }
    }
    
    void clearPlanningHistory() {
        planningHistory.clear();
        std::cout << "[COMPLETE_PLANNER] Planning history cleared" << std::endl;
    }
    
    double getAveragePlanningTime() const {
        if (planningHistory.empty()) return 0.0;
        
        double totalTime = 0.0;
        size_t successfulPlans = 0;
        
        for (const auto& result : planningHistory) {
            if (result.successful) {
                totalTime += result.executionTime;
                successfulPlans++;
            }
        }
        
        return successfulPlans > 0 ? totalTime / successfulPlans : 0.0;
    }
    
    size_t getSuccessfulPlanCount() const {
        return std::count_if(planningHistory.begin(), planningHistory.end(),
                           [](const PlanningResult& result) { return result.successful; });
    }
};