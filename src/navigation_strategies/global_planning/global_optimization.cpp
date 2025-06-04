#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "path_operations/PathOptimizer.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>
#include <chrono>
#include <corecrt_math_defines.h>
#include <numeric>

class GlobalOptimizer {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStar;
    
    struct OptimizationResult {
        std::vector<int> optimizedPath;
        double originalCost;
        double optimizedCost;
        double improvementRatio;
        std::string method;
        double executionTime;
        
        OptimizationResult() : originalCost(0.0), optimizedCost(0.0), 
                              improvementRatio(0.0), executionTime(0.0) {}
    };
    
    std::vector<OptimizationResult> optimizationHistory;
    double convergenceThreshold;
    int maxIterations;
    bool enableMultiObjective;
    
    double calculatePathCost(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double cost = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            cost += from.euclideanDistance(to);
        }
        return cost;
    }
    
    std::vector<int> optimizeWithShortcutting(const std::vector<int>& originalPath) {
        std::cout << "[GLOBAL_OPT] Applying shortcutting optimization" << std::endl;
        
        if (originalPath.size() < 3) return originalPath;
        
        std::vector<int> optimizedPath = originalPath;
        bool improved = true;
        int iterations = 0;
        
        while (improved && iterations < maxIterations) {
            improved = false;
            iterations++;
            
            for (size_t i = 0; i < optimizedPath.size() - 2; ++i) {
                for (size_t j = i + 2; j < optimizedPath.size(); ++j) {
                    // Try to shortcut from i to j
                    if (canShortcut(optimizedPath[i], optimizedPath[j])) {
                        // Calculate improvement
                        double oldSegmentCost = calculateSegmentCost(optimizedPath, i, j);
                        double newSegmentCost = getDirectCost(optimizedPath[i], optimizedPath[j]);
                        
                        if (newSegmentCost < oldSegmentCost * (1.0 - convergenceThreshold)) {
                            // Apply shortcut
                            std::vector<int> newPath;
                            newPath.insert(newPath.end(), optimizedPath.begin(), optimizedPath.begin() + i + 1);
                            newPath.push_back(optimizedPath[j]);
                            newPath.insert(newPath.end(), optimizedPath.begin() + j + 1, optimizedPath.end());
                            
                            optimizedPath = newPath;
                            improved = true;
                            
                            std::cout << "[GLOBAL_OPT] Applied shortcut from " << optimizedPath[i] 
                                      << " to " << optimizedPath[j] << std::endl;
                            break;
                        }
                    }
                }
                if (improved) break;
            }
        }
        
        std::cout << "[GLOBAL_OPT] Shortcutting completed after " << iterations << " iterations" << std::endl;
        return optimizedPath;
    }
    
    std::vector<int> optimizeWithSimulatedAnnealing(const std::vector<int>& originalPath) {
        std::cout << "[GLOBAL_OPT] Applying simulated annealing optimization" << std::endl;
        
        if (originalPath.size() < 3) return originalPath;
        
        std::vector<int> currentPath = originalPath;
        std::vector<int> bestPath = originalPath;
        double currentCost = calculatePathCost(currentPath);
        double bestCost = currentCost;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        double temperature = 100.0;
        double coolingRate = 0.95;
        double minTemperature = 0.1;
        
        int iterations = 0;
        while (temperature > minTemperature && iterations < maxIterations) {
            // Generate neighbor solution by swapping two random nodes
            std::vector<int> neighborPath = generateNeighborSolution(currentPath);
            double neighborCost = calculatePathCost(neighborPath);
            
            // Accept or reject the neighbor
            double deltaE = neighborCost - currentCost;
            if (deltaE < 0 || dis(gen) < std::exp(-deltaE / temperature)) {
                currentPath = neighborPath;
                currentCost = neighborCost;
                
                if (currentCost < bestCost) {
                    bestPath = currentPath;
                    bestCost = currentCost;
                    std::cout << "[GLOBAL_OPT] New best cost: " << bestCost << std::endl;
                }
            }
            
            temperature *= coolingRate;
            iterations++;
        }
        
        std::cout << "[GLOBAL_OPT] Simulated annealing completed after " 
                  << iterations << " iterations" << std::endl;
        return bestPath;
    }
    
    std::vector<int> optimizeWithGeneticAlgorithm(const std::vector<int>& originalPath) {
        std::cout << "[GLOBAL_OPT] Applying genetic algorithm optimization" << std::endl;
        
        if (originalPath.size() < 3) return originalPath;
        
        const int populationSize = 50;
        const double mutationRate = 0.1;
        const double crossoverRate = 0.8;
        
        // Initialize population
        std::vector<std::vector<int>> population;
        for (int i = 0; i < populationSize; ++i) {
            std::vector<int> individual = originalPath;
            if (i > 0) {
                mutateIndividual(individual);
            }
            population.push_back(individual);
        }
        
        std::vector<int> bestIndividual = originalPath;
        double bestFitness = 1.0 / (1.0 + calculatePathCost(originalPath));
        
        for (int generation = 0; generation < maxIterations / 10; ++generation) {
            // Evaluate fitness
            std::vector<double> fitness;
            for (const auto& individual : population) {
                double cost = calculatePathCost(individual);
                double individualFitness = 1.0 / (1.0 + cost);
                fitness.push_back(individualFitness);
                
                if (individualFitness > bestFitness) {
                    bestFitness = individualFitness;
                    bestIndividual = individual;
                }
            }
            
            // Create new generation
            std::vector<std::vector<int>> newPopulation;
            
            for (int i = 0; i < populationSize; ++i) {
                // Selection
                int parent1 = selectParent(fitness);
                int parent2 = selectParent(fitness);
                
                // Crossover
                std::vector<int> offspring = population[parent1];
                if (generateRandomDouble() < crossoverRate) {
                    offspring = crossover(population[parent1], population[parent2]);
                }
                
                // Mutation
                if (generateRandomDouble() < mutationRate) {
                    mutateIndividual(offspring);
                }
                
                newPopulation.push_back(offspring);
            }
            
            population = newPopulation;
        }
        
        std::cout << "[GLOBAL_OPT] Genetic algorithm completed" << std::endl;
        return bestIndividual;
    }
    
    std::vector<int> optimizeMultiObjective(const std::vector<int>& originalPath, 
                                          const std::vector<double>& weights) {
        std::cout << "[GLOBAL_OPT] Applying multi-objective optimization" << std::endl;
        
        // Multi-objective optimization considering:
        // 1. Path length (distance)
        // 2. Path smoothness (curvature)
        // 3. Safety (clearance from obstacles)
        
        std::vector<int> currentPath = originalPath;
        double currentScore = calculateMultiObjectiveScore(currentPath, weights);
        
        bool improved = true;
        int iterations = 0;
        
        while (improved && iterations < maxIterations) {
            improved = false;
            
            // Try local improvements
            for (size_t i = 1; i < currentPath.size() - 1; ++i) {
                std::vector<int> neighbors = graph->getNeighbors(currentPath[i]);
                
                for (int neighbor : neighbors) {
                    if (std::find(currentPath.begin(), currentPath.end(), neighbor) == currentPath.end()) {
                        // Try replacing current node with neighbor
                        std::vector<int> testPath = currentPath;
                        testPath[i] = neighbor;
                        
                        if (isValidPath(testPath)) {
                            double testScore = calculateMultiObjectiveScore(testPath, weights);
                            
                            if (testScore < currentScore) {
                                currentPath = testPath;
                                currentScore = testScore;
                                improved = true;
                                break;
                            }
                        }
                    }
                }
                if (improved) break;
            }
            
            iterations++;
        }
        
        std::cout << "[GLOBAL_OPT] Multi-objective optimization completed after " 
                  << iterations << " iterations" << std::endl;
        return currentPath;
    }
    
    double calculateMultiObjectiveScore(const std::vector<int>& path, 
                                       const std::vector<double>& weights) const {
        if (weights.size() < 3) return calculatePathCost(path);
        
        double distance = calculatePathCost(path);
        double smoothness = calculatePathSmoothness(path);
        double safety = calculatePathSafety(path);
        
        return weights[0] * distance + weights[1] * smoothness + weights[2] * safety;
    }
    
    double calculatePathSmoothness(const std::vector<int>& path) const {
        if (path.size() < 3) return 0.0;
        
        double totalCurvature = 0.0;
        for (size_t i = 1; i < path.size() - 1; ++i) {
            const Node& prev = graph->getNode(path[i-1]);
            const Node& curr = graph->getNode(path[i]);
            const Node& next = graph->getNode(path[i+1]);
            
            // Calculate angle change
            double angle1 = std::atan2(curr.getY() - prev.getY(), curr.getX() - prev.getX());
            double angle2 = std::atan2(next.getY() - curr.getY(), next.getX() - curr.getX());
            double angleChange = std::abs(angle2 - angle1);
            
            if (angleChange > M_PI) angleChange = 2 * M_PI - angleChange;
            totalCurvature += angleChange;
        }
        
        return totalCurvature;
    }
    
    double calculatePathSafety(const std::vector<int>& path) const {
        // Placeholder for safety calculation
        // In practice, this would consider clearance from obstacles
        return 0.0;
    }
    
    bool canShortcut(int fromNode, int toNode) const {
        return graph->hasEdge(fromNode, toNode);
    }
    
    double calculateSegmentCost(const std::vector<int>& path, size_t start, size_t end) const {
        double cost = 0.0;
        for (size_t i = start; i < end; ++i) {
            const Node& from = graph->getNode(path[i]);
            const Node& to = graph->getNode(path[i+1]);
            cost += from.euclideanDistance(to);
        }
        return cost;
    }
    
    double getDirectCost(int fromNode, int toNode) const {
        const Node& from = graph->getNode(fromNode);
        const Node& to = graph->getNode(toNode);
        return from.euclideanDistance(to);
    }
    
    std::vector<int> generateNeighborSolution(const std::vector<int>& path) const {
        if (path.size() < 3) return path;
        
        std::vector<int> neighbor = path;
        
        // Random 2-opt swap
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, static_cast<int>(path.size()) - 2);
        
        int i = dis(gen);
        int j = dis(gen);
        if (i > j) std::swap(i, j);
        
        std::reverse(neighbor.begin() + i, neighbor.begin() + j + 1);
        
        return neighbor;
    }
    
    void mutateIndividual(std::vector<int>& individual) const {
        if (individual.size() < 3) return;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, static_cast<int>(individual.size()) - 2);
        
        int pos = dis(gen);
        std::vector<int> neighbors = graph->getNeighbors(individual[pos]);
        
        if (!neighbors.empty()) {
            std::uniform_int_distribution<> neighborDis(0, static_cast<int>(neighbors.size()) - 1);
            individual[pos] = neighbors[neighborDis(gen)];
        }
    }
    
    int selectParent(const std::vector<double>& fitness) const {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0.0, 1.0);
        
        double totalFitness = std::accumulate(fitness.begin(), fitness.end(), 0.0);
        double randomValue = dis(gen) * totalFitness;
        
        double cumulative = 0.0;
        for (size_t i = 0; i < fitness.size(); ++i) {
            cumulative += fitness[i];
            if (cumulative >= randomValue) {
                return static_cast<int>(i);
            }
        }
        
        return static_cast<int>(fitness.size()) - 1;
    }
    
    std::vector<int> crossover(const std::vector<int>& parent1, const std::vector<int>& parent2) const {
        // Order crossover
        if (parent1.size() != parent2.size() || parent1.size() < 3) {
            return parent1;
        }
        
        std::vector<int> offspring = parent1;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(1, static_cast<int>(parent1.size()) - 2);
        
        int start = dis(gen);
        int end = dis(gen);
        if (start > end) std::swap(start, end);
        
        // Copy segment from parent2
        for (int i = start; i <= end; ++i) {
            offspring[i] = parent2[i];
        }
        
        return offspring;
    }
    
    bool isValidPath(const std::vector<int>& path) const {
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) {
                return false;
            }
        }
        return true;
    }
    
    double generateRandomDouble() const {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<> dis(0.0, 1.0);
        return dis(gen);
    }
    
public:
    GlobalOptimizer(const Graph* environment) 
        : graph(environment), convergenceThreshold(0.01), maxIterations(1000), 
          enableMultiObjective(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStar = std::make_unique<AStar>(graph);
        std::cout << "[GLOBAL_OPT] Global optimizer initialized" << std::endl;
    }
    
    std::vector<int> optimizePath(const std::vector<int>& originalPath, const std::string& method = "shortcut") {
        auto startTime = std::chrono::steady_clock::now();
        
        std::cout << "[GLOBAL_OPT] Optimizing path with " << originalPath.size() 
                  << " nodes using " << method << " method" << std::endl;
        
        OptimizationResult result;
        result.method = method;
        result.originalCost = calculatePathCost(originalPath);
        
        if (method == "shortcut") {
            result.optimizedPath = optimizeWithShortcutting(originalPath);
        } else if (method == "annealing") {
            result.optimizedPath = optimizeWithSimulatedAnnealing(originalPath);
        } else if (method == "genetic") {
            result.optimizedPath = optimizeWithGeneticAlgorithm(originalPath);
        } else {
            result.optimizedPath = optimizeWithShortcutting(originalPath);
        }
        
        auto endTime = std::chrono::steady_clock::now();
        result.executionTime = std::chrono::duration<double>(endTime - startTime).count();
        result.optimizedCost = calculatePathCost(result.optimizedPath);
        result.improvementRatio = (result.originalCost - result.optimizedCost) / result.originalCost;
        
        optimizationHistory.push_back(result);
        
        std::cout << "[GLOBAL_OPT] Optimization completed: " << result.originalCost 
                  << " -> " << result.optimizedCost << " (" 
                  << (result.improvementRatio * 100) << "% improvement)" << std::endl;
        
        return result.optimizedPath;
    }
    
    std::vector<int> optimizeMultiObjectivePath(const std::vector<int>& originalPath,
                                              const std::vector<double>& weights) {
        enableMultiObjective = true;
        auto result = optimizeMultiObjective(originalPath, weights);
        enableMultiObjective = false;
        return result;
    }
    
    void setConvergenceThreshold(double threshold) {
        convergenceThreshold = threshold;
        std::cout << "[GLOBAL_OPT] Convergence threshold set to " << threshold << std::endl;
    }
    
    void setMaxIterations(int iterations) {
        maxIterations = iterations;
        std::cout << "[GLOBAL_OPT] Max iterations set to " << iterations << std::endl;
    }
    
    void printOptimizationStatistics() const {
        std::cout << "[GLOBAL_OPT] Optimization Statistics:" << std::endl;
        std::cout << "[GLOBAL_OPT]   Total optimizations: " << optimizationHistory.size() << std::endl;
        
        if (!optimizationHistory.empty()) {
            double totalImprovement = 0.0;
            double totalTime = 0.0;
            
            for (const auto& result : optimizationHistory) {
                totalImprovement += result.improvementRatio;
                totalTime += result.executionTime;
            }
            
            std::cout << "[GLOBAL_OPT]   Average improvement: " 
                      << (totalImprovement / optimizationHistory.size() * 100) << "%" << std::endl;
            std::cout << "[GLOBAL_OPT]   Average time: " 
                      << (totalTime / optimizationHistory.size()) << "s" << std::endl;
        }
    }
};