#include "navigation_strategies/MultiGoalPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include <iostream>
#include <algorithm>
#include <random>
#include <chrono>
#include <unordered_map>
#include <limits>
#include <cmath>

class TravelingSalesmanSolver {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStarPlanner;
    
    struct TSPSolution {
        std::vector<int> tour;
        std::vector<std::vector<int>> detailedPath;
        double totalCost;
        double totalDistance;
        std::string algorithm;
        double executionTime;
        bool isRoundTrip;
        
        TSPSolution() : totalCost(0.0), totalDistance(0.0), executionTime(0.0), isRoundTrip(false) {}
    };
    
    struct DistanceMatrix {
        std::unordered_map<std::string, double> distances;
        std::unordered_map<std::string, std::vector<int>> paths;
        std::vector<int> nodeList;
        
        void clear() {
            distances.clear();
            paths.clear();
            nodeList.clear();
        }
        
        std::string getKey(int from, int to) const {
            return std::to_string(from) + "_" + std::to_string(to);
        }
        
        double getDistance(int from, int to) const {
            auto it = distances.find(getKey(from, to));
            return (it != distances.end()) ? it->second : std::numeric_limits<double>::infinity();
        }
        
        std::vector<int> getPath(int from, int to) const {
            auto it = paths.find(getKey(from, to));
            return (it != paths.end()) ? it->second : std::vector<int>();
        }
        
        void setDistance(int from, int to, double distance) {
            distances[getKey(from, to)] = distance;
        }
        
        void setPath(int from, int to, const std::vector<int>& path) {
            paths[getKey(from, to)] = path;
        }
    };
    
    DistanceMatrix distanceMatrix;
    bool enableSymmetryOptimization;
    bool requireRoundTrip;
    int maxIterationsGA;
    double mutationRate;
    double coolingRate;
    
    void computeDistanceMatrix(const std::vector<int>& nodes) {
        std::cout << "[TSP_SOLVER] Computing distance matrix for " << nodes.size() << " nodes" << std::endl;
        
        distanceMatrix.clear();
        distanceMatrix.nodeList = nodes;
        
        for (size_t i = 0; i < nodes.size(); ++i) {
            for (size_t j = 0; j < nodes.size(); ++j) {
                if (i != j) {
                    int fromNode = nodes[i];
                    int toNode = nodes[j];
                    
                    std::vector<int> path = aStarPlanner->findPath(fromNode, toNode);
                    double distance = calculatePathDistance(path);
                    
                    distanceMatrix.setDistance(fromNode, toNode, distance);
                    distanceMatrix.setPath(fromNode, toNode, path);
                    
                    // If symmetric optimization is enabled, set reverse distance
                    if (enableSymmetryOptimization && i < j) {
                        distanceMatrix.setDistance(toNode, fromNode, distance);
                        std::vector<int> reversePath = path;
                        std::reverse(reversePath.begin(), reversePath.end());
                        distanceMatrix.setPath(toNode, fromNode, reversePath);
                    }
                }
            }
        }
        
        std::cout << "[TSP_SOLVER] Distance matrix computation completed" << std::endl;
    }
    
    double calculatePathDistance(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalDistance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalDistance += from.euclideanDistance(to);
        }
        return totalDistance;
    }
    
    double calculateTourCost(const std::vector<int>& tour) const {
        if (tour.size() < 2) return 0.0;
        
        double totalCost = 0.0;
        for (size_t i = 1; i < tour.size(); ++i) {
            totalCost += distanceMatrix.getDistance(tour[i-1], tour[i]);
        }
        
        if (requireRoundTrip && tour.size() > 2) {
            totalCost += distanceMatrix.getDistance(tour.back(), tour.front());
        }
        
        return totalCost;
    }
    
    TSPSolution solveNearestNeighbor(const std::vector<int>& nodes, int startNode) {
        std::cout << "[TSP_SOLVER] Solving TSP using Nearest Neighbor heuristic" << std::endl;
        
        auto startTime = std::chrono::steady_clock::now();
        
        TSPSolution solution;
        solution.algorithm = "Nearest Neighbor";
        solution.isRoundTrip = requireRoundTrip;
        
        std::unordered_set<int> unvisited(nodes.begin(), nodes.end());
        int current = startNode;
        solution.tour.push_back(current);
        unvisited.erase(current);
        
        while (!unvisited.empty()) {
            int nearest = -1;
            double minDistance = std::numeric_limits<double>::infinity();
            
            for (int candidate : unvisited) {
                double distance = distanceMatrix.getDistance(current, candidate);
                if (distance < minDistance) {
                    minDistance = distance;
                    nearest = candidate;
                }
            }
            
            if (nearest != -1) {
                solution.tour.push_back(nearest);
                unvisited.erase(nearest);
                current = nearest;
            } else {
                break;
            }
        }
        
        if (requireRoundTrip && solution.tour.size() > 1) {
            solution.tour.push_back(startNode);
        }
        
        solution.totalCost = calculateTourCost(solution.tour);
        solution.detailedPath = constructDetailedPath(solution.tour);
        solution.totalDistance = calculatePathDistance(flattenPath(solution.detailedPath));
        
        auto endTime = std::chrono::steady_clock::now();
        solution.executionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        std::cout << "[TSP_SOLVER] Nearest Neighbor solution: cost " << solution.totalCost 
                  << ", time " << solution.executionTime << "s" << std::endl;
        
        return solution;
    }
    
    TSPSolution solveGeneticAlgorithm(const std::vector<int>& nodes, int startNode) {
        std::cout << "[TSP_SOLVER] Solving TSP using Genetic Algorithm" << std::endl;
        
        auto startTime = std::chrono::steady_clock::now();
        
        TSPSolution solution;
        solution.algorithm = "Genetic Algorithm";
        solution.isRoundTrip = requireRoundTrip;
        
        const int populationSize = 100;
        const int maxGenerations = maxIterationsGA;
        
        // Create initial population
        std::vector<std::vector<int>> population = createInitialPopulation(nodes, startNode, populationSize);
        
        std::random_device rd;
        std::mt19937 gen(rd());
        
        std::vector<int> bestTour;
        double bestCost = std::numeric_limits<double>::infinity();
        
        for (int generation = 0; generation < maxGenerations; ++generation) {
            // Evaluate fitness
            std::vector<double> fitness;
            for (const auto& tour : population) {
                double cost = calculateTourCost(tour);
                fitness.push_back(1.0 / (1.0 + cost)); // Higher fitness = lower cost
                
                if (cost < bestCost) {
                    bestCost = cost;
                    bestTour = tour;
                }
            }
            
            // Create next generation
            std::vector<std::vector<int>> newPopulation;
            
            for (int i = 0; i < populationSize; ++i) {
                // Selection
                int parent1 = selectParent(fitness, gen);
                int parent2 = selectParent(fitness, gen);
                
                // Crossover
                std::vector<int> offspring = crossover(population[parent1], population[parent2], gen);
                
                // Mutation
                if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < mutationRate) {
                    mutate(offspring, gen);
                }
                
                newPopulation.push_back(offspring);
            }
            
            population = newPopulation;
            
            if (generation % 100 == 0) {
                std::cout << "[TSP_SOLVER] Generation " << generation << ", best cost: " << bestCost << std::endl;
            }
        }
        
        solution.tour = bestTour;
        solution.totalCost = bestCost;
        solution.detailedPath = constructDetailedPath(solution.tour);
        solution.totalDistance = calculatePathDistance(flattenPath(solution.detailedPath));
        
        auto endTime = std::chrono::steady_clock::now();
        solution.executionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        std::cout << "[TSP_SOLVER] Genetic Algorithm solution: cost " << solution.totalCost 
                  << ", time " << solution.executionTime << "s" << std::endl;
        
        return solution;
    }
    
    TSPSolution solveSimulatedAnnealing(const std::vector<int>& nodes, int startNode) {
        std::cout << "[TSP_SOLVER] Solving TSP using Simulated Annealing" << std::endl;
        
        auto startTime = std::chrono::steady_clock::now();
        
        TSPSolution solution;
        solution.algorithm = "Simulated Annealing";
        solution.isRoundTrip = requireRoundTrip;
        
        // Start with nearest neighbor solution
        std::vector<int> currentTour = solveNearestNeighbor(nodes, startNode).tour;
        std::vector<int> bestTour = currentTour;
        
        double currentCost = calculateTourCost(currentTour);
        double bestCost = currentCost;
        
        double temperature = 1000.0;
        const double minTemperature = 0.1;
        
        std::random_device rd;
        std::mt19937 gen(rd());
        
        while (temperature > minTemperature) {
            // Generate neighbor solution using 2-opt
            std::vector<int> neighborTour = generate2OptNeighbor(currentTour, gen);
            double neighborCost = calculateTourCost(neighborTour);
            
            // Accept or reject the neighbor
            double deltaE = neighborCost - currentCost;
            if (deltaE < 0 || std::uniform_real_distribution<>(0.0, 1.0)(gen) < std::exp(-deltaE / temperature)) {
                currentTour = neighborTour;
                currentCost = neighborCost;
                
                if (currentCost < bestCost) {
                    bestTour = currentTour;
                    bestCost = currentCost;
                }
            }
            
            temperature *= coolingRate;
        }
        
        solution.tour = bestTour;
        solution.totalCost = bestCost;
        solution.detailedPath = constructDetailedPath(solution.tour);
        solution.totalDistance = calculatePathDistance(flattenPath(solution.detailedPath));
        
        auto endTime = std::chrono::steady_clock::now();
        solution.executionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        std::cout << "[TSP_SOLVER] Simulated Annealing solution: cost " << solution.totalCost 
                  << ", time " << solution.executionTime << "s" << std::endl;
        
        return solution;
    }
    
    TSPSolution solveDynamicProgramming(const std::vector<int>& nodes, int startNode) {
        std::cout << "[TSP_SOLVER] Solving TSP using Dynamic Programming (Held-Karp)" << std::endl;
        
        if (nodes.size() > 15) {
            std::cout << "[TSP_SOLVER] Too many nodes for DP, falling back to heuristic" << std::endl;
            return solveNearestNeighbor(nodes, startNode);
        }
        
        auto startTime = std::chrono::steady_clock::now();
        
        TSPSolution solution;
        solution.algorithm = "Dynamic Programming";
        solution.isRoundTrip = requireRoundTrip;
        
        int n = nodes.size();
        int startIndex = std::find(nodes.begin(), nodes.end(), startNode) - nodes.begin();
        
        // DP table: dp[mask][i] = minimum cost to visit all nodes in mask ending at i
        std::vector<std::vector<double>> dp(1 << n, std::vector<double>(n, std::numeric_limits<double>::infinity()));
        std::vector<std::vector<int>> parent(1 << n, std::vector<int>(n, -1));
        
        // Base case
        dp[1 << startIndex][startIndex] = 0;
        
        // Fill DP table
        for (int mask = 0; mask < (1 << n); ++mask) {
            for (int u = 0; u < n; ++u) {
                if (!(mask & (1 << u))) continue;
                if (dp[mask][u] == std::numeric_limits<double>::infinity()) continue;
                
                for (int v = 0; v < n; ++v) {
                    if (mask & (1 << v)) continue;
                    
                    int newMask = mask | (1 << v);
                    double cost = dp[mask][u] + distanceMatrix.getDistance(nodes[u], nodes[v]);
                    
                    if (cost < dp[newMask][v]) {
                        dp[newMask][v] = cost;
                        parent[newMask][v] = u;
                    }
                }
            }
        }
        
        // Find optimal tour
        int finalMask = (1 << n) - 1;
        double minCost = std::numeric_limits<double>::infinity();
        int lastNode = -1;
        
        for (int i = 0; i < n; ++i) {
            if (i == startIndex) continue;
            
            double cost = dp[finalMask][i];
            if (requireRoundTrip) {
                cost += distanceMatrix.getDistance(nodes[i], nodes[startIndex]);
            }
            
            if (cost < minCost) {
                minCost = cost;
                lastNode = i;
            }
        }
        
        // Reconstruct tour
        std::vector<int> tour;
        int mask = finalMask;
        int current = lastNode;
        
        while (current != -1) {
            tour.push_back(nodes[current]);
            int next = parent[mask][current];
            mask ^= (1 << current);
            current = next;
        }
        
        std::reverse(tour.begin(), tour.end());
        
        if (requireRoundTrip && tour.size() > 1) {
            tour.push_back(tour[0]);
        }
        
        solution.tour = tour;
        solution.totalCost = minCost;
        solution.detailedPath = constructDetailedPath(solution.tour);
        solution.totalDistance = calculatePathDistance(flattenPath(solution.detailedPath));
        
        auto endTime = std::chrono::steady_clock::now();
        solution.executionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        std::cout << "[TSP_SOLVER] Dynamic Programming solution: cost " << solution.totalCost 
                  << ", time " << solution.executionTime << "s" << std::endl;
        
        return solution;
    }
    
    std::vector<std::vector<int>> createInitialPopulation(const std::vector<int>& nodes, int startNode, int populationSize) {
        std::vector<std::vector<int>> population;
        std::random_device rd;
        std::mt19937 gen(rd());
        
        for (int i = 0; i < populationSize; ++i) {
            std::vector<int> tour = nodes;
            
            // Ensure start node is at the beginning
            auto startIt = std::find(tour.begin(), tour.end(), startNode);
            if (startIt != tour.begin()) {
                std::iter_swap(tour.begin(), startIt);
            }
            
            // Shuffle remaining nodes
            std::shuffle(tour.begin() + 1, tour.end(), gen);
            
            if (requireRoundTrip) {
                tour.push_back(startNode);
            }
            
            population.push_back(tour);
        }
        
        return population;
    }
    
    int selectParent(const std::vector<double>& fitness, std::mt19937& gen) {
        std::discrete_distribution<> dist(fitness.begin(), fitness.end());
        return dist(gen);
    }
    
    std::vector<int> crossover(const std::vector<int>& parent1, const std::vector<int>& parent2, std::mt19937& gen) {
        // Order crossover (OX)
        int size = std::min(parent1.size(), parent2.size());
        if (requireRoundTrip) size--; // Exclude duplicate start node at end
        
        std::uniform_int_distribution<> dist(1, size - 2);
        int start = dist(gen);
        int end = dist(gen);
        if (start > end) std::swap(start, end);
        
        std::vector<int> offspring(size);
        std::unordered_set<int> used;
        
        // Copy segment from parent1
        for (int i = start; i <= end; ++i) {
            offspring[i] = parent1[i];
            used.insert(parent1[i]);
        }
        
        // Fill remaining positions with parent2's order
        int pos = (end + 1) % size;
        for (int i = 0; i < size; ++i) {
            int node = parent2[(end + 1 + i) % size];
            if (used.find(node) == used.end()) {
                offspring[pos] = node;
                pos = (pos + 1) % size;
            }
        }
        
        if (requireRoundTrip) {
            offspring.push_back(offspring[0]);
        }
        
        return offspring;
    }
    
    void mutate(std::vector<int>& tour, std::mt19937& gen) {
        int size = tour.size();
        if (requireRoundTrip) size--; // Don't mutate duplicate start node
        
        if (size < 3) return;
        
        std::uniform_int_distribution<> dist(1, size - 1);
        int i = dist(gen);
        int j = dist(gen);
        
        if (i != j) {
            std::swap(tour[i], tour[j]);
        }
    }
    
    std::vector<int> generate2OptNeighbor(const std::vector<int>& tour, std::mt19937& gen) {
        std::vector<int> neighbor = tour;
        int size = tour.size();
        if (requireRoundTrip) size--; // Don't include duplicate start node
        
        if (size < 4) return neighbor;
        
        std::uniform_int_distribution<> dist(1, size - 1);
        int i = dist(gen);
        int j = dist(gen);
        
        if (i > j) std::swap(i, j);
        if (j - i < 2) return neighbor;
        
        // Reverse segment between i and j
        std::reverse(neighbor.begin() + i, neighbor.begin() + j + 1);
        
        return neighbor;
    }
    
    std::vector<std::vector<int>> constructDetailedPath(const std::vector<int>& tour) {
        std::vector<std::vector<int>> detailedPath;
        
        for (size_t i = 1; i < tour.size(); ++i) {
            std::vector<int> segment = distanceMatrix.getPath(tour[i-1], tour[i]);
            if (!segment.empty()) {
                detailedPath.push_back(segment);
            }
        }
        
        return detailedPath;
    }
    
    std::vector<int> flattenPath(const std::vector<std::vector<int>>& detailedPath) {
        std::vector<int> flattened;
        
        for (size_t i = 0; i < detailedPath.size(); ++i) {
            const auto& segment = detailedPath[i];
            
            if (i == 0) {
                flattened.insert(flattened.end(), segment.begin(), segment.end());
            } else {
                // Skip first node to avoid duplication
                flattened.insert(flattened.end(), segment.begin() + 1, segment.end());
            }
        }
        
        return flattened;
    }
    
public:
    TravelingSalesmanSolver(const Graph* environment) 
        : graph(environment), enableSymmetryOptimization(true), requireRoundTrip(true),
          maxIterationsGA(1000), mutationRate(0.05), coolingRate(0.995) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStarPlanner = std::make_unique<AStar>(graph);
        
        std::cout << "[TSP_SOLVER] Traveling Salesman Problem solver initialized" << std::endl;
    }
    
    std::vector<int> solveTSP(const std::vector<int>& nodes, int startNode, 
                            const std::string& algorithm = "auto") {
        std::cout << "[TSP_SOLVER] Solving TSP for " << nodes.size() << " nodes starting from " 
                  << startNode << " using " << algorithm << " algorithm" << std::endl;
        
        if (nodes.size() < 2) {
            return nodes;
        }
        
        computeDistanceMatrix(nodes);
        
        TSPSolution solution;
        
        if (algorithm == "auto") {
            if (nodes.size() <= 10) {
                solution = solveDynamicProgramming(nodes, startNode);
            } else if (nodes.size() <= 20) {
                solution = solveSimulatedAnnealing(nodes, startNode);
            } else {
                solution = solveGeneticAlgorithm(nodes, startNode);
            }
        } else if (algorithm == "nearest_neighbor") {
            solution = solveNearestNeighbor(nodes, startNode);
        } else if (algorithm == "genetic") {
            solution = solveGeneticAlgorithm(nodes, startNode);
        } else if (algorithm == "simulated_annealing") {
            solution = solveSimulatedAnnealing(nodes, startNode);
        } else if (algorithm == "dynamic_programming") {
            solution = solveDynamicProgramming(nodes, startNode);
        } else {
            solution = solveNearestNeighbor(nodes, startNode);
        }
        
        std::cout << "[TSP_SOLVER] TSP solved using " << solution.algorithm 
                  << " - Tour cost: " << solution.totalCost 
                  << ", Execution time: " << solution.executionTime << "s" << std::endl;
        
        return flattenPath(solution.detailedPath);
    }
    
    void setRoundTripRequired(bool required) {
        requireRoundTrip = required;
        std::cout << "[TSP_SOLVER] Round trip " << (required ? "required" : "not required") << std::endl;
    }
    
    void enableSymmetryOptimization(bool enable) {
        enableSymmetryOptimization = enable;
        std::cout << "[TSP_SOLVER] Symmetry optimization " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void setGeneticAlgorithmParameters(int maxIterations, double mutationRate) {
        maxIterationsGA = maxIterations;
        this->mutationRate = mutationRate;
        std::cout << "[TSP_SOLVER] GA parameters: " << maxIterations 
                  << " iterations, " << mutationRate << " mutation rate" << std::endl;
    }
    
    void setSimulatedAnnealingCoolingRate(double rate) {
        coolingRate = rate;
        std::cout << "[TSP_SOLVER] SA cooling rate set to " << rate << std::endl;
    }
    
    double estimateTourCost(const std::vector<int>& nodes, int startNode) {
        computeDistanceMatrix(nodes);
        TSPSolution estimate = solveNearestNeighbor(nodes, startNode);
        return estimate.totalCost;
    }
    
    void printSolverConfiguration() const {
        std::cout << "[TSP_SOLVER] Configuration:" << std::endl;
        std::cout << "[TSP_SOLVER]   Round trip required: " << requireRoundTrip << std::endl;
        std::cout << "[TSP_SOLVER]   Symmetry optimization: " << enableSymmetryOptimization << std::endl;
        std::cout << "[TSP_SOLVER]   GA max iterations: " << maxIterationsGA << std::endl;
        std::cout << "[TSP_SOLVER]   Mutation rate: " << mutationRate << std::endl;
        std::cout << "[TSP_SOLVER]   SA cooling rate: " << coolingRate << std::endl;
    }
};