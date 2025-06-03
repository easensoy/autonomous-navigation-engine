#include "autonomous_navigator/NavigationCore.hpp"
#include "autonomous_navigator/AlgorithmSelector.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"
#include <iostream>
#include <chrono>
#include <memory>

class AlgorithmExecutionManager {
private:
    const Graph* environment;
    std::unique_ptr<AlgorithmSelector> algorithmSelector;
    std::unordered_map<AlgorithmType, std::unique_ptr<void>> algorithmInstances;
    AlgorithmType currentAlgorithm;
    
    struct ExecutionContext {
        int startId;
        int goalId;
        std::chrono::steady_clock::time_point executionStart;
        std::vector<int> resultPath;
        AlgorithmPerformanceMetrics metrics;
        bool executionSuccessful;
        std::string errorMessage;
    };
    
    ExecutionContext lastExecution;
    bool executionLogging;
    double timeoutLimit;
    
public:
    explicit AlgorithmExecutionManager(const Graph* graph) 
        : environment(graph), currentAlgorithm(AlgorithmType::ASTAR),
          executionLogging(true), timeoutLimit(10.0) {
        
        initializeAlgorithmSelector();
        initializeAlgorithmInstances();
    }
    
    std::vector<int> executePathfinding(int startId, int goalId) {
        std::cout << "[ALGORITHM_EXEC] Executing pathfinding from " << startId 
                  << " to " << goalId << std::endl;
        
        // Initialize execution context
        lastExecution = ExecutionContext{};
        lastExecution.startId = startId;
        lastExecution.goalId = goalId;
        lastExecution.executionStart = std::chrono::steady_clock::now();
        
        // Select optimal algorithm for this pathfinding task
        AlgorithmType selectedAlgorithm = algorithmSelector->selectOptimalAlgorithm(startId, goalId);
        currentAlgorithm = selectedAlgorithm;
        
        std::cout << "[ALGORITHM_EXEC] Selected algorithm: " << static_cast<int>(selectedAlgorithm) << std::endl;
        
        // Execute the selected algorithm
        std::vector<int> path = executeSpecificAlgorithm(selectedAlgorithm, startId, goalId);
        
        // Record execution results
        recordExecutionResults(path, selectedAlgorithm);
        
        if (path.empty()) {
            std::cout << "[ALGORITHM_EXEC] Pathfinding failed, attempting fallback algorithm" << std::endl;
            path = executeFallbackAlgorithm(startId, goalId);
        }
        
        return path;
    }
    
    std::vector<int> executeSpecificAlgorithm(AlgorithmType algorithm, int startId, int goalId) {
        std::cout << "[ALGORITHM_EXEC] Executing " << getAlgorithmName(algorithm) 
                  << " algorithm" << std::endl;
        
        auto executionStart = std::chrono::steady_clock::now();
        std::vector<int> path;
        
        try {
            switch (algorithm) {
                case AlgorithmType::ASTAR:
                    path = executeAStarAlgorithm(startId, goalId);
                    break;
                case AlgorithmType::DIJKSTRA:
                    path = executeDijkstraAlgorithm(startId, goalId);
                    break;
                case AlgorithmType::BFS:
                    path = executeBFSAlgorithm(startId, goalId);
                    break;
                case AlgorithmType::BELLMAN_FORD:
                    path = executeBellmanFordAlgorithm(startId, goalId);
                    break;
                case AlgorithmType::AUTO_SELECT:
                    return executePathfinding(startId, goalId); // Recursive call with selection
                default:
                    std::cout << "[ALGORITHM_EXEC] Unsupported algorithm type, defaulting to A*" << std::endl;
                    path = executeAStarAlgorithm(startId, goalId);
                    break;
            }
            
            auto executionEnd = std::chrono::steady_clock::now();
            double executionTime = std::chrono::duration<double>(executionEnd - executionStart).count();
            
            if (executionLogging) {
                std::cout << "[ALGORITHM_EXEC] " << getAlgorithmName(algorithm) 
                          << " execution completed in " << executionTime << " seconds" << std::endl;
                std::cout << "[ALGORITHM_EXEC] Path length: " << path.size() << " waypoints" << std::endl;
            }
            
            // Update performance metrics
            updateAlgorithmMetrics(algorithm, path, executionTime);
            
        } catch (const std::exception& e) {
            std::cout << "[ALGORITHM_EXEC] Algorithm execution failed: " << e.what() << std::endl;
            lastExecution.errorMessage = e.what();
            lastExecution.executionSuccessful = false;
        }
        
        return path;
    }
    
    std::vector<int> executeFallbackAlgorithm(int startId, int goalId) {
        std::cout << "[ALGORITHM_EXEC] Executing fallback algorithm strategy" << std::endl;
        
        // Try algorithms in order of reliability
        std::vector<AlgorithmType> fallbackOrder = {
            AlgorithmType::DIJKSTRA,
            AlgorithmType::ASTAR,
            AlgorithmType::BFS,
            AlgorithmType::BELLMAN_FORD
        };
        
        for (AlgorithmType algorithm : fallbackOrder) {
            if (algorithm == currentAlgorithm) {
                continue; // Skip the algorithm that already failed
            }
            
            std::cout << "[ALGORITHM_EXEC] Attempting fallback with " 
                      << getAlgorithmName(algorithm) << std::endl;
            
            std::vector<int> path = executeSpecificAlgorithm(algorithm, startId, goalId);
            if (!path.empty()) {
                std::cout << "[ALGORITHM_EXEC] Fallback algorithm succeeded" << std::endl;
                return path;
            }
        }
        
        std::cout << "[ALGORITHM_EXEC] All fallback algorithms failed" << std::endl;
        return {};
    }
    
    bool validateAlgorithmExecution(const std::vector<int>& path, int startId, int goalId) {
        std::cout << "[ALGORITHM_EXEC] Validating algorithm execution results" << std::endl;
        
        if (path.empty()) {
            std::cout << "[ALGORITHM_EXEC] Validation failed: Empty path" << std::endl;
            return false;
        }
        
        // Check path connectivity
        if (path.front() != startId) {
            std::cout << "[ALGORITHM_EXEC] Validation failed: Path does not start at specified node" << std::endl;
            return false;
        }
        
        if (path.back() != goalId) {
            std::cout << "[ALGORITHM_EXEC] Validation failed: Path does not end at specified goal" << std::endl;
            return false;
        }
        
        // Validate path segments
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (!environment->hasEdge(path[i], path[i + 1])) {
                std::cout << "[ALGORITHM_EXEC] Validation failed: Invalid edge between nodes " 
                          << path[i] << " and " << path[i + 1] << std::endl;
                return false;
            }
        }
        
        std::cout << "[ALGORITHM_EXEC] Path validation successful" << std::endl;
        return true;
    }
    
    void setExecutionTimeout(double timeoutSeconds) {
        timeoutLimit = timeoutSeconds;
        std::cout << "[ALGORITHM_EXEC] Execution timeout set to " << timeoutSeconds << " seconds" << std::endl;
    }
    
    void enableExecutionLogging(bool enable) {
        executionLogging = enable;
        std::cout << "[ALGORITHM_EXEC] Execution logging " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    AlgorithmPerformanceMetrics getLastExecutionMetrics() const {
        return lastExecution.metrics;
    }
    
    void benchmarkAllAlgorithms(int startId, int goalId) {
        std::cout << "[ALGORITHM_EXEC] Benchmarking all algorithms for path " 
                  << startId << " -> " << goalId << std::endl;
        
        std::vector<AlgorithmType> algorithmsToTest = {
            AlgorithmType::ASTAR,
            AlgorithmType::DIJKSTRA,
            AlgorithmType::BFS,
            AlgorithmType::BELLMAN_FORD
        };
        
        for (AlgorithmType algorithm : algorithmsToTest) {
            std::cout << "[ALGORITHM_EXEC] Benchmarking " << getAlgorithmName(algorithm) << std::endl;
            
            auto benchmarkStart = std::chrono::steady_clock::now();
            std::vector<int> path = executeSpecificAlgorithm(algorithm, startId, goalId);
            auto benchmarkEnd = std::chrono::steady_clock::now();
            
            double benchmarkTime = std::chrono::duration<double>(benchmarkEnd - benchmarkStart).count();
            
            std::cout << "[ALGORITHM_EXEC] " << getAlgorithmName(algorithm) 
                      << " benchmark: " << benchmarkTime << "s, path length: " << path.size() << std::endl;
        }
        
        algorithmSelector->benchmarkAlgorithms(startId, goalId);
        std::cout << "[ALGORITHM_EXEC] Algorithm benchmarking completed" << std::endl;
    }
    
    void generateExecutionReport() const {
        std::cout << "\n[ALGORITHM_EXEC] === ALGORITHM EXECUTION REPORT ===" << std::endl;
        
        std::cout << "[ALGORITHM_EXEC] Last execution details:" << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Start node: " << lastExecution.startId << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Goal node: " << lastExecution.goalId << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Algorithm used: " << getAlgorithmName(currentAlgorithm) << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Execution time: " << lastExecution.metrics.executionTime << "s" << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Nodes explored: " << lastExecution.metrics.nodesExplored << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Path length: " << lastExecution.metrics.pathLength << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Memory usage: " << lastExecution.metrics.memoryUsage << " bytes" << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Optimality: " << lastExecution.metrics.optimality << std::endl;
        std::cout << "[ALGORITHM_EXEC]   Success: " << (lastExecution.executionSuccessful ? "Yes" : "No") << std::endl;
        
        if (!lastExecution.executionSuccessful && !lastExecution.errorMessage.empty()) {
            std::cout << "[ALGORITHM_EXEC]   Error: " << lastExecution.errorMessage << std::endl;
        }
        
        std::cout << "[ALGORITHM_EXEC] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeAlgorithmSelector() {
        algorithmSelector = std::make_unique<AlgorithmSelector>(environment);
        std::cout << "[ALGORITHM_EXEC] Algorithm selector initialized" << std::endl;
    }
    
    void initializeAlgorithmInstances() {
        std::cout << "[ALGORITHM_EXEC] Initializing algorithm instances" << std::endl;
        
        // Initialize algorithm instances for reuse
        try {
            algorithmInstances[AlgorithmType::ASTAR] = 
                std::unique_ptr<void>(static_cast<void*>(new AStar(environment)));
            algorithmInstances[AlgorithmType::DIJKSTRA] = 
                std::unique_ptr<void>(static_cast<void*>(new Dijkstra(environment)));
            algorithmInstances[AlgorithmType::BFS] = 
                std::unique_ptr<void>(static_cast<void*>(new BFS(environment)));
            algorithmInstances[AlgorithmType::BELLMAN_FORD] = 
                std::unique_ptr<void>(static_cast<void*>(new BellmanFord(environment)));
            
            std::cout << "[ALGORITHM_EXEC] Algorithm instances initialized successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cout << "[ALGORITHM_EXEC] Failed to initialize algorithm instances: " << e.what() << std::endl;
        }
    }
    
    std::vector<int> executeAStarAlgorithm(int startId, int goalId) {
        auto astarInstance = static_cast<AStar*>(algorithmInstances[AlgorithmType::ASTAR].get());
        if (!astarInstance) {
            astarInstance = new AStar(environment);
        }
        
        return astarInstance->findPath(startId, goalId);
    }
    
    std::vector<int> executeDijkstraAlgorithm(int startId, int goalId) {
        auto dijkstraInstance = static_cast<Dijkstra*>(algorithmInstances[AlgorithmType::DIJKSTRA].get());
        if (!dijkstraInstance) {
            dijkstraInstance = new Dijkstra(environment);
        }
        
        return dijkstraInstance->findShortestPath(startId, goalId);
    }
    
    std::vector<int> executeBFSAlgorithm(int startId, int goalId) {
        auto bfsInstance = static_cast<BFS*>(algorithmInstances[AlgorithmType::BFS].get());
        if (!bfsInstance) {
            bfsInstance = new BFS(environment);
        }
        
        return bfsInstance->findPath(startId, goalId);
    }
    
    std::vector<int> executeBellmanFordAlgorithm(int startId, int goalId) {
        auto bellmanFordInstance = static_cast<BellmanFord*>(algorithmInstances[AlgorithmType::BELLMAN_FORD].get());
        if (!bellmanFordInstance) {
            bellmanFordInstance = new BellmanFord(environment);
        }
        
        return bellmanFordInstance->findShortestPath(startId, goalId);
    }
    
    void updateAlgorithmMetrics(AlgorithmType algorithm, const std::vector<int>& path, double executionTime) {
        AlgorithmPerformanceMetrics metrics;
        metrics.executionTime = executionTime;
        metrics.nodesExplored = estimateNodesExplored(algorithm, path);
        metrics.pathLength = calculatePathLength(path);
        metrics.memoryUsage = estimateMemoryUsage(algorithm, path);
        metrics.optimality = calculatePathOptimality(path);
        
        lastExecution.metrics = metrics;
        lastExecution.executionSuccessful = !path.empty();
        lastExecution.resultPath = path;
        
        // Update algorithm selector with performance data
        algorithmSelector->updatePerformanceMetrics(algorithm, metrics);
    }
    
    void recordExecutionResults(const std::vector<int>& path, AlgorithmType algorithm) {
        auto executionEnd = std::chrono::steady_clock::now();
        double totalExecutionTime = std::chrono::duration<double>(
            executionEnd - lastExecution.executionStart).count();
        
        if (executionLogging) {
            std::cout << "[ALGORITHM_EXEC] Execution summary:" << std::endl;
            std::cout << "[ALGORITHM_EXEC]   Algorithm: " << getAlgorithmName(algorithm) << std::endl;
            std::cout << "[ALGORITHM_EXEC]   Total time: " << totalExecutionTime << "s" << std::endl;
            std::cout << "[ALGORITHM_EXEC]   Path found: " << (!path.empty() ? "Yes" : "No") << std::endl;
            
            if (!path.empty()) {
                std::cout << "[ALGORITHM_EXEC]   Path validation: " 
                          << (validateAlgorithmExecution(path, lastExecution.startId, lastExecution.goalId) ? 
                              "Passed" : "Failed") << std::endl;
            }
        }
    }
    
    std::string getAlgorithmName(AlgorithmType algorithm) const {
        switch (algorithm) {
            case AlgorithmType::ASTAR: return "A*";
            case AlgorithmType::DIJKSTRA: return "Dijkstra";
            case AlgorithmType::BFS: return "BFS";
            case AlgorithmType::DFS: return "DFS";
            case AlgorithmType::BELLMAN_FORD: return "Bellman-Ford";
            case AlgorithmType::JUMP_POINT_SEARCH: return "Jump Point Search";
            case AlgorithmType::AUTO_SELECT: return "Auto-Select";
            default: return "Unknown";
        }
    }
    
    size_t estimateNodesExplored(AlgorithmType algorithm, const std::vector<int>& path) const {
        // Estimate based on algorithm characteristics and path length
        size_t pathLength = path.size();
        
        switch (algorithm) {
            case AlgorithmType::ASTAR:
                return pathLength * 2; // A* typically explores fewer nodes
            case AlgorithmType::DIJKSTRA:
                return pathLength * 4; // Dijkstra explores more comprehensively
            case AlgorithmType::BFS:
                return pathLength * 6; // BFS explores level by level
            case AlgorithmType::BELLMAN_FORD:
                return environment->getNodeCount(); // Processes all nodes
            default:
                return pathLength * 3; // Default estimate
        }
    }
    
    double calculatePathLength(const std::vector<int>& path) const {
        if (path.size() < 2) {
            return 0.0;
        }
        
        double totalLength = 0.0;
        for (size_t i = 0; i < path.size() - 1; ++i) {
            double edgeWeight = environment->getEdgeWeight(path[i], path[i + 1]);
            totalLength += (edgeWeight > 0) ? edgeWeight : 1.0; // Default weight if not found
        }
        
        return totalLength;
    }
    
    size_t estimateMemoryUsage(AlgorithmType algorithm, const std::vector<int>& path) const {
        // Estimate memory usage based on algorithm and path complexity
        size_t baseMemory = sizeof(path) + path.size() * sizeof(int);
        
        switch (algorithm) {
            case AlgorithmType::ASTAR:
                return baseMemory + environment->getNodeCount() * 64; // Open/closed lists
            case AlgorithmType::DIJKSTRA:
                return baseMemory + environment->getNodeCount() * 48; // Distance arrays
            case AlgorithmType::BFS:
                return baseMemory + environment->getNodeCount() * 32; // Queue and visited set
            case AlgorithmType::BELLMAN_FORD:
                return baseMemory + environment->getNodeCount() * 72; // Distance and predecessor arrays
            default:
                return baseMemory + environment->getNodeCount() * 40; // Default estimate
        }
    }
    
    double calculatePathOptimality(const std::vector<int>& path) const {
        if (path.size() < 2) {
            return 0.0;
        }
        
        // Calculate optimality as ratio of actual path length to theoretical minimum
        double actualLength = calculatePathLength(path);
        
        // Estimate theoretical minimum (straight-line distance)
        const Node& startNode = environment->getNode(path.front());
        const Node& goalNode = environment->getNode(path.back());
        double theoreticalMinimum = startNode.euclideanDistance(goalNode);
        
        if (theoreticalMinimum <= 0) {
            return 1.0; // Perfect optimality for zero distance
        }
        
        double optimality = theoreticalMinimum / actualLength;
        return std::min(1.0, optimality); // Cap at 1.0 for cases where path is optimal
    }
};