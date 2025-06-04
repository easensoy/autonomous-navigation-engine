#include <iostream>
#include <memory>
#include <vector>
#include <chrono>

// Core components
#include "core/Graph.hpp"
#include "core/Node.hpp"
#include "core/Edge.hpp"

// Pathfinding algorithms
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "pathfinding_algorithms/BFS.hpp"
#include "pathfinding_algorithms/BellmanFord.hpp"

// Navigation system
#include "autonomous_navigator/NavigationCore.hpp"
#include "autonomous_navigator/AlgorithmSelector.hpp"
#include "autonomous_navigator/PathExecutor.hpp"

// Navigation strategies
#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "navigation_strategies/LocalPathPlanner.hpp"
#include "navigation_strategies/DynamicReplanner.hpp"
#include "navigation_strategies/MultiGoalPlanner.hpp"

// Path operations
#include "path_operations/PathOptimizer.hpp"
#include "path_operations/PathValidator.hpp"
#include "path_operations/PathSmoother.hpp"
#include "path_operations/CostCalculator.hpp"

// Environment management
#include "environment/EnvironmentManager.hpp"
#include "environment/MapUpdater.hpp"
#include "environment/ObstacleDetector.hpp"

// Graph operations
#include "graph_operations/GraphBuilder.hpp"
#include "graph_operations/NodeManager.hpp"
#include "graph_operations/EdgeManager.hpp"
#include "graph_operations/ConnectivityAnalyzer.hpp"

// Utilities
#include "utilities/ConfigManager.hpp"
#include "utilities/MathUtils.hpp"
#include "utilities/PriorityQueue.hpp"

// Reporting and visualization
#include "reporting/NavigationLogger.hpp"
#include "reporting/PerformanceProfiler.hpp"
#include "reporting/PathVisualiser.hpp"

void createSampleGraph(Graph& graph) {
    std::cout << "\n=== Creating Sample Graph ===" << std::endl;
    
    // Add nodes in a grid pattern
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            int nodeId = i * 5 + j;
            std::string nodeName = "Node_" + std::to_string(nodeId);
            graph.addNode(nodeId, nodeName, i * 10.0, j * 10.0);
        }
    }
    
    // Add edges (4-connected grid)
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            int nodeId = i * 5 + j;
            
            // Right neighbor
            if (j < 4) {
                int rightNeighbor = i * 5 + (j + 1);
                graph.addEdge(nodeId, rightNeighbor, 10.0);
            }
            
            // Bottom neighbor
            if (i < 4) {
                int bottomNeighbor = (i + 1) * 5 + j;
                graph.addEdge(nodeId, bottomNeighbor, 10.0);
            }
        }
    }
    
    std::cout << "Created graph with " << graph.getNodeCount() << " nodes and " 
              << graph.getEdgeCount() << " edges" << std::endl;
}

void demonstratePathfinding(const Graph& graph) {
    std::cout << "\n=== Pathfinding Algorithm Comparison ===" << std::endl;
    
    int startId = 0;  // Top-left corner
    int goalId = 24;  // Bottom-right corner
    
    // A* Algorithm
    {
        AStar astar(&graph);
        astar.setHeuristicFunction([&graph](int nodeId, int goalId) {
            const Node& node = graph.getNode(nodeId);
            const Node& goal = graph.getNode(goalId);
            return node.euclideanDistance(goal);
        });
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<int> path = astar.findPath(startId, goalId);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "A*: Found path with " << path.size() << " nodes in " 
                  << duration.count() << " μs" << std::endl;
    }
    
    // Dijkstra Algorithm
    {
        Dijkstra dijkstra(&graph);
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<int> path = dijkstra.findShortestPath(startId, goalId);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "Dijkstra: Found path with " << path.size() << " nodes in " 
                  << duration.count() << " μs" << std::endl;
    }
    
    // BFS Algorithm
    {
        BFS bfs(&graph);
        
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<int> path = bfs.findPath(startId, goalId);
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
        std::cout << "BFS: Found path with " << path.size() << " nodes in " 
                  << duration.count() << " μs" << std::endl;
    }
}

void demonstrateNavigationSystem(const Graph& graph) {
    std::cout << "\n=== Navigation System Demo ===" << std::endl;
    
    NavigationCore navigator(&graph);
    
    int startId = 0;
    int goalId = 24;
    
    // Start navigation
    if (navigator.startNavigation(startId, goalId)) {
        std::cout << "Navigation started successfully" << std::endl;
        
        // Simulate navigation steps
        int steps = 0;
        while (!navigator.isNavigationComplete() && steps < 50) {
            if (navigator.executeNavigationStep()) {
                std::cout << "Navigation step " << steps << " completed" << std::endl;
                steps++;
            } else {
                break;
            }
        }
        
        std::cout << "Navigation status: " << navigator.getNavigationStatus() << std::endl;
        std::cout << "Current path length: " << navigator.getCurrentPath().size() << " nodes" << std::endl;
    }
}

void demonstratePathOperations(const Graph& graph) {
    std::cout << "\n=== Path Operations Demo ===" << std::endl;
    
    // Find a path to work with
    AStar astar(&graph);
    astar.setHeuristicFunction([&graph](int nodeId, int goalId) {
        const Node& node = graph.getNode(nodeId);
        const Node& goal = graph.getNode(goalId);
        return node.euclideanDistance(goal);
    });
    
    std::vector<int> originalPath = astar.findPath(0, 24);
    std::cout << "Original path length: " << originalPath.size() << " nodes" << std::endl;
    
    // Path validation
    PathValidator validator(&graph);
    bool isValid = validator.validatePath(originalPath);
    std::cout << "Path validation: " << (isValid ? "VALID" : "INVALID") << std::endl;
    
    // Path optimization
    PathOptimizer optimizer(&graph);
    std::vector<int> optimizedPath = optimizer.optimizePath(originalPath);
    std::cout << "Optimized path length: " << optimizedPath.size() << " nodes" << std::endl;
    
    // Path smoothing
    PathSmoother smoother(&graph);
    std::vector<int> smoothedPath = smoother.smoothPath(originalPath);
    std::cout << "Smoothed path length: " << smoothedPath.size() << " nodes" << std::endl;
    
    // Cost calculation
    CostCalculator calculator(&graph);
    double pathCost = calculator.calculatePathCost(originalPath);
    std::cout << "Path cost: " << pathCost << std::endl;
}

void demonstrateEnvironmentManagement(std::shared_ptr<Graph> graph) {
    std::cout << "\n=== Environment Management Demo ===" << std::endl;
    
    EnvironmentManager envManager(graph);
    std::cout << "Environment status: " << envManager.getEnvironmentStatus() << std::endl;
    std::cout << "Environment complexity: " << envManager.getEnvironmentComplexity() << std::endl;
    
    // Map updating
    MapUpdater mapUpdater(graph.get());
    mapUpdater.addNode(25, "NewNode", 50.0, 50.0);
    mapUpdater.updateEdgeWeight(0, 1, 15.0);
    std::cout << "Map updated with new node and edge weight" << std::endl;
    
    // Obstacle detection
    ObstacleDetector obstacleDetector(graph.get());
    obstacleDetector.addStaticObstacle(12); // Block center node
    std::vector<int> blockedNodes = obstacleDetector.getBlockedNodes();
    std::cout << "Detected " << blockedNodes.size() << " blocked nodes" << std::endl;
}

void demonstrateGraphOperations(Graph& graph) {
    std::cout << "\n=== Graph Operations Demo ===" << std::endl;
    
    // Node management
    NodeManager nodeManager(&graph);
    int newNodeId = nodeManager.addNode("DynamicNode", 60.0, 60.0);
    std::cout << "Added new node with ID: " << newNodeId << std::endl;
    
    std::vector<int> isolatedNodes = nodeManager.getIsolatedNodes();
    std::cout << "Found " << isolatedNodes.size() << " isolated nodes" << std::endl;
    
    // Edge management
    EdgeManager edgeManager(&graph);
    edgeManager.addEdge(24, newNodeId, 20.0);
    std::cout << "Added edge to new node" << std::endl;
    
    // Connectivity analysis
    ConnectivityAnalyzer analyzer(&graph);
    ConnectivityMetrics metrics = analyzer.analyzeConnectivity();
    std::cout << "Graph connectivity:" << std::endl;
    std::cout << "  Connected: " << (metrics.isConnected ? "Yes" : "No") << std::endl;
    std::cout << "  Components: " << metrics.componentCount << std::endl;
    std::cout << "  Largest component: " << metrics.largestComponentSize << " nodes" << std::endl;
}

void demonstrateConfigurationAndReporting() {
    std::cout << "\n=== Configuration and Reporting Demo ===" << std::endl;
    
    // Configuration management
    ConfigManager config;
    config.setValue("algorithm.type", std::string("astar"));
    config.setValue("performance.max_iterations", 1000);
    config.setValue("visualization.enabled", true);
    
    std::cout << "Algorithm type: " << config.getValue<std::string>("algorithm.type", "unknown") << std::endl;
    std::cout << "Max iterations: " << config.getValue<int>("performance.max_iterations", 0) << std::endl;
    
    // Navigation logging
    NavigationLogger logger("navigation.log");
    logger.info("Navigation system initialized");
    logger.debug("Starting pathfinding operation");
    logger.warning("Obstacle detected in path");
    
    // Performance profiling
    PerformanceProfiler profiler;
    profiler.startProfiling("demo_operation");
    
    // Simulate some work
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    
    profiler.stopProfiling();
    PerformanceMetrics metrics = profiler.getMetrics("demo_operation");
    std::cout << "Demo operation took: " << metrics.executionTime.count() << " seconds" << std::endl;
}

void demonstrateVisualization(const Graph& graph) {
    std::cout << "\n=== Visualization Demo ===" << std::endl;
    
    PathVisualizer visualizer(&graph);
    
    // Find a path to visualize
    AStar astar(&graph);
    astar.setHeuristicFunction([&graph](int nodeId, int goalId) {
        const Node& node = graph.getNode(nodeId);
        const Node& goal = graph.getNode(goalId);
        return node.euclideanDistance(goal);
    });
    
    std::vector<int> path = astar.findPath(0, 24);
    
    // Console visualization
    visualizer.visualizePath(path, VisualizationFormat::CONSOLE_TEXT);
    
    // Generate path statistics
    std::string stats = visualizer.generatePathStatistics(path);
    std::cout << "Path Statistics:\n" << stats << std::endl;
}

void demonstrateAdvancedFeatures(const Graph& graph) {
    std::cout << "\n=== Advanced Features Demo ===" << std::endl;
    
    // Multi-goal planning
    MultiGoalPlanner multiPlanner(&graph);
    std::vector<int> goals = {6, 12, 18, 24};
    std::vector<int> tour = multiPlanner.planOptimalTour(0, goals);
    std::cout << "Multi-goal tour length: " << tour.size() << " nodes" << std::endl;
    
    // Algorithm selection
    AlgorithmSelector selector(&graph);
    AlgorithmType optimal = selector.selectOptimalAlgorithm(0, 24);
    std::cout << "Selected optimal algorithm for this graph" << std::endl;
    
    // Dynamic replanning
    DynamicReplanner replanner(&graph);
    std::vector<int> initialPath = {0, 1, 2, 7, 12, 17, 22, 23, 24};
    replanner.setInitialPath(initialPath);
    
    std::vector<int> obstacles = {12}; // Obstacle in middle of path
    std::vector<int> newPath = replanner.updateAndReplan(2, 24, obstacles);
    std::cout << "Replanned path length: " << newPath.size() << " nodes" << std::endl;
}

int main() {
    std::cout << "=== Navigation System Demonstration ===" << std::endl;
    std::cout << "Comprehensive pathfinding and navigation framework" << std::endl;
    
    try {
        // Create the main graph
        auto graph = std::make_shared<Graph>();
        createSampleGraph(*graph);
        
        // Core pathfinding demonstrations
        demonstratePathfinding(*graph);
        demonstrateNavigationSystem(*graph);
        demonstratePathOperations(*graph);
        
        // Environment and graph management
        demonstrateEnvironmentManagement(graph);
        demonstrateGraphOperations(*graph);
        
        // Advanced features
        demonstrateAdvancedFeatures(*graph);
        
        // Configuration and reporting
        demonstrateConfigurationAndReporting();
        
        // Visualization
        demonstrateVisualization(*graph);
        
        std::cout << "\n=== Demo Complete ===" << std::endl;
        std::cout << "All navigation system components demonstrated successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error during demonstration: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}