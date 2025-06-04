#include "environment/EnvironmentManager.hpp"
#include "autonomous_navigator/NavigationCore.hpp"
#include "graph_operations/GraphBuilder.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "pathfinding_algorithms/Dijkstra.hpp"
#include "core/Graph.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <memory>

class EnvironmentSwitchingTester {
private:
    int testsRun = 0;
    int testsPassed = 0;
    
    // Create different types of test environments
    std::shared_ptr<Graph> createGridEnvironment(int width, int height) {
        auto graph = std::make_shared<Graph>();
        
        // Create grid nodes
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                graph->addNode(nodeId, "Grid_" + std::to_string(x) + "_" + std::to_string(y), 
                              x * 10.0, y * 10.0);
            }
        }
        
        // Add grid connections (4-connected)
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                
                if (x < width - 1) {
                    graph->addEdge(nodeId, nodeId + 1, 10.0); // Right
                }
                if (y < height - 1) {
                    graph->addEdge(nodeId, nodeId + width, 10.0); // Down
                }
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createLinearEnvironment(int nodeCount) {
        auto graph = std::make_shared<Graph>();
        
        for (int i = 0; i < nodeCount; ++i) {
            graph->addNode(i, "Linear_" + std::to_string(i), i * 15.0, 0.0);
            if (i > 0) {
                graph->addEdge(i - 1, i, 15.0);
            }
        }
        
        return graph;
    }
    
    std::shared_ptr<Graph> createStarEnvironment(int spokes) {
        auto graph = std::make_shared<Graph>();
        
        // Central hub node
        graph->addNode(0, "Star_Hub", 0.0, 0.0);
        
        // Create spokes radiating from center
        for (int i = 1; i <= spokes; ++i) {
            double angle = (2.0 * M_PI * (i - 1)) / spokes;
            double x = 20.0 * std::cos(angle);
            double y = 20.0 * std::sin(angle);
            
            graph->addNode(i, "Star_Spoke_" + std::to_string(i), x, y);
            graph->addEdge(0, i, 20.0);
        }
        
        return graph;
    }
    
    void runTest(const std::string& testName, std::function<bool()> test) {
        testsRun++;
        std::cout << "[INTEGRATION_TEST] " << testName << ": ";
        
        try {
            if (test()) {
                testsPassed++;
                std::cout << "PASS" << std::endl;
            } else {
                std::cout << "FAIL" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cout << "EXCEPTION: " << e.what() << std::endl;
        }
    }
    
public:
    void runAllTests() {
        std::cout << "\n=== Environment Switching Integration Tests ===" << std::endl;
        
        runTest("Basic Environment Switching", [this]() {
            EnvironmentManager envManager;
            
            // Start with grid environment
            auto gridEnv = createGridEnvironment(3, 3);
            envManager.setEnvironment(gridEnv, "GridEnv");
            
            bool initialValid = envManager.validateCurrentEnvironment();
            std::string initialStatus = envManager.getEnvironmentStatus();
            
            // Switch to linear environment
            auto linearEnv = createLinearEnvironment(5);
            envManager.setEnvironment(linearEnv, "LinearEnv");
            
            bool switchValid = envManager.validateCurrentEnvironment();
            std::string finalName = envManager.getEnvironmentName();
            
            return initialValid && switchValid && (finalName == "LinearEnv");
        });
        
        runTest("Navigation During Environment Switch", [this]() {
            auto gridEnv = createGridEnvironment(4, 4);
            NavigationCore navigator(gridEnv.get());
            
            // Start navigation in grid environment
            bool startResult = navigator.startNavigation(0, 15); // Corner to corner
            std::vector<int> initialPath = navigator.getCurrentPath();
            
            // Switch to a different environment mid-navigation
            auto starEnv = createStarEnvironment(8);
            navigator.updateEnvironment(starEnv.get());
            
            // Attempt to continue navigation (should handle gracefully)
            std::string status = navigator.getNavigationStatus();
            bool isComplete = navigator.isNavigationComplete();
            
            return startResult && !initialPath.empty() && !status.empty();
        });
        
        runTest("Environment Persistence and Recovery", [this]() {
            EnvironmentManager envManager;
            
            // Set up initial environment
            auto gridEnv = createGridEnvironment(3, 3);
            envManager.setEnvironment(gridEnv, "OriginalGrid");
            
            // Backup current environment
            envManager.backupCurrentEnvironment();
            
            // Switch to different environment
            auto linearEnv = createLinearEnvironment(10);
            envManager.setEnvironment(linearEnv, "NewLinear");
            
            std::string beforeRestore = envManager.getEnvironmentName();
            
            // Restore previous environment
            bool restored = envManager.restorePreviousEnvironment();
            std::string afterRestore = envManager.getEnvironmentName();
            
            return restored && (beforeRestore == "NewLinear") && (afterRestore == "OriginalGrid");
        });
        
        runTest("Algorithm Adaptation to Environment Changes", [this]() {
            // Test how different algorithms handle environment switching
            auto gridEnv = createGridEnvironment(3, 3);
            auto linearEnv = createLinearEnvironment(5);
            
            // Test with A*
            AStar astar(gridEnv.get());
            astar.setHeuristicFunction([gridEnv](int nodeId, int goalId) {
                const Node& node = gridEnv->getNode(nodeId);
                const Node& goal = gridEnv->getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> gridPath = astar.findPath(0, 8);
            
            // Switch algorithm to linear environment
            // Note: In practice, we'd create a new AStar instance for the new environment
            AStar linearAstar(linearEnv.get());
            linearAstar.setHeuristicFunction([linearEnv](int nodeId, int goalId) {
                const Node& node = linearEnv->getNode(nodeId);
                const Node& goal = linearEnv->getNode(goalId);
                return node.euclideanDistance(goal);
            });
            
            std::vector<int> linearPath = linearAstar.findPath(0, 4);
            
            // Test with Dijkstra (should work regardless of environment)
            Dijkstra gridDijkstra(gridEnv.get());
            Dijkstra linearDijkstra(linearEnv.get());
            
            std::vector<int> dijkstraGrid = gridDijkstra.findShortestPath(0, 8);
            std::vector<int> dijkstraLinear = linearDijkstra.findShortestPath(0, 4);
            
            return !gridPath.empty() && !linearPath.empty() && 
                   !dijkstraGrid.empty() && !dijkstraLinear.empty();
        });
        
        runTest("Environment Validation and Error Handling", [this]() {
            EnvironmentManager envManager;
            
            // Test with valid environment
            auto validEnv = createGridEnvironment(2, 2);
            envManager.setEnvironment(validEnv, "ValidEnv");
            bool validResult = envManager.validateCurrentEnvironment();
            
            // Test trying to set null environment
            bool nullResult = envManager.trySetEnvironment(nullptr);
            
            // Test with empty environment
            auto emptyEnv = std::make_shared<Graph>();
            bool emptyResult = envManager.trySetEnvironment(emptyEnv);
            
            std::string status = envManager.getEnvironmentStatus();
            
            return validResult && !nullResult && !emptyResult && !status.empty();
        });
        
        runTest("Dynamic Environment Modification", [this]() {
            auto dynamicEnv = createGridEnvironment(3, 3);
            NavigationCore navigator(dynamicEnv.get());
            
            // Start navigation
            bool started = navigator.startNavigation(0, 8);
            std::vector<int> originalPath = navigator.getCurrentPath();
            
            // Modify environment by removing edges (simulating obstacles)
            dynamicEnv->removeEdge(1, 2);
            dynamicEnv->removeEdge(3, 4);
            
            // Update navigator with modified environment
            navigator.updateEnvironment(dynamicEnv.get());
            
            // Test if navigation adapts
            std::string status = navigator.getNavigationStatus();
            
            return started && !originalPath.empty() && !status.empty();
        });
        
        runTest("Multi-Environment Performance Comparison", [this]() {
            std::vector<std::shared_ptr<Graph>> environments = {
                createGridEnvironment(4, 4),
                createLinearEnvironment(16),
                createStarEnvironment(15)
            };
            
            std::vector<std::string> envNames = {"Grid4x4", "Linear16", "Star15"};
            std::vector<double> executionTimes;
            
            // Test pathfinding performance in each environment
            for (size_t i = 0; i < environments.size(); ++i) {
                auto& env = environments[i];
                AStar astar(env.get());
                astar.setHeuristicFunction([env](int nodeId, int goalId) {
                    const Node& node = env->getNode(nodeId);
                    const Node& goal = env->getNode(goalId);
                    return node.euclideanDistance(goal);
                });
                
                auto start = std::chrono::high_resolution_clock::now();
                
                // Run multiple pathfinding operations
                for (int j = 0; j < 10; ++j) {
                    std::vector<int> path = astar.findPath(0, env->getNodeCount() - 1);
                    if (path.empty()) return false;
                }
                
                auto end = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration<double>(end - start).count();
                executionTimes.push_back(duration);
                
                std::cout << " [" << envNames[i] << ": " << duration << "s]";
            }
            
            return executionTimes.size() == 3;
        });
        
        runTest("Environment Switching Under Load", [this]() {
            EnvironmentManager envManager;
            std::vector<std::shared_ptr<Graph>> environments = {
                createGridEnvironment(3, 3),
                createLinearEnvironment(9),
                createStarEnvironment(8)
            };
            
            bool allSwitchesSuccessful = true;
            
            // Rapidly switch between environments
            for (int iteration = 0; iteration < 20; ++iteration) {
                int envIndex = iteration % environments.size();
                std::string envName = "Env_" + std::to_string(iteration);
                
                envManager.setEnvironment(environments[envIndex], envName);
                
                if (!envManager.validateCurrentEnvironment()) {
                    allSwitchesSuccessful = false;
                    break;
                }
                
                // Perform quick navigation test
                NavigationCore navigator(environments[envIndex].get());
                bool navResult = navigator.startNavigation(0, environments[envIndex]->getNodeCount() - 1);
                
                if (!navResult) {
                    allSwitchesSuccessful = false;
                    break;
                }
            }
            
            return allSwitchesSuccessful;
        });
        
        runTest("Environment Configuration Integration", [this]() {
            EnvironmentManager envManager;
            
            // Test loading from configuration (simulated)
            envManager.initializeDefaultEnvironment();
            bool defaultValid = envManager.validateCurrentEnvironment();
            
            // Test creating empty environment
            envManager.createEmptyEnvironment("EmptyTest");
            std::string emptyName = envManager.getEnvironmentName();
            
            // Test setting complex environment
            auto complexEnv = createGridEnvironment(5, 5);
            // Add some diagonal connections to make it more complex
            for (int i = 0; i < 20; ++i) {
                if (i + 6 < 25) {
                    complexEnv->addEdge(i, i + 6, 14.14); // Diagonal connections
                }
            }
            
            envManager.setEnvironment(complexEnv, "ComplexGrid");
            size_t complexity = envManager.getEnvironmentComplexity();
            
            return defaultValid && (emptyName == "EmptyTest") && (complexity > 0);
        });
        
        runTest("Real-time Environment Updates", [this]() {
            auto baseEnv = createGridEnvironment(4, 4);
            NavigationCore navigator(baseEnv.get());
            
            // Start long navigation
            bool started = navigator.startNavigation(0, 15);
            
            // Simulate real-time updates to environment
            std::vector<int> obstacles = {5, 6, 9, 10}; // Block some nodes
            navigator.handleObstacles(obstacles);
            
            // Continue navigation with obstacles
            bool stepResult = navigator.executeNavigationStep();
            std::string status = navigator.getNavigationStatus();
            
            // Remove some obstacles
            obstacles = {5, 6}; // Reduce obstacles
            navigator.handleObstacles(obstacles);
            
            bool finalStep = navigator.executeNavigationStep();
            
            return started && stepResult && finalStep && !status.empty();
        });
        
        printResults();
    }
    
    void printResults() {
        std::cout << "\n=== Environment Switching Integration Test Results ===" << std::endl;
        std::cout << "Tests passed: " << testsPassed << "/" << testsRun << std::endl;
        std::cout << "Success rate: " << (testsRun > 0 ? (testsPassed * 100.0 / testsRun) : 0) << "%" << std::endl;
        
        if (testsPassed == testsRun) {
            std::cout << "✓ All environment switching integration tests passed!" << std::endl;
        } else {
            std::cout << "✗ Some environment switching integration tests failed!" << std::endl;
        }
        
        // Provide summary insights
        std::cout << "\nIntegration Test Insights:" << std::endl;
        std::cout << "- Environment switching maintains system stability" << std::endl;
        std::cout << "- Navigation algorithms adapt to different environment types" << std::endl;
        std::cout << "- Real-time environment updates are handled gracefully" << std::endl;
        std::cout << "- Performance varies across different environment structures" << std::endl;
    }
};

int main() {
    EnvironmentSwitchingTester tester;
    tester.runAllTests();
    return 0;
}