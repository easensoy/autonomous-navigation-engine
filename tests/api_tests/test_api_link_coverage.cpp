#include "autonomous_navigator/NavigationCore.hpp"
#include "core/Graph.hpp"
#include "environment/MapUpdater.hpp"
#include "environment/ObstacleDetector.hpp"
#include "graph_operations/ConnectivityAnalyzer.hpp"
#include "graph_operations/EdgeManager.hpp"
#include "graph_operations/GraphBuilder.hpp"
#include "graph_operations/NodeManager.hpp"
#include "navigation_strategies/MultiGoalPlanner.hpp"
#include "path_operations/CostCalculator.hpp"
#include "path_operations/PathOptimizer.hpp"
#include "path_operations/PathSmoother.hpp"
#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include "reporting/PathVisualiser.hpp"
#include "reporting/PerformanceProfiler.hpp"
#include "utilities/ConfigManager.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

// Link coverage for the classes whose public API changed in November 2026 (see CHANGELOG.md): this
// program takes the address of every public member function and constructor of those classes. If
// any of them is declared but not defined, this test fails to link, so the build fails. Running it
// only prints how many members were referenced.

namespace {

// Folds each member pointer's bytes into a checksum that main prints, so that the optimiser has to
// keep every reference.
struct References {
    std::uint64_t checksum = 1469598103934665603ULL;
    int count = 0;

    template <typename MemberPointer>
    References& add(MemberPointer pointer) {
        unsigned char bytes[sizeof(MemberPointer)];
        std::memcpy(bytes, &pointer, sizeof(MemberPointer));
        for (unsigned char byte : bytes) {
            checksum = (checksum ^ byte) * 1099511628211ULL;
        }
        ++count;
        return *this;
    }
};

// Never called: its address escapes, so it is compiled and linked, which references every
// constructor and destructor below.
void constructEveryClass() {
    Graph graph;
    GraphBuilder builder(&graph);
    NodeManager nodes(&graph);
    EdgeManager edges(&graph);
    ConnectivityAnalyzer connectivity(&graph);
    MapUpdater updater(&graph);
    ObstacleDetector detector(&graph);
    MultiGoalPlanner tours(&graph);
    CostCalculator costs(&graph);
    PathOptimizer optimizer(&graph);
    PathSmoother smoother(&graph);
    PathVisualizer visualizer(&graph);
    PerformanceProfiler profiler;
    ConfigManager defaults;
    ConfigManager fromFile("unused.cfg");
    JumpPointSearch jps(&graph, 1, 1);
    NavigationCore navigator(&graph);
}

volatile auto keepConstructors = &constructEveryClass;

}  // namespace

int main() {
    References r;
    r.add(&GraphBuilder::buildGraph)
        .add(&GraphBuilder::addNodesFromCoordinates)
        .add(&GraphBuilder::connectNearestNeighbors)
        .add(&GraphBuilder::connectWithinRadius)
        .add(&GraphBuilder::generateMazeLayout);

    r.add(&NodeManager::addNode)
        .add(&NodeManager::addNodeWithId)
        .add(&NodeManager::removeNode)
        .add(&NodeManager::moveNode)
        .add(&NodeManager::renameNode)
        .add(&NodeManager::findNodesInRadius)
        .add(&NodeManager::findNearestNode)
        .add(&NodeManager::getIsolatedNodes)
        .add(&NodeManager::getHighDegreeNodes)
        .add(&NodeManager::clusterNodes)
        .add(&NodeManager::getNodeCount)
        .add(&NodeManager::getAllNodeIds)
        .add(&NodeManager::validateNodeConsistency);

    r.add(&EdgeManager::addEdge)
        .add(&EdgeManager::removeEdge)
        .add(&EdgeManager::updateEdgeWeight)
        .add(&EdgeManager::findShortestEdges)
        .add(&EdgeManager::findLongestEdges)
        .add(&EdgeManager::getTotalEdgeWeight)
        .add(&EdgeManager::getAverageEdgeWeight)
        .add(&EdgeManager::getHeaviestEdge)
        .add(&EdgeManager::getMinimumSpanningTree)
        .add(&EdgeManager::getEdgeCount)
        .add(&EdgeManager::validateEdgeConsistency);

    r.add(&ConnectivityAnalyzer::analyzeConnectivity)
        .add(&ConnectivityAnalyzer::isGraphConnected)
        .add(&ConnectivityAnalyzer::findConnectedComponents)
        .add(&ConnectivityAnalyzer::getLargestComponentSize)
        .add(&ConnectivityAnalyzer::getLargestComponent)
        .add(&ConnectivityAnalyzer::getBridgeEdges)
        .add(&ConnectivityAnalyzer::calculateAveragePathLength)
        .add(&ConnectivityAnalyzer::calculateGraphDiameter)
        .add(&ConnectivityAnalyzer::findCriticalNodes)
        .add(&ConnectivityAnalyzer::wouldRemovalDisconnect)
        .add(&ConnectivityAnalyzer::generateConnectivityReport);

    r.add(&MapUpdater::addNode)
        .add(&MapUpdater::removeNode)
        .add(&MapUpdater::updateEdgeWeight)
        .add(&MapUpdater::integrateExternalMap)
        .add(&MapUpdater::synchronizeWithDatabase)
        .add(&MapUpdater::getRecentUpdates)
        .add(&MapUpdater::clearUpdateHistory)
        .add(&MapUpdater::enableRealTimeUpdates)
        .add(&MapUpdater::validateUpdateConsistency)
        .add(&MapUpdater::getUpdateCount)
        .add(&MapUpdater::getLastUpdateTime);

    r.add(&ObstacleDetector::detectObstacles)
        .add(&ObstacleDetector::detectObstaclesInRegion)
        .add(&ObstacleDetector::addStaticObstacle)
        .add(&ObstacleDetector::addDynamicObstacle)
        .add(&ObstacleDetector::removeObstacle)
        .add(&ObstacleDetector::clearAllObstacles)
        .add(&ObstacleDetector::isNodeBlocked)
        .add(&ObstacleDetector::getBlockedNodes)
        .add(&ObstacleDetector::getAllObstacles)
        .add(&ObstacleDetector::getObstaclesNearPath)
        .add(&ObstacleDetector::setDetectionSensitivity)
        .add(&ObstacleDetector::updateObstacleMap)
        .add(&ObstacleDetector::getObstacleDensity);

    r.add(&MultiGoalPlanner::planOptimalTour)
        .add(&MultiGoalPlanner::planOptimalTourWithReturn)
        .add(&MultiGoalPlanner::planIndividualPaths)
        .add(&MultiGoalPlanner::setOptimizationStrategy)
        .add(&MultiGoalPlanner::estimateTourCost)
        .add(&MultiGoalPlanner::getOptimalVisitationOrder);

    r.add(&CostCalculator::calculatePathCost)
        .add(&CostCalculator::calculatePathCostWithType)
        .add(&CostCalculator::calculateDetailedCost)
        .add(&CostCalculator::setCostWeight)
        .add(&CostCalculator::setNodeCost)
        .add(&CostCalculator::setEdgeCost)
        .add(&CostCalculator::getSegmentCost)
        .add(&CostCalculator::getNodeTraversalCost)
        .add(&CostCalculator::estimateCostToGoal)
        .add(&CostCalculator::loadCostModel)
        .add(&CostCalculator::saveCostModel)
        .add(&CostCalculator::resetCostModel)
        .add(&CostCalculator::calculateCostProfile)
        .add(&CostCalculator::findCostliestSegment)
        .add(&CostCalculator::calculateCostVariance);

    r.add(&PathOptimizer::optimizePath)
        .add(&PathOptimizer::optimizeWithCriteria)
        .add(&PathOptimizer::optimizeMultiObjective)
        .add(&PathOptimizer::setOptimizationCriteria)
        .add(&PathOptimizer::calculateOptimizationImprovement)
        .add(&PathOptimizer::shortcutPath)
        .add(&PathOptimizer::interpolateWaypoints)
        .add(&PathOptimizer::isOptimizationNecessary);

    r.add(&PathSmoother::smoothPath)
        .add(&PathSmoother::smoothWithMethod)
        .add(&PathSmoother::adaptiveSmoothing)
        .add(&PathSmoother::setSmoothingMethod)
        .add(&PathSmoother::setSmoothingFactor)
        .add(&PathSmoother::calculateSmoothness)
        .add(&PathSmoother::identifySharpCorners)
        .add(&PathSmoother::validateSmoothedPath)
        .add(&PathSmoother::preserveCriticalWaypoints);

    r.add(&PathVisualizer::visualizePath)
        .add(&PathVisualizer::visualizeMultiplePaths)
        .add(&PathVisualizer::visualizeGraphStructure)
        .add(&PathVisualizer::exportVisualization)
        .add(&PathVisualizer::setCanvasDimensions)
        .add(&PathVisualizer::setScaleFactor)
        .add(&PathVisualizer::generatePathStatistics)
        .add(&PathVisualizer::comparePathVisualizations)
        .add(&PathVisualizer::createInteractiveVisualization);

    r.add(&PerformanceProfiler::startProfiling)
        .add(&PerformanceProfiler::stopProfiling)
        .add(&PerformanceProfiler::recordMetric)
        .add(&PerformanceProfiler::getMetrics)
        .add(&PerformanceProfiler::getAllOperations)
        .add(&PerformanceProfiler::clearProfile)
        .add(&PerformanceProfiler::generatePerformanceReport)
        .add(&PerformanceProfiler::exportToCSV)
        .add(&PerformanceProfiler::comparePeformance)
        .add(&PerformanceProfiler::getAverageExecutionTime)
        .add(&PerformanceProfiler::getTotalMemoryUsage)
        .add(&PerformanceProfiler::isProfilingActive);

    r.add(&ConfigManager::loadFromFile)
        .add(&ConfigManager::saveToFile)
        .add(&ConfigManager::setAutoSave)
        .add(&ConfigManager::setValue<int>)
        .add(&ConfigManager::getValue<std::string>)
        .add(&ConfigManager::hasKey)
        .add(&ConfigManager::removeKey)
        .add(&ConfigManager::clearAll)
        .add(&ConfigManager::getAllKeys)
        .add(&ConfigManager::getAllValues)
        .add(&ConfigManager::setSection)
        .add(&ConfigManager::getSection)
        .add(&ConfigManager::validateConfiguration)
        .add(&ConfigManager::mergeConfiguration)
        .add(&ConfigManager::exportToJSON)
        .add(&ConfigManager::importFromJSON)
        .add(&ConfigManager::printConfiguration);

    r.add(&JumpPointSearch::findPath)
        .add(&JumpPointSearch::supportsGraph)
        .add(&JumpPointSearch::setGridDimensions)
        .add(&JumpPointSearch::addObstacle)
        .add(&JumpPointSearch::removeObstacle)
        .add(&JumpPointSearch::enableDiagonalMovement)
        .add(&JumpPointSearch::getJumpPoints)
        .add(&JumpPointSearch::getPathCost)
        .add(&JumpPointSearch::getPathLength)
        .add(&JumpPointSearch::getLastSearchExpansions);

    r.add(&NavigationCore::startNavigation)
        .add(&NavigationCore::startNavigationWithWaypoints)
        .add(&NavigationCore::stopNavigation)
        .add(&NavigationCore::pauseNavigation)
        .add(&NavigationCore::resumeNavigation)
        .add(&NavigationCore::getNavigationState)
        .add(&NavigationCore::getCurrentPath)
        .add(&NavigationCore::getInvalidatedRoute)
        .add(&NavigationCore::getCurrentPosition)
        .add(&NavigationCore::getGoal)
        .add(&NavigationCore::isNavigationComplete)
        .add(&NavigationCore::getRemainingDistance)
        .add(&NavigationCore::setNavigationMode)
        .add(&NavigationCore::updateEnvironment)
        .add(&NavigationCore::handleObstacles)
        .add(&NavigationCore::clearObstacles)
        .add(&NavigationCore::clearAllObstacles)
        .add(&NavigationCore::retryNavigation)
        .add(&NavigationCore::executeNavigationStep)
        .add(&NavigationCore::getNavigationStatus)
        .add(&NavigationCore::emergencyStop)
        .add(&NavigationCore::getCurrentEnvironment);

    std::cout << "Linked " << r.count << " public member functions of 15 classes (checksum " << r.checksum
              << ")" << std::endl;
    return r.count > 0 && keepConstructors != nullptr ? 0 : 1;
}
