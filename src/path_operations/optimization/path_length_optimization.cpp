#include "path_operations/PathOptimizer.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>

class PathLengthOptimizer {
private:
    const Graph* graph;
    
    struct OptimizationResult {
        std::vector<int> optimizedPath;
        double originalLength;
        double optimizedLength;
        double lengthReduction;
        double reductionPercentage;
        int optimizationSteps;
        std::string method;
        std::vector<int> removedWaypoints;
        std::vector<std::pair<int, int>> shortcuts;
        
        OptimizationResult() : originalLength(0.0), optimizedLength(0.0), 
                              lengthReduction(0.0), reductionPercentage(0.0), optimizationSteps(0) {}
    };
    
    struct ShortcutCandidate {
        int startIndex;
        int endIndex;
        double savings;
        bool isValid;
        std::vector<int> bypassedNodes;
        
        ShortcutCandidate(int start, int end) 
            : startIndex(start), endIndex(end), savings(0.0), isValid(false) {}
        
        bool operator<(const ShortcutCandidate& other) const {
            return savings < other.savings; // For max-heap
        }
    };
    
    // Configuration parameters
    double maxShortcutLength;
    double minSavingsThreshold;
    int maxOptimizationIterations;
    bool enableShortcutValidation;
    bool preserveCriticalWaypoints;
    double simplificationTolerance;
    
    std::vector<int> criticalWaypoints;
    
    double calculatePathLength(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalLength = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            totalLength += from.euclideanDistance(to);
        }
        
        return totalLength;
    }
    
    double calculateSegmentLength(int nodeId1, int nodeId2) const {
        const Node& node1 = graph->getNode(nodeId1);
        const Node& node2 = graph->getNode(nodeId2);
        return node1.euclideanDistance(node2);
    }
    
    std::vector<ShortcutCandidate> identifyShortcutCandidates(const std::vector<int>& path) const {
        std::vector<ShortcutCandidate> candidates;
        
        std::cout << "[PATH_LENGTH_OPT] Identifying shortcut candidates for path with " 
                  << path.size() << " nodes" << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            for (size_t j = i + 2; j < path.size(); ++j) {
                // Skip if endpoints are too far apart
                double directDistance = calculateSegmentLength(path[i], path[j]);
                if (directDistance > maxShortcutLength) continue;
                
                // Skip if either node is critical
                if (preserveCriticalWaypoints && 
                    (isCriticalWaypoint(path[i]) || isCriticalWaypoint(path[j]))) {
                    continue;
                }
                
                ShortcutCandidate candidate(static_cast<int>(i), static_cast<int>(j));
                
                // Calculate original path length for this segment
                double originalLength = 0.0;
                for (size_t k = i + 1; k <= j; ++k) {
                    originalLength += calculateSegmentLength(path[k-1], path[k]);
                }
                
                // Calculate potential savings
                candidate.savings = originalLength - directDistance;
                
                // Check if shortcut is valid (direct connection exists or can be approximated)
                if (enableShortcutValidation) {
                    candidate.isValid = validateShortcut(path[i], path[j], path, i, j);
                } else {
                    candidate.isValid = graph->hasEdge(path[i], path[j]);
                }
                
                if (candidate.isValid && candidate.savings > minSavingsThreshold) {
                    // Store bypassed nodes
                    for (size_t k = i + 1; k < j; ++k) {
                        candidate.bypassedNodes.push_back(path[k]);
                    }
                    candidates.push_back(candidate);
                }
            }
        }
        
        // Sort by savings (highest first)
        std::sort(candidates.begin(), candidates.end(), 
                 [](const ShortcutCandidate& a, const ShortcutCandidate& b) {
                     return a.savings > b.savings;
                 });
        
        std::cout << "[PATH_LENGTH_OPT] Found " << candidates.size() << " valid shortcut candidates" << std::endl;
        
        return candidates;
    }
    
    bool validateShortcut(int startNode, int endNode, const std::vector<int>& originalPath, 
                         size_t startIndex, size_t endIndex) const {
        // Check if direct edge exists
        if (graph->hasEdge(startNode, endNode)) {
            return true;
        }
        
        // Check if a valid alternative path exists that's shorter than original
        std::vector<int> shortcutPath = findAlternativePath(startNode, endNode, originalPath, startIndex, endIndex);
        
        if (shortcutPath.empty()) return false;
        
        double shortcutLength = calculatePathLength(shortcutPath);
        double originalLength = 0.0;
        for (size_t i = startIndex + 1; i <= endIndex; ++i) {
            originalLength += calculateSegmentLength(originalPath[i-1], originalPath[i]);
        }
        
        return shortcutLength < originalLength;
    }
    
    std::vector<int> findAlternativePath(int startNode, int endNode, const std::vector<int>& originalPath,
                                       size_t startIndex, size_t endIndex) const {
        // Simple BFS to find shortest alternative path
        std::queue<std::vector<int>> pathQueue;
        std::unordered_set<int> visited;
        std::unordered_set<int> forbidden;
        
        // Forbid nodes from the original segment (except endpoints)
        for (size_t i = startIndex + 1; i < endIndex; ++i) {
            forbidden.insert(originalPath[i]);
        }
        
        pathQueue.push({startNode});
        visited.insert(startNode);
        
        while (!pathQueue.empty()) {
            std::vector<int> currentPath = pathQueue.front();
            pathQueue.pop();
            
            int currentNode = currentPath.back();
            
            if (currentNode == endNode) {
                return currentPath;
            }
            
            // Limit search depth
            if (currentPath.size() > 10) continue;
            
            std::vector<int> neighbors = graph->getNeighbors(currentNode);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end() && 
                    forbidden.find(neighbor) == forbidden.end()) {
                    
                    visited.insert(neighbor);
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(neighbor);
                    pathQueue.push(newPath);
                }
            }
        }
        
        return {}; // No alternative path found
    }
    
    OptimizationResult optimizeWithShortcuts(const std::vector<int>& originalPath) {
        std::cout << "[PATH_LENGTH_OPT] Applying shortcut-based optimization" << std::endl;
        
        OptimizationResult result;
        result.method = "Shortcuts";
        result.optimizedPath = originalPath;
        result.originalLength = calculatePathLength(originalPath);
        
        bool improved = true;
        int iteration = 0;
        
        while (improved && iteration < maxOptimizationIterations) {
            improved = false;
            iteration++;
            
            std::vector<ShortcutCandidate> candidates = identifyShortcutCandidates(result.optimizedPath);
            
            if (!candidates.empty()) {
                ShortcutCandidate bestCandidate = candidates[0];
                
                // Apply the best shortcut
                std::vector<int> newPath;
                
                // Add nodes before shortcut
                for (int i = 0; i <= bestCandidate.startIndex; ++i) {
                    newPath.push_back(result.optimizedPath[i]);
                }
                
                // Add shortcut (if not direct edge, add intermediate nodes from alternative path)
                if (!graph->hasEdge(result.optimizedPath[bestCandidate.startIndex], 
                                   result.optimizedPath[bestCandidate.endIndex])) {
                    std::vector<int> altPath = findAlternativePath(
                        result.optimizedPath[bestCandidate.startIndex],
                        result.optimizedPath[bestCandidate.endIndex],
                        result.optimizedPath, bestCandidate.startIndex, bestCandidate.endIndex);
                    
                    // Add intermediate nodes from alternative path (skip first node, already added)
                    for (size_t i = 1; i < altPath.size() - 1; ++i) {
                        newPath.push_back(altPath[i]);
                    }
                }
                
                // Add nodes after shortcut
                for (size_t i = bestCandidate.endIndex; i < result.optimizedPath.size(); ++i) {
                    newPath.push_back(result.optimizedPath[i]);
                }
                
                double newLength = calculatePathLength(newPath);
                if (newLength < result.optimizedLength || result.optimizationSteps == 0) {
                    result.optimizedPath = newPath;
                    result.optimizedLength = newLength;
                    result.shortcuts.emplace_back(bestCandidate.startIndex, bestCandidate.endIndex);
                    result.removedWaypoints.insert(result.removedWaypoints.end(), 
                                                 bestCandidate.bypassedNodes.begin(), 
                                                 bestCandidate.bypassedNodes.end());
                    improved = true;
                    result.optimizationSteps++;
                    
                    std::cout << "[PATH_LENGTH_OPT] Applied shortcut from index " 
                              << bestCandidate.startIndex << " to " << bestCandidate.endIndex
                              << ", savings: " << bestCandidate.savings << std::endl;
                }
            }
        }
        
        if (result.optimizationSteps == 0) {
            result.optimizedLength = result.originalLength;
        }
        
        result.lengthReduction = result.originalLength - result.optimizedLength;
        result.reductionPercentage = (result.lengthReduction / result.originalLength) * 100.0;
        
        std::cout << "[PATH_LENGTH_OPT] Shortcut optimization completed in " << iteration 
                  << " iterations, length reduced by " << result.reductionPercentage << "%" << std::endl;
        
        return result;
    }
    
    OptimizationResult optimizeWithDouglasPeucker(const std::vector<int>& originalPath) {
        std::cout << "[PATH_LENGTH_OPT] Applying Douglas-Peucker simplification" << std::endl;
        
        OptimizationResult result;
        result.method = "Douglas-Peucker";
        result.originalLength = calculatePathLength(originalPath);
        
        if (originalPath.size() < 3) {
            result.optimizedPath = originalPath;
            result.optimizedLength = result.originalLength;
            return result;
        }
        
        // Convert path to coordinate points
        std::vector<std::pair<double, double>> points;
        for (int nodeId : originalPath) {
            const Node& node = graph->getNode(nodeId);
            points.emplace_back(node.getX(), node.getY());
        }
        
        // Apply Douglas-Peucker algorithm
        std::vector<bool> keep(points.size(), false);
        keep[0] = keep[points.size() - 1] = true; // Always keep endpoints
        
        douglasPeuckerRecursive(points, 0, points.size() - 1, keep);
        
        // Build simplified path
        for (size_t i = 0; i < originalPath.size(); ++i) {
            if (keep[i]) {
                result.optimizedPath.push_back(originalPath[i]);
            } else {
                result.removedWaypoints.push_back(originalPath[i]);
            }
        }
        
        result.optimizedLength = calculatePathLength(result.optimizedPath);
        result.lengthReduction = result.originalLength - result.optimizedLength;
        result.reductionPercentage = (result.lengthReduction / result.originalLength) * 100.0;
        result.optimizationSteps = 1;
        
        std::cout << "[PATH_LENGTH_OPT] Douglas-Peucker completed, removed " 
                  << result.removedWaypoints.size() << " waypoints" << std::endl;
        
        return result;
    }
    
    void douglasPeuckerRecursive(const std::vector<std::pair<double, double>>& points, 
                               size_t start, size_t end, std::vector<bool>& keep) const {
        if (end <= start + 1) return;
        
        // Find the point with maximum distance from line segment
        double maxDistance = 0.0;
        size_t maxIndex = start;
        
        for (size_t i = start + 1; i < end; ++i) {
            double distance = perpendicularDistance(points[i], points[start], points[end]);
            if (distance > maxDistance) {
                maxDistance = distance;
                maxIndex = i;
            }
        }
        
        // If max distance is greater than tolerance, recursively simplify
        if (maxDistance > simplificationTolerance) {
            keep[maxIndex] = true;
            douglasPeuckerRecursive(points, start, maxIndex, keep);
            douglasPeuckerRecursive(points, maxIndex, end, keep);
        }
    }
    
    double perpendicularDistance(const std::pair<double, double>& point,
                               const std::pair<double, double>& lineStart,
                               const std::pair<double, double>& lineEnd) const {
        double dx = lineEnd.first - lineStart.first;
        double dy = lineEnd.second - lineStart.second;
        
        if (dx == 0 && dy == 0) {
            // Line start and end are the same point
            return std::sqrt(std::pow(point.first - lineStart.first, 2) + 
                           std::pow(point.second - lineStart.second, 2));
        }
        
        double t = ((point.first - lineStart.first) * dx + (point.second - lineStart.second) * dy) / 
                   (dx * dx + dy * dy);
        
        t = std::max(0.0, std::min(1.0, t));
        
        double projectionX = lineStart.first + t * dx;
        double projectionY = lineStart.second + t * dy;
        
        return std::sqrt(std::pow(point.first - projectionX, 2) + 
                        std::pow(point.second - projectionY, 2));
    }
    
    OptimizationResult optimizeWithWaypointRemoval(const std::vector<int>& originalPath) {
        std::cout << "[PATH_LENGTH_OPT] Applying redundant waypoint removal optimization" << std::endl;
        
        OptimizationResult result;
        result.method = "Waypoint Removal";
        result.optimizedPath = originalPath;
        result.originalLength = calculatePathLength(originalPath);
        
        if (originalPath.size() < 3) {
            result.optimizedLength = result.originalLength;
            return result;
        }
        
        bool improved = true;
        while (improved) {
            improved = false;
            
            for (size_t i = 1; i < result.optimizedPath.size() - 1; ++i) {
                if (preserveCriticalWaypoints && isCriticalWaypoint(result.optimizedPath[i])) {
                    continue;
                }
                
                // Check if removing this waypoint improves or maintains path validity
                if (canRemoveWaypoint(result.optimizedPath, i)) {
                    double lengthBefore = calculatePathLength(result.optimizedPath);
                    
                    std::vector<int> testPath = result.optimizedPath;
                    testPath.erase(testPath.begin() + i);
                    
                    if (validatePathConnectivity(testPath)) {
                        double lengthAfter = calculatePathLength(testPath);
                        
                        if (lengthAfter <= lengthBefore) {
                            result.removedWaypoints.push_back(result.optimizedPath[i]);
                            result.optimizedPath = testPath;
                            result.optimizationSteps++;
                            improved = true;
                            
                            std::cout << "[PATH_LENGTH_OPT] Removed waypoint " << result.optimizedPath[i] 
                                      << ", length change: " << (lengthAfter - lengthBefore) << std::endl;
                            break;
                        }
                    }
                }
            }
        }
        
        result.optimizedLength = calculatePathLength(result.optimizedPath);
        result.lengthReduction = result.originalLength - result.optimizedLength;
        result.reductionPercentage = (result.lengthReduction / result.originalLength) * 100.0;
        
        std::cout << "[PATH_LENGTH_OPT] Waypoint removal completed, removed " 
                  << result.removedWaypoints.size() << " waypoints" << std::endl;
        
        return result;
    }
    
    bool canRemoveWaypoint(const std::vector<int>& path, size_t index) const {
        if (index == 0 || index >= path.size() - 1) return false;
        
        int prevNode = path[index - 1];
        int nextNode = path[index + 1];
        
        // Check if direct connection exists or can be established
        return graph->hasEdge(prevNode, nextNode) || 
               !findAlternativePath(prevNode, nextNode, path, index - 1, index + 1).empty();
    }
    
    bool validatePathConnectivity(const std::vector<int>& path) const {
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(path[i-1], path[i])) {
                return false;
            }
        }
        return true;
    }
    
    bool isCriticalWaypoint(int nodeId) const {
        return std::find(criticalWaypoints.begin(), criticalWaypoints.end(), nodeId) 
               != criticalWaypoints.end();
    }
    
public:
    PathLengthOptimizer(const Graph* environment) 
        : graph(environment), maxShortcutLength(50.0), minSavingsThreshold(0.1),
          maxOptimizationIterations(100), enableShortcutValidation(true),
          preserveCriticalWaypoints(true), simplificationTolerance(1.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[PATH_LENGTH_OPT] Path length optimizer initialized" << std::endl;
    }
    
    std::vector<int> optimizePathLength(const std::vector<int>& originalPath, 
                                       const std::string& method = "shortcuts") {
        
        if (originalPath.size() < 2) {
            std::cout << "[PATH_LENGTH_OPT] Path too short for optimization" << std::endl;
            return originalPath;
        }
        
        std::cout << "[PATH_LENGTH_OPT] Optimizing path length using " << method 
                  << " method for " << originalPath.size() << " nodes" << std::endl;
        
        OptimizationResult result;
        
        if (method == "shortcuts") {
            result = optimizeWithShortcuts(originalPath);
        } else if (method == "douglas_peucker") {
            result = optimizeWithDouglasPeucker(originalPath);
        } else if (method == "waypoint_removal") {
            result = optimizeWithWaypointRemoval(originalPath);
        } else if (method == "combined") {
            // Apply multiple methods in sequence
            result = optimizeWithShortcuts(originalPath);
            result = optimizeWithWaypointRemoval(result.optimizedPath);
            OptimizationResult dpResult = optimizeWithDouglasPeucker(result.optimizedPath);
            if (dpResult.optimizedLength < result.optimizedLength) {
                result = dpResult;
                result.method = "Combined (shortcuts + waypoint_removal + douglas_peucker)";
            } else {
                result.method = "Combined (shortcuts + waypoint_removal)";
            }
        } else {
            std::cout << "[PATH_LENGTH_OPT] Unknown method, using shortcuts" << std::endl;
            result = optimizeWithShortcuts(originalPath);
        }
        
        std::cout << "[PATH_LENGTH_OPT] Optimization complete - Path length reduced by " 
                  << result.reductionPercentage << "% (" << result.lengthReduction 
                  << " units)" << std::endl;
        
        return result.optimizedPath;
    }
    
    double calculateOptimizationPotential(const std::vector<int>& path) const {
        if (path.size() < 3) return 0.0;
        
        double originalLength = calculatePathLength(path);
        double directLength = calculateSegmentLength(path.front(), path.back());
        
        return ((originalLength - directLength) / originalLength) * 100.0;
    }
    
    void setCriticalWaypoints(const std::vector<int>& waypoints) {
        criticalWaypoints = waypoints;
        std::cout << "[PATH_LENGTH_OPT] Set " << waypoints.size() << " critical waypoints" << std::endl;
    }
    
    void setMaxShortcutLength(double maxLength) {
        maxShortcutLength = maxLength;
        std::cout << "[PATH_LENGTH_OPT] Max shortcut length set to " << maxLength << std::endl;
    }
    
    void setMinSavingsThreshold(double threshold) {
        minSavingsThreshold = threshold;
        std::cout << "[PATH_LENGTH_OPT] Min savings threshold set to " << threshold << std::endl;
    }
    
    void setSimplificationTolerance(double tolerance) {
        simplificationTolerance = tolerance;
        std::cout << "[PATH_LENGTH_OPT] Simplification tolerance set to " << tolerance << std::endl;
    }
    
    void enableWaypointPreservation(bool preserve) {
        preserveCriticalWaypoints = preserve;
        std::cout << "[PATH_LENGTH_OPT] Waypoint preservation " 
                  << (preserve ? "enabled" : "disabled") << std::endl;
    }
    
    void setMaxIterations(int iterations) {
        maxOptimizationIterations = iterations;
        std::cout << "[PATH_LENGTH_OPT] Max optimization iterations set to " << iterations << std::endl;
    }
    
    void printOptimizationConfiguration() const {
        std::cout << "[PATH_LENGTH_OPT] Configuration:" << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Max shortcut length: " << maxShortcutLength << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Min savings threshold: " << minSavingsThreshold << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Max iterations: " << maxOptimizationIterations << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Simplification tolerance: " << simplificationTolerance << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Preserve waypoints: " << preserveCriticalWaypoints << std::endl;
        std::cout << "[PATH_LENGTH_OPT]   Critical waypoints: " << criticalWaypoints.size() << std::endl;
    }
};