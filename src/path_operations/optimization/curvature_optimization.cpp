#include "path_operations/PathOptimizer.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>

class CurvatureOptimizer {
private:
    const Graph* graph;
    
    struct CurvaturePoint {
        int nodeId;
        double x, y;
        double curvature;
        double smoothnessScore;
        bool isInflectionPoint;
        std::vector<int> smoothingCandidates;
        
        CurvaturePoint(int id, double xPos, double yPos) 
            : nodeId(id), x(xPos), y(yPos), curvature(0.0), 
              smoothnessScore(0.0), isInflectionPoint(false) {}
    };
    
    struct OptimizationResult {
        std::vector<int> optimizedPath;
        std::vector<double> curvatureProfile;
        double totalCurvature;
        double maxCurvature;
        double smoothnessImprovement;
        int optimizationSteps;
        std::string method;
        
        OptimizationResult() : totalCurvature(0.0), maxCurvature(0.0), 
                              smoothnessImprovement(0.0), optimizationSteps(0) {}
    };
    
    // Configuration parameters
    double maxCurvatureThreshold;
    double smoothingRadius;
    int maxOptimizationIterations;
    bool enableInflectionPointDetection;
    bool preserveWaypoints;
    double convergenceThreshold;
    
    std::vector<int> criticalWaypoints;
    
    std::vector<CurvaturePoint> analyzeCurvatureProfile(const std::vector<int>& path) const {
        std::vector<CurvaturePoint> profile;
        
        if (path.size() < 3) return profile;
        
        std::cout << "[CURVATURE_OPT] Analyzing curvature profile for path with " 
                  << path.size() << " nodes" << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            const Node& node = graph->getNode(path[i]);
            CurvaturePoint point(path[i], node.getX(), node.getY());
            
            if (i > 0 && i < path.size() - 1) {
                point.curvature = calculateCurvatureAtPoint(path, i);
                point.smoothnessScore = calculateSmoothnessScore(path, i);
                point.isInflectionPoint = detectInflectionPoint(path, i);
            }
            
            profile.push_back(point);
        }
        
        identifySmoothingCandidates(profile);
        
        double totalCurv = 0.0;
        double maxCurv = 0.0;
        
        for (const auto& point : profile) {
            totalCurv += std::abs(point.curvature);
            maxCurv = std::max(maxCurv, std::abs(point.curvature));
        }
        
        std::cout << "[CURVATURE_OPT] Total curvature: " << totalCurv 
                  << ", Max curvature: " << maxCurv << std::endl;
        
        return profile;
    }
    
    double calculateCurvatureAtPoint(const std::vector<int>& path, size_t index) const {
        if (index == 0 || index >= path.size() - 1) return 0.0;
        
        const Node& prev = graph->getNode(path[index - 1]);
        const Node& curr = graph->getNode(path[index]);
        const Node& next = graph->getNode(path[index + 1]);
        
        return MathUtils::calculateCurvature(
            prev.getX(), prev.getY(),
            curr.getX(), curr.getY(),
            next.getX(), next.getY()
        );
    }
    
    double calculateSmoothnessScore(const std::vector<int>& path, size_t index) const {
        if (index < 2 || index >= path.size() - 2) return 1.0;
        
        double curvature1 = calculateCurvatureAtPoint(path, index - 1);
        double curvature2 = calculateCurvatureAtPoint(path, index);
        double curvature3 = calculateCurvatureAtPoint(path, index + 1);
        
        double curvatureVariation = std::abs(curvature2 - curvature1) + 
                                   std::abs(curvature3 - curvature2);
        
        return 1.0 / (1.0 + curvatureVariation);
    }
    
    bool detectInflectionPoint(const std::vector<int>& path, size_t index) const {
        if (!enableInflectionPointDetection || index < 2 || index >= path.size() - 2) {
            return false;
        }
        
        double prevCurvature = calculateCurvatureAtPoint(path, index - 1);
        double currCurvature = calculateCurvatureAtPoint(path, index);
        double nextCurvature = calculateCurvatureAtPoint(path, index + 1);
        
        return (prevCurvature > 0 && currCurvature < 0) || 
               (prevCurvature < 0 && currCurvature > 0) ||
               (currCurvature > 0 && nextCurvature < 0) || 
               (currCurvature < 0 && nextCurvature > 0);
    }
    
    void identifySmoothingCandidates(std::vector<CurvaturePoint>& profile) const {
        for (size_t i = 1; i < profile.size() - 1; ++i) {
            if (std::abs(profile[i].curvature) > maxCurvatureThreshold || 
                profile[i].smoothnessScore < 0.5) {
                
                std::vector<int> neighbors = graph->getNeighbors(profile[i].nodeId);
                for (int neighbor : neighbors) {
                    bool isInPath = false;
                    for (const auto& point : profile) {
                        if (point.nodeId == neighbor) {
                            isInPath = true;
                            break;
                        }
                    }
                    
                    if (!isInPath) {
                        const Node& neighborNode = graph->getNode(neighbor);
                        double distance = std::sqrt(
                            std::pow(neighborNode.getX() - profile[i].x, 2) + 
                            std::pow(neighborNode.getY() - profile[i].y, 2)
                        );
                        
                        if (distance <= smoothingRadius) {
                            profile[i].smoothingCandidates.push_back(neighbor);
                        }
                    }
                }
            }
        }
    }
    
    OptimizationResult optimizeWithLocalSmoothing(const std::vector<int>& originalPath) {
        std::cout << "[CURVATURE_OPT] Applying local smoothing optimization" << std::endl;
        
        OptimizationResult result;
        result.method = "Local Smoothing";
        result.optimizedPath = originalPath;
        
        std::vector<CurvaturePoint> profile = analyzeCurvatureProfile(result.optimizedPath);
        double initialCurvature = calculateTotalCurvature(profile);
        
        bool improved = true;
        int iteration = 0;
        
        while (improved && iteration < maxOptimizationIterations) {
            improved = false;
            iteration++;
            
            for (size_t i = 1; i < result.optimizedPath.size() - 1; ++i) {
                if (preserveWaypoints && isCriticalWaypoint(result.optimizedPath[i])) {
                    continue;
                }
                
                std::vector<int> bestAlternative = findBestSmoothingAlternative(result.optimizedPath, i);
                
                if (!bestAlternative.empty()) {
                    double newCurvature = evaluatePathCurvature(bestAlternative);
                    double currentCurvature = evaluatePathCurvature(result.optimizedPath);
                    
                    if (newCurvature < currentCurvature - convergenceThreshold) {
                        result.optimizedPath = bestAlternative;
                        improved = true;
                        result.optimizationSteps++;
                        
                        std::cout << "[CURVATURE_OPT] Smoothing improvement at iteration " 
                                  << iteration << ": " << currentCurvature << " -> " << newCurvature << std::endl;
                        break;
                    }
                }
            }
        }
        
        std::vector<CurvaturePoint> finalProfile = analyzeCurvatureProfile(result.optimizedPath);
        result.totalCurvature = calculateTotalCurvature(finalProfile);
        result.maxCurvature = calculateMaxCurvature(finalProfile);
        result.smoothnessImprovement = (initialCurvature - result.totalCurvature) / initialCurvature * 100.0;
        result.curvatureProfile = extractCurvatureValues(finalProfile);
        
        std::cout << "[CURVATURE_OPT] Local smoothing completed in " << iteration 
                  << " iterations, improvement: " << result.smoothnessImprovement << "%" << std::endl;
        
        return result;
    }
    
    OptimizationResult optimizeWithBezierSmoothing(const std::vector<int>& originalPath) {
        std::cout << "[CURVATURE_OPT] Applying Bezier curve smoothing optimization" << std::endl;
        
        OptimizationResult result;
        result.method = "Bezier Smoothing";
        result.optimizedPath = originalPath;
        
        if (originalPath.size() < 4) {
            result.optimizedPath = originalPath;
            return result;
        }
        
        std::vector<std::pair<double, double>> controlPoints;
        for (int nodeId : originalPath) {
            const Node& node = graph->getNode(nodeId);
            controlPoints.emplace_back(node.getX(), node.getY());
        }
        
        std::vector<std::pair<double, double>> smoothedPoints = generateBezierCurve(controlPoints);
        result.optimizedPath = mapSmoothPointsToNodes(smoothedPoints);
        
        std::vector<CurvaturePoint> finalProfile = analyzeCurvatureProfile(result.optimizedPath);
        result.totalCurvature = calculateTotalCurvature(finalProfile);
        result.maxCurvature = calculateMaxCurvature(finalProfile);
        result.curvatureProfile = extractCurvatureValues(finalProfile);
        result.optimizationSteps = 1;
        
        double originalCurvature = evaluatePathCurvature(originalPath);
        result.smoothnessImprovement = (originalCurvature - result.totalCurvature) / originalCurvature * 100.0;
        
        std::cout << "[CURVATURE_OPT] Bezier smoothing completed, improvement: " 
                  << result.smoothnessImprovement << "%" << std::endl;
        
        return result;
    }
    
    OptimizationResult optimizeWithSplineInterpolation(const std::vector<int>& originalPath) {
        std::cout << "[CURVATURE_OPT] Applying spline interpolation optimization" << std::endl;
        
        OptimizationResult result;
        result.method = "Spline Interpolation";
        result.optimizedPath = originalPath;
        
        if (originalPath.size() < 4) {
            result.optimizedPath = originalPath;
            return result;
        }
        
        std::vector<std::pair<double, double>> keyPoints;
        for (int nodeId : originalPath) {
            const Node& node = graph->getNode(nodeId);
            keyPoints.emplace_back(node.getX(), node.getY());
        }
        
        int interpolationPoints = static_cast<int>(originalPath.size() * 1.5);
        std::vector<std::pair<double, double>> interpolatedPoints = 
            MathUtils::interpolatePath(keyPoints, interpolationPoints);
        
        result.optimizedPath = mapSmoothPointsToNodes(interpolatedPoints);
        
        std::vector<CurvaturePoint> finalProfile = analyzeCurvatureProfile(result.optimizedPath);
        result.totalCurvature = calculateTotalCurvature(finalProfile);
        result.maxCurvature = calculateMaxCurvature(finalProfile);
        result.curvatureProfile = extractCurvatureValues(finalProfile);
        result.optimizationSteps = 1;
        
        double originalCurvature = evaluatePathCurvature(originalPath);
        result.smoothnessImprovement = (originalCurvature - result.totalCurvature) / originalCurvature * 100.0;
        
        std::cout << "[CURVATURE_OPT] Spline interpolation completed, improvement: " 
                  << result.smoothnessImprovement << "%" << std::endl;
        
        return result;
    }
    
    std::vector<int> findBestSmoothingAlternative(const std::vector<int>& path, size_t problemIndex) const {
        std::vector<int> neighbors = graph->getNeighbors(path[problemIndex]);
        std::vector<int> bestAlternative;
        double bestCurvature = std::numeric_limits<double>::infinity();
        
        for (int neighbor : neighbors) {
            bool isAlreadyInPath = std::find(path.begin(), path.end(), neighbor) != path.end();
            if (isAlreadyInPath) continue;
            
            std::vector<int> testPath = path;
            testPath[problemIndex] = neighbor;
            
            if (validatePathConnectivity(testPath)) {
                double testCurvature = evaluatePathCurvature(testPath);
                if (testCurvature < bestCurvature) {
                    bestCurvature = testCurvature;
                    bestAlternative = testPath;
                }
            }
        }
        
        return bestAlternative;
    }
    
    std::vector<std::pair<double, double>> generateBezierCurve(const std::vector<std::pair<double, double>>& controlPoints) const {
        std::vector<std::pair<double, double>> curvePoints;
        
        if (controlPoints.size() < 4) return controlPoints;
        
        const int numSegments = 50;
        
        for (size_t i = 0; i < controlPoints.size() - 3; i += 3) {
            for (int t = 0; t <= numSegments; ++t) {
                double u = static_cast<double>(t) / numSegments;
                double x = std::pow(1 - u, 3) * controlPoints[i].first +
                          3 * std::pow(1 - u, 2) * u * controlPoints[i + 1].first +
                          3 * (1 - u) * std::pow(u, 2) * controlPoints[i + 2].first +
                          std::pow(u, 3) * controlPoints[i + 3].first;
                          
                double y = std::pow(1 - u, 3) * controlPoints[i].second +
                          3 * std::pow(1 - u, 2) * u * controlPoints[i + 1].second +
                          3 * (1 - u) * std::pow(u, 2) * controlPoints[i + 2].second +
                          std::pow(u, 3) * controlPoints[i + 3].second;
                
                curvePoints.emplace_back(x, y);
            }
        }
        
        return curvePoints;
    }
    
    std::vector<int> mapSmoothPointsToNodes(const std::vector<std::pair<double, double>>& smoothPoints) const {
        std::vector<int> mappedPath;
        
        for (const auto& point : smoothPoints) {
            int nearestNode = findNearestNode(point.first, point.second);
            if (nearestNode != -1 && (mappedPath.empty() || mappedPath.back() != nearestNode)) {
                mappedPath.push_back(nearestNode);
            }
        }
        
        return mappedPath;
    }
    
    int findNearestNode(double x, double y) const {
        int nearestId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int nodeId : graph->getAllNodeIds()) {
            const Node& node = graph->getNode(nodeId);
            double distance = std::sqrt(
                std::pow(node.getX() - x, 2) + std::pow(node.getY() - y, 2)
            );
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = nodeId;
            }
        }
        
        return nearestId;
    }
    
    bool validatePathConnectivity(const std::vector<int>& path) const {
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(path[i-1], path[i])) {
                return false;
            }
        }
        return true;
    }
    
    double evaluatePathCurvature(const std::vector<int>& path) const {
        std::vector<CurvaturePoint> profile = analyzeCurvatureProfile(path);
        return calculateTotalCurvature(profile);
    }
    
    double calculateTotalCurvature(const std::vector<CurvaturePoint>& profile) const {
        double total = 0.0;
        for (const auto& point : profile) {
            total += std::abs(point.curvature);
        }
        return total;
    }
    
    double calculateMaxCurvature(const std::vector<CurvaturePoint>& profile) const {
        double maxCurv = 0.0;
        for (const auto& point : profile) {
            maxCurv = std::max(maxCurv, std::abs(point.curvature));
        }
        return maxCurv;
    }
    
    std::vector<double> extractCurvatureValues(const std::vector<CurvaturePoint>& profile) const {
        std::vector<double> values;
        for (const auto& point : profile) {
            values.push_back(point.curvature);
        }
        return values;
    }
    
    bool isCriticalWaypoint(int nodeId) const {
        return std::find(criticalWaypoints.begin(), criticalWaypoints.end(), nodeId) 
               != criticalWaypoints.end();
    }
    
public:
    CurvatureOptimizer(const Graph* environment) 
        : graph(environment), maxCurvatureThreshold(0.5), smoothingRadius(2.0),
          maxOptimizationIterations(100), enableInflectionPointDetection(true),
          preserveWaypoints(true), convergenceThreshold(0.01) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[CURVATURE_OPT] Curvature optimizer initialized with max threshold " 
                  << maxCurvatureThreshold << std::endl;
    }
    
    std::vector<int> optimizePathCurvature(const std::vector<int>& originalPath, 
                                         const std::string& method = "local_smoothing") {
        
        if (originalPath.size() < 3) {
            std::cout << "[CURVATURE_OPT] Path too short for curvature optimization" << std::endl;
            return originalPath;
        }
        
        std::cout << "[CURVATURE_OPT] Optimizing path curvature using " << method 
                  << " method for " << originalPath.size() << " nodes" << std::endl;
        
        OptimizationResult result;
        
        if (method == "local_smoothing") {
            result = optimizeWithLocalSmoothing(originalPath);
        } else if (method == "bezier") {
            result = optimizeWithBezierSmoothing(originalPath);
        } else if (method == "spline") {
            result = optimizeWithSplineInterpolation(originalPath);
        } else {
            std::cout << "[CURVATURE_OPT] Unknown method, using local smoothing" << std::endl;
            result = optimizeWithLocalSmoothing(originalPath);
        }
        
        std::cout << "[CURVATURE_OPT] Optimization complete - Total curvature reduced by " 
                  << result.smoothnessImprovement << "%" << std::endl;
        
        return result.optimizedPath;
    }
    
    std::vector<double> analyzeCurvatureDistribution(const std::vector<int>& path) const {
        std::vector<CurvaturePoint> profile = analyzeCurvatureProfile(path);
        return extractCurvatureValues(profile);
    }
    
    void setCriticalWaypoints(const std::vector<int>& waypoints) {
        criticalWaypoints = waypoints;
        std::cout << "[CURVATURE_OPT] Set " << waypoints.size() << " critical waypoints" << std::endl;
    }
    
    void setMaxCurvatureThreshold(double threshold) {
        maxCurvatureThreshold = threshold;
        std::cout << "[CURVATURE_OPT] Max curvature threshold set to " << threshold << std::endl;
    }
    
    void setSmoothingRadius(double radius) {
        smoothingRadius = radius;
        std::cout << "[CURVATURE_OPT] Smoothing radius set to " << radius << std::endl;
    }
    
    void enableWaypointPreservation(bool preserve) {
        preserveWaypoints = preserve;
        std::cout << "[CURVATURE_OPT] Waypoint preservation " 
                  << (preserve ? "enabled" : "disabled") << std::endl;
    }
    
    void setMaxIterations(int iterations) {
        maxOptimizationIterations = iterations;
        std::cout << "[CURVATURE_OPT] Max optimization iterations set to " << iterations << std::endl;
    }
    
    double calculatePathSmoothness(const std::vector<int>& path) const {
        if (path.size() < 3) return 1.0;
        
        std::vector<CurvaturePoint> profile = analyzeCurvatureProfile(path);
        double totalSmoothness = 0.0;
        int validPoints = 0;
        
        for (const auto& point : profile) {
            if (point.smoothnessScore > 0.0) {
                totalSmoothness += point.smoothnessScore;
                validPoints++;
            }
        }
        
        return validPoints > 0 ? totalSmoothness / validPoints : 1.0;
    }
    
    void printOptimizationConfiguration() const {
        std::cout << "[CURVATURE_OPT] Configuration:" << std::endl;
        std::cout << "[CURVATURE_OPT]   Max curvature threshold: " << maxCurvatureThreshold << std::endl;
        std::cout << "[CURVATURE_OPT]   Smoothing radius: " << smoothingRadius << std::endl;
        std::cout << "[CURVATURE_OPT]   Max iterations: " << maxOptimizationIterations << std::endl;
        std::cout << "[CURVATURE_OPT]   Preserve waypoints: " << preserveWaypoints << std::endl;
        std::cout << "[CURVATURE_OPT]   Inflection detection: " << enableInflectionPointDetection << std::endl;
        std::cout << "[CURVATURE_OPT]   Critical waypoints: " << criticalWaypoints.size() << std::endl;
    }
};