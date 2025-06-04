#include "path_operations/PathSmoother.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>

class BezierCurveSmoother {
private:
    const Graph* graph;
    
    struct BezierControlPoint {
        double x, y;
        bool isFixed;
        int originalNodeId;
        
        BezierControlPoint(double x = 0, double y = 0, bool fixed = false, int id = -1)
            : x(x), y(y), isFixed(fixed), originalNodeId(id) {}
    };
    
    struct BezierSegment {
        BezierControlPoint p0, p1, p2, p3;  // 4 control points for cubic Bezier
        std::vector<std::pair<double, double>> interpolatedPoints;
        double segmentLength;
        
        BezierSegment() : segmentLength(0.0) {}
    };
    
    int interpolationResolution;
    double smoothingStrength;
    bool preserveEndpoints;
    std::vector<int> fixedWaypoints;
    
    std::vector<BezierControlPoint> generateControlPoints(const std::vector<int>& path) const {
        std::vector<BezierControlPoint> controlPoints;
        
        for (size_t i = 0; i < path.size(); ++i) {
            const Node& node = graph->getNode(path[i]);
            bool isFixed = isFixedWaypoint(path[i]) || 
                          (preserveEndpoints && (i == 0 || i == path.size() - 1));
            
            controlPoints.emplace_back(node.getX(), node.getY(), isFixed, path[i]);
        }
        
        return controlPoints;
    }
    
    void calculateTangentControlPoints(std::vector<BezierControlPoint>& points) const {
        for (size_t i = 1; i < points.size() - 1; ++i) {
            if (points[i].isFixed) continue;
            
            // Calculate tangent direction from neighboring points
            double prevX = points[i-1].x, prevY = points[i-1].y;
            double nextX = points[i+1].x, nextY = points[i+1].y;
            double currX = points[i].x, currY = points[i].y;
            
            // Tangent vector
            double tangentX = (nextX - prevX) * 0.5;
            double tangentY = (nextY - prevY) * 0.5;
            
            // Apply smoothing strength
            double magnitude = std::sqrt(tangentX * tangentX + tangentY * tangentY);
            if (magnitude > 0) {
                tangentX = (tangentX / magnitude) * magnitude * smoothingStrength;
                tangentY = (tangentY / magnitude) * magnitude * smoothingStrength;
            }
            
            // Adjust control point position
            points[i].x = currX + tangentX * 0.1;
            points[i].y = currY + tangentY * 0.1;
        }
    }
    
    std::vector<BezierSegment> createBezierSegments(const std::vector<BezierControlPoint>& controlPoints) const {
        std::vector<BezierSegment> segments;
        
        for (size_t i = 0; i < controlPoints.size() - 1; ++i) {
            BezierSegment segment;
            
            // Start and end points
            segment.p0 = controlPoints[i];
            segment.p3 = controlPoints[i + 1];
            
            // Calculate intermediate control points
            double dx = segment.p3.x - segment.p0.x;
            double dy = segment.p3.y - segment.p0.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // Control points at 1/3 and 2/3 distance
            segment.p1.x = segment.p0.x + dx * 0.33;
            segment.p1.y = segment.p0.y + dy * 0.33;
            segment.p2.x = segment.p0.x + dx * 0.67;
            segment.p2.y = segment.p0.y + dy * 0.67;
            
            // Apply curvature based on angle between segments
            if (i > 0 && i < controlPoints.size() - 2) {
                double curvature = calculateCurvature(i, controlPoints);
                adjustControlPointsForCurvature(segment, curvature);
            }
            
            // Generate interpolated points
            interpolateBezierSegment(segment);
            
            segments.push_back(segment);
        }
        
        return segments;
    }
    
    double calculateCurvature(size_t index, const std::vector<BezierControlPoint>& points) const {
        if (index == 0 || index >= points.size() - 1) return 0.0;
        
        const auto& prev = points[index - 1];
        const auto& curr = points[index];
        const auto& next = points[index + 1];
        
        return MathUtils::calculateCurvature(prev.x, prev.y, curr.x, curr.y, next.x, next.y);
    }
    
    void adjustControlPointsForCurvature(BezierSegment& segment, double curvature) const {
        double adjustment = std::abs(curvature) * smoothingStrength * 0.5;
        
        // Adjust control points to create smoother curves
        double midX = (segment.p0.x + segment.p3.x) * 0.5;
        double midY = (segment.p0.y + segment.p3.y) * 0.5;
        
        // Move control points slightly toward the midpoint for smoothing
        segment.p1.x = segment.p1.x + (midX - segment.p1.x) * adjustment;
        segment.p1.y = segment.p1.y + (midY - segment.p1.y) * adjustment;
        segment.p2.x = segment.p2.x + (midX - segment.p2.x) * adjustment;
        segment.p2.y = segment.p2.y + (midY - segment.p2.y) * adjustment;
    }
    
    void interpolateBezierSegment(BezierSegment& segment) const {
        segment.interpolatedPoints.clear();
        
        for (int t = 0; t <= interpolationResolution; ++t) {
            double u = static_cast<double>(t) / interpolationResolution;
            
            // Cubic Bezier formula: B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
            double u1 = 1.0 - u;
            double u1_2 = u1 * u1;
            double u1_3 = u1_2 * u1;
            double u_2 = u * u;
            double u_3 = u_2 * u;
            
            double x = u1_3 * segment.p0.x + 3 * u1_2 * u * segment.p1.x + 
                      3 * u1 * u_2 * segment.p2.x + u_3 * segment.p3.x;
            double y = u1_3 * segment.p0.y + 3 * u1_2 * u * segment.p1.y + 
                      3 * u1 * u_2 * segment.p2.y + u_3 * segment.p3.y;
            
            segment.interpolatedPoints.emplace_back(x, y);
        }
        
        // Calculate segment length
        segment.segmentLength = 0.0;
        for (size_t i = 1; i < segment.interpolatedPoints.size(); ++i) {
            double dx = segment.interpolatedPoints[i].first - segment.interpolatedPoints[i-1].first;
            double dy = segment.interpolatedPoints[i].second - segment.interpolatedPoints[i-1].second;
            segment.segmentLength += std::sqrt(dx * dx + dy * dy);
        }
    }
    
    std::vector<int> mapSmoothPointsToNodes(const std::vector<BezierSegment>& segments) const {
        std::vector<int> smoothedPath;
        std::vector<std::pair<double, double>> allPoints;
        
        // Collect all interpolated points
        for (const auto& segment : segments) {
            for (const auto& point : segment.interpolatedPoints) {
                allPoints.push_back(point);
            }
        }
        
        // Map points to nearest nodes
        for (const auto& point : allPoints) {
            int nearestNode = findNearestNode(point.first, point.second);
            if (nearestNode != -1 && 
                (smoothedPath.empty() || smoothedPath.back() != nearestNode)) {
                smoothedPath.push_back(nearestNode);
            }
        }
        
        return smoothedPath;
    }
    
    int findNearestNode(double x, double y) const {
        int nearestId = -1;
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int nodeId : graph->getAllNodeIds()) {
            const Node& node = graph->getNode(nodeId);
            double distance = std::sqrt(std::pow(node.getX() - x, 2) + std::pow(node.getY() - y, 2));
            
            if (distance < minDistance) {
                minDistance = distance;
                nearestId = nodeId;
            }
        }
        
        return nearestId;
    }
    
    bool isFixedWaypoint(int nodeId) const {
        return std::find(fixedWaypoints.begin(), fixedWaypoints.end(), nodeId) 
               != fixedWaypoints.end();
    }
    
public:
    BezierCurveSmoother(const Graph* environment) 
        : graph(environment), interpolationResolution(20), smoothingStrength(0.3), 
          preserveEndpoints(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[BEZIER_SMOOTHER] Bezier curve smoother initialized" << std::endl;
    }
    
    std::vector<int> smoothPath(const std::vector<int>& originalPath) {
        if (originalPath.size() < 3) {
            std::cout << "[BEZIER_SMOOTHER] Path too short for Bezier smoothing" << std::endl;
            return originalPath;
        }
        
        std::cout << "[BEZIER_SMOOTHER] Applying Bezier curve smoothing to " 
                  << originalPath.size() << " nodes" << std::endl;
        
        // Generate control points from path
        std::vector<BezierControlPoint> controlPoints = generateControlPoints(originalPath);
        
        // Calculate tangent-based adjustments
        calculateTangentControlPoints(controlPoints);
        
        // Create Bezier segments
        std::vector<BezierSegment> segments = createBezierSegments(controlPoints);
        
        // Map smooth curve back to graph nodes
        std::vector<int> smoothedPath = mapSmoothPointsToNodes(segments);
        
        // Ensure path connectivity
        smoothedPath = ensurePathConnectivity(smoothedPath);
        
        std::cout << "[BEZIER_SMOOTHER] Smoothing completed, path length: " 
                  << originalPath.size() << " -> " << smoothedPath.size() << std::endl;
        
        return smoothedPath;
    }
    
    void setInterpolationResolution(int resolution) {
        interpolationResolution = std::max(5, resolution);
        std::cout << "[BEZIER_SMOOTHER] Interpolation resolution set to " << resolution << std::endl;
    }
    
    void setSmoothingStrength(double strength) {
        smoothingStrength = MathUtils::clamp(strength, 0.0, 1.0);
        std::cout << "[BEZIER_SMOOTHER] Smoothing strength set to " << strength << std::endl;
    }
    
    void setFixedWaypoints(const std::vector<int>& waypoints) {
        fixedWaypoints = waypoints;
        std::cout << "[BEZIER_SMOOTHER] Set " << waypoints.size() << " fixed waypoints" << std::endl;
    }
    
    void enableEndpointPreservation(bool preserve) {
        preserveEndpoints = preserve;
        std::cout << "[BEZIER_SMOOTHER] Endpoint preservation " 
                  << (preserve ? "enabled" : "disabled") << std::endl;
    }
    
private:
    std::vector<int> ensurePathConnectivity(const std::vector<int>& path) const {
        std::vector<int> connectedPath;
        if (path.empty()) return connectedPath;
        
        connectedPath.push_back(path[0]);
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(connectedPath.back(), path[i])) {
                // Find intermediate path
                std::vector<int> intermediatePath = findShortestPath(connectedPath.back(), path[i]);
                if (!intermediatePath.empty()) {
                    connectedPath.insert(connectedPath.end(), 
                                       intermediatePath.begin() + 1, intermediatePath.end());
                } else {
                    connectedPath.push_back(path[i]); // Force connection
                }
            } else {
                connectedPath.push_back(path[i]);
            }
        }
        
        return connectedPath;
    }
    
    std::vector<int> findShortestPath(int start, int end) const {
        // Simple BFS for shortest path
        std::queue<std::vector<int>> pathQueue;
        std::unordered_set<int> visited;
        
        pathQueue.push({start});
        visited.insert(start);
        
        while (!pathQueue.empty()) {
            std::vector<int> currentPath = pathQueue.front();
            pathQueue.pop();
            
            int currentNode = currentPath.back();
            if (currentNode == end) {
                return currentPath;
            }
            
            if (currentPath.size() > 10) continue; // Limit search depth
            
            for (int neighbor : graph->getNeighbors(currentNode)) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(neighbor);
                    pathQueue.push(newPath);
                }
            }
        }
        
        return {}; // No path found
    }
};