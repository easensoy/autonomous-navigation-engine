#include "path_operations/PathSmoother.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>

class SplineInterpolationSmoother {
private:
    const Graph* graph;
    
    struct SplinePoint {
        double x, y;
        double t; // Parameter value
        double curvature;
        bool isOriginal;
        int originalNodeId;
        
        SplinePoint(double x = 0, double y = 0, double t = 0, bool orig = false, int id = -1)
            : x(x), y(y), t(t), curvature(0.0), isOriginal(orig), originalNodeId(id) {}
    };
    
    struct CubicSplineSegment {
        double a, b, c, d; // Coefficients for x(t) = at³ + bt² + ct + d
        double ax, bx, cx, dx; // X component coefficients
        double ay, by, cy, dy; // Y component coefficients
        double tStart, tEnd;
        std::vector<SplinePoint> interpolatedPoints;
        
        CubicSplineSegment() : a(0), b(0), c(0), d(0), ax(0), bx(0), cx(0), dx(0),
                               ay(0), by(0), cy(0), dy(0), tStart(0), tEnd(1) {}
    };
    
    struct SplineResult {
        std::vector<CubicSplineSegment> segments;
        std::vector<int> smoothedPath;
        double totalLength;
        double maxCurvature;
        double averageCurvature;
        int interpolatedPoints;
        
        SplineResult() : totalLength(0.0), maxCurvature(0.0), 
                        averageCurvature(0.0), interpolatedPoints(0) {}
    };
    
    int interpolationDensity;
    double tensionParameter;
    bool preserveOriginalNodes;
    std::vector<int> fixedControlPoints;
    
    std::vector<SplinePoint> createControlPoints(const std::vector<int>& path) const {
        std::vector<SplinePoint> controlPoints;
        
        for (size_t i = 0; i < path.size(); ++i) {
            const Node& node = graph->getNode(path[i]);
            double t = static_cast<double>(i) / (path.size() - 1);
            
            controlPoints.emplace_back(node.getX(), node.getY(), t, true, path[i]);
        }
        
        return controlPoints;
    }
    
    std::vector<CubicSplineSegment> calculateCubicSpline(const std::vector<SplinePoint>& controlPoints) const {
        std::vector<CubicSplineSegment> segments;
        
        if (controlPoints.size() < 2) return segments;
        
        size_t n = controlPoints.size() - 1;
        
        // Calculate spline coefficients using natural spline conditions
        std::vector<double> h(n), alpha(n);
        std::vector<double> l(n + 1), mu(n + 1), z(n + 1);
        
        // Calculate intervals and differences
        for (size_t i = 0; i < n; ++i) {
            h[i] = controlPoints[i + 1].t - controlPoints[i].t;
        }
        
        // Calculate alpha values for X coordinates
        std::vector<double> alphaX(n), alphaY(n);
        for (size_t i = 1; i < n; ++i) {
            alphaX[i] = 3.0 * ((controlPoints[i + 1].x - controlPoints[i].x) / h[i] - 
                              (controlPoints[i].x - controlPoints[i - 1].x) / h[i - 1]);
            alphaY[i] = 3.0 * ((controlPoints[i + 1].y - controlPoints[i].y) / h[i] - 
                              (controlPoints[i].y - controlPoints[i - 1].y) / h[i - 1]);
        }
        
        // Solve tridiagonal system for X coordinates
        std::vector<double> cX = solveTridiagonal(h, alphaX, n);
        std::vector<double> cY = solveTridiagonal(h, alphaY, n);
        
        // Calculate remaining coefficients
        std::vector<double> bX(n), dX(n), bY(n), dY(n);
        
        for (size_t i = 0; i < n; ++i) {
            bX[i] = (controlPoints[i + 1].x - controlPoints[i].x) / h[i] - 
                    h[i] * (cX[i + 1] + 2.0 * cX[i]) / 3.0;
            dX[i] = (cX[i + 1] - cX[i]) / (3.0 * h[i]);
            
            bY[i] = (controlPoints[i + 1].y - controlPoints[i].y) / h[i] - 
                    h[i] * (cY[i + 1] + 2.0 * cY[i]) / 3.0;
            dY[i] = (cY[i + 1] - cY[i]) / (3.0 * h[i]);
        }
        
        // Create spline segments
        for (size_t i = 0; i < n; ++i) {
            CubicSplineSegment segment;
            
            segment.ax = controlPoints[i].x;
            segment.bx = bX[i];
            segment.cx = cX[i];
            segment.dx = dX[i];
            
            segment.ay = controlPoints[i].y;
            segment.by = bY[i];
            segment.cy = cY[i];
            segment.dy = dY[i];
            
            segment.tStart = controlPoints[i].t;
            segment.tEnd = controlPoints[i + 1].t;
            
            // Generate interpolated points for this segment
            interpolateSegment(segment);
            
            segments.push_back(segment);
        }
        
        return segments;
    }
    
    std::vector<double> solveTridiagonal(const std::vector<double>& h, 
                                       const std::vector<double>& alpha, size_t n) const {
        std::vector<double> l(n + 1), mu(n + 1), z(n + 1), c(n + 1);
        
        // Forward elimination
        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;
        
        for (size_t i = 1; i < n; ++i) {
            l[i] = 2.0 * (h[i - 1] + h[i]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }
        
        l[n] = 1.0;
        z[n] = 0.0;
        c[n] = 0.0;
        
        // Back substitution
        for (int i = n - 1; i >= 0; --i) {
            c[i] = z[i] - mu[i] * c[i + 1];
        }
        
        return c;
    }
    
    void interpolateSegment(CubicSplineSegment& segment) const {
        segment.interpolatedPoints.clear();
        
        double dt = (segment.tEnd - segment.tStart) / interpolationDensity;
        
        for (int i = 0; i <= interpolationDensity; ++i) {
            double t = segment.tStart + i * dt;
            double localT = t - segment.tStart;
            
            // Evaluate spline at parameter t
            double x = segment.ax + segment.bx * localT + 
                      segment.cx * localT * localT + 
                      segment.dx * localT * localT * localT;
            double y = segment.ay + segment.by * localT + 
                      segment.cy * localT * localT + 
                      segment.dy * localT * localT * localT;
            
            SplinePoint point(x, y, t);
            
            // Calculate curvature at this point
            point.curvature = calculatePointCurvature(segment, localT);
            
            segment.interpolatedPoints.push_back(point);
        }
    }
    
    double calculatePointCurvature(const CubicSplineSegment& segment, double t) const {
        // First derivatives
        double dx = segment.bx + 2.0 * segment.cx * t + 3.0 * segment.dx * t * t;
        double dy = segment.by + 2.0 * segment.cy * t + 3.0 * segment.dy * t * t;
        
        // Second derivatives
        double ddx = 2.0 * segment.cx + 6.0 * segment.dx * t;
        double ddy = 2.0 * segment.cy + 6.0 * segment.dy * t;
        
        // Curvature formula: κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
        double numerator = std::abs(dx * ddy - dy * ddx);
        double denominator = std::pow(dx * dx + dy * dy, 1.5);
        
        return (denominator > 0) ? numerator / denominator : 0.0;
    }
    
    std::vector<int> mapSplineToNodes(const std::vector<CubicSplineSegment>& segments) const {
        std::vector<int> smoothedPath;
        std::vector<std::pair<double, double>> allPoints;
        
        // Collect all interpolated points
        for (const auto& segment : segments) {
            for (const auto& point : segment.interpolatedPoints) {
                allPoints.emplace_back(point.x, point.y);
            }
        }
        
        // Map points to nodes, preserving original nodes if requested
        for (size_t i = 0; i < allPoints.size(); ++i) {
            int nearestNode = findNearestNode(allPoints[i].first, allPoints[i].second);
            
            if (nearestNode != -1) {
                if (smoothedPath.empty() || smoothedPath.back() != nearestNode) {
                    smoothedPath.push_back(nearestNode);
                }
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
    
    SplineResult performSplineInterpolation(const std::vector<int>& originalPath) {
        SplineResult result;
        
        // Create control points from original path
        std::vector<SplinePoint> controlPoints = createControlPoints(originalPath);
        
        // Apply tension parameter to adjust smoothness
        if (tensionParameter != 0.0) {
            applyTension(controlPoints);
        }
        
        // Calculate cubic spline
        result.segments = calculateCubicSpline(controlPoints);
        
        // Map spline to node path
        result.smoothedPath = mapSplineToNodes(result.segments);
        
        // Calculate metrics
        calculateSplineMetrics(result);
        
        // Ensure connectivity
        result.smoothedPath = ensurePathConnectivity(result.smoothedPath);
        
        return result;
    }
    
    void applyTension(std::vector<SplinePoint>& controlPoints) const {
        if (controlPoints.size() < 3) return;
        
        // Apply tension to intermediate control points
        for (size_t i = 1; i < controlPoints.size() - 1; ++i) {
            if (isFixedControlPoint(controlPoints[i].originalNodeId)) continue;
            
            // Calculate tension adjustment
            double prevX = controlPoints[i - 1].x;
            double prevY = controlPoints[i - 1].y;
            double nextX = controlPoints[i + 1].x;
            double nextY = controlPoints[i + 1].y;
            
            double midX = (prevX + nextX) * 0.5;
            double midY = (prevY + nextY) * 0.5;
            
            // Apply tension (0 = no change, 1 = move to midpoint)
            controlPoints[i].x += (midX - controlPoints[i].x) * tensionParameter * 0.5;
            controlPoints[i].y += (midY - controlPoints[i].y) * tensionParameter * 0.5;
        }
    }
    
    void calculateSplineMetrics(SplineResult& result) const {
        result.totalLength = 0.0;
        result.maxCurvature = 0.0;
        double totalCurvature = 0.0;
        result.interpolatedPoints = 0;
        
        for (const auto& segment : result.segments) {
            for (size_t i = 1; i < segment.interpolatedPoints.size(); ++i) {
                const auto& p1 = segment.interpolatedPoints[i - 1];
                const auto& p2 = segment.interpolatedPoints[i];
                
                double dx = p2.x - p1.x;
                double dy = p2.y - p1.y;
                result.totalLength += std::sqrt(dx * dx + dy * dy);
                
                result.maxCurvature = std::max(result.maxCurvature, p2.curvature);
                totalCurvature += p2.curvature;
                result.interpolatedPoints++;
            }
        }
        
        result.averageCurvature = (result.interpolatedPoints > 0) ? 
                                 totalCurvature / result.interpolatedPoints : 0.0;
    }
    
    bool isFixedControlPoint(int nodeId) const {
        return std::find(fixedControlPoints.begin(), fixedControlPoints.end(), nodeId) 
               != fixedControlPoints.end();
    }
    
public:
    SplineInterpolationSmoother(const Graph* environment) 
        : graph(environment), interpolationDensity(10), tensionParameter(0.0), 
          preserveOriginalNodes(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[SPLINE_SMOOTHER] Spline interpolation smoother initialized" << std::endl;
    }
    
    std::vector<int> smoothPath(const std::vector<int>& originalPath) {
        if (originalPath.size() < 3) {
            std::cout << "[SPLINE_SMOOTHER] Path too short for spline interpolation" << std::endl;
            return originalPath;
        }
        
        std::cout << "[SPLINE_SMOOTHER] Applying spline interpolation to " 
                  << originalPath.size() << " nodes" << std::endl;
        
        SplineResult result = performSplineInterpolation(originalPath);
        
        std::cout << "[SPLINE_SMOOTHER] Spline interpolation completed: " 
                  << result.interpolatedPoints << " interpolated points, "
                  << "max curvature: " << result.maxCurvature << std::endl;
        
        return result.smoothedPath;
    }
    
    void setInterpolationDensity(int density) {
        interpolationDensity = std::max(2, density);
        std::cout << "[SPLINE_SMOOTHER] Interpolation density set to " << density << std::endl;
    }
    
    void setTensionParameter(double tension) {
        tensionParameter = MathUtils::clamp(tension, 0.0, 1.0);
        std::cout << "[SPLINE_SMOOTHER] Tension parameter set to " << tension << std::endl;
    }
    
    void setFixedControlPoints(const std::vector<int>& points) {
        fixedControlPoints = points;
        std::cout << "[SPLINE_SMOOTHER] Set " << points.size() << " fixed control points" << std::endl;
    }
    
    void enableOriginalNodePreservation(bool preserve) {
        preserveOriginalNodes = preserve;
        std::cout << "[SPLINE_SMOOTHER] Original node preservation " 
                  << (preserve ? "enabled" : "disabled") << std::endl;
    }
    
private:
    std::vector<int> ensurePathConnectivity(const std::vector<int>& path) const {
        std::vector<int> connectedPath;
        if (path.empty()) return connectedPath;
        
        connectedPath.push_back(path[0]);
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasEdge(connectedPath.back(), path[i])) {
                std::vector<int> bridge = findShortestPath(connectedPath.back(), path[i]);
                if (!bridge.empty() && bridge.size() > 1) {
                    connectedPath.insert(connectedPath.end(), bridge.begin() + 1, bridge.end());
                } else {
                    connectedPath.push_back(path[i]);
                }
            } else {
                connectedPath.push_back(path[i]);
            }
        }
        
        return connectedPath;
    }
    
    std::vector<int> findShortestPath(int start, int end) const {
        std::queue<std::vector<int>> pathQueue;
        std::unordered_set<int> visited;
        
        pathQueue.push({start});
        visited.insert(start);
        
        while (!pathQueue.empty()) {
            std::vector<int> currentPath = pathQueue.front();
            pathQueue.pop();
            
            if (currentPath.back() == end) {
                return currentPath;
            }
            
            if (currentPath.size() > 8) continue;
            
            for (int neighbor : graph->getNeighbors(currentPath.back())) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(neighbor);
                    pathQueue.push(newPath);
                }
            }
        }
        
        return {};
    }
};