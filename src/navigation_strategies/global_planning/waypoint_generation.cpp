#include "navigation_strategies/GlobalPathPlanner.hpp"
#include "pathfinding_algorithms/AStar.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_set>

class WaypointGenerator {
private:
    const Graph* graph;
    std::unique_ptr<AStar> aStar;
    
    struct Waypoint {
        int nodeId;
        double x, y;
        double heading;
        double speed;
        double curvature;
        bool isCritical;
        
        Waypoint(int id, double xPos, double yPos) 
            : nodeId(id), x(xPos), y(yPos), heading(0.0), 
              speed(1.0), curvature(0.0), isCritical(false) {}
    };
    
    double maxWaypointSpacing;
    double minWaypointSpacing;
    double curvatureThreshold;
    bool generateHeadings;
    bool adaptiveSpacing;
    
    std::vector<Waypoint> generateUniformWaypoints(const std::vector<int>& path, double spacing) {
        std::cout << "[WAYPOINT_GEN] Generating uniform waypoints with spacing " << spacing << std::endl;
        
        std::vector<Waypoint> waypoints;
        
        if (path.empty()) return waypoints;
        
        // Add start waypoint
        const Node& startNode = graph->getNode(path[0]);
        waypoints.emplace_back(path[0], startNode.getX(), startNode.getY());
        waypoints.back().isCritical = true;
        
        double accumulatedDistance = 0.0;
        
        for (size_t i = 1; i < path.size(); ++i) {
            const Node& prevNode = graph->getNode(path[i-1]);
            const Node& currNode = graph->getNode(path[i]);
            
            double segmentLength = prevNode.euclideanDistance(currNode);
            accumulatedDistance += segmentLength;
            
            // Add waypoints along this segment
            while (accumulatedDistance >= spacing) {
                double ratio = (segmentLength - (accumulatedDistance - spacing)) / segmentLength;
                double waypointX = prevNode.getX() + ratio * (currNode.getX() - prevNode.getX());
                double waypointY = prevNode.getY() + ratio * (currNode.getY() - prevNode.getY());
                
                Waypoint wp(-1, waypointX, waypointY);
                waypoints.push_back(wp);
                
                accumulatedDistance -= spacing;
            }
            
            // Check if current node should be a waypoint
            if (i == path.size() - 1 || shouldAddNodeAsWaypoint(path, i)) {
                waypoints.emplace_back(path[i], currNode.getX(), currNode.getY());
                if (i == path.size() - 1) {
                    waypoints.back().isCritical = true;
                }
                accumulatedDistance = 0.0;
            }
        }
        
        std::cout << "[WAYPOINT_GEN] Generated " << waypoints.size() << " uniform waypoints" << std::endl;
        return waypoints;
    }
    
    std::vector<Waypoint> generateAdaptiveWaypoints(const std::vector<int>& path) {
        std::cout << "[WAYPOINT_GEN] Generating adaptive waypoints" << std::endl;
        
        std::vector<Waypoint> waypoints;
        if (path.empty()) return waypoints;
        
        // Add start
        const Node& startNode = graph->getNode(path[0]);
        waypoints.emplace_back(path[0], startNode.getX(), startNode.getY());
        waypoints.back().isCritical = true;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            const Node& prevNode = graph->getNode(path[i-1]);
            const Node& currNode = graph->getNode(path[i]);
            const Node& nextNode = graph->getNode(path[i+1]);
            
            // Calculate curvature at this point
            double curvature = calculateCurvature(
                prevNode.getX(), prevNode.getY(),
                currNode.getX(), currNode.getY(),
                nextNode.getX(), nextNode.getY()
            );
            
            // Determine spacing based on curvature
            double spacing = minWaypointSpacing + 
                           (maxWaypointSpacing - minWaypointSpacing) * 
                           std::exp(-curvature / curvatureThreshold);
            
            // Check if we should add a waypoint
            double distanceFromLast = 0.0;
            if (!waypoints.empty()) {
                distanceFromLast = std::sqrt(
                    std::pow(currNode.getX() - waypoints.back().x, 2) +
                    std::pow(currNode.getY() - waypoints.back().y, 2)
                );
            }
            
            if (distanceFromLast >= spacing || curvature > curvatureThreshold) {
                Waypoint wp(path[i], currNode.getX(), currNode.getY());
                wp.curvature = curvature;
                wp.isCritical = (curvature > curvatureThreshold * 2.0);
                waypoints.push_back(wp);
            }
        }
        
        // Add goal
        const Node& goalNode = graph->getNode(path.back());
        waypoints.emplace_back(path.back(), goalNode.getX(), goalNode.getY());
        waypoints.back().isCritical = true;
        
        std::cout << "[WAYPOINT_GEN] Generated " << waypoints.size() << " adaptive waypoints" << std::endl;
        return waypoints;
    }
    
    std::vector<Waypoint> generateCriticalWaypoints(const std::vector<int>& path) {
        std::cout << "[WAYPOINT_GEN] Generating critical waypoints only" << std::endl;
        
        std::vector<Waypoint> waypoints;
        if (path.empty()) return waypoints;
        
        // Always include start and goal
        const Node& startNode = graph->getNode(path[0]);
        waypoints.emplace_back(path[0], startNode.getX(), startNode.getY());
        waypoints.back().isCritical = true;
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            if (isCriticalPoint(path, i)) {
                const Node& node = graph->getNode(path[i]);
                Waypoint wp(path[i], node.getX(), node.getY());
                wp.isCritical = true;
                waypoints.push_back(wp);
            }
        }
        
        const Node& goalNode = graph->getNode(path.back());
        waypoints.emplace_back(path.back(), goalNode.getX(), goalNode.getY());
        waypoints.back().isCritical = true;
        
        std::cout << "[WAYPOINT_GEN] Generated " << waypoints.size() << " critical waypoints" << std::endl;
        return waypoints;
    }
    
    std::vector<Waypoint> generateSplineWaypoints(const std::vector<int>& path, int numWaypoints) {
        std::cout << "[WAYPOINT_GEN] Generating " << numWaypoints << " spline-interpolated waypoints" << std::endl;
        
        std::vector<Waypoint> waypoints;
        if (path.size() < 2) return waypoints;
        
        // Create control points from path
        std::vector<std::pair<double, double>> controlPoints;
        for (int nodeId : path) {
            const Node& node = graph->getNode(nodeId);
            controlPoints.emplace_back(node.getX(), node.getY());
        }
        
        // Generate interpolated points
        std::vector<std::pair<double, double>> interpolatedPoints = 
            MathUtils::interpolatePath(controlPoints, numWaypoints);
        
        for (size_t i = 0; i < interpolatedPoints.size(); ++i) {
            Waypoint wp(-1, interpolatedPoints[i].first, interpolatedPoints[i].second);
            
            // Mark start and end as critical
            if (i == 0 || i == interpolatedPoints.size() - 1) {
                wp.isCritical = true;
            }
            
            waypoints.push_back(wp);
        }
        
        std::cout << "[WAYPOINT_GEN] Generated " << waypoints.size() << " spline waypoints" << std::endl;
        return waypoints;
    }
    
    void calculateHeadings(std::vector<Waypoint>& waypoints) {
        if (waypoints.size() < 2) return;
        
        std::cout << "[WAYPOINT_GEN] Calculating headings for waypoints" << std::endl;
        
        for (size_t i = 0; i < waypoints.size(); ++i) {
            if (i == 0) {
                // First waypoint - heading towards next
                double dx = waypoints[i+1].x - waypoints[i].x;
                double dy = waypoints[i+1].y - waypoints[i].y;
                waypoints[i].heading = std::atan2(dy, dx);
            } else if (i == waypoints.size() - 1) {
                // Last waypoint - heading from previous
                double dx = waypoints[i].x - waypoints[i-1].x;
                double dy = waypoints[i].y - waypoints[i-1].y;
                waypoints[i].heading = std::atan2(dy, dx);
            } else {
                // Middle waypoint - average of incoming and outgoing
                double dx1 = waypoints[i].x - waypoints[i-1].x;
                double dy1 = waypoints[i].y - waypoints[i-1].y;
                double dx2 = waypoints[i+1].x - waypoints[i].x;
                double dy2 = waypoints[i+1].y - waypoints[i].y;
                
                double heading1 = std::atan2(dy1, dx1);
                double heading2 = std::atan2(dy2, dx2);
                
                // Average angles properly (handle wraparound)
                waypoints[i].heading = MathUtils::normalizeAngle((heading1 + heading2) / 2.0);
            }
        }
    }
    
    void calculateSpeeds(std::vector<Waypoint>& waypoints) {
        if (waypoints.empty()) return;
        
        std::cout << "[WAYPOINT_GEN] Calculating speeds for waypoints" << std::endl;
        
        const double maxSpeed = 2.0;
        const double minSpeed = 0.5;
        
        for (auto& waypoint : waypoints) {
            // Reduce speed based on curvature
            double speedFactor = 1.0 / (1.0 + waypoint.curvature * 5.0);
            waypoint.speed = minSpeed + (maxSpeed - minSpeed) * speedFactor;
            
            // Critical waypoints get reduced speed
            if (waypoint.isCritical) {
                waypoint.speed *= 0.7;
            }
        }
    }
    
    double calculateCurvature(double x1, double y1, double x2, double y2, double x3, double y3) {
        return MathUtils::calculateCurvature(x1, y1, x2, y2, x3, y3);
    }
    
    bool shouldAddNodeAsWaypoint(const std::vector<int>& path, size_t index) {
        if (index >= path.size()) return false;
        
        // Add waypoint at junctions (nodes with multiple connections)
        std::vector<int> neighbors = graph->getNeighbors(path[index]);
        if (neighbors.size() > 2) {
            return true;
        }
        
        // Add waypoint at significant direction changes
        if (index > 0 && index < path.size() - 1) {
            const Node& prev = graph->getNode(path[index-1]);
            const Node& curr = graph->getNode(path[index]);
            const Node& next = graph->getNode(path[index+1]);
            
            double curvature = calculateCurvature(
                prev.getX(), prev.getY(),
                curr.getX(), curr.getY(),
                next.getX(), next.getY()
            );
            
            return curvature > curvatureThreshold;
        }
        
        return false;
    }
    
    bool isCriticalPoint(const std::vector<int>& path, size_t index) {
        if (index >= path.size()) return false;
        
        // Junction points
        std::vector<int> neighbors = graph->getNeighbors(path[index]);
        if (neighbors.size() > 2) return true;
        
        // High curvature points
        if (index > 0 && index < path.size() - 1) {
            const Node& prev = graph->getNode(path[index-1]);
            const Node& curr = graph->getNode(path[index]);
            const Node& next = graph->getNode(path[index+1]);
            
            double curvature = calculateCurvature(
                prev.getX(), prev.getY(),
                curr.getX(), curr.getY(),
                next.getX(), next.getY()
            );
            
            return curvature > curvatureThreshold * 2.0;
        }
        
        return false;
    }
    
    std::vector<int> waypointsToNodePath(const std::vector<Waypoint>& waypoints) {
        std::vector<int> nodePath;
        
        for (const auto& waypoint : waypoints) {
            if (waypoint.nodeId != -1) {
                nodePath.push_back(waypoint.nodeId);
            } else {
                // Find nearest node to this waypoint
                int nearestNode = findNearestNode(waypoint.x, waypoint.y);
                if (nearestNode != -1) {
                    nodePath.push_back(nearestNode);
                }
            }
        }
        
        return nodePath;
    }
    
    int findNearestNode(double x, double y) {
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
    
public:
    WaypointGenerator(const Graph* environment) 
        : graph(environment), maxWaypointSpacing(5.0), minWaypointSpacing(1.0),
          curvatureThreshold(0.5), generateHeadings(true), adaptiveSpacing(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        aStar = std::make_unique<AStar>(graph);
        std::cout << "[WAYPOINT_GEN] Waypoint generator initialized" << std::endl;
    }
    
    std::vector<int> generateWaypoints(const std::vector<int>& path, const std::string& method = "adaptive") {
        std::cout << "[WAYPOINT_GEN] Generating waypoints for path with " << path.size() 
                  << " nodes using " << method << " method" << std::endl;
        
        if (path.empty()) return {};
        
        std::vector<Waypoint> waypoints;
        
        if (method == "uniform") {
            waypoints = generateUniformWaypoints(path, maxWaypointSpacing);
        } else if (method == "adaptive") {
            waypoints = generateAdaptiveWaypoints(path);
        } else if (method == "critical") {
            waypoints = generateCriticalWaypoints(path);
        } else if (method == "spline") {
            int numWaypoints = static_cast<int>(path.size() * 1.5);
            waypoints = generateSplineWaypoints(path, numWaypoints);
        } else {
            waypoints = generateAdaptiveWaypoints(path);
        }
        
        if (generateHeadings) {
            calculateHeadings(waypoints);
        }
        
        calculateSpeeds(waypoints);
        
        return waypointsToNodePath(waypoints);
    }
    
    std::vector<int> generateIntermediateWaypoints(int startId, int goalId, int numWaypoints) {
        std::cout << "[WAYPOINT_GEN] Generating " << numWaypoints 
                  << " intermediate waypoints from " << startId << " to " << goalId << std::endl;
        
        // First find a path
        std::vector<int> basePath = aStar->findPath(startId, goalId);
        if (basePath.empty()) {
            std::cout << "[WAYPOINT_GEN] No base path found" << std::endl;
            return {};
        }
        
        // Generate spline waypoints
        std::vector<Waypoint> waypoints = generateSplineWaypoints(basePath, numWaypoints);
        
        if (generateHeadings) {
            calculateHeadings(waypoints);
        }
        
        return waypointsToNodePath(waypoints);
    }
    
    std::vector<int> addWaypointsToPath(const std::vector<int>& originalPath, double spacing) {
        std::cout << "[WAYPOINT_GEN] Adding waypoints to existing path with spacing " << spacing << std::endl;
        
        std::vector<Waypoint> waypoints = generateUniformWaypoints(originalPath, spacing);
        return waypointsToNodePath(waypoints);
    }
    
    std::vector<int> optimizeWaypoints(const std::vector<int>& waypoints) {
        std::cout << "[WAYPOINT_GEN] Optimizing waypoint sequence" << std::endl;
        
        if (waypoints.size() < 3) return waypoints;
        
        std::vector<int> optimized;
        optimized.push_back(waypoints[0]); // Keep start
        
        for (size_t i = 1; i < waypoints.size() - 1; ++i) {
            // Check if we can skip this waypoint
            bool canSkip = graph->hasEdge(optimized.back(), waypoints[i+1]);
            
            if (!canSkip) {
                optimized.push_back(waypoints[i]);
            }
        }
        
        optimized.push_back(waypoints.back()); // Keep goal
        
        std::cout << "[WAYPOINT_GEN] Optimized " << waypoints.size() 
                  << " waypoints to " << optimized.size() << std::endl;
        
        return optimized;
    }
    
    void setWaypointSpacing(double minSpacing, double maxSpacing) {
        minWaypointSpacing = minSpacing;
        maxWaypointSpacing = maxSpacing;
        std::cout << "[WAYPOINT_GEN] Waypoint spacing set to [" << minSpacing 
                  << ", " << maxSpacing << "]" << std::endl;
    }
    
    void setCurvatureThreshold(double threshold) {
        curvatureThreshold = threshold;
        std::cout << "[WAYPOINT_GEN] Curvature threshold set to " << threshold << std::endl;
    }
    
    void enableHeadingCalculation(bool enable) {
        generateHeadings = enable;
        std::cout << "[WAYPOINT_GEN] Heading calculation " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableAdaptiveSpacing(bool enable) {
        adaptiveSpacing = enable;
        std::cout << "[WAYPOINT_GEN] Adaptive spacing " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    double calculateWaypointDensity(const std::vector<int>& waypoints) {
        if (waypoints.size() < 2) return 0.0;
        
        double totalLength = 0.0;
        for (size_t i = 1; i < waypoints.size(); ++i) {
            const Node& from = graph->getNode(waypoints[i-1]);
            const Node& to = graph->getNode(waypoints[i]);
            totalLength += from.euclideanDistance(to);
        }
        
        return static_cast<double>(waypoints.size()) / totalLength;
    }
    
    bool validateWaypointPath(const std::vector<int>& waypoints) {
        if (waypoints.empty()) return true;
        
        for (size_t i = 1; i < waypoints.size(); ++i) {
            if (!graph->hasNode(waypoints[i-1]) || !graph->hasNode(waypoints[i])) {
                return false;
            }
        }
        
        return true;
    }
    
    void printWaypointStatistics(const std::vector<int>& waypoints) {
        std::cout << "[WAYPOINT_GEN] Waypoint Statistics:" << std::endl;
        std::cout << "[WAYPOINT_GEN]   Total waypoints: " << waypoints.size() << std::endl;
        
        if (waypoints.size() >= 2) {
            double density = calculateWaypointDensity(waypoints);
            std::cout << "[WAYPOINT_GEN]   Density: " << density << " waypoints/unit" << std::endl;
            
            bool isValid = validateWaypointPath(waypoints);
            std::cout << "[WAYPOINT_GEN]   Valid path: " << (isValid ? "Yes" : "No") << std::endl;
        }
    }
};