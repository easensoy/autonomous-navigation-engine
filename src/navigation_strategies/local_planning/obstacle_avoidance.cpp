#include "navigation_strategies/LocalPathPlanner.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <corecrt_math_defines.h>

class ObstacleAvoidance {
private:
    const Graph* graph;
    
    struct AvoidanceVector {
        double x, y;
        double magnitude;
        int sourceObstacle;
        
        AvoidanceVector(double xVal = 0.0, double yVal = 0.0, int source = -1)
            : x(xVal), y(yVal), magnitude(std::sqrt(x*x + y*y)), sourceObstacle(source) {}
    };
    
    struct SafetyZone {
        int centerNode;
        double radius;
        double severity;
        bool isDynamic;
        
        SafetyZone(int node, double r, double sev = 1.0, bool dynamic = false)
            : centerNode(node), radius(r), severity(sev), isDynamic(dynamic) {}
    };
    
    double avoidanceRadius;
    double safetyMargin;
    double emergencyStopDistance;
    double maxDeviationAngle;
    bool enablePreemptiveAvoidance;
    bool enableSmoothTrajectories;
    
    std::vector<SafetyZone> staticObstacles;
    std::vector<SafetyZone> dynamicObstacles;
    
    AvoidanceVector calculateRepulsiveForce(const Node& currentPos, const Node& obstacle, 
                                          double obstacleRadius, double severity) const {
        double dx = currentPos.getX() - obstacle.getX();
        double dy = currentPos.getY() - obstacle.getY();
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > avoidanceRadius) {
            return AvoidanceVector(0.0, 0.0);
        }
        
        if (distance < 0.01) distance = 0.01; // Avoid division by zero
        
        // Repulsive force inversely proportional to distance squared
        double forceMagnitude = severity * avoidanceRadius / (distance * distance);
        
        // Normalize direction
        double forceX = (dx / distance) * forceMagnitude;
        double forceY = (dy / distance) * forceMagnitude;
        
        return AvoidanceVector(forceX, forceY);
    }
    
    AvoidanceVector calculateTotalAvoidanceForce(const Node& currentPos, 
                                               const std::vector<int>& obstacles) const {
        AvoidanceVector totalForce;
        
        for (int obstacleId : obstacles) {
            const Node& obstacle = graph->getNode(obstacleId);
            AvoidanceVector force = calculateRepulsiveForce(currentPos, obstacle, safetyMargin, 1.0);
            
            totalForce.x += force.x;
            totalForce.y += force.y;
        }
        
        totalForce.magnitude = std::sqrt(totalForce.x*totalForce.x + totalForce.y*totalForce.y);
        return totalForce;
    }
    
    std::vector<int> findAvoidancePath(int currentNode, int targetNode, 
                                     const AvoidanceVector& avoidanceForce) const {
        const Node& current = graph->getNode(currentNode);
        const Node& target = graph->getNode(targetNode);
        
        // Calculate desired direction to target
        double targetDx = target.getX() - current.getX();
        double targetDy = target.getY() - current.getY();
        double targetDistance = std::sqrt(targetDx*targetDx + targetDy*targetDy);
        
        if (targetDistance < 0.01) {
            return {currentNode};
        }
        
        // Normalize target direction
        targetDx /= targetDistance;
        targetDy /= targetDistance;
        
        // Combine target direction with avoidance force
        double combinedX = targetDx + avoidanceForce.x;
        double combinedY = targetDy + avoidanceForce.y;
        double combinedMagnitude = std::sqrt(combinedX*combinedX + combinedY*combinedY);
        
        if (combinedMagnitude > 0.01) {
            combinedX /= combinedMagnitude;
            combinedY /= combinedMagnitude;
        }
        
        // Find best neighbor in combined direction
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        int bestNeighbor = -1;
        double bestAlignment = -1.0;
        
        for (int neighbor : neighbors) {
            const Node& neighborNode = graph->getNode(neighbor);
            double neighborDx = neighborNode.getX() - current.getX();
            double neighborDy = neighborNode.getY() - current.getY();
            double neighborDistance = std::sqrt(neighborDx*neighborDx + neighborDy*neighborDy);
            
            if (neighborDistance > 0.01) {
                neighborDx /= neighborDistance;
                neighborDy /= neighborDistance;
                
                double alignment = neighborDx * combinedX + neighborDy * combinedY;
                
                if (alignment > bestAlignment) {
                    bestAlignment = alignment;
                    bestNeighbor = neighbor;
                }
            }
        }
        
        if (bestNeighbor != -1) {
            return {currentNode, bestNeighbor};
        }
        
        return {currentNode};
    }
    
    std::vector<int> findEmergencyPath(int currentNode, const std::vector<int>& obstacles) const {
        std::cout << "[OBSTACLE_AVOID] Finding emergency path from " << currentNode << std::endl;
        
        // Find the safest neighboring node
        std::vector<int> neighbors = graph->getNeighbors(currentNode);
        int safestNeighbor = -1;
        double maxSafetyDistance = 0.0;
        
        for (int neighbor : neighbors) {
            double minObstacleDistance = calculateMinObstacleDistance(neighbor, obstacles);
            
            if (minObstacleDistance > maxSafetyDistance) {
                maxSafetyDistance = minObstacleDistance;
                safestNeighbor = neighbor;
            }
        }
        
        if (safestNeighbor != -1) {
            std::cout << "[OBSTACLE_AVOID] Emergency path found to node " << safestNeighbor << std::endl;
            return {currentNode, safestNeighbor};
        }
        
        std::cout << "[OBSTACLE_AVOID] No safe emergency path found" << std::endl;
        return {currentNode};
    }
    
    double calculateMinObstacleDistance(int nodeId, const std::vector<int>& obstacles) const {
        const Node& node = graph->getNode(nodeId);
        double minDistance = std::numeric_limits<double>::infinity();
        
        for (int obstacleId : obstacles) {
            const Node& obstacle = graph->getNode(obstacleId);
            double distance = node.euclideanDistance(obstacle);
            minDistance = std::min(minDistance, distance);
        }
        
        return minDistance;
    }
    
    std::vector<int> smoothAvoidancePath(const std::vector<int>& rawPath) const {
        if (!enableSmoothTrajectories || rawPath.size() < 3) {
            return rawPath;
        }
        
        std::vector<int> smoothedPath;
        smoothedPath.push_back(rawPath[0]);
        
        for (size_t i = 1; i < rawPath.size() - 1; ++i) {
            // Check if we can smooth by skipping intermediate nodes
            bool canSmooth = true;
            for (size_t j = i + 1; j < rawPath.size(); ++j) {
                if (!graph->hasEdge(smoothedPath.back(), rawPath[j])) {
                    canSmooth = false;
                    break;
                }
                
                if (j - i > 2) break; // Limit smoothing distance
            }
            
            if (!canSmooth) {
                smoothedPath.push_back(rawPath[i]);
            }
        }
        
        smoothedPath.push_back(rawPath.back());
        return smoothedPath;
    }
    
    bool isPathSafe(const std::vector<int>& path, const std::vector<int>& obstacles) const {
        for (int nodeId : path) {
            double minDistance = calculateMinObstacleDistance(nodeId, obstacles);
            if (minDistance < safetyMargin) {
                return false;
            }
        }
        return true;
    }
    
    std::vector<int> findAlternativeRoute(int currentNode, int targetNode, 
                                        const std::vector<int>& obstacles) const {
        std::cout << "[OBSTACLE_AVOID] Finding alternative route" << std::endl;
        
        // Use BFS to find alternative path avoiding obstacles
        std::queue<int> frontier;
        std::unordered_map<int, int> parent;
        std::unordered_set<int> visited;
        std::unordered_set<int> obstacleSet(obstacles.begin(), obstacles.end());
        
        frontier.push(currentNode);
        visited.insert(currentNode);
        parent[currentNode] = -1;
        
        while (!frontier.empty()) {
            int current = frontier.front();
            frontier.pop();
            
            if (current == targetNode) {
                // Reconstruct path
                std::vector<int> path;
                int node = targetNode;
                while (node != -1) {
                    path.push_back(node);
                    node = parent[node];
                }
                std::reverse(path.begin(), path.end());
                
                std::cout << "[OBSTACLE_AVOID] Alternative route found with " 
                          << path.size() << " nodes" << std::endl;
                return path;
            }
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end() && 
                    obstacleSet.find(neighbor) == obstacleSet.end()) {
                    
                    visited.insert(neighbor);
                    parent[neighbor] = current;
                    frontier.push(neighbor);
                }
            }
        }
        
        std::cout << "[OBSTACLE_AVOID] No alternative route found" << std::endl;
        return {currentNode};
    }
    
public:
    ObstacleAvoidance(const Graph* environment) 
        : graph(environment), avoidanceRadius(3.0), safetyMargin(1.0),
          emergencyStopDistance(0.5), maxDeviationAngle(M_PI/3),
          enablePreemptiveAvoidance(true), enableSmoothTrajectories(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[OBSTACLE_AVOID] Obstacle avoidance system initialized" << std::endl;
    }
    
    std::vector<int> avoidObstacles(int currentNode, int targetNode, 
                                  const std::vector<int>& obstacles) {
        std::cout << "[OBSTACLE_AVOID] Avoiding " << obstacles.size() 
                  << " obstacles from " << currentNode << " to " << targetNode << std::endl;
        
        if (obstacles.empty()) {
            std::cout << "[OBSTACLE_AVOID] No obstacles to avoid" << std::endl;
            return {currentNode, targetNode};
        }
        
        const Node& current = graph->getNode(currentNode);
        
        // Check for emergency situation
        double minObstacleDistance = calculateMinObstacleDistance(currentNode, obstacles);
        if (minObstacleDistance < emergencyStopDistance) {
            std::cout << "[OBSTACLE_AVOID] EMERGENCY: Too close to obstacle (" 
                      << minObstacleDistance << ")" << std::endl;
            return findEmergencyPath(currentNode, obstacles);
        }
        
        // Calculate avoidance forces
        AvoidanceVector totalForce = calculateTotalAvoidanceForce(current, obstacles);
        
        std::cout << "[OBSTACLE_AVOID] Total avoidance force: (" << totalForce.x 
                  << ", " << totalForce.y << "), magnitude: " << totalForce.magnitude << std::endl;
        
        // Generate avoidance path
        std::vector<int> avoidancePath;
        
        if (totalForce.magnitude > 0.1) {
            // Strong avoidance needed
            avoidancePath = findAvoidancePath(currentNode, targetNode, totalForce);
        } else {
            // Weak avoidance or preemptive
            if (enablePreemptiveAvoidance) {
                avoidancePath = findAlternativeRoute(currentNode, targetNode, obstacles);
            } else {
                avoidancePath = {currentNode, targetNode};
            }
        }
        
        // Verify path safety
        if (!isPathSafe(avoidancePath, obstacles)) {
            std::cout << "[OBSTACLE_AVOID] Generated path is unsafe, finding alternative" << std::endl;
            avoidancePath = findAlternativeRoute(currentNode, targetNode, obstacles);
        }
        
        // Apply smoothing if enabled
        if (enableSmoothTrajectories) {
            avoidancePath = smoothAvoidancePath(avoidancePath);
        }
        
        std::cout << "[OBSTACLE_AVOID] Generated avoidance path with " 
                  << avoidancePath.size() << " nodes" << std::endl;
        
        return avoidancePath;
    }
    
    bool checkImmediateDanger(int nodeId, const std::vector<int>& obstacles) const {
        double minDistance = calculateMinObstacleDistance(nodeId, obstacles);
        return minDistance < emergencyStopDistance;
    }
    
    double calculateSafetyClearance(int nodeId, const std::vector<int>& obstacles) const {
        return calculateMinObstacleDistance(nodeId, obstacles);
    }
    
    void addStaticObstacle(int nodeId, double radius, double severity = 1.0) {
        staticObstacles.emplace_back(nodeId, radius, severity, false);
        std::cout << "[OBSTACLE_AVOID] Added static obstacle at node " << nodeId 
                  << " with radius " << radius << std::endl;
    }
    
    void addDynamicObstacle(int nodeId, double radius, double severity = 1.0) {
        dynamicObstacles.emplace_back(nodeId, radius, severity, true);
        std::cout << "[OBSTACLE_AVOID] Added dynamic obstacle at node " << nodeId 
                  << " with radius " << radius << std::endl;
    }
    
    void clearDynamicObstacles() {
        dynamicObstacles.clear();
        std::cout << "[OBSTACLE_AVOID] Cleared all dynamic obstacles" << std::endl;
    }
    
    void setAvoidanceRadius(double radius) {
        avoidanceRadius = radius;
        std::cout << "[OBSTACLE_AVOID] Avoidance radius set to " << radius << std::endl;
    }
    
    void setSafetyMargin(double margin) {
        safetyMargin = margin;
        std::cout << "[OBSTACLE_AVOID] Safety margin set to " << margin << std::endl;
    }
    
    void setEmergencyStopDistance(double distance) {
        emergencyStopDistance = distance;
        std::cout << "[OBSTACLE_AVOID] Emergency stop distance set to " << distance << std::endl;
    }
    
    void enablePreemptiveMode(bool enable) {
        enablePreemptiveAvoidance = enable;
        std::cout << "[OBSTACLE_AVOID] Preemptive avoidance " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableTrajectorySmoothing(bool enable) {
        enableSmoothTrajectories = enable;
        std::cout << "[OBSTACLE_AVOID] Trajectory smoothing " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void printConfiguration() const {
        std::cout << "[OBSTACLE_AVOID] Configuration:" << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Avoidance radius: " << avoidanceRadius << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Safety margin: " << safetyMargin << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Emergency stop distance: " << emergencyStopDistance << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Static obstacles: " << staticObstacles.size() << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Dynamic obstacles: " << dynamicObstacles.size() << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Preemptive avoidance: " << enablePreemptiveAvoidance << std::endl;
        std::cout << "[OBSTACLE_AVOID]   Trajectory smoothing: " << enableSmoothTrajectories << std::endl;
    }
};