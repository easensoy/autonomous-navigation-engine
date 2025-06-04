#include "path_operations/PathOptimizer.hpp"
#include "utilities/MathUtils.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <chrono>
#include <unordered_map>

class TimeOptimalPlanner {
private:
    const Graph* graph;
    
    struct VelocityProfile {
        double maxVelocity;
        double maxAcceleration;
        double maxDeceleration;
        double cruiseVelocity;
        double currentVelocity;
        
        VelocityProfile() : maxVelocity(10.0), maxAcceleration(2.0), maxDeceleration(3.0),
                           cruiseVelocity(8.0), currentVelocity(0.0) {}
    };
    
    struct SegmentTiming {
        int fromNode;
        int toNode;
        double distance;
        double optimalTime;
        double averageVelocity;
        double accelerationTime;
        double cruiseTime;
        double decelerationTime;
        double congestionFactor;
        double weatherFactor;
        
        SegmentTiming() : fromNode(-1), toNode(-1), distance(0.0), optimalTime(0.0),
                         averageVelocity(0.0), accelerationTime(0.0), cruiseTime(0.0),
                         decelerationTime(0.0), congestionFactor(1.0), weatherFactor(1.0) {}
    };
    
    struct TimeOptimizationResult {
        std::vector<int> optimizedPath;
        std::vector<SegmentTiming> timingProfile;
        double totalTime;
        double originalTime;
        double timeSavings;
        double timeSavingsPercentage;
        int optimizationSteps;
        std::string method;
        std::vector<std::pair<int, double>> velocityChanges;
        std::vector<int> congestionAvoidanceNodes;
        
        TimeOptimizationResult() : totalTime(0.0), originalTime(0.0), timeSavings(0.0),
                                  timeSavingsPercentage(0.0), optimizationSteps(0) {}
    };
    
    struct DynamicObstacle {
        int nodeId;
        std::chrono::steady_clock::time_point startTime;
        std::chrono::duration<double> duration;
        double impactRadius;
        double velocityReduction;
        bool isActive;
        
        DynamicObstacle(int id, double radius, double reduction) 
            : nodeId(id), startTime(std::chrono::steady_clock::now()),
              duration(std::chrono::minutes(10)), impactRadius(radius),
              velocityReduction(reduction), isActive(true) {}
    };
    
    struct TrafficModel {
        std::unordered_map<int, double> nodeCongestionLevels;
        std::unordered_map<std::string, double> edgeTrafficFactors;
        std::chrono::steady_clock::time_point lastUpdate;
        bool isRealTimeEnabled;
        
        TrafficModel() : lastUpdate(std::chrono::steady_clock::now()), isRealTimeEnabled(false) {}
    };
    
    // Configuration parameters
    VelocityProfile defaultProfile;
    double timeHorizon;
    bool enableDynamicObstacles;
    bool enableTrafficModel;
    bool enableWeatherEffects;
    double congestionThreshold;
    int maxPlanningIterations;
    
    // Dynamic data
    std::vector<DynamicObstacle> dynamicObstacles;
    TrafficModel trafficModel;
    std::unordered_map<std::string, double> weatherFactors;
    
    double calculateSegmentDistance(int fromNode, int toNode) const {
        const Node& from = graph->getNode(fromNode);
        const Node& to = graph->getNode(toNode);
        return from.euclideanDistance(to);
    }
    
    SegmentTiming calculateOptimalSegmentTiming(int fromNode, int toNode, 
                                              const VelocityProfile& profile,
                                              double initialVelocity, double finalVelocity) const {
        SegmentTiming timing;
        timing.fromNode = fromNode;
        timing.toNode = toNode;
        timing.distance = calculateSegmentDistance(fromNode, toNode);
        
        // Apply dynamic factors
        timing.congestionFactor = getCongestionFactor(fromNode, toNode);
        timing.weatherFactor = getWeatherFactor();
        
        double effectiveMaxVel = profile.maxVelocity * timing.congestionFactor * timing.weatherFactor;
        double effectiveMaxAccel = profile.maxAcceleration * timing.weatherFactor;
        double effectiveMaxDecel = profile.maxDeceleration * timing.weatherFactor;
        
        // Calculate optimal time using trapezoidal velocity profile
        timing = calculateTrapezoidalProfile(timing, initialVelocity, finalVelocity,
                                           effectiveMaxVel, effectiveMaxAccel, effectiveMaxDecel);
        
        return timing;
    }
    
    SegmentTiming calculateTrapezoidalProfile(SegmentTiming timing, double v0, double vf,
                                            double vmax, double amax, double dmax) const {
        double distance = timing.distance;
        
        // Check if we can reach max velocity
        double accelDistance = (vmax * vmax - v0 * v0) / (2.0 * amax);
        double decelDistance = (vmax * vmax - vf * vf) / (2.0 * dmax);
        
        if (accelDistance + decelDistance <= distance) {
            // Trapezoidal profile with cruise phase
            timing.accelerationTime = (vmax - v0) / amax;
            timing.decelerationTime = (vmax - vf) / dmax;
            timing.cruiseTime = (distance - accelDistance - decelDistance) / vmax;
            timing.optimalTime = timing.accelerationTime + timing.cruiseTime + timing.decelerationTime;
            timing.averageVelocity = distance / timing.optimalTime;
        } else {
            // Triangular profile without cruise phase
            double vPeak = std::sqrt((2.0 * amax * dmax * distance + dmax * v0 * v0 + amax * vf * vf) / (amax + dmax));
            
            timing.accelerationTime = (vPeak - v0) / amax;
            timing.decelerationTime = (vPeak - vf) / dmax;
            timing.cruiseTime = 0.0;
            timing.optimalTime = timing.accelerationTime + timing.decelerationTime;
            timing.averageVelocity = distance / timing.optimalTime;
        }
        
        return timing;
    }
    
    double getCongestionFactor(int fromNode, int toNode) const {
        if (!enableTrafficModel) return 1.0;
        
        // Check node congestion levels
        double avgCongestion = 0.0;
        int count = 0;
        
        auto fromCongestion = trafficModel.nodeCongestionLevels.find(fromNode);
        if (fromCongestion != trafficModel.nodeCongestionLevels.end()) {
            avgCongestion += fromCongestion->second;
            count++;
        }
        
        auto toCongestion = trafficModel.nodeCongestionLevels.find(toNode);
        if (toCongestion != trafficModel.nodeCongestionLevels.end()) {
            avgCongestion += toCongestion->second;
            count++;
        }
        
        if (count > 0) {
            avgCongestion /= count;
        }
        
        // Check edge-specific traffic
        std::string edgeKey = std::to_string(fromNode) + "_" + std::to_string(toNode);
        auto edgeTraffic = trafficModel.edgeTrafficFactors.find(edgeKey);
        if (edgeTraffic != trafficModel.edgeTrafficFactors.end()) {
            avgCongestion = std::max(avgCongestion, edgeTraffic->second);
        }
        
        // Convert congestion level to velocity factor (higher congestion = lower velocity)
        return std::max(0.1, 1.0 - avgCongestion);
    }
    
    double getWeatherFactor() const {
        if (!enableWeatherEffects) return 1.0;
        
        double factor = 1.0;
        for (const auto& [weatherType, intensity] : weatherFactors) {
            if (weatherType == "rain") {
                factor *= (1.0 - intensity * 0.3);
            } else if (weatherType == "snow") {
                factor *= (1.0 - intensity * 0.5);
            } else if (weatherType == "fog") {
                factor *= (1.0 - intensity * 0.4);
            } else if (weatherType == "wind") {
                factor *= (1.0 - intensity * 0.2);
            }
        }
        
        return std::max(0.1, factor);
    }
    
    TimeOptimizationResult optimizeWithVelocityPlanning(const std::vector<int>& originalPath) {
        std::cout << "[TIME_OPTIMAL] Applying velocity-based time optimization" << std::endl;
        
        TimeOptimizationResult result;
        result.method = "Velocity Planning";
        result.optimizedPath = originalPath;
        
        if (originalPath.size() < 2) {
            return result;
        }
        
        // Calculate timing for each segment
        double currentVelocity = defaultProfile.currentVelocity;
        
        for (size_t i = 1; i < originalPath.size(); ++i) {
            double nextVelocity = (i == originalPath.size() - 1) ? 0.0 : defaultProfile.cruiseVelocity;
            
            SegmentTiming timing = calculateOptimalSegmentTiming(
                originalPath[i-1], originalPath[i], defaultProfile, currentVelocity, nextVelocity);
            
            result.timingProfile.push_back(timing);
            result.totalTime += timing.optimalTime;
            
            currentVelocity = nextVelocity;
        }
        
        // Calculate original time with constant velocity
        result.originalTime = calculateConstantVelocityTime(originalPath);
        result.timeSavings = result.originalTime - result.totalTime;
        result.timeSavingsPercentage = (result.timeSavings / result.originalTime) * 100.0;
        result.optimizationSteps = 1;
        
        std::cout << "[TIME_OPTIMAL] Velocity planning completed, time savings: " 
                  << result.timeSavingsPercentage << "%" << std::endl;
        
        return result;
    }
    
    TimeOptimizationResult optimizeWithCongestionAvoidance(const std::vector<int>& originalPath) {
        std::cout << "[TIME_OPTIMAL] Applying congestion avoidance optimization" << std::endl;
        
        TimeOptimizationResult result;
        result.method = "Congestion Avoidance";
        result.optimizedPath = originalPath;
        
        if (!enableTrafficModel) {
            std::cout << "[TIME_OPTIMAL] Traffic model disabled, returning original path" << std::endl;
            return optimizeWithVelocityPlanning(originalPath);
        }
        
        bool improved = true;
        int iteration = 0;
        
        while (improved && iteration < maxPlanningIterations) {
            improved = false;
            iteration++;
            
            // Find most congested segments
            std::vector<std::pair<size_t, double>> congestedSegments;
            for (size_t i = 1; i < result.optimizedPath.size(); ++i) {
                double congestionFactor = getCongestionFactor(result.optimizedPath[i-1], result.optimizedPath[i]);
                if (congestionFactor < (1.0 - congestionThreshold)) {
                    congestedSegments.emplace_back(i-1, 1.0 - congestionFactor);
                }
            }
            
            if (!congestedSegments.empty()) {
                // Sort by congestion level (highest first)
                std::sort(congestedSegments.begin(), congestedSegments.end(),
                         [](const auto& a, const auto& b) { return a.second > b.second; });
                
                // Try to find alternative for most congested segment
                auto [segmentIndex, congestionLevel] = congestedSegments[0];
                std::vector<int> alternative = findCongestionAlternative(
                    result.optimizedPath, segmentIndex);
                
                if (!alternative.empty()) {
                    double alternativeTime = calculatePathTime(alternative);
                    double currentTime = calculatePathTime(result.optimizedPath);
                    
                    if (alternativeTime < currentTime) {
                        result.optimizedPath = alternative;
                        result.congestionAvoidanceNodes.push_back(result.optimizedPath[segmentIndex]);
                        improved = true;
                        result.optimizationSteps++;
                        
                        std::cout << "[TIME_OPTIMAL] Found congestion alternative, time improvement: " 
                                  << (currentTime - alternativeTime) << "s" << std::endl;
                    }
                }
            }
        }
        
        // Recalculate final timing
        TimeOptimizationResult finalResult = optimizeWithVelocityPlanning(result.optimizedPath);
        finalResult.method = result.method;
        finalResult.optimizationSteps = result.optimizationSteps;
        finalResult.congestionAvoidanceNodes = result.congestionAvoidanceNodes;
        
        std::cout << "[TIME_OPTIMAL] Congestion avoidance completed in " << iteration 
                  << " iterations" << std::endl;
        
        return finalResult;
    }
    
    TimeOptimizationResult optimizeWithDynamicObstacles(const std::vector<int>& originalPath) {
        std::cout << "[TIME_OPTIMAL] Applying dynamic obstacle avoidance optimization" << std::endl;
        
        TimeOptimizationResult result;
        result.method = "Dynamic Obstacle Avoidance";
        result.optimizedPath = originalPath;
        
        if (!enableDynamicObstacles || dynamicObstacles.empty()) {
            return optimizeWithVelocityPlanning(originalPath);
        }
        
        // Update obstacle status
        updateDynamicObstacles();
        
        // Check for obstacles affecting the path
        std::vector<int> affectedNodes = findObstacleAffectedNodes(originalPath);
        
        if (!affectedNodes.empty()) {
            std::cout << "[TIME_OPTIMAL] Found " << affectedNodes.size() 
                      << " nodes affected by dynamic obstacles" << std::endl;
            
            // Try to find alternative path avoiding obstacles
            std::vector<int> alternativePath = findObstacleAvoidancePath(originalPath, affectedNodes);
            
            if (!alternativePath.empty()) {
                double originalTime = calculatePathTime(originalPath);
                double alternativeTime = calculatePathTime(alternativePath);
                
                if (alternativeTime < originalTime * 1.2) { // Accept up to 20% time penalty for safety
                    result.optimizedPath = alternativePath;
                    std::cout << "[TIME_OPTIMAL] Using obstacle avoidance path, time change: " 
                              << (alternativeTime - originalTime) << "s" << std::endl;
                }
            }
        }
        
        // Calculate final timing with obstacle considerations
        TimeOptimizationResult finalResult = optimizeWithVelocityPlanning(result.optimizedPath);
        finalResult.method = result.method;
        
        return finalResult;
    }
    
    std::vector<int> findCongestionAlternative(const std::vector<int>& path, size_t segmentIndex) const {
        if (segmentIndex >= path.size() - 1) return {};
        
        int startNode = path[segmentIndex];
        int endNode = path[segmentIndex + 1];
        
        // Simple BFS to find alternative route
        std::queue<std::vector<int>> pathQueue;
        std::unordered_set<int> visited;
        
        pathQueue.push({startNode});
        visited.insert(startNode);
        
        while (!pathQueue.empty()) {
            std::vector<int> currentPath = pathQueue.front();
            pathQueue.pop();
            
            int currentNode = currentPath.back();
            
            if (currentNode == endNode) {
                // Found alternative, construct full path
                std::vector<int> fullPath;
                fullPath.insert(fullPath.end(), path.begin(), path.begin() + segmentIndex);
                fullPath.insert(fullPath.end(), currentPath.begin(), currentPath.end());
                fullPath.insert(fullPath.end(), path.begin() + segmentIndex + 2, path.end());
                return fullPath;
            }
            
            if (currentPath.size() > 5) continue; // Limit search depth
            
            std::vector<int> neighbors = graph->getNeighbors(currentNode);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end() && 
                    getCongestionFactor(currentNode, neighbor) > (1.0 - congestionThreshold)) {
                    
                    visited.insert(neighbor);
                    std::vector<int> newPath = currentPath;
                    newPath.push_back(neighbor);
                    pathQueue.push(newPath);
                }
            }
        }
        
        return {}; // No alternative found
    }
    
    void updateDynamicObstacles() {
        auto currentTime = std::chrono::steady_clock::now();
        
        for (auto& obstacle : dynamicObstacles) {
            if (obstacle.isActive) {
                auto timeElapsed = currentTime - obstacle.startTime;
                if (timeElapsed > obstacle.duration) {
                    obstacle.isActive = false;
                    std::cout << "[TIME_OPTIMAL] Dynamic obstacle at node " << obstacle.nodeId 
                              << " has expired" << std::endl;
                }
            }
        }
    }
    
    std::vector<int> findObstacleAffectedNodes(const std::vector<int>& path) const {
        std::vector<int> affectedNodes;
        
        for (int nodeId : path) {
            for (const auto& obstacle : dynamicObstacles) {
                if (obstacle.isActive) {
                    double distance = calculateSegmentDistance(nodeId, obstacle.nodeId);
                    if (distance <= obstacle.impactRadius) {
                        affectedNodes.push_back(nodeId);
                        break;
                    }
                }
            }
        }
        
        return affectedNodes;
    }
    
    std::vector<int> findObstacleAvoidancePath(const std::vector<int>& originalPath,
                                             const std::vector<int>& affectedNodes) const {
        // Simple implementation: try to find path avoiding affected nodes
        // In practice, this would use more sophisticated planning algorithms
        return {}; // Placeholder - would implement A* with obstacle avoidance
    }
    
    double calculatePathTime(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalTime = 0.0;
        double currentVelocity = defaultProfile.currentVelocity;
        
        for (size_t i = 1; i < path.size(); ++i) {
            double nextVelocity = (i == path.size() - 1) ? 0.0 : defaultProfile.cruiseVelocity;
            
            SegmentTiming timing = calculateOptimalSegmentTiming(
                path[i-1], path[i], defaultProfile, currentVelocity, nextVelocity);
            
            totalTime += timing.optimalTime;
            currentVelocity = nextVelocity;
        }
        
        return totalTime;
    }
    
    double calculateConstantVelocityTime(const std::vector<int>& path) const {
        if (path.size() < 2) return 0.0;
        
        double totalDistance = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            totalDistance += calculateSegmentDistance(path[i-1], path[i]);
        }
        
        return totalDistance / defaultProfile.cruiseVelocity;
    }
    
public:
    TimeOptimalPlanner(const Graph* environment) 
        : graph(environment), timeHorizon(300.0), enableDynamicObstacles(false),
          enableTrafficModel(false), enableWeatherEffects(false), congestionThreshold(0.3),
          maxPlanningIterations(50) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[TIME_OPTIMAL] Time-optimal planner initialized" << std::endl;
    }
    
    std::vector<int> optimizePathTime(const std::vector<int>& originalPath, 
                                     const std::string& method = "velocity_planning") {
        
        if (originalPath.size() < 2) {
            std::cout << "[TIME_OPTIMAL] Path too short for time optimization" << std::endl;
            return originalPath;
        }
        
        std::cout << "[TIME_OPTIMAL] Optimizing path time using " << method 
                  << " method for " << originalPath.size() << " nodes" << std::endl;
        
        TimeOptimizationResult result;
        
        if (method == "velocity_planning") {
            result = optimizeWithVelocityPlanning(originalPath);
        } else if (method == "congestion_avoidance") {
            result = optimizeWithCongestionAvoidance(originalPath);
        } else if (method == "dynamic_obstacles") {
            result = optimizeWithDynamicObstacles(originalPath);
        } else if (method == "comprehensive") {
            // Apply multiple methods in sequence
            result = optimizeWithDynamicObstacles(originalPath);
            result = optimizeWithCongestionAvoidance(result.optimizedPath);
            TimeOptimizationResult finalResult = optimizeWithVelocityPlanning(result.optimizedPath);
            finalResult.method = "Comprehensive (obstacles + congestion + velocity)";
            finalResult.optimizationSteps = result.optimizationSteps + finalResult.optimizationSteps;
            result = finalResult;
        } else {
            std::cout << "[TIME_OPTIMAL] Unknown method, using velocity planning" << std::endl;
            result = optimizeWithVelocityPlanning(originalPath);
        }
        
        std::cout << "[TIME_OPTIMAL] Time optimization complete - Travel time reduced by " 
                  << result.timeSavingsPercentage << "% (" << result.timeSavings 
                  << " seconds)" << std::endl;
        
        return result.optimizedPath;
    }
    
    void setVelocityProfile(double maxVel, double maxAccel, double maxDecel, double cruiseVel) {
        defaultProfile.maxVelocity = maxVel;
        defaultProfile.maxAcceleration = maxAccel;
        defaultProfile.maxDeceleration = maxDecel;
        defaultProfile.cruiseVelocity = cruiseVel;
        
        std::cout << "[TIME_OPTIMAL] Velocity profile updated - Max: " << maxVel 
                  << ", Cruise: " << cruiseVel << ", Accel: " << maxAccel 
                  << ", Decel: " << maxDecel << std::endl;
    }
    
    void addDynamicObstacle(int nodeId, double impactRadius, double velocityReduction, 
                           std::chrono::duration<double> duration) {
        DynamicObstacle obstacle(nodeId, impactRadius, velocityReduction);
        obstacle.duration = duration;
        dynamicObstacles.push_back(obstacle);
        
        std::cout << "[TIME_OPTIMAL] Added dynamic obstacle at node " << nodeId 
                  << " for " << duration.count() << " seconds" << std::endl;
    }
    
    void updateTrafficCongestion(int nodeId, double congestionLevel) {
        trafficModel.nodeCongestionLevels[nodeId] = congestionLevel;
        trafficModel.lastUpdate = std::chrono::steady_clock::now();
        
        std::cout << "[TIME_OPTIMAL] Updated traffic congestion at node " << nodeId 
                  << " to level " << congestionLevel << std::endl;
    }
    
    void setWeatherCondition(const std::string& weatherType, double intensity) {
        weatherFactors[weatherType] = intensity;
        
        std::cout << "[TIME_OPTIMAL] Set weather condition " << weatherType 
                  << " with intensity " << intensity << std::endl;
    }
    
    void enableTrafficModel(bool enable) {
        enableTrafficModel = enable;
        if (!enable) {
            trafficModel.nodeCongestionLevels.clear();
            trafficModel.edgeTrafficFactors.clear();
        }
        std::cout << "[TIME_OPTIMAL] Traffic model " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableDynamicObstacleAvoidance(bool enable) {
        enableDynamicObstacles = enable;
        if (!enable) {
            dynamicObstacles.clear();
        }
        std::cout << "[TIME_OPTIMAL] Dynamic obstacle avoidance " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableWeatherModel(bool enable) {
        enableWeatherEffects = enable;
        if (!enable) {
            weatherFactors.clear();
        }
        std::cout << "[TIME_OPTIMAL] Weather effects " << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    double estimatePathTime(const std::vector<int>& path) const {
        return calculatePathTime(path);
    }
    
    void setCongestionThreshold(double threshold) {
        congestionThreshold = threshold;
        std::cout << "[TIME_OPTIMAL] Congestion threshold set to " << threshold << std::endl;
    }
    
    void setMaxPlanningIterations(int iterations) {
        maxPlanningIterations = iterations;
        std::cout << "[TIME_OPTIMAL] Max planning iterations set to " << iterations << std::endl;
    }
    
    void printPlanningConfiguration() const {
        std::cout << "[TIME_OPTIMAL] Configuration:" << std::endl;
        std::cout << "[TIME_OPTIMAL]   Max velocity: " << defaultProfile.maxVelocity << std::endl;
        std::cout << "[TIME_OPTIMAL]   Cruise velocity: " << defaultProfile.cruiseVelocity << std::endl;
        std::cout << "[TIME_OPTIMAL]   Max acceleration: " << defaultProfile.maxAcceleration << std::endl;
        std::cout << "[TIME_OPTIMAL]   Max deceleration: " << defaultProfile.maxDeceleration << std::endl;
        std::cout << "[TIME_OPTIMAL]   Traffic model: " << (enableTrafficModel ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[TIME_OPTIMAL]   Dynamic obstacles: " << (enableDynamicObstacles ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[TIME_OPTIMAL]   Weather effects: " << (enableWeatherEffects ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[TIME_OPTIMAL]   Active obstacles: " << dynamicObstacles.size() << std::endl;
        std::cout << "[TIME_OPTIMAL]   Congestion nodes: " << trafficModel.nodeCongestionLevels.size() << std::endl;
    }
};