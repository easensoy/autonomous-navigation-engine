#include "environment/ObstacleDetection.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <queue>
#include <unordered_map>
#include <vector>
#include <memory>

class CollisionPredictionManager {
private:
    const Graph* environment;
    mutable std::mutex predictionMutex;
    
    struct TrajectoryPoint {
        double x, y;
        std::chrono::steady_clock::time_point timestamp;
        double velocity;
        double heading;
    };
    
    struct ObstacleState {
        int obstacleId;
        std::vector<TrajectoryPoint> trajectory;
        double predictedVelocity;
        double predictedHeading;
        std::chrono::steady_clock::time_point lastUpdate;
        bool isDynamic;
        double confidence;
    };
    
    struct CollisionRisk {
        int obstacleId;
        double timeToCollision;
        double riskLevel;
        std::pair<double, double> collisionPoint;
        std::string riskCategory;
        double confidenceLevel;
    };
    
    struct PredictionMetrics {
        size_t totalPredictions;
        size_t highRiskDetections;
        size_t correctPredictions;
        size_t falsePositives;
        double averagePredictionTime;
        double averageAccuracy;
        std::chrono::steady_clock::time_point lastPrediction;
    };
    
    std::unordered_map<int, ObstacleState> trackedObstacles;
    std::vector<TrajectoryPoint> currentPath;
    std::queue<CollisionRisk> riskQueue;
    PredictionMetrics metrics;
    
    double criticalRiskThreshold;
    double warningRiskThreshold;
    double predictionHorizon;
    double velocityEstimationWindow;
    bool adaptivePrediction;
    std::function<void(const CollisionRisk&)> collisionCallback;
    
public:
    explicit CollisionPredictionManager(const Graph* graph) 
        : environment(graph), criticalRiskThreshold(0.8), warningRiskThreshold(0.6),
          predictionHorizon(5.0), velocityEstimationWindow(2.0), adaptivePrediction(true) {
        
        if (!environment) {
            throw std::invalid_argument("Graph environment cannot be null");
        }
        
        initializePredictionMetrics();
        std::cout << "[COLLISION_PREDICTION] Collision prediction system initialized" << std::endl;
    }
    
    void updateCurrentPath(const std::vector<int>& pathNodes) {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        std::cout << "[COLLISION_PREDICTION] Updating current navigation path with " 
                  << pathNodes.size() << " waypoints" << std::endl;
        
        currentPath.clear();
        auto currentTime = std::chrono::steady_clock::now();
        
        for (size_t i = 0; i < pathNodes.size(); ++i) {
            if (environment->hasNode(pathNodes[i])) {
                const Node& node = environment->getNode(pathNodes[i]);
                
                TrajectoryPoint point;
                point.x = node.getX();
                point.y = node.getY();
                point.timestamp = currentTime + std::chrono::milliseconds(i * 1000); // Estimated timing
                point.velocity = 1.0; // Default velocity
                point.heading = calculateHeading(i, pathNodes);
                
                currentPath.push_back(point);
            }
        }
        
        std::cout << "[COLLISION_PREDICTION] Path trajectory updated with " 
                  << currentPath.size() << " trajectory points" << std::endl;
    }
    
    void updateObstacleState(int obstacleId, double x, double y, bool isDynamic) {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        auto currentTime = std::chrono::steady_clock::now();
        
        TrajectoryPoint newPoint;
        newPoint.x = x;
        newPoint.y = y;
        newPoint.timestamp = currentTime;
        newPoint.velocity = 0.0;
        newPoint.heading = 0.0;
        
        if (trackedObstacles.find(obstacleId) == trackedObstacles.end()) {
            std::cout << "[COLLISION_PREDICTION] Tracking new obstacle: " << obstacleId << std::endl;
            
            ObstacleState newObstacle;
            newObstacle.obstacleId = obstacleId;
            newObstacle.isDynamic = isDynamic;
            newObstacle.lastUpdate = currentTime;
            newObstacle.confidence = 0.9;
            newObstacle.trajectory.push_back(newPoint);
            
            trackedObstacles[obstacleId] = newObstacle;
        } else {
            ObstacleState& obstacle = trackedObstacles[obstacleId];
            obstacle.trajectory.push_back(newPoint);
            obstacle.lastUpdate = currentTime;
            
            // Calculate velocity and heading if we have previous points
            if (obstacle.trajectory.size() >= 2) {
                updateObstacleKinematics(obstacle);
            }
            
            // Limit trajectory history
            if (obstacle.trajectory.size() > 50) {
                obstacle.trajectory.erase(obstacle.trajectory.begin());
            }
        }
        
        std::cout << "[COLLISION_PREDICTION] Updated obstacle " << obstacleId 
                  << " at position (" << x << "," << y << ")" << std::endl;
    }
    
    std::vector<CollisionRisk> predictCollisions() {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        auto predictionStart = std::chrono::steady_clock::now();
        std::cout << "[COLLISION_PREDICTION] Executing collision prediction analysis" << std::endl;
        
        std::vector<CollisionRisk> detectedRisks;
        
        if (currentPath.empty()) {
            std::cout << "[COLLISION_PREDICTION] No current path available for prediction" << std::endl;
            return detectedRisks;
        }
        
        for (const auto& [obstacleId, obstacle] : trackedObstacles) {
            if (isObstacleStale(obstacle)) {
                continue;
            }
            
            CollisionRisk risk = analyzeCollisionRisk(obstacle);
            
            if (risk.riskLevel > warningRiskThreshold) {
                detectedRisks.push_back(risk);
                
                std::cout << "[COLLISION_PREDICTION] Collision risk detected - Obstacle: " 
                          << obstacleId << ", Risk: " << risk.riskLevel 
                          << ", Time to collision: " << risk.timeToCollision << "s" << std::endl;
                
                if (risk.riskLevel > criticalRiskThreshold && collisionCallback) {
                    collisionCallback(risk);
                }
            }
        }
        
        updatePredictionMetrics(predictionStart, detectedRisks.size());
        
        std::cout << "[COLLISION_PREDICTION] Prediction analysis completed, " 
                  << detectedRisks.size() << " risks identified" << std::endl;
        
        return detectedRisks;
    }
    
    CollisionRisk analyzePathSegmentRisk(int startNode, int endNode) {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        std::cout << "[COLLISION_PREDICTION] Analyzing collision risk for path segment " 
                  << startNode << " -> " << endNode << std::endl;
        
        CollisionRisk segmentRisk;
        segmentRisk.obstacleId = -1;
        segmentRisk.timeToCollision = std::numeric_limits<double>::infinity();
        segmentRisk.riskLevel = 0.0;
        segmentRisk.riskCategory = "path_segment";
        segmentRisk.confidenceLevel = 0.8;
        
        if (!environment->hasNode(startNode) || !environment->hasNode(endNode)) {
            std::cout << "[COLLISION_PREDICTION] Invalid path segment nodes" << std::endl;
            return segmentRisk;
        }
        
        const Node& start = environment->getNode(startNode);
        const Node& end = environment->getNode(endNode);
        
        segmentRisk.collisionPoint = {(start.getX() + end.getX()) / 2.0, 
                                    (start.getY() + end.getY()) / 2.0};
        
        // Check for obstacles along the path segment
        double maxRisk = 0.0;
        double minTimeToCollision = std::numeric_limits<double>::infinity();
        
        for (const auto& [obstacleId, obstacle] : trackedObstacles) {
            if (isObstacleStale(obstacle) || obstacle.trajectory.empty()) {
                continue;
            }
            
            double distanceToPath = calculateDistanceToPathSegment(
                obstacle.trajectory.back(), start.getX(), start.getY(), 
                end.getX(), end.getY());
            
            if (distanceToPath < 2.0) { // Within 2 units of path
                double risk = calculateProximityRisk(distanceToPath, obstacle);
                double timeToCollision = estimateTimeToCollision(obstacle, start, end);
                
                if (risk > maxRisk) {
                    maxRisk = risk;
                    segmentRisk.obstacleId = obstacleId;
                }
                
                if (timeToCollision < minTimeToCollision) {
                    minTimeToCollision = timeToCollision;
                }
            }
        }
        
        segmentRisk.riskLevel = maxRisk;
        segmentRisk.timeToCollision = minTimeToCollision;
        
        std::cout << "[COLLISION_PREDICTION] Path segment risk analysis completed - Risk: " 
                  << segmentRisk.riskLevel << std::endl;
        
        return segmentRisk;
    }
    
    void setCollisionCallback(std::function<void(const CollisionRisk&)> callback) {
        collisionCallback = callback;
        std::cout << "[COLLISION_PREDICTION] Collision callback configured" << std::endl;
    }
    
    void configurePredictionParameters(double criticalThreshold, double warningThreshold, 
                                     double horizon, bool adaptive) {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        criticalRiskThreshold = criticalThreshold;
        warningRiskThreshold = warningThreshold;
        predictionHorizon = horizon;
        adaptivePrediction = adaptive;
        
        std::cout << "[COLLISION_PREDICTION] Prediction parameters configured:" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Critical threshold: " << criticalRiskThreshold << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Warning threshold: " << warningRiskThreshold << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Prediction horizon: " << predictionHorizon << "s" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Adaptive prediction: " << (adaptivePrediction ? "Enabled" : "Disabled") << std::endl;
    }
    
    void clearStaleObstacles() {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        auto currentTime = std::chrono::steady_clock::now();
        size_t removedCount = 0;
        
        auto it = trackedObstacles.begin();
        while (it != trackedObstacles.end()) {
            auto timeSinceUpdate = std::chrono::duration<double>(currentTime - it->second.lastUpdate);
            
            if (timeSinceUpdate.count() > 10.0) { // 10 seconds staleness threshold
                std::cout << "[COLLISION_PREDICTION] Removing stale obstacle: " << it->first << std::endl;
                it = trackedObstacles.erase(it);
                removedCount++;
            } else {
                ++it;
            }
        }
        
        if (removedCount > 0) {
            std::cout << "[COLLISION_PREDICTION] Cleared " << removedCount << " stale obstacles" << std::endl;
        }
    }
    
    PredictionMetrics getPredictionMetrics() const {
        std::lock_guard<std::mutex> lock(predictionMutex);
        return metrics;
    }
    
    void generatePredictionReport() const {
        std::lock_guard<std::mutex> lock(predictionMutex);
        
        std::cout << "\n[COLLISION_PREDICTION] === COLLISION PREDICTION REPORT ===" << std::endl;
        
        std::cout << "[COLLISION_PREDICTION] System Configuration:" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Critical risk threshold: " << criticalRiskThreshold << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Warning risk threshold: " << warningRiskThreshold << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Prediction horizon: " << predictionHorizon << " seconds" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Adaptive prediction: " << (adaptivePrediction ? "Enabled" : "Disabled") << std::endl;
        
        std::cout << "[COLLISION_PREDICTION] Current State:" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Tracked obstacles: " << trackedObstacles.size() << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Current path points: " << currentPath.size() << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Pending risk assessments: " << riskQueue.size() << std::endl;
        
        std::cout << "[COLLISION_PREDICTION] Performance Metrics:" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Total predictions: " << metrics.totalPredictions << std::endl;
        std::cout << "[COLLISION_PREDICTION]   High-risk detections: " << metrics.highRiskDetections << std::endl;
        std::cout << "[COLLISION_PREDICTION]   Prediction accuracy: ";
        
        if (metrics.totalPredictions > 0) {
            double accuracy = static_cast<double>(metrics.correctPredictions) / metrics.totalPredictions * 100.0;
            std::cout << accuracy << "%" << std::endl;
        } else {
            std::cout << "No data available" << std::endl;
        }
        
        std::cout << "[COLLISION_PREDICTION]   Average prediction time: " << metrics.averagePredictionTime << "s" << std::endl;
        std::cout << "[COLLISION_PREDICTION]   False positive rate: ";
        
        if (metrics.totalPredictions > 0) {
            double falsePositiveRate = static_cast<double>(metrics.falsePositives) / metrics.totalPredictions * 100.0;
            std::cout << falsePositiveRate << "%" << std::endl;
        } else {
            std::cout << "No data available" << std::endl;
        }
        
        std::cout << "[COLLISION_PREDICTION] === END REPORT ===" << std::endl;
    }
    
private:
    void initializePredictionMetrics() {
        metrics.totalPredictions = 0;
        metrics.highRiskDetections = 0;
        metrics.correctPredictions = 0;
        metrics.falsePositives = 0;
        metrics.averagePredictionTime = 0.0;
        metrics.averageAccuracy = 0.0;
        metrics.lastPrediction = std::chrono::steady_clock::now();
    }
    
    double calculateHeading(size_t currentIndex, const std::vector<int>& pathNodes) {
        if (currentIndex + 1 >= pathNodes.size()) {
            return 0.0;
        }
        
        if (!environment->hasNode(pathNodes[currentIndex]) || 
            !environment->hasNode(pathNodes[currentIndex + 1])) {
            return 0.0;
        }
        
        const Node& current = environment->getNode(pathNodes[currentIndex]);
        const Node& next = environment->getNode(pathNodes[currentIndex + 1]);
        
        double dx = next.getX() - current.getX();
        double dy = next.getY() - current.getY();
        
        return std::atan2(dy, dx);
    }
    
    void updateObstacleKinematics(ObstacleState& obstacle) {
        if (obstacle.trajectory.size() < 2) {
            return;
        }
        
        const TrajectoryPoint& latest = obstacle.trajectory.back();
        const TrajectoryPoint& previous = obstacle.trajectory[obstacle.trajectory.size() - 2];
        
        double dx = latest.x - previous.x;
        double dy = latest.y - previous.y;
        auto timeDiff = std::chrono::duration<double>(latest.timestamp - previous.timestamp);
        
        if (timeDiff.count() > 0) {
            double distance = std::sqrt(dx * dx + dy * dy);
            obstacle.predictedVelocity = distance / timeDiff.count();
            obstacle.predictedHeading = std::atan2(dy, dx);
            
            std::cout << "[COLLISION_PREDICTION] Updated obstacle " << obstacle.obstacleId 
                      << " kinematics - Velocity: " << obstacle.predictedVelocity 
                      << ", Heading: " << obstacle.predictedHeading << std::endl;
        }
    }
    
    bool isObstacleStale(const ObstacleState& obstacle) const {
        auto currentTime = std::chrono::steady_clock::now();
        auto timeSinceUpdate = std::chrono::duration<double>(currentTime - obstacle.lastUpdate);
        return timeSinceUpdate.count() > 5.0; // 5 seconds staleness threshold
    }
    
    CollisionRisk analyzeCollisionRisk(const ObstacleState& obstacle) {
        CollisionRisk risk;
        risk.obstacleId = obstacle.obstacleId;
        risk.timeToCollision = std::numeric_limits<double>::infinity();
        risk.riskLevel = 0.0;
        risk.confidenceLevel = obstacle.confidence;
        risk.riskCategory = obstacle.isDynamic ? "dynamic_obstacle" : "static_obstacle";
        
        if (obstacle.trajectory.empty()) {
            return risk;
        }
        
        const TrajectoryPoint& obstaclePos = obstacle.trajectory.back();
        risk.collisionPoint = {obstaclePos.x, obstaclePos.y};
        
        // Find closest approach to current path
        double minDistance = std::numeric_limits<double>::max();
        double timeToClosestApproach = 0.0;
        
        for (size_t i = 0; i < currentPath.size(); ++i) {
            const TrajectoryPoint& pathPoint = currentPath[i];
            
            // Predict obstacle position at this time
            auto timeDiff = std::chrono::duration<double>(pathPoint.timestamp - obstaclePos.timestamp);
            double predictedX = obstaclePos.x + obstacle.predictedVelocity * std::cos(obstacle.predictedHeading) * timeDiff.count();
            double predictedY = obstaclePos.y + obstacle.predictedVelocity * std::sin(obstacle.predictedHeading) * timeDiff.count();
            
            double distance = std::sqrt(std::pow(predictedX - pathPoint.x, 2) + std::pow(predictedY - pathPoint.y, 2));
            
            if (distance < minDistance) {
                minDistance = distance;
                timeToClosestApproach = timeDiff.count();
                risk.collisionPoint = {predictedX, predictedY};
            }
        }
        
        // Calculate risk based on minimum distance and time
        if (minDistance < 3.0) { // Critical distance threshold
            risk.riskLevel = std::max(0.0, 1.0 - (minDistance / 3.0));
            risk.timeToCollision = std::max(0.0, timeToClosestApproach);
            
            if (minDistance < 1.0) {
                risk.riskLevel = std::min(1.0, risk.riskLevel * 1.5); // Boost risk for very close approaches
            }
        }
        
        return risk;
    }
    
    double calculateDistanceToPathSegment(const TrajectoryPoint& point, double x1, double y1, double x2, double y2) {
        double A = point.x - x1;
        double B = point.y - y1;
        double C = x2 - x1;
        double D = y2 - y1;
        
        double dot = A * C + B * D;
        double lenSq = C * C + D * D;
        
        if (lenSq == 0) {
            return std::sqrt(A * A + B * B);
        }
        
        double param = dot / lenSq;
        double xx, yy;
        
        if (param < 0) {
            xx = x1;
            yy = y1;
        } else if (param > 1) {
            xx = x2;
            yy = y2;
        } else {
            xx = x1 + param * C;
            yy = y1 + param * D;
        }
        
        double dx = point.x - xx;
        double dy = point.y - yy;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    double calculateProximityRisk(double distance, const ObstacleState& obstacle) {
        double baseRisk = std::max(0.0, 1.0 - (distance / 5.0)); // Risk decreases with distance
        
        if (obstacle.isDynamic) {
            baseRisk *= 1.3; // Higher risk for dynamic obstacles
        }
        
        baseRisk *= obstacle.confidence; // Scale by confidence in obstacle detection
        
        return std::min(1.0, baseRisk);
    }
    
    double estimateTimeToCollision(const ObstacleState& obstacle, const Node& start, const Node& end) {
        if (obstacle.trajectory.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        
        const TrajectoryPoint& obstaclePos = obstacle.trajectory.back();
        
        // Simplified time to collision calculation
        double pathLength = std::sqrt(std::pow(end.getX() - start.getX(), 2) + std::pow(end.getY() - start.getY(), 2));
        double pathVelocity = 1.0; // Assumed navigation velocity
        
        double distanceToPath = calculateDistanceToPathSegment(obstaclePos, start.getX(), start.getY(), end.getX(), end.getY());
        
        if (distanceToPath > 3.0 || obstacle.predictedVelocity <= 0) {
            return std::numeric_limits<double>::infinity();
        }
        
        // Estimate when obstacle will intersect path
        double timeToIntersection = distanceToPath / std::max(0.1, obstacle.predictedVelocity);
        
        return timeToIntersection;
    }
    
    void updatePredictionMetrics(std::chrono::steady_clock::time_point startTime, size_t risksDetected) {
        auto endTime = std::chrono::steady_clock::now();
        double predictionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        metrics.totalPredictions++;
        metrics.highRiskDetections += risksDetected;
        metrics.lastPrediction = endTime;
        
        // Update average prediction time
        if (metrics.averagePredictionTime == 0.0) {
            metrics.averagePredictionTime = predictionTime;
        } else {
            metrics.averagePredictionTime = (metrics.averagePredictionTime + predictionTime) / 2.0;
        }
    }
};