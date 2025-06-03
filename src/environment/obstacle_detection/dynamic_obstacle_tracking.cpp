#include "environment/ObstacleDetection.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <queue>
#include <unordered_map>
#include <vector>
#include <memory>
#include <numeric>

class DynamicObstacleTracker {
private:
    const Graph* environment;
    mutable std::mutex trackingMutex;
    std::atomic<bool> trackingActive;
    std::thread trackingThread;
    
    struct ObstacleDetection {
        int detectionId;
        double x, y;
        std::chrono::steady_clock::time_point timestamp;
        double confidence;
        std::string sensorSource;
    };
    
    struct KalmanState {
        double x, y;           // Position
        double vx, vy;         // Velocity
        double ax, ay;         // Acceleration
        double px, py;         // Position uncertainty
        double pvx, pvy;       // Velocity uncertainty
        double pax, pay;       // Acceleration uncertainty
    };
    
    struct DynamicObstacle {
        int obstacleId;
        KalmanState state;
        std::vector<ObstacleDetection> detectionHistory;
        std::chrono::steady_clock::time_point firstDetection;
        std::chrono::steady_clock::time_point lastUpdate;
        double trackingConfidence;
        bool isConfirmed;
        int consecutiveDetections;
        int missedDetections;
        std::string classification;
        double estimatedSize;
        double maxVelocity;
    };
    
    struct TrackingMetrics {
        size_t totalObstaclesTracked;
        size_t activeTracking;
        size_t confirmedObstacles;
        size_t lostTracks;
        double averageTrackingDuration;
        double averageTrackingAccuracy;
        std::chrono::steady_clock::time_point lastTrackingUpdate;
        size_t detectionAssociations;
        size_t falseTracks;
    };
    
    std::unordered_map<int, DynamicObstacle> trackedObstacles;
    std::queue<ObstacleDetection> detectionQueue;
    mutable std::mutex detectionQueueMutex;
    TrackingMetrics metrics;
    
    int nextObstacleId;
    double associationThreshold;
    double confirmationThreshold;
    double trackLossThreshold;
    double processNoise;
    double measurementNoise;
    bool enablePrediction;
    std::function<void(const DynamicObstacle&)> obstacleCallback;
    std::function<void(int)> trackLossCallback;
    
public:
    explicit DynamicObstacleTracker(const Graph* graph) 
        : environment(graph), trackingActive(false), nextObstacleId(1000),
          associationThreshold(2.0), confirmationThreshold(0.8), trackLossThreshold(3.0),
          processNoise(0.1), measurementNoise(0.5), enablePrediction(true) {
        
        if (!environment) {
            throw std::invalid_argument("Graph environment cannot be null");
        }
        
        initializeTrackingMetrics();
        std::cout << "[DYNAMIC_TRACKING] Dynamic obstacle tracking system initialized" << std::endl;
    }
    
    ~DynamicObstacleTracker() {
        stopTracking();
    }
    
    bool startTracking() {
        std::cout << "[DYNAMIC_TRACKING] Initiating dynamic obstacle tracking services" << std::endl;
        
        if (trackingActive.load()) {
            std::cout << "[DYNAMIC_TRACKING] Tracking services already operational" << std::endl;
            return true;
        }
        
        trackingActive.store(true);
        trackingThread = std::thread(&DynamicObstacleTracker::trackingLoop, this);
        
        std::cout << "[DYNAMIC_TRACKING] Dynamic obstacle tracking services started successfully" << std::endl;
        return true;
    }
    
    void stopTracking() {
        if (!trackingActive.load()) {
            return;
        }
        
        std::cout << "[DYNAMIC_TRACKING] Initiating tracking shutdown sequence" << std::endl;
        
        trackingActive.store(false);
        
        if (trackingThread.joinable()) {
            trackingThread.join();
        }
        
        {
            std::lock_guard<std::mutex> lock(detectionQueueMutex);
            while (!detectionQueue.empty()) {
                detectionQueue.pop();
            }
        }
        
        std::cout << "[DYNAMIC_TRACKING] Dynamic obstacle tracking services terminated" << std::endl;
    }
    
    void processObstacleDetection(double x, double y, double confidence, const std::string& source) {
        auto currentTime = std::chrono::steady_clock::now();
        
        ObstacleDetection detection;
        detection.detectionId = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime.time_since_epoch()).count()) % 100000;
        detection.x = x;
        detection.y = y;
        detection.timestamp = currentTime;
        detection.confidence = confidence;
        detection.sensorSource = source;
        
        {
            std::lock_guard<std::mutex> lock(detectionQueueMutex);
            detectionQueue.push(detection);
        }
        
        std::cout << "[DYNAMIC_TRACKING] Obstacle detection queued at (" << x << "," << y 
                  << ") with confidence " << confidence << " from source: " << source << std::endl;
    }
    
    void processBatchDetections(const std::vector<std::tuple<double, double, double>>& detections) {
        std::cout << "[DYNAMIC_TRACKING] Processing batch of " << detections.size() 
                  << " obstacle detections" << std::endl;
        
        auto currentTime = std::chrono::steady_clock::now();
        
        for (const auto& [x, y, confidence] : detections) {
            ObstacleDetection detection;
            detection.detectionId = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTime.time_since_epoch()).count()) % 100000;
            detection.x = x;
            detection.y = y;
            detection.timestamp = currentTime;
            detection.confidence = confidence;
            detection.sensorSource = "batch_processing";
            
            {
                std::lock_guard<std::mutex> lock(detectionQueueMutex);
                detectionQueue.push(detection);
            }
        }
        
        std::cout << "[DYNAMIC_TRACKING] Batch detection processing completed" << std::endl;
    }
    
    std::vector<DynamicObstacle> getActiveObstacles() const {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        std::vector<DynamicObstacle> activeObstacles;
        auto currentTime = std::chrono::steady_clock::now();
        
        for (const auto& [obstacleId, obstacle] : trackedObstacles) {
            auto timeSinceUpdate = std::chrono::duration<double>(currentTime - obstacle.lastUpdate);
            
            if (timeSinceUpdate.count() < 5.0 && obstacle.isConfirmed) {
                activeObstacles.push_back(obstacle);
            }
        }
        
        return activeObstacles;
    }
    
    std::vector<std::pair<double, double>> predictObstaclePositions(double predictionTime) {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        std::cout << "[DYNAMIC_TRACKING] Predicting obstacle positions " << predictionTime 
                  << " seconds into the future" << std::endl;
        
        std::vector<std::pair<double, double>> predictedPositions;
        
        for (const auto& [obstacleId, obstacle] : trackedObstacles) {
            if (!obstacle.isConfirmed) {
                continue;
            }
            
            // Predict position using kinematic model
            double predictedX = obstacle.state.x + obstacle.state.vx * predictionTime + 
                              0.5 * obstacle.state.ax * predictionTime * predictionTime;
            double predictedY = obstacle.state.y + obstacle.state.vy * predictionTime + 
                              0.5 * obstacle.state.ay * predictionTime * predictionTime;
            
            predictedPositions.emplace_back(predictedX, predictedY);
        }
        
        std::cout << "[DYNAMIC_TRACKING] Generated " << predictedPositions.size() 
                  << " obstacle position predictions" << std::endl;
        
        return predictedPositions;
    }
    
    void setTrackingCallbacks(std::function<void(const DynamicObstacle&)> obstacleDetected,
                            std::function<void(int)> trackLost) {
        obstacleCallback = obstacleDetected;
        trackLossCallback = trackLost;
        std::cout << "[DYNAMIC_TRACKING] Tracking callbacks configured" << std::endl;
    }
    
    void configureTrackingParameters(double associationThresh, double confirmationThresh,
                                   double lossThresh, double procNoise, double measNoise) {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        associationThreshold = associationThresh;
        confirmationThreshold = confirmationThresh;
        trackLossThreshold = lossThresh;
        processNoise = procNoise;
        measurementNoise = measNoise;
        
        std::cout << "[DYNAMIC_TRACKING] Tracking parameters configured:" << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Association threshold: " << associationThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Confirmation threshold: " << confirmationThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Track loss threshold: " << trackLossThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Process noise: " << processNoise << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Measurement noise: " << measurementNoise << std::endl;
    }
    
    void clearLostTracks() {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        auto currentTime = std::chrono::steady_clock::now();
        size_t removedCount = 0;
        
        auto it = trackedObstacles.begin();
        while (it != trackedObstacles.end()) {
            auto timeSinceUpdate = std::chrono::duration<double>(currentTime - it->second.lastUpdate);
            
            if (timeSinceUpdate.count() > 10.0 || it->second.missedDetections > 5) {
                std::cout << "[DYNAMIC_TRACKING] Removing lost track: " << it->first << std::endl;
                
                if (trackLossCallback) {
                    trackLossCallback(it->first);
                }
                
                it = trackedObstacles.erase(it);
                removedCount++;
                metrics.lostTracks++;
            } else {
                ++it;
            }
        }
        
        if (removedCount > 0) {
            std::cout << "[DYNAMIC_TRACKING] Cleared " << removedCount << " lost tracks" << std::endl;
        }
    }
    
    TrackingMetrics getTrackingMetrics() const {
        std::lock_guard<std::mutex> lock(trackingMutex);
        return metrics;
    }
    
    void generateTrackingReport() const {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        std::cout << "\n[DYNAMIC_TRACKING] === DYNAMIC OBSTACLE TRACKING REPORT ===" << std::endl;
        
        std::cout << "[DYNAMIC_TRACKING] System Configuration:" << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Association threshold: " << associationThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Confirmation threshold: " << confirmationThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Track loss threshold: " << trackLossThreshold << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Process noise: " << processNoise << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Measurement noise: " << measurementNoise << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Prediction enabled: " << (enablePrediction ? "Yes" : "No") << std::endl;
        
        std::cout << "[DYNAMIC_TRACKING] Current State:" << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Active tracking: " << (trackingActive.load() ? "Yes" : "No") << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Tracked obstacles: " << trackedObstacles.size() << std::endl;
        
        size_t confirmedCount = 0;
        for (const auto& [id, obstacle] : trackedObstacles) {
            if (obstacle.isConfirmed) {
                confirmedCount++;
            }
        }
        std::cout << "[DYNAMIC_TRACKING]   Confirmed obstacles: " << confirmedCount << std::endl;
        
        {
            std::lock_guard<std::mutex> detectionLock(detectionQueueMutex);
            std::cout << "[DYNAMIC_TRACKING]   Pending detections: " << detectionQueue.size() << std::endl;
        }
        
        std::cout << "[DYNAMIC_TRACKING] Performance Metrics:" << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Total obstacles tracked: " << metrics.totalObstaclesTracked << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Lost tracks: " << metrics.lostTracks << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Detection associations: " << metrics.detectionAssociations << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   False tracks: " << metrics.falseTracks << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Average tracking duration: " << metrics.averageTrackingDuration << "s" << std::endl;
        std::cout << "[DYNAMIC_TRACKING]   Average tracking accuracy: " << metrics.averageTrackingAccuracy << std::endl;
        
        if (metrics.totalObstaclesTracked > 0) {
            double trackLossRate = static_cast<double>(metrics.lostTracks) / metrics.totalObstaclesTracked * 100.0;
            std::cout << "[DYNAMIC_TRACKING]   Track loss rate: " << trackLossRate << "%" << std::endl;
        }
        
        std::cout << "[DYNAMIC_TRACKING] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeTrackingMetrics() {
        metrics.totalObstaclesTracked = 0;
        metrics.activeTracking = 0;
        metrics.confirmedObstacles = 0;
        metrics.lostTracks = 0;
        metrics.averageTrackingDuration = 0.0;
        metrics.averageTrackingAccuracy = 0.0;
        metrics.lastTrackingUpdate = std::chrono::steady_clock::now();
        metrics.detectionAssociations = 0;
        metrics.falseTracks = 0;
    }
    
    void trackingLoop() {
        std::cout << "[DYNAMIC_TRACKING] Obstacle tracking monitoring loop initiated" << std::endl;
        
        while (trackingActive.load()) {
            try {
                processDetectionQueue();
                updateTrackedObstacles();
                performTrackMaintenance();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
            } catch (const std::exception& e) {
                std::cout << "[DYNAMIC_TRACKING] Exception in tracking loop: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        std::cout << "[DYNAMIC_TRACKING] Obstacle tracking monitoring loop terminated" << std::endl;
    }
    
    void processDetectionQueue() {
        std::vector<ObstacleDetection> currentDetections;
        
        {
            std::lock_guard<std::mutex> lock(detectionQueueMutex);
            while (!detectionQueue.empty()) {
                currentDetections.push_back(detectionQueue.front());
                detectionQueue.pop();
            }
        }
        
        if (currentDetections.empty()) {
            return;
        }
        
        std::lock_guard<std::mutex> trackingLock(trackingMutex);
        
        for (const ObstacleDetection& detection : currentDetections) {
            associateDetectionWithTrack(detection);
        }
    }
    
    void associateDetectionWithTrack(const ObstacleDetection& detection) {
        int bestTrackId = -1;
        double bestDistance = std::numeric_limits<double>::max();
        
        // Find the closest existing track
        for (auto& [trackId, obstacle] : trackedObstacles) {
            double distance = calculateDistance(detection.x, detection.y, 
                                              obstacle.state.x, obstacle.state.y);
            
            if (distance < associationThreshold && distance < bestDistance) {
                bestDistance = distance;
                bestTrackId = trackId;
            }
        }
        
        if (bestTrackId != -1) {
            // Associate with existing track
            updateTrackWithDetection(trackedObstacles[bestTrackId], detection);
            metrics.detectionAssociations++;
            
            std::cout << "[DYNAMIC_TRACKING] Detection associated with track " << bestTrackId 
                      << " at distance " << bestDistance << std::endl;
        } else {
            // Create new track
            createNewTrack(detection);
        }
    }
    
    void createNewTrack(const ObstacleDetection& detection) {
        DynamicObstacle newObstacle;
        newObstacle.obstacleId = nextObstacleId++;
        newObstacle.firstDetection = detection.timestamp;
        newObstacle.lastUpdate = detection.timestamp;
        newObstacle.trackingConfidence = detection.confidence;
        newObstacle.isConfirmed = false;
        newObstacle.consecutiveDetections = 1;
        newObstacle.missedDetections = 0;
        newObstacle.classification = "unknown";
        newObstacle.estimatedSize = 1.0;
        newObstacle.maxVelocity = 0.0;
        
        // Initialize Kalman state
        initializeKalmanState(newObstacle.state, detection.x, detection.y);
        
        // Add detection to history
        newObstacle.detectionHistory.push_back(detection);
        
        trackedObstacles[newObstacle.obstacleId] = newObstacle;
        metrics.totalObstaclesTracked++;
        
        std::cout << "[DYNAMIC_TRACKING] Created new track " << newObstacle.obstacleId 
                  << " at position (" << detection.x << "," << detection.y << ")" << std::endl;
    }
    
    void updateTrackWithDetection(DynamicObstacle& obstacle, const ObstacleDetection& detection) {
        // Predict step
        predictKalmanState(obstacle.state, detection.timestamp - obstacle.lastUpdate);
        
        // Update step
        updateKalmanState(obstacle.state, detection.x, detection.y, measurementNoise);
        
        // Update obstacle properties
        obstacle.lastUpdate = detection.timestamp;
        obstacle.consecutiveDetections++;
        obstacle.missedDetections = 0;
        obstacle.detectionHistory.push_back(detection);
        
        // Update tracking confidence
        obstacle.trackingConfidence = std::min(1.0, obstacle.trackingConfidence + 0.1);
        
        // Check for confirmation
        if (!obstacle.isConfirmed && obstacle.consecutiveDetections >= 3 && 
            obstacle.trackingConfidence > confirmationThreshold) {
            
            obstacle.isConfirmed = true;
            metrics.confirmedObstacles++;
            
            std::cout << "[DYNAMIC_TRACKING] Track " << obstacle.obstacleId << " confirmed" << std::endl;
            
            if (obstacleCallback) {
                obstacleCallback(obstacle);
            }
        }
        
        // Update velocity estimate
        updateVelocityEstimate(obstacle);
        
        // Limit detection history
        if (obstacle.detectionHistory.size() > 20) {
            obstacle.detectionHistory.erase(obstacle.detectionHistory.begin());
        }
    }
    
    void updateTrackedObstacles() {
        std::lock_guard<std::mutex> lock(trackingMutex);
        
        auto currentTime = std::chrono::steady_clock::now();
        
        for (auto& [trackId, obstacle] : trackedObstacles) {
            auto timeSinceUpdate = std::chrono::duration<double>(currentTime - obstacle.lastUpdate);
            
            if (timeSinceUpdate.count() > 0.5) { // If not updated recently
                obstacle.missedDetections++;
                obstacle.trackingConfidence = std::max(0.0, obstacle.trackingConfidence - 0.05);
                
                // Predict forward in time
                if (enablePrediction) {
                    predictKalmanState(obstacle.state, timeSinceUpdate);
                    obstacle.lastUpdate = currentTime;
                }
            }
        }
    }
    
    void performTrackMaintenance() {
        clearLostTracks();
        updateTrackingMetrics();
    }
    
    void initializeKalmanState(KalmanState& state, double x, double y) {
        state.x = x;
        state.y = y;
        state.vx = 0.0;
        state.vy = 0.0;
        state.ax = 0.0;
        state.ay = 0.0;
        
        // Initialize uncertainties
        state.px = 1.0;
        state.py = 1.0;
        state.pvx = 1.0;
        state.pvy = 1.0;
        state.pax = 0.5;
        state.pay = 0.5;
    }
    
    void predictKalmanState(KalmanState& state, std::chrono::duration<double> deltaTime) {
        double dt = deltaTime.count();
        
        if (dt <= 0) return;
        
        // Predict position and velocity
        state.x += state.vx * dt + 0.5 * state.ax * dt * dt;
        state.y += state.vy * dt + 0.5 * state.ay * dt * dt;
        state.vx += state.ax * dt;
        state.vy += state.ay * dt;
        
        // Predict uncertainties (simplified)
        state.px += state.pvx * dt + processNoise * dt * dt;
        state.py += state.pvy * dt + processNoise * dt * dt;
        state.pvx += processNoise * dt;
        state.pvy += processNoise * dt;
    }
    
    void updateKalmanState(KalmanState& state, double measX, double measY, double measNoise) {
        // Kalman gain calculation (simplified)
        double kx = state.px / (state.px + measNoise);
        double ky = state.py / (state.py + measNoise);
        
        // Update estimates
        state.x = state.x + kx * (measX - state.x);
        state.y = state.y + ky * (measY - state.y);
        
        // Update uncertainties
        state.px = (1.0 - kx) * state.px;
        state.py = (1.0 - ky) * state.py;
    }
    
    void updateVelocityEstimate(DynamicObstacle& obstacle) {
        if (obstacle.detectionHistory.size() < 2) {
            return;
        }
        
        const auto& latest = obstacle.detectionHistory.back();
        const auto& previous = obstacle.detectionHistory[obstacle.detectionHistory.size() - 2];
        
        auto timeDiff = std::chrono::duration<double>(latest.timestamp - previous.timestamp);
        if (timeDiff.count() <= 0) {
            return;
        }
        
        double dx = latest.x - previous.x;
        double dy = latest.y - previous.y;
        double velocity = std::sqrt(dx * dx + dy * dy) / timeDiff.count();
        
        obstacle.maxVelocity = std::max(obstacle.maxVelocity, velocity);
        
        // Update acceleration estimate
        if (obstacle.detectionHistory.size() >= 3) {
            const auto& prevPrev = obstacle.detectionHistory[obstacle.detectionHistory.size() - 3];
            auto prevTimeDiff = std::chrono::duration<double>(previous.timestamp - prevPrev.timestamp);
            
            if (prevTimeDiff.count() > 0) {
                double prevDx = previous.x - prevPrev.x;
                double prevDy = previous.y - prevPrev.y;
                double prevVelocity = std::sqrt(prevDx * prevDx + prevDy * prevDy) / prevTimeDiff.count();
                
                double acceleration = (velocity - prevVelocity) / timeDiff.count();
                obstacle.state.ax = (dx / std::sqrt(dx * dx + dy * dy)) * acceleration;
                obstacle.state.ay = (dy / std::sqrt(dx * dx + dy * dy)) * acceleration;
            }
        }
    }
    
    double calculateDistance(double x1, double y1, double x2, double y2) {
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    
    void updateTrackingMetrics() {
        metrics.activeTracking = trackedObstacles.size();
        metrics.lastTrackingUpdate = std::chrono::steady_clock::now();
        
        // Calculate average tracking duration
        if (!trackedObstacles.empty()) {
            double totalDuration = 0.0;
            auto currentTime = std::chrono::steady_clock::now();
            
            for (const auto& [id, obstacle] : trackedObstacles) {
                auto duration = std::chrono::duration<double>(currentTime - obstacle.firstDetection);
                totalDuration += duration.count();
            }
            
            metrics.averageTrackingDuration = totalDuration / trackedObstacles.size();
        }
    }
};