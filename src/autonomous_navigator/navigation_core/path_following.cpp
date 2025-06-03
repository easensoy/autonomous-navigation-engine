#include "autonomous_navigator/NavigationCore.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <numeric>

class PathFollowingController {
private:
    NavigationCore* navigationCore;
    std::vector<int> currentFollowingPath;
    size_t currentWaypointIndex;
    double followingSpeed;
    double waypointTolerance;
    double lookaheadDistance;
    bool pathFollowingActive;
    bool smoothFollowing;
    
    struct FollowingMetrics {
        double totalDistance;
        double distanceTraveled;
        double averageSpeed;
        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point lastWaypointTime;
        std::vector<double> segmentTimes;
        double pathDeviation;
        size_t waypointsCompleted;
    };
    
    FollowingMetrics currentMetrics;
    std::vector<int> deviationPoints;
    bool adaptiveSpeedEnabled;
    double minFollowingSpeed;
    double maxFollowingSpeed;
    
public:
    explicit PathFollowingController(NavigationCore* navCore) 
        : navigationCore(navCore), currentWaypointIndex(0), followingSpeed(1.0),
          waypointTolerance(0.1), lookaheadDistance(2.0), pathFollowingActive(false),
          smoothFollowing(true), adaptiveSpeedEnabled(true), 
          minFollowingSpeed(0.2), maxFollowingSpeed(2.0) {}
    
    bool startPathFollowing(const std::vector<int>& path) {
        std::cout << "[PATH_FOLLOWING] Starting path following with " << path.size() 
                  << " waypoints" << std::endl;
        
        if (path.empty()) {
            std::cout << "[PATH_FOLLOWING] Cannot start following empty path" << std::endl;
            return false;
        }
        
        if (pathFollowingActive) {
            std::cout << "[PATH_FOLLOWING] Path following already active, stopping current path" << std::endl;
            stopPathFollowing();
        }
        
        // Initialize path following state
        currentFollowingPath = path;
        currentWaypointIndex = 0;
        pathFollowingActive = true;
        
        // Initialize metrics
        initializeFollowingMetrics();
        
        // Validate path before starting
        if (!validatePathForFollowing(path)) {
            std::cout << "[PATH_FOLLOWING] Path validation failed, cannot start following" << std::endl;
            pathFollowingActive = false;
            return false;
        }
        
        // Calculate path characteristics for optimal following
        analyzePathCharacteristics();
        
        std::cout << "[PATH_FOLLOWING] Path following started successfully" << std::endl;
        return true;
    }
    
    bool executePathFollowingStep() {
        if (!pathFollowingActive) {
            return false;
        }
        
        if (currentWaypointIndex >= currentFollowingPath.size()) {
            std::cout << "[PATH_FOLLOWING] Path following completed" << std::endl;
            completePathFollowing();
            return false;
        }
        
        // Get current target waypoint
        int targetWaypoint = currentFollowingPath[currentWaypointIndex];
        
        std::cout << "[PATH_FOLLOWING] Following to waypoint " << targetWaypoint 
                  << " (" << (currentWaypointIndex + 1) << "/" << currentFollowingPath.size() << ")" << std::endl;
        
        // Execute movement toward target waypoint
        bool waypointReached = moveTowardWaypoint(targetWaypoint);
        
        if (waypointReached) {
            handleWaypointReached(targetWaypoint);
        }
        
        // Update following metrics
        updateFollowingMetrics();
        
        // Perform adaptive adjustments if enabled
        if (adaptiveSpeedEnabled) {
            adjustFollowingParameters();
        }
        
        // Check for path deviations
        monitorPathDeviation();
        
        return pathFollowingActive;
    }
    
    void stopPathFollowing() {
        if (!pathFollowingActive) {
            return;
        }
        
        std::cout << "[PATH_FOLLOWING] Stopping path following" << std::endl;
        
        pathFollowingActive = false;
        
        // Generate following completion report
        generateFollowingReport();
        
        // Clean up following state
        currentFollowingPath.clear();
        currentWaypointIndex = 0;
        deviationPoints.clear();
        
        std::cout << "[PATH_FOLLOWING] Path following stopped" << std::endl;
    }
    
    void pausePathFollowing() {
        if (pathFollowingActive) {
            std::cout << "[PATH_FOLLOWING] Pausing path following at waypoint " 
                      << currentWaypointIndex << std::endl;
            pathFollowingActive = false;
        }
    }
    
    void resumePathFollowing() {
        if (!pathFollowingActive && !currentFollowingPath.empty()) {
            std::cout << "[PATH_FOLLOWING] Resuming path following from waypoint " 
                      << currentWaypointIndex << std::endl;
            pathFollowingActive = true;
            
            // Recalibrate following parameters after pause
            recalibrateAfterPause();
        }
    }
    
    void setFollowingSpeed(double speed) {
        followingSpeed = std::max(minFollowingSpeed, std::min(maxFollowingSpeed, speed));
        std::cout << "[PATH_FOLLOWING] Following speed set to " << followingSpeed << std::endl;
    }
    
    void setWaypointTolerance(double tolerance) {
        waypointTolerance = tolerance;
        std::cout << "[PATH_FOLLOWING] Waypoint tolerance set to " << tolerance << std::endl;
    }
    
    void setLookaheadDistance(double distance) {
        lookaheadDistance = distance;
        std::cout << "[PATH_FOLLOWING] Lookahead distance set to " << distance << std::endl;
    }
    
    void enableSmoothFollowing(bool enable) {
        smoothFollowing = enable;
        std::cout << "[PATH_FOLLOWING] Smooth following " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void enableAdaptiveSpeed(bool enable) {
        adaptiveSpeedEnabled = enable;
        std::cout << "[PATH_FOLLOWING] Adaptive speed control " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    bool isPathFollowingActive() const {
        return pathFollowingActive;
    }
    
    double getFollowingProgress() const {
        if (currentFollowingPath.empty()) {
            return 0.0;
        }
        return static_cast<double>(currentWaypointIndex) / currentFollowingPath.size();
    }
    
    double getRemainingDistance() const {
        if (!pathFollowingActive || currentWaypointIndex >= currentFollowingPath.size()) {
            return 0.0;
        }
        
        double remainingDistance = 0.0;
        for (size_t i = currentWaypointIndex; i < currentFollowingPath.size() - 1; ++i) {
            // Calculate distance between consecutive waypoints
            remainingDistance += calculateWaypointDistance(currentFollowingPath[i], currentFollowingPath[i + 1]);
        }
        
        return remainingDistance;
    }
    
    std::vector<int> getCurrentPath() const {
        return currentFollowingPath;
    }
    
    size_t getCurrentWaypointIndex() const {
        return currentWaypointIndex;
    }
    
    void handlePathObstruction(const std::vector<int>& obstructedWaypoints) {
        std::cout << "[PATH_FOLLOWING] Handling path obstruction at " 
                  << obstructedWaypoints.size() << " waypoints" << std::endl;
        
        if (!pathFollowingActive) {
            return;
        }
        
        // Check if current path segment is obstructed
        bool currentSegmentObstructed = false;
        for (int obstructedWaypoint : obstructedWaypoints) {
            for (size_t i = currentWaypointIndex; i < std::min(currentWaypointIndex + 3, currentFollowingPath.size()); ++i) {
                if (currentFollowingPath[i] == obstructedWaypoint) {
                    currentSegmentObstructed = true;
                    break;
                }
            }
            if (currentSegmentObstructed) break;
        }
        
        if (currentSegmentObstructed) {
            std::cout << "[PATH_FOLLOWING] Current path segment obstructed, requesting replanning" << std::endl;
            
            // Pause following and request new path
            pausePathFollowing();
            
            // Delegate to navigation core for replanning
            navigationCore->handleObstacles(obstructedWaypoints);
            
            // Get updated path and resume following
            std::vector<int> newPath = navigationCore->getCurrentPath();
            if (!newPath.empty()) {
                updateFollowingPath(newPath);
                resumePathFollowing();
            } else {
                std::cout << "[PATH_FOLLOWING] Failed to get updated path, stopping following" << std::endl;
                stopPathFollowing();
            }
        } else {
            std::cout << "[PATH_FOLLOWING] Obstruction does not affect current path segment, continuing" << std::endl;
        }
    }
    
    void updateFollowingPath(const std::vector<int>& newPath) {
        std::cout << "[PATH_FOLLOWING] Updating following path (length: " 
                  << newPath.size() << ")" << std::endl;
        
        if (newPath.empty()) {
            std::cout << "[PATH_FOLLOWING] Cannot update to empty path" << std::endl;
            return;
        }
        
        // Find the best insertion point in the new path
        size_t newStartIndex = findBestPathAlignment(currentFollowingPath, newPath, currentWaypointIndex);
        
        currentFollowingPath = newPath;
        currentWaypointIndex = newStartIndex;
        
        // Recalculate path characteristics
        analyzePathCharacteristics();
        
        std::cout << "[PATH_FOLLOWING] Path updated, resuming from waypoint index " << newStartIndex << std::endl;
    }
    
private:
    bool validatePathForFollowing(const std::vector<int>& path) {
        std::cout << "[PATH_FOLLOWING] Validating path for following" << std::endl;
        
        // Check path connectivity
        for (size_t i = 0; i < path.size() - 1; ++i) {
            if (!navigationCore->getCurrentEnvironment() || 
                !navigationCore->getCurrentEnvironment()->hasEdge(path[i], path[i + 1])) {
                std::cout << "[PATH_FOLLOWING] Path validation failed: No edge between waypoints " 
                          << path[i] << " and " << path[i + 1] << std::endl;
                return false;
            }
        }
        
        // Check for path loops that might cause following issues
        std::unordered_set<int> visitedWaypoints;
        for (int waypoint : path) {
            if (visitedWaypoints.find(waypoint) != visitedWaypoints.end()) {
                std::cout << "[PATH_FOLLOWING] Warning: Path contains repeated waypoint " << waypoint << std::endl;
            }
            visitedWaypoints.insert(waypoint);
        }
        
        std::cout << "[PATH_FOLLOWING] Path validation completed successfully" << std::endl;
        return true;
    }
    
    void initializeFollowingMetrics() {
        currentMetrics = FollowingMetrics{};
        currentMetrics.startTime = std::chrono::steady_clock::now();
        currentMetrics.lastWaypointTime = currentMetrics.startTime;
        currentMetrics.totalDistance = calculateTotalPathDistance();
        currentMetrics.segmentTimes.clear();
        currentMetrics.segmentTimes.resize(currentFollowingPath.size(), 0.0);
        currentMetrics.waypointsCompleted = 0;
        currentMetrics.pathDeviation = 0.0;
        currentMetrics.distanceTraveled = 0.0;
        currentMetrics.averageSpeed = 0.0;
        
        std::cout << "[PATH_FOLLOWING] Following metrics initialized, total path distance: " 
                  << currentMetrics.totalDistance << std::endl;
    }
    
    void analyzePathCharacteristics() {
        std::cout << "[PATH_FOLLOWING] Analyzing path characteristics for optimal following" << std::endl;
        
        if (currentFollowingPath.size() < 2) {
            return;
        }
        
        // Analyze path curvature and adjust following parameters
        std::vector<double> segmentAngles;
        for (size_t i = 1; i < currentFollowingPath.size() - 1; ++i) {
            double angle = calculatePathCurvature(i);
            segmentAngles.push_back(angle);
        }
        
        // Calculate average path complexity
        double avgCurvature = 0.0;
        if (!segmentAngles.empty()) {
            avgCurvature = std::accumulate(segmentAngles.begin(), segmentAngles.end(), 0.0) / segmentAngles.size();
        }
        
        // Adjust following parameters based on path complexity
        if (avgCurvature > 45.0) { // High curvature path
            setFollowingSpeed(followingSpeed * 0.8); // Reduce speed for complex paths
            lookaheadDistance = std::max(1.0, lookaheadDistance * 0.7); // Reduce lookahead
        } else if (avgCurvature < 15.0) { // Straight path
            setFollowingSpeed(std::min(maxFollowingSpeed, followingSpeed * 1.2)); // Increase speed for straight paths
            lookaheadDistance = std::min(5.0, lookaheadDistance * 1.3); // Increase lookahead
        }
        
        std::cout << "[PATH_FOLLOWING] Path analysis complete, average curvature: " 
                  << avgCurvature << " degrees" << std::endl;
    }
    
    bool moveTowardWaypoint(int targetWaypoint) {
        // Simulate movement toward waypoint
        std::cout << "[PATH_FOLLOWING] Moving toward waypoint " << targetWaypoint << std::endl;
        
        // Calculate movement parameters
        double movementSpeed = calculateDynamicSpeed();
        
        // Simulate movement time based on speed
        auto movementTime = std::chrono::milliseconds(static_cast<int>(1000.0 / movementSpeed));
        std::this_thread::sleep_for(movementTime);
        
        // Apply smooth following if enabled
        if (smoothFollowing) {
            applySmoothFollowing(targetWaypoint);
        }
        
        // Check if waypoint is reached within tolerance
        bool waypointReached = isWaypointReached(targetWaypoint);
        
        if (waypointReached) {
            std::cout << "[PATH_FOLLOWING] Waypoint " << targetWaypoint << " reached" << std::endl;
        }
        
        return waypointReached;
    }
    
    bool isWaypointReached(int waypoint) const {
        // Simulate waypoint proximity check
        // In a real implementation, this would check actual position vs waypoint position
        
        // For simulation, we consider waypoint reached based on following parameters
        static int stepCount = 0;
        stepCount++;
        
        // Simulate varying approach times based on following speed
        int stepsToReach = static_cast<int>(2.0 / followingSpeed);
        return (stepCount % stepsToReach) == 0;
    }
    
    void handleWaypointReached(int waypoint) {
        std::cout << "[PATH_FOLLOWING] Handling waypoint " << waypoint << " completion" << std::endl;
        
        auto currentTime = std::chrono::steady_clock::now();
        
        // Record timing for this segment
        if (currentWaypointIndex < currentMetrics.segmentTimes.size()) {
            auto segmentDuration = std::chrono::duration<double>(currentTime - currentMetrics.lastWaypointTime);
            currentMetrics.segmentTimes[currentWaypointIndex] = segmentDuration.count();
        }
        
        // Update metrics
        currentMetrics.waypointsCompleted++;
        currentMetrics.lastWaypointTime = currentTime;
        
        // Calculate distance traveled for this segment
        if (currentWaypointIndex > 0) {
            double segmentDistance = calculateWaypointDistance(
                currentFollowingPath[currentWaypointIndex - 1], waypoint);
            currentMetrics.distanceTraveled += segmentDistance;
        }
        
        // Move to next waypoint
        currentWaypointIndex++;
        
        // Prepare for next waypoint if available
        if (currentWaypointIndex < currentFollowingPath.size()) {
            prepareForNextWaypoint();
        }
        
        std::cout << "[PATH_FOLLOWING] Waypoint completion processed, progress: " 
                  << (getFollowingProgress() * 100.0) << "%" << std::endl;
    }
    
    void updateFollowingMetrics() {
        auto currentTime = std::chrono::steady_clock::now();
        auto totalElapsed = std::chrono::duration<double>(currentTime - currentMetrics.startTime);
        
        if (totalElapsed.count() > 0 && currentMetrics.distanceTraveled > 0) {
            currentMetrics.averageSpeed = currentMetrics.distanceTraveled / totalElapsed.count();
        }
    }
    
    void adjustFollowingParameters() {
        // Adaptive speed adjustment based on performance
        double currentProgress = getFollowingProgress();
        
        if (currentProgress > 0.1) { // Only adjust after some progress
            double expectedTime = currentMetrics.totalDistance / followingSpeed;
            auto actualElapsed = std::chrono::steady_clock::now() - currentMetrics.startTime;
            double actualTime = std::chrono::duration<double>(actualElapsed).count();
            
            if (actualTime > expectedTime * 1.2) {
                // Running behind schedule, increase speed slightly
                setFollowingSpeed(std::min(maxFollowingSpeed, followingSpeed * 1.05));
                std::cout << "[PATH_FOLLOWING] Adaptive adjustment: Increased speed to " 
                          << followingSpeed << std::endl;
            } else if (actualTime < expectedTime * 0.8) {
                // Running ahead of schedule, can reduce speed for precision
                setFollowingSpeed(std::max(minFollowingSpeed, followingSpeed * 0.95));
                std::cout << "[PATH_FOLLOWING] Adaptive adjustment: Reduced speed to " 
                          << followingSpeed << std::endl;
            }
        }
    }
    
    void monitorPathDeviation() {
        // Monitor for deviations from intended path
        // In a real implementation, this would compare actual position with intended path
        
        // Simulate occasional path deviation detection
        static int deviationCheckCounter = 0;
        deviationCheckCounter++;
        
        if (deviationCheckCounter % 50 == 0) { // Check every 50 steps
            if (currentWaypointIndex < currentFollowingPath.size()) {
                // Simulate minor deviation detection
                double deviation = (rand() % 100) / 1000.0; // Random deviation up to 0.1
                
                if (deviation > waypointTolerance * 0.5) {
                    currentMetrics.pathDeviation += deviation;
                    deviationPoints.push_back(currentFollowingPath[currentWaypointIndex]);
                    
                    std::cout << "[PATH_FOLLOWING] Path deviation detected: " << deviation 
                              << " at waypoint " << currentFollowingPath[currentWaypointIndex] << std::endl;
                    
                    // Apply corrective action
                    applyCourseCorrection(deviation);
                }
            }
        }
    }
    
    void applySmoothFollowing(int targetWaypoint) {
        // Implement smooth path following using lookahead
        if (currentWaypointIndex + 1 < currentFollowingPath.size()) {
            int nextWaypoint = currentFollowingPath[currentWaypointIndex + 1];
            
            // Calculate smooth trajectory toward next waypoint
            std::cout << "[PATH_FOLLOWING] Applying smooth following toward waypoint " 
                      << nextWaypoint << " (lookahead)" << std::endl;
            
            // Adjust movement based on upcoming path segment
            double upcomingCurvature = calculatePathCurvature(currentWaypointIndex + 1);
            if (upcomingCurvature > 30.0) {
                // Prepare for sharp turn by reducing speed
                double adjustedSpeed = followingSpeed * 0.9;
                std::cout << "[PATH_FOLLOWING] Reducing speed for upcoming turn: " << adjustedSpeed << std::endl;
            }
        }
    }
    
    double calculateDynamicSpeed() const {
        double baseSpeed = followingSpeed;
        
        // Adjust speed based on path characteristics
        if (currentWaypointIndex + 1 < currentFollowingPath.size()) {
            double upcomingCurvature = calculatePathCurvature(currentWaypointIndex + 1);
            
            if (upcomingCurvature > 45.0) {
                baseSpeed *= 0.7; // Slow down for sharp turns
            } else if (upcomingCurvature < 15.0) {
                baseSpeed *= 1.1; // Speed up for straight segments
            }
        }
        
        return std::max(minFollowingSpeed, std::min(maxFollowingSpeed, baseSpeed));
    }
    
    double calculatePathCurvature(size_t waypointIndex) const {
        if (waypointIndex == 0 || waypointIndex >= currentFollowingPath.size() - 1) {
            return 0.0; // No curvature at endpoints
        }
        
        // Simulate curvature calculation
        // In a real implementation, this would calculate the angle between path segments
        int prevWaypoint = currentFollowingPath[waypointIndex - 1];
        int currentWaypoint = currentFollowingPath[waypointIndex];
        int nextWaypoint = currentFollowingPath[waypointIndex + 1];
        
        // Simplified curvature calculation based on waypoint IDs
        double angle = std::abs((nextWaypoint - currentWaypoint) - (currentWaypoint - prevWaypoint)) * 5.0;
        return std::min(90.0, angle); // Cap at 90 degrees
    }
    
    double calculateWaypointDistance(int waypoint1, int waypoint2) const {
        // Simulate distance calculation between waypoints
        // In a real implementation, this would use actual coordinates
        return std::abs(waypoint2 - waypoint1) * 0.5; // Simplified distance metric
    }
    
    double calculateTotalPathDistance() const {
        double totalDistance = 0.0;
        
        for (size_t i = 0; i < currentFollowingPath.size() - 1; ++i) {
            totalDistance += calculateWaypointDistance(currentFollowingPath[i], currentFollowingPath[i + 1]);
        }
        
        return totalDistance;
    }
    
    void prepareForNextWaypoint() {
        if (currentWaypointIndex >= currentFollowingPath.size()) {
            return;
        }
        
        int nextWaypoint = currentFollowingPath[currentWaypointIndex];
        std::cout << "[PATH_FOLLOWING] Preparing for next waypoint: " << nextWaypoint << std::endl;
        
        // Pre-calculate parameters for upcoming segment
        if (adaptiveSpeedEnabled) {
            double segmentComplexity = calculatePathCurvature(currentWaypointIndex);
            
            if (segmentComplexity > 30.0) {
                std::cout << "[PATH_FOLLOWING] Upcoming complex segment, adjusting parameters" << std::endl;
            }
        }
    }
    
    void applyCourseCorrection(double deviation) {
        std::cout << "[PATH_FOLLOWING] Applying course correction for deviation: " << deviation << std::endl;
        
        // Temporarily reduce speed for precision correction
        double correctionSpeed = followingSpeed * 0.8;
        
        // Simulate correction time
        auto correctionDelay = std::chrono::milliseconds(static_cast<int>(deviation * 1000));
        std::this_thread::sleep_for(correctionDelay);
        
        std::cout << "[PATH_FOLLOWING] Course correction applied" << std::endl;
    }
    
    void recalibrateAfterPause() {
        std::cout << "[PATH_FOLLOWING] Recalibrating following parameters after pause" << std::endl;
        
        // Reset timing references
        currentMetrics.lastWaypointTime = std::chrono::steady_clock::now();
        
        // Gradually restore following speed
        setFollowingSpeed(followingSpeed * 0.8); // Start slower after pause
        
        std::cout << "[PATH_FOLLOWING] Recalibration completed" << std::endl;
    }
    
    size_t findBestPathAlignment(const std::vector<int>& oldPath, const std::vector<int>& newPath, size_t currentIndex) const {
        if (currentIndex >= oldPath.size() || newPath.empty()) {
            return 0;
        }
        
        int currentWaypoint = oldPath[currentIndex];
        
        // Find the current waypoint in the new path
        auto it = std::find(newPath.begin(), newPath.end(), currentWaypoint);
        if (it != newPath.end()) {
            return std::distance(newPath.begin(), it);
        }
        
        // If current waypoint not found, find closest waypoint
        size_t bestIndex = 0;
        int minDistance = std::abs(newPath[0] - currentWaypoint);
        
        for (size_t i = 1; i < newPath.size(); ++i) {
            int distance = std::abs(newPath[i] - currentWaypoint);
            if (distance < minDistance) {
                minDistance = distance;
                bestIndex = i;
            }
        }
        
        return bestIndex;
    }
    
    void completePathFollowing() {
        std::cout << "[PATH_FOLLOWING] Path following completed successfully" << std::endl;
        
        pathFollowingActive = false;
        
        // Update final metrics
        auto completionTime = std::chrono::steady_clock::now();
        auto totalDuration = std::chrono::duration<double>(completionTime - currentMetrics.startTime);
        currentMetrics.averageSpeed = currentMetrics.totalDistance / totalDuration.count();
        
        generateFollowingReport();
    }
    
    void generateFollowingReport() const {
        std::cout << "\n[PATH_FOLLOWING] === PATH FOLLOWING REPORT ===" << std::endl;
        
        auto currentTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration<double>(currentTime - currentMetrics.startTime);
        
        std::cout << "[PATH_FOLLOWING] Path completion status: " 
                  << (currentWaypointIndex >= currentFollowingPath.size() ? "COMPLETED" : "INCOMPLETE") << std::endl;
        std::cout << "[PATH_FOLLOWING] Total waypoints: " << currentFollowingPath.size() << std::endl;
        std::cout << "[PATH_FOLLOWING] Waypoints completed: " << currentMetrics.waypointsCompleted << std::endl;
        std::cout << "[PATH_FOLLOWING] Progress: " << (getFollowingProgress() * 100.0) << "%" << std::endl;
        std::cout << "[PATH_FOLLOWING] Total distance: " << currentMetrics.totalDistance << std::endl;
        std::cout << "[PATH_FOLLOWING] Distance traveled: " << currentMetrics.distanceTraveled << std::endl;
        std::cout << "[PATH_FOLLOWING] Total time: " << totalTime.count() << " seconds" << std::endl;
        std::cout << "[PATH_FOLLOWING] Average speed: " << currentMetrics.averageSpeed << std::endl;
        std::cout << "[PATH_FOLLOWING] Total path deviation: " << currentMetrics.pathDeviation << std::endl;
        std::cout << "[PATH_FOLLOWING] Deviation points: " << deviationPoints.size() << std::endl;
        
        if (!currentMetrics.segmentTimes.empty()) {
            double avgSegmentTime = std::accumulate(currentMetrics.segmentTimes.begin(), 
                                                   currentMetrics.segmentTimes.end(), 0.0) / 
                                   currentMetrics.segmentTimes.size();
            std::cout << "[PATH_FOLLOWING] Average segment time: " << avgSegmentTime << " seconds" << std::endl;
        }
        
        std::cout << "[PATH_FOLLOWING] === END REPORT ===" << std::endl;
    }
};