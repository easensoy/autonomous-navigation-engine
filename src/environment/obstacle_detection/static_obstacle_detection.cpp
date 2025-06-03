#include "environment/ObstacleDetection.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <fstream>
#include <queue>

class StaticObstacleDetector {
private:
    const Graph* environment;
    mutable std::mutex detectionMutex;
    std::atomic<bool> detectionActive;
    std::thread detectionThread;
    
    struct StaticObstacle {
        int obstacleId;
        double x, y;
        double width, height;
        std::chrono::steady_clock::time_point firstDetection;
        std::chrono::steady_clock::time_point lastConfirmation;
        double confidence;
        std::string obstacleType;
        bool isConfirmed;
        bool isPersistent;
        int confirmationCount;
        std::vector<int> affectedNodes;
        std::vector<std::pair<int, int>> affectedEdges;
        std::string detectionSource;
    };
    
    struct ScanRegion {
        double minX, minY, maxX, maxY;
        std::chrono::steady_clock::time_point lastScanned;
        int priority;
        bool requiresRescanning;
    };
    
    struct DetectionMetrics {
        size_t totalObstaclesDetected;
        size_t confirmedObstacles;
        size_t falsePositives;
        size_t mapUpdates;
        double averageDetectionTime;
        double averageConfidence;
        std::chrono::steady_clock::time_point lastDetection;
        size_t scanRegionsProcessed;
        double mapCoverage;
    };
    
    std::unordered_map<int, StaticObstacle> detectedObstacles;
    std::vector<ScanRegion> scanRegions;
    std::queue<ScanRegion> scanQueue;
    mutable std::mutex scanQueueMutex;
    DetectionMetrics metrics;
    
    int nextObstacleId;
    double detectionThreshold;
    double confirmationThreshold;
    double scanResolution;
    double maxScanRange;
    bool persistentStorage;
    bool automaticMapUpdate;
    std::string persistenceFilePath;
    std::function<void(const StaticObstacle&)> obstacleDetectedCallback;
    std::function<void(int)> obstacleRemovedCallback;
    
public:
    explicit StaticObstacleDetector(const Graph* graph) 
        : environment(graph), detectionActive(false), nextObstacleId(2000),
          detectionThreshold(0.7), confirmationThreshold(0.85), scanResolution(0.5),
          maxScanRange(10.0), persistentStorage(true), automaticMapUpdate(false),
          persistenceFilePath("static_obstacles.dat") {
        
        if (!environment) {
            throw std::invalid_argument("Graph environment cannot be null");
        }
        
        initializeDetectionMetrics();
        initializeScanRegions();
        loadPersistedObstacles();
        
        std::cout << "[STATIC_DETECTION] Static obstacle detection system initialized" << std::endl;
    }
    
    ~StaticObstacleDetector() {
        stopDetection();
        if (persistentStorage) {
            persistObstacles();
        }
    }
    
    bool startDetection() {
        std::cout << "[STATIC_DETECTION] Initiating static obstacle detection services" << std::endl;
        
        if (detectionActive.load()) {
            std::cout << "[STATIC_DETECTION] Detection services already operational" << std::endl;
            return true;
        }
        
        detectionActive.store(true);
        detectionThread = std::thread(&StaticObstacleDetector::detectionLoop, this);
        
        std::cout << "[STATIC_DETECTION] Static obstacle detection services started successfully" << std::endl;
        return true;
    }
    
    void stopDetection() {
        if (!detectionActive.load()) {
            return;
        }
        
        std::cout << "[STATIC_DETECTION] Initiating detection shutdown sequence" << std::endl;
        
        detectionActive.store(false);
        
        if (detectionThread.joinable()) {
            detectionThread.join();
        }
        
        std::cout << "[STATIC_DETECTION] Static obstacle detection services terminated" << std::endl;
    }
    
    void performEnvironmentScan() {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        auto scanStart = std::chrono::steady_clock::now();
        std::cout << "[STATIC_DETECTION] Executing comprehensive environment scan" << std::endl;
        
        size_t obstaclesDetected = 0;
        
        for (ScanRegion& region : scanRegions) {
            if (performRegionScan(region)) {
                obstaclesDetected++;
            }
            region.lastScanned = scanStart;
        }
        
        updateDetectionMetrics(scanStart, obstaclesDetected);
        
        std::cout << "[STATIC_DETECTION] Environment scan completed, " 
                  << obstaclesDetected << " new obstacles detected" << std::endl;
    }
    
    void scanSpecificRegion(double minX, double minY, double maxX, double maxY) {
        std::cout << "[STATIC_DETECTION] Scanning specific region (" << minX << "," << minY 
                  << ") to (" << maxX << "," << maxY << ")" << std::endl;
        
        ScanRegion targetRegion;
        targetRegion.minX = minX;
        targetRegion.minY = minY;
        targetRegion.maxX = maxX;
        targetRegion.maxY = maxY;
        targetRegion.lastScanned = std::chrono::steady_clock::now();
        targetRegion.priority = 10; // High priority for specific scans
        targetRegion.requiresRescanning = false;
        
        {
            std::lock_guard<std::mutex> queueLock(scanQueueMutex);
            scanQueue.push(targetRegion);
        }
        
        std::cout << "[STATIC_DETECTION] Region scan queued for processing" << std::endl;
    }
    
    bool validateObstacleDetection(int obstacleId) {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::cout << "[STATIC_DETECTION] Validating obstacle detection: " << obstacleId << std::endl;
        
        auto it = detectedObstacles.find(obstacleId);
        if (it == detectedObstacles.end()) {
            std::cout << "[STATIC_DETECTION] Obstacle not found for validation" << std::endl;
            return false;
        }
        
        StaticObstacle& obstacle = it->second;
        
        // Perform validation checks
        bool structuralValidation = performStructuralValidation(obstacle);
        bool spatialValidation = performSpatialValidation(obstacle);
        bool consistencyValidation = performConsistencyValidation(obstacle);
        
        bool overallValidation = structuralValidation && spatialValidation && consistencyValidation;
        
        if (overallValidation) {
            obstacle.confirmationCount++;
            obstacle.confidence = std::min(1.0, obstacle.confidence + 0.1);
            obstacle.lastConfirmation = std::chrono::steady_clock::now();
            
            if (!obstacle.isConfirmed && obstacle.confidence > confirmationThreshold) {
                confirmObstacle(obstacle);
            }
        } else {
            obstacle.confidence = std::max(0.0, obstacle.confidence - 0.2);
            
            if (obstacle.confidence < 0.3) {
                std::cout << "[STATIC_DETECTION] Removing obstacle due to low confidence: " << obstacleId << std::endl;
                removeObstacle(obstacleId);
                return false;
            }
        }
        
        std::cout << "[STATIC_DETECTION] Obstacle validation completed - Result: " 
                  << (overallValidation ? "Valid" : "Invalid") 
                  << ", Confidence: " << obstacle.confidence << std::endl;
        
        return overallValidation;
    }
    
    void addStaticObstacle(double x, double y, double width, double height, 
                          const std::string& type, const std::string& source) {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::cout << "[STATIC_DETECTION] Adding static obstacle at (" << x << "," << y 
                  << ") size (" << width << "x" << height << ") type: " << type << std::endl;
        
        StaticObstacle newObstacle;
        newObstacle.obstacleId = nextObstacleId++;
        newObstacle.x = x;
        newObstacle.y = y;
        newObstacle.width = width;
        newObstacle.height = height;
        newObstacle.firstDetection = std::chrono::steady_clock::now();
        newObstacle.lastConfirmation = newObstacle.firstDetection;
        newObstacle.confidence = 0.9;
        newObstacle.obstacleType = type;
        newObstacle.isConfirmed = false;
        newObstacle.isPersistent = true;
        newObstacle.confirmationCount = 1;
        newObstacle.detectionSource = source;
        
        // Calculate affected nodes and edges
        calculateAffectedElements(newObstacle);
        
        detectedObstacles[newObstacle.obstacleId] = newObstacle;
        metrics.totalObstaclesDetected++;
        
        // Validate immediately for manually added obstacles
        if (validateObstacleDetection(newObstacle.obstacleId)) {
            std::cout << "[STATIC_DETECTION] Static obstacle added and validated successfully" << std::endl;
        }
    }
    
    bool removeObstacle(int obstacleId) {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        auto it = detectedObstacles.find(obstacleId);
        if (it == detectedObstacles.end()) {
            std::cout << "[STATIC_DETECTION] Cannot remove obstacle - not found: " << obstacleId << std::endl;
            return false;
        }
        
        std::cout << "[STATIC_DETECTION] Removing static obstacle: " << obstacleId << std::endl;
        
        if (obstacleRemovedCallback) {
            obstacleRemovedCallback(obstacleId);
        }
        
        detectedObstacles.erase(it);
        
        std::cout << "[STATIC_DETECTION] Static obstacle removed successfully" << std::endl;
        return true;
    }
    
    std::vector<StaticObstacle> getConfirmedObstacles() const {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::vector<StaticObstacle> confirmedObstacles;
        
        for (const auto& [obstacleId, obstacle] : detectedObstacles) {
            if (obstacle.isConfirmed) {
                confirmedObstacles.push_back(obstacle);
            }
        }
        
        return confirmedObstacles;
    }
    
    std::vector<int> getObstaclesInRegion(double minX, double minY, double maxX, double maxY) const {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::vector<int> obstaclesInRegion;
        
        for (const auto& [obstacleId, obstacle] : detectedObstacles) {
            if (obstacle.isConfirmed &&
                obstacle.x >= minX && obstacle.x <= maxX &&
                obstacle.y >= minY && obstacle.y <= maxY) {
                obstaclesInRegion.push_back(obstacleId);
            }
        }
        
        std::cout << "[STATIC_DETECTION] Found " << obstaclesInRegion.size() 
                  << " obstacles in specified region" << std::endl;
        
        return obstaclesInRegion;
    }
    
    void setDetectionCallbacks(std::function<void(const StaticObstacle&)> obstacleDetected,
                              std::function<void(int)> obstacleRemoved) {
        obstacleDetectedCallback = obstacleDetected;
        obstacleRemovedCallback = obstacleRemoved;
        std::cout << "[STATIC_DETECTION] Detection callbacks configured" << std::endl;
    }
    
    void configureDetectionParameters(double detectionThresh, double confirmationThresh,
                                    double resolution, double range, bool autoUpdate) {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        detectionThreshold = detectionThresh;
        confirmationThreshold = confirmationThresh;
        scanResolution = resolution;
        maxScanRange = range;
        automaticMapUpdate = autoUpdate;
        
        std::cout << "[STATIC_DETECTION] Detection parameters configured:" << std::endl;
        std::cout << "[STATIC_DETECTION]   Detection threshold: " << detectionThreshold << std::endl;
        std::cout << "[STATIC_DETECTION]   Confirmation threshold: " << confirmationThreshold << std::endl;
        std::cout << "[STATIC_DETECTION]   Scan resolution: " << scanResolution << std::endl;
        std::cout << "[STATIC_DETECTION]   Max scan range: " << maxScanRange << std::endl;
        std::cout << "[STATIC_DETECTION]   Automatic map update: " << (automaticMapUpdate ? "Enabled" : "Disabled") << std::endl;
    }
    
    void exportObstacleMap(const std::string& filename) const {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::cout << "[STATIC_DETECTION] Exporting obstacle map to file: " << filename << std::endl;
        
        try {
            std::ofstream file(filename);
            if (!file.is_open()) {
                std::cout << "[STATIC_DETECTION] Failed to open export file" << std::endl;
                return;
            }
            
            file << "# Static Obstacle Map Export\n";
            file << "# Format: ID,X,Y,Width,Height,Type,Confidence\n";
            
            for (const auto& [obstacleId, obstacle] : detectedObstacles) {
                if (obstacle.isConfirmed) {
                    file << obstacle.obstacleId << ","
                         << obstacle.x << ","
                         << obstacle.y << ","
                         << obstacle.width << ","
                         << obstacle.height << ","
                         << obstacle.obstacleType << ","
                         << obstacle.confidence << "\n";
                }
            }
            
            file.close();
            std::cout << "[STATIC_DETECTION] Obstacle map exported successfully" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[STATIC_DETECTION] Export failed: " << e.what() << std::endl;
        }
    }
    
    DetectionMetrics getDetectionMetrics() const {
        std::lock_guard<std::mutex> lock(detectionMutex);
        return metrics;
    }
    
    void generateDetectionReport() const {
        std::lock_guard<std::mutex> lock(detectionMutex);
        
        std::cout << "\n[STATIC_DETECTION] === STATIC OBSTACLE DETECTION REPORT ===" << std::endl;
        
        std::cout << "[STATIC_DETECTION] System Configuration:" << std::endl;
        std::cout << "[STATIC_DETECTION]   Detection threshold: " << detectionThreshold << std::endl;
        std::cout << "[STATIC_DETECTION]   Confirmation threshold: " << confirmationThreshold << std::endl;
        std::cout << "[STATIC_DETECTION]   Scan resolution: " << scanResolution << std::endl;
        std::cout << "[STATIC_DETECTION]   Max scan range: " << maxScanRange << std::endl;
        std::cout << "[STATIC_DETECTION]   Persistent storage: " << (persistentStorage ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[STATIC_DETECTION]   Automatic map update: " << (automaticMapUpdate ? "Enabled" : "Disabled") << std::endl;
        
        std::cout << "[STATIC_DETECTION] Current State:" << std::endl;
        std::cout << "[STATIC_DETECTION]   Detection active: " << (detectionActive.load() ? "Yes" : "No") << std::endl;
        std::cout << "[STATIC_DETECTION]   Total detected obstacles: " << detectedObstacles.size() << std::endl;
        
        size_t confirmedCount = 0;
        for (const auto& [id, obstacle] : detectedObstacles) {
            if (obstacle.isConfirmed) {
                confirmedCount++;
            }
        }
        std::cout << "[STATIC_DETECTION]   Confirmed obstacles: " << confirmedCount << std::endl;
        std::cout << "[STATIC_DETECTION]   Scan regions: " << scanRegions.size() << std::endl;
        
        {
            std::lock_guard<std::mutex> queueLock(scanQueueMutex);
            std::cout << "[STATIC_DETECTION]   Pending scans: " << scanQueue.size() << std::endl;
        }
        
        std::cout << "[STATIC_DETECTION] Performance Metrics:" << std::endl;
        std::cout << "[STATIC_DETECTION]   Total obstacles detected: " << metrics.totalObstaclesDetected << std::endl;
        std::cout << "[STATIC_DETECTION]   Confirmed obstacles: " << metrics.confirmedObstacles << std::endl;
        std::cout << "[STATIC_DETECTION]   False positives: " << metrics.falsePositives << std::endl;
        std::cout << "[STATIC_DETECTION]   Map updates: " << metrics.mapUpdates << std::endl;
        std::cout << "[STATIC_DETECTION]   Average detection time: " << metrics.averageDetectionTime << "s" << std::endl;
        std::cout << "[STATIC_DETECTION]   Average confidence: " << metrics.averageConfidence << std::endl;
        std::cout << "[STATIC_DETECTION]   Map coverage: " << (metrics.mapCoverage * 100.0) << "%" << std::endl;
        
        if (metrics.totalObstaclesDetected > 0) {
            double falsePositiveRate = static_cast<double>(metrics.falsePositives) / metrics.totalObstaclesDetected * 100.0;
            std::cout << "[STATIC_DETECTION]   False positive rate: " << falsePositiveRate << "%" << std::endl;
        }
        
        std::cout << "[STATIC_DETECTION] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeDetectionMetrics() {
        metrics.totalObstaclesDetected = 0;
        metrics.confirmedObstacles = 0;
        metrics.falsePositives = 0;
        metrics.mapUpdates = 0;
        metrics.averageDetectionTime = 0.0;
        metrics.averageConfidence = 0.0;
        metrics.lastDetection = std::chrono::steady_clock::now();
        metrics.scanRegionsProcessed = 0;
        metrics.mapCoverage = 0.0;
    }
    
    void initializeScanRegions() {
        std::cout << "[STATIC_DETECTION] Initializing scan regions based on environment bounds" << std::endl;
        
        // Create scan regions based on environment size
        double envMinX = -10.0, envMinY = -10.0, envMaxX = 10.0, envMaxY = 10.0;
        
        // Try to determine environment bounds from nodes
        if (!environment->isEmpty()) {
            bool firstNode = true;
            for (int nodeId : environment->getAllNodeIds()) {
                const Node& node = environment->getNode(nodeId);
                
                if (firstNode) {
                    envMinX = envMaxX = node.getX();
                    envMinY = envMaxY = node.getY();
                    firstNode = false;
                } else {
                    envMinX = std::min(envMinX, node.getX());
                    envMaxX = std::max(envMaxX, node.getX());
                    envMinY = std::min(envMinY, node.getY());
                    envMaxY = std::max(envMaxY, node.getY());
                }
            }
            
            // Add margin
            envMinX -= 5.0;
            envMinY -= 5.0;
            envMaxX += 5.0;
            envMaxY += 5.0;
        }
        
        // Create grid of scan regions
        double regionSize = 5.0;
        for (double x = envMinX; x < envMaxX; x += regionSize) {
            for (double y = envMinY; y < envMaxY; y += regionSize) {
                ScanRegion region;
                region.minX = x;
                region.minY = y;
                region.maxX = std::min(x + regionSize, envMaxX);
                region.maxY = std::min(y + regionSize, envMaxY);
                region.lastScanned = std::chrono::steady_clock::now() - std::chrono::hours(1);
                region.priority = 1;
                region.requiresRescanning = true;
                
                scanRegions.push_back(region);
            }
        }
        
        std::cout << "[STATIC_DETECTION] Initialized " << scanRegions.size() << " scan regions" << std::endl;
    }
    
    void detectionLoop() {
        std::cout << "[STATIC_DETECTION] Static obstacle detection monitoring loop initiated" << std::endl;
        
        while (detectionActive.load()) {
            try {
                processScanQueue();
                performPeriodicValidation();
                updateMapCoverage();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                
            } catch (const std::exception& e) {
                std::cout << "[STATIC_DETECTION] Exception in detection loop: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        std::cout << "[STATIC_DETECTION] Static obstacle detection monitoring loop terminated" << std::endl;
    }
    
    void processScanQueue() {
        ScanRegion currentRegion;
        bool hasRegion = false;
        
        {
            std::lock_guard<std::mutex> lock(scanQueueMutex);
            if (!scanQueue.empty()) {
                currentRegion = scanQueue.front();
                scanQueue.pop();
                hasRegion = true;
            }
        }
        
        if (hasRegion) {
            std::lock_guard<std::mutex> detectionLock(detectionMutex);
            performRegionScan(currentRegion);
            metrics.scanRegionsProcessed++;
        }
    }
    
    bool performRegionScan(ScanRegion& region) {
        std::cout << "[STATIC_DETECTION] Scanning region (" << region.minX << "," << region.minY 
                  << ") to (" << region.maxX << "," << region.maxY << ")" << std::endl;
        
        bool obstacleDetected = false;
        
        // Simulate obstacle detection within the region
        for (double x = region.minX; x < region.maxX; x += scanResolution) {
            for (double y = region.minY; y < region.maxY; y += scanResolution) {
                if (simulateObstacleDetection(x, y)) {
                    // Check if this is a new obstacle
                    if (!isObstacleAtLocation(x, y)) {
                        createNewObstacle(x, y, "detected");
                        obstacleDetected = true;
                    }
                }
            }
        }
        
        region.lastScanned = std::chrono::steady_clock::now();
        region.requiresRescanning = false;
        
        return obstacleDetected;
    }
    
    bool simulateObstacleDetection(double x, double y) {
        // Simulate sensor-based obstacle detection
        // In a real implementation, this would interface with actual sensors
        
        // Create some deterministic "obstacles" for demonstration
        int gridX = static_cast<int>(x);
        int gridY = static_cast<int>(y);
        
        // Simple pattern: obstacles at certain grid intersections
        if ((gridX % 7 == 0 && gridY % 5 == 0) || 
            (gridX % 11 == 0 && gridY % 3 == 0)) {
            
            // Random confidence simulation
            double confidence = 0.6 + (rand() % 40) / 100.0;
            return confidence > detectionThreshold;
        }
        
        return false;
    }
    
    bool isObstacleAtLocation(double x, double y) {
        for (const auto& [id, obstacle] : detectedObstacles) {
            double distance = std::sqrt((obstacle.x - x) * (obstacle.x - x) + 
                                      (obstacle.y - y) * (obstacle.y - y));
            if (distance < 1.0) { // Within 1 unit
                return true;
            }
        }
        return false;
    }
    
    void createNewObstacle(double x, double y, const std::string& source) {
        StaticObstacle newObstacle;
        newObstacle.obstacleId = nextObstacleId++;
        newObstacle.x = x;
        newObstacle.y = y;
        newObstacle.width = 1.0;
        newObstacle.height = 1.0;
        newObstacle.firstDetection = std::chrono::steady_clock::now();
        newObstacle.lastConfirmation = newObstacle.firstDetection;
        newObstacle.confidence = 0.8;
        newObstacle.obstacleType = "unknown";
        newObstacle.isConfirmed = false;
        newObstacle.isPersistent = false;
        newObstacle.confirmationCount = 1;
        newObstacle.detectionSource = source;
        
        calculateAffectedElements(newObstacle);
        
        detectedObstacles[newObstacle.obstacleId] = newObstacle;
        metrics.totalObstaclesDetected++;
        
        std::cout << "[STATIC_DETECTION] New obstacle created: " << newObstacle.obstacleId 
                  << " at (" << x << "," << y << ")" << std::endl;
    }
    
    void calculateAffectedElements(StaticObstacle& obstacle) {
        obstacle.affectedNodes.clear();
        obstacle.affectedEdges.clear();
        
        // Find nodes within obstacle bounds
        for (int nodeId : environment->getAllNodeIds()) {
            const Node& node = environment->getNode(nodeId);
            
            if (node.getX() >= obstacle.x - obstacle.width/2 &&
                node.getX() <= obstacle.x + obstacle.width/2 &&
                node.getY() >= obstacle.y - obstacle.height/2 &&
                node.getY() <= obstacle.y + obstacle.height/2) {
                obstacle.affectedNodes.push_back(nodeId);
            }
        }
        
        // Find edges that intersect with obstacle
        for (int nodeId : environment->getAllNodeIds()) {
            const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
            
            for (const Edge& edge : edges) {
                if (edgeIntersectsObstacle(edge, obstacle)) {
                    obstacle.affectedEdges.emplace_back(edge.getFromNode(), edge.getToNode());
                }
            }
        }
        
        std::cout << "[STATIC_DETECTION] Obstacle affects " << obstacle.affectedNodes.size() 
                  << " nodes and " << obstacle.affectedEdges.size() << " edges" << std::endl;
    }
    
    bool edgeIntersectsObstacle(const Edge& edge, const StaticObstacle& obstacle) {
        const Node& fromNode = environment->getNode(edge.getFromNode());
        const Node& toNode = environment->getNode(edge.getToNode());
        
        // Simplified intersection check - line segment vs rectangle
        double x1 = fromNode.getX(), y1 = fromNode.getY();
        double x2 = toNode.getX(), y2 = toNode.getY();
        
        double obsLeft = obstacle.x - obstacle.width/2;
        double obsRight = obstacle.x + obstacle.width/2;
        double obsBottom = obstacle.y - obstacle.height/2;
        double obsTop = obstacle.y + obstacle.height/2;
        
        // Check if either endpoint is inside the obstacle
        if ((x1 >= obsLeft && x1 <= obsRight && y1 >= obsBottom && y1 <= obsTop) ||
            (x2 >= obsLeft && x2 <= obsRight && y2 >= obsBottom && y2 <= obsTop)) {
            return true;
        }
        
        // Additional intersection checks would be implemented here
        return false;
    }
    
    bool performStructuralValidation(const StaticObstacle& obstacle) {
        // Validate obstacle structure and properties
        if (obstacle.width <= 0 || obstacle.height <= 0) {
            return false;
        }
        
        if (obstacle.confidence < 0.0 || obstacle.confidence > 1.0) {
            return false;
        }
        
        return true;
    }
    
    bool performSpatialValidation(const StaticObstacle& obstacle) {
        // Validate obstacle position relative to environment
        
        // Check if obstacle position is within reasonable bounds
        bool withinBounds = (obstacle.x >= -100.0 && obstacle.x <= 100.0 &&
                           obstacle.y >= -100.0 && obstacle.y <= 100.0);
        
        return withinBounds;
    }
    
    bool performConsistencyValidation(const StaticObstacle& obstacle) {
        // Check consistency with other obstacles and environment
        
        // Check for overlapping obstacles
        for (const auto& [otherId, otherObstacle] : detectedObstacles) {
            if (otherId == obstacle.obstacleId || !otherObstacle.isConfirmed) {
                continue;
            }
            
            double distance = std::sqrt((obstacle.x - otherObstacle.x) * (obstacle.x - otherObstacle.x) +
                                      (obstacle.y - otherObstacle.y) * (obstacle.y - otherObstacle.y));
            
            if (distance < (obstacle.width + otherObstacle.width) / 2) {
                return false; // Overlapping obstacles
            }
        }
        
        return true;
    }
    
    void confirmObstacle(StaticObstacle& obstacle) {
        obstacle.isConfirmed = true;
        metrics.confirmedObstacles++;
        
        std::cout << "[STATIC_DETECTION] Obstacle confirmed: " << obstacle.obstacleId << std::endl;
        
        if (obstacleDetectedCallback) {
            obstacleDetectedCallback(obstacle);
        }
        
        if (automaticMapUpdate) {
            updateMapWithObstacle(obstacle);
        }
    }
    
    void updateMapWithObstacle(const StaticObstacle& obstacle) {
        std::cout << "[STATIC_DETECTION] Updating map with confirmed obstacle: " 
                  << obstacle.obstacleId << std::endl;
        
        // In a real implementation, this would interface with the map update system
        metrics.mapUpdates++;
    }
    
    void performPeriodicValidation() {
        auto currentTime = std::chrono::steady_clock::now();
        
        for (auto& [obstacleId, obstacle] : detectedObstacles) {
            auto timeSinceConfirmation = std::chrono::duration<double>(
                currentTime - obstacle.lastConfirmation);
            
            if (timeSinceConfirmation.count() > 30.0) { // 30 seconds without confirmation
                validateObstacleDetection(obstacleId);
            }
        }
    }
    
    void updateMapCoverage() {
        // Calculate what percentage of the environment has been scanned recently
        auto currentTime = std::chrono::steady_clock::now();
        size_t recentlyScanned = 0;
        
        for (const auto& region : scanRegions) {
            auto timeSinceScan = std::chrono::duration<double>(currentTime - region.lastScanned);
            if (timeSinceScan.count() < 60.0) { // Within last minute
                recentlyScanned++;
            }
        }
        
        metrics.mapCoverage = scanRegions.empty() ? 0.0 : 
                             static_cast<double>(recentlyScanned) / scanRegions.size();
    }
    
    void updateDetectionMetrics(std::chrono::steady_clock::time_point startTime, size_t obstaclesDetected) {
        auto endTime = std::chrono::steady_clock::now();
        double detectionTime = std::chrono::duration<double>(endTime - startTime).count();
        
        metrics.lastDetection = endTime;
        
        // Update average detection time
        if (metrics.averageDetectionTime == 0.0) {
            metrics.averageDetectionTime = detectionTime;
        } else {
            metrics.averageDetectionTime = (metrics.averageDetectionTime + detectionTime) / 2.0;
        }
        
        // Update average confidence
        if (!detectedObstacles.empty()) {
            double totalConfidence = 0.0;
            for (const auto& [id, obstacle] : detectedObstacles) {
                totalConfidence += obstacle.confidence;
            }
            metrics.averageConfidence = totalConfidence / detectedObstacles.size();
        }
    }
    
    void loadPersistedObstacles() {
        if (!persistentStorage) {
            return;
        }
        
        std::cout << "[STATIC_DETECTION] Loading persisted obstacles from: " << persistenceFilePath << std::endl;
        
        try {
            std::ifstream file(persistenceFilePath, std::ios::binary);
            if (!file.is_open()) {
                std::cout << "[STATIC_DETECTION] No persistence file found, starting fresh" << std::endl;
                return;
            }
            
            // Simple binary format loading (would be more sophisticated in production)
            size_t obstacleCount;
            file.read(reinterpret_cast<char*>(&obstacleCount), sizeof(obstacleCount));
            
            for (size_t i = 0; i < obstacleCount; ++i) {
                StaticObstacle obstacle;
                file.read(reinterpret_cast<char*>(&obstacle.obstacleId), sizeof(obstacle.obstacleId));
                file.read(reinterpret_cast<char*>(&obstacle.x), sizeof(obstacle.x));
                file.read(reinterpret_cast<char*>(&obstacle.y), sizeof(obstacle.y));
                file.read(reinterpret_cast<char*>(&obstacle.width), sizeof(obstacle.width));
                file.read(reinterpret_cast<char*>(&obstacle.height), sizeof(obstacle.height));
                file.read(reinterpret_cast<char*>(&obstacle.confidence), sizeof(obstacle.confidence));
                file.read(reinterpret_cast<char*>(&obstacle.isConfirmed), sizeof(obstacle.isConfirmed));
                file.read(reinterpret_cast<char*>(&obstacle.isPersistent), sizeof(obstacle.isPersistent));
                
                if (obstacle.isPersistent && obstacle.isConfirmed) {
                    obstacle.firstDetection = std::chrono::steady_clock::now();
                    obstacle.lastConfirmation = obstacle.firstDetection;
                    obstacle.confirmationCount = 5; // High confirmation for persisted obstacles
                    obstacle.obstacleType = "persisted";
                    obstacle.detectionSource = "persistence_file";
                    
                    calculateAffectedElements(obstacle);
                    detectedObstacles[obstacle.obstacleId] = obstacle;
                    
                    nextObstacleId = std::max(nextObstacleId, obstacle.obstacleId + 1);
                }
            }
            
            file.close();
            std::cout << "[STATIC_DETECTION] Loaded " << detectedObstacles.size() << " persisted obstacles" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[STATIC_DETECTION] Failed to load persisted obstacles: " << e.what() << std::endl;
        }
    }
    
    void persistObstacles() {
        if (!persistentStorage) {
            return;
        }
        
        std::cout << "[STATIC_DETECTION] Persisting obstacles to: " << persistenceFilePath << std::endl;
        
        try {
            std::ofstream file(persistenceFilePath, std::ios::binary);
            if (!file.is_open()) {
                std::cout << "[STATIC_DETECTION] Failed to open persistence file for writing" << std::endl;
                return;
            }
            
            // Count persistent confirmed obstacles
            size_t persistentCount = 0;
            for (const auto& [id, obstacle] : detectedObstacles) {
                if (obstacle.isPersistent && obstacle.isConfirmed) {
                    persistentCount++;
                }
            }
            
            file.write(reinterpret_cast<const char*>(&persistentCount), sizeof(persistentCount));
            
            for (const auto& [id, obstacle] : detectedObstacles) {
                if (obstacle.isPersistent && obstacle.isConfirmed) {
                    file.write(reinterpret_cast<const char*>(&obstacle.obstacleId), sizeof(obstacle.obstacleId));
                    file.write(reinterpret_cast<const char*>(&obstacle.x), sizeof(obstacle.x));
                    file.write(reinterpret_cast<const char*>(&obstacle.y), sizeof(obstacle.y));
                    file.write(reinterpret_cast<const char*>(&obstacle.width), sizeof(obstacle.width));
                    file.write(reinterpret_cast<const char*>(&obstacle.height), sizeof(obstacle.height));
                    file.write(reinterpret_cast<const char*>(&obstacle.confidence), sizeof(obstacle.confidence));
                    file.write(reinterpret_cast<const char*>(&obstacle.isConfirmed), sizeof(obstacle.isConfirmed));
                    file.write(reinterpret_cast<const char*>(&obstacle.isPersistent), sizeof(obstacle.isPersistent));
                }
            }
            
            file.close();
            std::cout << "[STATIC_DETECTION] Persisted " << persistentCount << " obstacles" << std::endl;
            
        } catch (const std::exception& e) {
            std::cout << "[STATIC_DETECTION] Failed to persist obstacles: " << e.what() << std::endl;
        }
    }
};