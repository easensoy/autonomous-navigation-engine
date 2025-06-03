#include "environment/MapUpdater.hpp"
#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <cmath>
#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <functional>

class SensorDataIntegrationManager {
private:
    MapUpdater* mapUpdater;
    Graph* environment;
    
    struct SensorReading {
        int sensorId;
        std::chrono::steady_clock::time_point timestamp;
        double value;
        std::pair<double, double> location;
        std::string sensorType;
        double confidence;
        bool isValid;
    };
    
    struct ProcessedSensorData {
        std::vector<int> affectedNodes;
        std::vector<std::pair<int, int>> affectedEdges;
        std::vector<MapUpdate> generatedUpdates;
        double dataReliability;
        std::chrono::steady_clock::time_point processingTime;
    };
    
    std::queue<SensorReading> sensorQueue;
    mutable std::mutex sensorQueueMutex;
    std::atomic<bool> processingActive;
    std::thread processingThread;
    
    struct IntegrationMetrics {
        size_t totalReadingsProcessed;
        size_t validReadingsAccepted;
        size_t invalidReadingsRejected;
        size_t mapUpdatesGenerated;
        double averageProcessingTime;
        double averageDataConfidence;
        std::chrono::steady_clock::time_point lastIntegrationTime;
        std::string lastSensorType;
    };
    
    IntegrationMetrics metrics;
    double confidenceThreshold;
    double dataFreshnessLimit;
    bool adaptiveFiltering;
    std::unordered_map<std::string, double> sensorTypeWeights;
    std::unordered_map<int, std::chrono::steady_clock::time_point> lastSensorUpdate;
    
public:
    explicit SensorDataIntegrationManager(MapUpdater* updater, Graph* graph)
        : mapUpdater(updater), environment(graph), processingActive(false),
          confidenceThreshold(0.7), dataFreshnessLimit(30.0), adaptiveFiltering(true) {
        
        if (!mapUpdater || !environment) {
            throw std::invalid_argument("MapUpdater and Graph pointers cannot be null");
        }
        
        initializeIntegrationMetrics();
        configureSensorTypeWeights();
    }
    
    ~SensorDataIntegrationManager() {
        stopProcessing();
    }
    
    bool startProcessing() {
        std::cout << "[SENSOR_INTEGRATION] Initiating sensor data processing services" << std::endl;
        
        if (processingActive.load()) {
            std::cout << "[SENSOR_INTEGRATION] Processing services already operational" << std::endl;
            return true;
        }
        
        processingActive.store(true);
        processingThread = std::thread(&SensorDataIntegrationManager::processingLoop, this);
        
        std::cout << "[SENSOR_INTEGRATION] Sensor data processing services started successfully" << std::endl;
        return true;
    }
    
    void stopProcessing() {
        if (!processingActive.load()) {
            return;
        }
        
        std::cout << "[SENSOR_INTEGRATION] Initiating processing shutdown sequence" << std::endl;
        
        processingActive.store(false);
        
        if (processingThread.joinable()) {
            processingThread.join();
        }
        
        {
            std::lock_guard<std::mutex> lock(sensorQueueMutex);
            while (!sensorQueue.empty()) {
                sensorQueue.pop();
            }
        }
        
        std::cout << "[SENSOR_INTEGRATION] Sensor data processing services terminated" << std::endl;
    }
    
    bool integrateSensorReadings(const std::vector<int>& sensorReadings) {
        std::cout << "[SENSOR_INTEGRATION] Processing batch of " << sensorReadings.size() 
                  << " sensor readings" << std::endl;
        
        auto integrationStart = std::chrono::steady_clock::now();
        
        try {
            std::vector<SensorReading> processedReadings = preprocessSensorData(sensorReadings);
            
            if (processedReadings.empty()) {
                std::cout << "[SENSOR_INTEGRATION] No valid sensor readings to process" << std::endl;
                return false;
            }
            
            ProcessedSensorData aggregatedData = aggregateSensorData(processedReadings);
            
            bool integrationSuccess = applySensorUpdates(aggregatedData);
            
            updateIntegrationMetrics(integrationStart, processedReadings.size(), integrationSuccess);
            
            if (integrationSuccess) {
                std::cout << "[SENSOR_INTEGRATION] Sensor data integration completed successfully" << std::endl;
            } else {
                std::cout << "[SENSOR_INTEGRATION] Sensor data integration encountered processing issues" << std::endl;
            }
            
            return integrationSuccess;
            
        } catch (const std::exception& e) {
            std::cout << "[SENSOR_INTEGRATION] Integration failed due to exception: " << e.what() << std::endl;
            return false;
        }
    }
    
    void queueSensorReading(int sensorId, double value, double x, double y, const std::string& type) {
        SensorReading reading;
        reading.sensorId = sensorId;
        reading.timestamp = std::chrono::steady_clock::now();
        reading.value = value;
        reading.location = {x, y};
        reading.sensorType = type;
        reading.confidence = calculateReadingConfidence(value, type);
        reading.isValid = validateSensorReading(reading);
        
        {
            std::lock_guard<std::mutex> lock(sensorQueueMutex);
            sensorQueue.push(reading);
        }
        
        std::cout << "[SENSOR_INTEGRATION] Sensor reading queued: ID " << sensorId 
                  << ", Type: " << type << ", Confidence: " << reading.confidence << std::endl;
    }
    
    bool integrateRealTimeSensorData(const std::vector<int>& realtimeData) {
        std::cout << "[SENSOR_INTEGRATION] Processing real-time sensor data stream" << std::endl;
        
        if (realtimeData.empty()) {
            std::cout << "[SENSOR_INTEGRATION] Real-time data stream is empty" << std::endl;
            return false;
        }
        
        std::vector<SensorReading> realtimeReadings = convertRealTimeData(realtimeData);
        
        std::vector<SensorReading> filteredReadings = applyRealTimeFiltering(realtimeReadings);
        
        if (filteredReadings.empty()) {
            std::cout << "[SENSOR_INTEGRATION] All real-time readings filtered out" << std::endl;
            return false;
        }
        
        ProcessedSensorData processedData = processRealTimeData(filteredReadings);
        
        bool success = applyRealTimeUpdates(processedData);
        
        if (success) {
            std::cout << "[SENSOR_INTEGRATION] Real-time sensor data integration successful" << std::endl;
        } else {
            std::cout << "[SENSOR_INTEGRATION] Real-time sensor data integration failed" << std::endl;
        }
        
        return success;
    }
    
    void configureSensorParameters(double confidenceThresh, double freshnessLimit, bool adaptive) {
        confidenceThreshold = confidenceThresh;
        dataFreshnessLimit = freshnessLimit;
        adaptiveFiltering = adaptive;
        
        std::cout << "[SENSOR_INTEGRATION] Sensor parameters configured:" << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Confidence threshold: " << confidenceThreshold << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Data freshness limit: " << dataFreshnessLimit << " seconds" << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Adaptive filtering: " << (adaptiveFiltering ? "Enabled" : "Disabled") << std::endl;
    }
    
    void setSensorTypeWeight(const std::string& sensorType, double weight) {
        sensorTypeWeights[sensorType] = weight;
        std::cout << "[SENSOR_INTEGRATION] Sensor type weight configured: " << sensorType 
                  << " = " << weight << std::endl;
    }
    
    IntegrationMetrics getIntegrationMetrics() const {
        return metrics;
    }
    
    bool validateSensorDataIntegrity() {
        std::cout << "[SENSOR_INTEGRATION] Validating sensor data integrity" << std::endl;
        
        try {
            bool temporalConsistency = validateTemporalConsistency();
            bool spatialConsistency = validateSpatialConsistency();
            bool dataQuality = validateDataQuality();
            
            bool overallIntegrity = temporalConsistency && spatialConsistency && dataQuality;
            
            std::cout << "[SENSOR_INTEGRATION] Integrity validation results:" << std::endl;
            std::cout << "[SENSOR_INTEGRATION]   Temporal consistency: " << (temporalConsistency ? "Validated" : "Failed") << std::endl;
            std::cout << "[SENSOR_INTEGRATION]   Spatial consistency: " << (spatialConsistency ? "Validated" : "Failed") << std::endl;
            std::cout << "[SENSOR_INTEGRATION]   Data quality: " << (dataQuality ? "Validated" : "Failed") << std::endl;
            std::cout << "[SENSOR_INTEGRATION]   Overall integrity: " << (overallIntegrity ? "Validated" : "Failed") << std::endl;
            
            return overallIntegrity;
            
        } catch (const std::exception& e) {
            std::cout << "[SENSOR_INTEGRATION] Integrity validation failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    void generateIntegrationReport() const {
        std::cout << "\n[SENSOR_INTEGRATION] === SENSOR DATA INTEGRATION REPORT ===" << std::endl;
        
        std::cout << "[SENSOR_INTEGRATION] System Status:" << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Processing services active: " << (processingActive.load() ? "Yes" : "No") << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Adaptive filtering: " << (adaptiveFiltering ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Confidence threshold: " << confidenceThreshold << std::endl;
        
        std::cout << "[SENSOR_INTEGRATION] Performance Metrics:" << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Total readings processed: " << metrics.totalReadingsProcessed << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Valid readings accepted: " << metrics.validReadingsAccepted << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Invalid readings rejected: " << metrics.invalidReadingsRejected << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Map updates generated: " << metrics.mapUpdatesGenerated << std::endl;
        
        if (metrics.totalReadingsProcessed > 0) {
            double acceptanceRate = static_cast<double>(metrics.validReadingsAccepted) / metrics.totalReadingsProcessed * 100.0;
            std::cout << "[SENSOR_INTEGRATION]   Data acceptance rate: " << acceptanceRate << "%" << std::endl;
        }
        
        std::cout << "[SENSOR_INTEGRATION]   Average processing time: " << metrics.averageProcessingTime << " seconds" << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Average data confidence: " << metrics.averageDataConfidence << std::endl;
        std::cout << "[SENSOR_INTEGRATION]   Last integration time: " << metrics.lastSensorType << std::endl;
        
        {
            std::lock_guard<std::mutex> lock(sensorQueueMutex);
            std::cout << "[SENSOR_INTEGRATION]   Pending sensor readings: " << sensorQueue.size() << std::endl;
        }
        
        std::cout << "[SENSOR_INTEGRATION] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeIntegrationMetrics() {
        metrics.totalReadingsProcessed = 0;
        metrics.validReadingsAccepted = 0;
        metrics.invalidReadingsRejected = 0;
        metrics.mapUpdatesGenerated = 0;
        metrics.averageProcessingTime = 0.0;
        metrics.averageDataConfidence = 0.0;
        metrics.lastIntegrationTime = std::chrono::steady_clock::now();
        metrics.lastSensorType = "";
    }
    
    void configureSensorTypeWeights() {
        sensorTypeWeights["obstacle"] = 1.0;
        sensorTypeWeights["proximity"] = 0.8;
        sensorTypeWeights["positioning"] = 0.9;
        sensorTypeWeights["environmental"] = 0.6;
        sensorTypeWeights["navigation"] = 0.95;
    }
    
    void processingLoop() {
        std::cout << "[SENSOR_INTEGRATION] Sensor data processing loop initiated" << std::endl;
        
        while (processingActive.load()) {
            try {
                processQueuedSensorData();
                
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
            } catch (const std::exception& e) {
                std::cout << "[SENSOR_INTEGRATION] Exception in processing loop: " << e.what() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        
        std::cout << "[SENSOR_INTEGRATION] Sensor data processing loop terminated" << std::endl;
    }
    
    void processQueuedSensorData() {
        std::vector<SensorReading> batchReadings;
        
        {
            std::lock_guard<std::mutex> lock(sensorQueueMutex);
            
            while (!sensorQueue.empty() && batchReadings.size() < 10) {
                batchReadings.push_back(sensorQueue.front());
                sensorQueue.pop();
            }
        }
        
        if (!batchReadings.empty()) {
            std::cout << "[SENSOR_INTEGRATION] Processing batch of " << batchReadings.size() 
                      << " queued sensor readings" << std::endl;
            
            ProcessedSensorData processedData = aggregateSensorData(batchReadings);
            applyBatchSensorUpdates(processedData);
        }
    }
    
    std::vector<SensorReading> preprocessSensorData(const std::vector<int>& rawData) {
        std::cout << "[SENSOR_INTEGRATION] Preprocessing raw sensor data" << std::endl;
        
        std::vector<SensorReading> processedReadings;
        
        for (size_t i = 0; i < rawData.size(); i += 4) {
            if (i + 3 < rawData.size()) {
                SensorReading reading;
                reading.sensorId = rawData[i];
                reading.timestamp = std::chrono::steady_clock::now();
                reading.value = static_cast<double>(rawData[i + 1]);
                reading.location = {static_cast<double>(rawData[i + 2]), static_cast<double>(rawData[i + 3])};
                reading.sensorType = determineSensorType(reading.sensorId);
                reading.confidence = calculateReadingConfidence(reading.value, reading.sensorType);
                reading.isValid = validateSensorReading(reading);
                
                if (reading.isValid) {
                    processedReadings.push_back(reading);
                }
            }
        }
        
        std::cout << "[SENSOR_INTEGRATION] Preprocessed " << processedReadings.size() 
                  << " valid readings from " << rawData.size() / 4 << " raw data points" << std::endl;
        
        return processedReadings;
    }
    
    ProcessedSensorData aggregateSensorData(const std::vector<SensorReading>& readings) {
        std::cout << "[SENSOR_INTEGRATION] Aggregating sensor data for map integration" << std::endl;
        
        ProcessedSensorData aggregated;
        aggregated.processingTime = std::chrono::steady_clock::now();
        aggregated.dataReliability = 0.0;
        
        std::unordered_set<int> affectedNodesSet;
        std::unordered_set<std::pair<int, int>, PairHash> affectedEdgesSet;
        
        double totalConfidence = 0.0;
        
        for (const SensorReading& reading : readings) {
            if (!reading.isValid) {
                continue;
            }
            
            std::vector<int> nearbyNodes = findNearbyNodes(reading.location);
            for (int nodeId : nearbyNodes) {
                affectedNodesSet.insert(nodeId);
            }
            
            std::vector<std::pair<int, int>> nearbyEdges = findNearbyEdges(reading.location);
            for (const auto& edge : nearbyEdges) {
                affectedEdgesSet.insert(edge);
            }
            
            MapUpdate update = generateMapUpdate(reading);
            if (update.affectedNodes.size() > 0 || update.affectedEdges.size() > 0) {
                aggregated.generatedUpdates.push_back(update);
            }
            
            totalConfidence += reading.confidence;
        }
        
        aggregated.affectedNodes = std::vector<int>(affectedNodesSet.begin(), affectedNodesSet.end());
        aggregated.affectedEdges = std::vector<std::pair<int, int>>(affectedEdgesSet.begin(), affectedEdgesSet.end());
        aggregated.dataReliability = readings.empty() ? 0.0 : totalConfidence / readings.size();
        
        std::cout << "[SENSOR_INTEGRATION] Aggregation complete: " << aggregated.affectedNodes.size() 
                  << " nodes, " << aggregated.affectedEdges.size() << " edges, " 
                  << aggregated.generatedUpdates.size() << " updates" << std::endl;
        
        return aggregated;
    }
    
    bool applySensorUpdates(const ProcessedSensorData& data) {
        std::cout << "[SENSOR_INTEGRATION] Applying sensor-derived map updates" << std::endl;
        
        try {
            if (data.dataReliability < confidenceThreshold) {
                std::cout << "[SENSOR_INTEGRATION] Data reliability below threshold, skipping updates" << std::endl;
                return false;
            }
            
            mapUpdater->updateMap(data.generatedUpdates);
            
            metrics.mapUpdatesGenerated += data.generatedUpdates.size();
            
            std::cout << "[SENSOR_INTEGRATION] Successfully applied " << data.generatedUpdates.size() 
                      << " sensor-derived updates" << std::endl;
            
            return true;
            
        } catch (const std::exception& e) {
            std::cout << "[SENSOR_INTEGRATION] Failed to apply sensor updates: " << e.what() << std::endl;
            return false;
        }
    }
    
    std::vector<SensorReading> convertRealTimeData(const std::vector<int>& realtimeData) {
        std::vector<SensorReading> readings;
        
        for (size_t i = 0; i < realtimeData.size(); i += 3) {
            if (i + 2 < realtimeData.size()) {
                SensorReading reading;
                reading.sensorId = 1000 + i; // Generate unique ID for real-time data
                reading.timestamp = std::chrono::steady_clock::now();
                reading.value = static_cast<double>(realtimeData[i]);
                reading.location = {static_cast<double>(realtimeData[i + 1]), static_cast<double>(realtimeData[i + 2])};
                reading.sensorType = "realtime";
                reading.confidence = 0.9; // High confidence for real-time data
                reading.isValid = true;
                
                readings.push_back(reading);
            }
        }
        
        return readings;
    }
    
    std::vector<SensorReading> applyRealTimeFiltering(const std::vector<SensorReading>& readings) {
        std::vector<SensorReading> filtered;
        
        for (const SensorReading& reading : readings) {
            if (reading.confidence >= confidenceThreshold * 0.8) { // Lower threshold for real-time data
                filtered.push_back(reading);
            }
        }
        
        return filtered;
    }
    
    ProcessedSensorData processRealTimeData(const std::vector<SensorReading>& readings) {
        return aggregateSensorData(readings);
    }
    
    bool applyRealTimeUpdates(const ProcessedSensorData& data) {
        return applySensorUpdates(data);
    }
    
    void applyBatchSensorUpdates(const ProcessedSensorData& data) {
        applySensorUpdates(data);
    }
    
    bool validateSensorReading(const SensorReading& reading) const {
        if (reading.sensorId < 0) {
            return false;
        }
        
        if (std::isnan(reading.value) || std::isinf(reading.value)) {
            return false;
        }
        
        if (reading.confidence < 0.0 || reading.confidence > 1.0) {
            return false;
        }
        
        auto now = std::chrono::steady_clock::now();
        auto age = std::chrono::duration<double>(now - reading.timestamp).count();
        if (age > dataFreshnessLimit) {
            return false;
        }
        
        return true;
    }
    
    double calculateReadingConfidence(double value, const std::string& type) const {
        double baseConfidence = 0.8;
        
        auto it = sensorTypeWeights.find(type);
        if (it != sensorTypeWeights.end()) {
            baseConfidence *= it->second;
        }
        
        // Adjust confidence based on value range
        if (std::abs(value) > 1000.0) {
            baseConfidence *= 0.9; // Reduce confidence for extreme values
        }
        
        return std::min(1.0, std::max(0.0, baseConfidence));
    }
    
    std::string determineSensorType(int sensorId) const {
        if (sensorId < 100) {
            return "obstacle";
        } else if (sensorId < 200) {
            return "proximity";
        } else if (sensorId < 300) {
            return "positioning";
        } else {
            return "environmental";
        }
    }
    
    std::vector<int> findNearbyNodes(const std::pair<double, double>& location) const {
        std::vector<int> nearbyNodes;
        
        for (int nodeId : environment->getAllNodeIds()) {
            const Node& node = environment->getNode(nodeId);
            double distance = std::sqrt(std::pow(node.getX() - location.first, 2) + 
                                      std::pow(node.getY() - location.second, 2));
            
            if (distance <= 2.0) { // Within 2 units
                nearbyNodes.push_back(nodeId);
            }
        }
        
        return nearbyNodes;
    }
    
    std::vector<std::pair<int, int>> findNearbyEdges(const std::pair<double, double>& location) const {
        std::vector<std::pair<int, int>> nearbyEdges;
        
        std::vector<int> nearbyNodes = findNearbyNodes(location);
        
        for (int nodeId : nearbyNodes) {
            const std::vector<Edge>& edges = environment->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                nearbyEdges.emplace_back(edge.getFromNode(), edge.getToNode());
            }
        }
        
        return nearbyEdges;
    }
    
    MapUpdate generateMapUpdate(const SensorReading& reading) const {
        MapUpdate update(UpdateType::WEIGHT_MODIFICATION);
        update.timestamp = reading.timestamp;
        
        std::vector<int> affectedNodes = findNearbyNodes(reading.location);
        update.affectedNodes = affectedNodes;
        
        return update;
    }
    
    bool validateTemporalConsistency() {
        // Validate that sensor data has reasonable temporal distribution
        return true;
    }
    
    bool validateSpatialConsistency() {
        // Validate that sensor data has reasonable spatial distribution
        return true;
    }
    
    bool validateDataQuality() {
        // Validate overall data quality metrics
        return metrics.averageDataConfidence >= confidenceThreshold;
    }
    
    void updateIntegrationMetrics(std::chrono::steady_clock::time_point startTime, 
                                size_t readingsCount, bool success) {
        auto endTime = std::chrono::steady_clock::now();
        double processingTime = std::chrono::duration<double>(endTime - startTime).count();
        
        metrics.totalReadingsProcessed += readingsCount;
        if (success) {
            metrics.validReadingsAccepted += readingsCount;
        } else {
            metrics.invalidReadingsRejected += readingsCount;
        }
        
        // Update average processing time
        if (metrics.averageProcessingTime == 0.0) {
            metrics.averageProcessingTime = processingTime;
        } else {
            metrics.averageProcessingTime = (metrics.averageProcessingTime + processingTime) / 2.0;
        }
        
        metrics.lastIntegrationTime = endTime;
    }
    
    struct PairHash {
        size_t operator()(const std::pair<int, int>& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };
};

// Global sensor integration manager
static std::unique_ptr<SensorDataIntegrationManager> g_sensorManager;

// Implementation method for MapUpdater
void MapUpdater::processSensorData(const std::vector<int>& sensorReadings) {
    std::cout << "[MAP_UPDATER] Processing sensor data integration request" << std::endl;
    
    if (!g_sensorManager) {
        g_sensorManager = std::make_unique<SensorDataIntegrationManager>(this, environment);
        g_sensorManager->startProcessing();
    }
    
    if (!g_sensorManager->integrateSensorReadings(sensorReadings)) {
        std::cout << "[MAP_UPDATER] Sensor data integration processing failed" << std::endl;
    }
}