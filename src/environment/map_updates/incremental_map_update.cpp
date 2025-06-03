#include "environment/MapUpdater.hpp"
#include <iostream>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include <mutex>
#include <numeric>

class IncrementalMapUpdateManager {
private:
    MapUpdater* mapUpdater;
    Graph* environment;
    std::queue<MapUpdate> updateQueue;
    mutable std::mutex updateQueueMutex;
    std::atomic<bool> processingActive;
    std::thread processingThread;
    
    struct UpdateBatch {
        std::vector<MapUpdate> updates;
        std::chrono::steady_clock::time_point timestamp;
        double batchPriority;
        bool requiresValidation;
    };
    
    std::vector<UpdateBatch> batchHistory;
    size_t maxBatchHistorySize;
    double batchProcessingInterval;
    bool batchModeEnabled;
    
    struct IncrementalMetrics {
        size_t totalUpdatesProcessed;
        size_t batchesProcessed;
        double averageUpdateTime;
        double averageBatchSize;
        std::chrono::steady_clock::time_point lastUpdateTime;
        std::vector<double> updateTimes;
    };
    
    IncrementalMetrics metrics;
    
public:
    explicit IncrementalMapUpdateManager(MapUpdater* updater, Graph* graph) 
        : mapUpdater(updater), environment(graph), processingActive(false),
          maxBatchHistorySize(50), batchProcessingInterval(0.1), batchModeEnabled(true) {
        
        initializeMetrics();
    }
    
    ~IncrementalMapUpdateManager() {
        stopIncrementalProcessing();
    }
    
    void startIncrementalProcessing() {
        std::cout << "[INCREMENTAL_UPDATE] Starting incremental map update processing" << std::endl;
        
        if (processingActive.load()) {
            std::cout << "[INCREMENTAL_UPDATE] Processing already active" << std::endl;
            return;
        }
        
        processingActive.store(true);
        processingThread = std::thread(&IncrementalMapUpdateManager::processUpdateLoop, this);
        
        std::cout << "[INCREMENTAL_UPDATE] Incremental processing started successfully" << std::endl;
    }
    
    void stopIncrementalProcessing() {
        if (!processingActive.load()) {
            return;
        }
        
        std::cout << "[INCREMENTAL_UPDATE] Stopping incremental map update processing" << std::endl;
        
        processingActive.store(false);
        
        if (processingThread.joinable()) {
            processingThread.join();
        }
        
        // Process any remaining updates
        processRemainingUpdates();
        
        std::cout << "[INCREMENTAL_UPDATE] Incremental processing stopped" << std::endl;
    }
    
    void queueNodeAddition(int nodeId, const std::string& name, double x, double y) {
        MapUpdate update(UpdateType::NODE_ADDITION);
        update.affectedNodes.push_back(nodeId);
        update.timestamp = std::chrono::steady_clock::now();
        
        // Store additional data for node addition
        // In a real implementation, this would be stored in a more structured way
        std::cout << "[INCREMENTAL_UPDATE] Queueing node addition: " << nodeId 
                  << " (" << name << ") at (" << x << "," << y << ")" << std::endl;
        
        queueUpdate(update);
    }
    
    void queueNodeRemoval(int nodeId) {
        MapUpdate update(UpdateType::NODE_REMOVAL);
        update.affectedNodes.push_back(nodeId);
        update.timestamp = std::chrono::steady_clock::now();
        
        std::cout << "[INCREMENTAL_UPDATE] Queueing node removal: " << nodeId << std::endl;
        
        queueUpdate(update);
    }
    
    void queueEdgeAddition(int fromId, int toId, double weight) {
        MapUpdate update(UpdateType::EDGE_ADDITION);
        update.affectedEdges.emplace_back(fromId, toId);
        update.timestamp = std::chrono::steady_clock::now();
        
        std::cout << "[INCREMENTAL_UPDATE] Queueing edge addition: " << fromId 
                  << " -> " << toId << " (weight: " << weight << ")" << std::endl;
        
        queueUpdate(update);
    }
    
    void queueEdgeRemoval(int fromId, int toId) {
        MapUpdate update(UpdateType::EDGE_REMOVAL);
        update.affectedEdges.emplace_back(fromId, toId);
        update.timestamp = std::chrono::steady_clock::now();
        
        std::cout << "[INCREMENTAL_UPDATE] Queueing edge removal: " << fromId << " -> " << toId << std::endl;
        
        queueUpdate(update);
    }
    
    void queueWeightModification(int fromId, int toId, double newWeight) {
        MapUpdate update(UpdateType::WEIGHT_MODIFICATION);
        update.affectedEdges.emplace_back(fromId, toId);
        update.timestamp = std::chrono::steady_clock::now();
        
        std::cout << "[INCREMENTAL_UPDATE] Queueing weight modification: " << fromId 
                  << " -> " << toId << " (new weight: " << newWeight << ")" << std::endl;
        
        queueUpdate(update);
    }
    
    void queueBatchUpdate(const std::vector<MapUpdate>& updates) {
        std::cout << "[INCREMENTAL_UPDATE] Queueing batch update with " << updates.size() << " updates" << std::endl;
        
        for (const MapUpdate& update : updates) {
            queueUpdate(update);
        }
    }
    
    bool processImmediateUpdate(const MapUpdate& update) {
        std::cout << "[INCREMENTAL_UPDATE] Processing immediate update of type: " 
                  << static_cast<int>(update.type) << std::endl;
        
        auto updateStart = std::chrono::steady_clock::now();
        
        bool success = applyMapUpdate(update);
        
        if (success) {
            updateMetrics(updateStart);
            std::cout << "[INCREMENTAL_UPDATE] Immediate update processed successfully" << std::endl;
        } else {
            std::cout << "[INCREMENTAL_UPDATE] Immediate update processing failed" << std::endl;
        }
        
        return success;
    }
    
    void setBatchMode(bool enabled) {
        batchModeEnabled = enabled;
        std::cout << "[INCREMENTAL_UPDATE] Batch mode " << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    void setBatchProcessingInterval(double intervalSeconds) {
        batchProcessingInterval = intervalSeconds;
        std::cout << "[INCREMENTAL_UPDATE] Batch processing interval set to " 
                  << intervalSeconds << " seconds" << std::endl;
    }
    
    size_t getQueueSize() const {
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        return updateQueue.size();
    }
    
    bool isProcessingActive() const {
        return processingActive.load();
    }
    
    void forceProcessPendingUpdates() {
        std::cout << "[INCREMENTAL_UPDATE] Force processing all pending updates" << std::endl;
        
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        
        while (!updateQueue.empty()) {
            MapUpdate update = updateQueue.front();
            updateQueue.pop();
            
            processImmediateUpdate(update);
        }
        
        std::cout << "[INCREMENTAL_UPDATE] All pending updates processed" << std::endl;
    }
    
    void optimizeUpdateSequence() {
        std::cout << "[INCREMENTAL_UPDATE] Optimizing update sequence for efficiency" << std::endl;
        
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        
        if (updateQueue.empty()) {
            return;
        }
        
        // Convert queue to vector for optimization
        std::vector<MapUpdate> updates;
        while (!updateQueue.empty()) {
            updates.push_back(updateQueue.front());
            updateQueue.pop();
        }
        
        // Optimize update order
        optimizeUpdateOrder(updates);
        
        // Put optimized updates back in queue
        for (const MapUpdate& update : updates) {
            updateQueue.push(update);
        }
        
        std::cout << "[INCREMENTAL_UPDATE] Update sequence optimized" << std::endl;
    }
    
    void generateIncrementalReport() const {
        std::cout << "\n[INCREMENTAL_UPDATE] === INCREMENTAL UPDATE REPORT ===" << std::endl;
        
        std::cout << "[INCREMENTAL_UPDATE] Processing Status:" << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Active: " << (processingActive.load() ? "Yes" : "No") << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Queue size: " << getQueueSize() << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Batch mode: " << (batchModeEnabled ? "Enabled" : "Disabled") << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Batch interval: " << batchProcessingInterval << " seconds" << std::endl;
        
        std::cout << "[INCREMENTAL_UPDATE] Performance Metrics:" << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Total updates processed: " << metrics.totalUpdatesProcessed << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Batches processed: " << metrics.batchesProcessed << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Average update time: " << metrics.averageUpdateTime << " seconds" << std::endl;
        std::cout << "[INCREMENTAL_UPDATE]   Average batch size: " << metrics.averageBatchSize << std::endl;
        
        if (!metrics.updateTimes.empty()) {
            auto minMax = std::minmax_element(metrics.updateTimes.begin(), metrics.updateTimes.end());
            std::cout << "[INCREMENTAL_UPDATE]   Fastest update: " << *minMax.first << " seconds" << std::endl;
            std::cout << "[INCREMENTAL_UPDATE]   Slowest update: " << *minMax.second << " seconds" << std::endl;
        }
        
        std::cout << "[INCREMENTAL_UPDATE]   Batch history size: " << batchHistory.size() << std::endl;
        
        std::cout << "[INCREMENTAL_UPDATE] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeMetrics() {
        metrics.totalUpdatesProcessed = 0;
        metrics.batchesProcessed = 0;
        metrics.averageUpdateTime = 0.0;
        metrics.averageBatchSize = 0.0;
        metrics.lastUpdateTime = std::chrono::steady_clock::now();
        metrics.updateTimes.clear();
    }
    
    void queueUpdate(const MapUpdate& update) {
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        updateQueue.push(update);
    }
    
    void processUpdateLoop() {
        std::cout << "[INCREMENTAL_UPDATE] Update processing loop started" << std::endl;
        
        while (processingActive.load()) {
            try {
                if (batchModeEnabled) {
                    processBatchUpdates();
                } else {
                    processIndividualUpdates();
                }
                
                // Sleep based on processing interval
                auto sleepDuration = std::chrono::milliseconds(
                    static_cast<int>(batchProcessingInterval * 1000));
                std::this_thread::sleep_for(sleepDuration);
                
            } catch (const std::exception& e) {
                std::cout << "[INCREMENTAL_UPDATE] Exception in processing loop: " << e.what() << std::endl;
            }
        }
        
        std::cout << "[INCREMENTAL_UPDATE] Update processing loop stopped" << std::endl;
    }
    
    void processBatchUpdates() {
        std::vector<MapUpdate> batch;
        
        // Collect updates for batch processing
        {
            std::lock_guard<std::mutex> lock(updateQueueMutex);
            
            // Collect updates up to a reasonable batch size
            size_t maxBatchSize = 10;
            while (!updateQueue.empty() && batch.size() < maxBatchSize) {
                batch.push_back(updateQueue.front());
                updateQueue.pop();
            }
        }
        
        if (!batch.empty()) {
            processBatch(batch);
        }
    }
    
    void processIndividualUpdates() {
        MapUpdate update(UpdateType::NODE_ADDITION); // Default initialization
        
        {
            std::lock_guard<std::mutex> lock(updateQueueMutex);
            if (updateQueue.empty()) {
                return;
            }
            
            update = updateQueue.front();
            updateQueue.pop();
        }
        
        processImmediateUpdate(update);
    }
    
    void processBatch(const std::vector<MapUpdate>& batch) {
        std::cout << "[INCREMENTAL_UPDATE] Processing batch of " << batch.size() << " updates" << std::endl;
        
        auto batchStart = std::chrono::steady_clock::now();
        
        // Sort batch by priority and dependencies
        std::vector<MapUpdate> sortedBatch = batch;
        optimizeUpdateOrder(sortedBatch);
        
        // Process updates in optimized order
        size_t successfulUpdates = 0;
        for (const MapUpdate& update : sortedBatch) {
            if (applyMapUpdate(update)) {
                successfulUpdates++;
            }
        }
        
        // Update batch metrics
        updateBatchMetrics(batchStart, batch.size(), successfulUpdates);
        
        // Store batch in history
        storeBatchInHistory(batch);
        
        std::cout << "[INCREMENTAL_UPDATE] Batch processing completed: " 
                  << successfulUpdates << "/" << batch.size() << " successful" << std::endl;
    }
    
    bool applyMapUpdate(const MapUpdate& update) {
        try {
            switch (update.type) {
                case UpdateType::NODE_ADDITION:
                    return processNodeAddition(update);
                case UpdateType::NODE_REMOVAL:
                    return processNodeRemoval(update);
                case UpdateType::EDGE_ADDITION:
                    return processEdgeAddition(update);
                case UpdateType::EDGE_REMOVAL:
                    return processEdgeRemoval(update);
                case UpdateType::WEIGHT_MODIFICATION:
                    return processWeightModification(update);
                case UpdateType::FULL_RECONSTRUCTION:
                    return processFullReconstruction(update);
                default:
                    std::cout << "[INCREMENTAL_UPDATE] Unknown update type: " 
                              << static_cast<int>(update.type) << std::endl;
                    return false;
            }
        } catch (const std::exception& e) {
            std::cout << "[INCREMENTAL_UPDATE] Exception applying update: " << e.what() << std::endl;
            return false;
        }
    }
    
    bool processNodeAddition(const MapUpdate& update) {
        if (update.affectedNodes.empty()) {
            return false;
        }
        
        int nodeId = update.affectedNodes[0];
        
        // In a real implementation, node details would be stored in the update
        std::string nodeName = "Node_" + std::to_string(nodeId);
        double x = nodeId * 1.0; // Simplified positioning
        double y = nodeId * 0.5;
        
        mapUpdater->addNode(nodeId, nodeName, x, y);
        return true;
    }
    
    bool processNodeRemoval(const MapUpdate& update) {
        if (update.affectedNodes.empty()) {
            return false;
        }
        
        int nodeId = update.affectedNodes[0];
        mapUpdater->removeNode(nodeId);
        return true;
    }
    
    bool processEdgeAddition(const MapUpdate& update) {
        if (update.affectedEdges.empty()) {
            return false;
        }
        
        auto edge = update.affectedEdges[0];
        double weight = 1.0; // Default weight, would be stored in update in real implementation
        
        mapUpdater->updateEdgeWeight(edge.first, edge.second, weight);
        return true;
    }
    
    bool processEdgeRemoval(const MapUpdate& update) {
        if (update.affectedEdges.empty()) {
            return false;
        }
        
        auto edge = update.affectedEdges[0];
        
        // Edge removal by setting weight to negative (indicating removal)
        mapUpdater->updateEdgeWeight(edge.first, edge.second, -1.0);
        return true;
    }
    
    bool processWeightModification(const MapUpdate& update) {
        if (update.affectedEdges.empty()) {
            return false;
        }
        
        auto edge = update.affectedEdges[0];
        double newWeight = 1.0; // Would be stored in update in real implementation
        
        mapUpdater->updateEdgeWeight(edge.first, edge.second, newWeight);
        return true;
    }
    
    bool processFullReconstruction(const MapUpdate& update) {
        std::cout << "[INCREMENTAL_UPDATE] Processing full map reconstruction" << std::endl;
        
        // This would trigger a complete map rebuild
        mapUpdater->optimizeMapStructure();
        return true;
    }
    
    void optimizeUpdateOrder(std::vector<MapUpdate>& updates) {
        // Sort updates to optimize processing order
        std::sort(updates.begin(), updates.end(), [](const MapUpdate& a, const MapUpdate& b) {
            // Process node additions before edge operations
            if (a.type == UpdateType::NODE_ADDITION && b.type != UpdateType::NODE_ADDITION) {
                return true;
            }
            if (a.type != UpdateType::NODE_ADDITION && b.type == UpdateType::NODE_ADDITION) {
                return false;
            }
            
            // Process node removals last
            if (a.type == UpdateType::NODE_REMOVAL && b.type != UpdateType::NODE_REMOVAL) {
                return false;
            }
            if (a.type != UpdateType::NODE_REMOVAL && b.type == UpdateType::NODE_REMOVAL) {
                return true;
            }
            
            // Sort by timestamp for same types
            return a.timestamp < b.timestamp;
        });
    }
    
    void updateMetrics(std::chrono::steady_clock::time_point updateStart) {
        auto updateEnd = std::chrono::steady_clock::now();
        double updateTime = std::chrono::duration<double>(updateEnd - updateStart).count();
        
        metrics.updateTimes.push_back(updateTime);
        metrics.totalUpdatesProcessed++;
        metrics.lastUpdateTime = updateEnd;
        
        // Update average update time
        double totalTime = std::accumulate(metrics.updateTimes.begin(), metrics.updateTimes.end(), 0.0);
        metrics.averageUpdateTime = totalTime / metrics.updateTimes.size();
        
        // Limit stored update times to avoid memory growth
        if (metrics.updateTimes.size() > 100) {
            metrics.updateTimes.erase(metrics.updateTimes.begin());
        }
    }
    
    void updateBatchMetrics(std::chrono::steady_clock::time_point batchStart, size_t batchSize, size_t successfulUpdates) {
        metrics.batchesProcessed++;
        metrics.totalUpdatesProcessed += successfulUpdates;
        
        // Update average batch size
        metrics.averageBatchSize = static_cast<double>(metrics.totalUpdatesProcessed) / metrics.batchesProcessed;
        
        auto batchEnd = std::chrono::steady_clock::now();
        double batchTime = std::chrono::duration<double>(batchEnd - batchStart).count();
        
        if (batchSize > 0) {
            double avgUpdateTimeInBatch = batchTime / batchSize;
            metrics.updateTimes.push_back(avgUpdateTimeInBatch);
            
            // Update average update time
            double totalTime = std::accumulate(metrics.updateTimes.begin(), metrics.updateTimes.end(), 0.0);
            metrics.averageUpdateTime = totalTime / metrics.updateTimes.size();
        }
    }
    
    void storeBatchInHistory(const std::vector<MapUpdate>& batch) {
        UpdateBatch batchRecord;
        batchRecord.updates = batch;
        batchRecord.timestamp = std::chrono::steady_clock::now();
        batchRecord.batchPriority = calculateBatchPriority(batch);
        batchRecord.requiresValidation = containsCriticalUpdates(batch);
        
        batchHistory.push_back(batchRecord);
        
        // Limit history size
        if (batchHistory.size() > maxBatchHistorySize) {
            batchHistory.erase(batchHistory.begin());
        }
    }
    
    double calculateBatchPriority(const std::vector<MapUpdate>& batch) {
        double priority = 0.0;
        
        for (const MapUpdate& update : batch) {
            switch (update.type) {
                case UpdateType::NODE_ADDITION:
                case UpdateType::NODE_REMOVAL:
                    priority += 2.0; // Higher priority for structural changes
                    break;
                case UpdateType::EDGE_ADDITION:
                case UpdateType::EDGE_REMOVAL:
                    priority += 1.5;
                    break;
                case UpdateType::WEIGHT_MODIFICATION:
                    priority += 1.0;
                    break;
                case UpdateType::FULL_RECONSTRUCTION:
                    priority += 5.0; // Highest priority
                    break;
            }
        }
        
        return priority / batch.size(); // Average priority per update
    }
    
    bool containsCriticalUpdates(const std::vector<MapUpdate>& batch) {
        for (const MapUpdate& update : batch) {
            if (update.type == UpdateType::NODE_REMOVAL || update.type == UpdateType::FULL_RECONSTRUCTION) {
                return true;
            }
        }
        return false;
    }
    
    void processRemainingUpdates() {
        std::cout << "[INCREMENTAL_UPDATE] Processing remaining queued updates" << std::endl;
        
        std::lock_guard<std::mutex> lock(updateQueueMutex);
        
        while (!updateQueue.empty()) {
            MapUpdate update = updateQueue.front();
            updateQueue.pop();
            
            applyMapUpdate(update);
        }
    }
};