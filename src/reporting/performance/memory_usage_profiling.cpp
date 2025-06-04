#include "reporting/PerformanceProfiler.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <vector>
#include <unordered_map>
#include <queue>
#include <numeric>
#include <cmath>
#include <thread>
#include <mutex>
#include <memory>
#include <fstream>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#elif __APPLE__
#include <mach/mach.h>
#include <mach/task.h>
#elif __linux__
#include <sys/resource.h>
#include <fstream>
#include <string>
#endif

class MemoryUsageProfiler {
private:
    struct MemorySnapshot {
        std::chrono::steady_clock::time_point timestamp;
        size_t totalMemoryUsed;
        size_t heapMemoryUsed;
        size_t stackMemoryUsed;
        size_t peakMemoryUsed;
        size_t availableMemory;
        std::string operationContext;
        std::unordered_map<std::string, size_t> categoryBreakdown;
        
        MemorySnapshot() : totalMemoryUsed(0), heapMemoryUsed(0), stackMemoryUsed(0),
                          peakMemoryUsed(0), availableMemory(0) {}
    };
    
    struct MemoryOperation {
        std::string operationName;
        MemorySnapshot startSnapshot;
        MemorySnapshot endSnapshot;
        size_t memoryDelta;
        size_t peakMemoryDuringOperation;
        std::vector<MemorySnapshot> intermediateSnapshots;
        std::chrono::duration<double> operationDuration;
        bool hasMemoryLeak;
        
        MemoryOperation(const std::string& name) 
            : operationName(name), memoryDelta(0), peakMemoryDuringOperation(0), 
              hasMemoryLeak(false) {}
    };
    
    struct MemoryAllocation {
        void* address;
        size_t size;
        std::string category;
        std::chrono::steady_clock::time_point allocationTime;
        std::string stackTrace;
        bool isActive;
        
        MemoryAllocation(void* addr, size_t sz, const std::string& cat) 
            : address(addr), size(sz), category(cat), 
              allocationTime(std::chrono::steady_clock::now()), isActive(true) {}
    };
    
    struct MemoryAnalysis {
        double memoryEfficiency;
        size_t wastedMemory;
        size_t fragmentationLevel;
        std::vector<std::string> optimizationRecommendations;
        double averageAllocationSize;
        size_t allocationCount;
        size_t deallocationCount;
        size_t activeAllocations;
        std::chrono::duration<double> analysisTime;
        
        MemoryAnalysis() : memoryEfficiency(100.0), wastedMemory(0), 
                          fragmentationLevel(0), averageAllocationSize(0.0),
                          allocationCount(0), deallocationCount(0), activeAllocations(0) {}
    };
    
    struct MemoryTrend {
        std::vector<size_t> memoryUsageHistory;
        std::vector<std::chrono::steady_clock::time_point> timestamps;
        double growthRate;
        std::string trendDirection;
        size_t maxMemoryObserved;
        size_t minMemoryObserved;
        
        MemoryTrend() : growthRate(0.0), trendDirection("STABLE"), 
                       maxMemoryObserved(0), minMemoryObserved(SIZE_MAX) {}
    };
    
    // Data storage
    std::unordered_map<std::string, MemoryOperation> activeOperations;
    std::vector<MemoryOperation> completedOperations;
    std::unordered_map<void*, MemoryAllocation> allocations;
    std::vector<MemorySnapshot> memoryHistory;
    MemoryTrend globalTrend;
    
    // Configuration
    bool enableDetailedTracking;
    bool enableLeakDetection;
    bool enableRealTimeMonitoring;
    size_t maxHistorySize;
    size_t snapshotInterval;
    double memoryThreshold;
    bool enableStackTraces;
    
    // Thread safety
    mutable std::mutex profilingMutex;
    
    // Background monitoring
    std::atomic<bool> monitoringActive;
    std::thread monitoringThread;
    std::chrono::duration<double> monitoringInterval;
    
    // Statistics
    size_t totalAllocations;
    size_t totalDeallocations;
    size_t currentAllocations;
    size_t peakMemoryUsage;
    size_t totalMemoryAllocated;
    size_t totalMemoryDeallocated;
    
public:
    MemoryUsageProfiler() 
        : enableDetailedTracking(true), enableLeakDetection(true), 
          enableRealTimeMonitoring(false), maxHistorySize(10000), 
          snapshotInterval(100), memoryThreshold(100.0 * 1024 * 1024), // 100MB
          enableStackTraces(false), monitoringActive(false),
          monitoringInterval(std::chrono::seconds(1)), totalAllocations(0),
          totalDeallocations(0), currentAllocations(0), peakMemoryUsage(0),
          totalMemoryAllocated(0), totalMemoryDeallocated(0) {
        
        std::cout << "[MEMORY_PROF] Memory usage profiler initialized" << std::endl;
    }
    
    ~MemoryUsageProfiler() {
        stopRealTimeMonitoring();
    }
    
    void startOperation(const std::string& operationName) {
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        MemoryOperation operation(operationName);
        operation.startSnapshot = takeMemorySnapshot(operationName + "_start");
        
        activeOperations[operationName] = operation;
        
        if (enableDetailedTracking) {
            std::cout << "[MEMORY_PROF] Started memory tracking for: " << operationName 
                      << " (baseline: " << formatMemorySize(operation.startSnapshot.totalMemoryUsed) 
                      << ")" << std::endl;
        }
    }
    
    void stopOperation(const std::string& operationName) {
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        auto it = activeOperations.find(operationName);
        if (it == activeOperations.end()) {
            std::cout << "[MEMORY_PROF] Warning: No active operation found: " << operationName << std::endl;
            return;
        }
        
        MemoryOperation& operation = it->second;
        operation.endSnapshot = takeMemorySnapshot(operationName + "_end");
        
        // Calculate memory delta
        if (operation.endSnapshot.totalMemoryUsed >= operation.startSnapshot.totalMemoryUsed) {
            operation.memoryDelta = operation.endSnapshot.totalMemoryUsed - 
                                   operation.startSnapshot.totalMemoryUsed;
        } else {
            operation.memoryDelta = 0; // Memory was freed
        }
        
        // Check for memory leaks
        if (enableLeakDetection) {
            operation.hasMemoryLeak = detectMemoryLeak(operation);
        }
        
        // Calculate peak memory during operation
        operation.peakMemoryDuringOperation = calculatePeakMemoryDuringOperation(operation);
        
        // Store completed operation
        completedOperations.push_back(operation);
        if (completedOperations.size() > maxHistorySize) {
            completedOperations.erase(completedOperations.begin());
        }
        
        activeOperations.erase(it);
        
        if (enableDetailedTracking) {
            std::cout << "[MEMORY_PROF] Completed memory tracking for: " << operationName 
                      << " (delta: " << formatMemorySize(operation.memoryDelta) 
                      << ", peak: " << formatMemorySize(operation.peakMemoryDuringOperation) 
                      << ")" << std::endl;
        }
        
        // Check memory threshold
        if (operation.endSnapshot.totalMemoryUsed > memoryThreshold) {
            std::cout << "[MEMORY_PROF] Memory warning: " << operationName 
                      << " exceeded threshold (" 
                      << formatMemorySize(operation.endSnapshot.totalMemoryUsed) 
                      << " > " << formatMemorySize(memoryThreshold) << ")" << std::endl;
        }
    }
    
    void recordAllocation(void* address, size_t size, const std::string& category = "general") {
        if (!enableDetailedTracking) return;
        
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        MemoryAllocation allocation(address, size, category);
        if (enableStackTraces) {
            allocation.stackTrace = captureStackTrace();
        }
        
        allocations[address] = allocation;
        totalAllocations++;
        currentAllocations++;
        totalMemoryAllocated += size;
        
        // Update peak memory if necessary
        size_t currentTotal = getCurrentMemoryUsage();
        if (currentTotal > peakMemoryUsage) {
            peakMemoryUsage = currentTotal;
        }
    }
    
    void recordDeallocation(void* address) {
        if (!enableDetailedTracking) return;
        
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        auto it = allocations.find(address);
        if (it != allocations.end()) {
            it->second.isActive = false;
            totalDeallocations++;
            currentAllocations--;
            totalMemoryDeallocated += it->second.size;
            allocations.erase(it);
        }
    }
    
    MemorySnapshot takeMemorySnapshot(const std::string& context = "") {
        MemorySnapshot snapshot;
        snapshot.timestamp = std::chrono::steady_clock::now();
        snapshot.operationContext = context;
        
        // Get system memory information
        snapshot.totalMemoryUsed = getCurrentMemoryUsage();
        snapshot.heapMemoryUsed = getHeapMemoryUsage();
        snapshot.stackMemoryUsed = getStackMemoryUsage();
        snapshot.peakMemoryUsed = peakMemoryUsage;
        snapshot.availableMemory = getAvailableMemory();
        
        // Calculate category breakdown
        if (enableDetailedTracking) {
            calculateCategoryBreakdown(snapshot);
        }
        
        // Store in history
        std::lock_guard<std::mutex> lock(profilingMutex);
        memoryHistory.push_back(snapshot);
        if (memoryHistory.size() > maxHistorySize) {
            memoryHistory.erase(memoryHistory.begin());
        }
        
        // Update global trend
        updateGlobalTrend(snapshot);
        
        return snapshot;
    }
    
    MemoryAnalysis analyzeMemoryUsage() {
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        auto analysisStart = std::chrono::steady_clock::now();
        MemoryAnalysis analysis;
        
        std::cout << "[MEMORY_PROF] Performing comprehensive memory analysis..." << std::endl;
        
        // Calculate memory efficiency
        analysis.memoryEfficiency = calculateMemoryEfficiency();
        
        // Detect wasted memory
        analysis.wastedMemory = calculateWastedMemory();
        
        // Assess fragmentation
        analysis.fragmentationLevel = assessFragmentation();
        
        // Calculate allocation statistics
        analysis.allocationCount = totalAllocations;
        analysis.deallocationCount = totalDeallocations;
        analysis.activeAllocations = currentAllocations;
        
        if (totalAllocations > 0) {
            analysis.averageAllocationSize = static_cast<double>(totalMemoryAllocated) / totalAllocations;
        }
        
        // Generate optimization recommendations
        generateOptimizationRecommendations(analysis);
        
        auto analysisEnd = std::chrono::steady_clock::now();
        analysis.analysisTime = analysisEnd - analysisStart;
        
        std::cout << "[MEMORY_PROF] Memory analysis completed in " 
                  << analysis.analysisTime.count() << "s" << std::endl;
        
        return analysis;
    }
    
    void startRealTimeMonitoring() {
        if (monitoringActive.load()) {
            std::cout << "[MEMORY_PROF] Real-time monitoring already active" << std::endl;
            return;
        }
        
        enableRealTimeMonitoring = true;
        monitoringActive.store(true);
        
        monitoringThread = std::thread([this]() {
            std::cout << "[MEMORY_PROF] Real-time memory monitoring started" << std::endl;
            
            while (monitoringActive.load()) {
                takeMemorySnapshot("realtime_monitor");
                std::this_thread::sleep_for(monitoringInterval);
            }
            
            std::cout << "[MEMORY_PROF] Real-time memory monitoring stopped" << std::endl;
        });
    }
    
    void stopRealTimeMonitoring() {
        if (monitoringActive.load()) {
            monitoringActive.store(false);
            if (monitoringThread.joinable()) {
                monitoringThread.join();
            }
            enableRealTimeMonitoring = false;
        }
    }
    
    void generateMemoryReport() const {
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        std::cout << "\n[MEMORY_PROF] === MEMORY USAGE ANALYSIS REPORT ===" << std::endl;
        
        // Current memory status
        size_t currentMemory = getCurrentMemoryUsage();
        std::cout << "[MEMORY_PROF] Current Memory Status:" << std::endl;
        std::cout << "[MEMORY_PROF]   Current usage: " << formatMemorySize(currentMemory) << std::endl;
        std::cout << "[MEMORY_PROF]   Peak usage: " << formatMemorySize(peakMemoryUsage) << std::endl;
        std::cout << "[MEMORY_PROF]   Available: " << formatMemorySize(getAvailableMemory()) << std::endl;
        
        // Allocation statistics
        std::cout << "[MEMORY_PROF] Allocation Statistics:" << std::endl;
        std::cout << "[MEMORY_PROF]   Total allocations: " << totalAllocations << std::endl;
        std::cout << "[MEMORY_PROF]   Total deallocations: " << totalDeallocations << std::endl;
        std::cout << "[MEMORY_PROF]   Active allocations: " << currentAllocations << std::endl;
        std::cout << "[MEMORY_PROF]   Memory allocated: " << formatMemorySize(totalMemoryAllocated) << std::endl;
        std::cout << "[MEMORY_PROF]   Memory deallocated: " << formatMemorySize(totalMemoryDeallocated) << std::endl;
        
        if (totalAllocations > 0) {
            double avgAllocationSize = static_cast<double>(totalMemoryAllocated) / totalAllocations;
            std::cout << "[MEMORY_PROF]   Average allocation size: " 
                      << formatMemorySize(static_cast<size_t>(avgAllocationSize)) << std::endl;
        }
        
        // Memory leak detection
        if (enableLeakDetection && currentAllocations > 0) {
            std::cout << "[MEMORY_PROF] Potential Memory Leaks:" << std::endl;
            
            std::unordered_map<std::string, size_t> leaksByCategory;
            size_t totalLeakSize = 0;
            
            for (const auto& [addr, allocation] : allocations) {
                if (allocation.isActive) {
                    leaksByCategory[allocation.category] += allocation.size;
                    totalLeakSize += allocation.size;
                }
            }
            
            std::cout << "[MEMORY_PROF]   Total potential leaks: " 
                      << formatMemorySize(totalLeakSize) << std::endl;
            
            for (const auto& [category, size] : leaksByCategory) {
                std::cout << "[MEMORY_PROF]     " << category << ": " 
                          << formatMemorySize(size) << std::endl;
            }
        }
        
        // Memory trend analysis
        if (!memoryHistory.empty()) {
            std::cout << "[MEMORY_PROF] Memory Trends:" << std::endl;
            std::cout << "[MEMORY_PROF]   Direction: " << globalTrend.trendDirection << std::endl;
            std::cout << "[MEMORY_PROF]   Growth rate: " << globalTrend.growthRate << " bytes/snapshot" << std::endl;
            std::cout << "[MEMORY_PROF]   Historical peak: " << formatMemorySize(globalTrend.maxMemoryObserved) << std::endl;
            std::cout << "[MEMORY_PROF]   Historical minimum: " << formatMemorySize(globalTrend.minMemoryObserved) << std::endl;
        }
        
        // Recent operations
        if (!completedOperations.empty()) {
            std::cout << "[MEMORY_PROF] Recent Operations Impact:" << std::endl;
            
            size_t recentOpsToShow = std::min(static_cast<size_t>(10), completedOperations.size());
            for (size_t i = completedOperations.size() - recentOpsToShow; i < completedOperations.size(); ++i) {
                const auto& op = completedOperations[i];
                std::cout << "[MEMORY_PROF]   " << op.operationName << ": " 
                          << formatMemorySize(op.memoryDelta) << " delta";
                if (op.hasMemoryLeak) {
                    std::cout << " (POTENTIAL LEAK)";
                }
                std::cout << std::endl;
            }
        }
        
        std::cout << "[MEMORY_PROF] === END REPORT ===" << std::endl;
    }
    
    void exportMemoryData(const std::string& filename) const {
        std::lock_guard<std::mutex> lock(profilingMutex);
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[MEMORY_PROF] Failed to open file for export: " << filename << std::endl;
            return;
        }
        
        file << "timestamp,total_memory,heap_memory,stack_memory,peak_memory,context\n";
        
        for (const auto& snapshot : memoryHistory) {
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
                snapshot.timestamp.time_since_epoch()).count();
            
            file << timestamp << ","
                 << snapshot.totalMemoryUsed << ","
                 << snapshot.heapMemoryUsed << ","
                 << snapshot.stackMemoryUsed << ","
                 << snapshot.peakMemoryUsed << ","
                 << snapshot.operationContext << "\n";
        }
        
        file.close();
        std::cout << "[MEMORY_PROF] Memory data exported to: " << filename << std::endl;
    }
    
    void setMemoryThreshold(size_t threshold) {
        memoryThreshold = threshold;
        std::cout << "[MEMORY_PROF] Memory threshold set to " << formatMemorySize(threshold) << std::endl;
    }
    
    void setMonitoringInterval(std::chrono::duration<double> interval) {
        monitoringInterval = interval;
        std::cout << "[MEMORY_PROF] Monitoring interval set to " << interval.count() << "s" << std::endl;
    }
    
    void clearHistory() {
        std::lock_guard<std::mutex> lock(profilingMutex);
        memoryHistory.clear();
        completedOperations.clear();
        globalTrend = MemoryTrend();
        std::cout << "[MEMORY_PROF] Memory history cleared" << std::endl;
    }
    
private:
    size_t getCurrentMemoryUsage() const {
#ifdef _WIN32
        PROCESS_MEMORY_COUNTERS pmc;
        if (GetProcessMemoryInfo(GetCurrentProcess(), &pmc, sizeof(pmc))) {
            return pmc.WorkingSetSize;
        }
#elif __APPLE__
        struct task_basic_info info;
        mach_msg_type_number_t size = sizeof(info);
        if (task_info(mach_task_self(), TASK_BASIC_INFO, (task_info_t)&info, &size) == KERN_SUCCESS) {
            return info.resident_size;
        }
#elif __linux__
        std::ifstream statm("/proc/self/statm");
        if (statm.is_open()) {
            size_t size, resident, share, text, lib, data, dt;
            statm >> size >> resident >> share >> text >> lib >> data >> dt;
            return resident * getpagesize();
        }
#endif
        return 0;
    }
    
    size_t getHeapMemoryUsage() const {
        // Simplified heap usage estimation
        size_t heapUsage = 0;
        for (const auto& [addr, allocation] : allocations) {
            if (allocation.isActive) {
                heapUsage += allocation.size;
            }
        }
        return heapUsage;
    }
    
    size_t getStackMemoryUsage() const {
        // Simplified stack usage estimation
#ifdef _WIN32
        MEMORY_BASIC_INFORMATION mbi;
        VirtualQuery(&mbi, &mbi, sizeof(mbi));
        return mbi.RegionSize;
#else
        struct rlimit rl;
        if (getrlimit(RLIMIT_STACK, &rl) == 0) {
            return rl.rlim_cur;
        }
#endif
        return 0;
    }
    
    size_t getAvailableMemory() const {
#ifdef _WIN32
        MEMORYSTATUSEX memInfo;
        memInfo.dwLength = sizeof(MEMORYSTATUSEX);
        if (GlobalMemoryStatusEx(&memInfo)) {
            return memInfo.ullAvailPhys;
        }
#elif __APPLE__
        vm_size_t page_size;
        vm_statistics64_data_t vm_stat;
        mach_msg_type_number_t host_size = sizeof(vm_statistics64_data_t) / sizeof(natural_t);
        
        if (host_page_size(mach_host_self(), &page_size) == KERN_SUCCESS &&
            host_statistics64(mach_host_self(), HOST_VM_INFO64, (host_info64_t)&vm_stat, &host_size) == KERN_SUCCESS) {
            return vm_stat.free_count * page_size;
        }
#elif __linux__
        std::ifstream meminfo("/proc/meminfo");
        if (meminfo.is_open()) {
            std::string line;
            while (std::getline(meminfo, line)) {
                if (line.substr(0, 12) == "MemAvailable") {
                    size_t pos = line.find_first_of("0123456789");
                    if (pos != std::string::npos) {
                        return std::stoull(line.substr(pos)) * 1024; // Convert from kB to bytes
                    }
                }
            }
        }
#endif
        return 0;
    }
    
    void calculateCategoryBreakdown(MemorySnapshot& snapshot) {
        for (const auto& [addr, allocation] : allocations) {
            if (allocation.isActive) {
                snapshot.categoryBreakdown[allocation.category] += allocation.size;
            }
        }
    }
    
    void updateGlobalTrend(const MemorySnapshot& snapshot) {
        globalTrend.memoryUsageHistory.push_back(snapshot.totalMemoryUsed);
        globalTrend.timestamps.push_back(snapshot.timestamp);
        
        // Update min/max
        if (snapshot.totalMemoryUsed > globalTrend.maxMemoryObserved) {
            globalTrend.maxMemoryObserved = snapshot.totalMemoryUsed;
        }
        if (snapshot.totalMemoryUsed < globalTrend.minMemoryObserved) {
            globalTrend.minMemoryObserved = snapshot.totalMemoryUsed;
        }
        
        // Calculate growth rate (simple linear regression)
        if (globalTrend.memoryUsageHistory.size() >= 10) {
            calculateTrendGrowthRate();
        }
        
        // Maintain history size
        if (globalTrend.memoryUsageHistory.size() > maxHistorySize) {
            globalTrend.memoryUsageHistory.erase(globalTrend.memoryUsageHistory.begin());
            globalTrend.timestamps.erase(globalTrend.timestamps.begin());
        }
    }
    
    void calculateTrendGrowthRate() {
        const auto& history = globalTrend.memoryUsageHistory;
        if (history.size() < 2) return;
        
        // Simple linear trend calculation
        size_t n = std::min(static_cast<size_t>(100), history.size());
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        
        for (size_t i = history.size() - n; i < history.size(); ++i) {
            double x = static_cast<double>(i);
            double y = static_cast<double>(history[i]);
            
            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        
        double slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
        globalTrend.growthRate = slope;
        
        // Determine trend direction
        if (slope > 100) {
            globalTrend.trendDirection = "INCREASING";
        } else if (slope < -100) {
            globalTrend.trendDirection = "DECREASING";
        } else {
            globalTrend.trendDirection = "STABLE";
        }
    }
    
    bool detectMemoryLeak(const MemoryOperation& operation) {
        // Simple leak detection: check if memory increased significantly
        if (operation.memoryDelta > 1024 * 1024) { // 1MB threshold
            return true;
        }
        
        // Check for allocation/deallocation imbalance during operation
        size_t allocsDuringOp = 0;
        size_t deallocsDuringOp = 0;
        
        for (const auto& [addr, allocation] : allocations) {
            if (allocation.allocationTime >= operation.startSnapshot.timestamp &&
                allocation.allocationTime <= operation.endSnapshot.timestamp) {
                allocsDuringOp++;
                if (!allocation.isActive) {
                    deallocsDuringOp++;
                }
            }
        }
        
        return (allocsDuringOp > deallocsDuringOp + 10); // Allow some tolerance
    }
    
    size_t calculatePeakMemoryDuringOperation(const MemoryOperation& operation) {
        size_t peak = operation.startSnapshot.totalMemoryUsed;
        
        for (const auto& snapshot : operation.intermediateSnapshots) {
            if (snapshot.totalMemoryUsed > peak) {
                peak = snapshot.totalMemoryUsed;
            }
        }
        
        if (operation.endSnapshot.totalMemoryUsed > peak) {
            peak = operation.endSnapshot.totalMemoryUsed;
        }
        
        return peak;
    }
    
    double calculateMemoryEfficiency() {
        size_t usedMemory = getCurrentMemoryUsage();
        size_t availableMemory = getAvailableMemory();
        
        if (usedMemory + availableMemory == 0) return 100.0;
        
        return (static_cast<double>(availableMemory) / (usedMemory + availableMemory)) * 100.0;
    }
    
    size_t calculateWastedMemory() {
        size_t wastedMemory = 0;
        
        // Calculate memory that's allocated but potentially unused
        for (const auto& [addr, allocation] : allocations) {
            if (allocation.isActive) {
                // Simple heuristic: large allocations that are old might be wasted
                auto age = std::chrono::steady_clock::now() - allocation.allocationTime;
                if (allocation.size > 1024 * 1024 && age > std::chrono::minutes(10)) {
                    wastedMemory += allocation.size / 2; // Estimate half as potentially wasted
                }
            }
        }
        
        return wastedMemory;
    }
    
    size_t assessFragmentation() {
        // Simplified fragmentation assessment
        std::vector<size_t> allocSizes;
        for (const auto& [addr, allocation] : allocations) {
            if (allocation.isActive) {
                allocSizes.push_back(allocation.size);
            }
        }
        
        if (allocSizes.empty()) return 0;
        
        std::sort(allocSizes.begin(), allocSizes.end());
        
        // Calculate size variance as fragmentation indicator
        double mean = std::accumulate(allocSizes.begin(), allocSizes.end(), 0.0) / allocSizes.size();
        double variance = 0.0;
        
        for (size_t size : allocSizes) {
            variance += std::pow(static_cast<double>(size) - mean, 2);
        }
        variance /= allocSizes.size();
        
        return static_cast<size_t>(std::sqrt(variance));
    }
    
    void generateOptimizationRecommendations(MemoryAnalysis& analysis) {
        // Memory efficiency recommendations
        if (analysis.memoryEfficiency < 70.0) {
            analysis.optimizationRecommendations.push_back(
                "Memory efficiency is low - consider reducing memory usage or increasing available memory");
        }
        
        // Wasted memory recommendations
        if (analysis.wastedMemory > 10 * 1024 * 1024) { // 10MB
            analysis.optimizationRecommendations.push_back(
                "Significant wasted memory detected - review long-lived large allocations");
        }
        
        // Fragmentation recommendations
        if (analysis.fragmentationLevel > 1024 * 1024) { // 1MB
            analysis.optimizationRecommendations.push_back(
                "High memory fragmentation - consider memory pooling or defragmentation");
        }
        
        // Allocation pattern recommendations
        if (currentAllocations > 10000) {
            analysis.optimizationRecommendations.push_back(
                "Large number of active allocations - consider object pooling");
        }
        
        if (analysis.averageAllocationSize < 64) {
            analysis.optimizationRecommendations.push_back(
                "Many small allocations detected - consider memory consolidation");
        }
        
        // Leak detection recommendations
        if (currentAllocations > totalDeallocations * 1.1) {
            analysis.optimizationRecommendations.push_back(
                "Potential memory leaks detected - review allocation/deallocation patterns");
        }
    }
    
    std::string captureStackTrace() {
        // Simplified stack trace capture (platform-specific implementation needed)
        return "Stack trace capture not implemented";
    }
    
    std::string formatMemorySize(size_t bytes) const {
        const char* units[] = {"B", "KB", "MB", "GB", "TB"};
        int unit = 0;
        double size = static_cast<double>(bytes);
        
        while (size >= 1024.0 && unit < 4) {
            size /= 1024.0;
            unit++;
        }
        
        char buffer[64];
        if (unit == 0) {
            snprintf(buffer, sizeof(buffer), "%zu %s", bytes, units[unit]);
        } else {
            snprintf(buffer, sizeof(buffer), "%.2f %s", size, units[unit]);
        }
        
        return std::string(buffer);
    }
};

// Global memory profiler
static std::unique_ptr<MemoryUsageProfiler> g_memoryProfiler;

void initializeMemoryProfiler() {
    g_memoryProfiler = std::make_unique<MemoryUsageProfiler>();
}

void startMemoryOperation(const std::string& operationName) {
    if (!g_memoryProfiler) {
        g_memoryProfiler = std::make_unique<MemoryUsageProfiler>();
    }
    g_memoryProfiler->startOperation(operationName);
}

void stopMemoryOperation(const std::string& operationName) {
    if (g_memoryProfiler) {
        g_memoryProfiler->stopOperation(operationName);
    }
}

void recordMemoryAllocation(void* address, size_t size, const std::string& category) {
    if (g_memoryProfiler) {
        g_memoryProfiler->recordAllocation(address, size, category);
    }
}

void recordMemoryDeallocation(void* address) {
    if (g_memoryProfiler) {
        g_memoryProfiler->recordDeallocation(address);
    }
}

void takeMemorySnapshot(const std::string& context) {
    if (g_memoryProfiler) {
        g_memoryProfiler->takeMemorySnapshot(context);
    }
}

void generateMemoryReport() {
    if (g_memoryProfiler) {
        g_memoryProfiler->generateMemoryReport();
    }
}

void startMemoryMonitoring() {
    if (!g_memoryProfiler) {
        g_memoryProfiler = std::make_unique<MemoryUsageProfiler>();
    }
    g_memoryProfiler->startRealTimeMonitoring();
}

void stopMemoryMonitoring() {
    if (g_memoryProfiler) {
        g_memoryProfiler->stopRealTimeMonitoring();
    }
}

// RAII memory tracker class
class ScopedMemoryTracker {
private:
    std::string operationName;
    
public:
    explicit ScopedMemoryTracker(const std::string& name) : operationName(name) {
        startMemoryOperation(operationName);
    }
    
    ~ScopedMemoryTracker() {
        stopMemoryOperation(operationName);
    }
};

#define TRACK_MEMORY(name) ScopedMemoryTracker memTracker_##__LINE__(name)