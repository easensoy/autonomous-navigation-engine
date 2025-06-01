#pragma once
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>

struct PerformanceMetrics {
    std::chrono::duration<double> executionTime;
    size_t memoryUsed;
    size_t nodesExplored;
    size_t pathLength;
    double pathOptimality;
    double algorithmEfficiency;
};

class PerformanceProfiler {
private:
    std::unordered_map<std::string, PerformanceMetrics> profileData;
    std::chrono::steady_clock::time_point currentStartTime;
    std::string currentOperation;
    bool profilingActive;

public:
    PerformanceProfiler();
    
    void startProfiling(const std::string& operationName);
    void stopProfiling();
    void recordMetric(const std::string& metricName, double value);
    
    PerformanceMetrics getMetrics(const std::string& operationName) const;
    std::vector<std::string> getAllOperations() const;
    void clearProfile(const std::string& operationName);
    
    void generatePerformanceReport() const;
    void exportToCSV(const std::string& filename) const;
    void comparePeformance(const std::string& operation1, const std::string& operation2) const;
    
    double getAverageExecutionTime(const std::string& operationName) const;
    size_t getTotalMemoryUsage() const;
    void setBenchmarkBaseline(const std::string& operationName);
    
    void enableContinuousProfiling(bool enable);
    void setMemoryTrackingPrecision(int precision);
    bool isProfilingActive() const;
};