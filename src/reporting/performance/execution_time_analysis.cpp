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

class ExecutionTimeAnalyzer {
private:
    struct TimingResult {
        std::chrono::steady_clock::time_point startTime;
        std::chrono::steady_clock::time_point endTime;
        std::chrono::duration<double> executionTime;
        std::string operationName;
        size_t inputSize;
        std::unordered_map<std::string, double> operationMetrics;
        std::vector<std::chrono::duration<double>> iterationTimes;
        
        TimingResult(const std::string& name) 
            : operationName(name), inputSize(0), executionTime(std::chrono::duration<double>::zero()) {}
    };
    
    struct PerformanceBenchmark {
        std::string benchmarkName;
        std::vector<TimingResult> results;
        double averageTime;
        double medianTime;
        double minTime;
        double maxTime;
        double standardDeviation;
        double throughput;
        size_t sampleCount;
        std::chrono::steady_clock::time_point benchmarkTimestamp;
        
        PerformanceBenchmark(const std::string& name) 
            : benchmarkName(name), averageTime(0.0), medianTime(0.0), 
              minTime(std::numeric_limits<double>::infinity()), 
              maxTime(0.0), standardDeviation(0.0), throughput(0.0), 
              sampleCount(0), benchmarkTimestamp(std::chrono::steady_clock::now()) {}
    };
    
    struct RealTimeMetrics {
        std::chrono::steady_clock::time_point lastMeasurement;
        std::queue<std::chrono::duration<double>> recentTimes;
        double rollingAverage;
        double peakTime;
        size_t measurementCount;
        double cumulativeTime;
        
        RealTimeMetrics() : rollingAverage(0.0), peakTime(0.0), 
                           measurementCount(0), cumulativeTime(0.0) {}
    };
    
    // Data storage
    std::unordered_map<std::string, TimingResult> activeOperations;
    std::unordered_map<std::string, PerformanceBenchmark> benchmarks;
    std::unordered_map<std::string, RealTimeMetrics> realTimeMetrics;
    std::vector<TimingResult> completedOperations;
    
    // Configuration
    bool enableDetailedProfiling;
    bool enableRealTimeTracking;
    size_t maxHistorySize;
    size_t rollingWindowSize;
    double performanceThreshold;
    bool enableAutomaticReporting;
    
    // Thread safety
    mutable std::mutex analysisMutex;
    
    // Analysis parameters
    std::chrono::duration<double> reportingInterval;
    std::chrono::steady_clock::time_point lastReportTime;
    
public:
    ExecutionTimeAnalyzer() 
        : enableDetailedProfiling(true), enableRealTimeTracking(true), 
          maxHistorySize(10000), rollingWindowSize(100), performanceThreshold(1.0),
          enableAutomaticReporting(false), reportingInterval(std::chrono::seconds(60)),
          lastReportTime(std::chrono::steady_clock::now()) {
        
        std::cout << "[EXEC_TIME] Execution time analyzer initialized" << std::endl;
    }
    
    void startOperation(const std::string& operationName, size_t inputSize = 0) {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        TimingResult result(operationName);
        result.startTime = std::chrono::steady_clock::now();
        result.inputSize = inputSize;
        
        activeOperations[operationName] = result;
        
        if (enableDetailedProfiling) {
            std::cout << "[EXEC_TIME] Started timing operation: " << operationName 
                      << " (input size: " << inputSize << ")" << std::endl;
        }
    }
    
    void stopOperation(const std::string& operationName, 
                      const std::unordered_map<std::string, double>& metrics = {}) {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        auto it = activeOperations.find(operationName);
        if (it == activeOperations.end()) {
            std::cout << "[EXEC_TIME] Warning: No active operation found: " << operationName << std::endl;
            return;
        }
        
        TimingResult& result = it->second;
        result.endTime = std::chrono::steady_clock::now();
        result.executionTime = result.endTime - result.startTime;
        result.operationMetrics = metrics;
        
        // Update real-time metrics
        if (enableRealTimeTracking) {
            updateRealTimeMetrics(operationName, result.executionTime);
        }
        
        // Store completed operation
        completedOperations.push_back(result);
        if (completedOperations.size() > maxHistorySize) {
            completedOperations.erase(completedOperations.begin());
        }
        
        // Remove from active operations
        activeOperations.erase(it);
        
        if (enableDetailedProfiling) {
            std::cout << "[EXEC_TIME] Completed operation: " << operationName 
                      << " in " << result.executionTime.count() << "s" << std::endl;
        }
        
        // Check performance threshold
        if (result.executionTime.count() > performanceThreshold) {
            std::cout << "[EXEC_TIME] Performance warning: " << operationName 
                      << " exceeded threshold (" << result.executionTime.count() 
                      << "s > " << performanceThreshold << "s)" << std::endl;
        }
        
        // Automatic reporting
        if (enableAutomaticReporting) {
            auto currentTime = std::chrono::steady_clock::now();
            if (currentTime - lastReportTime > reportingInterval) {
                generatePerformanceReport();
                lastReportTime = currentTime;
            }
        }
    }
    
    void markIteration(const std::string& operationName) {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        auto it = activeOperations.find(operationName);
        if (it != activeOperations.end()) {
            auto currentTime = std::chrono::steady_clock::now();
            auto iterationTime = currentTime - it->second.startTime;
            it->second.iterationTimes.push_back(iterationTime);
            it->second.startTime = currentTime; // Reset for next iteration
        }
    }
    
    void runBenchmark(const std::string& benchmarkName, 
                     std::function<void()> operation, 
                     size_t iterations = 100,
                     size_t inputSize = 0) {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        std::cout << "[EXEC_TIME] Running benchmark: " << benchmarkName 
                  << " (" << iterations << " iterations)" << std::endl;
        
        PerformanceBenchmark benchmark(benchmarkName);
        
        for (size_t i = 0; i < iterations; ++i) {
            TimingResult result(benchmarkName + "_iteration_" + std::to_string(i));
            result.inputSize = inputSize;
            result.startTime = std::chrono::steady_clock::now();
            
            try {
                operation();
            } catch (const std::exception& e) {
                std::cout << "[EXEC_TIME] Benchmark iteration failed: " << e.what() << std::endl;
                continue;
            }
            
            result.endTime = std::chrono::steady_clock::now();
            result.executionTime = result.endTime - result.startTime;
            
            benchmark.results.push_back(result);
            
            // Progress reporting
            if ((i + 1) % 10 == 0 || i == iterations - 1) {
                std::cout << "[EXEC_TIME] Benchmark progress: " << (i + 1) 
                          << "/" << iterations << " iterations" << std::endl;
            }
        }
        
        // Calculate benchmark statistics
        calculateBenchmarkStatistics(benchmark);
        benchmarks[benchmarkName] = benchmark;
        
        std::cout << "[EXEC_TIME] Benchmark completed: " << benchmarkName 
                  << " - Average: " << benchmark.averageTime << "s, "
                  << "Std Dev: " << benchmark.standardDeviation << "s" << std::endl;
    }
    
    void runComparativeBenchmark(const std::vector<std::pair<std::string, std::function<void()>>>& operations,
                                size_t iterations = 50) {
        std::cout << "[EXEC_TIME] Running comparative benchmark with " 
                  << operations.size() << " operations" << std::endl;
        
        std::vector<PerformanceBenchmark> results;
        
        for (const auto& [name, operation] : operations) {
            runBenchmark(name, operation, iterations);
            results.push_back(benchmarks[name]);
        }
        
        // Generate comparison report
        generateComparisonReport(results);
    }
    
    double getOperationTime(const std::string& operationName) const {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        // Check completed operations (most recent)
        for (auto it = completedOperations.rbegin(); it != completedOperations.rend(); ++it) {
            if (it->operationName == operationName) {
                return it->executionTime.count();
            }
        }
        
        return -1.0; // Not found
    }
    
    std::vector<double> getOperationHistory(const std::string& operationName, size_t maxResults = 100) const {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        std::vector<double> history;
        size_t count = 0;
        
        for (auto it = completedOperations.rbegin(); 
             it != completedOperations.rend() && count < maxResults; ++it) {
            if (it->operationName == operationName) {
                history.push_back(it->executionTime.count());
                count++;
            }
        }
        
        return history;
    }
    
    double getAverageExecutionTime(const std::string& operationName) const {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        auto it = realTimeMetrics.find(operationName);
        if (it != realTimeMetrics.end()) {
            return it->second.rollingAverage;
        }
        
        // Calculate from completed operations
        std::vector<double> times;
        for (const auto& result : completedOperations) {
            if (result.operationName == operationName) {
                times.push_back(result.executionTime.count());
            }
        }
        
        if (times.empty()) return 0.0;
        
        return std::accumulate(times.begin(), times.end(), 0.0) / times.size();
    }
    
    void analyzePerformanceTrends(const std::string& operationName) {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        std::vector<double> times = getOperationHistory(operationName, 1000);
        if (times.size() < 10) {
            std::cout << "[EXEC_TIME] Insufficient data for trend analysis: " << operationName << std::endl;
            return;
        }
        
        std::cout << "[EXEC_TIME] Performance trend analysis for: " << operationName << std::endl;
        
        // Calculate moving averages
        std::vector<double> movingAverage;
        size_t windowSize = std::min(static_cast<size_t>(20), times.size() / 2);
        
        for (size_t i = windowSize - 1; i < times.size(); ++i) {
            double avg = 0.0;
            for (size_t j = i - windowSize + 1; j <= i; ++j) {
                avg += times[j];
            }
            movingAverage.push_back(avg / windowSize);
        }
        
        // Detect trend
        if (movingAverage.size() >= 2) {
            double slope = (movingAverage.back() - movingAverage.front()) / (movingAverage.size() - 1);
            std::string trend = (slope > 0.01) ? "DEGRADING" : 
                               (slope < -0.01) ? "IMPROVING" : "STABLE";
            
            std::cout << "[EXEC_TIME] Trend: " << trend << " (slope: " << slope << ")" << std::endl;
        }
        
        // Performance statistics
        auto minMax = std::minmax_element(times.begin(), times.end());
        double variance = calculateVariance(times);
        
        std::cout << "[EXEC_TIME] Statistics - Min: " << *minMax.first 
                  << "s, Max: " << *minMax.second << "s, Variance: " << variance << std::endl;
    }
    
    void generatePerformanceReport() const {
        std::lock_guard<std::mutex> lock(analysisMutex);
        
        std::cout << "\n[EXEC_TIME] === EXECUTION TIME ANALYSIS REPORT ===" << std::endl;
        
        // Active operations
        if (!activeOperations.empty()) {
            std::cout << "[EXEC_TIME] Active Operations:" << std::endl;
            for (const auto& [name, result] : activeOperations) {
                auto currentTime = std::chrono::steady_clock::now();
                auto elapsed = currentTime - result.startTime;
                std::cout << "[EXEC_TIME]   " << name << ": " 
                          << elapsed.count() << "s (running)" << std::endl;
            }
        }
        
        // Real-time metrics summary
        if (!realTimeMetrics.empty()) {
            std::cout << "[EXEC_TIME] Real-time Performance Metrics:" << std::endl;
            for (const auto& [name, metrics] : realTimeMetrics) {
                std::cout << "[EXEC_TIME]   " << name << ":" << std::endl;
                std::cout << "[EXEC_TIME]     Average: " << metrics.rollingAverage << "s" << std::endl;
                std::cout << "[EXEC_TIME]     Peak: " << metrics.peakTime << "s" << std::endl;
                std::cout << "[EXEC_TIME]     Count: " << metrics.measurementCount << std::endl;
            }
        }
        
        // Recent performance summary
        if (!completedOperations.empty()) {
            std::cout << "[EXEC_TIME] Recent Operations Summary:" << std::endl;
            
            std::unordered_map<std::string, std::vector<double>> operationTimes;
            for (const auto& result : completedOperations) {
                operationTimes[result.operationName].push_back(result.executionTime.count());
            }
            
            for (const auto& [name, times] : operationTimes) {
                if (!times.empty()) {
                    double avg = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
                    auto minMax = std::minmax_element(times.begin(), times.end());
                    
                    std::cout << "[EXEC_TIME]   " << name << ": " 
                              << times.size() << " executions, avg " << avg 
                              << "s (range: " << *minMax.first << "-" << *minMax.second << "s)" << std::endl;
                }
            }
        }
        
        // Benchmarks summary
        if (!benchmarks.empty()) {
            std::cout << "[EXEC_TIME] Benchmark Results:" << std::endl;
            for (const auto& [name, benchmark] : benchmarks) {
                std::cout << "[EXEC_TIME]   " << name << ": " 
                          << benchmark.averageTime << "s ± " << benchmark.standardDeviation 
                          << "s (" << benchmark.sampleCount << " samples)" << std::endl;
            }
        }
        
        std::cout << "[EXEC_TIME] === END REPORT ===" << std::endl;
    }
    
    void setPerformanceThreshold(double threshold) {
        performanceThreshold = threshold;
        std::cout << "[EXEC_TIME] Performance threshold set to " << threshold << "s" << std::endl;
    }
    
    void enableAutomaticReporting(bool enable, std::chrono::duration<double> interval = std::chrono::seconds(60)) {
        enableAutomaticReporting = enable;
        reportingInterval = interval;
        std::cout << "[EXEC_TIME] Automatic reporting " << (enable ? "enabled" : "disabled") 
                  << " (interval: " << interval.count() << "s)" << std::endl;
    }
    
    void clearHistory() {
        std::lock_guard<std::mutex> lock(analysisMutex);
        completedOperations.clear();
        benchmarks.clear();
        realTimeMetrics.clear();
        std::cout << "[EXEC_TIME] Performance history cleared" << std::endl;
    }
    
private:
    void updateRealTimeMetrics(const std::string& operationName, 
                              std::chrono::duration<double> executionTime) {
        RealTimeMetrics& metrics = realTimeMetrics[operationName];
        
        metrics.lastMeasurement = std::chrono::steady_clock::now();
        metrics.recentTimes.push(executionTime);
        metrics.measurementCount++;
        metrics.cumulativeTime += executionTime.count();
        
        // Update peak time
        if (executionTime.count() > metrics.peakTime) {
            metrics.peakTime = executionTime.count();
        }
        
        // Maintain rolling window
        while (metrics.recentTimes.size() > rollingWindowSize) {
            metrics.recentTimes.pop();
        }
        
        // Calculate rolling average
        if (!metrics.recentTimes.empty()) {
            double sum = 0.0;
            std::queue<std::chrono::duration<double>> tempQueue = metrics.recentTimes;
            while (!tempQueue.empty()) {
                sum += tempQueue.front().count();
                tempQueue.pop();
            }
            metrics.rollingAverage = sum / metrics.recentTimes.size();
        }
    }
    
    void calculateBenchmarkStatistics(PerformanceBenchmark& benchmark) {
        if (benchmark.results.empty()) return;
        
        std::vector<double> times;
        for (const auto& result : benchmark.results) {
            times.push_back(result.executionTime.count());
        }
        
        benchmark.sampleCount = times.size();
        
        // Calculate basic statistics
        std::sort(times.begin(), times.end());
        benchmark.minTime = times.front();
        benchmark.maxTime = times.back();
        benchmark.medianTime = times[times.size() / 2];
        benchmark.averageTime = std::accumulate(times.begin(), times.end(), 0.0) / times.size();
        
        // Calculate standard deviation
        benchmark.standardDeviation = std::sqrt(calculateVariance(times));
        
        // Calculate throughput (operations per second)
        if (benchmark.averageTime > 0) {
            benchmark.throughput = 1.0 / benchmark.averageTime;
        }
    }
    
    double calculateVariance(const std::vector<double>& values) const {
        if (values.size() <= 1) return 0.0;
        
        double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
        double variance = 0.0;
        
        for (double value : values) {
            variance += std::pow(value - mean, 2);
        }
        
        return variance / (values.size() - 1);
    }
    
    void generateComparisonReport(const std::vector<PerformanceBenchmark>& benchmarks) const {
        std::cout << "\n[EXEC_TIME] === COMPARATIVE BENCHMARK REPORT ===" << std::endl;
        
        if (benchmarks.empty()) return;
        
        // Find best performing benchmark
        auto bestBenchmark = std::min_element(benchmarks.begin(), benchmarks.end(),
            [](const PerformanceBenchmark& a, const PerformanceBenchmark& b) {
                return a.averageTime < b.averageTime;
            });
        
        std::cout << "[EXEC_TIME] Best performer: " << bestBenchmark->benchmarkName 
                  << " (" << bestBenchmark->averageTime << "s average)" << std::endl;
        
        // Compare all benchmarks to best
        for (const auto& benchmark : benchmarks) {
            double relativePerformance = benchmark.averageTime / bestBenchmark->averageTime;
            std::cout << "[EXEC_TIME] " << benchmark.benchmarkName << ": " 
                      << benchmark.averageTime << "s (x" << relativePerformance 
                      << " relative to best)" << std::endl;
        }
        
        std::cout << "[EXEC_TIME] === END COMPARISON ===" << std::endl;
    }
};

// Global execution time analyzer
static std::unique_ptr<ExecutionTimeAnalyzer> g_timeAnalyzer;

void initializeExecutionTimeAnalyzer() {
    g_timeAnalyzer = std::make_unique<ExecutionTimeAnalyzer>();
}

void startTimingOperation(const std::string& operationName, size_t inputSize) {
    if (!g_timeAnalyzer) {
        g_timeAnalyzer = std::make_unique<ExecutionTimeAnalyzer>();
    }
    g_timeAnalyzer->startOperation(operationName, inputSize);
}

void stopTimingOperation(const std::string& operationName, 
                        const std::unordered_map<std::string, double>& metrics) {
    if (g_timeAnalyzer) {
        g_timeAnalyzer->stopOperation(operationName, metrics);
    }
}

void markIterationTiming(const std::string& operationName) {
    if (g_timeAnalyzer) {
        g_timeAnalyzer->markIteration(operationName);
    }
}

double getLastExecutionTime(const std::string& operationName) {
    if (g_timeAnalyzer) {
        return g_timeAnalyzer->getOperationTime(operationName);
    }
    return -1.0;
}

void runPerformanceBenchmark(const std::string& benchmarkName, 
                            std::function<void()> operation, 
                            size_t iterations) {
    if (!g_timeAnalyzer) {
        g_timeAnalyzer = std::make_unique<ExecutionTimeAnalyzer>();
    }
    g_timeAnalyzer->runBenchmark(benchmarkName, operation, iterations);
}

void generateExecutionTimeReport() {
    if (g_timeAnalyzer) {
        g_timeAnalyzer->generatePerformanceReport();
    }
}

// RAII timer class for automatic timing
class ScopedTimer {
private:
    std::string operationName;
    
public:
    explicit ScopedTimer(const std::string& name, size_t inputSize = 0) 
        : operationName(name) {
        startTimingOperation(operationName, inputSize);
    }
    
    ~ScopedTimer() {
        stopTimingOperation(operationName);
    }
};

#define TIME_OPERATION(name) ScopedTimer timer_##__LINE__(name)
#define TIME_OPERATION_WITH_SIZE(name, size) ScopedTimer timer_##__LINE__(name, size)