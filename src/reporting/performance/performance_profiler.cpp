#include "reporting/PerformanceProfiler.hpp"
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <chrono>
#include <string>

// PerformanceProfiler: one active timed operation at a time; metrics are kept per operation name.

PerformanceProfiler::PerformanceProfiler() : profilingActive(false) {}

void PerformanceProfiler::startProfiling(const std::string& operationName) {
    if (profilingActive) {
        stopProfiling();
    }
    currentOperation = operationName;
    currentStartTime = std::chrono::steady_clock::now();
    profilingActive = true;
}

void PerformanceProfiler::stopProfiling() {
    if (!profilingActive) {
        return;
    }
    const std::chrono::duration<double> elapsed = std::chrono::steady_clock::now() - currentStartTime;
    PerformanceMetrics& metrics = profileData[currentOperation];
    metrics.executionTime = elapsed;
    auto& totals = timingTotals[currentOperation];
    totals.first += elapsed.count();
    totals.second += 1;
    profilingActive = false;
}

void PerformanceProfiler::recordMetric(const std::string& metricName, double value) {
    if (currentOperation.empty()) {
        return;
    }
    PerformanceMetrics& metrics = profileData[currentOperation];
    if (metricName == "memoryUsed") {
        metrics.memoryUsed = static_cast<size_t>(value);
    } else if (metricName == "nodesExplored") {
        metrics.nodesExplored = static_cast<size_t>(value);
    } else if (metricName == "pathLength") {
        metrics.pathLength = static_cast<size_t>(value);
    } else if (metricName == "pathOptimality") {
        metrics.pathOptimality = value;
    } else if (metricName == "algorithmEfficiency") {
        metrics.algorithmEfficiency = value;
    }
}

PerformanceMetrics PerformanceProfiler::getMetrics(const std::string& operationName) const {
    auto it = profileData.find(operationName);
    if (it == profileData.end()) {
        return PerformanceMetrics{};  // zeroed metrics for an operation that was never profiled
    }
    return it->second;
}

std::vector<std::string> PerformanceProfiler::getAllOperations() const {
    std::vector<std::string> names;
    for (const auto& entry : profileData) {
        names.push_back(entry.first);
    }
    std::sort(names.begin(), names.end());
    return names;
}

void PerformanceProfiler::clearProfile(const std::string& operationName) {
    profileData.erase(operationName);
    timingTotals.erase(operationName);
}

void PerformanceProfiler::generatePerformanceReport() const {
    std::cout << "\n=== Performance Report ===" << std::endl;
    std::cout << std::left << std::setw(32) << "Operation" << std::right << std::setw(14) << "Last (ms)"
              << std::setw(14) << "Avg (ms)" << std::setw(8) << "Runs" << std::endl;
    for (const std::string& name : getAllOperations()) {
        const auto& metrics = profileData.at(name);
        auto totals = timingTotals.find(name);
        const size_t runs = totals != timingTotals.end() ? totals->second.second : 0;
        std::cout << std::left << std::setw(32) << name << std::right << std::fixed << std::setprecision(3)
                  << std::setw(14) << metrics.executionTime.count() * 1000.0 << std::setw(14)
                  << getAverageExecutionTime(name) * 1000.0 << std::setw(8) << runs << std::endl;
    }
}

void PerformanceProfiler::exportToCSV(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    out << "operation,last_seconds,average_seconds,runs,memory_used,nodes_explored,path_length,path_optimality,"
           "algorithm_efficiency\n";
    for (const std::string& name : getAllOperations()) {
        const auto& m = profileData.at(name);
        auto totals = timingTotals.find(name);
        out << name << "," << m.executionTime.count() << "," << getAverageExecutionTime(name) << ","
            << (totals != timingTotals.end() ? totals->second.second : 0) << "," << m.memoryUsed << ","
            << m.nodesExplored << "," << m.pathLength << "," << m.pathOptimality << "," << m.algorithmEfficiency
            << "\n";
    }
}

void PerformanceProfiler::comparePeformance(const std::string& operation1, const std::string& operation2) const {
    const double t1 = getAverageExecutionTime(operation1);
    const double t2 = getAverageExecutionTime(operation2);
    std::cout << "[PROFILER] " << operation1 << ": " << t1 * 1000.0 << " ms, " << operation2 << ": "
              << t2 * 1000.0 << " ms";
    if (t1 > 0.0 && t2 > 0.0) {
        std::cout << " (" << operation1 << " is " << (t2 / t1) << "x the speed of " << operation2 << ")";
    }
    std::cout << std::endl;
}

double PerformanceProfiler::getAverageExecutionTime(const std::string& operationName) const {
    auto it = timingTotals.find(operationName);
    if (it == timingTotals.end() || it->second.second == 0) {
        return 0.0;
    }
    return it->second.first / static_cast<double>(it->second.second);
}

size_t PerformanceProfiler::getTotalMemoryUsage() const {
    size_t total = 0;
    for (const auto& entry : profileData) {
        total += entry.second.memoryUsed;
    }
    return total;
}

bool PerformanceProfiler::isProfilingActive() const {
    return profilingActive;
}
