#include "reporting/NavigationLogger.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <stack>

enum class ErrorSeverity {
    LOW,
    MEDIUM,
    HIGH,
    CRITICAL
};

enum class ErrorCategory {
    GRAPH_ERROR,
    ALGORITHM_ERROR,
    NAVIGATION_ERROR,
    SYSTEM_ERROR,
    VALIDATION_ERROR,
    PERFORMANCE_ERROR,
    USER_ERROR
};

struct ErrorInfo {
    int errorId;
    ErrorCategory category;
    ErrorSeverity severity;
    std::string errorCode;
    std::string description;
    std::string context;
    std::chrono::steady_clock::time_point timestamp;
    std::unordered_map<std::string, std::string> metadata;
    std::string stackTrace;
    bool resolved;
    
    ErrorInfo(int id, ErrorCategory cat, ErrorSeverity sev, const std::string& code, const std::string& desc)
        : errorId(id), category(cat), severity(sev), errorCode(code), description(desc),
          timestamp(std::chrono::steady_clock::now()), resolved(false) {}
};

class ErrorReporter {
private:
    std::unique_ptr<NavigationLogger> logger;
    std::vector<ErrorInfo> errorHistory;
    std::unordered_map<std::string, int> errorCounts;
    int nextErrorId;
    bool enableStackTrace;
    bool enableAutoReporting;
    size_t maxHistorySize;
    
    std::unordered_map<ErrorCategory, std::string> categoryNames = {
        {ErrorCategory::GRAPH_ERROR, "GRAPH_ERROR"},
        {ErrorCategory::ALGORITHM_ERROR, "ALGORITHM_ERROR"},
        {ErrorCategory::NAVIGATION_ERROR, "NAVIGATION_ERROR"},
        {ErrorCategory::SYSTEM_ERROR, "SYSTEM_ERROR"},
        {ErrorCategory::VALIDATION_ERROR, "VALIDATION_ERROR"},
        {ErrorCategory::PERFORMANCE_ERROR, "PERFORMANCE_ERROR"},
        {ErrorCategory::USER_ERROR, "USER_ERROR"}
    };
    
    std::unordered_map<ErrorSeverity, std::string> severityNames = {
        {ErrorSeverity::LOW, "LOW"},
        {ErrorSeverity::MEDIUM, "MEDIUM"},
        {ErrorSeverity::HIGH, "HIGH"},
        {ErrorSeverity::CRITICAL, "CRITICAL"}
    };
    
public:
    ErrorReporter(const std::string& logFile) 
        : logger(std::make_unique<NavigationLogger>(logFile)), 
          nextErrorId(1), enableStackTrace(true), enableAutoReporting(true), maxHistorySize(1000) {
        
        logger->setLogLevel(LogLevel::ERROR);
        logger->enableConsoleOutput(true);
        
        std::cout << "[ERROR_REPORTER] Error reporting system initialized" << std::endl;
    }
    
    int reportError(ErrorCategory category, ErrorSeverity severity, const std::string& errorCode, 
                   const std::string& description, const std::string& context = "") {
        
        ErrorInfo error(nextErrorId++, category, severity, errorCode, description);
        error.context = context;
        
        if (enableStackTrace) {
            error.stackTrace = captureStackTrace();
        }
        
        // Convert severity to log level
        LogLevel logLevel = severityToLogLevel(severity);
        
        // Prepare metadata
        std::unordered_map<std::string, std::string> metadata;
        metadata["error_id"] = std::to_string(error.errorId);
        metadata["category"] = categoryNames[category];
        metadata["severity"] = severityNames[severity];
        metadata["error_code"] = errorCode;
        metadata["context"] = context;
        
        // Log the error
        std::string logMessage = formatErrorMessage(error);
        logger->logWithMetadata(logLevel, "ERROR_REPORT", logMessage, metadata);
        
        // Store in history
        errorHistory.push_back(error);
        errorCounts[errorCode]++;
        
        // Maintain history size
        if (errorHistory.size() > maxHistorySize) {
            errorHistory.erase(errorHistory.begin());
        }
        
        // Handle critical errors
        if (severity == ErrorSeverity::CRITICAL) {
            handleCriticalError(error);
        }
        
        std::cout << "[ERROR_REPORTER] Error " << error.errorId << " reported: " << description << std::endl;
        
        return error.errorId;
    }
    
    void reportGraphError(const std::string& operation, int nodeId = -1, const std::string& details = "") {
        std::string context = "Operation: " + operation;
        if (nodeId != -1) context += ", Node: " + std::to_string(nodeId);
        if (!details.empty()) context += ", Details: " + details;
        
        reportError(ErrorCategory::GRAPH_ERROR, ErrorSeverity::HIGH, "GRAPH_OP_FAILED", 
                   "Graph operation failed", context);
    }
    
    void reportAlgorithmError(const std::string& algorithm, const std::string& phase, 
                             const std::string& errorDetails) {
        std::string context = "Algorithm: " + algorithm + ", Phase: " + phase;
        reportError(ErrorCategory::ALGORITHM_ERROR, ErrorSeverity::HIGH, "ALGO_EXECUTION_ERROR",
                   "Algorithm execution error: " + errorDetails, context);
    }
    
    void reportNavigationError(const std::string& operation, const std::vector<int>& path, 
                              const std::string& errorDetails) {
        std::ostringstream pathStr;
        for (size_t i = 0; i < path.size() && i < 10; ++i) {
            if (i > 0) pathStr << "->";
            pathStr << path[i];
        }
        if (path.size() > 10) pathStr << "...";
        
        std::string context = "Operation: " + operation + ", Path: " + pathStr.str();
        reportError(ErrorCategory::NAVIGATION_ERROR, ErrorSeverity::MEDIUM, "NAV_ERROR",
                   "Navigation error: " + errorDetails, context);
    }
    
    void reportValidationError(const std::string& validationType, const std::vector<int>& path,
                              const std::string& violationDetails) {
        std::string context = "Validation: " + validationType + ", Path length: " + std::to_string(path.size());
        reportError(ErrorCategory::VALIDATION_ERROR, ErrorSeverity::MEDIUM, "VALIDATION_FAILED",
                   "Path validation failed: " + violationDetails, context);
    }
    
    void reportPerformanceError(const std::string& operation, double executionTime, 
                               double threshold, const std::string& details = "") {
        std::string context = "Operation: " + operation + ", Time: " + std::to_string(executionTime) + 
                             "s, Threshold: " + std::to_string(threshold) + "s";
        if (!details.empty()) context += ", Details: " + details;
        
        reportError(ErrorCategory::PERFORMANCE_ERROR, ErrorSeverity::LOW, "PERFORMANCE_DEGRADATION",
                   "Performance threshold exceeded", context);
    }
    
    void reportSystemError(const std::string& component, const std::string& errorDetails) {
        std::string context = "Component: " + component;
        reportError(ErrorCategory::SYSTEM_ERROR, ErrorSeverity::CRITICAL, "SYSTEM_FAILURE",
                   "System component failure: " + errorDetails, context);
    }
    
    bool resolveError(int errorId, const std::string& resolution = "") {
        auto it = std::find_if(errorHistory.begin(), errorHistory.end(),
                              [errorId](const ErrorInfo& error) { return error.errorId == errorId; });
        
        if (it == errorHistory.end()) {
            logger->warning("Attempted to resolve non-existent error ID: " + std::to_string(errorId));
            return false;
        }
        
        it->resolved = true;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["error_id"] = std::to_string(errorId);
        metadata["resolution"] = resolution;
        
        logger->logWithMetadata(LogLevel::INFO, "ERROR_RESOLVED", 
                               "Error " + std::to_string(errorId) + " resolved: " + resolution, metadata);
        
        std::cout << "[ERROR_REPORTER] Error " << errorId << " resolved" << std::endl;
        return true;
    }
    
    std::vector<ErrorInfo> getUnresolvedErrors() const {
        std::vector<ErrorInfo> unresolved;
        for (const auto& error : errorHistory) {
            if (!error.resolved) {
                unresolved.push_back(error);
            }
        }
        return unresolved;
    }
    
    std::vector<ErrorInfo> getErrorsByCategory(ErrorCategory category) const {
        std::vector<ErrorInfo> categoryErrors;
        for (const auto& error : errorHistory) {
            if (error.category == category) {
                categoryErrors.push_back(error);
            }
        }
        return categoryErrors;
    }
    
    std::vector<ErrorInfo> getErrorsBySeverity(ErrorSeverity severity) const {
        std::vector<ErrorInfo> severityErrors;
        for (const auto& error : errorHistory) {
            if (error.severity == severity) {
                severityErrors.push_back(error);
            }
        }
        return severityErrors;
    }
    
    void generateErrorReport() const {
        std::ostringstream report;
        
        report << "\n=== Error Report ===\n";
        report << "Total Errors: " << errorHistory.size() << "\n";
        
        // Count by category
        std::unordered_map<ErrorCategory, int> categoryCount;
        std::unordered_map<ErrorSeverity, int> severityCount;
        int unresolvedCount = 0;
        
        for (const auto& error : errorHistory) {
            categoryCount[error.category]++;
            severityCount[error.severity]++;
            if (!error.resolved) unresolvedCount++;
        }
        
        report << "Unresolved Errors: " << unresolvedCount << "\n\n";
        
        report << "Errors by Category:\n";
        for (const auto& [category, count] : categoryCount) {
            report << "  " << categoryNames.at(category) << ": " << count << "\n";
        }
        
        report << "\nErrors by Severity:\n";
        for (const auto& [severity, count] : severityCount) {
            report << "  " << severityNames.at(severity) << ": " << count << "\n";
        }
        
        report << "\nTop Error Codes:\n";
        std::vector<std::pair<std::string, int>> sortedErrors(errorCounts.begin(), errorCounts.end());
        std::sort(sortedErrors.begin(), sortedErrors.end(),
                 [](const auto& a, const auto& b) { return a.second > b.second; });
        
        for (size_t i = 0; i < std::min(size_t(5), sortedErrors.size()); ++i) {
            report << "  " << sortedErrors[i].first << ": " << sortedErrors[i].second << "\n";
        }
        
        logger->info(report.str());
    }
    
    void exportErrorLog(const std::string& filename = "") const {
        std::string logFile = filename.empty() ? 
            "error_log_" + std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".log" :
            filename;
        
        std::ofstream file(logFile);
        if (!file.is_open()) {
            logger->error("Failed to create error log file: " + logFile);
            return;
        }
        
        file << "Detailed Error Log\n";
        file << "==================\n\n";
        
        for (const auto& error : errorHistory) {
            file << "Error ID: " << error.errorId << "\n";
            file << "Timestamp: " << formatTimestamp(error.timestamp) << "\n";
            file << "Category: " << categoryNames.at(error.category) << "\n";
            file << "Severity: " << severityNames.at(error.severity) << "\n";
            file << "Code: " << error.errorCode << "\n";
            file << "Description: " << error.description << "\n";
            file << "Context: " << error.context << "\n";
            file << "Resolved: " << (error.resolved ? "Yes" : "No") << "\n";
            
            if (!error.stackTrace.empty()) {
                file << "Stack Trace:\n" << error.stackTrace << "\n";
            }
            
            file << "---\n\n";
        }
        
        file.close();
        logger->info("Error log exported to: " + logFile);
    }
    
    void clearResolvedErrors() {
        size_t originalSize = errorHistory.size();
        errorHistory.erase(std::remove_if(errorHistory.begin(), errorHistory.end(),
                                         [](const ErrorInfo& error) { return error.resolved; }),
                          errorHistory.end());
        
        size_t removedCount = originalSize - errorHistory.size();
        logger->info("Cleared " + std::to_string(removedCount) + " resolved errors");
    }
    
    void setMaxHistorySize(size_t size) {
        maxHistorySize = size;
        logger->info("Max error history size set to " + std::to_string(size));
    }
    
    void enableStackTracing(bool enable) {
        enableStackTrace = enable;
        logger->info("Stack tracing " + std::string(enable ? "enabled" : "disabled"));
    }
    
private:
    LogLevel severityToLogLevel(ErrorSeverity severity) const {
        switch (severity) {
            case ErrorSeverity::LOW: return LogLevel::WARNING;
            case ErrorSeverity::MEDIUM: return LogLevel::ERROR;
            case ErrorSeverity::HIGH: return LogLevel::ERROR;
            case ErrorSeverity::CRITICAL: return LogLevel::CRITICAL;
            default: return LogLevel::ERROR;
        }
    }
    
    std::string formatErrorMessage(const ErrorInfo& error) const {
        std::ostringstream msg;
        msg << "[" << severityNames.at(error.severity) << "] ";
        msg << categoryNames.at(error.category) << " - ";
        msg << error.errorCode << ": " << error.description;
        if (!error.context.empty()) {
            msg << " (" << error.context << ")";
        }
        return msg.str();
    }
    
    std::string captureStackTrace() const {
        // Simplified stack trace capture
        return "Stack trace capture not implemented in this demo version";
    }
    
    void handleCriticalError(const ErrorInfo& error) {
        logger->critical("CRITICAL ERROR DETECTED - System may be unstable");
        
        // Auto-export critical error details
        std::string criticalLogFile = "critical_error_" + std::to_string(error.errorId) + ".log";
        std::ofstream file(criticalLogFile);
        if (file.is_open()) {
            file << "CRITICAL ERROR REPORT\n";
            file << "=====================\n";
            file << "Error ID: " << error.errorId << "\n";
            file << "Timestamp: " << formatTimestamp(error.timestamp) << "\n";
            file << "Code: " << error.errorCode << "\n";
            file << "Description: " << error.description << "\n";
            file << "Context: " << error.context << "\n";
            if (!error.stackTrace.empty()) {
                file << "Stack Trace:\n" << error.stackTrace << "\n";
            }
            file.close();
        }
        
        if (enableAutoReporting) {
            generateErrorReport();
        }
    }
    
    std::string formatTimestamp(const std::chrono::steady_clock::time_point& timestamp) const {
        auto timeT = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now() + 
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                timestamp - std::chrono::steady_clock::now()));
        
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&timeT), "%Y-%m-%d %H:%M:%S");
        return oss.str();
    }
};

// Global error reporter instance
static std::unique_ptr<ErrorReporter> g_errorReporter;

void initializeErrorReporting(const std::string& logFile) {
    g_errorReporter = std::make_unique<ErrorReporter>(logFile);
}

int reportError(ErrorCategory category, ErrorSeverity severity, const std::string& code, 
               const std::string& description, const std::string& context) {
    if (g_errorReporter) {
        return g_errorReporter->reportError(category, severity, code, description, context);
    }
    return -1;
}

void reportGraphError(const std::string& operation, int nodeId, const std::string& details) {
    if (g_errorReporter) {
        g_errorReporter->reportGraphError(operation, nodeId, details);
    }
}

void reportAlgorithmError(const std::string& algorithm, const std::string& phase, const std::string& details) {
    if (g_errorReporter) {
        g_errorReporter->reportAlgorithmError(algorithm, phase, details);
    }
}

bool resolveError(int errorId, const std::string& resolution) {
    if (g_errorReporter) {
        return g_errorReporter->resolveError(errorId, resolution);
    }
    return false;
}

void generateErrorReport() {
    if (g_errorReporter) {
        g_errorReporter->generateErrorReport();
    }
}