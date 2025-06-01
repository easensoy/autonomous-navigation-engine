#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <chrono>

enum class LogLevel {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    CRITICAL
};

struct LogEntry {
    std::chrono::steady_clock::time_point timestamp;
    LogLevel level;
    std::string operation;
    std::string message;
    std::unordered_map<std::string, std::string> metadata;
};

class NavigationLogger {
private:
    std::string logFilename;
    std::ofstream logFile;
    LogLevel currentLogLevel;
    bool consoleOutput;
    std::vector<LogEntry> logBuffer;
    
    std::string formatLogEntry(const LogEntry& entry) const;
    std::string logLevelToString(LogLevel level) const;

public:
    explicit NavigationLogger(const std::string& filename);
    ~NavigationLogger();
    
    void log(LogLevel level, const std::string& operation, const std::string& message);
    void logWithMetadata(LogLevel level, const std::string& operation, const std::string& message, const std::unordered_map<std::string, std::string>& metadata);
    
    void debug(const std::string& message);
    void info(const std::string& message);
    void warning(const std::string& message);
    void error(const std::string& message);
    void critical(const std::string& message);
    
    void setLogLevel(LogLevel level);
    void enableConsoleOutput(bool enable);
    void flushLogs();
    
    std::vector<LogEntry> getRecentLogs(std::chrono::duration<double> timeWindow) const;
    std::vector<LogEntry> getLogsByLevel(LogLevel level) const;
    void clearLogs();
    
    void exportLogsToJSON(const std::string& filename) const;
    void generateLogSummary() const;
    void archiveLogs(const std::string& archiveFilename);
};