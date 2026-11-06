#include "reporting/NavigationLogger.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>

// NavigationLogger: levelled log written to a file, kept in memory, and optionally echoed to the console.

namespace {
// Shared start time so every logger reports seconds since program start.
const std::chrono::steady_clock::time_point kProcessStart = std::chrono::steady_clock::now();

std::string jsonEscape(const std::string& text) {
    std::string out;
    for (char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default: out += c;
        }
    }
    return out;
}
}  // namespace

NavigationLogger::NavigationLogger(const std::string& filename)
    : logFilename(filename), currentLogLevel(LogLevel::INFO), consoleOutput(true) {
    if (!logFilename.empty()) {
        logFile.open(logFilename, std::ios::app);
        if (!logFile) {
            std::cerr << "[NavigationLogger] Could not open log file " << logFilename
                      << "; logging to memory and console only" << std::endl;
        }
    }
}

NavigationLogger::~NavigationLogger() {
    flushLogs();
    if (logFile.is_open()) {
        logFile.close();
    }
}

void NavigationLogger::log(LogLevel level, const std::string& operation, const std::string& message) {
    logWithMetadata(level, operation, message, {});
}

void NavigationLogger::logWithMetadata(LogLevel level, const std::string& operation, const std::string& message,
                                       const std::unordered_map<std::string, std::string>& metadata) {
    if (level < currentLogLevel) {
        return;
    }
    LogEntry entry{std::chrono::steady_clock::now(), level, operation, message, metadata};
    const std::string line = formatLogEntry(entry);
    logBuffer.push_back(std::move(entry));
    if (logFile.is_open()) {
        logFile << line << '\n';
    }
    if (consoleOutput) {
        (level >= LogLevel::ERROR ? std::cerr : std::cout) << line << std::endl;
    }
}

void NavigationLogger::debug(const std::string& message) { log(LogLevel::DEBUG, "general", message); }
void NavigationLogger::info(const std::string& message) { log(LogLevel::INFO, "general", message); }
void NavigationLogger::warning(const std::string& message) { log(LogLevel::WARNING, "general", message); }
void NavigationLogger::error(const std::string& message) { log(LogLevel::ERROR, "general", message); }
void NavigationLogger::critical(const std::string& message) { log(LogLevel::CRITICAL, "general", message); }

void NavigationLogger::setLogLevel(LogLevel level) {
    currentLogLevel = level;
}

void NavigationLogger::enableConsoleOutput(bool enable) {
    consoleOutput = enable;
}

void NavigationLogger::flushLogs() {
    if (logFile.is_open()) {
        logFile.flush();
    }
}

std::vector<LogEntry> NavigationLogger::getRecentLogs(std::chrono::duration<double> timeWindow) const {
    const auto cutoff = std::chrono::steady_clock::now() -
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeWindow);
    std::vector<LogEntry> recent;
    for (const LogEntry& entry : logBuffer) {
        if (entry.timestamp >= cutoff) {
            recent.push_back(entry);
        }
    }
    return recent;
}

std::vector<LogEntry> NavigationLogger::getLogsByLevel(LogLevel level) const {
    std::vector<LogEntry> matching;
    std::copy_if(logBuffer.begin(), logBuffer.end(), std::back_inserter(matching),
                 [level](const LogEntry& entry) { return entry.level == level; });
    return matching;
}

void NavigationLogger::clearLogs() {
    logBuffer.clear();
}

void NavigationLogger::exportLogsToJSON(const std::string& filename) const {
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    out << "[\n";
    for (size_t i = 0; i < logBuffer.size(); ++i) {
        const LogEntry& entry = logBuffer[i];
        const double seconds = std::chrono::duration<double>(entry.timestamp - kProcessStart).count();
        out << "  {\"time\": " << seconds << ", \"level\": \"" << logLevelToString(entry.level)
            << "\", \"operation\": \"" << jsonEscape(entry.operation) << "\", \"message\": \""
            << jsonEscape(entry.message) << "\"";
        if (!entry.metadata.empty()) {
            out << ", \"metadata\": {";
            bool first = true;
            for (const auto& kv : entry.metadata) {
                out << (first ? "" : ", ") << "\"" << jsonEscape(kv.first) << "\": \"" << jsonEscape(kv.second) << "\"";
                first = false;
            }
            out << "}";
        }
        out << "}" << (i + 1 < logBuffer.size() ? "," : "") << "\n";
    }
    out << "]\n";
}

void NavigationLogger::generateLogSummary() const {
    std::map<LogLevel, size_t> counts;
    for (const LogEntry& entry : logBuffer) {
        counts[entry.level]++;
    }
    std::cout << "[NavigationLogger] " << logBuffer.size() << " entries";
    for (const auto& kv : counts) {
        std::cout << ", " << logLevelToString(kv.first) << ": " << kv.second;
    }
    std::cout << std::endl;
}

void NavigationLogger::archiveLogs(const std::string& archiveFilename) {
    std::ofstream out(archiveFilename, std::ios::app);
    if (!out) {
        throw std::runtime_error("Could not open archive file: " + archiveFilename);
    }
    for (const LogEntry& entry : logBuffer) {
        out << formatLogEntry(entry) << '\n';
    }
    logBuffer.clear();
}

std::string NavigationLogger::formatLogEntry(const LogEntry& entry) const {
    std::ostringstream line;
    const double seconds = std::chrono::duration<double>(entry.timestamp - kProcessStart).count();
    line << "[+" << std::fixed << std::setprecision(3) << seconds << "s] [" << logLevelToString(entry.level) << "] ["
         << entry.operation << "] " << entry.message;
    for (const auto& kv : entry.metadata) {
        line << " " << kv.first << "=" << kv.second;
    }
    return line.str();
}

std::string NavigationLogger::logLevelToString(LogLevel level) const {
    switch (level) {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO: return "INFO";
        case LogLevel::WARNING: return "WARNING";
        case LogLevel::ERROR: return "ERROR";
        case LogLevel::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}
