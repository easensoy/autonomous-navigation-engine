#include "reporting/NavigationLogger.hpp"
#include "reporting/PerformanceProfiler.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

enum class NavigationEventType {
    NAVIGATION_START,
    NAVIGATION_END,
    WAYPOINT_REACHED,
    PATH_PLANNED,
    PATH_REPLANNED,
    OBSTACLE_DETECTED,
    EMERGENCY_STOP,
    ROUTE_DEVIATION,
    SPEED_CHANGE,
    ALGORITHM_SWITCH,
    SYSTEM_STATUS_CHANGE
};

struct NavigationEvent {
    int eventId;
    NavigationEventType type;
    std::chrono::steady_clock::time_point timestamp;
    std::string description;
    std::unordered_map<std::string, std::string> data;
    LogLevel severity;
    
    NavigationEvent(int id, NavigationEventType t, const std::string& desc, LogLevel lvl = LogLevel::INFO)
        : eventId(id), type(t), timestamp(std::chrono::steady_clock::now()), 
          description(desc), severity(lvl) {}
};

class NavigationEventLogger {
private:
    std::unique_ptr<NavigationLogger> logger;
    std::unique_ptr<PerformanceProfiler> profiler;
    std::vector<NavigationEvent> eventHistory;
    int nextEventId;
    bool sessionActive;
    std::chrono::steady_clock::time_point sessionStartTime;
    std::string currentSessionId;
    
    std::unordered_map<NavigationEventType, std::string> eventTypeNames = {
        {NavigationEventType::NAVIGATION_START, "NAVIGATION_START"},
        {NavigationEventType::NAVIGATION_END, "NAVIGATION_END"},
        {NavigationEventType::WAYPOINT_REACHED, "WAYPOINT_REACHED"},
        {NavigationEventType::PATH_PLANNED, "PATH_PLANNED"},
        {NavigationEventType::PATH_REPLANNED, "PATH_REPLANNED"},
        {NavigationEventType::OBSTACLE_DETECTED, "OBSTACLE_DETECTED"},
        {NavigationEventType::EMERGENCY_STOP, "EMERGENCY_STOP"},
        {NavigationEventType::ROUTE_DEVIATION, "ROUTE_DEVIATION"},
        {NavigationEventType::SPEED_CHANGE, "SPEED_CHANGE"},
        {NavigationEventType::ALGORITHM_SWITCH, "ALGORITHM_SWITCH"},
        {NavigationEventType::SYSTEM_STATUS_CHANGE, "SYSTEM_STATUS_CHANGE"}
    };
    
public:
    NavigationEventLogger(const std::string& logFile) 
        : logger(std::make_unique<NavigationLogger>(logFile)),
          profiler(std::make_unique<PerformanceProfiler>()),
          nextEventId(1), sessionActive(false) {
        
        logger->setLogLevel(LogLevel::INFO);
        logger->enableConsoleOutput(true);
        
        std::cout << "[NAV_EVENT_LOGGER] Navigation event logger initialized" << std::endl;
    }
    
    void startNavigationSession(int startNode, int goalNode, const std::string& algorithm = "") {
        sessionActive = true;
        sessionStartTime = std::chrono::steady_clock::now();
        currentSessionId = generateSessionId();
        
        NavigationEvent event(nextEventId++, NavigationEventType::NAVIGATION_START, 
                             "Navigation session started");
        event.data["session_id"] = currentSessionId;
        event.data["start_node"] = std::to_string(startNode);
        event.data["goal_node"] = std::to_string(goalNode);
        if (!algorithm.empty()) event.data["algorithm"] = algorithm;
        
        logEvent(event);
        profiler->startProfiling("navigation_session_" + currentSessionId);
        
        std::cout << "[NAV_EVENT_LOGGER] Navigation session " << currentSessionId << " started" << std::endl;
    }
    
    void endNavigationSession(bool successful, const std::string& result = "") {
        if (!sessionActive) return;
        
        NavigationEvent event(nextEventId++, NavigationEventType::NAVIGATION_END,
                             "Navigation session ended");
        event.data["session_id"] = currentSessionId;
        event.data["successful"] = successful ? "true" : "false";
        event.data["result"] = result;
        
        auto sessionDuration = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - sessionStartTime).count();
        event.data["session_duration"] = std::to_string(sessionDuration);
        
        event.severity = successful ? LogLevel::INFO : LogLevel::WARNING;
        
        logEvent(event);
        profiler->stopProfiling();
        
        generateSessionSummary();
        sessionActive = false;
        
        std::cout << "[NAV_EVENT_LOGGER] Navigation session " << currentSessionId 
                  << " ended (" << (successful ? "success" : "failure") << ")" << std::endl;
    }
    
    void logWaypointReached(int nodeId, double currentSpeed = -1, double distanceTraveled = -1) {
        NavigationEvent event(nextEventId++, NavigationEventType::WAYPOINT_REACHED,
                             "Waypoint reached: " + std::to_string(nodeId));
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["node_id"] = std::to_string(nodeId);
        if (currentSpeed >= 0) event.data["current_speed"] = std::to_string(currentSpeed);
        if (distanceTraveled >= 0) event.data["distance_traveled"] = std::to_string(distanceTraveled);
        
        auto sessionTime = sessionActive ? 
            std::chrono::duration<double>(std::chrono::steady_clock::now() - sessionStartTime).count() : 0.0;
        event.data["session_time"] = std::to_string(sessionTime);
        
        logEvent(event);
    }
    
    void logPathPlanned(const std::vector<int>& path, const std::string& algorithm, 
                       double planningTime, double pathCost = -1) {
        NavigationEvent event(nextEventId++, NavigationEventType::PATH_PLANNED,
                             "Path planned using " + algorithm);
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["algorithm"] = algorithm;
        event.data["path_length"] = std::to_string(path.size());
        event.data["planning_time"] = std::to_string(planningTime);
        if (pathCost >= 0) event.data["path_cost"] = std::to_string(pathCost);
        
        // Store path (truncated if too long)
        std::ostringstream pathStr;
        size_t maxNodes = 20;
        for (size_t i = 0; i < std::min(path.size(), maxNodes); ++i) {
            if (i > 0) pathStr << "->";
            pathStr << path[i];
        }
        if (path.size() > maxNodes) pathStr << "...";
        event.data["path"] = pathStr.str();
        
        logEvent(event);
        profiler->recordMetric("path_planning_" + algorithm, planningTime);
    }
    
    void logPathReplanned(const std::vector<int>& newPath, const std::string& reason,
                         const std::string& algorithm, double replanningTime) {
        NavigationEvent event(nextEventId++, NavigationEventType::PATH_REPLANNED,
                             "Path replanned: " + reason);
        event.severity = LogLevel::WARNING;
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["reason"] = reason;
        event.data["algorithm"] = algorithm;
        event.data["new_path_length"] = std::to_string(newPath.size());
        event.data["replanning_time"] = std::to_string(replanningTime);
        
        logEvent(event);
        profiler->recordMetric("path_replanning", replanningTime);
    }
    
    void logObstacleDetected(int nodeId, const std::string& obstacleType, 
                            double severity = 1.0, const std::string& action = "") {
        NavigationEvent event(nextEventId++, NavigationEventType::OBSTACLE_DETECTED,
                             "Obstacle detected at node " + std::to_string(nodeId));
        event.severity = severity > 0.7 ? LogLevel::WARNING : LogLevel::INFO;
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["node_id"] = std::to_string(nodeId);
        event.data["obstacle_type"] = obstacleType;
        event.data["severity"] = std::to_string(severity);
        if (!action.empty()) event.data["action_taken"] = action;
        
        logEvent(event);
    }
    
    void logEmergencyStop(const std::string& reason, int currentNode = -1) {
        NavigationEvent event(nextEventId++, NavigationEventType::EMERGENCY_STOP,
                             "Emergency stop: " + reason);
        event.severity = LogLevel::CRITICAL;
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["reason"] = reason;
        if (currentNode >= 0) event.data["current_node"] = std::to_string(currentNode);
        
        auto sessionTime = sessionActive ? 
            std::chrono::duration<double>(std::chrono::steady_clock::now() - sessionStartTime).count() : 0.0;
        event.data["session_time"] = std::to_string(sessionTime);
        
        logEvent(event);
    }
    
    void logRouteDeviation(int expectedNode, int actualNode, double deviation) {
        NavigationEvent event(nextEventId++, NavigationEventType::ROUTE_DEVIATION,
                             "Route deviation detected");
        event.severity = deviation > 5.0 ? LogLevel::WARNING : LogLevel::INFO;
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["expected_node"] = std::to_string(expectedNode);
        event.data["actual_node"] = std::to_string(actualNode);
        event.data["deviation"] = std::to_string(deviation);
        
        logEvent(event);
    }
    
    void logSpeedChange(double oldSpeed, double newSpeed, const std::string& reason = "") {
        NavigationEvent event(nextEventId++, NavigationEventType::SPEED_CHANGE,
                             "Speed changed from " + std::to_string(oldSpeed) + 
                             " to " + std::to_string(newSpeed));
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["old_speed"] = std::to_string(oldSpeed);
        event.data["new_speed"] = std::to_string(newSpeed);
        event.data["speed_change"] = std::to_string(newSpeed - oldSpeed);
        if (!reason.empty()) event.data["reason"] = reason;
        
        logEvent(event);
    }
    
    void logAlgorithmSwitch(const std::string& fromAlgorithm, const std::string& toAlgorithm,
                           const std::string& reason) {
        NavigationEvent event(nextEventId++, NavigationEventType::ALGORITHM_SWITCH,
                             "Algorithm switched from " + fromAlgorithm + " to " + toAlgorithm);
        event.severity = LogLevel::WARNING;
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["from_algorithm"] = fromAlgorithm;
        event.data["to_algorithm"] = toAlgorithm;
        event.data["reason"] = reason;
        
        logEvent(event);
    }
    
    void logSystemStatusChange(const std::string& component, const std::string& oldStatus,
                              const std::string& newStatus, const std::string& reason = "") {
        NavigationEvent event(nextEventId++, NavigationEventType::SYSTEM_STATUS_CHANGE,
                             component + " status changed: " + oldStatus + " -> " + newStatus);
        
        if (sessionActive) event.data["session_id"] = currentSessionId;
        event.data["component"] = component;
        event.data["old_status"] = oldStatus;
        event.data["new_status"] = newStatus;
        if (!reason.empty()) event.data["reason"] = reason;
        
        // Determine severity based on status change
        if (newStatus == "ERROR" || newStatus == "FAILED") {
            event.severity = LogLevel::ERROR;
        } else if (newStatus == "WARNING" || newStatus == "DEGRADED") {
            event.severity = LogLevel::WARNING;
        }
        
        logEvent(event);
    }
    
    void generateSessionSummary() {
        if (!sessionActive && currentSessionId.empty()) return;
        
        std::vector<NavigationEvent> sessionEvents;
        for (const auto& event : eventHistory) {
            auto sessionIt = event.data.find("session_id");
            if (sessionIt != event.data.end() && sessionIt->second == currentSessionId) {
                sessionEvents.push_back(event);
            }
        }
        
        if (sessionEvents.empty()) return;
        
        std::ostringstream summary;
        summary << "\n=== Navigation Session Summary ===\n";
        summary << "Session ID: " << currentSessionId << "\n";
        summary << "Total Events: " << sessionEvents.size() << "\n";
        
        auto sessionDuration = std::chrono::duration<double>(
            sessionEvents.back().timestamp - sessionEvents.front().timestamp).count();
        summary << "Session Duration: " << sessionDuration << "s\n";
        
        // Count events by type
        std::unordered_map<NavigationEventType, int> eventCounts;
        for (const auto& event : sessionEvents) {
            eventCounts[event.type]++;
        }
        
        summary << "Event Breakdown:\n";
        for (const auto& [type, count] : eventCounts) {
            summary << "  " << eventTypeNames[type] << ": " << count << "\n";
        }
        
        // Performance metrics
        PerformanceMetrics metrics = profiler->getMetrics("navigation_session_" + currentSessionId);
        summary << "Performance:\n";
        summary << "  Execution Time: " << metrics.executionTime.count() << "s\n";
        summary << "  Memory Used: " << metrics.memoryUsed << " bytes\n";
        
        logger->info(summary.str());
        exportSessionLog();
    }
    
    void exportSessionLog() {
        std::string filename = "navigation_session_" + currentSessionId + ".log";
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            logger->error("Failed to create session log file: " + filename);
            return;
        }
        
        file << "Navigation Session Log\n";
        file << "======================\n";
        file << "Session ID: " << currentSessionId << "\n\n";
        
        for (const auto& event : eventHistory) {
            auto sessionIt = event.data.find("session_id");
            if (sessionIt != event.data.end() && sessionIt->second == currentSessionId) {
                file << "[" << formatTimestamp(event.timestamp) << "] ";
                file << eventTypeNames[event.type] << ": " << event.description << "\n";
                
                for (const auto& [key, value] : event.data) {
                    if (key != "session_id") {
                        file << "  " << key << ": " << value << "\n";
                    }
                }
                file << "\n";
            }
        }
        
        file.close();
        logger->info("Session log exported to: " + filename);
    }
    
    std::vector<NavigationEvent> getEventsByType(NavigationEventType type) const {
        std::vector<NavigationEvent> typeEvents;
        for (const auto& event : eventHistory) {
            if (event.type == type) {
                typeEvents.push_back(event);
            }
        }
        return typeEvents;
    }
    
    std::vector<NavigationEvent> getSessionEvents(const std::string& sessionId) const {
        std::vector<NavigationEvent> sessionEvents;
        for (const auto& event : eventHistory) {
            auto sessionIt = event.data.find("session_id");
            if (sessionIt != event.data.end() && sessionIt->second == sessionId) {
                sessionEvents.push_back(event);
            }
        }
        return sessionEvents;
    }
    
    void setLogLevel(LogLevel level) {
        logger->setLogLevel(level);
    }
    
    void enableConsoleOutput(bool enable) {
        logger->enableConsoleOutput(enable);
    }
    
private:
    void logEvent(const NavigationEvent& event) {
        logger->logWithMetadata(event.severity, eventTypeNames[event.type], 
                               event.description, event.data);
        eventHistory.push_back(event);
        
        // Maintain history size
        if (eventHistory.size() > 1000) {
            eventHistory.erase(eventHistory.begin());
        }
    }
    
    std::string generateSessionId() {
        auto now = std::chrono::system_clock::now();
        auto timeT = std::chrono::system_clock::to_time_t(now);
        
        std::ostringstream oss;
        oss << "NAV_" << std::put_time(std::localtime(&timeT), "%Y%m%d_%H%M%S");
        return oss.str();
    }
    
    std::string formatTimestamp(const std::chrono::steady_clock::time_point& timestamp) const {
        auto timeT = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now() + 
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                timestamp - std::chrono::steady_clock::now()));
        
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&timeT), "%H:%M:%S");
        return oss.str();
    }
};

// Global navigation event logger instance
static std::unique_ptr<NavigationEventLogger> g_navEventLogger;

void initializeNavigationEventLogger(const std::string& logFile) {
    g_navEventLogger = std::make_unique<NavigationEventLogger>(logFile);
}

void startNavigationSession(int start, int goal, const std::string& algorithm) {
    if (g_navEventLogger) {
        g_navEventLogger->startNavigationSession(start, goal, algorithm);
    }
}

void endNavigationSession(bool successful, const std::string& result) {
    if (g_navEventLogger) {
        g_navEventLogger->endNavigationSession(successful, result);
    }
}

void logWaypointReached(int nodeId, double speed, double distance) {
    if (g_navEventLogger) {
        g_navEventLogger->logWaypointReached(nodeId, speed, distance);
    }
}

void logPathPlanned(const std::vector<int>& path, const std::string& algorithm, double time, double cost) {
    if (g_navEventLogger) {
        g_navEventLogger->logPathPlanned(path, algorithm, time, cost);
    }
}

void logObstacleDetected(int nodeId, const std::string& type, double severity, const std::string& action) {
    if (g_navEventLogger) {
        g_navEventLogger->logObstacleDetected(nodeId, type, severity, action);
    }
}

void logEmergencyStop(const std::string& reason, int currentNode) {
    if (g_navEventLogger) {
        g_navEventLogger->logEmergencyStop(reason, currentNode);
    }
}