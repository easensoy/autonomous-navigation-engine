#include "reporting/NavigationLogger.hpp"
#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <unordered_map>

class AlgorithmStepLogger {
private:
    std::unique_ptr<NavigationLogger> logger;
    std::string currentAlgorithm;
    std::chrono::steady_clock::time_point algorithmStartTime;
    int stepCounter;
    bool detailedLogging;
    
    struct StepInfo {
        int stepNumber;
        std::string description;
        std::chrono::steady_clock::time_point timestamp;
        std::unordered_map<std::string, std::string> data;
        LogLevel level;
        
        StepInfo(int num, const std::string& desc, LogLevel lvl = LogLevel::INFO)
            : stepNumber(num), description(desc), timestamp(std::chrono::steady_clock::now()), level(lvl) {}
    };
    
    std::vector<StepInfo> currentSession;
    
public:
    AlgorithmStepLogger(const std::string& logFile) 
        : logger(std::make_unique<NavigationLogger>(logFile)), 
          stepCounter(0), detailedLogging(true) {
        
        logger->setLogLevel(LogLevel::DEBUG);
        logger->enableConsoleOutput(true);
        
        std::cout << "[ALGO_LOGGER] Algorithm step logger initialized" << std::endl;
    }
    
    void startAlgorithm(const std::string& algorithmName, int startNode = -1, int goalNode = -1) {
        currentAlgorithm = algorithmName;
        algorithmStartTime = std::chrono::steady_clock::now();
        stepCounter = 0;
        currentSession.clear();
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = algorithmName;
        if (startNode != -1) metadata["start_node"] = std::to_string(startNode);
        if (goalNode != -1) metadata["goal_node"] = std::to_string(goalNode);
        
        logger->logWithMetadata(LogLevel::INFO, "ALGORITHM_START", 
                               "Starting " + algorithmName + " execution", metadata);
        
        StepInfo startStep(0, "Algorithm initialization", LogLevel::INFO);
        startStep.data = metadata;
        currentSession.push_back(startStep);
    }
    
    void logStep(const std::string& stepDescription, LogLevel level = LogLevel::DEBUG) {
        stepCounter++;
        
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration<double>(now - algorithmStartTime).count();
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["elapsed_time"] = std::to_string(elapsed);
        
        std::string logMessage = "Step " + std::to_string(stepCounter) + ": " + stepDescription;
        logger->logWithMetadata(level, "ALGORITHM_STEP", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, stepDescription, level);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void logNodeExploration(int nodeId, double gScore = -1, double fScore = -1, int parent = -1) {
        stepCounter++;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["node_id"] = std::to_string(nodeId);
        metadata["operation"] = "explore_node";
        
        if (gScore >= 0) metadata["g_score"] = std::to_string(gScore);
        if (fScore >= 0) metadata["f_score"] = std::to_string(fScore);
        if (parent >= 0) metadata["parent"] = std::to_string(parent);
        
        std::string logMessage = "Exploring node " + std::to_string(nodeId);
        if (gScore >= 0) logMessage += " (g=" + std::to_string(gScore) + ")";
        if (fScore >= 0) logMessage += " (f=" + std::to_string(fScore) + ")";
        
        logger->logWithMetadata(LogLevel::DEBUG, "NODE_EXPLORATION", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, "Node exploration: " + std::to_string(nodeId), LogLevel::DEBUG);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void logEdgeRelaxation(int fromNode, int toNode, double oldDistance, double newDistance, bool improved) {
        stepCounter++;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["from_node"] = std::to_string(fromNode);
        metadata["to_node"] = std::to_string(toNode);
        metadata["old_distance"] = std::to_string(oldDistance);
        metadata["new_distance"] = std::to_string(newDistance);
        metadata["improved"] = improved ? "true" : "false";
        metadata["operation"] = "edge_relaxation";
        
        std::string logMessage = "Edge relaxation " + std::to_string(fromNode) + "->" + std::to_string(toNode) + 
                                ": " + std::to_string(oldDistance) + " -> " + std::to_string(newDistance) + 
                                (improved ? " (improved)" : " (no change)");
        
        LogLevel level = improved ? LogLevel::DEBUG : LogLevel::DEBUG;
        logger->logWithMetadata(level, "EDGE_RELAXATION", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, logMessage, level);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void logQueueOperation(const std::string& operation, int nodeId, double priority = -1, size_t queueSize = 0) {
        stepCounter++;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["queue_operation"] = operation;
        metadata["node_id"] = std::to_string(nodeId);
        metadata["queue_size"] = std::to_string(queueSize);
        metadata["operation"] = "queue_operation";
        
        if (priority >= 0) metadata["priority"] = std::to_string(priority);
        
        std::string logMessage = "Queue " + operation + ": node " + std::to_string(nodeId);
        if (priority >= 0) logMessage += " (priority=" + std::to_string(priority) + ")";
        logMessage += " [queue size: " + std::to_string(queueSize) + "]";
        
        logger->logWithMetadata(LogLevel::DEBUG, "QUEUE_OPERATION", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, logMessage, LogLevel::DEBUG);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void logPathConstruction(const std::vector<int>& path, const std::string& stage = "final") {
        stepCounter++;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["operation"] = "path_construction";
        metadata["stage"] = stage;
        metadata["path_length"] = std::to_string(path.size());
        
        std::ostringstream pathStr;
        for (size_t i = 0; i < path.size(); ++i) {
            if (i > 0) pathStr << "->";
            pathStr << path[i];
        }
        metadata["path"] = pathStr.str();
        
        std::string logMessage = "Path construction (" + stage + "): " + pathStr.str() + 
                                " [length: " + std::to_string(path.size()) + "]";
        
        logger->logWithMetadata(LogLevel::INFO, "PATH_CONSTRUCTION", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, logMessage, LogLevel::INFO);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void logAlgorithmDecision(const std::string& decision, const std::string& reasoning) {
        stepCounter++;
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["step_number"] = std::to_string(stepCounter);
        metadata["decision"] = decision;
        metadata["reasoning"] = reasoning;
        metadata["operation"] = "algorithm_decision";
        
        std::string logMessage = "Decision: " + decision + " (Reason: " + reasoning + ")";
        
        logger->logWithMetadata(LogLevel::INFO, "ALGORITHM_DECISION", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo step(stepCounter, logMessage, LogLevel::INFO);
            step.data = metadata;
            currentSession.push_back(step);
        }
    }
    
    void endAlgorithm(bool successful, const std::string& result = "") {
        auto endTime = std::chrono::steady_clock::now();
        auto totalTime = std::chrono::duration<double>(endTime - algorithmStartTime).count();
        
        std::unordered_map<std::string, std::string> metadata;
        metadata["algorithm"] = currentAlgorithm;
        metadata["successful"] = successful ? "true" : "false";
        metadata["total_steps"] = std::to_string(stepCounter);
        metadata["execution_time"] = std::to_string(totalTime);
        metadata["result"] = result;
        
        LogLevel level = successful ? LogLevel::INFO : LogLevel::WARNING;
        std::string logMessage = currentAlgorithm + " completed " + 
                               (successful ? "successfully" : "with errors") + 
                               " in " + std::to_string(totalTime) + "s (" + 
                               std::to_string(stepCounter) + " steps)";
        
        if (!result.empty()) {
            logMessage += " - " + result;
        }
        
        logger->logWithMetadata(level, "ALGORITHM_END", logMessage, metadata);
        
        if (detailedLogging) {
            StepInfo endStep(stepCounter + 1, "Algorithm completion", level);
            endStep.data = metadata;
            currentSession.push_back(endStep);
        }
        
        generateSessionSummary();
    }
    
    void generateSessionSummary() {
        if (currentSession.empty()) return;
        
        std::ostringstream summary;
        summary << "\n=== Algorithm Session Summary ===\n";
        summary << "Algorithm: " << currentAlgorithm << "\n";
        summary << "Total Steps: " << stepCounter << "\n";
        
        auto totalTime = std::chrono::duration<double>(
            currentSession.back().timestamp - currentSession.front().timestamp).count();
        summary << "Total Time: " << totalTime << "s\n";
        
        // Count operations by type
        std::unordered_map<std::string, int> operationCounts;
        for (const auto& step : currentSession) {
            auto opIt = step.data.find("operation");
            std::string operation = (opIt != step.data.end()) ? opIt->second : "general";
            operationCounts[operation]++;
        }
        
        summary << "Operations:\n";
        for (const auto& [operation, count] : operationCounts) {
            summary << "  " << operation << ": " << count << "\n";
        }
        
        logger->info(summary.str());
        
        if (detailedLogging) {
            exportDetailedLog();
        }
    }
    
    void exportDetailedLog() {
        std::string filename = "algorithm_session_" + currentAlgorithm + "_" + 
                              std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                              ".log";
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            logger->error("Failed to create detailed log file: " + filename);
            return;
        }
        
        file << "Detailed Algorithm Execution Log\n";
        file << "================================\n";
        file << "Algorithm: " << currentAlgorithm << "\n";
        file << "Start Time: " << formatTimestamp(currentSession.front().timestamp) << "\n";
        file << "End Time: " << formatTimestamp(currentSession.back().timestamp) << "\n\n";
        
        for (const auto& step : currentSession) {
            file << "[" << formatTimestamp(step.timestamp) << "] ";
            file << "Step " << step.stepNumber << ": " << step.description << "\n";
            
            for (const auto& [key, value] : step.data) {
                file << "  " << key << ": " << value << "\n";
            }
            file << "\n";
        }
        
        file.close();
        logger->info("Detailed log exported to: " + filename);
    }
    
    void setDetailedLogging(bool enabled) {
        detailedLogging = enabled;
        logger->info("Detailed logging " + std::string(enabled ? "enabled" : "disabled"));
    }
    
    void setLogLevel(LogLevel level) {
        logger->setLogLevel(level);
    }
    
    void enableConsoleOutput(bool enable) {
        logger->enableConsoleOutput(enable);
    }
    
private:
    std::string formatTimestamp(const std::chrono::steady_clock::time_point& timestamp) {
        auto timeT = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now() + 
            std::chrono::duration_cast<std::chrono::system_clock::duration>(
                timestamp - std::chrono::steady_clock::now()));
        
        std::ostringstream oss;
        oss << std::put_time(std::localtime(&timeT), "%H:%M:%S");
        return oss.str();
    }
};

// Global algorithm logger instance
static std::unique_ptr<AlgorithmStepLogger> g_algorithmLogger;

void initializeAlgorithmLogger(const std::string& logFile) {
    g_algorithmLogger = std::make_unique<AlgorithmStepLogger>(logFile);
}

void logAlgorithmStart(const std::string& algorithm, int start, int goal) {
    if (g_algorithmLogger) {
        g_algorithmLogger->startAlgorithm(algorithm, start, goal);
    }
}

void logAlgorithmStep(const std::string& description, LogLevel level) {
    if (g_algorithmLogger) {
        g_algorithmLogger->logStep(description, level);
    }
}

void logNodeExploration(int nodeId, double gScore, double fScore, int parent) {
    if (g_algorithmLogger) {
        g_algorithmLogger->logNodeExploration(nodeId, gScore, fScore, parent);
    }
}

void logEdgeRelaxation(int from, int to, double oldDist, double newDist, bool improved) {
    if (g_algorithmLogger) {
        g_algorithmLogger->logEdgeRelaxation(from, to, oldDist, newDist, improved);
    }
}

void logAlgorithmEnd(bool successful, const std::string& result) {
    if (g_algorithmLogger) {
        g_algorithmLogger->endAlgorithm(successful, result);
    }
}