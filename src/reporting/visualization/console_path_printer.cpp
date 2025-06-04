#include "reporting/PathVisualiser.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

class ConsolePathPrinter {
private:
    const Graph* graph;
    bool enableColors;
    bool showCoordinates;
    bool showDistances;
    bool showNodeDetails;
    
    // ANSI color codes
    const std::string COLOR_RESET = "\033[0m";
    const std::string COLOR_GREEN = "\033[32m";   // Start node
    const std::string COLOR_RED = "\033[31m";     // End node
    const std::string COLOR_YELLOW = "\033[33m";  // Path nodes
    const std::string COLOR_BLUE = "\033[34m";    // Distances
    const std::string COLOR_CYAN = "\033[36m";    // Coordinates
    const std::string COLOR_BOLD = "\033[1m";     // Bold text
    
public:
    ConsolePathPrinter(const Graph* environment) 
        : graph(environment), enableColors(true), showCoordinates(true), 
          showDistances(true), showNodeDetails(false) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[CONSOLE_PRINTER] Console path printer initialized" << std::endl;
    }
    
    void printPath(const std::vector<int>& path) {
        if (path.empty()) {
            std::cout << "Empty path - nothing to display." << std::endl;
            return;
        }
        
        std::cout << "[CONSOLE_PRINTER] Printing path with " << path.size() << " nodes" << std::endl;
        
        printPathHeader(path);
        printPathSteps(path);
        printPathSummary(path);
    }
    
    void printDetailedPath(const std::vector<int>& path) {
        showNodeDetails = true;
        printPath(path);
        showNodeDetails = false;
    }
    
    void printPathComparison(const std::vector<int>& path1, const std::vector<int>& path2,
                            const std::string& label1 = "Path 1", const std::string& label2 = "Path 2") {
        
        std::cout << "\n" << formatBold("=== PATH COMPARISON ===") << std::endl;
        
        std::cout << "\n" << formatBold(label1 + ":") << std::endl;
        printPathSummaryOnly(path1);
        
        std::cout << "\n" << formatBold(label2 + ":") << std::endl;
        printPathSummaryOnly(path2);
        
        // Comparison metrics
        std::cout << "\n" << formatBold("Comparison:") << std::endl;
        
        double length1 = calculatePathLength(path1);
        double length2 = calculatePathLength(path2);
        
        std::cout << "  Length difference: " << formatDistance(std::abs(length1 - length2)) << std::endl;
        std::cout << "  Node count difference: " << std::abs(static_cast<int>(path1.size() - path2.size())) << std::endl;
        
        if (length1 > 0 && length2 > 0) {
            double efficiencyRatio = std::min(length1, length2) / std::max(length1, length2) * 100.0;
            std::cout << "  Efficiency ratio: " << std::fixed << std::setprecision(1) 
                      << efficiencyRatio << "%" << std::endl;
        }
        
        // Find common nodes
        std::vector<int> commonNodes;
        for (int node : path1) {
            if (std::find(path2.begin(), path2.end(), node) != path2.end()) {
                commonNodes.push_back(node);
            }
        }
        std::cout << "  Common nodes: " << commonNodes.size() << std::endl;
    }
    
    void printMultiplePaths(const std::vector<std::vector<int>>& paths, 
                           const std::vector<std::string>& labels = {}) {
        
        std::cout << "\n" << formatBold("=== MULTIPLE PATHS ===") << std::endl;
        
        for (size_t i = 0; i < paths.size(); ++i) {
            std::string label = (i < labels.size()) ? labels[i] : ("Path " + std::to_string(i + 1));
            
            std::cout << "\n" << formatBold(label + ":") << std::endl;
            printPathSummaryOnly(paths[i]);
        }
        
        if (paths.size() > 1) {
            printPathsComparison(paths, labels);
        }
    }
    
    void printNodeDetails(int nodeId) {
        if (!graph->hasNode(nodeId)) {
            std::cout << "Node " << nodeId << " not found in graph." << std::endl;
            return;
        }
        
        const Node& node = graph->getNode(nodeId);
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        
        std::cout << formatBold("Node Details:") << std::endl;
        std::cout << "  ID: " << formatNodeId(nodeId) << std::endl;
        std::cout << "  Name: " << node.getName() << std::endl;
        std::cout << "  Coordinates: " << formatCoordinates(node.getX(), node.getY()) << std::endl;
        std::cout << "  Degree: " << neighbors.size() << std::endl;
        
        if (!neighbors.empty()) {
            std::cout << "  Neighbors: ";
            for (size_t i = 0; i < neighbors.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << formatNodeId(neighbors[i]);
                
                if (showDistances) {
                    const Node& neighbor = graph->getNode(neighbors[i]);
                    double distance = node.euclideanDistance(neighbor);
                    std::cout << " (" << formatDistance(distance) << ")";
                }
            }
            std::cout << std::endl;
        }
    }
    
    void setColorEnabled(bool enabled) {
        enableColors = enabled;
        std::cout << "[CONSOLE_PRINTER] Colors " << (enabled ? "enabled" : "disabled") << std::endl;
    }
    
    void setShowCoordinates(bool show) {
        showCoordinates = show;
    }
    
    void setShowDistances(bool show) {
        showDistances = show;
    }
    
private:
    void printPathHeader(const std::vector<int>& path) {
        std::cout << "\n" << formatBold("=== PATH VISUALIZATION ===") << std::endl;
        std::cout << "Path length: " << path.size() << " nodes" << std::endl;
        
        if (path.size() >= 2) {
            std::cout << "Start: " << formatStartNode(path.front()) << std::endl;
            std::cout << "Goal:  " << formatEndNode(path.back()) << std::endl;
        }
        
        double totalDistance = calculatePathLength(path);
        if (totalDistance > 0) {
            std::cout << "Total distance: " << formatDistance(totalDistance) << std::endl;
        }
        
        std::cout << std::endl;
    }
    
    void printPathSteps(const std::vector<int>& path) {
        std::cout << formatBold("Path Steps:") << std::endl;
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) {
                std::cout << std::setw(3) << (i + 1) << ". " << formatError("Node " + std::to_string(path[i]) + " [NOT FOUND]") << std::endl;
                continue;
            }
            
            const Node& node = graph->getNode(path[i]);
            
            std::ostringstream step;
            step << std::setw(3) << (i + 1) << ". ";
            
            // Node information
            if (i == 0) {
                step << formatStartNode(path[i]);
            } else if (i == path.size() - 1) {
                step << formatEndNode(path[i]);
            } else {
                step << formatPathNode(path[i]);
            }
            
            if (showCoordinates) {
                step << " " << formatCoordinates(node.getX(), node.getY());
            }
            
            // Distance to next node
            if (i < path.size() - 1 && showDistances) {
                if (graph->hasNode(path[i + 1])) {
                    const Node& nextNode = graph->getNode(path[i + 1]);
                    double distance = node.euclideanDistance(nextNode);
                    step << " → " << formatDistance(distance);
                }
            }
            
            std::cout << step.str() << std::endl;
            
            // Additional node details if enabled
            if (showNodeDetails) {
                std::vector<int> neighbors = graph->getNeighbors(path[i]);
                std::cout << "     └─ " << neighbors.size() << " connections";
                if (!node.getName().empty() && node.getName() != ("node_" + std::to_string(path[i]))) {
                    std::cout << ", name: \"" << node.getName() << "\"";
                }
                std::cout << std::endl;
            }
        }
    }
    
    void printPathSummary(const std::vector<int>& path) {
        std::cout << "\n" << formatBold("Path Summary:") << std::endl;
        
        double totalDistance = calculatePathLength(path);
        if (totalDistance > 0) {
            std::cout << "  Total distance: " << formatDistance(totalDistance) << std::endl;
            
            if (path.size() >= 2) {
                double directDistance = calculateDirectDistance(path.front(), path.back());
                if (directDistance > 0) {
                    double efficiency = directDistance / totalDistance * 100.0;
                    std::cout << "  Direct distance: " << formatDistance(directDistance) << std::endl;
                    std::cout << "  Path efficiency: " << std::fixed << std::setprecision(1) 
                              << efficiency << "%" << std::endl;
                }
            }
        }
        
        // Check for path validity
        bool isValid = validatePath(path);
        std::cout << "  Path validity: " << (isValid ? formatSuccess("VALID") : formatError("INVALID")) << std::endl;
        
        if (!isValid) {
            std::vector<int> problems = findPathProblems(path);
            if (!problems.empty()) {
                std::cout << "  Problem segments: ";
                for (size_t i = 0; i < problems.size(); ++i) {
                    if (i > 0) std::cout << ", ";
                    std::cout << problems[i] << "->" << (problems[i] + 1);
                }
                std::cout << std::endl;
            }
        }
    }
    
    void printPathSummaryOnly(const std::vector<int>& path) {
        if (path.empty()) {
            std::cout << "  Empty path" << std::endl;
            return;
        }
        
        std::cout << "  Nodes: " << path.size() << std::endl;
        
        double totalDistance = calculatePathLength(path);
        if (totalDistance > 0) {
            std::cout << "  Distance: " << formatDistance(totalDistance) << std::endl;
        }
        
        bool isValid = validatePath(path);
        std::cout << "  Valid: " << (isValid ? "Yes" : "No") << std::endl;
    }
    
    void printPathsComparison(const std::vector<std::vector<int>>& paths,
                             const std::vector<std::string>& labels) {
        
        std::cout << "\n" << formatBold("Comparison Summary:") << std::endl;
        
        // Find shortest and longest paths
        size_t shortestIdx = 0, longestIdx = 0;
        double shortestDist = std::numeric_limits<double>::max();
        double longestDist = 0.0;
        
        for (size_t i = 0; i < paths.size(); ++i) {
            double dist = calculatePathLength(paths[i]);
            if (dist < shortestDist) {
                shortestDist = dist;
                shortestIdx = i;
            }
            if (dist > longestDist) {
                longestDist = dist;
                longestIdx = i;
            }
        }
        
        std::string shortestLabel = (shortestIdx < labels.size()) ? 
                                   labels[shortestIdx] : ("Path " + std::to_string(shortestIdx + 1));
        std::string longestLabel = (longestIdx < labels.size()) ? 
                                  labels[longestIdx] : ("Path " + std::to_string(longestIdx + 1));
        
        std::cout << "  Shortest: " << shortestLabel << " (" << formatDistance(shortestDist) << ")" << std::endl;
        std::cout << "  Longest:  " << longestLabel << " (" << formatDistance(longestDist) << ")" << std::endl;
        
        if (longestDist > 0 && shortestDist > 0) {
            double variation = (longestDist - shortestDist) / shortestDist * 100.0;
            std::cout << "  Variation: " << std::fixed << std::setprecision(1) << variation << "%" << std::endl;
        }
    }
    
    // Formatting helper methods
    std::string formatBold(const std::string& text) {
        return enableColors ? (COLOR_BOLD + text + COLOR_RESET) : text;
    }
    
    std::string formatStartNode(int nodeId) {
        std::string formatted = "START [" + std::to_string(nodeId) + "]";
        return enableColors ? (COLOR_GREEN + formatted + COLOR_RESET) : formatted;
    }
    
    std::string formatEndNode(int nodeId) {
        std::string formatted = "GOAL  [" + std::to_string(nodeId) + "]";
        return enableColors ? (COLOR_RED + formatted + COLOR_RESET) : formatted;
    }
    
    std::string formatPathNode(int nodeId) {
        std::string formatted = "Node  [" + std::to_string(nodeId) + "]";
        return enableColors ? (COLOR_YELLOW + formatted + COLOR_RESET) : formatted;
    }
    
    std::string formatNodeId(int nodeId) {
        return std::to_string(nodeId);
    }
    
    std::string formatCoordinates(double x, double y) {
        std::ostringstream coords;
        coords << "(" << std::fixed << std::setprecision(1) << x << "," << y << ")";
        return enableColors ? (COLOR_CYAN + coords.str() + COLOR_RESET) : coords.str();
    }
    
    std::string formatDistance(double distance) {
        std::ostringstream dist;
        dist << std::fixed << std::setprecision(2) << distance << " units";
        return enableColors ? (COLOR_BLUE + dist.str() + COLOR_RESET) : dist.str();
    }
    
    std::string formatSuccess(const std::string& text) {
        return enableColors ? (COLOR_GREEN + text + COLOR_RESET) : text;
    }
    
    std::string formatError(const std::string& text) {
        return enableColors ? (COLOR_RED + text + COLOR_RESET) : text;
    }
    
    // Utility methods
    double calculatePathLength(const std::vector<int>& path) {
        if (path.size() < 2) return 0.0;
        
        double totalLength = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (graph->hasNode(path[i-1]) && graph->hasNode(path[i])) {
                const Node& from = graph->getNode(path[i-1]);
                const Node& to = graph->getNode(path[i]);
                totalLength += from.euclideanDistance(to);
            }
        }
        return totalLength;
    }
    
    double calculateDirectDistance(int startId, int endId) {
        if (!graph->hasNode(startId) || !graph->hasNode(endId)) {
            return 0.0;
        }
        
        const Node& start = graph->getNode(startId);
        const Node& end = graph->getNode(endId);
        return start.euclideanDistance(end);
    }
    
    bool validatePath(const std::vector<int>& path) {
        if (path.empty()) return false;
        
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) return false;
            
            if (i > 0 && !graph->hasEdge(path[i-1], path[i])) {
                return false;
            }
        }
        return true;
    }
    
    std::vector<int> findPathProblems(const std::vector<int>& path) {
        std::vector<int> problems;
        
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i]) || 
                !graph->hasEdge(path[i-1], path[i])) {
                problems.push_back(static_cast<int>(i-1));
            }
        }
        
        return problems;
    }
};

// Global console printer
static std::unique_ptr<ConsolePathPrinter> g_consolePrinter;

void initializeConsolePrinter(const Graph* graph) {
    g_consolePrinter = std::make_unique<ConsolePathPrinter>(graph);
}

void printPathToConsole(const Graph* graph, const std::vector<int>& path) {
    if (!g_consolePrinter) {
        g_consolePrinter = std::make_unique<ConsolePathPrinter>(graph);
    }
    g_consolePrinter->printPath(path);
}

void printDetailedPath(const Graph* graph, const std::vector<int>& path) {
    if (!g_consolePrinter) {
        g_consolePrinter = std::make_unique<ConsolePathPrinter>(graph);
    }
    g_consolePrinter->printDetailedPath(path);
}

void printPathComparison(const Graph* graph, const std::vector<int>& path1, 
                        const std::vector<int>& path2, const std::string& label1, 
                        const std::string& label2) {
    if (!g_consolePrinter) {
        g_consolePrinter = std::make_unique<ConsolePathPrinter>(graph);
    }
    g_consolePrinter->printPathComparison(path1, path2, label1, label2);
}

void setConsoleColors(bool enabled) {
    if (g_consolePrinter) {
        g_consolePrinter->setColorEnabled(enabled);
    }
}