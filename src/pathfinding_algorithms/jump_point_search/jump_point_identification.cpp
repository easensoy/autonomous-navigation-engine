#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <queue>
#include <cmath>

class JumpPointSearch::JumpPointIdentifier {
private:
    const Graph* graph;
    int gridWidth;
    int gridHeight;
    std::unordered_set<std::string> identifiedJumpPoints;
    std::unordered_map<std::string, std::string> jumpPointReasons;
    bool recursiveOptimization;
    
public:
    JumpPointIdentifier(const Graph* environment, int width, int height)
        : graph(environment), gridWidth(width), gridHeight(height), recursiveOptimization(true) {
        
        std::cout << "[JPS_IDENTIFICATION] Jump point identifier initialized for " 
                  << width << "x" << height << " grid" << std::endl;
    }
    
    std::pair<int, int> identifyJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY) {
        int currentX = x + dx;
        int currentY = y + dy;
        
        if (!isValidPosition(currentX, currentY) || hasObstacle(currentX, currentY)) {
            return {-1, -1};
        }
        
        std::cout << "[JPS_IDENTIFICATION] Searching for jump point from (" << x << "," << y 
                  << ") in direction (" << dx << "," << dy << ")" << std::endl;
        
        if (currentX == goalX && currentY == goalY) {
            std::string key = std::to_string(currentX) + "," + std::to_string(currentY);
            identifiedJumpPoints.insert(key);
            jumpPointReasons[key] = "goal_reached";
            
            std::cout << "[JPS_IDENTIFICATION] Jump point found at goal (" 
                      << currentX << "," << currentY << ")" << std::endl;
            return {currentX, currentY};
        }
        
        if (isDiagonalMovement(dx, dy)) {
            return identifyDiagonalJumpPoint(currentX, currentY, dx, dy, goalX, goalY);
        } else {
            return identifyCardinalJumpPoint(currentX, currentY, dx, dy, goalX, goalY);
        }
    }
    
    std::pair<int, int> identifyDiagonalJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY) {
        std::vector<std::pair<int, int>> forcedNeighbors = checkForForcedNeighbors(x, y, dx, dy);
        
        if (!forcedNeighbors.empty()) {
            std::string key = std::to_string(x) + "," + std::to_string(y);
            identifiedJumpPoints.insert(key);
            jumpPointReasons[key] = "forced_neighbors_diagonal";
            
            std::cout << "[JPS_IDENTIFICATION] Diagonal jump point found at (" << x << "," << y 
                      << ") with " << forcedNeighbors.size() << " forced neighbors" << std::endl;
            return {x, y};
        }
        
        auto horizontalJump = identifyJumpPoint(x, y, dx, 0, goalX, goalY);
        if (horizontalJump.first != -1) {
            std::string key = std::to_string(x) + "," + std::to_string(y);
            identifiedJumpPoints.insert(key);
            jumpPointReasons[key] = "horizontal_jump_found";
            
            std::cout << "[JPS_IDENTIFICATION] Diagonal jump point found at (" << x << "," << y 
                      << ") due to horizontal jump at (" << horizontalJump.first 
                      << "," << horizontalJump.second << ")" << std::endl;
            return {x, y};
        }
        
        auto verticalJump = identifyJumpPoint(x, y, 0, dy, goalX, goalY);
        if (verticalJump.first != -1) {
            std::string key = std::to_string(x) + "," + std::to_string(y);
            identifiedJumpPoints.insert(key);
            jumpPointReasons[key] = "vertical_jump_found";
            
            std::cout << "[JPS_IDENTIFICATION] Diagonal jump point found at (" << x << "," << y 
                      << ") due to vertical jump at (" << verticalJump.first 
                      << "," << verticalJump.second << ")" << std::endl;
            return {x, y};
        }
        
        if (recursiveOptimization) {
            return identifyJumpPoint(x, y, dx, dy, goalX, goalY);
        }
        
        return {-1, -1};
    }
    
    std::pair<int, int> identifyCardinalJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY) {
        std::vector<std::pair<int, int>> forcedNeighbors = checkForForcedNeighbors(x, y, dx, dy);
        
        if (!forcedNeighbors.empty()) {
            std::string key = std::to_string(x) + "," + std::to_string(y);
            identifiedJumpPoints.insert(key);
            jumpPointReasons[key] = "forced_neighbors_cardinal";
            
            std::cout << "[JPS_IDENTIFICATION] Cardinal jump point found at (" << x << "," << y 
                      << ") with " << forcedNeighbors.size() << " forced neighbors" << std::endl;
            return {x, y};
        }
        
        if (recursiveOptimization) {
            return identifyJumpPoint(x, y, dx, dy, goalX, goalY);
        }
        
        return {-1, -1};
    }
    
    std::vector<std::pair<int, int>> checkForForcedNeighbors(int x, int y, int dx, int dy) {
        std::vector<std::pair<int, int>> forcedNeighbors;
        
        if (isDiagonalMovement(dx, dy)) {
            if (!isValidPosition(x - dx, y) || hasObstacle(x - dx, y)) {
                if (isValidPosition(x - dx, y + dy) && !hasObstacle(x - dx, y + dy)) {
                    forcedNeighbors.emplace_back(x - dx, y + dy);
                }
            }
            
            if (!isValidPosition(x, y - dy) || hasObstacle(x, y - dy)) {
                if (isValidPosition(x + dx, y - dy) && !hasObstacle(x + dx, y - dy)) {
                    forcedNeighbors.emplace_back(x + dx, y - dy);
                }
            }
        } else if (dx != 0) {
            if (!isValidPosition(x, y - 1) || hasObstacle(x, y - 1)) {
                if (isValidPosition(x + dx, y - 1) && !hasObstacle(x + dx, y - 1)) {
                    forcedNeighbors.emplace_back(x + dx, y - 1);
                }
            }
            
            if (!isValidPosition(x, y + 1) || hasObstacle(x, y + 1)) {
                if (isValidPosition(x + dx, y + 1) && !hasObstacle(x + dx, y + 1)) {
                    forcedNeighbors.emplace_back(x + dx, y + 1);
                }
            }
        } else if (dy != 0) {
            if (!isValidPosition(x - 1, y) || hasObstacle(x - 1, y)) {
                if (isValidPosition(x - 1, y + dy) && !hasObstacle(x - 1, y + dy)) {
                    forcedNeighbors.emplace_back(x - 1, y + dy);
                }
            }
            
            if (!isValidPosition(x + 1, y) || hasObstacle(x + 1, y)) {
                if (isValidPosition(x + 1, y + dy) && !hasObstacle(x + 1, y + dy)) {
                    forcedNeighbors.emplace_back(x + 1, y + dy);
                }
            }
        }
        
        return forcedNeighbors;
    }
    
    std::vector<std::pair<int, int>> findAllJumpPoints(int startX, int startY, int goalX, int goalY) {
        std::cout << "[JPS_IDENTIFICATION] Finding all jump points from (" << startX << "," << startY 
                  << ") to (" << goalX << "," << goalY << ")" << std::endl;
        
        std::vector<std::pair<int, int>> allJumpPoints;
        std::unordered_set<std::string> visited;
        std::queue<std::pair<int, int>> searchQueue;
        
        searchQueue.push({startX, startY});
        visited.insert(std::to_string(startX) + "," + std::to_string(startY));
        
        static const std::vector<std::pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
        };
        
        while (!searchQueue.empty()) {
            auto [currentX, currentY] = searchQueue.front();
            searchQueue.pop();
            
            for (auto [dx, dy] : directions) {
                auto jumpPoint = identifyJumpPoint(currentX, currentY, dx, dy, goalX, goalY);
                
                if (jumpPoint.first != -1) {
                    std::string jumpKey = std::to_string(jumpPoint.first) + "," + std::to_string(jumpPoint.second);
                    
                    if (visited.find(jumpKey) == visited.end()) {
                        allJumpPoints.push_back(jumpPoint);
                        visited.insert(jumpKey);
                        searchQueue.push(jumpPoint);
                    }
                }
            }
        }
        
        std::cout << "[JPS_IDENTIFICATION] Found " << allJumpPoints.size() 
                  << " total jump points" << std::endl;
        
        return allJumpPoints;
    }
    
    bool isJumpPoint(int x, int y, int parentX, int parentY, int goalX, int goalY) {
        if (x == goalX && y == goalY) {
            return true;
        }
        
        if (parentX == -1 && parentY == -1) {
            return true;
        }
        
        int dx = normalizeDirection(x - parentX);
        int dy = normalizeDirection(y - parentY);
        
        std::vector<std::pair<int, int>> forcedNeighbors = checkForForcedNeighbors(x, y, dx, dy);
        return !forcedNeighbors.empty();
    }
    
    void analyzeJumpPointDistribution() {
        std::unordered_map<std::string, int> reasonCounts;
        
        for (const auto& [point, reason] : jumpPointReasons) {
            reasonCounts[reason]++;
        }
        
        std::cout << "[JPS_IDENTIFICATION] Jump point analysis:" << std::endl;
        std::cout << "[JPS_IDENTIFICATION]   Total jump points: " << identifiedJumpPoints.size() << std::endl;
        
        for (const auto& [reason, count] : reasonCounts) {
            double percentage = static_cast<double>(count) / identifiedJumpPoints.size() * 100.0;
            std::cout << "[JPS_IDENTIFICATION]   " << reason << ": " << count 
                      << " (" << percentage << "%)" << std::endl;
        }
    }
    
    double calculateJumpPointDensity() {
        size_t totalGridNodes = gridWidth * gridHeight;
        double density = static_cast<double>(identifiedJumpPoints.size()) / totalGridNodes * 100.0;
        
        std::cout << "[JPS_IDENTIFICATION] Jump point density: " << identifiedJumpPoints.size() 
                  << "/" << totalGridNodes << " (" << density << "%)" << std::endl;
        
        return density;
    }
    
    void optimizeJumpPointIdentification(bool enable) {
        recursiveOptimization = enable;
        std::cout << "[JPS_IDENTIFICATION] Recursive optimization " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void clearJumpPointCache() {
        identifiedJumpPoints.clear();
        jumpPointReasons.clear();
        std::cout << "[JPS_IDENTIFICATION] Jump point cache cleared" << std::endl;
    }
    
    std::vector<std::pair<int, int>> getJumpPointsInRegion(int minX, int minY, int maxX, int maxY) {
        std::vector<std::pair<int, int>> regionJumpPoints;
        
        for (const std::string& pointKey : identifiedJumpPoints) {
            auto pos = pointKey.find(',');
            if (pos != std::string::npos) {
                int x = std::stoi(pointKey.substr(0, pos));
                int y = std::stoi(pointKey.substr(pos + 1));
                
                if (x >= minX && x <= maxX && y >= minY && y <= maxY) {
                    regionJumpPoints.emplace_back(x, y);
                }
            }
        }
        
        std::cout << "[JPS_IDENTIFICATION] Found " << regionJumpPoints.size() 
                  << " jump points in region (" << minX << "," << minY 
                  << ") to (" << maxX << "," << maxY << ")" << std::endl;
        
        return regionJumpPoints;
    }
    
private:
    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
    }
    
    bool hasObstacle(int x, int y) const {
        return false;
    }
    
    bool isDiagonalMovement(int dx, int dy) const {
        return dx != 0 && dy != 0;
    }
    
    int normalizeDirection(int value) const {
        if (value > 0) return 1;
        if (value < 0) return -1;
        return 0;
    }
};

void JumpPointSearch::initializeJumpPointIdentification() {
    if (!jumpPointIdentifier) {
        jumpPointIdentifier = std::make_unique<JumpPointIdentifier>(graph, gridWidth, gridHeight);
    }
}

std::pair<int, int> JumpPointSearch::identifyJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->identifyJumpPoint(x, y, dx, dy, goalX, goalY);
}

std::vector<std::pair<int, int>> JumpPointSearch::findAllJumpPointsInPath(int startX, int startY, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->findAllJumpPoints(startX, startY, goalX, goalY);
}

bool JumpPointSearch::isNodeJumpPoint(int x, int y, int parentX, int parentY, int goalX, int goalY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->isJumpPoint(x, y, parentX, parentY, goalX, goalY);
}

std::vector<std::pair<int, int>> JumpPointSearch::identifyForcedNeighborsAtPosition(int x, int y, int dx, int dy) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->checkForForcedNeighbors(x, y, dx, dy);
}

void JumpPointSearch::analyzeJumpPointDistribution() {
    if (jumpPointIdentifier) {
        jumpPointIdentifier->analyzeJumpPointDistribution();
    }
}

double JumpPointSearch::calculateJumpPointDensity() {
    return jumpPointIdentifier ? jumpPointIdentifier->calculateJumpPointDensity() : 0.0;
}

void JumpPointSearch::optimizeJumpPointIdentification(bool enable) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    jumpPointIdentifier->optimizeJumpPointIdentification(enable);
}

void JumpPointSearch::clearJumpPointCache() {
    if (jumpPointIdentifier) {
        jumpPointIdentifier->clearJumpPointCache();
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::getJumpPointsInRegion(int minX, int minY, int maxX, int maxY) {
    if (!jumpPointIdentifier) {
        initializeJumpPointIdentification();
    }
    
    return jumpPointIdentifier->getJumpPointsInRegion(minX, minY, maxX, maxY);
}