#include "../../../include/pathfinding_algorithms/JumpPointSearch.hpp"
#include <iostream>
#include <unordered_set>
#include <algorithm>
#include <cmath>

class JumpPointSearch::GridPruningManager {
private:
    const Graph* graph;
    int gridWidth;
    int gridHeight;
    std::unordered_set<std::string> prunedNodes;
    std::unordered_map<std::string, std::vector<std::pair<int, int>>> naturalNeighbors;
    std::unordered_map<std::string, std::vector<std::pair<int, int>>> forcedNeighbors;
    bool symmetryBreaking;
    
public:
    GridPruningManager(const Graph* environment, int width, int height)
        : graph(environment), gridWidth(width), gridHeight(height), symmetryBreaking(true) {
        
        std::cout << "[JPS_PRUNING] Grid pruning manager initialized for " 
                  << width << "x" << height << " grid" << std::endl;
    }
    
    std::vector<std::pair<int, int>> pruneNeighbors(int x, int y, int parentX, int parentY) {
        std::string nodeKey = std::to_string(x) + "," + std::to_string(y);
        
        if (parentX == -1 && parentY == -1) {
            return getAllValidNeighbors(x, y);
        }
        
        std::vector<std::pair<int, int>> prunedNeighbors;
        
        int dx = normalizeDirection(x - parentX);
        int dy = normalizeDirection(y - parentY);
        
        if (dx != 0 && dy != 0) {
            prunedNeighbors = pruneDiagonalNeighbors(x, y, dx, dy);
        } else if (dx != 0) {
            prunedNeighbors = pruneHorizontalNeighbors(x, y, dx);
        } else if (dy != 0) {
            prunedNeighbors = pruneVerticalNeighbors(x, y, dy);
        }
        
        std::vector<std::pair<int, int>> forcedNeighborsList = identifyForcedNeighbors(x, y, dx, dy);
        prunedNeighbors.insert(prunedNeighbors.end(), forcedNeighborsList.begin(), forcedNeighborsList.end());
        
        std::cout << "[JPS_PRUNING] Node (" << x << "," << y << ") pruned from " 
                  << getAllValidNeighbors(x, y).size() << " to " << prunedNeighbors.size() 
                  << " neighbors" << std::endl;
        
        return prunedNeighbors;
    }
    
    std::vector<std::pair<int, int>> pruneDiagonalNeighbors(int x, int y, int dx, int dy) {
        std::vector<std::pair<int, int>> neighbors;
        
        if (isValidAndPassable(x + dx, y)) {
            neighbors.emplace_back(x + dx, y);
        }
        
        if (isValidAndPassable(x, y + dy)) {
            neighbors.emplace_back(x, y + dy);
        }
        
        if (isValidAndPassable(x + dx, y + dy)) {
            neighbors.emplace_back(x + dx, y + dy);
        }
        
        if (!isValidAndPassable(x - dx, y) && isValidAndPassable(x - dx, y + dy)) {
            neighbors.emplace_back(x - dx, y + dy);
        }
        
        if (!isValidAndPassable(x, y - dy) && isValidAndPassable(x + dx, y - dy)) {
            neighbors.emplace_back(x + dx, y - dy);
        }
        
        std::cout << "[JPS_PRUNING] Diagonal pruning at (" << x << "," << y 
                  << ") with direction (" << dx << "," << dy << ") yielded " 
                  << neighbors.size() << " neighbors" << std::endl;
        
        return neighbors;
    }
    
    std::vector<std::pair<int, int>> pruneHorizontalNeighbors(int x, int y, int dx) {
        std::vector<std::pair<int, int>> neighbors;
        
        if (isValidAndPassable(x + dx, y)) {
            neighbors.emplace_back(x + dx, y);
        }
        
        if (!isValidAndPassable(x, y - 1) && isValidAndPassable(x + dx, y - 1)) {
            neighbors.emplace_back(x + dx, y - 1);
        }
        
        if (!isValidAndPassable(x, y + 1) && isValidAndPassable(x + dx, y + 1)) {
            neighbors.emplace_back(x + dx, y + 1);
        }
        
        std::cout << "[JPS_PRUNING] Horizontal pruning at (" << x << "," << y 
                  << ") with direction (" << dx << ",0) yielded " 
                  << neighbors.size() << " neighbors" << std::endl;
        
        return neighbors;
    }
    
    std::vector<std::pair<int, int>> pruneVerticalNeighbors(int x, int y, int dy) {
        std::vector<std::pair<int, int>> neighbors;
        
        if (isValidAndPassable(x, y + dy)) {
            neighbors.emplace_back(x, y + dy);
        }
        
        if (!isValidAndPassable(x - 1, y) && isValidAndPassable(x - 1, y + dy)) {
            neighbors.emplace_back(x - 1, y + dy);
        }
        
        if (!isValidAndPassable(x + 1, y) && isValidAndPassable(x + 1, y + dy)) {
            neighbors.emplace_back(x + 1, y + dy);
        }
        
        std::cout << "[JPS_PRUNING] Vertical pruning at (" << x << "," << y 
                  << ") with direction (0," << dy << ") yielded " 
                  << neighbors.size() << " neighbors" << std::endl;
        
        return neighbors;
    }
    
    std::vector<std::pair<int, int>> identifyForcedNeighbors(int x, int y, int dx, int dy) {
        std::vector<std::pair<int, int>> forcedNeighbors;
        
        if (dx != 0 && dy != 0) {
            if (!isValidAndPassable(x - dx, y) && isValidAndPassable(x - dx, y + dy)) {
                forcedNeighbors.emplace_back(x - dx, y + dy);
                std::cout << "[JPS_PRUNING] Forced neighbor identified at (" 
                          << (x - dx) << "," << (y + dy) << ") due to blocked (" 
                          << (x - dx) << "," << y << ")" << std::endl;
            }
            
            if (!isValidAndPassable(x, y - dy) && isValidAndPassable(x + dx, y - dy)) {
                forcedNeighbors.emplace_back(x + dx, y - dy);
                std::cout << "[JPS_PRUNING] Forced neighbor identified at (" 
                          << (x + dx) << "," << (y - dy) << ") due to blocked (" 
                          << x << "," << (y - dy) << ")" << std::endl;
            }
        } else if (dx != 0) {
            if (!isValidAndPassable(x, y - 1) && isValidAndPassable(x + dx, y - 1)) {
                forcedNeighbors.emplace_back(x + dx, y - 1);
            }
            
            if (!isValidAndPassable(x, y + 1) && isValidAndPassable(x + dx, y + 1)) {
                forcedNeighbors.emplace_back(x + dx, y + 1);
            }
        } else if (dy != 0) {
            if (!isValidAndPassable(x - 1, y) && isValidAndPassable(x - 1, y + dy)) {
                forcedNeighbors.emplace_back(x - 1, y + dy);
            }
            
            if (!isValidAndPassable(x + 1, y) && isValidAndPassable(x + 1, y + dy)) {
                forcedNeighbors.emplace_back(x + 1, y + dy);
            }
        }
        
        return forcedNeighbors;
    }
    
    bool shouldPruneNode(int x, int y, int parentX, int parentY, int goalX, int goalY) {
        if (x == goalX && y == goalY) {
            return false;
        }
        
        if (parentX == -1 && parentY == -1) {
            return false;
        }
        
        if (!symmetryBreaking) {
            return false;
        }
        
        int dx = normalizeDirection(x - parentX);
        int dy = normalizeDirection(y - parentY);
        
        if (dx == 0 && dy == 0) {
            return false;
        }
        
        bool hasShorterAlternativePath = checkForShorterAlternative(x, y, parentX, parentY, dx, dy);
        
        if (hasShorterAlternativePath) {
            std::cout << "[JPS_PRUNING] Node (" << x << "," << y 
                      << ") pruned due to shorter alternative path" << std::endl;
            return true;
        }
        
        return false;
    }
    
    double calculatePruningEfficiency() {
        size_t totalNodes = gridWidth * gridHeight;
        size_t prunedNodeCount = prunedNodes.size();
        
        double efficiency = static_cast<double>(prunedNodeCount) / totalNodes * 100.0;
        
        std::cout << "[JPS_PRUNING] Pruning efficiency: " << prunedNodeCount 
                  << "/" << totalNodes << " nodes (" << efficiency << "%)" << std::endl;
        
        return efficiency;
    }
    
    void enableSymmetryBreaking(bool enable) {
        symmetryBreaking = enable;
        std::cout << "[JPS_PRUNING] Symmetry breaking " 
                  << (enable ? "enabled" : "disabled") << std::endl;
    }
    
    void resetPruningState() {
        prunedNodes.clear();
        naturalNeighbors.clear();
        forcedNeighbors.clear();
        std::cout << "[JPS_PRUNING] Pruning state reset" << std::endl;
    }
    
    void analyzePruningPatterns() {
        std::unordered_map<std::string, int> pruningReasons;
        
        for (const std::string& nodeKey : prunedNodes) {
            auto pos = nodeKey.find(',');
            if (pos != std::string::npos) {
                int x = std::stoi(nodeKey.substr(0, pos));
                int y = std::stoi(nodeKey.substr(pos + 1));
                
                std::string reason = categorizePruningReason(x, y);
                pruningReasons[reason]++;
            }
        }
        
        std::cout << "[JPS_PRUNING] Pruning pattern analysis:" << std::endl;
        for (const auto& [reason, count] : pruningReasons) {
            std::cout << "[JPS_PRUNING]   " << reason << ": " << count << " nodes" << std::endl;
        }
    }
    
private:
    std::vector<std::pair<int, int>> getAllValidNeighbors(int x, int y) {
        std::vector<std::pair<int, int>> neighbors;
        
        static const std::vector<std::pair<int, int>> directions = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},           {0, 1},
            {1, -1},  {1, 0},  {1, 1}
        };
        
        for (auto [dx, dy] : directions) {
            int newX = x + dx;
            int newY = y + dy;
            
            if (isValidAndPassable(newX, newY)) {
                neighbors.emplace_back(newX, newY);
            }
        }
        
        return neighbors;
    }
    
    bool isValidAndPassable(int x, int y) {
        return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight && !hasObstacleAt(x, y);
    }
    
    bool hasObstacleAt(int x, int y) {
        return false;
    }
    
    int normalizeDirection(int value) {
        if (value > 0) return 1;
        if (value < 0) return -1;
        return 0;
    }
    
    bool checkForShorterAlternative(int x, int y, int parentX, int parentY, int dx, int dy) {
        if (dx != 0 && dy != 0) {
            return isValidAndPassable(parentX + dx, parentY) && 
                   isValidAndPassable(parentX, parentY + dy);
        }
        
        return false;
    }
    
    std::string categorizePruningReason(int x, int y) {
        if (x == 0 || x == gridWidth - 1 || y == 0 || y == gridHeight - 1) {
            return "boundary_optimization";
        }
        
        int neighborCount = getAllValidNeighbors(x, y).size();
        if (neighborCount <= 3) {
            return "sparse_connectivity";
        } else if (neighborCount >= 7) {
            return "dense_connectivity";
        }
        
        return "symmetry_breaking";
    }
};

void JumpPointSearch::initializeGridPruning() {
    if (!pruningManager) {
        pruningManager = std::make_unique<GridPruningManager>(graph, gridWidth, gridHeight);
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneNeighbors(int x, int y, int parentX, int parentY) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->pruneNeighbors(x, y, parentX, parentY);
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneDiagonalMovement(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->pruneDiagonalNeighbors(x, y, dx, dy);
}

std::vector<std::pair<int, int>> JumpPointSearch::pruneCardinalMovement(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    if (dx != 0) {
        return pruningManager->pruneHorizontalNeighbors(x, y, dx);
    } else {
        return pruningManager->pruneVerticalNeighbors(x, y, dy);
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::identifyForcedNeighbors(int x, int y, int dx, int dy) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->identifyForcedNeighbors(x, y, dx, dy);
}

bool JumpPointSearch::shouldPruneNode(int x, int y, int parentX, int parentY, int goalX, int goalY) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    return pruningManager->shouldPruneNode(x, y, parentX, parentY, goalX, goalY);
}

double JumpPointSearch::calculatePruningEfficiency() {
    return pruningManager ? pruningManager->calculatePruningEfficiency() : 0.0;
}

void JumpPointSearch::enableSymmetryBreaking(bool enable) {
    if (!pruningManager) {
        initializeGridPruning();
    }
    
    pruningManager->enableSymmetryBreaking(enable);
}

void JumpPointSearch::resetPruningState() {
    if (pruningManager) {
        pruningManager->resetPruningState();
    }
}

void JumpPointSearch::analyzePruningPatterns() {
    if (pruningManager) {
        pruningManager->analyzePruningPatterns();
    }
}