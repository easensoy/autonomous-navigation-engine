#include "pathfinding_algorithms/JumpPointSearch.hpp"
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <algorithm>

JumpPointSearch::JumpPointSearch(const Graph* environment, int width, int height)
    : graph(environment), gridWidth(width), gridHeight(height) {
    
    if (!graph) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
    
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
    
    std::cout << "[JPS] Jump Point Search initialized for " << width << "x" << height 
              << " grid with " << graph->getNodeCount() << " nodes" << std::endl;
}

JumpPointSearch::GridNode::GridNode(int xPos, int yPos) 
    : x(xPos), y(yPos), gScore(std::numeric_limits<double>::infinity()),
      fScore(std::numeric_limits<double>::infinity()), parentX(-1), parentY(-1) {}

bool JumpPointSearch::GridNode::operator>(const GridNode& other) const {
    return fScore > other.fScore;
}

std::vector<int> JumpPointSearch::findPath(int startId, int goalId) {
    std::cout << "[JPS] Finding path from node " << startId << " to node " << goalId << std::endl;
    
    if (!graph->hasNode(startId) || !graph->hasNode(goalId)) {
        throw std::invalid_argument("Start or goal node does not exist");
    }
    
    const Node& startNode = graph->getNode(startId);
    const Node& goalNode = graph->getNode(goalId);
    
    int startX = static_cast<int>(startNode.getX());
    int startY = static_cast<int>(startNode.getY());
    int goalX = static_cast<int>(goalNode.getX());
    int goalY = static_cast<int>(goalNode.getY());
    
    std::cout << "[JPS] Grid coordinates: Start(" << startX << "," << startY 
              << ") Goal(" << goalX << "," << goalY << ")" << std::endl;
    
    if (!isValidPosition(startX, startY) || !isValidPosition(goalX, goalY)) {
        std::cout << "[JPS] Invalid start or goal coordinates" << std::endl;
        return {};
    }
    
    std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> openSet;
    std::unordered_set<std::string> closedSet;
    std::unordered_map<std::string, GridNode> allNodes;
    
    GridNode startGridNode(startX, startY);
    startGridNode.gScore = 0.0;
    startGridNode.fScore = heuristic(startX, startY, goalX, goalY);
    
    openSet.push(startGridNode);
    allNodes[std::to_string(startX) + "," + std::to_string(startY)] = startGridNode;
    
    while (!openSet.empty()) {
        GridNode current = openSet.top();
        openSet.pop();
        
        std::string currentKey = std::to_string(current.x) + "," + std::to_string(current.y);
        
        if (closedSet.find(currentKey) != closedSet.end()) {
            continue;
        }
        
        closedSet.insert(currentKey);
        
        std::cout << "[JPS] Processing node (" << current.x << "," << current.y 
                  << ") with f-score " << current.fScore << std::endl;
        
        if (current.x == goalX && current.y == goalY) {
            std::cout << "[JPS] Goal reached! Reconstructing path..." << std::endl;
            return reconstructGridPath(allNodes, startX, startY, goalX, goalY);
        }
        
        std::vector<std::pair<int, int>> jumpPoints = findJumpPoints(current.x, current.y, goalX, goalY);
        
        for (auto [jumpX, jumpY] : jumpPoints) {
            std::string jumpKey = std::to_string(jumpX) + "," + std::to_string(jumpY);
            
            if (closedSet.find(jumpKey) != closedSet.end()) {
                continue;
            }
            
            double tentativeGScore = current.gScore + heuristic(current.x, current.y, jumpX, jumpY);
            
            auto nodeIt = allNodes.find(jumpKey);
            if (nodeIt == allNodes.end() || tentativeGScore < nodeIt->second.gScore) {
                GridNode jumpNode(jumpX, jumpY);
                jumpNode.gScore = tentativeGScore;
                jumpNode.fScore = tentativeGScore + heuristic(jumpX, jumpY, goalX, goalY);
                jumpNode.parentX = current.x;
                jumpNode.parentY = current.y;
                
                allNodes[jumpKey] = jumpNode;
                openSet.push(jumpNode);
                
                std::cout << "[JPS] Added jump point (" << jumpX << "," << jumpY 
                          << ") with g-score " << tentativeGScore << std::endl;
            }
        }
    }
    
    std::cout << "[JPS] No path found to goal" << std::endl;
    return {};
}

void JumpPointSearch::setGridDimensions(int width, int height) {
    if (width <= 0 || height <= 0) {
        throw std::invalid_argument("Grid dimensions must be positive");
    }
    
    gridWidth = width;
    gridHeight = height;
    
    std::cout << "[JPS] Grid dimensions updated to " << width << "x" << height << std::endl;
}

void JumpPointSearch::addObstacle(int x, int y) {
    if (!isValidPosition(x, y)) {
        throw std::invalid_argument("Invalid obstacle position");
    }
    
    obstacles.insert(std::to_string(x) + "," + std::to_string(y));
    std::cout << "[JPS] Added obstacle at (" << x << "," << y << ")" << std::endl;
}

void JumpPointSearch::removeObstacle(int x, int y) {
    std::string key = std::to_string(x) + "," + std::to_string(y);
    if (obstacles.erase(key) > 0) {
        std::cout << "[JPS] Removed obstacle at (" << x << "," << y << ")" << std::endl;
    }
}

bool JumpPointSearch::isValidPosition(int x, int y) const {
    return x >= 0 && x < gridWidth && y >= 0 && y < gridHeight;
}

bool JumpPointSearch::hasObstacle(int x, int y) const {
    if (!isValidPosition(x, y)) {
        return true;
    }
    
    std::string key = std::to_string(x) + "," + std::to_string(y);
    return obstacles.find(key) != obstacles.end();
}

std::vector<std::pair<int, int>> JumpPointSearch::getNeighbors(int x, int y) const {
    std::vector<std::pair<int, int>> neighbors;
    
    static const std::vector<std::pair<int, int>> directions = {
        {-1, -1}, {-1, 0}, {-1, 1},
        {0, -1},           {0, 1},
        {1, -1},  {1, 0},  {1, 1}
    };
    
    for (auto [dx, dy] : directions) {
        int newX = x + dx;
        int newY = y + dy;
        
        if (isValidPosition(newX, newY) && !hasObstacle(newX, newY)) {
            neighbors.emplace_back(newX, newY);
        }
    }
    
    return neighbors;
}

std::pair<int, int> JumpPointSearch::jump(int x, int y, int dx, int dy, int goalX, int goalY) const {
    int newX = x + dx;
    int newY = y + dy;
    
    if (!isValidPosition(newX, newY) || hasObstacle(newX, newY)) {
        return {-1, -1};
    }
    
    if (newX == goalX && newY == goalY) {
        return {newX, newY};
    }
    
    if (dx != 0 && dy != 0) {
        if ((isValidPosition(newX - dx, newY + dy) && hasObstacle(newX - dx, newY) && !hasObstacle(newX - dx, newY + dy)) ||
            (isValidPosition(newX + dx, newY - dy) && hasObstacle(newX, newY - dy) && !hasObstacle(newX + dx, newY - dy))) {
            return {newX, newY};
        }
    } else {
        if (dx != 0) {
            if ((isValidPosition(newX, newY + 1) && hasObstacle(newX - dx, newY + 1) && !hasObstacle(newX, newY + 1)) ||
                (isValidPosition(newX, newY - 1) && hasObstacle(newX - dx, newY - 1) && !hasObstacle(newX, newY - 1))) {
                return {newX, newY};
            }
        } else {
            if ((isValidPosition(newX + 1, newY) && hasObstacle(newX + 1, newY - dy) && !hasObstacle(newX + 1, newY)) ||
                (isValidPosition(newX - 1, newY) && hasObstacle(newX - 1, newY - dy) && !hasObstacle(newX - 1, newY))) {
                return {newX, newY};
            }
        }
    }
    
    if (dx != 0 && dy != 0) {
        auto horizontalJump = jump(newX, newY, dx, 0, goalX, goalY);
        auto verticalJump = jump(newX, newY, 0, dy, goalX, goalY);
        
        if (horizontalJump.first != -1 || verticalJump.first != -1) {
            return {newX, newY};
        }
    }
    
    return jump(newX, newY, dx, dy, goalX, goalY);
}

double JumpPointSearch::heuristic(int x1, int y1, int x2, int y2) const {
    double dx = std::abs(x1 - x2);
    double dy = std::abs(y1 - y2);
    
    if (diagonalMovement) {
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    } else {
        return dx + dy;
    }
}

std::vector<std::pair<int, int>> JumpPointSearch::getJumpPoints(int startX, int startY, int goalX, int goalY) {
    return findJumpPoints(startX, startY, goalX, goalY);
}

double JumpPointSearch::getPathLength(const std::vector<int>& path) {
    if (path.size() < 2) {
        return 0.0;
    }
    
    double totalLength = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        const Node& current = graph->getNode(path[i]);
        const Node& next = graph->getNode(path[i + 1]);
        totalLength += current.euclideanDistance(next);
    }
    
    return totalLength;
}

void JumpPointSearch::enableDiagonalMovement(bool enable) {
    diagonalMovement = enable;
    std::cout << "[JPS] Diagonal movement " << (enable ? "enabled" : "disabled") << std::endl;
}

std::vector<std::pair<int, int>> JumpPointSearch::findJumpPoints(int x, int y, int goalX, int goalY) const {
    std::vector<std::pair<int, int>> jumpPoints;
    
    static const std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
    };
    
    for (auto [dx, dy] : directions) {
        if (!diagonalMovement && dx != 0 && dy != 0) {
            continue;
        }
        
        auto jumpPoint = jump(x, y, dx, dy, goalX, goalY);
        if (jumpPoint.first != -1) {
            jumpPoints.push_back(jumpPoint);
        }
    }
    
    return jumpPoints;
}

std::vector<int> JumpPointSearch::reconstructGridPath(const std::unordered_map<std::string, GridNode>& allNodes,
                                                     int startX, int startY, int goalX, int goalY) const {
    std::vector<std::pair<int, int>> gridPath;
    int currentX = goalX;
    int currentY = goalY;
    
    while (currentX != -1 && currentY != -1) {
        gridPath.emplace_back(currentX, currentY);
        
        if (currentX == startX && currentY == startY) {
            break;
        }
        
        std::string key = std::to_string(currentX) + "," + std::to_string(currentY);
        auto it = allNodes.find(key);
        
        if (it != allNodes.end()) {
            currentX = it->second.parentX;
            currentY = it->second.parentY;
        } else {
            break;
        }
    }
    
    std::reverse(gridPath.begin(), gridPath.end());
    
    std::vector<int> nodePath;
    for (auto [x, y] : gridPath) {
        for (int nodeId : graph->getAllNodeIds()) {
            const Node& node = graph->getNode(nodeId);
            if (static_cast<int>(node.getX()) == x && static_cast<int>(node.getY()) == y) {
                nodePath.push_back(nodeId);
                break;
            }
        }
    }
    
    std::cout << "[JPS] Reconstructed path with " << nodePath.size() << " nodes" << std::endl;
    return nodePath;
}