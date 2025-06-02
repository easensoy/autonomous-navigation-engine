#include "pathfinding_algorithms/AStar.hpp"
#include <queue>
#include <unordered_set>

class AStar::OpenListManager {
private:
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openQueue;
    std::unordered_set<int> openSet;
    
public:
    void push(const AStarNode& node) {
        openQueue.push(node);
        openSet.insert(node.nodeId);
    }
    
    AStarNode pop() {
        while (!openQueue.empty()) {
            AStarNode current = openQueue.top();
            openQueue.pop();
            
            if (openSet.find(current.nodeId) != openSet.end()) {
                openSet.erase(current.nodeId);
                return current;
            }
        }
        throw std::runtime_error("Attempted to pop from empty open list");
    }
    
    bool empty() const {
        return openQueue.empty() || openSet.empty();
    }
    
    bool contains(int nodeId) const {
        return openSet.find(nodeId) != openSet.end();
    }
    
    void remove(int nodeId) {
        openSet.erase(nodeId);
    }
    
    size_t size() const {
        return openSet.size();
    }
    
    void clear() {
        while (!openQueue.empty()) {
            openQueue.pop();
        }
        openSet.clear();
    }
};

void AStar::initializeOpenList() {
    if (!openListManager) {
        openListManager = std::make_unique<OpenListManager>();
    }
    openListManager->clear();
}

void AStar::addToOpenList(const AStarNode& node) {
    if (!openListManager) {
        initializeOpenList();
    }
    openListManager->push(node);
}

AStar::AStarNode AStar::getNextFromOpenList() {
    if (!openListManager || openListManager->empty()) {
        throw std::runtime_error("Open list is empty");
    }
    return openListManager->pop();
}

bool AStar::isOpenListEmpty() const {
    return !openListManager || openListManager->empty();
}

bool AStar::isInOpenList(int nodeId) const {
    return openListManager && openListManager->contains(nodeId);
}

void AStar::removeFromOpenList(int nodeId) {
    if (openListManager) {
        openListManager->remove(nodeId);
    }
}

size_t AStar::getOpenListSize() const {
    return openListManager ? openListManager->size() : 0;
}