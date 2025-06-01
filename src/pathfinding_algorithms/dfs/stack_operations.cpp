#include "../../../include/pathfinding_algorithms/DFS.hpp"
#include <stack>
#include <iostream>

class DFS::StackManager {
private:
    std::stack<int> nodeStack;
    std::stack<std::vector<int>> pathStack;
    size_t maxStackSize;
    size_t totalPushed;
    size_t totalPopped;
    
public:
    StackManager() : maxStackSize(0), totalPushed(0), totalPopped(0) {}
    
    void pushNode(int nodeId) {
        nodeStack.push(nodeId);
        totalPushed++;
        
        if (nodeStack.size() > maxStackSize) {
            maxStackSize = nodeStack.size();
        }
    }
    
    void pushNodeWithPath(int nodeId, const std::vector<int>& path) {
        nodeStack.push(nodeId);
        pathStack.push(path);
        totalPushed++;
        
        if (nodeStack.size() > maxStackSize) {
            maxStackSize = nodeStack.size();
        }
    }
    
    int popNode() {
        if (nodeStack.empty()) {
            throw std::runtime_error("Cannot pop from empty stack");
        }
        
        int node = nodeStack.top();
        nodeStack.pop();
        totalPopped++;
        
        if (!pathStack.empty()) {
            pathStack.pop();
        }
        
        return node;
    }
    
    std::pair<int, std::vector<int>> popNodeWithPath() {
        if (nodeStack.empty() || pathStack.empty()) {
            throw std::runtime_error("Cannot pop from empty stack");
        }
        
        int node = nodeStack.top();
        std::vector<int> path = pathStack.top();
        
        nodeStack.pop();
        pathStack.pop();
        totalPopped++;
        
        return {node, path};
    }
    
    int top() const {
        if (nodeStack.empty()) {
            throw std::runtime_error("Stack is empty");
        }
        return nodeStack.top();
    }
    
    bool empty() const {
        return nodeStack.empty();
    }
    
    size_t size() const {
        return nodeStack.size();
    }
    
    void clear() {
        while (!nodeStack.empty()) {
            nodeStack.pop();
        }
        while (!pathStack.empty()) {
            pathStack.pop();
        }
    }
    
    size_t getMaxStackSize() const {
        return maxStackSize;
    }
    
    size_t getTotalPushed() const {
        return totalPushed;
    }
    
    size_t getTotalPopped() const {
        return totalPopped;
    }
    
    void resetStatistics() {
        maxStackSize = nodeStack.size();
        totalPushed = 0;
        totalPopped = 0;
    }
    
    void printStatistics() const {
        std::cout << "[DFS_STACK] Stack Statistics:" << std::endl;
        std::cout << "[DFS_STACK]   Current size: " << size() << std::endl;
        std::cout << "[DFS_STACK]   Maximum depth reached: " << maxStackSize << std::endl;
        std::cout << "[DFS_STACK]   Total pushed: " << totalPushed << std::endl;
        std::cout << "[DFS_STACK]   Total popped: " << totalPopped << std::endl;
        std::cout << "[DFS_STACK]   Current depth: " << (totalPushed - totalPopped) << std::endl;
    }
};

void DFS::initializeStack() {
    if (!stackManager) {
        stackManager = std::make_unique<StackManager>();
    }
    stackManager->clear();
    stackManager->resetStatistics();
}

void DFS::pushToStack(int nodeId) {
    if (!stackManager) {
        initializeStack();
    }
    stackManager->pushNode(nodeId);
}

void DFS::pushToStackWithPath(int nodeId, const std::vector<int>& path) {
    if (!stackManager) {
        initializeStack();
    }
    stackManager->pushNodeWithPath(nodeId, path);
}

int DFS::popFromStack() {
    if (!stackManager) {
        throw std::runtime_error("Stack not initialized");
    }
    return stackManager->popNode();
}

std::pair<int, std::vector<int>> DFS::popFromStackWithPath() {
    if (!stackManager) {
        throw std::runtime_error("Stack not initialized");
    }
    return stackManager->popNodeWithPath();
}

bool DFS::isStackEmpty() const {
    return !stackManager || stackManager->empty();
}

size_t DFS::getStackSize() const {
    return stackManager ? stackManager->size() : 0;
}

size_t DFS::getMaxStackDepth() const {
    return stackManager ? stackManager->getMaxStackSize() : 0;
}

void DFS::clearStack() {
    if (stackManager) {
        stackManager->clear();
    }
}

void DFS::printStackStatistics() const {
    if (stackManager) {
        stackManager->printStatistics();
    } else {
        std::cout << "[DFS] Stack manager not initialized" << std::endl;
    }
}