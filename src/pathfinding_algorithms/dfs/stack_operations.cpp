#include "pathfinding_algorithms/DFS.hpp"
#include "dfs_internals.hpp"
#include <stack>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

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