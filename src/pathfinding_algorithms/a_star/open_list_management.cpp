#include "pathfinding_algorithms/AStar.hpp"
#include <queue>
#include <stdexcept>
#include <unordered_set>
#include <memory>

// AStar::OpenListManager is declared in AStar.hpp; its members are defined here.

void AStar::OpenListManager::push(const AStarNode& node) {
    openQueue.push(node);
    openSet.insert(node.nodeId);
}

AStar::AStarNode AStar::OpenListManager::pop() {
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

bool AStar::OpenListManager::empty() const {
    return openQueue.empty() || openSet.empty();
}

bool AStar::OpenListManager::contains(int nodeId) const {
    return openSet.find(nodeId) != openSet.end();
}

void AStar::OpenListManager::remove(int nodeId) {
    openSet.erase(nodeId);
}

size_t AStar::OpenListManager::size() const {
    return openSet.size();
}

void AStar::OpenListManager::clear() {
    while (!openQueue.empty()) {
        openQueue.pop();
    }
    openSet.clear();
}

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
