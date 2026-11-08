#include "pathfinding_algorithms/AStar.hpp"
#include <queue>
#include <stdexcept>
#include <memory>

// AStar::OpenListManager is declared in AStar.hpp; its members are defined here.
// The open list is a plain priority queue with lazy deletion: a node gets one entry per cheaper
// route found, and findPath() skips the entries that a cheaper one has made stale. The queue does
// not try to tell live entries from stale ones itself; when it did (by erasing the node from an
// open set on its first pop), a stale entry tied with the live one on f could hide it.

void AStar::OpenListManager::push(const AStarNode& node) {
    openQueue.push(node);
}

AStar::AStarNode AStar::OpenListManager::pop() {
    if (openQueue.empty()) {
        throw std::runtime_error("Attempted to pop from empty open list");
    }
    AStarNode current = openQueue.top();
    openQueue.pop();
    return current;
}

bool AStar::OpenListManager::empty() const {
    return openQueue.empty();
}

size_t AStar::OpenListManager::size() const {
    return openQueue.size();
}

void AStar::OpenListManager::clear() {
    while (!openQueue.empty()) {
        openQueue.pop();
    }
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

size_t AStar::getOpenListSize() const {
    return openListManager ? openListManager->size() : 0;
}
