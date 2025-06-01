#include "../../../include/pathfinding_algorithms/AStar.hpp"
#include <unordered_set>

class AStar::ClosedListManager {
private:
    std::unordered_set<int> closedSet;
    std::unordered_map<int, double> gScores;
    std::unordered_map<int, double> fScores;
    
public:
    void add(int nodeId, double gScore, double fScore) {
        closedSet.insert(nodeId);
        gScores[nodeId] = gScore;
        fScores[nodeId] = fScore;
    }
    
    bool contains(int nodeId) const {
        return closedSet.find(nodeId) != closedSet.end();
    }
    
    double getGScore(int nodeId) const {
        auto it = gScores.find(nodeId);
        return (it != gScores.end()) ? it->second : std::numeric_limits<double>::infinity();
    }
    
    double getFScore(int nodeId) const {
        auto it = fScores.find(nodeId);
        return (it != fScores.end()) ? it->second : std::numeric_limits<double>::infinity();
    }
    
    void remove(int nodeId) {
        closedSet.erase(nodeId);
        gScores.erase(nodeId);
        fScores.erase(nodeId);
    }
    
    size_t size() const {
        return closedSet.size();
    }
    
    void clear() {
        closedSet.clear();
        gScores.clear();
        fScores.clear();
    }
    
    std::vector<int> getAllNodes() const {
        return std::vector<int>(closedSet.begin(), closedSet.end());
    }
};

void AStar::initializeClosedList() {
    if (!closedListManager) {
        closedListManager = std::make_unique<ClosedListManager>();
    }
    closedListManager->clear();
}

void AStar::addToClosedList(int nodeId, double gScore, double fScore) {
    if (!closedListManager) {
        initializeClosedList();
    }
    closedListManager->add(nodeId, gScore, fScore);
}

bool AStar::isInClosedList(int nodeId) const {
    return closedListManager && closedListManager->contains(nodeId);
}

void AStar::removeFromClosedList(int nodeId) {
    if (closedListManager) {
        closedListManager->remove(nodeId);
    }
}

double AStar::getClosedListGScore(int nodeId) const {
    return closedListManager ? closedListManager->getGScore(nodeId) : 
           std::numeric_limits<double>::infinity();
}

size_t AStar::getClosedListSize() const {
    return closedListManager ? closedListManager->size() : 0;
}

std::vector<int> AStar::getExploredNodes() const {
    return closedListManager ? closedListManager->getAllNodes() : std::vector<int>();
}

void AStar::clearSearchState() {
    if (openListManager) {
        openListManager->clear();
    }
    if (closedListManager) {
        closedListManager->clear();
    }
}