#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_map>
#include <functional>
#include <memory>
#include <limits>
#include <queue>
#include <unordered_set>

class AStar {
private:
    const Graph* graph;
    std::function<double(int, int)> heuristicFunction;
    
    struct AStarNode {
        int nodeId;
        double gScore;
        double fScore;
        int parent;
        
        AStarNode(int id, double g, double f, int p = -1);
        bool operator>(const AStarNode& other) const;
    };

    class OpenListManager {
    private:
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openQueue;
        std::unordered_set<int> openSet;
        
    public:
        void push(const AStarNode& node);
        AStarNode pop();
        bool empty() const;
        bool contains(int nodeId) const;
        void remove(int nodeId);
        size_t size() const;
        void clear();
    };
    
    class ClosedListManager {
    private:
        std::unordered_set<int> closedSet;
        std::unordered_map<int, double> gScores;
        std::unordered_map<int, double> fScores;
        
    public:
        void add(int nodeId, double gScore, double fScore);
        bool contains(int nodeId) const;
        double getGScore(int nodeId) const;
        double getFScore(int nodeId) const;
        void remove(int nodeId);
        size_t size() const;
        void clear();
        std::vector<int> getAllNodes() const;
    };
    
    std::unique_ptr<OpenListManager> openListManager;
    std::unique_ptr<ClosedListManager> closedListManager;
    
    mutable size_t nodesExplored;
    mutable double lastSearchTime;
    mutable bool searchStatisticsEnabled;
    
    double calculateHeuristic(int nodeId, int goalId) const;
    std::vector<int> reconstructPath(const std::unordered_map<int, int>& cameFrom, int current) const;
    
    void initializeOpenList();
    void addToOpenList(const AStarNode& node);
    AStarNode getNextFromOpenList();
    bool isOpenListEmpty() const;
    bool isInOpenList(int nodeId) const;
    void removeFromOpenList(int nodeId);
    size_t getOpenListSize() const;
    
    void initializeClosedList();
    void addToClosedList(int nodeId, double gScore, double fScore);
    bool isInClosedList(int nodeId) const;
    void removeFromClosedList(int nodeId);
    double getClosedListGScore(int nodeId) const;
    size_t getClosedListSize() const;
    std::vector<int> getExploredNodes() const;
    
    void clearSearchState();
    void updateSearchStatistics() const;

public:
    /**
     * Constructs A* algorithm instance with specified graph environment.
     * @param environment Pointer to graph representing the navigation environment
     * @throws std::invalid_argument if environment pointer is null
     */
    explicit AStar(const Graph* environment);
    
    ~AStar() = default;
    
    AStar(const AStar& other) = delete;
    AStar& operator=(const AStar& other) = delete;
    AStar(AStar&& other) noexcept = default;
    AStar& operator=(AStar&& other) noexcept = default;
    
    /**
     * Configures the heuristic function used for pathfinding optimization.
     * @param heuristic Function that estimates cost between two nodes
     * @throws std::invalid_argument if heuristic function is null
     */
    void setHeuristicFunction(std::function<double(int, int)> heuristic);
    
    /**
     * Finds optimal path between specified start and goal nodes using A* algorithm.
     * @param startId Identifier of the starting node
     * @param goalId Identifier of the destination node
     * @return Vector of node identifiers representing the optimal path
     * @throws std::invalid_argument if start or goal nodes do not exist
     */
    std::vector<int> findPath(int startId, int goalId) const;
    
    /**
     * Finds optimal path using a custom heuristic function for this search only.
     * @param startId Identifier of the starting node
     * @param goalId Identifier of the destination node
     * @param customHeuristic Temporary heuristic function for this search
     * @return Vector of node identifiers representing the optimal path
     * @throws std::invalid_argument if parameters are invalid
     */
    std::vector<int> findPathWithHeuristic(int startId, int goalId, 
                                         std::function<double(int, int)> customHeuristic);
    
    /**
     * Calculates the total cost of traversing the specified path.
     * @param path Vector of node identifiers representing a path
     * @return Total cost of path traversal
     * @throws std::runtime_error if path contains invalid edges
     */
    double getPathCost(const std::vector<int>& path) const;
    
    void printSearchStatistics() const;
    
    double manhattanHeuristic(int nodeId, int goalId) const;
    double euclideanHeuristic(int nodeId, int goalId) const;
    double chebyshevHeuristic(int nodeId, int goalId) const;
    
    /**
     * Validates whether the configured heuristic function is admissible.
     * @param startId Starting node for validation
     * @param goalId Goal node for validation
     * @return True if heuristic is admissible, false otherwise
     */
    bool isHeuristicAdmissible(int startId, int goalId) const;
    
    size_t getLastSearchNodesExplored() const;
    double getLastSearchExecutionTime() const;
    void enableSearchStatistics(bool enable);
    
    /**
     * Retrieves list of all nodes explored during the most recent search.
     * @return Vector of node identifiers that were examined during search
     */
    std::vector<int> getLastSearchExploredNodes() const;
    
    /**
     * Estimates memory usage for the current search state.
     * @return Approximate memory usage in bytes
     */
    size_t estimateMemoryUsage() const;
    
    void resetSearchState();
};