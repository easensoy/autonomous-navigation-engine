#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <string>
#include <utility>

// Jump Point Search on a grid laid over the graph. Each node stands on the integer cell nearest its
// coordinates; cells with no node, or marked with addObstacle, are blocked. A route is returned node
// by node, and only if every step is one of the graph's edges, so the graph should be an 8-connected
// grid whose edge costs match the step lengths.
// Known limitation: routes are valid but not guaranteed shortest, and some routes are missed. The
// straight and diagonal jump rules come from different JPS variants (without and with corner
// cutting), and with diagonal movement disabled, goals off the start's row and column are missed.
// Use A* when the shortest route matters.
class JumpPointSearch
{
    private:
        const Graph* graph;
        int gridWidth;
        int gridHeight;
        std::unordered_set<std::string> obstacles;
        bool diagonalMovement;
        std::unordered_map<long long, int> cellNodes;  // node on each open cell, rebuilt per search
        
        class JumpPointIdentifier;
        class GridPruningManager;
        
        std::unique_ptr<JumpPointIdentifier> jumpPointIdentifier;
        std::unique_ptr<GridPruningManager> pruningManager;

        struct GridNode
        {
            int x, y;
            double gScore, fScore;
            int parentX, parentY;

            GridNode(int xPos, int yPos);
            bool operator>(const GridNode& other) const;
        };

        bool isValidPosition(int x, int y) const;
        bool hasObstacle(int x, int y) const;
        long long cellKey(int x, int y) const;
        bool mapGraphOntoGrid();  // false if two nodes share a cell
        std::vector<int> expandToGraphRoute(const std::vector<std::pair<int, int>>& jumpPoints) const;
        std::vector<std::pair<int, int>> getNeighbors(int x, int y) const;
        std::pair<int, int> jump(int x, int y, int dx, int dy, int goalX, int goalY) const;
        double heuristic(int x1, int y1, int x2, int y2) const;
        std::vector<std::pair<int, int>> findJumpPoints(int x, int y, int goalX, int goalY) const;
        std::vector<int> reconstructGridPath(const std::unordered_map<std::string, GridNode>& allNodes,
                                           int startX, int startY, int goalX, int goalY) const;
        
        void initializeJumpPointIdentification();
        std::pair<int, int> identifyJumpPoint(int x, int y, int dx, int dy, int goalX, int goalY);
        std::vector<std::pair<int, int>> findAllJumpPointsInPath(int startX, int startY, int goalX, int goalY);
        bool isNodeJumpPoint(int x, int y, int parentX, int parentY, int goalX, int goalY);
        std::vector<std::pair<int, int>> identifyForcedNeighborsAtPosition(int x, int y, int dx, int dy);
        void analyzeJumpPointDistribution();
        double calculateJumpPointDensity();
        void optimizeJumpPointIdentification(bool enable);
        void clearJumpPointCache();
        std::vector<std::pair<int, int>> getJumpPointsInRegion(int minX, int minY, int maxX, int maxY);
        
        void initializeGridPruning();
        std::vector<std::pair<int, int>> pruneNeighbors(int x, int y, int parentX, int parentY);
        std::vector<std::pair<int, int>> pruneDiagonalMovement(int x, int y, int dx, int dy);
        std::vector<std::pair<int, int>> pruneCardinalMovement(int x, int y, int dx, int dy);
        std::vector<std::pair<int, int>> identifyForcedNeighbors(int x, int y, int dx, int dy);
        bool shouldPruneNode(int x, int y, int parentX, int parentY, int goalX, int goalY);
        double calculatePruningEfficiency();
        void enableSymmetryBreaking(bool enable);
        void resetPruningState();
        void analyzePruningPatterns();

    public:
        JumpPointSearch(const Graph* environment, int width, int height);
        ~JumpPointSearch();  // defined in jps_core.cpp, where the helper classes are complete

        std::vector<int> findPath(int startId, int goalId);
        void setGridDimensions(int width, int height);
        void addObstacle(int x, int y);
        void removeObstacle(int x, int y);

        std::vector<std::pair<int, int>> getJumpPoints(int startX, int startY, int goalX, int goalY);
        double getPathLength(const std::vector<int>& path);
        void enableDiagonalMovement(bool enable);
};