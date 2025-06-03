#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <memory>

class JumpPointSearch
{
    private:
        const Graph* graph;
        int gridWidth;
        int gridHeight;
        std::unordered_set<std::string> obstacles;
        bool diagonalMovement;
        
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

        std::vector<int> findPath(int startId, int goalId);
        void setGridDimensions(int width, int height);
        void addObstacle(int x, int y);
        void removeObstacle(int x, int y);

        std::vector<std::pair<int, int>> getJumpPoints(int startX, int startY, int goalX, int goalY);
        double getPathLength(const std::vector<int>& path);
        void enableDiagonalMovement(bool enable);
};