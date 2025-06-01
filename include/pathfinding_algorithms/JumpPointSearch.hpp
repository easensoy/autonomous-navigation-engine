#pragma once
#include "../core/Graph.hpp"
#include <vector>
#include <unordered_set>

class JumpPointSearch
{
    private:
        const Graph* graph;
        int gridWith;
        int gridHeight;

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