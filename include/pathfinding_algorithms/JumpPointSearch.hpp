#pragma once
#include "core/Graph.hpp"
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// Jump Point Search (Harabor and Grastien, 2011) for uniform-cost grid graphs, in the variant
// without corner cutting.
//
// Supported graphs. The graph is checked against the conditions below before a search, once per
// graph revision, grid size and movement mode. findPath and getJumpPoints throw
// std::invalid_argument naming the first violation; supportsGraph reports it without throwing.
//   1. Cells. Every node stands on an integer lattice point (x, y) with 0 <= x < width and
//      0 <= y < height, and no two nodes share a point.
//   2. Moves. The edges are exactly the grid moves between cells that hold nodes, each present in
//      both directions:
//        - orthogonal neighbours, weight s;
//        - with diagonal movement enabled (the default), diagonal neighbours whose two shared
//          orthogonal neighbours also hold nodes (no corner cutting), weight s * sqrt(2).
//      The same s > 0 applies to every orthogonal edge (relative tolerance 1e-9).
// Weighted cells, corner-cutting diagonals, longer edges and one-way edges are outside this domain;
// search such graphs with AStar or Dijkstra.
//
// Marked cells. A cell marked with addObstacle is closed: no route enters it, and no diagonal passes
// its corner. The graph itself is not modified.
//
// Guarantee. On a supported graph, findPath returns a least-cost route over the open cells; with no
// cells marked, its cost equals the least cost Dijkstra finds on the graph. The route is empty when
// the start or goal is unknown or stands on a marked cell, or when no route exists.
//
// Search. Best-first over jump points, ordered by f = g + h with h the octile distance (Manhattan
// distance when diagonal movement is disabled) times s, which is consistent on this domain. Ties on
// f prefer the larger g. Frontier entries made stale by a cheaper route are skipped by comparing g,
// and a closed cell reopens if a strictly cheaper route reaches it. Successors follow the
// no-corner-cutting rules: a straight jump stops at the goal or at a cell whose side neighbour is
// open while the cell behind that neighbour is closed (a forced neighbour); a diagonal jump has no
// forced neighbours and stops where a straight jump from it finds a jump point. With diagonal
// movement disabled, vertical jumps take the diagonal's role and stop where a horizontal jump from
// them finds a jump point.
class JumpPointSearch
{
    public:
        // Throws std::invalid_argument for a null graph or a non-positive grid size.
        JumpPointSearch(const Graph* environment, int width, int height);
        ~JumpPointSearch();

        // Least-cost route from startId to goalId as node ids, under the guarantee above.
        std::vector<int> findPath(int startId, int goalId);

        // True if the graph is in the supported domain for the current grid size and movement mode.
        // Otherwise false, and the first violation is stored in *reason when reason is not null.
        bool supportsGraph(std::string* reason = nullptr) const;

        // Throws std::invalid_argument for a non-positive size. Marked cells are kept.
        void setGridDimensions(int width, int height);
        // Closes or reopens a cell. addObstacle throws std::invalid_argument outside the grid.
        void addObstacle(int x, int y);
        void removeObstacle(int x, int y);
        void enableDiagonalMovement(bool enable);

        // Jump points generated when the search expands cell (startX, startY) without a parent, aiming
        // for the goal cell (goalX, goalY). Empty if the start cell is closed.
        std::vector<std::pair<int, int>> getJumpPoints(int startX, int startY, int goalX, int goalY);

        // Sum of the edge weights along a route. Throws std::invalid_argument if a step is not an edge.
        double getPathCost(const std::vector<int>& path) const;
        // Sum of Node::euclideanDistance along a route, which is 0.1 x the coordinate distance (the
        // library's COORDINATE_TO_METER_SCALE; see docs/TECHNICAL_NOTES.md, Units).
        double getPathLength(const std::vector<int>& path);

        // Jump points expanded by the most recent findPath call.
        std::size_t getLastSearchExpansions() const;

    private:
        // The graph laid onto the grid, rebuilt when the graph revision, grid size or movement mode
        // changes. problem is empty when the graph is in the supported domain.
        struct GridModel {
            bool built = false;
            std::uint64_t revision = 0;
            int width = 0;
            int height = 0;
            bool diagonal = true;
            std::string problem;
            std::vector<int> cellNode;  // node id on each cell (index y * width + x), -1 when empty
            double step = 1.0;          // weight s of an orthogonal edge
        };

        const Graph* graph;
        int gridWidth;
        int gridHeight;
        bool diagonalMovement = true;
        std::unordered_set<std::uint64_t> markedCells;  // packed (x, y), see packCell
        mutable GridModel model;
        std::vector<char> openCells;  // per search: a node stands on the cell and it is not marked
        std::size_t lastExpansions = 0;

        static std::uint64_t packCell(int x, int y);
        const GridModel& currentModel() const;
        void buildModel() const;
        void requireSupportedGraph() const;
        void refreshOpenCells();

        bool isOpen(int x, int y) const;
        bool canMove(int x, int y, int dx, int dy) const;
        bool hasForcedNeighbour(int x, int y, int dx, int dy) const;
        bool straightJumpFinds(int x, int y, int dx, int dy, int goalX, int goalY) const;
        bool jump(int x, int y, int dx, int dy, int goalX, int goalY, int& jumpX, int& jumpY) const;
        void successorDirections(int x, int y, int parentX, int parentY,
                                 std::vector<std::pair<int, int>>& directions) const;
        double heuristic(int x1, int y1, int x2, int y2) const;
        double segmentCost(int x1, int y1, int x2, int y2) const;
        std::vector<int> expandToRoute(const std::vector<int>& jumpCells) const;
};
