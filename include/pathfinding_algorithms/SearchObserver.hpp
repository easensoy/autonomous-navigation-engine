#pragma once

// An opt-in, read-only view of a search while it runs. AStar, Dijkstra and BFS call an attached
// observer at the points where they do the work, so the events arrive in execution order.
// No observer is attached by default; each hook is then one null-pointer check.
// The observer must outlive the searches it watches and must not change the graph.
class SearchObserver
{
    public:
        virtual ~SearchObserver() = default;

        // A node was taken off the frontier and is expanded. A* and Dijkstra call this when a node
        // is settled at its best known cost (A* also for the goal, just before it stops); BFS calls
        // it when a node is dequeued. costSoFar is the weighted cost of the route that reached the
        // node (A*, Dijkstra) or its depth in edges (BFS); priority is the value the frontier
        // ordered it by, as in onFrontierPush.
        virtual void onExpand(int /*nodeId*/, double /*costSoFar*/, double /*priority*/) {}

        // A node was put on the frontier via parentId (-1 for the start), with the cost of the route
        // that reached it and the priority the frontier orders it by: g + h for A*, g for Dijkstra,
        // the depth in edges for BFS.
        virtual void onFrontierPush(int /*nodeId*/, int /*parentId*/, double /*costSoFar*/,
                                    double /*priority*/) {}
};
