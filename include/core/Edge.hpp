#pragma once
#include <ostream>

class Edge 
{
    private:
        int fromNodeId;
        int toNodeId;
        double weight;
        bool bidirectional;
    
    public:
        Edge(int from, int to, double edgeWeight, bool isBidirectional = true);
        Edge(const Edge& other) = default;
        Edge& operator = (const Edge& other) = default;
        ~Edge() = default;

        // Getters
        int getFromNode() const;
        int getToNode() const;
        double getWeight() const;
        bool isBidirectional() const;

        // Utilities
        bool connectsNodes(int nodeA, int nodeB) const;
        int getOtherNode(int nodeId) const;

        // Operators
        bool operator==(const Edge& other) const;
        bool operator<(const Edge& other) const;

        friend std::ostream& operator<<(std::ostream& os, const Edge& edge);
};