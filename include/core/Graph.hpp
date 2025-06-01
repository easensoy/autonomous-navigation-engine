#pragma once
#include "Node.hpp"
#include "Edge.hpp"
#include <unordered_map>
#include <vector>
#include <memory>

class Graph
{
    private:
        std::unordered_map<int, std::unique_ptr<Node>> nodes;
        std::unordered_map<int, std::vector<Edge>> adjacencyList;

        void validateNodeExists(int nodeId) const;
    
    public:
        Graph();
        ~Graph() = default;

        Graph(const Graph& other);
        Graph& operator = (const Graph& other);
        Graph(Graph&& other) noexcept;
        Graph& operator=(Graph&& other) noexcept;

        // Node operations
        void addNode(int id, const std::string& name, double x = 0.0, double y = 0.0);
        void addNode(const Node& node);
        bool removeNode(int nodeId);
        bool hasNode(int nodeId) const;
        const Node& getNode(int nodeId) const;
        Node& getNode(int nodeId);

        void addEdge(int fromId, int toId, double weight, bool bidirectional = true);
        void addEdge(const Edge& edge);
        bool removeEdge(int fromId, int toId);
        bool hasEdge(int fromId, int toId) const;

        const std::vector<Edge>& getEdgesFrom(int nodeId) const;
        std::vector<int> getNeighbors(int nodeId) const;
        size_t getNodeCount() const;
        size_t getEdgeCount() const;

        void clear();
        bool isEmpty() const;
        std::vector<int> getAllNodeIds() const;
        double getEdgeWeight(int fromId, int toId) const;

        bool isConnected() const;
        void printGraphStructure() const;

        auto begin() const { return nodes.begin(); }
        auto end() const { return nodes.end(); }
};