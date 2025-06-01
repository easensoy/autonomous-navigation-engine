#include "../../../include/core/Graph.hpp"
#include <iostream>

Graph::Graph() {
}

Graph::Graph(const Graph& other) {
    for (const auto& pair : other.nodes) {
        nodes[pair.first] = std::make_unique<Node>(*pair.second);
    }
    adjacencyList = other.adjacencyList;
}

Graph& Graph::operator=(const Graph& other) {
    if (this != &other) {
        clear();
        for (const auto& pair : other.nodes) {
            nodes[pair.first] = std::make_unique<Node>(*pair.second);
        }
        adjacencyList = other.adjacencyList;
    }
    return *this;
}

Graph::Graph(Graph&& other) noexcept
    : nodes(std::move(other.nodes)), adjacencyList(std::move(other.adjacencyList)) {
}

Graph& Graph::operator=(Graph&& other) noexcept {
    if (this != &other) {
        nodes = std::move(other.nodes);
        adjacencyList = std::move(other.adjacencyList);
    }
    return *this;
}