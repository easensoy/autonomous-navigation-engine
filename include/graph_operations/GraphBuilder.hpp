#pragma once
#include "../core/Graph.hpp"
#include <string>
#include <vector>

enum class GraphType {
    GRID,
    RANDOM,
    DELAUNAY,
    VISIBILITY,
    ROADMAP,
    CUSTOM
};

class GraphBuilder {
private:
    Graph* targetGraph;
    
    void buildGridGraph(int width, int height, double spacing);
    void buildRandomGraph(int nodeCount, double connectionProbability);
    void buildDelaunayGraph(const std::vector<std::pair<double, double>>& points);
    void buildVisibilityGraph(const std::vector<std::pair<double, double>>& points, const std::vector<std::vector<std::pair<double, double>>>& obstacles);

public:
    explicit GraphBuilder(Graph* graph);
    
    void buildGraph(GraphType type, const std::unordered_map<std::string, double>& parameters);
    void loadFromFile(const std::string& filename);
    void saveToFile(const std::string& filename) const;
    
    void addNodesFromCoordinates(const std::vector<std::pair<double, double>>& coordinates);
    void connectNearestNeighbors(int k);
    void connectWithinRadius(double radius);
    
    void generateWarehouseLayout(int width, int height);
    void generateCityRoadNetwork(int blockCount);
    void generateMazeLayout(int width, int height);
    
    bool loadFromJSON(const std::string& jsonFile);
    bool saveToJSON(const std::string& jsonFile) const;
    void importFromOSM(const std::string& osmFile);
    
    void validateGraphStructure() const;
    void optimizeGraphLayout();
    void generateGraphStatistics() const;
};