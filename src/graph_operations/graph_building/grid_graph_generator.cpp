#include "graph_operations/GraphBuilder.hpp"
#include <iostream>
#include <cmath>

class GridGraphGenerator {
private:
    Graph* graph;
    int width, height;
    double spacing;
    bool allowDiagonals;
    bool addObstacles;
    double obstacleRatio;
    
public:
    GridGraphGenerator(Graph* targetGraph, int w, int h, double s) 
        : graph(targetGraph), width(w), height(h), spacing(s), 
          allowDiagonals(false), addObstacles(false), obstacleRatio(0.0) {}
    
    void generateRegularGrid() {
        std::cout << "[GRID_GENERATOR] Creating " << width << "x" << height 
                  << " regular grid with spacing " << spacing << std::endl;
        
        // Create nodes
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                double posX = x * spacing;
                double posY = y * spacing;
                
                graph->addNode(nodeId, "grid_" + std::to_string(x) + "_" + std::to_string(y), 
                              posX, posY);
            }
        }
        
        // Create edges
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int nodeId = y * width + x;
                
                // Right neighbor
                if (x + 1 < width) {
                    int rightId = y * width + (x + 1);
                    graph->addEdge(nodeId, rightId, spacing, true);
                }
                
                // Bottom neighbor
                if (y + 1 < height) {
                    int bottomId = (y + 1) * width + x;
                    graph->addEdge(nodeId, bottomId, spacing, true);
                }
                
                // Diagonal neighbors if enabled
                if (allowDiagonals) {
                    double diagonalDist = spacing * std::sqrt(2.0);
                    
                    // Bottom-right diagonal
                    if (x + 1 < width && y + 1 < height) {
                        int diagId = (y + 1) * width + (x + 1);
                        graph->addEdge(nodeId, diagId, diagonalDist, true);
                    }
                    
                    // Bottom-left diagonal
                    if (x - 1 >= 0 && y + 1 < height) {
                        int diagId = (y + 1) * width + (x - 1);
                        graph->addEdge(nodeId, diagId, diagonalDist, true);
                    }
                }
            }
        }
        
        std::cout << "[GRID_GENERATOR] Generated grid with " << (width * height) 
                  << " nodes and " << graph->getEdgeCount() << " edges" << std::endl;
    }
    
    void generateHexagonalGrid() {
        std::cout << "[GRID_GENERATOR] Creating hexagonal grid" << std::endl;
        
        double hexWidth = spacing * std::sqrt(3.0);
        double hexHeight = spacing * 1.5;
        
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                int nodeId = row * width + col;
                
                // Offset every other row
                double offsetX = (row % 2) * (hexWidth / 2.0);
                double posX = col * hexWidth + offsetX;
                double posY = row * hexHeight;
                
                graph->addNode(nodeId, "hex_" + std::to_string(col) + "_" + std::to_string(row),
                              posX, posY);
            }
        }
        
        // Create hexagonal connections
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                int nodeId = row * width + col;
                
                // Right neighbor
                if (col + 1 < width) {
                    graph->addEdge(nodeId, nodeId + 1, spacing, true);
                }
                
                // Bottom neighbors (depends on row parity)
                if (row + 1 < height) {
                    int bottomLeft, bottomRight;
                    
                    if (row % 2 == 0) { // Even row
                        bottomLeft = (row + 1) * width + col;
                        bottomRight = (col + 1 < width) ? (row + 1) * width + col + 1 : -1;
                    } else { // Odd row
                        bottomLeft = (col > 0) ? (row + 1) * width + col - 1 : -1;
                        bottomRight = (row + 1) * width + col;
                    }
                    
                    if (bottomLeft >= 0 && bottomLeft < width * height) {
                        graph->addEdge(nodeId, bottomLeft, spacing, true);
                    }
                    if (bottomRight >= 0 && bottomRight < width * height) {
                        graph->addEdge(nodeId, bottomRight, spacing, true);
                    }
                }
            }
        }
        
        std::cout << "[GRID_GENERATOR] Generated hexagonal grid with " << (width * height) 
                  << " nodes" << std::endl;
    }
    
    void generateTriangularGrid() {
        std::cout << "[GRID_GENERATOR] Creating triangular grid" << std::endl;
        
        double triHeight = spacing * std::sqrt(3.0) / 2.0;
        
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                int nodeId = row * width + col;
                
                double posX = col * spacing + (row % 2) * (spacing / 2.0);
                double posY = row * triHeight;
                
                graph->addNode(nodeId, "tri_" + std::to_string(col) + "_" + std::to_string(row),
                              posX, posY);
            }
        }
        
        // Create triangular connections
        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                int nodeId = row * width + col;
                
                // Right neighbor
                if (col + 1 < width) {
                    graph->addEdge(nodeId, nodeId + 1, spacing, true);
                }
                
                // Diagonal connections based on row parity
                if (row + 1 < height) {
                    if (row % 2 == 0) {
                        // Even row: connect to bottom and bottom-right
                        graph->addEdge(nodeId, (row + 1) * width + col, spacing, true);
                        if (col + 1 < width) {
                            graph->addEdge(nodeId, (row + 1) * width + col + 1, spacing, true);
                        }
                    } else {
                        // Odd row: connect to bottom and bottom-left
                        graph->addEdge(nodeId, (row + 1) * width + col, spacing, true);
                        if (col > 0) {
                            graph->addEdge(nodeId, (row + 1) * width + col - 1, spacing, true);
                        }
                    }
                }
            }
        }
        
        std::cout << "[GRID_GENERATOR] Generated triangular grid" << std::endl;
    }
    
    void addRandomObstacles() {
        if (!addObstacles || obstacleRatio <= 0.0) return;
        
        int totalNodes = width * height;
        int obstaclesToAdd = static_cast<int>(totalNodes * obstacleRatio);
        
        std::cout << "[GRID_GENERATOR] Adding " << obstaclesToAdd << " random obstacles" << std::endl;
        
        std::vector<int> nodeIds;
        for (int i = 0; i < totalNodes; ++i) {
            nodeIds.push_back(i);
        }
        
        // Shuffle and remove first N nodes
        std::random_shuffle(nodeIds.begin(), nodeIds.end());
        
        for (int i = 0; i < obstaclesToAdd && i < totalNodes; ++i) {
            graph->removeNode(nodeIds[i]);
        }
        
        std::cout << "[GRID_GENERATOR] Obstacles added, " << graph->getNodeCount() 
                  << " nodes remain" << std::endl;
    }
    
    void enableDiagonals(bool enable) {
        allowDiagonals = enable;
    }
    
    void setObstacles(double ratio) {
        addObstacles = true;
        obstacleRatio = ratio;
    }
};

void GraphBuilder::buildGridGraph(int width, int height, double spacing) {
    std::cout << "[GRAPH_BUILDER] Building grid graph" << std::endl;
    
    if (!targetGraph) {
        std::cout << "[GRAPH_BUILDER] Cannot build graph - null target graph" << std::endl;
        return;
    }
    
    if (width <= 0 || height <= 0) {
        std::cout << "[GRAPH_BUILDER] Invalid grid dimensions: " << width << "x" << height << std::endl;
        return;
    }
    
    GridGraphGenerator generator(targetGraph, width, height, spacing);
    generator.generateRegularGrid();
    
    std::cout << "[GRAPH_BUILDER] Grid graph construction completed" << std::endl;
}

// Extended grid building functions
void buildGridWithDiagonals(Graph* graph, int width, int height, double spacing) {
    GridGraphGenerator generator(graph, width, height, spacing);
    generator.enableDiagonals(true);
    generator.generateRegularGrid();
}

void buildGridWithObstacles(Graph* graph, int width, int height, double spacing, double obstacleRatio) {
    GridGraphGenerator generator(graph, width, height, spacing);
    generator.setObstacles(obstacleRatio);
    generator.generateRegularGrid();
    generator.addRandomObstacles();
}

void buildHexagonalGrid(Graph* graph, int width, int height, double spacing) {
    GridGraphGenerator generator(graph, width, height, spacing);
    generator.generateHexagonalGrid();
}

void buildTriangularGrid(Graph* graph, int width, int height, double spacing) {
    GridGraphGenerator generator(graph, width, height, spacing);
    generator.generateTriangularGrid();
}

// Maze generation using grid
void generateMaze(Graph* graph, int width, int height, double spacing) {
    std::cout << "[GRID_GENERATOR] Generating maze using recursive backtracking" << std::endl;
    
    // Start with full grid
    GridGraphGenerator generator(graph, width, height, spacing);
    generator.generateRegularGrid();
    
    // Remove all edges initially
    std::vector<int> nodeIds = graph->getAllNodeIds();
    for (int nodeId : nodeIds) {
        std::vector<int> neighbors = graph->getNeighbors(nodeId);
        for (int neighbor : neighbors) {
            if (nodeId < neighbor) { // Avoid removing edge twice
                graph->removeEdge(nodeId, neighbor);
            }
        }
    }
    
    // Recursive backtracking maze generation
    std::vector<bool> visited(width * height, false);
    std::vector<int> stack;
    
    int current = 0;
    visited[current] = true;
    stack.push_back(current);
    
    while (!stack.empty()) {
        std::vector<int> unvisitedNeighbors;
        
        int x = current % width;
        int y = current / width;
        
        // Check all 4 directions
        std::vector<std::pair<int, int>> directions = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
        
        for (auto [dx, dy] : directions) {
            int nx = x + dx;
            int ny = y + dy;
            
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                int neighborId = ny * width + nx;
                if (!visited[neighborId]) {
                    unvisitedNeighbors.push_back(neighborId);
                }
            }
        }
        
        if (!unvisitedNeighbors.empty()) {
            // Choose random unvisited neighbor
            int next = unvisitedNeighbors[rand() % unvisitedNeighbors.size()];
            visited[next] = true;
            
            // Add edge between current and next
            graph->addEdge(current, next, spacing, true);
            
            stack.push_back(next);
            current = next;
        } else {
            stack.pop_back();
            if (!stack.empty()) {
                current = stack.back();
            }
        }
    }
    
    std::cout << "[GRID_GENERATOR] Maze generation completed" << std::endl;
}