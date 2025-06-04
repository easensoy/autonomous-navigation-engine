#include "reporting/PathVisualiser.hpp"
#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <iomanip>

class ASCIIGraphRenderer {
private:
    const Graph* graph;
    int canvasWidth;
    int canvasHeight;
    double scaleFactor;
    
    struct CanvasPoint {
        int x, y;
        char symbol;
        
        CanvasPoint(int xPos, int yPos, char sym) : x(xPos), y(yPos), symbol(sym) {}
    };
    
    struct BoundingBox {
        double minX, maxX, minY, maxY;
        
        BoundingBox() : minX(1e9), maxX(-1e9), minY(1e9), maxY(-1e9) {}
        
        void update(double x, double y) {
            minX = std::min(minX, x);
            maxX = std::max(maxX, x);
            minY = std::min(minY, y);
            maxY = std::max(maxY, y);
        }
        
        double width() const { return maxX - minX; }
        double height() const { return maxY - minY; }
    };
    
public:
    ASCIIGraphRenderer(const Graph* environment, int width = 80, int height = 40, double scale = 1.0)
        : graph(environment), canvasWidth(width), canvasHeight(height), scaleFactor(scale) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[ASCII_RENDERER] ASCII graph renderer initialized (" 
                  << width << "x" << height << ")" << std::endl;
    }
    
    std::string renderGraph(const std::vector<int>& highlightedPath = {}) {
        std::cout << "[ASCII_RENDERER] Rendering graph with " << graph->getNodeCount() 
                  << " nodes" << std::endl;
        
        if (graph->getNodeCount() == 0) {
            return "Empty graph - no nodes to display\n";
        }
        
        // Calculate bounding box
        BoundingBox bounds = calculateBounds();
        
        // Create canvas
        std::vector<std::vector<char>> canvas(canvasHeight, 
                                            std::vector<char>(canvasWidth, ' '));
        
        // Draw edges first (so nodes appear on top)
        drawEdges(canvas, bounds, highlightedPath);
        
        // Draw nodes
        drawNodes(canvas, bounds, highlightedPath);
        
        // Convert canvas to string
        return canvasToString(canvas, bounds);
    }
    
    std::string renderPathOnly(const std::vector<int>& path) {
        std::cout << "[ASCII_RENDERER] Rendering path with " << path.size() << " nodes" << std::endl;
        
        if (path.empty()) {
            return "Empty path - nothing to display\n";
        }
        
        // Calculate bounding box for path nodes only
        BoundingBox bounds;
        for (int nodeId : path) {
            if (graph->hasNode(nodeId)) {
                const Node& node = graph->getNode(nodeId);
                bounds.update(node.getX(), node.getY());
            }
        }
        
        // Add some padding
        double padding = std::max(bounds.width(), bounds.height()) * 0.1;
        bounds.minX -= padding;
        bounds.maxX += padding;
        bounds.minY -= padding;
        bounds.maxY += padding;
        
        // Create canvas
        std::vector<std::vector<char>> canvas(canvasHeight, 
                                            std::vector<char>(canvasWidth, ' '));
        
        // Draw path
        drawPath(canvas, bounds, path);
        
        return canvasToString(canvas, bounds);
    }
    
    void setCanvasDimensions(int width, int height) {
        canvasWidth = width;
        canvasHeight = height;
        std::cout << "[ASCII_RENDERER] Canvas dimensions set to " << width << "x" << height << std::endl;
    }
    
    void setScaleFactor(double scale) {
        scaleFactor = scale;
        std::cout << "[ASCII_RENDERER] Scale factor set to " << scale << std::endl;
    }
    
private:
    BoundingBox calculateBounds() {
        BoundingBox bounds;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            bounds.update(node.getX(), node.getY());
        }
        
        // Add padding
        double padding = std::max(bounds.width(), bounds.height()) * 0.1;
        bounds.minX -= padding;
        bounds.maxX += padding;
        bounds.minY -= padding;
        bounds.maxY += padding;
        
        return bounds;
    }
    
    std::pair<int, int> worldToCanvas(double worldX, double worldY, const BoundingBox& bounds) {
        int canvasX = static_cast<int>((worldX - bounds.minX) / bounds.width() * (canvasWidth - 1));
        int canvasY = static_cast<int>((bounds.maxY - worldY) / bounds.height() * (canvasHeight - 1));
        
        canvasX = std::max(0, std::min(canvasWidth - 1, canvasX));
        canvasY = std::max(0, std::min(canvasHeight - 1, canvasY));
        
        return {canvasX, canvasY};
    }
    
    void drawNodes(std::vector<std::vector<char>>& canvas, const BoundingBox& bounds,
                   const std::vector<int>& highlightedPath) {
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            auto [canvasX, canvasY] = worldToCanvas(node.getX(), node.getY(), bounds);
            
            char symbol = '*';
            
            // Use different symbols for highlighted path
            if (!highlightedPath.empty()) {
                auto it = std::find(highlightedPath.begin(), highlightedPath.end(), nodeId);
                if (it != highlightedPath.end()) {
                    if (it == highlightedPath.begin()) {
                        symbol = 'S'; // Start
                    } else if (it == highlightedPath.end() - 1) {
                        symbol = 'G'; // Goal
                    } else {
                        symbol = 'o'; // Path node
                    }
                } else {
                    symbol = '.'; // Regular node
                }
            }
            
            canvas[canvasY][canvasX] = symbol;
        }
    }
    
    void drawEdges(std::vector<std::vector<char>>& canvas, const BoundingBox& bounds,
                   const std::vector<int>& highlightedPath) {
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            const Node& fromNode = graph->getNode(nodeId);
            
            for (const Edge& edge : edges) {
                if (!graph->hasNode(edge.getToNode())) continue;
                
                const Node& toNode = graph->getNode(edge.getToNode());
                
                char edgeSymbol = '-';
                
                // Check if edge is part of highlighted path
                bool isPathEdge = false;
                if (!highlightedPath.empty()) {
                    for (size_t i = 1; i < highlightedPath.size(); ++i) {
                        if ((highlightedPath[i-1] == nodeId && highlightedPath[i] == edge.getToNode()) ||
                            (highlightedPath[i-1] == edge.getToNode() && highlightedPath[i] == nodeId)) {
                            isPathEdge = true;
                            edgeSymbol = '=';
                            break;
                        }
                    }
                }
                
                if (!isPathEdge && !highlightedPath.empty()) {
                    edgeSymbol = '.'; // Dimmed edge
                }
                
                drawLine(canvas, bounds, fromNode.getX(), fromNode.getY(),
                        toNode.getX(), toNode.getY(), edgeSymbol);
            }
        }
    }
    
    void drawPath(std::vector<std::vector<char>>& canvas, const BoundingBox& bounds,
                  const std::vector<int>& path) {
        
        // Draw path edges
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) continue;
            
            const Node& from = graph->getNode(path[i-1]);
            const Node& to = graph->getNode(path[i]);
            
            drawLine(canvas, bounds, from.getX(), from.getY(), to.getX(), to.getY(), '=');
        }
        
        // Draw path nodes
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) continue;
            
            const Node& node = graph->getNode(path[i]);
            auto [canvasX, canvasY] = worldToCanvas(node.getX(), node.getY(), bounds);
            
            char symbol;
            if (i == 0) {
                symbol = 'S'; // Start
            } else if (i == path.size() - 1) {
                symbol = 'G'; // Goal
            } else {
                symbol = 'o'; // Waypoint
            }
            
            canvas[canvasY][canvasX] = symbol;
        }
    }
    
    void drawLine(std::vector<std::vector<char>>& canvas, const BoundingBox& bounds,
                  double x1, double y1, double x2, double y2, char symbol) {
        
        auto [startX, startY] = worldToCanvas(x1, y1, bounds);
        auto [endX, endY] = worldToCanvas(x2, y2, bounds);
        
        // Bresenham's line algorithm
        int dx = std::abs(endX - startX);
        int dy = std::abs(endY - startY);
        int sx = (startX < endX) ? 1 : -1;
        int sy = (startY < endY) ? 1 : -1;
        int err = dx - dy;
        
        int x = startX, y = startY;
        
        while (true) {
            if (x >= 0 && x < canvasWidth && y >= 0 && y < canvasHeight) {
                if (canvas[y][x] == ' ') {
                    canvas[y][x] = symbol;
                }
            }
            
            if (x == endX && y == endY) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x += sx;
            }
            if (e2 < dx) {
                err += dx;
                y += sy;
            }
        }
    }
    
    std::string canvasToString(const std::vector<std::vector<char>>& canvas, 
                              const BoundingBox& bounds) {
        std::ostringstream result;
        
        // Add header
        result << "ASCII Graph Visualization (" << canvasWidth << "x" << canvasHeight << ")\n";
        result << "Bounds: (" << std::fixed << std::setprecision(1) 
               << bounds.minX << "," << bounds.minY << ") to (" 
               << bounds.maxX << "," << bounds.maxY << ")\n";
        result << "Legend: S=Start, G=Goal, o=Path, *=Node, ==Path, --Edge\n";
        result << std::string(canvasWidth + 2, '-') << "\n";
        
        // Add canvas content
        for (int y = 0; y < canvasHeight; ++y) {
            result << "|";
            for (int x = 0; x < canvasWidth; ++x) {
                result << canvas[y][x];
            }
            result << "|\n";
        }
        
        result << std::string(canvasWidth + 2, '-') << "\n";
        
        return result.str();
    }
};

// Global ASCII renderer
static std::unique_ptr<ASCIIGraphRenderer> g_asciiRenderer;

void initializeASCIIRenderer(const Graph* graph, int width, int height, double scale) {
    g_asciiRenderer = std::make_unique<ASCIIGraphRenderer>(graph, width, height, scale);
}

std::string renderGraphASCII(const Graph* graph, const std::vector<int>& highlightPath, 
                            int width, int height) {
    if (!g_asciiRenderer) {
        g_asciiRenderer = std::make_unique<ASCIIGraphRenderer>(graph, width, height);
    }
    return g_asciiRenderer->renderGraph(highlightPath);
}

std::string renderPathASCII(const Graph* graph, const std::vector<int>& path, 
                           int width, int height) {
    if (!g_asciiRenderer) {
        g_asciiRenderer = std::make_unique<ASCIIGraphRenderer>(graph, width, height);
    }
    return g_asciiRenderer->renderPathOnly(path);
}

void setASCIICanvasDimensions(int width, int height) {
    if (g_asciiRenderer) {
        g_asciiRenderer->setCanvasDimensions(width, height);
    }
}