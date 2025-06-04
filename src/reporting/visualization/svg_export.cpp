#include "reporting/PathVisualiser.hpp"
#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cmath>

class SVGExporter {
private:
    const Graph* graph;
    double canvasWidth;
    double canvasHeight;
    double scaleFactor;
    double padding;
    
    struct SVGStyle {
        std::string nodeColor = "#4A90E2";
        std::string nodeStroke = "#2E5C8A";
        std::string pathNodeColor = "#F5A623";
        std::string startNodeColor = "#7ED321";
        std::string goalNodeColor = "#D0021B";
        std::string edgeColor = "#8E8E93";
        std::string pathEdgeColor = "#F5A623";
        std::string backgroundColor = "#FFFFFF";
        std::string textColor = "#333333";
        
        double nodeRadius = 6.0;
        double nodeStrokeWidth = 2.0;
        double edgeStrokeWidth = 1.5;
        double pathEdgeStrokeWidth = 3.0;
        std::string fontFamily = "Arial, sans-serif";
        double fontSize = 12.0;
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
    
    SVGStyle style;
    
public:
    SVGExporter(const Graph* environment, double width = 800, double height = 600, double scale = 1.0)
        : graph(environment), canvasWidth(width), canvasHeight(height), 
          scaleFactor(scale), padding(50.0) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        std::cout << "[SVG_EXPORTER] SVG exporter initialized (" 
                  << width << "x" << height << ")" << std::endl;
    }
    
    bool exportGraph(const std::string& filename, const std::vector<int>& highlightPath = {}) {
        std::cout << "[SVG_EXPORTER] Exporting graph to " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[SVG_EXPORTER] Failed to open file: " << filename << std::endl;
            return false;
        }
        
        BoundingBox bounds = calculateBounds();
        std::string svgContent = generateSVG(bounds, highlightPath);
        
        file << svgContent;
        file.close();
        
        std::cout << "[SVG_EXPORTER] Graph exported successfully" << std::endl;
        return true;
    }
    
    bool exportPath(const std::string& filename, const std::vector<int>& path) {
        std::cout << "[SVG_EXPORTER] Exporting path to " << filename << std::endl;
        
        std::ofstream file(filename);
        if (!file.is_open()) {
            std::cout << "[SVG_EXPORTER] Failed to open file: " << filename << std::endl;
            return false;
        }
        
        BoundingBox bounds = calculatePathBounds(path);
        std::string svgContent = generatePathSVG(bounds, path);
        
        file << svgContent;
        file.close();
        
        std::cout << "[SVG_EXPORTER] Path exported successfully" << std::endl;
        return true;
    }
    
    std::string generateSVGString(const std::vector<int>& highlightPath = {}) {
        BoundingBox bounds = calculateBounds();
        return generateSVG(bounds, highlightPath);
    }
    
    void setCanvasSize(double width, double height) {
        canvasWidth = width;
        canvasHeight = height;
        std::cout << "[SVG_EXPORTER] Canvas size set to " << width << "x" << height << std::endl;
    }
    
    void setNodeStyle(const std::string& color, double radius, double strokeWidth = 2.0) {
        style.nodeColor = color;
        style.nodeRadius = radius;
        style.nodeStrokeWidth = strokeWidth;
    }
    
    void setPathStyle(const std::string& nodeColor, const std::string& edgeColor, double edgeWidth = 3.0) {
        style.pathNodeColor = nodeColor;
        style.pathEdgeColor = edgeColor;
        style.pathEdgeStrokeWidth = edgeWidth;
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
        double extraPadding = std::max(bounds.width(), bounds.height()) * 0.1;
        bounds.minX -= extraPadding;
        bounds.maxX += extraPadding;
        bounds.minY -= extraPadding;
        bounds.maxY += extraPadding;
        
        return bounds;
    }
    
    BoundingBox calculatePathBounds(const std::vector<int>& path) {
        BoundingBox bounds;
        
        for (int nodeId : path) {
            if (graph->hasNode(nodeId)) {
                const Node& node = graph->getNode(nodeId);
                bounds.update(node.getX(), node.getY());
            }
        }
        
        // Add padding
        double extraPadding = std::max(bounds.width(), bounds.height()) * 0.1;
        bounds.minX -= extraPadding;
        bounds.maxX += extraPadding;
        bounds.minY -= extraPadding;
        bounds.maxY += extraPadding;
        
        return bounds;
    }
    
    std::pair<double, double> worldToSVG(double worldX, double worldY, const BoundingBox& bounds) {
        double scaleX = (canvasWidth - 2 * padding) / bounds.width();
        double scaleY = (canvasHeight - 2 * padding) / bounds.height();
        double scale = std::min(scaleX, scaleY) * scaleFactor;
        
        double svgX = padding + (worldX - bounds.minX) * scale;
        double svgY = padding + (bounds.maxY - worldY) * scale; // Flip Y-axis
        
        return {svgX, svgY};
    }
    
    std::string generateSVG(const BoundingBox& bounds, const std::vector<int>& highlightPath) {
        std::ostringstream svg;
        
        // SVG header
        svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        svg << "<svg width=\"" << canvasWidth << "\" height=\"" << canvasHeight << "\" "
            << "xmlns=\"http://www.w3.org/2000/svg\">\n";
        
        // Background
        svg << "  <rect width=\"100%\" height=\"100%\" fill=\"" << style.backgroundColor << "\"/>\n";
        
        // Title
        svg << "  <text x=\"" << canvasWidth/2 << "\" y=\"30\" "
            << "text-anchor=\"middle\" font-family=\"" << style.fontFamily << "\" "
            << "font-size=\"18\" font-weight=\"bold\" fill=\"" << style.textColor << "\">"
            << "Graph Visualization</text>\n";
        
        // Edges
        drawEdges(svg, bounds, highlightPath);
        
        // Nodes
        drawNodes(svg, bounds, highlightPath);
        
        // Legend
        drawLegend(svg, !highlightPath.empty());
        
        svg << "</svg>\n";
        
        return svg.str();
    }
    
    std::string generatePathSVG(const BoundingBox& bounds, const std::vector<int>& path) {
        std::ostringstream svg;
        
        // SVG header
        svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        svg << "<svg width=\"" << canvasWidth << "\" height=\"" << canvasHeight << "\" "
            << "xmlns=\"http://www.w3.org/2000/svg\">\n";
        
        // Background
        svg << "  <rect width=\"100%\" height=\"100%\" fill=\"" << style.backgroundColor << "\"/>\n";
        
        // Title
        svg << "  <text x=\"" << canvasWidth/2 << "\" y=\"30\" "
            << "text-anchor=\"middle\" font-family=\"" << style.fontFamily << "\" "
            << "font-size=\"18\" font-weight=\"bold\" fill=\"" << style.textColor << "\">"
            << "Path Visualization</text>\n";
        
        // Path edges
        drawPathEdges(svg, bounds, path);
        
        // Path nodes
        drawPathNodes(svg, bounds, path);
        
        // Path info
        drawPathInfo(svg, path);
        
        svg << "</svg>\n";
        
        return svg.str();
    }
    
    void drawEdges(std::ostringstream& svg, const BoundingBox& bounds, const std::vector<int>& highlightPath) {
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            const Node& fromNode = graph->getNode(nodeId);
            
            for (const Edge& edge : edges) {
                if (!graph->hasNode(edge.getToNode())) continue;
                
                const Node& toNode = graph->getNode(edge.getToNode());
                
                auto [x1, y1] = worldToSVG(fromNode.getX(), fromNode.getY(), bounds);
                auto [x2, y2] = worldToSVG(toNode.getX(), toNode.getY(), bounds);
                
                bool isPathEdge = isEdgeInPath(nodeId, edge.getToNode(), highlightPath);
                
                std::string color = isPathEdge ? style.pathEdgeColor : style.edgeColor;
                double width = isPathEdge ? style.pathEdgeStrokeWidth : style.edgeStrokeWidth;
                double opacity = (!highlightPath.empty() && !isPathEdge) ? 0.3 : 1.0;
                
                svg << "  <line x1=\"" << x1 << "\" y1=\"" << y1 << "\" "
                    << "x2=\"" << x2 << "\" y2=\"" << y2 << "\" "
                    << "stroke=\"" << color << "\" stroke-width=\"" << width << "\" "
                    << "opacity=\"" << opacity << "\"/>\n";
            }
        }
    }
    
    void drawNodes(std::ostringstream& svg, const BoundingBox& bounds, const std::vector<int>& highlightPath) {
        std::vector<int> nodeIds = graph->getAllNodeIds();
        
        for (int nodeId : nodeIds) {
            const Node& node = graph->getNode(nodeId);
            auto [x, y] = worldToSVG(node.getX(), node.getY(), bounds);
            
            std::string fillColor = style.nodeColor;
            double opacity = 1.0;
            
            if (!highlightPath.empty()) {
                auto it = std::find(highlightPath.begin(), highlightPath.end(), nodeId);
                if (it != highlightPath.end()) {
                    if (it == highlightPath.begin()) {
                        fillColor = style.startNodeColor;
                    } else if (it == highlightPath.end() - 1) {
                        fillColor = style.goalNodeColor;
                    } else {
                        fillColor = style.pathNodeColor;
                    }
                } else {
                    opacity = 0.4;
                }
            }
            
            svg << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" "
                << "r=\"" << style.nodeRadius << "\" "
                << "fill=\"" << fillColor << "\" "
                << "stroke=\"" << style.nodeStroke << "\" "
                << "stroke-width=\"" << style.nodeStrokeWidth << "\" "
                << "opacity=\"" << opacity << "\"/>\n";
            
            // Node labels
            svg << "  <text x=\"" << x << "\" y=\"" << (y + style.fontSize/3) << "\" "
                << "text-anchor=\"middle\" font-family=\"" << style.fontFamily << "\" "
                << "font-size=\"" << (style.fontSize * 0.8) << "\" "
                << "fill=\"white\" font-weight=\"bold\">"
                << nodeId << "</text>\n";
        }
    }
    
    void drawPathEdges(std::ostringstream& svg, const BoundingBox& bounds, const std::vector<int>& path) {
        for (size_t i = 1; i < path.size(); ++i) {
            if (!graph->hasNode(path[i-1]) || !graph->hasNode(path[i])) continue;
            
            const Node& fromNode = graph->getNode(path[i-1]);
            const Node& toNode = graph->getNode(path[i]);
            
            auto [x1, y1] = worldToSVG(fromNode.getX(), fromNode.getY(), bounds);
            auto [x2, y2] = worldToSVG(toNode.getX(), toNode.getY(), bounds);
            
            svg << "  <line x1=\"" << x1 << "\" y1=\"" << y1 << "\" "
                << "x2=\"" << x2 << "\" y2=\"" << y2 << "\" "
                << "stroke=\"" << style.pathEdgeColor << "\" "
                << "stroke-width=\"" << style.pathEdgeStrokeWidth << "\" "
                << "stroke-linecap=\"round\"/>\n";
            
            // Add arrow marker for direction
            if (i < path.size() - 1) {
                double angle = std::atan2(y2 - y1, x2 - x1);
                double arrowLength = 10.0;
                double arrowAngle = 0.5;
                
                double midX = (x1 + x2) / 2;
                double midY = (y1 + y2) / 2;
                
                double arrowX1 = midX - arrowLength * std::cos(angle - arrowAngle);
                double arrowY1 = midY - arrowLength * std::sin(angle - arrowAngle);
                double arrowX2 = midX - arrowLength * std::cos(angle + arrowAngle);
                double arrowY2 = midY - arrowLength * std::sin(angle + arrowAngle);
                
                svg << "  <polygon points=\"" << midX << "," << midY << " "
                    << arrowX1 << "," << arrowY1 << " "
                    << arrowX2 << "," << arrowY2 << "\" "
                    << "fill=\"" << style.pathEdgeColor << "\"/>\n";
            }
        }
    }
    
    void drawPathNodes(std::ostringstream& svg, const BoundingBox& bounds, const std::vector<int>& path) {
        for (size_t i = 0; i < path.size(); ++i) {
            if (!graph->hasNode(path[i])) continue;
            
            const Node& node = graph->getNode(path[i]);
            auto [x, y] = worldToSVG(node.getX(), node.getY(), bounds);
            
            std::string fillColor;
            if (i == 0) {
                fillColor = style.startNodeColor;
            } else if (i == path.size() - 1) {
                fillColor = style.goalNodeColor;
            } else {
                fillColor = style.pathNodeColor;
            }
            
            svg << "  <circle cx=\"" << x << "\" cy=\"" << y << "\" "
                << "r=\"" << style.nodeRadius << "\" "
                << "fill=\"" << fillColor << "\" "
                << "stroke=\"" << style.nodeStroke << "\" "
                << "stroke-width=\"" << style.nodeStrokeWidth << "\"/>\n";
            
            // Node labels
            svg << "  <text x=\"" << x << "\" y=\"" << (y + style.fontSize/3) << "\" "
                << "text-anchor=\"middle\" font-family=\"" << style.fontFamily << "\" "
                << "font-size=\"" << (style.fontSize * 0.8) << "\" "
                << "fill=\"white\" font-weight=\"bold\">"
                << path[i] << "</text>\n";
        }
    }
    
    void drawLegend(std::ostringstream& svg, bool hasPath) {
        double legendX = canvasWidth - 150;
        double legendY = 60;
        double itemHeight = 25;
        
        svg << "  <rect x=\"" << (legendX - 10) << "\" y=\"" << (legendY - 10) << "\" "
            << "width=\"140\" height=\"" << (hasPath ? 120 : 80) << "\" "
            << "fill=\"white\" stroke=\"#cccccc\" stroke-width=\"1\" rx=\"5\"/>\n";
        
        svg << "  <text x=\"" << legendX << "\" y=\"" << legendY << "\" "
            << "font-family=\"" << style.fontFamily << "\" font-size=\"14\" "
            << "font-weight=\"bold\" fill=\"" << style.textColor << "\">Legend</text>\n";
        
        // Regular node
        svg << "  <circle cx=\"" << (legendX + 10) << "\" cy=\"" << (legendY + itemHeight) << "\" "
            << "r=\"6\" fill=\"" << style.nodeColor << "\" stroke=\"" << style.nodeStroke << "\"/>\n";
        svg << "  <text x=\"" << (legendX + 25) << "\" y=\"" << (legendY + itemHeight + 4) << "\" "
            << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
            << "fill=\"" << style.textColor << "\">Node</text>\n";
        
        if (hasPath) {
            // Start node
            svg << "  <circle cx=\"" << (legendX + 10) << "\" cy=\"" << (legendY + 2*itemHeight) << "\" "
                << "r=\"6\" fill=\"" << style.startNodeColor << "\" stroke=\"" << style.nodeStroke << "\"/>\n";
            svg << "  <text x=\"" << (legendX + 25) << "\" y=\"" << (legendY + 2*itemHeight + 4) << "\" "
                << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
                << "fill=\"" << style.textColor << "\">Start</text>\n";
            
            // Goal node
            svg << "  <circle cx=\"" << (legendX + 10) << "\" cy=\"" << (legendY + 3*itemHeight) << "\" "
                << "r=\"6\" fill=\"" << style.goalNodeColor << "\" stroke=\"" << style.nodeStroke << "\"/>\n";
            svg << "  <text x=\"" << (legendX + 25) << "\" y=\"" << (legendY + 3*itemHeight + 4) << "\" "
                << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
                << "fill=\"" << style.textColor << "\">Goal</text>\n";
            
            // Path edge
            svg << "  <line x1=\"" << (legendX + 5) << "\" y1=\"" << (legendY + 4*itemHeight) << "\" "
                << "x2=\"" << (legendX + 20) << "\" y2=\"" << (legendY + 4*itemHeight) << "\" "
                << "stroke=\"" << style.pathEdgeColor << "\" stroke-width=\"3\"/>\n";
            svg << "  <text x=\"" << (legendX + 25) << "\" y=\"" << (legendY + 4*itemHeight + 4) << "\" "
                << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
                << "fill=\"" << style.textColor << "\">Path</text>\n";
        }
    }
    
    void drawPathInfo(std::ostringstream& svg, const std::vector<int>& path) {
        double infoX = 20;
        double infoY = canvasHeight - 80;
        
        svg << "  <rect x=\"" << (infoX - 10) << "\" y=\"" << (infoY - 10) << "\" "
            << "width=\"200\" height=\"60\" "
            << "fill=\"white\" stroke=\"#cccccc\" stroke-width=\"1\" rx=\"5\"/>\n";
        
        svg << "  <text x=\"" << infoX << "\" y=\"" << infoY << "\" "
            << "font-family=\"" << style.fontFamily << "\" font-size=\"14\" "
            << "font-weight=\"bold\" fill=\"" << style.textColor << "\">Path Info</text>\n";
        
        svg << "  <text x=\"" << infoX << "\" y=\"" << (infoY + 20) << "\" "
            << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
            << "fill=\"" << style.textColor << "\">Nodes: " << path.size() << "</text>\n";
        
        double pathLength = calculatePathLength(path);
        svg << "  <text x=\"" << infoX << "\" y=\"" << (infoY + 35) << "\" "
            << "font-family=\"" << style.fontFamily << "\" font-size=\"12\" "
            << "fill=\"" << style.textColor << "\">Length: " 
            << std::fixed << std::setprecision(1) << pathLength << "</text>\n";
    }
    
    bool isEdgeInPath(int fromNode, int toNode, const std::vector<int>& path) {
        for (size_t i = 1; i < path.size(); ++i) {
            if ((path[i-1] == fromNode && path[i] == toNode) ||
                (path[i-1] == toNode && path[i] == fromNode)) {
                return true;
            }
        }
        return false;
    }
    
    double calculatePathLength(const std::vector<int>& path) {
        if (path.size() < 2) return 0.0;
        
        double totalLength = 0.0;
        for (size_t i = 1; i < path.size(); ++i) {
            if (graph->hasNode(path[i-1]) && graph->hasNode(path[i])) {
                const Node& from = graph->getNode(path[i-1]);
                const Node& to = graph->getNode(path[i]);
                totalLength += from.euclideanDistance(to);
            }
        }
        return totalLength;
    }
};

// Global SVG exporter
static std::unique_ptr<SVGExporter> g_svgExporter;

void initializeSVGExporter(const Graph* graph, double width, double height) {
    g_svgExporter = std::make_unique<SVGExporter>(graph, width, height);
}

bool exportGraphToSVG(const Graph* graph, const std::string& filename, 
                     const std::vector<int>& highlightPath, double width, double height) {
    if (!g_svgExporter) {
        g_svgExporter = std::make_unique<SVGExporter>(graph, width, height);
    }
    return g_svgExporter->exportGraph(filename, highlightPath);
}

bool exportPathToSVG(const Graph* graph, const std::string& filename, 
                    const std::vector<int>& path, double width, double height) {
    if (!g_svgExporter) {
        g_svgExporter = std::make_unique<SVGExporter>(graph, width, height);
    }
    return g_svgExporter->exportPath(filename, path);
}

std::string generateSVGString(const Graph* graph, const std::vector<int>& highlightPath, 
                             double width, double height) {
    if (!g_svgExporter) {
        g_svgExporter = std::make_unique<SVGExporter>(graph, width, height);
    }
    return g_svgExporter->generateSVGString(highlightPath);
}