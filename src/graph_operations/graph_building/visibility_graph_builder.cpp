#include "graph_operations/GraphBuilder.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    
    bool operator==(const Point& other) const {
        return std::abs(x - other.x) < 1e-9 && std::abs(y - other.y) < 1e-9;
    }
};

struct Polygon {
    std::vector<Point> vertices;
    
    Polygon(const std::vector<Point>& verts) : vertices(verts) {}
    
    bool contains(const Point& p) const {
        bool inside = false;
        size_t n = vertices.size();
        
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            if (((vertices[i].y > p.y) != (vertices[j].y > p.y)) &&
                (p.x < (vertices[j].x - vertices[i].x) * (p.y - vertices[i].y) / 
                (vertices[j].y - vertices[i].y) + vertices[i].x)) {
                inside = !inside;
            }
        }
        return inside;
    }
};

class VisibilityGraphBuilder {
private:
    Graph* graph;
    std::vector<Point> points;
    std::vector<Polygon> obstacles;
    double epsilon;
    
public:
    VisibilityGraphBuilder(Graph* targetGraph) : graph(targetGraph), epsilon(1e-9) {}
    
    void addPoint(double x, double y) {
        points.emplace_back(x, y);
    }
    
    void addObstacle(const std::vector<std::pair<double, double>>& vertices) {
        std::vector<Point> obstaclePoints;
        for (const auto& [x, y] : vertices) {
            obstaclePoints.emplace_back(x, y);
        }
        obstacles.emplace_back(obstaclePoints);
        
        // Add obstacle vertices as potential visibility points
        for (const Point& vertex : obstaclePoints) {
            points.push_back(vertex);
        }
        
        std::cout << "[VISIBILITY] Added obstacle with " << vertices.size() << " vertices" << std::endl;
    }
    
    void buildVisibilityGraph() {
        std::cout << "[VISIBILITY] Building visibility graph with " << points.size() 
                  << " points and " << obstacles.size() << " obstacles" << std::endl;
        
        // Remove duplicate points
        removeDuplicatePoints();
        
        // Add nodes to graph
        for (size_t i = 0; i < points.size(); ++i) {
            graph->addNode(static_cast<int>(i), "vis_" + std::to_string(i), 
                          points[i].x, points[i].y);
        }
        
        // Check visibility between all pairs of points
        int edgesAdded = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                if (isVisible(points[i], points[j])) {
                    double distance = calculateDistance(points[i], points[j]);
                    graph->addEdge(static_cast<int>(i), static_cast<int>(j), distance, true);
                    edgesAdded++;
                }
            }
        }
        
        std::cout << "[VISIBILITY] Added " << edgesAdded << " visibility edges" << std::endl;
    }
    
    void buildReducedVisibilityGraph() {
        std::cout << "[VISIBILITY] Building reduced visibility graph (only obstacle vertices)" << std::endl;
        
        std::vector<Point> obstacleVertices;
        
        // Collect all obstacle vertices
        for (const Polygon& obstacle : obstacles) {
            for (const Point& vertex : obstacle.vertices) {
                obstacleVertices.push_back(vertex);
            }
        }
        
        removeDuplicates(obstacleVertices);
        
        // Add nodes
        for (size_t i = 0; i < obstacleVertices.size(); ++i) {
            graph->addNode(static_cast<int>(i), "rvis_" + std::to_string(i),
                          obstacleVertices[i].x, obstacleVertices[i].y);
        }
        
        // Check visibility between obstacle vertices
        int edgesAdded = 0;
        for (size_t i = 0; i < obstacleVertices.size(); ++i) {
            for (size_t j = i + 1; j < obstacleVertices.size(); ++j) {
                if (isVisible(obstacleVertices[i], obstacleVertices[j])) {
                    double distance = calculateDistance(obstacleVertices[i], obstacleVertices[j]);
                    graph->addEdge(static_cast<int>(i), static_cast<int>(j), distance, true);
                    edgesAdded++;
                }
            }
        }
        
        std::cout << "[VISIBILITY] Reduced graph: " << obstacleVertices.size() 
                  << " nodes, " << edgesAdded << " edges" << std::endl;
    }
    
    void addStartGoalPoints(double startX, double startY, double goalX, double goalY) {
        std::cout << "[VISIBILITY] Adding start and goal points to visibility graph" << std::endl;
        
        Point start(startX, startY);
        Point goal(goalX, goalY);
        
        int startId = static_cast<int>(graph->getNodeCount());
        int goalId = startId + 1;
        
        graph->addNode(startId, "start", startX, startY);
        graph->addNode(goalId, "goal", goalX, goalY);
        
        // Connect start and goal to visible existing nodes
        std::vector<int> existingNodes = graph->getAllNodeIds();
        
        for (int nodeId : existingNodes) {
            if (nodeId == startId || nodeId == goalId) continue;
            
            const Node& node = graph->getNode(nodeId);
            Point nodePoint(node.getX(), node.getY());
            
            // Check start visibility
            if (isVisible(start, nodePoint)) {
                double distance = calculateDistance(start, nodePoint);
                graph->addEdge(startId, nodeId, distance, true);
            }
            
            // Check goal visibility
            if (isVisible(goal, nodePoint)) {
                double distance = calculateDistance(goal, nodePoint);
                graph->addEdge(goalId, nodeId, distance, true);
            }
        }
        
        // Check direct start-goal visibility
        if (isVisible(start, goal)) {
            double distance = calculateDistance(start, goal);
            graph->addEdge(startId, goalId, distance, true);
        }
        
        std::cout << "[VISIBILITY] Connected start (node " << startId 
                  << ") and goal (node " << goalId << ")" << std::endl;
    }
    
private:
    bool isVisible(const Point& p1, const Point& p2) {
        // Check if line segment from p1 to p2 intersects any obstacle
        for (const Polygon& obstacle : obstacles) {
            if (lineIntersectsPolygon(p1, p2, obstacle)) {
                return false;
            }
        }
        return true;
    }
    
    bool lineIntersectsPolygon(const Point& p1, const Point& p2, const Polygon& polygon) {
        // Check if endpoints are inside polygon (except for polygon vertices)
        if (!isPolygonVertex(p1, polygon) && polygon.contains(p1)) return true;
        if (!isPolygonVertex(p2, polygon) && polygon.contains(p2)) return true;
        
        // Check intersection with polygon edges
        size_t n = polygon.vertices.size();
        for (size_t i = 0; i < n; ++i) {
            Point edgeStart = polygon.vertices[i];
            Point edgeEnd = polygon.vertices[(i + 1) % n];
            
            if (lineSegmentsIntersect(p1, p2, edgeStart, edgeEnd)) {
                // Allow intersection at endpoints
                if (!(p1 == edgeStart || p1 == edgeEnd || p2 == edgeStart || p2 == edgeEnd)) {
                    return true;
                }
            }
        }
        
        return false;
    }
    
    bool lineSegmentsIntersect(const Point& p1, const Point& q1, 
                              const Point& p2, const Point& q2) {
        auto orientation = [](const Point& p, const Point& q, const Point& r) {
            double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
            if (std::abs(val) < epsilon) return 0;  // collinear
            return (val > 0) ? 1 : 2;  // clockwise or counterclockwise
        };
        
        auto onSegment = [](const Point& p, const Point& q, const Point& r) {
            return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
                   q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
        };
        
        int o1 = orientation(p1, q1, p2);
        int o2 = orientation(p1, q1, q2);
        int o3 = orientation(p2, q2, p1);
        int o4 = orientation(p2, q2, q1);
        
        // General case
        if (o1 != o2 && o3 != o4) return true;
        
        // Special cases for collinear points
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        
        return false;
    }
    
    bool isPolygonVertex(const Point& p, const Polygon& polygon) {
        for (const Point& vertex : polygon.vertices) {
            if (p == vertex) return true;
        }
        return false;
    }
    
    double calculateDistance(const Point& p1, const Point& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }
    
    void removeDuplicatePoints() {
        removeDuplicates(points);
    }
    
    void removeDuplicates(std::vector<Point>& pointList) {
        std::sort(pointList.begin(), pointList.end(), 
                 [](const Point& a, const Point& b) {
                     return a.x < b.x || (std::abs(a.x - b.x) < 1e-9 && a.y < b.y);
                 });
        
        pointList.erase(
            std::unique(pointList.begin(), pointList.end(),
                       [](const Point& a, const Point& b) { return a == b; }),
            pointList.end()
        );
    }
};

void GraphBuilder::buildVisibilityGraph(const std::vector<std::pair<double, double>>& points,
                                       const std::vector<std::vector<std::pair<double, double>>>& obstacles) {
    std::cout << "[GRAPH_BUILDER] Building visibility graph" << std::endl;
    
    if (!targetGraph) {
        std::cout << "[GRAPH_BUILDER] Cannot build graph - null target graph" << std::endl;
        return;
    }
    
    VisibilityGraphBuilder builder(targetGraph);
    
    // Add all points
    for (const auto& [x, y] : points) {
        builder.addPoint(x, y);
    }
    
    // Add obstacles
    for (const auto& obstacle : obstacles) {
        builder.addObstacle(obstacle);
    }
    
    builder.buildVisibilityGraph();
    
    std::cout << "[GRAPH_BUILDER] Visibility graph construction completed" << std::endl;
}

// Extended visibility graph functions
void buildReducedVisibilityGraph(Graph* graph, 
                               const std::vector<std::vector<std::pair<double, double>>>& obstacles) {
    VisibilityGraphBuilder builder(graph);
    
    for (const auto& obstacle : obstacles) {
        builder.addObstacle(obstacle);
    }
    
    builder.buildReducedVisibilityGraph();
}

void buildVisibilityGraphWithStartGoal(Graph* graph,
                                     const std::vector<std::vector<std::pair<double, double>>>& obstacles,
                                     double startX, double startY, double goalX, double goalY) {
    VisibilityGraphBuilder builder(graph);
    
    for (const auto& obstacle : obstacles) {
        builder.addObstacle(obstacle);
    }
    
    builder.buildReducedVisibilityGraph();
    builder.addStartGoalPoints(startX, startY, goalX, goalY);
}

// Helper function to create rectangular obstacles
std::vector<std::pair<double, double>> createRectangleObstacle(double x, double y, double width, double height) {
    return {
        {x, y},
        {x + width, y},
        {x + width, y + height},
        {x, y + height}
    };
}

// Helper function to create circular obstacles (approximated as polygons)
std::vector<std::pair<double, double>> createCircleObstacle(double centerX, double centerY, double radius, int sides = 16) {
    std::vector<std::pair<double, double>> vertices;
    
    for (int i = 0; i < sides; ++i) {
        double angle = 2 * M_PI * i / sides;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);
        vertices.emplace_back(x, y);
    }
    
    return vertices;
}