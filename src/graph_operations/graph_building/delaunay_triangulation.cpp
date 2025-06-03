#include "graph_operations/GraphBuilder.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <vector>
#include <set>

struct Triangle {
    int a, b, c;
    
    Triangle(int a, int b, int c) : a(a), b(b), c(c) {}
    
    bool contains(int vertex) const {
        return a == vertex || b == vertex || c == vertex;
    }
    
    bool hasEdge(int v1, int v2) const {
        return (a == v1 && (b == v2 || c == v2)) ||
               (b == v1 && (a == v2 || c == v2)) ||
               (c == v1 && (a == v2 || b == v2));
    }
};

struct Circle {
    double x, y, radius;
    
    Circle(double x, double y, double radius) : x(x), y(y), radius(radius) {}
    
    bool contains(double px, double py) const {
        double dx = px - x;
        double dy = py - y;
        return (dx * dx + dy * dy) < (radius * radius);
    }
};

class DelaunayTriangulator {
private:
    std::vector<std::pair<double, double>> points;
    std::vector<Triangle> triangles;
    
public:
    DelaunayTriangulator(const std::vector<std::pair<double, double>>& inputPoints) 
        : points(inputPoints) {
        std::cout << "[DELAUNAY] Initializing triangulation with " << points.size() << " points" << std::endl;
    }
    
    void triangulate() {
        if (points.size() < 3) {
            std::cout << "[DELAUNAY] Not enough points for triangulation" << std::endl;
            return;
        }
        
        std::cout << "[DELAUNAY] Starting Bowyer-Watson triangulation algorithm" << std::endl;
        
        // Create super triangle that encompasses all points
        createSuperTriangle();
        
        // Add points one by one
        for (size_t i = 0; i < points.size(); ++i) {
            addPoint(static_cast<int>(i));
        }
        
        // Remove triangles that share vertices with super triangle
        removeSuperTriangle();
        
        std::cout << "[DELAUNAY] Triangulation completed with " << triangles.size() << " triangles" << std::endl;
    }
    
    void buildGraph(Graph* graph) {
        if (!graph) {
            std::cout << "[DELAUNAY] Cannot build graph - null pointer" << std::endl;
            return;
        }
        
        std::cout << "[DELAUNAY] Building graph from triangulation" << std::endl;
        
        // Add nodes for each point
        for (size_t i = 0; i < points.size(); ++i) {
            graph->addNode(static_cast<int>(i), "delaunay_" + std::to_string(i), 
                          points[i].first, points[i].second);
        }
        
        // Add edges from triangulation
        std::set<std::pair<int, int>> addedEdges;
        
        for (const Triangle& tri : triangles) {
            addEdgeIfNew(graph, addedEdges, tri.a, tri.b);
            addEdgeIfNew(graph, addedEdges, tri.b, tri.c);
            addEdgeIfNew(graph, addedEdges, tri.c, tri.a);
        }
        
        std::cout << "[DELAUNAY] Graph built with " << addedEdges.size() << " edges" << std::endl;
    }
    
private:
    void createSuperTriangle() {
        // Find bounding box
        double minX = points[0].first, maxX = points[0].first;
        double minY = points[0].second, maxY = points[0].second;
        
        for (const auto& point : points) {
            minX = std::min(minX, point.first);
            maxX = std::max(maxX, point.first);
            minY = std::min(minY, point.second);
            maxY = std::max(maxY, point.second);
        }
        
        // Create super triangle vertices (way outside bounding box)
        double dx = maxX - minX;
        double dy = maxY - minY;
        double deltaMax = std::max(dx, dy) * 2;
        
        int superA = static_cast<int>(points.size());
        int superB = superA + 1;
        int superC = superA + 2;
        
        points.push_back({minX - deltaMax, minY - deltaMax});
        points.push_back({maxX + deltaMax, minY - deltaMax});
        points.push_back({(minX + maxX) / 2, maxY + deltaMax});
        
        triangles.push_back(Triangle(superA, superB, superC));
        
        std::cout << "[DELAUNAY] Created super triangle with vertices " 
                  << superA << ", " << superB << ", " << superC << std::endl;
    }
    
    void addPoint(int pointIndex) {
        std::vector<Triangle> badTriangles;
        
        // Find triangles whose circumcircle contains the point
        for (const Triangle& tri : triangles) {
            Circle circumcircle = getCircumcircle(tri);
            if (circumcircle.contains(points[pointIndex].first, points[pointIndex].second)) {
                badTriangles.push_back(tri);
            }
        }
        
        // Find the boundary of the polygonal hole
        std::vector<std::pair<int, int>> polygon;
        for (const Triangle& badTri : badTriangles) {
            addBoundaryEdges(badTri, badTriangles, polygon);
        }
        
        // Remove bad triangles
        triangles.erase(
            std::remove_if(triangles.begin(), triangles.end(),
                [&badTriangles](const Triangle& tri) {
                    return std::find_if(badTriangles.begin(), badTriangles.end(),
                        [&tri](const Triangle& bad) {
                            return tri.a == bad.a && tri.b == bad.b && tri.c == bad.c;
                        }) != badTriangles.end();
                }),
            triangles.end()
        );
        
        // Create new triangles from the point to each boundary edge
        for (const auto& edge : polygon) {
            triangles.push_back(Triangle(pointIndex, edge.first, edge.second));
        }
    }
    
    void addBoundaryEdges(const Triangle& triangle, 
                         const std::vector<Triangle>& badTriangles,
                         std::vector<std::pair<int, int>>& polygon) {
        
        std::vector<std::pair<int, int>> edges = {
            {triangle.a, triangle.b},
            {triangle.b, triangle.c},
            {triangle.c, triangle.a}
        };
        
        for (const auto& edge : edges) {
            bool isShared = false;
            
            // Check if this edge is shared with another bad triangle
            for (const Triangle& other : badTriangles) {
                if (&triangle != &other && other.hasEdge(edge.first, edge.second)) {
                    isShared = true;
                    break;
                }
            }
            
            if (!isShared) {
                polygon.push_back(edge);
            }
        }
    }
    
    Circle getCircumcircle(const Triangle& triangle) {
        double ax = points[triangle.a].first, ay = points[triangle.a].second;
        double bx = points[triangle.b].first, by = points[triangle.b].second;
        double cx = points[triangle.c].first, cy = points[triangle.c].second;
        
        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        
        if (std::abs(d) < 1e-10) {
            // Degenerate triangle, return large circle
            return Circle(0, 0, 1e10);
        }
        
        double ux = ((ax * ax + ay * ay) * (by - cy) + 
                     (bx * bx + by * by) * (cy - ay) + 
                     (cx * cx + cy * cy) * (ay - by)) / d;
        
        double uy = ((ax * ax + ay * ay) * (cx - bx) + 
                     (bx * bx + by * by) * (ax - cx) + 
                     (cx * cx + cy * cy) * (bx - ax)) / d;
        
        double radius = std::sqrt((ux - ax) * (ux - ax) + (uy - ay) * (uy - ay));
        
        return Circle(ux, uy, radius);
    }
    
    void removeSuperTriangle() {
        int superVertexStart = static_cast<int>(points.size()) - 3;
        
        triangles.erase(
            std::remove_if(triangles.begin(), triangles.end(),
                [superVertexStart](const Triangle& tri) {
                    return tri.a >= superVertexStart || 
                           tri.b >= superVertexStart || 
                           tri.c >= superVertexStart;
                }),
            triangles.end()
        );
        
        // Remove super triangle vertices
        points.erase(points.end() - 3, points.end());
        
        std::cout << "[DELAUNAY] Removed super triangle, " << triangles.size() 
                  << " triangles remain" << std::endl;
    }
    
    void addEdgeIfNew(Graph* graph, std::set<std::pair<int, int>>& addedEdges, 
                     int a, int b) {
        if (a >= static_cast<int>(points.size()) - 3 && 
            b >= static_cast<int>(points.size()) - 3) {
            return; // Skip super triangle edges
        }
        
        std::pair<int, int> edge = {std::min(a, b), std::max(a, b)};
        
        if (addedEdges.find(edge) == addedEdges.end()) {
            addedEdges.insert(edge);
            
            double distance = std::sqrt(
                std::pow(points[a].first - points[b].first, 2) +
                std::pow(points[a].second - points[b].second, 2)
            );
            
            graph->addEdge(a, b, distance, true);
        }
    }
};

void GraphBuilder::buildDelaunayGraph(const std::vector<std::pair<double, double>>& points) {
    std::cout << "[GRAPH_BUILDER] Building Delaunay triangulation graph" << std::endl;
    
    if (!targetGraph) {
        std::cout << "[GRAPH_BUILDER] Cannot build graph - null target graph" << std::endl;
        return;
    }
    
    if (points.size() < 3) {
        std::cout << "[GRAPH_BUILDER] Need at least 3 points for Delaunay triangulation" << std::endl;
        return;
    }
    
    DelaunayTriangulator triangulator(points);
    triangulator.triangulate();
    triangulator.buildGraph(targetGraph);
    
    std::cout << "[GRAPH_BUILDER] Delaunay graph construction completed" << std::endl;
}

// Helper function for random point generation
std::vector<std::pair<double, double>> generateRandomPoints(int count, double width, double height) {
    std::vector<std::pair<double, double>> points;
    points.reserve(count);
    
    std::cout << "[DELAUNAY] Generating " << count << " random points in " 
              << width << "x" << height << " area" << std::endl;
    
    for (int i = 0; i < count; ++i) {
        double x = (static_cast<double>(rand()) / RAND_MAX) * width;
        double y = (static_cast<double>(rand()) / RAND_MAX) * height;
        points.emplace_back(x, y);
    }
    
    return points;
}