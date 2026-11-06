#include "reporting/PathVisualiser.hpp"
#include "renderers.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

// PathVisualizer is the public front end for the three renderers in this folder.

namespace {

bool isValidGraphPath(const Graph& graph, const std::vector<int>& path) {
    if (path.empty()) {
        return false;
    }
    for (size_t i = 0; i < path.size(); ++i) {
        if (!graph.hasNode(path[i])) {
            return false;
        }
        if (i > 0 && !graph.hasEdge(path[i - 1], path[i])) {
            return false;
        }
    }
    return true;
}

double coordinateDistance(const Node& a, const Node& b) {
    return std::hypot(a.getX() - b.getX(), a.getY() - b.getY());
}

}  // namespace

PathVisualizer::PathVisualizer(const Graph* graph)
    : environment(graph), scaleFactor(1.0), canvasWidth(60), canvasHeight(20) {
    if (!environment) {
        throw std::invalid_argument("Graph pointer cannot be null");
    }
}

void PathVisualizer::visualizePath(const std::vector<int>& path, VisualizationFormat format) const {
    switch (format) {
        case VisualizationFormat::CONSOLE_TEXT:
            initializeConsolePrinter(environment);
            printPathToConsole(environment, path);
            break;
        case VisualizationFormat::ASCII_ART:
            std::cout << generateASCIIVisualization(path);
            break;
        case VisualizationFormat::SVG:
            std::cout << generateSVGVisualization(path) << std::endl;
            break;
        case VisualizationFormat::DOT_GRAPH:
            std::cout << generateDotVisualization(path);
            break;
        case VisualizationFormat::JSON: {
            std::ostringstream json;
            json << "{\"path\": [";
            for (size_t i = 0; i < path.size(); ++i) {
                json << (i ? ", " : "") << path[i];
            }
            json << "]}";
            std::cout << json.str() << std::endl;
            break;
        }
    }
}

void PathVisualizer::visualizeMultiplePaths(const std::vector<std::vector<int>>& paths) const {
    for (size_t i = 0; i < paths.size(); ++i) {
        std::cout << "--- Path " << (i + 1) << " of " << paths.size() << " ---" << std::endl;
        std::cout << generateASCIIVisualization(paths[i]);
    }
}

void PathVisualizer::visualizeGraphStructure() const {
    std::cout << generateASCIIVisualization({});
}

void PathVisualizer::exportVisualization(const std::vector<int>& path, const std::string& filename,
                                         VisualizationFormat format) const {
    if (format == VisualizationFormat::SVG) {
        const double width = 800.0 * scaleFactor;
        const double height = 600.0 * scaleFactor;
        initializeSVGExporter(environment, width, height);
        if (!exportGraphToSVG(environment, filename, path, width, height)) {
            throw std::runtime_error("Could not write SVG file: " + filename);
        }
        return;
    }

    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Could not open file for writing: " + filename);
    }
    switch (format) {
        case VisualizationFormat::DOT_GRAPH:
            out << generateDotVisualization(path);
            break;
        case VisualizationFormat::CONSOLE_TEXT:
            out << generatePathStatistics(path);
            break;
        default:
            out << generateASCIIVisualization(path);
            break;
    }
}

void PathVisualizer::setCanvasDimensions(int width, int height) {
    if (width < 10 || height < 5) {
        throw std::invalid_argument("Canvas must be at least 10x5 characters");
    }
    canvasWidth = width;
    canvasHeight = height;
}

void PathVisualizer::setScaleFactor(double scale) {
    if (scale <= 0.0) {
        throw std::invalid_argument("Scale factor must be positive");
    }
    scaleFactor = scale;
}

std::string PathVisualizer::generatePathStatistics(const std::vector<int>& path) const {
    std::ostringstream stats;
    stats << std::fixed << std::setprecision(2);
    stats << "Nodes: " << path.size() << "\n";
    if (path.empty()) {
        stats << "Path is empty\n";
        return stats.str();
    }

    const bool valid = isValidGraphPath(*environment, path);
    double edgeCost = 0.0;
    double geometricLength = 0.0;
    int turns = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (environment->hasEdge(path[i - 1], path[i])) {
            edgeCost += environment->getEdgeWeight(path[i - 1], path[i]);
        }
        if (environment->hasNode(path[i - 1]) && environment->hasNode(path[i])) {
            geometricLength += coordinateDistance(environment->getNode(path[i - 1]), environment->getNode(path[i]));
        }
        if (i >= 2 && environment->hasNode(path[i - 2])) {
            const Node& a = environment->getNode(path[i - 2]);
            const Node& b = environment->getNode(path[i - 1]);
            const Node& c = environment->getNode(path[i]);
            const double cross = (b.getX() - a.getX()) * (c.getY() - b.getY()) -
                                 (b.getY() - a.getY()) * (c.getX() - b.getX());
            if (std::abs(cross) > 1e-9) {
                ++turns;
            }
        }
    }

    stats << "Segments: " << (path.size() - 1) << "\n";
    stats << "Edge-weight cost: " << edgeCost << "\n";
    stats << "Geometric length (coordinate units): " << geometricLength << "\n";
    if (environment->hasNode(path.front()) && environment->hasNode(path.back())) {
        const double direct = coordinateDistance(environment->getNode(path.front()), environment->getNode(path.back()));
        stats << "Straight-line distance (coordinate units): " << direct << "\n";
        if (geometricLength > 0.0) {
            stats << "Efficiency: " << (direct / geometricLength * 100.0) << "%\n";
        }
    }
    stats << "Turns: " << turns << "\n";
    stats << "Valid (every step follows an edge): " << (valid ? "yes" : "no") << "\n";
    return stats.str();
}

void PathVisualizer::comparePathVisualizations(const std::vector<int>& path1, const std::vector<int>& path2) const {
    initializeConsolePrinter(environment);
    printPathComparison(environment, path1, path2, "Path 1", "Path 2");
}

void PathVisualizer::createInteractiveVisualization(const std::vector<int>& path, const std::string& outputFile) const {
    std::ofstream out(outputFile);
    if (!out) {
        throw std::runtime_error("Could not open file for writing: " + outputFile);
    }
    out << "<!DOCTYPE html>\n<html><head><meta charset=\"utf-8\"><title>Path Visualization</title></head><body>\n"
        << generateSVGVisualization(path) << "\n<pre>" << generatePathStatistics(path) << "</pre>\n</body></html>\n";
}

std::string PathVisualizer::generateASCIIVisualization(const std::vector<int>& path) const {
    initializeASCIIRenderer(environment, canvasWidth, canvasHeight, scaleFactor);
    return renderGraphASCII(environment, path, canvasWidth, canvasHeight);
}

std::string PathVisualizer::generateSVGVisualization(const std::vector<int>& path) const {
    const double width = 800.0 * scaleFactor;
    const double height = 600.0 * scaleFactor;
    initializeSVGExporter(environment, width, height);
    return generateSVGString(environment, path, width, height);
}

std::string PathVisualizer::generateDotVisualization(const std::vector<int>& path) const {
    auto onPath = [&path](int from, int to) {
        for (size_t i = 1; i < path.size(); ++i) {
            if ((path[i - 1] == from && path[i] == to) || (path[i - 1] == to && path[i] == from)) {
                return true;
            }
        }
        return false;
    };

    std::ostringstream dot;
    dot << "graph Navigation {\n";
    for (int id : environment->getAllNodeIds()) {
        const Node& node = environment->getNode(id);
        dot << "  n" << id << " [label=\"" << id << "\", pos=\"" << node.getX() << "," << node.getY() << "!\"";
        if (!path.empty() && (id == path.front() || id == path.back())) {
            dot << ", style=filled, fillcolor=\"" << (id == path.front() ? "palegreen" : "tomato") << "\"";
        }
        dot << "];\n";
    }
    for (int id : environment->getAllNodeIds()) {
        for (const Edge& edge : environment->getEdgesFrom(id)) {
            if (edge.getToNode() < id && edge.isBidirectional()) {
                continue;  // draw each two-way edge once
            }
            dot << "  n" << id << " -- n" << edge.getToNode() << " [label=\"" << edge.getWeight() << "\"";
            if (onPath(id, edge.getToNode())) {
                dot << ", color=orange, penwidth=3";
            }
            dot << "];\n";
        }
    }
    dot << "}\n";
    return dot.str();
}
