#pragma once
#include "core/Graph.hpp"
#include <vector>
#include <string>

enum class VisualizationFormat {
    ASCII_ART,
    SVG,
    JSON,
    DOT_GRAPH,
    CONSOLE_TEXT
};

class PathVisualizer {
private:
    const Graph* environment;
    double scaleFactor;
    int canvasWidth, canvasHeight;
    
    std::string generateASCIIVisualization(const std::vector<int>& path) const;
    std::string generateSVGVisualization(const std::vector<int>& path) const;
    std::string generateDotVisualization(const std::vector<int>& path) const;

public:
    explicit PathVisualizer(const Graph* graph);
    
    void visualizePath(const std::vector<int>& path, VisualizationFormat format = VisualizationFormat::CONSOLE_TEXT) const;
    void visualizeMultiplePaths(const std::vector<std::vector<int>>& paths) const;
    void visualizeGraphStructure() const;
    
    void exportVisualization(const std::vector<int>& path, const std::string& filename, VisualizationFormat format) const;
    void setCanvasDimensions(int width, int height);
    void setScaleFactor(double scale);
    
    
    std::string generatePathStatistics(const std::vector<int>& path) const;
    void comparePathVisualizations(const std::vector<int>& path1, const std::vector<int>& path2) const;
    void createInteractiveVisualization(const std::vector<int>& path, const std::string& outputFile) const;
};