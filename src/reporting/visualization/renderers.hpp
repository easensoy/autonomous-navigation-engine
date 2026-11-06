#pragma once
// Declarations of the free functions exported by the three renderer sources in this folder
// (ascii_graph_renderer.cpp, console_path_printer.cpp, svg_export.cpp). Each renderer keeps a
// file-static instance bound to the graph it was last initialised with, so callers that switch
// graphs call the matching initialize* function first.
#include "core/Graph.hpp"
#include <string>
#include <vector>

// ascii_graph_renderer.cpp
void initializeASCIIRenderer(const Graph* graph, int width, int height, double scale);
std::string renderGraphASCII(const Graph* graph, const std::vector<int>& highlightPath, int width, int height);
std::string renderPathASCII(const Graph* graph, const std::vector<int>& path, int width, int height);
void setASCIICanvasDimensions(int width, int height);

// console_path_printer.cpp
void initializeConsolePrinter(const Graph* graph);
void printPathToConsole(const Graph* graph, const std::vector<int>& path);
void printDetailedPath(const Graph* graph, const std::vector<int>& path);
void printPathComparison(const Graph* graph, const std::vector<int>& path1, const std::vector<int>& path2,
                         const std::string& label1, const std::string& label2);
void setConsoleColors(bool enabled);

// svg_export.cpp
void initializeSVGExporter(const Graph* graph, double width, double height);
bool exportGraphToSVG(const Graph* graph, const std::string& filename, const std::vector<int>& highlightPath,
                      double width, double height);
bool exportPathToSVG(const Graph* graph, const std::string& filename, const std::vector<int>& path,
                     double width, double height);
std::string generateSVGString(const Graph* graph, const std::vector<int>& highlightPath, double width,
                              double height);
