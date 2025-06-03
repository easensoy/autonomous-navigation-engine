#include "graph_operations/GraphBuilder.hpp"
#include <iostream>
#include <random>
#include <algorithm>
#include <cmath>

class RandomGraphGenerator {
private:
    Graph* graph;
    std::mt19937 rng;
    
public:
    RandomGraphGenerator(Graph* targetGraph) : graph(targetGraph), rng(std::random_device{}()) {}
    
    void generateErdosRenyi(int nodeCount, double probability) {
        std::cout << "[RANDOM_GENERATOR] Creating Erdős-Rényi graph with " << nodeCount 
                  << " nodes, p=" << probability << std::endl;
        
        // Add nodes
        for (int i = 0; i < nodeCount; ++i) {
            double x = std::uniform_real_distribution<double>(0, 100)(rng);
            double y = std::uniform_real_distribution<double>(0, 100)(rng);
            graph->addNode(i, "random_" + std::to_string(i), x, y);
        }
        
        // Add edges with probability p
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        int edgesAdded = 0;
        
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = i + 1; j < nodeCount; ++j) {
                if (dist(rng) < probability) {
                    const Node& node1 = graph->getNode(i);
                    const Node& node2 = graph->getNode(j);
                    double weight = node1.euclideanDistance(node2);
                    graph->addEdge(i, j, weight, true);
                    edgesAdded++;
                }
            }
        }
        
        std::cout << "[RANDOM_GENERATOR] Generated " << edgesAdded << " edges" << std::endl;
    }
    
    void generateBarabasiAlbert(int nodeCount, int edgesPerNode) {
        std::cout << "[RANDOM_GENERATOR] Creating Barabási-Albert graph with " << nodeCount 
                  << " nodes, m=" << edgesPerNode << std::endl;
        
        if (edgesPerNode >= nodeCount) {
            std::cout << "[RANDOM_GENERATOR] Too many edges per node requested" << std::endl;
            return;
        }
        
        // Start with complete graph of m nodes
        for (int i = 0; i < edgesPerNode + 1; ++i) {
            double x = std::uniform_real_distribution<double>(0, 100)(rng);
            double y = std::uniform_real_distribution<double>(0, 100)(rng);
            graph->addNode(i, "ba_" + std::to_string(i), x, y);
        }
        
        // Connect initial nodes
        for (int i = 0; i < edgesPerNode + 1; ++i) {
            for (int j = i + 1; j < edgesPerNode + 1; ++j) {
                const Node& node1 = graph->getNode(i);
                const Node& node2 = graph->getNode(j);
                double weight = node1.euclideanDistance(node2);
                graph->addEdge(i, j, weight, true);
            }
        }
        
        // Add remaining nodes with preferential attachment
        for (int newNode = edgesPerNode + 1; newNode < nodeCount; ++newNode) {
            double x = std::uniform_real_distribution<double>(0, 100)(rng);
            double y = std::uniform_real_distribution<double>(0, 100)(rng);
            graph->addNode(newNode, "ba_" + std::to_string(newNode), x, y);
            
            // Calculate degrees for preferential attachment
            std::vector<int> degrees(newNode);
            int totalDegree = 0;
            
            for (int i = 0; i < newNode; ++i) {
                degrees[i] = graph->getNeighbors(i).size();
                totalDegree += degrees[i];
            }
            
            // Select m nodes to connect to based on degree
            std::vector<int> targetNodes;
            for (int m = 0; m < edgesPerNode; ++m) {
                int selected = selectByDegree(degrees, totalDegree, targetNodes);
                if (selected != -1) {
                    targetNodes.push_back(selected);
                    const Node& node1 = graph->getNode(newNode);
                    const Node& node2 = graph->getNode(selected);
                    double weight = node1.euclideanDistance(node2);
                    graph->addEdge(newNode, selected, weight, true);
                }
            }
        }
        
        std::cout << "[RANDOM_GENERATOR] Barabási-Albert graph completed" << std::endl;
    }
    
    void generateWattsStrogatz(int nodeCount, int nearestNeighbors, double rewiringProb) {
        std::cout << "[RANDOM_GENERATOR] Creating Watts-Strogatz graph with " << nodeCount 
                  << " nodes, k=" << nearestNeighbors << ", p=" << rewiringProb << std::endl;
        
        // Create nodes in a circle
        for (int i = 0; i < nodeCount; ++i) {
            double angle = 2 * M_PI * i / nodeCount;
            double x = 50 + 40 * std::cos(angle);
            double y = 50 + 40 * std::sin(angle);
            graph->addNode(i, "ws_" + std::to_string(i), x, y);
        }
        
        // Create regular ring lattice
        for (int i = 0; i < nodeCount; ++i) {
            for (int k = 1; k <= nearestNeighbors / 2; ++k) {
                int j = (i + k) % nodeCount;
                const Node& node1 = graph->getNode(i);
                const Node& node2 = graph->getNode(j);
                double weight = node1.euclideanDistance(node2);
                graph->addEdge(i, j, weight, true);
            }
        }
        
        // Rewire edges with probability p
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        std::uniform_int_distribution<int> nodeDist(0, nodeCount - 1);
        
        std::vector<std::pair<int, int>> edgesToRewire;
        for (int i = 0; i < nodeCount; ++i) {
            for (int k = 1; k <= nearestNeighbors / 2; ++k) {
                int j = (i + k) % nodeCount;
                if (dist(rng) < rewiringProb) {
                    edgesToRewire.push_back({i, j});
                }
            }
        }
        
        for (auto [i, j] : edgesToRewire) {
            graph->removeEdge(i, j);
            
            // Find new target that's not already connected
            int newTarget;
            do {
                newTarget = nodeDist(rng);
            } while (newTarget == i || graph->hasEdge(i, newTarget));
            
            const Node& node1 = graph->getNode(i);
            const Node& node2 = graph->getNode(newTarget);
            double weight = node1.euclideanDistance(node2);
            graph->addEdge(i, newTarget, weight, true);
        }
        
        std::cout << "[RANDOM_GENERATOR] Rewired " << edgesToRewire.size() << " edges" << std::endl;
    }
    
    void generateRandomGeometric(int nodeCount, double radius) {
        std::cout << "[RANDOM_GENERATOR] Creating random geometric graph with " << nodeCount 
                  << " nodes, radius=" << radius << std::endl;
        
        // Place nodes randomly
        std::uniform_real_distribution<double> posDist(0, 100);
        
        for (int i = 0; i < nodeCount; ++i) {
            double x = posDist(rng);
            double y = posDist(rng);
            graph->addNode(i, "rgg_" + std::to_string(i), x, y);
        }
        
        // Connect nodes within radius
        int edgesAdded = 0;
        for (int i = 0; i < nodeCount; ++i) {
            for (int j = i + 1; j < nodeCount; ++j) {
                const Node& node1 = graph->getNode(i);
                const Node& node2 = graph->getNode(j);
                double distance = node1.euclideanDistance(node2);
                
                if (distance <= radius) {
                    graph->addEdge(i, j, distance, true);
                    edgesAdded++;
                }
            }
        }
        
        std::cout << "[RANDOM_GENERATOR] Connected " << edgesAdded << " node pairs within radius" << std::endl;
    }
    
    void generateScaleFree(int nodeCount, double gamma) {
        std::cout << "[RANDOM_GENERATOR] Creating scale-free graph with " << nodeCount 
                  << " nodes, γ=" << gamma << std::endl;
        
        // Create nodes
        for (int i = 0; i < nodeCount; ++i) {
            double x = std::uniform_real_distribution<double>(0, 100)(rng);
            double y = std::uniform_real_distribution<double>(0, 100)(rng);
            graph->addNode(i, "sf_" + std::to_string(i), x, y);
        }
        
        // Generate degree sequence following power law
        std::vector<int> degrees;
        double normalizationConstant = 0;
        
        for (int k = 1; k <= nodeCount; ++k) {
            normalizationConstant += std::pow(k, -gamma);
        }
        
        std::uniform_real_distribution<double> dist(0.0, 1.0);
        
        for (int i = 0; i < nodeCount; ++i) {
            double r = dist(rng);
            double cumulativeProb = 0;
            int degree = 1;
            
            for (int k = 1; k <= nodeCount; ++k) {
                cumulativeProb += std::pow(k, -gamma) / normalizationConstant;
                if (r <= cumulativeProb) {
                    degree = k;
                    break;
                }
            }
            
            degrees.push_back(std::min(degree, nodeCount - 1));
        }
        
        // Configuration model to realize degree sequence
        realizeDegreeSequence(degrees);
        
        std::cout << "[RANDOM_GENERATOR] Scale-free graph completed" << std::endl;
    }
    
private:
    int selectByDegree(const std::vector<int>& degrees, int totalDegree, 
                      const std::vector<int>& exclude) {
        if (totalDegree == 0) return -1;
        
        std::uniform_int_distribution<int> dist(0, totalDegree - 1);
        int target = dist(rng);
        int sum = 0;
        
        for (size_t i = 0; i < degrees.size(); ++i) {
            if (std::find(exclude.begin(), exclude.end(), i) != exclude.end()) {
                continue;
            }
            
            sum += degrees[i];
            if (target < sum) {
                return static_cast<int>(i);
            }
        }
        
        return -1;
    }
    
    void realizeDegreeSequence(std::vector<int>& degrees) {
        // Ensure sum of degrees is even
        int sum = std::accumulate(degrees.begin(), degrees.end(), 0);
        if (sum % 2 == 1) {
            degrees[0]++;
        }
        
        // Create edge list based on degrees
        std::vector<int> stubs;
        for (size_t i = 0; i < degrees.size(); ++i) {
            for (int j = 0; j < degrees[i]; ++j) {
                stubs.push_back(static_cast<int>(i));
            }
        }
        
        // Randomly pair stubs to create edges
        std::shuffle(stubs.begin(), stubs.end(), rng);
        
        for (size_t i = 0; i < stubs.size(); i += 2) {
            if (i + 1 < stubs.size()) {
                int node1 = stubs[i];
                int node2 = stubs[i + 1];
                
                if (node1 != node2 && !graph->hasEdge(node1, node2)) {
                    const Node& n1 = graph->getNode(node1);
                    const Node& n2 = graph->getNode(node2);
                    double weight = n1.euclideanDistance(n2);
                    graph->addEdge(node1, node2, weight, true);
                }
            }
        }
    }
};

void GraphBuilder::buildRandomGraph(int nodeCount, double connectionProbability) {
    std::cout << "[GRAPH_BUILDER] Building random graph" << std::endl;
    
    if (!targetGraph) {
        std::cout << "[GRAPH_BUILDER] Cannot build graph - null target graph" << std::endl;
        return;
    }
    
    RandomGraphGenerator generator(targetGraph);
    generator.generateErdosRenyi(nodeCount, connectionProbability);
    
    std::cout << "[GRAPH_BUILDER] Random graph construction completed" << std::endl;
}

// Extended random graph functions
void buildBarabasiAlbertGraph(Graph* graph, int nodeCount, int edgesPerNode) {
    RandomGraphGenerator generator(graph);
    generator.generateBarabasiAlbert(nodeCount, edgesPerNode);
}

void buildWattsStrogatzGraph(Graph* graph, int nodeCount, int k, double p) {
    RandomGraphGenerator generator(graph);
    generator.generateWattsStrogatz(nodeCount, k, p);
}

void buildRandomGeometricGraph(Graph* graph, int nodeCount, double radius) {
    RandomGraphGenerator generator(graph);
    generator.generateRandomGeometric(nodeCount, radius);
}

void buildScaleFreeGraph(Graph* graph, int nodeCount, double gamma) {
    RandomGraphGenerator generator(graph);
    generator.generateScaleFree(nodeCount, gamma);
}