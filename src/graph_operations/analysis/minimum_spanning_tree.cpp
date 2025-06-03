#include "graph_operations/GraphAnalysis.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <numeric>

class MinimumSpanningTreeCalculator {
private:
    const Graph* graph;
    mutable std::mutex calculationMutex;
    
    struct MSTEdge {
        int fromNode;
        int toNode;
        double weight;
        
        bool operator<(const MSTEdge& other) const {
            return weight < other.weight;
        }
        
        bool operator>(const MSTEdge& other) const {
            return weight > other.weight;
        }
    };
    
    struct MSTResult {
        std::vector<MSTEdge> mstEdges;
        double totalWeight;
        size_t edgeCount;
        std::chrono::steady_clock::time_point calculationTimestamp;
        double calculationTime;
        std::string algorithm;
        bool isValid;
        bool isConnected;
        size_t componentsFound;
        size_t edgesProcessed;
        size_t nodesProcessed;
    };
    
    struct CalculationMetrics {
        size_t totalCalculations;
        size_t kruskalCalculations;
        size_t primCalculations;
        double averageCalculationTime;
        double averageMSTWeight;
        double averageEdgeCount;
        std::chrono::steady_clock::time_point lastCalculation;
        size_t cacheHits;
        size_t cacheMisses;
        double averageEdgesProcessed;
        double averageNodesProcessed;
    };
    
    struct UnionFind {
        std::unordered_map<int, int> parent;
        std::unordered_map<int, int> rank;
        
        void makeSet(int x) {
            parent[x] = x;
            rank[x] = 0;
        }
        
        int find(int x) {
            if (parent[x] != x) {
                parent[x] = find(parent[x]); // Path compression
            }
            return parent[x];
        }
        
        bool unite(int x, int y) {
            int rootX = find(x);
            int rootY = find(y);
            
            if (rootX == rootY) {
                return false; // Already in same set
            }
            
            // Union by rank
            if (rank[rootX] < rank[rootY]) {
                parent[rootX] = rootY;
            } else if (rank[rootX] > rank[rootY]) {
                parent[rootY] = rootX;
            } else {
                parent[rootY] = rootX;
                rank[rootX]++;
            }
            
            return true;
        }
    };
    
    MSTResult cachedResult;
    CalculationMetrics metrics;
    bool cacheEnabled;
    double cacheValidityDuration;
    std::string preferredAlgorithm;
    bool includeDisconnectedComponents;
    std::function<void(const MSTResult&)> resultCallback;
    
public:
    explicit MinimumSpanningTreeCalculator(const Graph* environment) 
        : graph(environment), cacheEnabled(true), cacheValidityDuration(300.0),
          preferredAlgorithm("auto"), includeDisconnectedComponents(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeCalculationMetrics();
        invalidateCachedResult();
        
        std::cout << "[MST_CALCULATOR] Minimum spanning tree calculation system initialized" << std::endl;
    }
    
    MSTResult calculateMST() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        auto calculationStart = std::chrono::steady_clock::now();
        std::cout << "[MST_CALCULATOR] Initiating minimum spanning tree calculation" << std::endl;
        
        // Check cache validity
        if (cacheEnabled && isCachedResultValid()) {
            std::cout << "[MST_CALCULATOR] Using cached MST result" << std::endl;
            metrics.cacheHits++;
            return cachedResult;
        }
        
        metrics.cacheMisses++;
        
        // Select algorithm based on graph characteristics
        std::string algorithm = selectOptimalAlgorithm();
        
        MSTResult result;
        if (algorithm == "kruskal") {
            std::cout << "[MST_CALCULATOR] Using Kruskal's algorithm" << std::endl;
            result = calculateMSTKruskal();
            metrics.kruskalCalculations++;
        } else if (algorithm == "prim") {
            std::cout << "[MST_CALCULATOR] Using Prim's algorithm" << std::endl;
            result = calculateMSTPrim();
            metrics.primCalculations++;
        } else {
            std::cout << "[MST_CALCULATOR] Auto-selecting algorithm based on graph density" << std::endl;
            
            // Choose based on graph density
            double density = calculateGraphDensity();
            if (density > 0.5) {
                result = calculateMSTPrim();
                metrics.primCalculations++;
            } else {
                result = calculateMSTKruskal();
                metrics.kruskalCalculations++;
            }
        }
        
        auto calculationEnd = std::chrono::steady_clock::now();
        result.calculationTime = std::chrono::duration<double>(calculationEnd - calculationStart).count();
        result.calculationTimestamp = calculationEnd;
        result.isValid = true;
        
        // Update metrics
        updateCalculationMetrics(result);
        
        // Cache the result
        if (cacheEnabled) {
            cachedResult = result;
        }
        
        // Trigger callback if configured
        if (resultCallback) {
            resultCallback(result);
        }
        
        std::cout << "[MST_CALCULATOR] MST calculation completed: " << result.mstEdges.size() 
                  << " edges, total weight: " << result.totalWeight 
                  << " (algorithm: " << result.algorithm << ", time: " 
                  << result.calculationTime << "s)" << std::endl;
        
        return result;
    }
    
    MSTResult calculateMSTKruskal() {
        std::cout << "[MST_CALCULATOR] Executing Kruskal's algorithm for MST calculation" << std::endl;
        
        MSTResult result;
        result.algorithm = "kruskal";
        result.totalWeight = 0.0;
        result.edgeCount = 0;
        result.isConnected = false;
        result.componentsFound = 0;
        result.edgesProcessed = 0;
        result.nodesProcessed = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[MST_CALCULATOR] Empty graph - no MST to calculate" << std::endl;
            result.isValid = false;
            return result;
        }
        
        if (nodeIds.size() == 1) {
            std::cout << "[MST_CALCULATOR] Single node graph - MST contains no edges" << std::endl;
            result.isConnected = true;
            result.componentsFound = 1;
            return result;
        }
        
        // Collect all edges
        std::vector<MSTEdge> allEdges;
        std::unordered_set<std::string> addedEdges; // To avoid duplicate edges
        
        for (int nodeId : nodeIds) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            
            for (const Edge& edge : edges) {
                // Create unique edge identifier to avoid duplicates
                std::string edgeId = std::to_string(std::min(edge.getFromNode(), edge.getToNode())) + 
                                   "_" + std::to_string(std::max(edge.getFromNode(), edge.getToNode()));
                
                if (addedEdges.find(edgeId) == addedEdges.end()) {
                    MSTEdge mstEdge;
                    mstEdge.fromNode = edge.getFromNode();
                    mstEdge.toNode = edge.getToNode();
                    mstEdge.weight = edge.getWeight();
                    
                    allEdges.push_back(mstEdge);
                    addedEdges.insert(edgeId);
                }
            }
        }
        
        result.edgesProcessed = allEdges.size();
        result.nodesProcessed = nodeIds.size();
        
        std::cout << "[MST_CALCULATOR] Collected " << allEdges.size() << " edges for processing" << std::endl;
        
        // Sort edges by weight
        std::sort(allEdges.begin(), allEdges.end());
        
        // Initialize Union-Find structure
        UnionFind uf;
        for (int nodeId : nodeIds) {
            uf.makeSet(nodeId);
        }
        
        // Kruskal's algorithm
        for (const MSTEdge& edge : allEdges) {
            if (uf.unite(edge.fromNode, edge.toNode)) {
                result.mstEdges.push_back(edge);
                result.totalWeight += edge.weight;
                result.edgeCount++;
                
                // Stop if we have enough edges for a spanning tree
                if (result.edgeCount == nodeIds.size() - 1) {
                    result.isConnected = true;
                    result.componentsFound = 1;
                    break;
                }
            }
        }
        
        // Check if graph is connected
        if (result.edgeCount < nodeIds.size() - 1) {
            std::cout << "[MST_CALCULATOR] Graph is not connected - found spanning forest" << std::endl;
            result.isConnected = false;
            
            // Count connected components
            std::unordered_set<int> componentRoots;
            for (int nodeId : nodeIds) {
                componentRoots.insert(uf.find(nodeId));
            }
            result.componentsFound = componentRoots.size();
        }
        
        std::cout << "[MST_CALCULATOR] Kruskal's algorithm completed - " << result.edgeCount 
                  << " edges added, " << result.componentsFound << " components" << std::endl;
        
        return result;
    }
    
    MSTResult calculateMSTPrim() {
        std::cout << "[MST_CALCULATOR] Executing Prim's algorithm for MST calculation" << std::endl;
        
        MSTResult result;
        result.algorithm = "prim";
        result.totalWeight = 0.0;
        result.edgeCount = 0;
        result.isConnected = false;
        result.componentsFound = 0;
        result.edgesProcessed = 0;
        result.nodesProcessed = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[MST_CALCULATOR] Empty graph - no MST to calculate" << std::endl;
            result.isValid = false;
            return result;
        }
        
        if (nodeIds.size() == 1) {
            std::cout << "[MST_CALCULATOR] Single node graph - MST contains no edges" << std::endl;
            result.isConnected = true;
            result.componentsFound = 1;
            return result;
        }
        
        // Start from first node
        std::unordered_set<int> inMST;
        std::priority_queue<MSTEdge, std::vector<MSTEdge>, std::greater<MSTEdge>> edgeQueue;
        
        int startNode = nodeIds[0];
        inMST.insert(startNode);
        result.nodesProcessed++;
        
        // Add all edges from start node to priority queue
        addNodeEdgesToQueue(startNode, inMST, edgeQueue);
        
        std::cout << "[MST_CALCULATOR] Starting Prim's algorithm from node " << startNode << std::endl;
        
        while (!edgeQueue.empty() && inMST.size() < nodeIds.size()) {
            MSTEdge currentEdge = edgeQueue.top();
            edgeQueue.pop();
            result.edgesProcessed++;
            
            // Check if this edge connects to a new node
            int newNode = -1;
            if (inMST.find(currentEdge.fromNode) != inMST.end() && 
                inMST.find(currentEdge.toNode) == inMST.end()) {
                newNode = currentEdge.toNode;
            } else if (inMST.find(currentEdge.toNode) != inMST.end() && 
                      inMST.find(currentEdge.fromNode) == inMST.end()) {
                newNode = currentEdge.fromNode;
            }
            
            if (newNode != -1) {
                // Add edge to MST
                result.mstEdges.push_back(currentEdge);
                result.totalWeight += currentEdge.weight;
                result.edgeCount++;
                
                // Add new node to MST
                inMST.insert(newNode);
                result.nodesProcessed++;
                
                // Add all edges from new node to queue
                addNodeEdgesToQueue(newNode, inMST, edgeQueue);
                
                std::cout << "[MST_CALCULATOR] Added edge (" << currentEdge.fromNode 
                          << "," << currentEdge.toNode << ") weight: " << currentEdge.weight 
                          << ", MST size: " << inMST.size() << std::endl;
            }
        }
        
        // Check connectivity
        if (inMST.size() == nodeIds.size()) {
            result.isConnected = true;
            result.componentsFound = 1;
            std::cout << "[MST_CALCULATOR] Graph is connected - found complete spanning tree" << std::endl;
        } else {
            result.isConnected = false;
            
            // Handle disconnected components if enabled
            if (includeDisconnectedComponents) {
                result.componentsFound = handleDisconnectedComponents(result, nodeIds, inMST);
            } else {
                result.componentsFound = calculateComponentCount(nodeIds);
            }
            
            std::cout << "[MST_CALCULATOR] Graph is not connected - processed " 
                      << inMST.size() << "/" << nodeIds.size() << " nodes" << std::endl;
        }
        
        std::cout << "[MST_CALCULATOR] Prim's algorithm completed - " << result.edgeCount 
                  << " edges, " << result.componentsFound << " components" << std::endl;
        
        return result;
    }
    
    std::vector<MSTResult> calculateMSTForest() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "[MST_CALCULATOR] Calculating minimum spanning forest for disconnected graph" << std::endl;
        
        std::vector<MSTResult> forest;
        std::vector<int> nodeIds = graph->getAllNodeIds();
        std::unordered_set<int> processedNodes;
        
        while (processedNodes.size() < nodeIds.size()) {
            // Find next unprocessed component
            int startNode = -1;
            for (int nodeId : nodeIds) {
                if (processedNodes.find(nodeId) == processedNodes.end()) {
                    startNode = nodeId;
                    break;
                }
            }
            
            if (startNode == -1) break;
            
            // Find connected component starting from this node
            std::vector<int> componentNodes = findConnectedComponent(startNode);
            
            // Calculate MST for this component
            MSTResult componentMST = calculateMSTForComponent(componentNodes);
            forest.push_back(componentMST);
            
            // Mark nodes as processed
            for (int nodeId : componentNodes) {
                processedNodes.insert(nodeId);
            }
            
            std::cout << "[MST_CALCULATOR] Processed component with " << componentNodes.size() 
                      << " nodes, MST weight: " << componentMST.totalWeight << std::endl;
        }
        
        std::cout << "[MST_CALCULATOR] Spanning forest calculation completed - " 
                  << forest.size() << " trees" << std::endl;
        
        return forest;
    }
    
    double calculateMSTWeight() {
        MSTResult result = calculateMST();
        return result.totalWeight;
    }
    
    bool isGraphConnected() {
        MSTResult result = calculateMST();
        return result.isConnected;
    }
    
    size_t getComponentCount() {
        MSTResult result = calculateMST();
        return result.componentsFound;
    }
    
    void configureMST(const std::string& algorithm, bool enableCache, 
                     double cacheValiditySeconds, bool includeDisconnected) {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        preferredAlgorithm = algorithm;
        cacheEnabled = enableCache;
        cacheValidityDuration = cacheValiditySeconds;
        includeDisconnectedComponents = includeDisconnected;
        
        std::cout << "[MST_CALCULATOR] MST calculation parameters configured:" << std::endl;
        std::cout << "[MST_CALCULATOR]   Preferred algorithm: " << preferredAlgorithm << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[MST_CALCULATOR]   Include disconnected components: " << (includeDisconnectedComponents ? "Yes" : "No") << std::endl;
    }
    
    void setResultCallback(std::function<void(const MSTResult&)> callback) {
        resultCallback = callback;
        std::cout << "[MST_CALCULATOR] Result callback configured" << std::endl;
    }
    
    void invalidateCache() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        invalidateCachedResult();
        std::cout << "[MST_CALCULATOR] MST calculation cache invalidated" << std::endl;
    }
    
    CalculationMetrics getCalculationMetrics() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        return metrics;
    }
    
    void generateMSTReport() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "\n[MST_CALCULATOR] === MINIMUM SPANNING TREE ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[MST_CALCULATOR] System Configuration:" << std::endl;
        std::cout << "[MST_CALCULATOR]   Preferred algorithm: " << preferredAlgorithm << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[MST_CALCULATOR]   Include disconnected components: " << (includeDisconnectedComponents ? "Yes" : "No") << std::endl;
        
        std::cout << "[MST_CALCULATOR] Graph Properties:" << std::endl;
        std::cout << "[MST_CALCULATOR]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[MST_CALCULATOR]   Total edges: " << graph->getEdgeCount() << std::endl;
        std::cout << "[MST_CALCULATOR]   Graph density: " << calculateGraphDensity() << std::endl;
        
        if (cachedResult.isValid) {
            std::cout << "[MST_CALCULATOR] Last Calculation Results:" << std::endl;
            std::cout << "[MST_CALCULATOR]   MST total weight: " << cachedResult.totalWeight << std::endl;
            std::cout << "[MST_CALCULATOR]   MST edge count: " << cachedResult.edgeCount << std::endl;
            std::cout << "[MST_CALCULATOR]   Graph connected: " << (cachedResult.isConnected ? "Yes" : "No") << std::endl;
            std::cout << "[MST_CALCULATOR]   Connected components: " << cachedResult.componentsFound << std::endl;
            std::cout << "[MST_CALCULATOR]   Algorithm used: " << cachedResult.algorithm << std::endl;
            std::cout << "[MST_CALCULATOR]   Calculation time: " << cachedResult.calculationTime << " seconds" << std::endl;
            std::cout << "[MST_CALCULATOR]   Edges processed: " << cachedResult.edgesProcessed << std::endl;
            std::cout << "[MST_CALCULATOR]   Nodes processed: " << cachedResult.nodesProcessed << std::endl;
        }
        
        std::cout << "[MST_CALCULATOR] Performance Metrics:" << std::endl;
        std::cout << "[MST_CALCULATOR]   Total calculations: " << metrics.totalCalculations << std::endl;
        std::cout << "[MST_CALCULATOR]   Kruskal calculations: " << metrics.kruskalCalculations << std::endl;
        std::cout << "[MST_CALCULATOR]   Prim calculations: " << metrics.primCalculations << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache hits: " << metrics.cacheHits << std::endl;
        std::cout << "[MST_CALCULATOR]   Cache misses: " << metrics.cacheMisses << std::endl;
        std::cout << "[MST_CALCULATOR]   Average calculation time: " << metrics.averageCalculationTime << " seconds" << std::endl;
        std::cout << "[MST_CALCULATOR]   Average MST weight: " << metrics.averageMSTWeight << std::endl;
        std::cout << "[MST_CALCULATOR]   Average edge count: " << metrics.averageEdgeCount << std::endl;
        
        if (metrics.totalCalculations > 0) {
            double cacheHitRate = static_cast<double>(metrics.cacheHits) / 
                                 (metrics.cacheHits + metrics.cacheMisses) * 100.0;
            std::cout << "[MST_CALCULATOR]   Cache hit rate: " << cacheHitRate << "%" << std::endl;
            
            if (metrics.kruskalCalculations > 0 && metrics.primCalculations > 0) {
                double kruskalRatio = static_cast<double>(metrics.kruskalCalculations) / 
                                     metrics.totalCalculations * 100.0;
                std::cout << "[MST_CALCULATOR]   Kruskal usage ratio: " << kruskalRatio << "%" << std::endl;
            }
        }
        
        std::cout << "[MST_CALCULATOR] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeCalculationMetrics() {
        metrics.totalCalculations = 0;
        metrics.kruskalCalculations = 0;
        metrics.primCalculations = 0;
        metrics.averageCalculationTime = 0.0;
        metrics.averageMSTWeight = 0.0;
        metrics.averageEdgeCount = 0.0;
        metrics.lastCalculation = std::chrono::steady_clock::now();
        metrics.cacheHits = 0;
        metrics.cacheMisses = 0;
        metrics.averageEdgesProcessed = 0.0;
        metrics.averageNodesProcessed = 0.0;
    }
    
    void invalidateCachedResult() {
        cachedResult.mstEdges.clear();
        cachedResult.totalWeight = 0.0;
        cachedResult.edgeCount = 0;
        cachedResult.calculationTimestamp = std::chrono::steady_clock::now() - std::chrono::hours(1);
        cachedResult.calculationTime = 0.0;
        cachedResult.algorithm = "";
        cachedResult.isValid = false;
        cachedResult.isConnected = false;
        cachedResult.componentsFound = 0;
        cachedResult.edgesProcessed = 0;
        cachedResult.nodesProcessed = 0;
    }
    
    bool isCachedResultValid() const {
        if (!cachedResult.isValid) {
            return false;
        }
        
        auto currentTime = std::chrono::steady_clock::now();
        auto timeSinceCalculation = std::chrono::duration<double>(
            currentTime - cachedResult.calculationTimestamp);
        
        return timeSinceCalculation.count() < cacheValidityDuration;
    }
    
    std::string selectOptimalAlgorithm() {
        if (preferredAlgorithm != "auto") {
            return preferredAlgorithm;
        }
        
        // Auto-select based on graph characteristics
        double density = calculateGraphDensity();
        size_t nodeCount = graph->getNodeCount();
        size_t edgeCount = graph->getEdgeCount();
        
        // Prim's is generally better for dense graphs
        // Kruskal's is generally better for sparse graphs
        if (density > 0.5 || (nodeCount < 100 && edgeCount > nodeCount * 2)) {
            return "prim";
        } else {
            return "kruskal";
        }
    }
    
    double calculateGraphDensity() const {
        size_t nodeCount = graph->getNodeCount();
        size_t edgeCount = graph->getEdgeCount();
        
        if (nodeCount <= 1) {
            return 0.0;
        }
        
        size_t maxPossibleEdges = nodeCount * (nodeCount - 1) / 2;
        return static_cast<double>(edgeCount) / maxPossibleEdges;
    }
    
    void addNodeEdgesToQueue(int nodeId, const std::unordered_set<int>& inMST, 
                           std::priority_queue<MSTEdge, std::vector<MSTEdge>, std::greater<MSTEdge>>& edgeQueue) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        
        for (const Edge& edge : edges) {
            int otherNode = edge.getToNode();
            
            // Only add edges to nodes not yet in MST
            if (inMST.find(otherNode) == inMST.end()) {
                MSTEdge mstEdge;
                mstEdge.fromNode = edge.getFromNode();
                mstEdge.toNode = edge.getToNode();
                mstEdge.weight = edge.getWeight();
                
                edgeQueue.push(mstEdge);
            }
        }
    }
    
    size_t handleDisconnectedComponents(MSTResult& result, const std::vector<int>& nodeIds, 
                                      const std::unordered_set<int>& processedNodes) {
        std::unordered_set<int> remainingNodes;
        for (int nodeId : nodeIds) {
            if (processedNodes.find(nodeId) == processedNodes.end()) {
                remainingNodes.insert(nodeId);
            }
        }
        
        size_t componentCount = 1; // We already have one component
        
        while (!remainingNodes.empty()) {
            int startNode = *remainingNodes.begin();
            std::vector<int> componentNodes = findConnectedComponent(startNode);
            
            // Calculate MST for this component and merge with main result
            MSTResult componentMST = calculateMSTForComponent(componentNodes);
            
            // Add component edges to main result
            result.mstEdges.insert(result.mstEdges.end(), 
                                 componentMST.mstEdges.begin(), componentMST.mstEdges.end());
            result.totalWeight += componentMST.totalWeight;
            result.edgeCount += componentMST.edgeCount;
            
            // Remove processed nodes
            for (int nodeId : componentNodes) {
                remainingNodes.erase(nodeId);
            }
            
            componentCount++;
        }
        
        return componentCount;
    }
    
    std::vector<int> findConnectedComponent(int startNode) {
        std::vector<int> component;
        std::unordered_set<int> visited;
        std::queue<int> toVisit;
        
        toVisit.push(startNode);
        visited.insert(startNode);
        
        while (!toVisit.empty()) {
            int current = toVisit.front();
            toVisit.pop();
            component.push_back(current);
            
            std::vector<int> neighbors = graph->getNeighbors(current);
            for (int neighbor : neighbors) {
                if (visited.find(neighbor) == visited.end()) {
                    visited.insert(neighbor);
                    toVisit.push(neighbor);
                }
            }
        }
        
        return component;
    }
    
    MSTResult calculateMSTForComponent(const std::vector<int>& componentNodes) {
        // This is a simplified component MST calculation
        // In practice, you would create a subgraph and run the full algorithm
        MSTResult componentResult;
        componentResult.algorithm = "component_" + preferredAlgorithm;
        componentResult.totalWeight = 0.0;
        componentResult.edgeCount = 0;
        componentResult.isConnected = true;
        componentResult.componentsFound = 1;
        componentResult.isValid = true;
        
        // For simplicity, we'll use a basic approach
        if (componentNodes.size() <= 1) {
            return componentResult;
        }
        
        std::unordered_set<int> inMST;
        inMST.insert(componentNodes[0]);
        
        while (inMST.size() < componentNodes.size()) {
            double minWeight = std::numeric_limits<double>::infinity();
            MSTEdge bestEdge;
            bool foundEdge = false;
            
            for (int nodeInMST : inMST) {
                const std::vector<Edge>& edges = graph->getEdgesFrom(nodeInMST);
                
                for (const Edge& edge : edges) {
                    int otherNode = edge.getToNode();
                    
                    if (inMST.find(otherNode) == inMST.end() && 
                        std::find(componentNodes.begin(), componentNodes.end(), otherNode) != componentNodes.end()) {
                        
                        if (edge.getWeight() < minWeight) {
                            minWeight = edge.getWeight();
                            bestEdge.fromNode = edge.getFromNode();
                            bestEdge.toNode = edge.getToNode();
                            bestEdge.weight = edge.getWeight();
                            foundEdge = true;
                        }
                    }
                }
            }
            
            if (foundEdge) {
                componentResult.mstEdges.push_back(bestEdge);
                componentResult.totalWeight += bestEdge.weight;
                componentResult.edgeCount++;
                inMST.insert(bestEdge.toNode);
            } else {
                break;
            }
        }
        
        return componentResult;
    }
    
    size_t calculateComponentCount(const std::vector<int>& nodeIds) {
        std::unordered_set<int> visited;
        size_t componentCount = 0;
        
        for (int nodeId : nodeIds) {
            if (visited.find(nodeId) == visited.end()) {
                std::vector<int> component = findConnectedComponent(nodeId);
                for (int componentNode : component) {
                    visited.insert(componentNode);
                }
                componentCount++;
            }
        }
        
        return componentCount;
    }
    
    void updateCalculationMetrics(const MSTResult& result) {
        metrics.totalCalculations++;
        metrics.lastCalculation = result.calculationTimestamp;
        
        // Update averages
        if (metrics.averageCalculationTime == 0.0) {
            metrics.averageCalculationTime = result.calculationTime;
        } else {
            metrics.averageCalculationTime = (metrics.averageCalculationTime + result.calculationTime) / 2.0;
        }
        
        if (metrics.averageMSTWeight == 0.0) {
            metrics.averageMSTWeight = result.totalWeight;
        } else {
            metrics.averageMSTWeight = (metrics.averageMSTWeight + result.totalWeight) / 2.0;
        }
        
        if (metrics.averageEdgeCount == 0.0) {
            metrics.averageEdgeCount = static_cast<double>(result.edgeCount);
        } else {
            metrics.averageEdgeCount = (metrics.averageEdgeCount + result.edgeCount) / 2.0;
        }
        
        if (metrics.averageEdgesProcessed == 0.0) {
            metrics.averageEdgesProcessed = static_cast<double>(result.edgesProcessed);
        } else {
            metrics.averageEdgesProcessed = (metrics.averageEdgesProcessed + result.edgesProcessed) / 2.0;
        }
        
        if (metrics.averageNodesProcessed == 0.0) {
            metrics.averageNodesProcessed = static_cast<double>(result.nodesProcessed);
        } else {
            metrics.averageNodesProcessed = (metrics.averageNodesProcessed + result.nodesProcessed) / 2.0;
        }
    }
};