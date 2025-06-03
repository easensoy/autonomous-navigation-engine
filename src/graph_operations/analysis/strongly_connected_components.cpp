#include "graph_operations/GraphAnalysis.hpp"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <mutex>
#include <stack>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <numeric>

class StronglyConnectedComponentsCalculator {
private:
    const Graph* graph;
    mutable std::mutex calculationMutex;
    
    struct SCCComponent {
        int componentId;
        std::vector<int> nodes;
        size_t nodeCount;
        size_t internalEdges;
        size_t outgoingEdges;
        size_t incomingEdges;
        bool isTrivial; // Single node with no self-loop
        double componentDensity;
        std::vector<int> connectedComponents; // Components this one connects to
    };
    
    struct SCCResult {
        std::vector<SCCComponent> components;
        size_t componentCount;
        size_t largestComponentSize;
        size_t smallestComponentSize;
        double averageComponentSize;
        bool isStronglyConnected;
        std::chrono::steady_clock::time_point calculationTimestamp;
        double calculationTime;
        std::string algorithm;
        bool isValid;
        size_t nodesProcessed;
        size_t edgesProcessed;
        size_t trivialComponents;
        size_t nonTrivialComponents;
    };
    
    struct CalculationMetrics {
        size_t totalCalculations;
        size_t tarjanCalculations;
        size_t kosarajuCalculations;
        double averageCalculationTime;
        double averageComponentCount;
        double averageComponentSize;
        std::chrono::steady_clock::time_point lastCalculation;
        size_t cacheHits;
        size_t cacheMisses;
        double averageNodesProcessed;
        double averageEdgesProcessed;
        size_t stronglyConnectedGraphs;
        size_t disconnectedGraphs;
    };
    
    struct TarjanState {
        std::unordered_map<int, int> indices;
        std::unordered_map<int, int> lowlinks;
        std::unordered_set<int> onStack;
        std::stack<int> nodeStack;
        int currentIndex;
        std::vector<std::vector<int>> components;
    };
    
    SCCResult cachedResult;
    CalculationMetrics metrics;
    bool cacheEnabled;
    double cacheValidityDuration;
    std::string preferredAlgorithm;
    bool includeComponentAnalysis;
    bool detectTrivialComponents;
    std::function<void(const SCCResult&)> resultCallback;
    
public:
    explicit StronglyConnectedComponentsCalculator(const Graph* environment) 
        : graph(environment), cacheEnabled(true), cacheValidityDuration(300.0),
          preferredAlgorithm("tarjan"), includeComponentAnalysis(true), detectTrivialComponents(true) {
        
        if (!graph) {
            throw std::invalid_argument("Graph pointer cannot be null");
        }
        
        initializeCalculationMetrics();
        invalidateCachedResult();
        
        std::cout << "[SCC_CALCULATOR] Strongly connected components calculation system initialized" << std::endl;
    }
    
    SCCResult calculateSCC() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        auto calculationStart = std::chrono::steady_clock::now();
        std::cout << "[SCC_CALCULATOR] Initiating strongly connected components calculation" << std::endl;
        
        // Check cache validity
        if (cacheEnabled && isCachedResultValid()) {
            std::cout << "[SCC_CALCULATOR] Using cached SCC result" << std::endl;
            metrics.cacheHits++;
            return cachedResult;
        }
        
        metrics.cacheMisses++;
        
        SCCResult result;
        if (preferredAlgorithm == "tarjan") {
            std::cout << "[SCC_CALCULATOR] Using Tarjan's algorithm" << std::endl;
            result = calculateSCCTarjan();
            metrics.tarjanCalculations++;
        } else if (preferredAlgorithm == "kosaraju") {
            std::cout << "[SCC_CALCULATOR] Using Kosaraju's algorithm" << std::endl;
            result = calculateSCCKosaraju();
            metrics.kosarajuCalculations++;
        } else {
            std::cout << "[SCC_CALCULATOR] Auto-selecting algorithm based on graph characteristics" << std::endl;
            
            // Choose based on graph size and density
            if (graph->getNodeCount() > 10000) {
                result = calculateSCCTarjan(); // Generally more efficient for large graphs
                metrics.tarjanCalculations++;
            } else {
                result = calculateSCCKosaraju(); // Simpler and often faster for smaller graphs
                metrics.kosarajuCalculations++;
            }
        }
        
        auto calculationEnd = std::chrono::steady_clock::now();
        result.calculationTime = std::chrono::duration<double>(calculationEnd - calculationStart).count();
        result.calculationTimestamp = calculationEnd;
        result.isValid = true;
        
        // Perform additional analysis if enabled
        if (includeComponentAnalysis) {
            analyzeComponents(result);
        }
        
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
        
        std::cout << "[SCC_CALCULATOR] SCC calculation completed: " << result.componentCount 
                  << " components found (algorithm: " << result.algorithm 
                  << ", time: " << result.calculationTime << "s)" << std::endl;
        
        return result;
    }
    
    SCCResult calculateSCCTarjan() {
        std::cout << "[SCC_CALCULATOR] Executing Tarjan's algorithm for SCC calculation" << std::endl;
        
        SCCResult result;
        result.algorithm = "tarjan";
        result.componentCount = 0;
        result.isStronglyConnected = false;
        result.nodesProcessed = 0;
        result.edgesProcessed = 0;
        result.trivialComponents = 0;
        result.nonTrivialComponents = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[SCC_CALCULATOR] Empty graph - no components to calculate" << std::endl;
            result.isValid = false;
            return result;
        }
        
        TarjanState state;
        state.currentIndex = 0;
        
        // Run Tarjan's algorithm on all unvisited nodes
        for (int nodeId : nodeIds) {
            if (state.indices.find(nodeId) == state.indices.end()) {
                tarjanDFS(nodeId, state);
            }
        }
        
        result.nodesProcessed = nodeIds.size();
        result.componentCount = state.components.size();
        
        // Convert raw components to structured components
        for (size_t i = 0; i < state.components.size(); ++i) {
            SCCComponent component;
            component.componentId = static_cast<int>(i);
            component.nodes = state.components[i];
            component.nodeCount = component.nodes.size();
            component.isTrivial = (component.nodeCount == 1) && !hasSelfLoop(component.nodes[0]);
            
            if (component.isTrivial) {
                result.trivialComponents++;
            } else {
                result.nonTrivialComponents++;
            }
            
            result.components.push_back(component);
        }
        
        // Check if graph is strongly connected
        result.isStronglyConnected = (result.componentCount == 1) && (result.components[0].nodeCount == nodeIds.size());
        
        // Calculate size statistics
        calculateSizeStatistics(result);
        
        std::cout << "[SCC_CALCULATOR] Tarjan's algorithm completed - " << result.componentCount 
                  << " components (" << result.trivialComponents << " trivial, " 
                  << result.nonTrivialComponents << " non-trivial)" << std::endl;
        
        return result;
    }
    
    SCCResult calculateSCCKosaraju() {
        std::cout << "[SCC_CALCULATOR] Executing Kosaraju's algorithm for SCC calculation" << std::endl;
        
        SCCResult result;
        result.algorithm = "kosaraju";
        result.componentCount = 0;
        result.isStronglyConnected = false;
        result.nodesProcessed = 0;
        result.edgesProcessed = 0;
        result.trivialComponents = 0;
        result.nonTrivialComponents = 0;
        
        std::vector<int> nodeIds = graph->getAllNodeIds();
        if (nodeIds.empty()) {
            std::cout << "[SCC_CALCULATOR] Empty graph - no components to calculate" << std::endl;
            result.isValid = false;
            return result;
        }
        
        // Step 1: Perform DFS to get finishing times
        std::vector<int> finishOrder;
        std::unordered_set<int> visited;
        
        for (int nodeId : nodeIds) {
            if (visited.find(nodeId) == visited.end()) {
                kosarajuDFS1(nodeId, visited, finishOrder);
            }
        }
        
        // Step 2: Create transpose graph (conceptually - we'll handle this in DFS)
        // Step 3: Perform DFS on transpose graph in reverse finish order
        std::reverse(finishOrder.begin(), finishOrder.end());
        visited.clear();
        
        std::vector<std::vector<int>> components;
        
        for (int nodeId : finishOrder) {
            if (visited.find(nodeId) == visited.end()) {
                std::vector<int> component;
                kosarajuDFS2(nodeId, visited, component);
                if (!component.empty()) {
                    components.push_back(component);
                }
            }
        }
        
        result.nodesProcessed = nodeIds.size();
        result.componentCount = components.size();
        
        // Convert raw components to structured components
        for (size_t i = 0; i < components.size(); ++i) {
            SCCComponent component;
            component.componentId = static_cast<int>(i);
            component.nodes = components[i];
            component.nodeCount = component.nodes.size();
            component.isTrivial = (component.nodeCount == 1) && !hasSelfLoop(component.nodes[0]);
            
            if (component.isTrivial) {
                result.trivialComponents++;
            } else {
                result.nonTrivialComponents++;
            }
            
            result.components.push_back(component);
        }
        
        // Check if graph is strongly connected
        result.isStronglyConnected = (result.componentCount == 1) && (result.components[0].nodeCount == nodeIds.size());
        
        // Calculate size statistics
        calculateSizeStatistics(result);
        
        std::cout << "[SCC_CALCULATOR] Kosaraju's algorithm completed - " << result.componentCount 
                  << " components (" << result.trivialComponents << " trivial, " 
                  << result.nonTrivialComponents << " non-trivial)" << std::endl;
        
        return result;
    }
    
    bool isStronglyConnected() {
        SCCResult result = calculateSCC();
        return result.isStronglyConnected;
    }
    
    size_t getComponentCount() {
        SCCResult result = calculateSCC();
        return result.componentCount;
    }
    
    std::vector<int> getLargestComponent() {
        SCCResult result = calculateSCC();
        
        if (result.components.empty()) {
            return {};
        }
        
        auto largestComponent = std::max_element(result.components.begin(), result.components.end(),
            [](const SCCComponent& a, const SCCComponent& b) {
                return a.nodeCount < b.nodeCount;
            });
        
        return largestComponent->nodes;
    }
    
    std::vector<std::vector<int>> getNonTrivialComponents() {
        SCCResult result = calculateSCC();
        std::vector<std::vector<int>> nonTrivialComponents;
        
        for (const SCCComponent& component : result.components) {
            if (!component.isTrivial) {
                nonTrivialComponents.push_back(component.nodes);
            }
        }
        
        std::cout << "[SCC_CALCULATOR] Found " << nonTrivialComponents.size() 
                  << " non-trivial components" << std::endl;
        
        return nonTrivialComponents;
    }
    
    std::unordered_map<int, int> getNodeToComponentMapping() {
        SCCResult result = calculateSCC();
        std::unordered_map<int, int> nodeToComponent;
        
        for (const SCCComponent& component : result.components) {
            for (int nodeId : component.nodes) {
                nodeToComponent[nodeId] = component.componentId;
            }
        }
        
        return nodeToComponent;
    }
    
    bool areNodesInSameComponent(int nodeA, int nodeB) {
        auto nodeToComponent = getNodeToComponentMapping();
        
        auto itA = nodeToComponent.find(nodeA);
        auto itB = nodeToComponent.find(nodeB);
        
        if (itA == nodeToComponent.end() || itB == nodeToComponent.end()) {
            return false;
        }
        
        return itA->second == itB->second;
    }
    
    void configureSCC(const std::string& algorithm, bool enableCache, 
                     double cacheValiditySeconds, bool includeAnalysis, bool detectTrivial) {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        preferredAlgorithm = algorithm;
        cacheEnabled = enableCache;
        cacheValidityDuration = cacheValiditySeconds;
        includeComponentAnalysis = includeAnalysis;
        detectTrivialComponents = detectTrivial;
        
        std::cout << "[SCC_CALCULATOR] SCC calculation parameters configured:" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Preferred algorithm: " << preferredAlgorithm << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Include component analysis: " << (includeComponentAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[SCC_CALCULATOR]   Detect trivial components: " << (detectTrivialComponents ? "Yes" : "No") << std::endl;
    }
    
    void setResultCallback(std::function<void(const SCCResult&)> callback) {
        resultCallback = callback;
        std::cout << "[SCC_CALCULATOR] Result callback configured" << std::endl;
    }
    
    void invalidateCache() {
        std::lock_guard<std::mutex> lock(calculationMutex);
        invalidateCachedResult();
        std::cout << "[SCC_CALCULATOR] SCC calculation cache invalidated" << std::endl;
    }
    
    CalculationMetrics getCalculationMetrics() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        return metrics;
    }
    
    void generateSCCReport() const {
        std::lock_guard<std::mutex> lock(calculationMutex);
        
        std::cout << "\n[SCC_CALCULATOR] === STRONGLY CONNECTED COMPONENTS ANALYSIS REPORT ===" << std::endl;
        
        std::cout << "[SCC_CALCULATOR] System Configuration:" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Preferred algorithm: " << preferredAlgorithm << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache enabled: " << (cacheEnabled ? "Yes" : "No") << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache validity: " << cacheValidityDuration << " seconds" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Include component analysis: " << (includeComponentAnalysis ? "Yes" : "No") << std::endl;
        std::cout << "[SCC_CALCULATOR]   Detect trivial components: " << (detectTrivialComponents ? "Yes" : "No") << std::endl;
        
        std::cout << "[SCC_CALCULATOR] Graph Properties:" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Total nodes: " << graph->getNodeCount() << std::endl;
        std::cout << "[SCC_CALCULATOR]   Total edges: " << graph->getEdgeCount() << std::endl;
        
        if (cachedResult.isValid) {
            std::cout << "[SCC_CALCULATOR] Last Calculation Results:" << std::endl;
            std::cout << "[SCC_CALCULATOR]   Strongly connected: " << (cachedResult.isStronglyConnected ? "Yes" : "No") << std::endl;
            std::cout << "[SCC_CALCULATOR]   Total components: " << cachedResult.componentCount << std::endl;
            std::cout << "[SCC_CALCULATOR]   Trivial components: " << cachedResult.trivialComponents << std::endl;
            std::cout << "[SCC_CALCULATOR]   Non-trivial components: " << cachedResult.nonTrivialComponents << std::endl;
            std::cout << "[SCC_CALCULATOR]   Largest component size: " << cachedResult.largestComponentSize << std::endl;
            std::cout << "[SCC_CALCULATOR]   Smallest component size: " << cachedResult.smallestComponentSize << std::endl;
            std::cout << "[SCC_CALCULATOR]   Average component size: " << cachedResult.averageComponentSize << std::endl;
            std::cout << "[SCC_CALCULATOR]   Algorithm used: " << cachedResult.algorithm << std::endl;
            std::cout << "[SCC_CALCULATOR]   Calculation time: " << cachedResult.calculationTime << " seconds" << std::endl;
            std::cout << "[SCC_CALCULATOR]   Nodes processed: " << cachedResult.nodesProcessed << std::endl;
            std::cout << "[SCC_CALCULATOR]   Edges processed: " << cachedResult.edgesProcessed << std::endl;
            
            if (includeComponentAnalysis && !cachedResult.components.empty()) {
                std::cout << "[SCC_CALCULATOR] Component Details:" << std::endl;
                for (size_t i = 0; i < std::min(static_cast<size_t>(5), cachedResult.components.size()); ++i) {
                    const SCCComponent& comp = cachedResult.components[i];
                    std::cout << "[SCC_CALCULATOR]   Component " << comp.componentId 
                              << ": " << comp.nodeCount << " nodes"
                              << (comp.isTrivial ? " (trivial)" : " (non-trivial)") << std::endl;
                }
                if (cachedResult.components.size() > 5) {
                    std::cout << "[SCC_CALCULATOR]   ... and " << (cachedResult.components.size() - 5) 
                              << " more components" << std::endl;
                }
            }
        }
        
        std::cout << "[SCC_CALCULATOR] Performance Metrics:" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Total calculations: " << metrics.totalCalculations << std::endl;
        std::cout << "[SCC_CALCULATOR]   Tarjan calculations: " << metrics.tarjanCalculations << std::endl;
        std::cout << "[SCC_CALCULATOR]   Kosaraju calculations: " << metrics.kosarajuCalculations << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache hits: " << metrics.cacheHits << std::endl;
        std::cout << "[SCC_CALCULATOR]   Cache misses: " << metrics.cacheMisses << std::endl;
        std::cout << "[SCC_CALCULATOR]   Average calculation time: " << metrics.averageCalculationTime << " seconds" << std::endl;
        std::cout << "[SCC_CALCULATOR]   Average component count: " << metrics.averageComponentCount << std::endl;
        std::cout << "[SCC_CALCULATOR]   Average component size: " << metrics.averageComponentSize << std::endl;
        std::cout << "[SCC_CALCULATOR]   Strongly connected graphs analyzed: " << metrics.stronglyConnectedGraphs << std::endl;
        std::cout << "[SCC_CALCULATOR]   Disconnected graphs analyzed: " << metrics.disconnectedGraphs << std::endl;
        
        if (metrics.totalCalculations > 0) {
            double cacheHitRate = static_cast<double>(metrics.cacheHits) / 
                                 (metrics.cacheHits + metrics.cacheMisses) * 100.0;
            std::cout << "[SCC_CALCULATOR]   Cache hit rate: " << cacheHitRate << "%" << std::endl;
            
            if (metrics.tarjanCalculations > 0 && metrics.kosarajuCalculations > 0) {
                double tarjanRatio = static_cast<double>(metrics.tarjanCalculations) / 
                                   metrics.totalCalculations * 100.0;
                std::cout << "[SCC_CALCULATOR]   Tarjan usage ratio: " << tarjanRatio << "%" << std::endl;
            }
            
            double stronglyConnectedRatio = static_cast<double>(metrics.stronglyConnectedGraphs) / 
                                          metrics.totalCalculations * 100.0;
            std::cout << "[SCC_CALCULATOR]   Strongly connected ratio: " << stronglyConnectedRatio << "%" << std::endl;
        }
        
        std::cout << "[SCC_CALCULATOR] === END REPORT ===" << std::endl;
    }
    
private:
    void initializeCalculationMetrics() {
        metrics.totalCalculations = 0;
        metrics.tarjanCalculations = 0;
        metrics.kosarajuCalculations = 0;
        metrics.averageCalculationTime = 0.0;
        metrics.averageComponentCount = 0.0;
        metrics.averageComponentSize = 0.0;
        metrics.lastCalculation = std::chrono::steady_clock::now();
        metrics.cacheHits = 0;
        metrics.cacheMisses = 0;
        metrics.averageNodesProcessed = 0.0;
        metrics.averageEdgesProcessed = 0.0;
        metrics.stronglyConnectedGraphs = 0;
        metrics.disconnectedGraphs = 0;
    }
    
    void invalidateCachedResult() {
        cachedResult.components.clear();
        cachedResult.componentCount = 0;
        cachedResult.largestComponentSize = 0;
        cachedResult.smallestComponentSize = 0;
        cachedResult.averageComponentSize = 0.0;
        cachedResult.isStronglyConnected = false;
        cachedResult.calculationTimestamp = std::chrono::steady_clock::now() - std::chrono::hours(1);
        cachedResult.calculationTime = 0.0;
        cachedResult.algorithm = "";
        cachedResult.isValid = false;
        cachedResult.nodesProcessed = 0;
        cachedResult.edgesProcessed = 0;
        cachedResult.trivialComponents = 0;
        cachedResult.nonTrivialComponents = 0;
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
    
    void tarjanDFS(int nodeId, TarjanState& state) {
        // Initialize node
        state.indices[nodeId] = state.currentIndex;
        state.lowlinks[nodeId] = state.currentIndex;
        state.currentIndex++;
        state.nodeStack.push(nodeId);
        state.onStack.insert(nodeId);
        
        // Visit neighbors
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            int neighbor = edge.getToNode();
            
            if (state.indices.find(neighbor) == state.indices.end()) {
                // Neighbor not yet visited
                tarjanDFS(neighbor, state);
                state.lowlinks[nodeId] = std::min(state.lowlinks[nodeId], state.lowlinks[neighbor]);
            } else if (state.onStack.find(neighbor) != state.onStack.end()) {
                // Neighbor is on stack and hence in current SCC
                state.lowlinks[nodeId] = std::min(state.lowlinks[nodeId], state.indices[neighbor]);
            }
        }
        
        // If nodeId is a root node, pop the stack and create an SCC
        if (state.lowlinks[nodeId] == state.indices[nodeId]) {
            std::vector<int> component;
            
            int currentNode;
            do {
                currentNode = state.nodeStack.top();
                state.nodeStack.pop();
                state.onStack.erase(currentNode);
                component.push_back(currentNode);
            } while (currentNode != nodeId);
            
            state.components.push_back(component);
        }
    }
    
    void kosarajuDFS1(int nodeId, std::unordered_set<int>& visited, std::vector<int>& finishOrder) {
        visited.insert(nodeId);
        
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            int neighbor = edge.getToNode();
            if (visited.find(neighbor) == visited.end()) {
                kosarajuDFS1(neighbor, visited, finishOrder);
            }
        }
        
        finishOrder.push_back(nodeId);
    }
    
    void kosarajuDFS2(int nodeId, std::unordered_set<int>& visited, std::vector<int>& component) {
        visited.insert(nodeId);
        component.push_back(nodeId);
        
        // For transpose graph, we need to follow incoming edges
        // Since we don't have explicit transpose representation, we'll find incoming edges
        for (int otherNodeId : graph->getAllNodeIds()) {
            if (visited.find(otherNodeId) == visited.end()) {
                const std::vector<Edge>& edges = graph->getEdgesFrom(otherNodeId);
                for (const Edge& edge : edges) {
                    if (edge.getToNode() == nodeId) {
                        kosarajuDFS2(otherNodeId, visited, component);
                    }
                }
            }
        }
    }
    
    bool hasSelfLoop(int nodeId) {
        const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
        for (const Edge& edge : edges) {
            if (edge.getToNode() == nodeId) {
                return true;
            }
        }
        return false;
    }
    
    void calculateSizeStatistics(SCCResult& result) {
        if (result.components.empty()) {
            result.largestComponentSize = 0;
            result.smallestComponentSize = 0;
            result.averageComponentSize = 0.0;
            return;
        }
        
        result.largestComponentSize = result.components[0].nodeCount;
        result.smallestComponentSize = result.components[0].nodeCount;
        size_t totalNodes = 0;
        
        for (const SCCComponent& component : result.components) {
            result.largestComponentSize = std::max(result.largestComponentSize, component.nodeCount);
            result.smallestComponentSize = std::min(result.smallestComponentSize, component.nodeCount);
            totalNodes += component.nodeCount;
        }
        
        result.averageComponentSize = static_cast<double>(totalNodes) / result.components.size();
    }
    
    void analyzeComponents(SCCResult& result) {
        std::cout << "[SCC_CALCULATOR] Performing detailed component analysis" << std::endl;
        
        for (SCCComponent& component : result.components) {
            analyzeIndividualComponent(component);
        }
        
        // Analyze connections between components
        analyzeComponentConnections(result);
    }
    
    void analyzeIndividualComponent(SCCComponent& component) {
        // Count internal edges
        component.internalEdges = 0;
        for (int nodeId : component.nodes) {
            const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
            for (const Edge& edge : edges) {
                if (std::find(component.nodes.begin(), component.nodes.end(), edge.getToNode()) != component.nodes.end()) {
                    component.internalEdges++;
                }
            }
        }
        
        // Calculate component density
        if (component.nodeCount > 1) {
            size_t maxPossibleEdges = component.nodeCount * (component.nodeCount - 1);
            component.componentDensity = static_cast<double>(component.internalEdges) / maxPossibleEdges;
        } else {
            component.componentDensity = component.internalEdges > 0 ? 1.0 : 0.0;
        }
        
        // Count outgoing edges (will be filled when analyzing connections)
        component.outgoingEdges = 0;
        component.incomingEdges = 0;
    }
    
    void analyzeComponentConnections(SCCResult& result) {
        // Create node to component mapping for quick lookup
        std::unordered_map<int, int> nodeToComponent;
        for (size_t i = 0; i < result.components.size(); ++i) {
            for (int nodeId : result.components[i].nodes) {
                nodeToComponent[nodeId] = static_cast<int>(i);
            }
        }
        
        // Count edges between components
        for (size_t i = 0; i < result.components.size(); ++i) {
            SCCComponent& component = result.components[i];
            std::unordered_set<int> connectedComponents;
            
            for (int nodeId : component.nodes) {
                const std::vector<Edge>& edges = graph->getEdgesFrom(nodeId);
                for (const Edge& edge : edges) {
                    auto targetComponentIt = nodeToComponent.find(edge.getToNode());
                    if (targetComponentIt != nodeToComponent.end()) {
                        int targetComponent = targetComponentIt->second;
                        
                        if (targetComponent != static_cast<int>(i)) {
                            component.outgoingEdges++;
                            result.components[targetComponent].incomingEdges++;
                            connectedComponents.insert(targetComponent);
                        }
                    }
                }
            }
            
            component.connectedComponents = std::vector<int>(connectedComponents.begin(), connectedComponents.end());
        }
    }
    
    void updateCalculationMetrics(const SCCResult& result) {
        metrics.totalCalculations++;
        metrics.lastCalculation = result.calculationTimestamp;
        
        if (result.isStronglyConnected) {
            metrics.stronglyConnectedGraphs++;
        } else {
            metrics.disconnectedGraphs++;
        }
        
        // Update averages
        if (metrics.averageCalculationTime == 0.0) {
            metrics.averageCalculationTime = result.calculationTime;
        } else {
            metrics.averageCalculationTime = (metrics.averageCalculationTime + result.calculationTime) / 2.0;
        }
        
        if (metrics.averageComponentCount == 0.0) {
            metrics.averageComponentCount = static_cast<double>(result.componentCount);
        } else {
            metrics.averageComponentCount = (metrics.averageComponentCount + result.componentCount) / 2.0;
        }
        
        if (metrics.averageComponentSize == 0.0) {
            metrics.averageComponentSize = result.averageComponentSize;
        } else {
            metrics.averageComponentSize = (metrics.averageComponentSize + result.averageComponentSize) / 2.0;
        }
        
        if (metrics.averageNodesProcessed == 0.0) {
            metrics.averageNodesProcessed = static_cast<double>(result.nodesProcessed);
        } else {
            metrics.averageNodesProcessed = (metrics.averageNodesProcessed + result.nodesProcessed) / 2.0;
        }
        
        if (metrics.averageEdgesProcessed == 0.0) {
            metrics.averageEdgesProcessed = static_cast<double>(result.edgesProcessed);
        } else {
            metrics.averageEdgesProcessed = (metrics.averageEdgesProcessed + result.edgesProcessed) / 2.0;
        }
    }
};