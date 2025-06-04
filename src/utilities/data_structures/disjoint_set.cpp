#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

class DisjointSet {
private:
    std::vector<int> parent;
    std::vector<int> rank;
    std::vector<int> size;
    int numSets;
    int maxSetSize;
    
public:
    explicit DisjointSet(int n) : parent(n), rank(n, 0), size(n, 1), numSets(n), maxSetSize(1) {
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
        std::cout << "[DISJOINT_SET] Initialized with " << n << " elements" << std::endl;
    }
    
    // Find with path compression
    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]); // Path compression
        }
        return parent[x];
    }
    
    // Union by rank with size tracking
    bool unionSets(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        
        if (rootX == rootY) {
            return false; // Already in same set
        }
        
        // Union by rank
        if (rank[rootX] < rank[rootY]) {
            std::swap(rootX, rootY);
        }
        
        parent[rootY] = rootX;
        size[rootX] += size[rootY];
        maxSetSize = std::max(maxSetSize, size[rootX]);
        
        if (rank[rootX] == rank[rootY]) {
            rank[rootX]++;
        }
        
        numSets--;
        return true;
    }
    
    bool connected(int x, int y) {
        return find(x) == find(y);
    }
    
    int getSetSize(int x) {
        return size[find(x)];
    }
    
    int getNumSets() const {
        return numSets;
    }
    
    int getMaxSetSize() const {
        return maxSetSize;
    }
    
    std::vector<std::vector<int>> getAllSets() {
        std::unordered_map<int, std::vector<int>> setMap;
        
        for (int i = 0; i < static_cast<int>(parent.size()); ++i) {
            setMap[find(i)].push_back(i);
        }
        
        std::vector<std::vector<int>> result;
        for (const auto& [root, elements] : setMap) {
            result.push_back(elements);
        }
        
        return result;
    }
    
    void printSets() {
        auto sets = getAllSets();
        std::cout << "[DISJOINT_SET] Current sets (" << sets.size() << " total):" << std::endl;
        
        for (size_t i = 0; i < sets.size(); ++i) {
            std::cout << "  Set " << i << ": {";
            for (size_t j = 0; j < sets[i].size(); ++j) {
                std::cout << sets[i][j];
                if (j < sets[i].size() - 1) std::cout << ", ";
            }
            std::cout << "}" << std::endl;
        }
    }
    
    void reset() {
        for (int i = 0; i < static_cast<int>(parent.size()); ++i) {
            parent[i] = i;
            rank[i] = 0;
            size[i] = 1;
        }
        numSets = parent.size();
        maxSetSize = 1;
        std::cout << "[DISJOINT_SET] Reset to initial state" << std::endl;
    }
};

// Specialized disjoint set for graph connectivity
class GraphConnectivityTracker {
private:
    DisjointSet ds;
    std::vector<std::pair<int, int>> edges;
    
public:
    explicit GraphConnectivityTracker(int numNodes) : ds(numNodes) {
        std::cout << "[GRAPH_CONNECTIVITY] Initialized for " << numNodes << " nodes" << std::endl;
    }
    
    void addEdge(int u, int v) {
        if (ds.unionSets(u, v)) {
            edges.emplace_back(u, v);
            std::cout << "[GRAPH_CONNECTIVITY] Added edge " << u << "-" << v 
                      << " (components: " << ds.getNumSets() << ")" << std::endl;
        }
    }
    
    bool isConnected() const {
        return ds.getNumSets() == 1;
    }
    
    int getComponentCount() const {
        return ds.getNumSets();
    }
    
    int getLargestComponentSize() const {
        return ds.getMaxSetSize();
    }
    
    bool areConnected(int u, int v) {
        return ds.connected(u, v);
    }
    
    std::vector<std::vector<int>> getComponents() {
        return ds.getAllSets();
    }
    
    void printConnectivityStatus() {
        std::cout << "[GRAPH_CONNECTIVITY] Status:" << std::endl;
        std::cout << "  Connected: " << (isConnected() ? "Yes" : "No") << std::endl;
        std::cout << "  Components: " << getComponentCount() << std::endl;
        std::cout << "  Largest component: " << getLargestComponentSize() << " nodes" << std::endl;
        std::cout << "  Total edges: " << edges.size() << std::endl;
    }
};

// Kruskal's MST using disjoint set
struct Edge {
    int u, v;
    double weight;
    
    Edge(int from, int to, double w) : u(from), v(to), weight(w) {}
    
    bool operator<(const Edge& other) const {
        return weight < other.weight;
    }
};

class KruskalMST {
private:
    std::vector<Edge> edges;
    int numNodes;
    
public:
    explicit KruskalMST(int n) : numNodes(n) {
        std::cout << "[KRUSKAL_MST] Initialized for " << n << " nodes" << std::endl;
    }
    
    void addEdge(int u, int v, double weight) {
        edges.emplace_back(u, v, weight);
    }
    
    std::pair<std::vector<Edge>, double> findMST() {
        std::sort(edges.begin(), edges.end());
        
        DisjointSet ds(numNodes);
        std::vector<Edge> mstEdges;
        double totalWeight = 0.0;
        
        std::cout << "[KRUSKAL_MST] Finding MST from " << edges.size() << " edges" << std::endl;
        
        for (const Edge& edge : edges) {
            if (ds.unionSets(edge.u, edge.v)) {
                mstEdges.push_back(edge);
                totalWeight += edge.weight;
                
                if (mstEdges.size() == static_cast<size_t>(numNodes - 1)) {
                    break; // MST complete
                }
            }
        }
        
        std::cout << "[KRUSKAL_MST] MST found with " << mstEdges.size() 
                  << " edges, total weight: " << totalWeight << std::endl;
        
        return {mstEdges, totalWeight};
    }
    
    void printMST() {
        auto [mstEdges, totalWeight] = findMST();
        
        std::cout << "[KRUSKAL_MST] Minimum Spanning Tree:" << std::endl;
        for (const Edge& edge : mstEdges) {
            std::cout << "  " << edge.u << " -- " << edge.v 
                      << " (weight: " << edge.weight << ")" << std::endl;
        }
        std::cout << "  Total weight: " << totalWeight << std::endl;
    }
};

// Global functions for easy access
DisjointSet* createDisjointSet(int size) {
    return new DisjointSet(size);
}

bool areElementsConnected(DisjointSet* ds, int x, int y) {
    return ds ? ds->connected(x, y) : false;
}

bool unionElements(DisjointSet* ds, int x, int y) {
    return ds ? ds->unionSets(x, y) : false;
}

void printDisjointSetStatus(DisjointSet* ds) {
    if (ds) {
        ds->printSets();
    }
}