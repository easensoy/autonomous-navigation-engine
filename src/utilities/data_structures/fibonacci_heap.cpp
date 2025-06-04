#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <functional>

template<typename T, typename Compare = std::less<T>>
class FibonacciHeap {
private:
    struct Node {
        T key;
        int degree;
        bool marked;
        Node* parent;
        Node* child;
        Node* left;
        Node* right;
        
        explicit Node(const T& k) : key(k), degree(0), marked(false), 
                                   parent(nullptr), child(nullptr), left(this), right(this) {}
    };
    
    Node* minNode;
    int nodeCount;
    Compare comparator;
    
    void addToRootList(Node* node) {
        if (!minNode) {
            minNode = node;
            node->left = node->right = node;
        } else {
            node->left = minNode;
            node->right = minNode->right;
            minNode->right->left = node;
            minNode->right = node;
            
            if (comparator(node->key, minNode->key)) {
                minNode = node;
            }
        }
    }
    
    void removeFromRootList(Node* node) {
        if (node->right == node) {
            minNode = nullptr;
        } else {
            node->left->right = node->right;
            node->right->left = node->left;
            if (minNode == node) {
                minNode = node->right;
            }
        }
    }
    
    void addChild(Node* parent, Node* child) {
        if (!parent->child) {
            parent->child = child;
            child->left = child->right = child;
        } else {
            child->left = parent->child;
            child->right = parent->child->right;
            parent->child->right->left = child;
            parent->child->right = child;
        }
        child->parent = parent;
        parent->degree++;
        child->marked = false;
    }
    
    void removeChild(Node* parent, Node* child) {
        if (child->right == child) {
            parent->child = nullptr;
        } else {
            child->left->right = child->right;
            child->right->left = child->left;
            if (parent->child == child) {
                parent->child = child->right;
            }
        }
        parent->degree--;
        child->parent = nullptr;
    }
    
    void consolidate() {
        int maxDegree = static_cast<int>(std::log2(nodeCount)) + 2;
        std::vector<Node*> degreeTable(maxDegree, nullptr);
        
        std::vector<Node*> rootNodes;
        Node* current = minNode;
        if (current) {
            do {
                rootNodes.push_back(current);
                current = current->right;
            } while (current != minNode);
        }
        
        for (Node* node : rootNodes) {
            int degree = node->degree;
            while (degreeTable[degree]) {
                Node* other = degreeTable[degree];
                if (comparator(other->key, node->key)) {
                    std::swap(node, other);
                }
                
                removeFromRootList(other);
                addChild(node, other);
                degreeTable[degree] = nullptr;
                degree++;
            }
            degreeTable[degree] = node;
        }
        
        minNode = nullptr;
        for (Node* node : degreeTable) {
            if (node) {
                addToRootList(node);
            }
        }
    }
    
    void cascadingCut(Node* node) {
        Node* parent = node->parent;
        if (parent) {
            if (!node->marked) {
                node->marked = true;
            } else {
                cut(node, parent);
                cascadingCut(parent);
            }
        }
    }
    
    void cut(Node* child, Node* parent) {
        removeChild(parent, child);
        addToRootList(child);
        child->marked = false;
    }
    
    void destroyTree(Node* node) {
        if (!node) return;
        
        Node* current = node;
        do {
            Node* next = current->right;
            if (current->child) {
                destroyTree(current->child);
            }
            delete current;
            current = next;
        } while (current != node);
    }
    
    Node* findNodeWithKey(Node* root, const T& key) {
        if (!root) return nullptr;
        
        Node* current = root;
        do {
            if (current->key == key) {
                return current;
            }
            Node* childResult = findNodeWithKey(current->child, key);
            if (childResult) {
                return childResult;
            }
            current = current->right;
        } while (current != root);
        
        return nullptr;
    }
    
public:
    FibonacciHeap() : minNode(nullptr), nodeCount(0) {
        std::cout << "[FIBONACCI_HEAP] Initialized" << std::endl;
    }
    
    explicit FibonacciHeap(const Compare& comp) : minNode(nullptr), nodeCount(0), comparator(comp) {
        std::cout << "[FIBONACCI_HEAP] Initialized with custom comparator" << std::endl;
    }
    
    ~FibonacciHeap() {
        clear();
    }
    
    // Copy constructor
    FibonacciHeap(const FibonacciHeap& other) : minNode(nullptr), nodeCount(0), comparator(other.comparator) {
        if (!other.empty()) {
            // For simplicity, insert all elements from other heap
            std::vector<T> elements = other.getAllElements();
            for (const T& element : elements) {
                insert(element);
            }
        }
    }
    
    // Move constructor
    FibonacciHeap(FibonacciHeap&& other) noexcept 
        : minNode(other.minNode), nodeCount(other.nodeCount), comparator(std::move(other.comparator)) {
        other.minNode = nullptr;
        other.nodeCount = 0;
    }
    
    Node* insert(const T& key) {
        Node* newNode = new Node(key);
        addToRootList(newNode);
        nodeCount++;
        
        return newNode;
    }
    
    T minimum() const {
        if (!minNode) {
            throw std::runtime_error("Heap is empty");
        }
        return minNode->key;
    }
    
    bool empty() const {
        return minNode == nullptr;
    }
    
    size_t size() const {
        return nodeCount;
    }
    
    T extractMin() {
        if (!minNode) {
            throw std::runtime_error("Heap is empty");
        }
        
        Node* min = minNode;
        T minKey = min->key;
        
        // Add all children to root list
        if (min->child) {
            Node* child = min->child;
            do {
                Node* nextChild = child->right;
                child->parent = nullptr;
                addToRootList(child);
                child = nextChild;
            } while (child != min->child);
        }
        
        removeFromRootList(min);
        nodeCount--;
        
        if (nodeCount == 0) {
            minNode = nullptr;
        } else {
            consolidate();
        }
        
        delete min;
        return minKey;
    }
    
    void decreaseKey(Node* node, const T& newKey) {
        if (!comparator(newKey, node->key)) {
            throw std::invalid_argument("New key is not smaller than current key");
        }
        
        node->key = newKey;
        Node* parent = node->parent;
        
        if (parent && comparator(node->key, parent->key)) {
            cut(node, parent);
            cascadingCut(parent);
        }
        
        if (comparator(node->key, minNode->key)) {
            minNode = node;
        }
    }
    
    void deleteNode(Node* node) {
        decreaseKey(node, T{}); // Decrease to minimum possible value
        extractMin();
    }
    
    void merge(FibonacciHeap& other) {
        if (other.empty()) return;
        
        if (empty()) {
            minNode = other.minNode;
        } else {
            // Merge root lists
            Node* thisLast = minNode->left;
            Node* otherLast = other.minNode->left;
            
            thisLast->right = other.minNode;
            other.minNode->left = thisLast;
            otherLast->right = minNode;
            minNode->left = otherLast;
            
            if (comparator(other.minNode->key, minNode->key)) {
                minNode = other.minNode;
            }
        }
        
        nodeCount += other.nodeCount;
        other.minNode = nullptr;
        other.nodeCount = 0;
    }
    
    void clear() {
        if (minNode) {
            destroyTree(minNode);
            minNode = nullptr;
            nodeCount = 0;
        }
    }
    
    std::vector<T> getAllElements() const {
        std::vector<T> elements;
        if (!minNode) return elements;
        
        std::function<void(Node*)> traverse = [&](Node* root) {
            if (!root) return;
            
            Node* current = root;
            do {
                elements.push_back(current->key);
                if (current->child) {
                    traverse(current->child);
                }
                current = current->right;
            } while (current != root);
        };
        
        traverse(minNode);
        return elements;
    }
    
    void printHeap() const {
        std::cout << "[FIBONACCI_HEAP] Heap structure (size: " << nodeCount << "):" << std::endl;
        if (!minNode) {
            std::cout << "  Empty heap" << std::endl;
            return;
        }
        
        std::function<void(Node*, int)> printNode = [&](Node* root, int depth) {
            if (!root) return;
            
            Node* current = root;
            do {
                for (int i = 0; i < depth; ++i) std::cout << "  ";
                std::cout << "Node(" << current->key << ", deg:" << current->degree 
                          << ", marked:" << (current->marked ? "T" : "F") << ")" << std::endl;
                
                if (current->child) {
                    printNode(current->child, depth + 1);
                }
                current = current->right;
            } while (current != root);
        };
        
        std::cout << "  Root list:" << std::endl;
        printNode(minNode, 1);
        std::cout << "  Minimum: " << minNode->key << std::endl;
    }
    
    // For pathfinding algorithms - find node by key
    Node* findNode(const T& key) {
        return findNodeWithKey(minNode, key);
    }
    
    // Performance statistics
    void printStatistics() const {
        std::cout << "[FIBONACCI_HEAP] Statistics:" << std::endl;
        std::cout << "  Size: " << nodeCount << std::endl;
        std::cout << "  Root list size: " << getRootListSize() << std::endl;
        std::cout << "  Maximum degree: " << getMaxDegree() << std::endl;
        if (!empty()) {
            std::cout << "  Minimum key: " << minNode->key << std::endl;
        }
    }
    
private:
    int getRootListSize() const {
        if (!minNode) return 0;
        
        int count = 0;
        Node* current = minNode;
        do {
            count++;
            current = current->right;
        } while (current != minNode);
        
        return count;
    }
    
    int getMaxDegree() const {
        if (!minNode) return 0;
        
        int maxDegree = 0;
        std::function<void(Node*)> traverse = [&](Node* root) {
            if (!root) return;
            
            Node* current = root;
            do {
                maxDegree = std::max(maxDegree, current->degree);
                if (current->child) {
                    traverse(current->child);
                }
                current = current->right;
            } while (current != root);
        };
        
        traverse(minNode);
        return maxDegree;
    }
};

// Specialized Fibonacci heap for pathfinding
struct PathfindingNode {
    int nodeId;
    double priority;
    
    PathfindingNode(int id, double p) : nodeId(id), priority(p) {}
    
    bool operator<(const PathfindingNode& other) const {
        return priority < other.priority;
    }
    
    bool operator==(const PathfindingNode& other) const {
        return nodeId == other.nodeId;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const PathfindingNode& pn) {
        os << "Node(" << pn.nodeId << ", " << pn.priority << ")";
        return os;
    }
};

using PathfindingFibHeap = FibonacciHeap<PathfindingNode>;

// Global utility functions
template<typename T>
FibonacciHeap<T>* createFibonacciHeap() {
    return new FibonacciHeap<T>();
}

template<typename T>
void mergeFibonacciHeaps(FibonacciHeap<T>& heap1, FibonacciHeap<T>& heap2) {
    heap1.merge(heap2);
}

// Benchmark comparison with standard priority queue
template<typename T>
void benchmarkFibonacciHeap(int numOperations = 10000) {
    std::cout << "[FIBONACCI_HEAP] Running benchmark with " << numOperations << " operations" << std::endl;
    
    FibonacciHeap<T> fibHeap;
    auto start = std::chrono::high_resolution_clock::now();
    
    // Insert operations
    std::vector<typename FibonacciHeap<T>::Node*> nodes;
    for (int i = 0; i < numOperations; ++i) {
        nodes.push_back(fibHeap.insert(static_cast<T>(rand() % 1000)));
    }
    
    // Extract min operations
    for (int i = 0; i < numOperations / 4; ++i) {
        if (!fibHeap.empty()) {
            fibHeap.extractMin();
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "[FIBONACCI_HEAP] Benchmark completed in " << duration.count() 
              << " microseconds" << std::endl;
    std::cout << "[FIBONACCI_HEAP] Final heap size: " << fibHeap.size() << std::endl;
}