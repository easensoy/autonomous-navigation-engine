#include "utilities/PriorityQueue.hpp"
#include <iostream>
#include <vector>
#include <functional>
#include <stdexcept>

template<typename T, typename Compare = std::greater<T>>
class BinaryHeap {
private:
    std::vector<T> heap;
    Compare comparator;
    
    void heapifyUp(size_t index) {
        while (index > 0) {
            size_t parentIndex = getParentIndex(index);
            if (!comparator(heap[index], heap[parentIndex])) {
                break;
            }
            std::swap(heap[index], heap[parentIndex]);
            index = parentIndex;
        }
    }
    
    void heapifyDown(size_t index) {
        while (true) {
            size_t leftChild = getLeftChildIndex(index);
            size_t rightChild = getRightChildIndex(index);
            size_t target = index;
            
            if (leftChild < heap.size() && comparator(heap[leftChild], heap[target])) {
                target = leftChild;
            }
            
            if (rightChild < heap.size() && comparator(heap[rightChild], heap[target])) {
                target = rightChild;
            }
            
            if (target == index) {
                break;
            }
            
            std::swap(heap[index], heap[target]);
            index = target;
        }
    }
    
    size_t getParentIndex(size_t index) const {
        return (index - 1) / 2;
    }
    
    size_t getLeftChildIndex(size_t index) const {
        return 2 * index + 1;
    }
    
    size_t getRightChildIndex(size_t index) const {
        return 2 * index + 2;
    }

public:
    BinaryHeap() = default;
    explicit BinaryHeap(const Compare& comp) : comparator(comp) {}
    
    BinaryHeap(std::initializer_list<T> init) : heap(init) {
        buildHeap();
    }
    
    BinaryHeap(const std::vector<T>& data) : heap(data) {
        buildHeap();
    }
    
    void push(const T& item) {
        heap.push_back(item);
        heapifyUp(heap.size() - 1);
    }
    
    void push(T&& item) {
        heap.push_back(std::move(item));
        heapifyUp(heap.size() - 1);
    }
    
    template<typename... Args>
    void emplace(Args&&... args) {
        heap.emplace_back(std::forward<Args>(args)...);
        heapifyUp(heap.size() - 1);
    }
    
    T top() const {
        if (heap.empty()) {
            throw std::runtime_error("Heap is empty");
        }
        return heap[0];
    }
    
    void pop() {
        if (heap.empty()) {
            throw std::runtime_error("Heap is empty");
        }
        
        heap[0] = std::move(heap.back());
        heap.pop_back();
        
        if (!heap.empty()) {
            heapifyDown(0);
        }
    }
    
    T extractTop() {
        T result = top();
        pop();
        return result;
    }
    
    bool empty() const {
        return heap.empty();
    }
    
    size_t size() const {
        return heap.size();
    }
    
    void clear() {
        heap.clear();
    }
    
    void reserve(size_t capacity) {
        heap.reserve(capacity);
    }
    
    bool contains(const T& item) const {
        return std::find(heap.begin(), heap.end(), item) != heap.end();
    }
    
    void merge(const BinaryHeap& other) {
        for (const T& item : other.heap) {
            push(item);
        }
    }
    
    void merge(BinaryHeap&& other) {
        for (T& item : other.heap) {
            push(std::move(item));
        }
        other.clear();
    }
    
    std::vector<T> extractAll() {
        std::vector<T> result;
        result.reserve(heap.size());
        
        while (!empty()) {
            result.push_back(extractTop());
        }
        
        return result;
    }
    
    void updatePriority(const T& oldItem, const T& newItem) {
        auto it = std::find(heap.begin(), heap.end(), oldItem);
        if (it == heap.end()) {
            throw std::runtime_error("Item not found in heap");
        }
        
        size_t index = std::distance(heap.begin(), it);
        T original = heap[index];
        heap[index] = newItem;
        
        // Determine direction to heapify
        if (comparator(newItem, original)) {
            heapifyUp(index);
        } else {
            heapifyDown(index);
        }
    }
    
    bool isValidHeap() const {
        for (size_t i = 1; i < heap.size(); ++i) {
            size_t parentIndex = getParentIndex(i);
            if (comparator(heap[i], heap[parentIndex])) {
                return false;
            }
        }
        return true;
    }
    
    void printHeap() const {
        std::cout << "[BINARY_HEAP] Heap contents: ";
        for (size_t i = 0; i < heap.size(); ++i) {
            std::cout << heap[i];
            if (i < heap.size() - 1) std::cout << " ";
        }
        std::cout << std::endl;
    }
    
    // Iterator support
    typename std::vector<T>::const_iterator begin() const { return heap.begin(); }
    typename std::vector<T>::const_iterator end() const { return heap.end(); }
    
private:
    void buildHeap() {
        if (heap.size() <= 1) return;
        
        // Start from last non-leaf node
        for (int i = static_cast<int>(heap.size() / 2) - 1; i >= 0; --i) {
            heapifyDown(static_cast<size_t>(i));
        }
    }
};

// Specialized heaps for common use cases
template<typename T>
using MaxHeap = BinaryHeap<T, std::greater<T>>;

template<typename T>
using MinHeap = BinaryHeap<T, std::less<T>>;

// Node priority heap for pathfinding algorithms
struct NodePriority {
    int nodeId;
    double priority;
    
    NodePriority(int id, double p) : nodeId(id), priority(p) {}
    
    bool operator>(const NodePriority& other) const {
        return priority < other.priority; // Min-heap by priority
    }
    
    bool operator<(const NodePriority& other) const {
        return priority > other.priority;
    }
    
    bool operator==(const NodePriority& other) const {
        return nodeId == other.nodeId;
    }
    
    friend std::ostream& operator<<(std::ostream& os, const NodePriority& np) {
        os << "Node(" << np.nodeId << ", " << np.priority << ")";
        return os;
    }
};

using NodePriorityHeap = BinaryHeap<NodePriority>;

// Distance pair for Dijkstra-like algorithms
struct DistancePair {
    int nodeId;
    double distance;
    int predecessor;
    
    DistancePair(int id, double dist, int pred = -1) 
        : nodeId(id), distance(dist), predecessor(pred) {}
    
    bool operator>(const DistancePair& other) const {
        return distance < other.distance; // Min-heap by distance
    }
    
    bool operator<(const DistancePair& other) const {
        return distance > other.distance;
    }
    
    bool operator==(const DistancePair& other) const {
        return nodeId == other.nodeId;
    }
};

using DistanceHeap = BinaryHeap<DistancePair>;

// Utility functions
template<typename T>
std::vector<T> heapSort(std::vector<T> data) {
    BinaryHeap<T, std::greater<T>> heap(std::move(data));
    return heap.extractAll();
}

template<typename T, typename Compare>
std::vector<T> heapSort(std::vector<T> data, Compare comp) {
    BinaryHeap<T, Compare> heap(comp);
    for (auto& item : data) {
        heap.push(std::move(item));
    }
    return heap.extractAll();
}

// Global heap operations for easy access
template<typename T>
T findKthLargest(std::vector<T>& data, size_t k) {
    if (k > data.size()) {
        throw std::out_of_range("K is larger than data size");
    }
    
    MinHeap<T> heap;
    
    for (const T& item : data) {
        if (heap.size() < k) {
            heap.push(item);
        } else if (item > heap.top()) {
            heap.pop();
            heap.push(item);
        }
    }
    
    return heap.top();
}

template<typename T>
T findKthSmallest(std::vector<T>& data, size_t k) {
    if (k > data.size()) {
        throw std::out_of_range("K is larger than data size");
    }
    
    MaxHeap<T> heap;
    
    for (const T& item : data) {
        if (heap.size() < k) {
            heap.push(item);
        } else if (item < heap.top()) {
            heap.pop();
            heap.push(item);
        }
    }
    
    return heap.top();
}

// Heap-based median finder
template<typename T>
class MedianFinder {
private:
    MaxHeap<T> leftHeap;  // Smaller half
    MinHeap<T> rightHeap; // Larger half
    
public:
    void addNumber(T num) {
        if (leftHeap.empty() || num <= leftHeap.top()) {
            leftHeap.push(num);
        } else {
            rightHeap.push(num);
        }
        
        // Balance heaps
        if (leftHeap.size() > rightHeap.size() + 1) {
            rightHeap.push(leftHeap.top());
            leftHeap.pop();
        } else if (rightHeap.size() > leftHeap.size() + 1) {
            leftHeap.push(rightHeap.top());
            rightHeap.pop();
        }
    }
    
    double findMedian() {
        if (leftHeap.size() == rightHeap.size()) {
            return (leftHeap.top() + rightHeap.top()) / 2.0;
        } else if (leftHeap.size() > rightHeap.size()) {
            return leftHeap.top();
        } else {
            return rightHeap.top();
        }
    }
    
    size_t size() const {
        return leftHeap.size() + rightHeap.size();
    }
};