#pragma once
#include <vector>
#include <functional>

template<typename T, typename Compare = std::greater<T>>
class PriorityQueue {
private:
    std::vector<T> heap;
    Compare comparator;
    
    void heapifyUp(size_t index);
    void heapifyDown(size_t index);
    size_t getParentIndex(size_t index) const;
    size_t getLeftChildIndex(size_t index) const;
    size_t getRightChildIndex(size_t index) const;

public:
    PriorityQueue();
    explicit PriorityQueue(const Compare& comp);
    
    void push(const T& item);
    void push(T&& item);
    T top() const;
    void pop();
    
    bool empty() const;
    size_t size() const;
    void clear();
    
    void reserve(size_t capacity);
    std::vector<T> extractAll();
    
    template<typename Iterator>
    void pushRange(Iterator begin, Iterator end);
    
    bool contains(const T& item) const;
    void updatePriority(const T& oldItem, const T& newItem);
};

template<typename T, typename Compare>
PriorityQueue<T, Compare>::PriorityQueue() = default;

template<typename T, typename Compare>
PriorityQueue<T, Compare>::PriorityQueue(const Compare& comp) : comparator(comp) {}

template<typename T, typename Compare>
void PriorityQueue<T, Compare>::push(const T& item) {
    heap.push_back(item);
    heapifyUp(heap.size() - 1);
}

template<typename T, typename Compare>
void PriorityQueue<T, Compare>::push(T&& item) {
    heap.push_back(std::move(item));
    heapifyUp(heap.size() - 1);
}

template<typename T, typename Compare>
T PriorityQueue<T, Compare>::top() const {
    if (heap.empty()) {
        throw std::runtime_error("Priority queue is empty");
    }
    return heap[0];
}

template<typename T, typename Compare>
void PriorityQueue<T, Compare>::pop() {
    if (heap.empty()) {
        throw std::runtime_error("Priority queue is empty");
    }
    
    heap[0] = heap.back();
    heap.pop_back();
    
    if (!heap.empty()) {
        heapifyDown(0);
    }
}

template<typename T, typename Compare>
bool PriorityQueue<T, Compare>::empty() const {
    return heap.empty();
}

template<typename T, typename Compare>
size_t PriorityQueue<T, Compare>::size() const {
    return heap.size();
}

template<typename T, typename Compare>
void PriorityQueue<T, Compare>::heapifyUp(size_t index) {
    while (index > 0) {
        size_t parentIndex = getParentIndex(index);
        if (!comparator(heap[index], heap[parentIndex])) {
            break;
        }
        std::swap(heap[index], heap[parentIndex]);
        index = parentIndex;
    }
}

template<typename T, typename Compare>
void PriorityQueue<T, Compare>::heapifyDown(size_t index) {
    while (true) {
        size_t leftChild = getLeftChildIndex(index);
        size_t rightChild = getRightChildIndex(index);
        size_t smallest = index;
        
        if (leftChild < heap.size() && comparator(heap[leftChild], heap[smallest])) {
            smallest = leftChild;
        }
        
        if (rightChild < heap.size() && comparator(heap[rightChild], heap[smallest])) {
            smallest = rightChild;
        }
        
        if (smallest == index) {
            break;
        }
        
        std::swap(heap[index], heap[smallest]);
        index = smallest;
    }
}