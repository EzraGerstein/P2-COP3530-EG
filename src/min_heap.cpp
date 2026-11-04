//
// Created by Ethan on 10/25/2026.
//

#include "min_heap.h"
#include "dijkstra.h"
#include <algorithm>
using namespace std;

template <typename T>
MinHeap<T>::MinHeap() {
}

//add a new value to the heap
template <typename T>
void MinHeap<T>::push(const T &value) {

    storage.push_back(value);

    sift_up(static_cast<int>(storage.size()) - 1);
}

template <typename T>
const T& MinHeap<T>::top() const {
    
    return storage.front();
}

template <typename T>
void MinHeap<T>::pop() {

    storage.front() = storage.back();

    storage.pop_back();

    if (!storage.empty()) {

        sift_down(0);
    }
}

template <typename T>
bool MinHeap<T>::empty() const {

    return storage.empty();
}

template <typename T>
size_t MinHeap<T>::size() const {

    return storage.size();
}



//sift up and down
template <typename T>
void MinHeap<T>::sift_up(int index) {

    while (index > 0) {

        int parent = (index - 1) / 2;


        //compare the current node with its parent
        if (storage[index] < storage[parent]) {
            swap(storage[index], storage[parent]);
            index = parent;

        }

        else {

            break;

        }



    }

}



template <typename T>
void MinHeap<T>::sift_down(int index) {

    int count = static_cast<int>(storage.size());

    while (2 * index + 1 < count) {
        int child = 2 * index + 1;

        if (child + 1 < count && storage[child + 1] < storage[child]) {
            child++;
        }

        if (storage[child] < storage[index]) {
            swap(storage[child], storage[index]);
            index = child;


        }

        else {

            break;


        }
    }
}


template class MinHeap<Search_entry_values>;
