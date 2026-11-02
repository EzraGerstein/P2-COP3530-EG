//
// Created by Ethan on 10/25/2026.
//

#ifndef PROJECT2_MIN_HEAP_H
#define PROJECT2_MIN_HEAP_H

#include <vector>
using namespace std;



    // simple binary min heap setup
    template <typename T>

    class MinHeap {
    private:
    
        vector<T> storage;

        
        void sift_up(int index);



        
        void sift_down(int index);

        public:

        MinHeap();

        void push(const T &value);


        const T& top() const;
        void pop();
        bool empty() const;
        size_t size() const;

    };




#endif //PROJECT2_MIN_HEAP_H
