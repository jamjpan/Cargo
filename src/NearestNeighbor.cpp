#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include "common.h"
#include "gtree/GTree.h"

// Input: set of customers R
// Output: set of assignments A

typedef std::vector<int> RequestBatch;
typedef std::vector<int> Assignments;

class NearestNeighbor : Solution {

}


Assignments NearestNeighbor(RequestBatch R) {
    Assignments A;
    const int increment = 10;
    for (int j = 0; j < R.size(); ++j) {
        int base = 0;
        int k = 0;
        bool matched = false;
        Request r = R.at(j);
        while (!matched && base <= n) {
            k = base + increment;
            std::vector<int> candidates = gtree.KNN(r.origin, k, vehicles);
            int i = base;
            while (!matched && i < k) {
                Schedule T;
                Schedule S = candidates[i].schedule;
                double c = schedule_insert_add(S, r, T);
                if (c != -1) {
                    A[j] = i;
                    matched = true;
                } else
                    ++i;
            }
            if (!matched)
                base += increment;
        }
    }
    return A;
}


int main() {
    std::cout << "hello" << std::endl;
    return 0;
}

