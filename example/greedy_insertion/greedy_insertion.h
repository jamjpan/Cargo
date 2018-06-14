// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include "libcargo.h"

// Implements the "cheap insertion" scheduling heuristic described in Jaw 1986.
// For each request, the algorithm looks for the "greedy" vehicle based on the
// heuristic, and assigns the request to this vehicle if it exists.
class GreedyInsertion : public cargo::RSAlgorithm { // <-- inherit from the base
public:
    GreedyInsertion();

    /* My Overrides */
    virtual void handle_customer(const cargo::Customer &);
    virtual void handle_vehicle(const cargo::Vehicle &);
    virtual void end();
    virtual void listen();

private:
    /* My Custom Variables */
    int nmatches;
    cargo::Grid grid_;
};

