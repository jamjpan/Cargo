// MIT License
//
// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#ifndef CARGO_INCLUDE_LIBCARGO_GRID_H_
#define CARGO_INCLUDE_LIBCARGO_GRID_H_
#include <memory>
#include <vector>

#include "classes.h"
#include "types.h"

namespace cargo {

// Based on "Optimization of Large-Scale, Real-Time Simulations by Spatial Hashing"
// by Erin J. Hastings, Jaruwan Mesit, Ratan K. Guha, SCSC 2005
//
// Buckets are numbered starting from lower-left to upper-right. In each row,
// the buckets are numbered from left to right. Vehicles in the grid are mutable
// to allow for refresh (if immutable, we would have to delete the old vehicle
// and replace it with a new one).
class Grid {
public:
    Grid(int); // int = number of cells; total grid size = int^2

    void insert(const Vehicle &);
    void insert(const MutableVehicle &);
    std::vector<std::shared_ptr<MutableVehicle>>& within_about(const DistanceDouble &, const NodeId &);
    void commit(std::shared_ptr<MutableVehicle> &, const std::vector<Waypoint> &, const std::vector<Stop> &,
            const DistanceInt&);
    void clear();

private:
    double x_dim_;
    double y_dim_;
    int n_;
    std::vector<std::vector<std::shared_ptr<MutableVehicle>>> data_;
    std::vector<std::shared_ptr<MutableVehicle>> res_; // store results of within_about here

    int hash(const Point &);
    int hash_x(const Point &);
    int hash_y(const Point &);
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_GRID_H_

