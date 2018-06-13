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
#include <iostream> /* debug */
#include <vector>

#include "types.h"

namespace cargo {

// Based on "Optimization of Large-Scale, Real-Time Simulations by Spatial Hashing"
// by Erin J. Hastings, Jaruwan Mesit, Ratan K. Guha, SCSC 2005
//
// Buckets are numbered starting from lower-left to upper-right. In each row,
// the buckets are numbered from left to right.
template <typename T>
class Grid {
public:
    Grid(int n) // int = number of cells; total grid size = n*n
    {
        x_dim_ = (Cargo::bbox().upper_right.lng - Cargo::bbox().lower_left.lng)/n;
        y_dim_ = (Cargo::bbox().upper_right.lat - Cargo::bbox().lower_left.lat)/n;
        data_.resize(n*n, {});
        n_ = n;
    }

    void insert(T obj, NodeId node) // insert T located at NodeId
    {
        data_.at(hash(Cargo::node2pt(node))).push_back(obj);
    }

    std::vector<T> within_about(DistanceDouble d, NodeId node) // get objs within ~dist
    {
        std::vector<T> res;
        int offset_x = std::ceil(metersTolngdegs(d, Cargo::node2pt(node).lat)/x_dim_);
        int offset_y = std::ceil(metersTolatdegs(d)/y_dim_);
        int base_x = hash_x(Cargo::node2pt(node));
        int base_y = hash_y(Cargo::node2pt(node));
        // i,j must be positive, and less than n_
        for (int j = std::max(0, base_y - offset_y); j <= std::min(base_y + offset_y, n_-1); ++j)
            for (int i = std::max(0, base_x - offset_x); i <= std::min(base_x + offset_x, n_-1); ++i) {
                int k = i+j*n_;
                res.insert(res.end(), data_.at(k).begin(), data_.at(k).end());
            }
        return res;
    }

    void clear()
    {
        data_.clear();
        data_.resize(n_*n_, {});
    }

private:
    double x_dim_;
    double y_dim_;
    int n_;
    std::vector<std::vector<T>> data_;

    int hash(Point coord)
    {
        // x nor y can be greater than n_
        return std::min(hash_x(coord), n_-1) + std::min(hash_y(coord), n_-1)*n_;
    }

    int hash_x(Point coord)
    {
        return (int)std::floor((coord.lng-Cargo::bbox().lower_left.lng)/x_dim_);
    }

    int hash_y(Point coord)
    {
        return (int)std::floor((coord.lat-Cargo::bbox().lower_left.lat)/y_dim_);
    }
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_GRID_H_

