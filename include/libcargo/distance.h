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
#ifndef CARGO_INCLUDE_LIBCARGO_DISTANCE_H_
#define CARGO_INCLUDE_LIBCARGO_DISTANCE_H_

#include "cargo.h" /* static gtree() */
#include "types.h"

#include <cmath>

namespace cargo {

inline DistDbl euclidean(const Point& u, const Point& v) {
  return std::hypot((u.lng - v.lng), (u.lat - v.lat));
}

inline DistDbl haversine(const Point& u, const Point& v) {
  double r = 6372800.0;  // radius of Earth (m)
  double x = (u.lng - v.lng) * (MathPI / 180);
  double y = (u.lat - v.lat) * (MathPI / 180);
  double a = std::sin(y / 2) * std::sin(y / 2)
      + std::sin(x / 2) * std::sin(x / 2)*
        std::cos(u.lat * (MathPI / 180)) *
        std::cos(v.lat * (MathPI / 180));
  return r * (2 * std::asin(std::sqrt(a)));  // meters
}

inline DistDbl haversine(NodeId u, NodeId v) {
  return haversine(Cargo::node2pt(u), Cargo::node2pt(v));
}

inline DistInt shortest_path_dist( // use specific gtree
        const NodeId& u,
        const NodeId& v,
        GTree::G_Tree& gtree) {
  return gtree.search(u, v);  // meters
}

inline DistInt shortest_path_dist( // use global gtree
        const NodeId& u,
        const NodeId& v) {
    return shortest_path_dist(u, v, Cargo::gtree());
}

// Convert meters to number of longitude degrees
// Really messes up at the poles
// https://stackoverflow.com/a/1253545
inline double metersTolngdegs(const DistDbl& meters, const Lat& lat) {
  return meters / (111320 * std::cos(lat * MathPI / 180));
}

// Convert meters to number of latitude degrees
// https://stackoverflow.com/a/1253545
inline double metersTolatdegs(const DistDbl& meters) {
  return meters * (1.0 / 110574);
}

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_DISTANCE_H_

