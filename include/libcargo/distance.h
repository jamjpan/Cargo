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

#include "cargo.h"
#include "types.h"

#include <cmath>
#include <mutex>

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

inline DistDbl haversine(const NodeId& u, const NodeId& v) {
  return haversine(Cargo::node2pt(u), Cargo::node2pt(v));
}

inline DistInt get_shortest_path(
    const NodeId        & u,
    const NodeId        & v,
          vec_t<Wayp>   & path,
          GTree::G_Tree & gtree,
    const int           & count = true)
{
  if (count) Cargo::count_sp() += 1;
  vec_t<NodeId> seg = {};
  path = {};

  // Do not change this part, used by Cargo::step()
  if (u == v) {
    Wayp wp = std::make_pair(0, u);
    path.push_back(wp);
    path.push_back(wp);
    return 0;
  }
  //------------------------

  if (!Cargo::spexist(u, v)) {
    // gtree seems to directly cause SIGSEGV if u or v is out of bounds so the
    // try-catch is useless
    try { gtree.find_path(u, v, seg); }
    catch (...) {
      std::cout << "gtree.find_path(" << u << "," << v << ") failed" << std::endl;
      throw std::runtime_error("find_path error");
    }
    // Acquire lock
    std::lock_guard<std::mutex> splock(Cargo::spmx);
    Cargo::spput(u, v, seg);
  }
  // Lock released (out-of-scope)
  else {
    // Acquire lock
    std::lock_guard<std::mutex> splock(Cargo::spmx);
    seg = Cargo::spget(u, v);
  }
  // Lock released (out-of-scope)

  DistInt cost = 0;
  Wayp wp = std::make_pair(cost, seg.at(0));
  // std::cout << "get_shortest_path push_back " << wp << std::endl;
  path.push_back(wp);
  for (size_t i = 1; i < seg.size(); ++i) {
    cost += Cargo::edgew(seg.at(i-1), seg.at(i));
    wp = std::make_pair(cost, seg.at(i));
    // std::cout << "get_shortest_path push_back " << wp << std::endl;
    path.push_back(wp);
  }
  return cost;
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v) {
  vec_t<Wayp> _ = {};
  return get_shortest_path(u, v, _, Cargo::gtree());
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v, vec_t<Wayp>& path) {
  return get_shortest_path(u, v, path, Cargo::gtree());
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v, GTree::G_Tree& gtree) {
  vec_t<Wayp> _ = {};
  return get_shortest_path(u, v, _, gtree);
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v, const bool& count) {
  vec_t<Wayp> _ = {};
  return get_shortest_path(u, v, _, Cargo::gtree(), count);
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v, vec_t<Wayp>& path, const bool& count) {
  return get_shortest_path(u, v, path, Cargo::gtree(), count);
}

inline DistInt get_shortest_path( const NodeId& u, const NodeId& v, GTree::G_Tree& gtree, const bool& count) {
  vec_t<Wayp> _ = {};
  return get_shortest_path(u, v, _, gtree, count);
}

// Convert meters to number of longitude degrees
// TODO Compensate near the poles
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

