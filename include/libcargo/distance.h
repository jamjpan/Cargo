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
#ifndef CARGO_INCLUDE_LIBCARGO_DISTANCE_H_
#define CARGO_INCLUDE_LIBCARGO_DISTANCE_H_

#include "types.h"

#include <cmath>

namespace cargo {
namespace distance {

inline DistanceDouble euclidean(const Point& u, const Point& v)
{
    return std::hypot((u.lng - v.lng), (u.lat - v.lat));
}

inline DistanceDouble haversine(const Point& u, const Point& v)
{
    double r = 6372800.0; // radius of Earth (m)
    double x = (u.lng - v.lng) * (MathPI / 180);
    double y = (u.lat - v.lat) * (MathPI / 180);
    double a = std::sin(y / 2) * std::sin(y / 2) +
               std::sin(x / 2) * std::sin(x / 2) *
                   std::cos(u.lat * (MathPI / 180)) *
                   std::cos(v.lat * (MathPI / 180));
    return r * (2 * std::asin(std::sqrt(a)));
}

} // namespace distance
} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_DISTANCE_H_
