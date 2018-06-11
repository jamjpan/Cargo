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
#ifndef CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_
#define CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_

#include "classes.h"
#include "types.h"

namespace cargo {

// Given a schedule, return the route through the schedule and its cost. Cost is
// integer because G_Tree only returns int.
// O(|schedule|*|nodes|)
DistanceInt route_through(const Schedule&, std::vector<Waypoint>&);
DistanceInt route_through(const std::vector<Stop>&, std::vector<Waypoint>&);

// Given a schedule, find if precedence is satisfied.
// O(|schedule|^2)
bool check_precedence_constr(const Schedule&);

// Given a schedule, find if time windows are satisfied
// O(|schedule|+|route|)
bool check_timewindow_constr(const Schedule&, const Route&);
bool check_timewindow_constr(const std::vector<Stop>&, const std::vector<Waypoint>&);

// Given a schedule and a customer, return the cost of the best-insertion
// schedule, and output the schedule and the route. The two bools are for
// fixing the end points. Set the first bool to true to fix the start, and
// set the second bool to true to fix the end.
// O(|schedule|^2*c_route_through)
DistanceInt sop_insert(const Vehicle&, const Customer&, std::vector<Stop>&, std::vector<Waypoint>&);
DistanceInt sop_insert(const Vehicle&, const Customer&, bool, bool, std::vector<Stop>&, std::vector<Waypoint>&);

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_

