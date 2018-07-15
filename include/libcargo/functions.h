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
#include <memory> /* shared_ptr */

#include "classes.h"
#include "types.h"

#include "../gtree/gtree.h"

namespace cargo {

DistInt pickup_range(const Customer &, const SimlTime &, GTree::G_Tree &);
DistInt pickup_range(const Customer &, const SimlTime &);

// Given a schedule, return the route through the schedule and its cost. Cost is
// integer because G_Tree only returns int.
// O(|schedule|*|nodes|)
DistInt route_through(const std::vector<Stop> &, std::vector<Wayp> &, GTree::G_Tree &);
DistInt route_through(const std::vector<Stop> &, std::vector<Wayp> &);
DistInt route_through(const Schedule &, std::vector<Wayp> &, GTree::G_Tree &);
DistInt route_through(const Schedule &, std::vector<Wayp> &);

// Given a schedule, find if precedence is satisfied.
// O(|schedule|^2)
bool chkpc(const Schedule &);

// Given a schedule, find if time windows are satisfied
// O(|schedule|+|route|)
bool chktw(const std::vector<Stop> &, const std::vector<Wayp> &);
bool chktw(const Schedule &, const Route &);

// Given a schedule and a customer, return the cost of the best-insertion
// schedule, and output the schedule and the route. The two bools are for
// fixing the end points. Set the first bool to true to fix the start, and
// set the second bool to true to fix the end.
// O(|schedule|^2*c_route_through)
DistInt sop_insert(
        const std::vector<Stop> &,
        const Stop &,
        const Stop &,
        bool,
        bool,
        std::vector<Stop> &,
        std::vector<Wayp> &,
        GTree::G_Tree &);

DistInt sop_insert( // use default GTree
        const std::vector<Stop> &,
        const Stop &,
        const Stop &,
        bool,
        bool,
        std::vector<Stop> &,
        std::vector<Wayp> &);

DistInt sop_insert( // use Vehicle/Customer
        const Vehicle &,
        const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &,
        GTree::G_Tree &);

DistInt sop_insert( // use Vehicle/Customer default GTree
        const Vehicle &,
        const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &);

DistInt sop_insert( // use MutableVehicle/Customer default GTree
        const std::shared_ptr<MutableVehicle> &,
        const Customer &,
        std::vector<Stop> &,
        std::vector<Wayp> &);

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_FUNCTIONS_H_

