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
#ifndef CARGO_INCLUDE_LIBCARGO_FILE_H_
#define CARGO_INCLUDE_LIBCARGO_FILE_H_
#include <unordered_map>
#include <map>

#include "classes.h"
#include "types.h"

namespace cargo {

// These functions throw runtime_errors if the file cannot be read.

// Returns number of nodes
size_t read_nodes(const Filepath&, KeyValueNodes&);

// Returns number of nodes, and output min/max lng/lat
size_t read_nodes(const Filepath&, KeyValueNodes&, BoundingBox&);

// Returns number of edges
size_t read_edges(const Filepath&, KeyValueEdges&);

// Returns number of trips
size_t read_problem(const Filepath&, ProblemSet&);

void write_solution(
    const std::string&, // problem name
    const std::string&, // road network name
    const int,          // number of vehicles
    const int,          // number of customers
    const int,          // base cost
    const DistanceInt,  // final_cost
    const Filepath&,
    const std::unordered_map<SimTime, std::map<VehicleId, NodeId>>&,
    const std::unordered_map<SimTime, std::map<CustomerId, CustomerStatus>>&,
    const std::unordered_map<SimTime, std::map<CustomerId, VehicleId>>&);

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_FILE_H_

