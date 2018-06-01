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
#ifndef CARGO_INCLUDE_LIBCARGO_CARGO_H_
#define CARGO_INCLUDE_LIBCARGO_CARGO_H_

#include "classes.h"
#include "file.h"
#include "message.h"
#include "options.h"
#include "types.h"

#include "../gtree/gtree.h"

// Cargo is a simulator and requests generator. Its two functionalities are to
// simulate the movement of vehicles, and to generate customer requests and
// vehicles in "real time". These are generated from a "problem instance", a
// special text file that lists customers, vehicles, and their properties.
// Vehicle movement is simulated based on vehicle speed, a road network,
// vehicle routes, and time. Speed determines how much distance is covered at
// every simulation timestep. Routes are sequences of nodes through the road
// network. By default, all vehicles travel along the shortest path from their
// origin to their destination.
//
// Cargo will poll an internal sqlite3 database for new vehicle routes. These
// routes can be updated through an RSAlgorithm (or manually!).
namespace cargo {

class Cargo {
public:
    Cargo(const Options &);
    const BoundingBox&          bbox()                  const;
    const std::string&          name();
    const std::string&          road_network();
    void                        run();

private:
    Message print_out;
    Message print_info;
    Message print_warning;
    Message print_error;
    Message print_success;

    GTree::G_Tree gtree_;

    KeyValueNodes nodes_;
    KeyValueEdges edges_; // usage: edges_[from_id][to_id] = weight
    BoundingBox bbox_;
    ProblemSet probset_;

    // - t_ = current sim time
    // - tmin_ = minimum sim duration (max trip.early)
    // - tmax_ = maximum sim duration (max vehicle.late)
    SimTime t_;
    SimTime tmin_;
    SimTime tmax_;

    size_t active_vehicles_;

    int sleep_interval_;

    void step();
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_CARGO_H_

