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
#ifndef CARGO_INCLUDE_SIMULATOR_H_
#define CARGO_INCLUDE_SIMULATOR_H_

#include <unordered_map>

#include "types.h"
#include "options.h"
#include "../gtree/gtree.h"

namespace cargo {

using opts::Options;

class Simulator {
  public:
    Simulator();

    void SetOptions(Options);

    // Attempt to load the nodes, edges, gtree, and problem instance from the
    // path parameters.
    void Initialize();

    // Start the simulation. Call Run() within its own thread, otherwise it
    // will block!
    void Run();

  private:
    // GTree index, used for fast shortest-path computation and knn search of
    // objects on the road network. The index is built from a .edges file,
    // hence the weights are all integer.
    GTree::G_Tree gtree_;

    // All the parameters for the simulator.
    Options opts_;

    // Lookup tables for the nodes and edges.
    NodeMap nodes_;
    EdgeMap edges_;

    // A problem instance.
    ProblemInstance pi_;

    // These mutable tables store the ground truth state of the vehicles.
    // Only the Simulator should have access to them!
    std::unordered_map<TripId, Route> routes_;
    std::unordered_map<TripId, Schedule> schedules_;
    //
    // The positions_ table stores the current position of every vehicle in the
    // simulation, using iterators to point to an element in the vehicle's
    // route. The iterator is constant, so it can be dereferenced to get the
    // node, but a new node cannot be assigned to it.
    //
    // Care must be taken here. Iterators become invalidated after several
    // operations. To be safe, after performing _any_ insertion, erasure, or
    // resizing, rediscover the iterator and update this table.
    std::unordered_map<TripId, Route::const_iterator> positions_;
    //
    // This table keeps track of how much distance each vehicle has remaining
    // until it reaches the next node in its route.
    std::unordered_map<TripId, Distance> residuals_;
    //
    // This table keeps track of the current capacities of each vehicle.
    // TODO: Should the capacity be reduced only after picking up a reqest, or
    // should it be reduced as soon as a request is assigned? If the former,
    // should the capacity table include information about the future capacity
    // of the vehicle?
    std::unordered_map<TripId, Demand> capacities_;

    // Holds the status of the simulator.
    SimulatorStatus status_;

    // Used to keep track of the time in the simulation world.
    SimTime t_;

    // The minimum length of the simulation, equal to the latest trip.early
    SimTime tmin_;

    // Holds the count of active vehicles.
    size_t count_active_;

    // This interval sets the simulation time with respect to real time. The
    // time in the problem instances is in seconds; hence set the sleep
    // interval to be equal to 1000 ms in order to approximate real time.
    // The unit is milliseconds.
    int sleep_;

    // Update the ground-truth tables by moving the vehicles, recomputing
    // positions, residuals, capacities.
    void AdvanceSimulationState();

    // Insert a new vehicle into the ground-truth tables
    //     routes_
    //     schedules_
    //     positions_
    //     residuals_
    //     capacities_
    void InsertVehicle(const Trip &);
};

} // namespace cargo

#endif // CARGO_INCLUDE_SIMULATOR_H_