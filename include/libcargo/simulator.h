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

#include "types.h"
#include "options.h"
#include "../gtree/gtree.h"

namespace cargo {

using opts::Options;

class Simulator {
  public:
    Simulator();

    void SetOptions(Options);

    // Reads the road network, edge file, problem instance, and gtree into
    // memory. Sets the minimum simulation time and the sleep interval. Set
    // the options first before calling this.
    void Initialize();

    // Call this in its own thread, otherwise it will block!
    void Run();

  private:
    // All the parameters for the simulator.
    Options opts_;

    // GTree index, used for fast shortest-path computation and knn search of
    // objects on the road network. The index is built from a .edges file,
    // hence the weights are all integer.
    GTree::G_Tree gtree_;

    // These mutable tables store the ground truth state of the simulation.
    // Only the Simulator should have access to them! Todo: maybe move these
    // out to a relational database. That way, we get benefits like constraint
    // checking and SQL syntax for updates and queries.
    LU_NODES        nodes_;         // k: int NodeId; v: Node
    LU_EDGES        edges_;         // usage: edges_[from_id][to_id] = weight
    LU_ROUTES       routes_;        // k: int TripId; v: Route (vector<Node>)
    LU_SCHEDULES    schedules_;     // k: int TripId; v: Schedule (vector<Stop>)
    LU_POSITIONS    positions_;     // k: int TripId; v: Route::const_itr
    LU_RESIDUALS    residuals_;     // k: int TripId; v: double Distance
    LU_CAPACITIES   capacities_;    // k: int TripId; v: int Demand

    // A problem instance.
    ProblemInstance pi_;

    // Holds the status of the simulator (enum : int)
    SimulatorStatus status_;

    // Used to keep track of the time in the simulation world (int)
    // Initializes to 0.
    SimTime t_;

    // The minimum length of the simulation, equals the latest trip.early (int)
    // Equal to the maximum key in pi_.
    SimTime tmin_;

    // Holds the count of active vehicles.
    // Initializes to 0.
    size_t count_active_;

    // This interval sets the simulation time with respect to real time. The
    // time in the problem instances is in seconds; hence set the sleep
    // interval to be equal to 1000 ms in order to approximate real time.
    // The unit is milliseconds.
    int sleep_;

    // Insert a new vehicle into the ground-truth tables.
    void InsertVehicle(const Vehicle &);

    // Update the ground-truth tables by moving the vehicle.
    void NextVehicleState(const VehicleId &);

    // Returns true if a vehicle is currently at one of its stops. If a vehicle
    // is "stopped" at a stop due to a long residual to the next node, this
    // function will continue to return true!
    bool IsStopped(const VehicleId &);

    // Sets a vehicle's schedule in schedules_ to contain only the remaining
    // stops. The strategy is to check every stop in the current schedule
    // against the vehicle's remaining route.
    void SynchronizeSchedule(const VehicleId &);
};

} // namespace cargo

#endif // CARGO_INCLUDE_SIMULATOR_H_
