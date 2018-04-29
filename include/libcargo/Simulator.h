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
#include "message.h"
#include "mqueue.h"
// #include "Solution.h"
#include "../gtree/gtree.h"

namespace cargo
{

using msg::Message;
using opts::Options;

class Solution;

class Simulator
{
  public:
    Simulator();

    // Do this first
    void SetOptions(Options);
    // Just like connecting to a http server
    void SetSolution(Solution *);

    // - Read road network, edge file, problem instance
    // - Set tmin_, sleep_
    void Initialize();

    // Call this in its own thread!
    // Returns false if finishes with no errors; true if finishes with errors.
    bool Run();
    void Terminate() { count_active_ = 0; };

    // Set the assignment
    // Returns false if the assignment is invalid(due to unacceptable route)
    bool RequestMatched(const CustomerId &, const VehicleId &, const Schedule &, const Route &);

    int Speed() { return opts_.VehicleSpeed; };

    int TotalMatch() { return total_match_; };

    int TotalTime() { return total_time_; };

  private:
    Message PRINT;
    Message INFO;
    Message WARN;
    Message ERROR;
    Message SUCCESS;

    Options opts_;

    GTree::G_Tree gtree_;

    // These tables store the ground truth state of the simulation.
    // Only the Simulator should have access to them!
    KeyValueNodes nodes_;
    KeyValueEdges edges_; // usage: edges_[from_id][to_id] = weight
    KeyValueVehicles vehicles_;
    KeyValueAssignments assignments_;
    KeyValueBroadcastTime broadcast_time_;

    ProblemInstance pi_;
    SimulatorStatus status_;

    // - t_ = current sim time
    // - tmin_ = minimum sim duration (max trip.early)
    // - tmax_ = maximum sim duration (max vehicle.late)
    SimTime t_, tmin_, tmax_;

    // Stores the count of active vehicles (inits to 0).
    size_t count_active_;

    // Stores the sleep interval per simulation time step, in milliseconds.
    // (set with Initialize()).
    int sleep_;

    // Total runtime of all matched requets
    int total_time_;
    // Count of matched
    int total_match_;
    // message queue handler
    mqd_t mq_;

    // Solution pointer, for calling VehicleOnline and RequestOnline
    // Use pointer instead of reference because we have to set the solution
    // after the contructor called
    Solution *solution_ = nullptr;

    void MoveVehicles();
};

} // namespace cargo

#endif // CARGO_INCLUDE_SIMULATOR_H_
