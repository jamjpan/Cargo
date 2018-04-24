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

#include <chrono>
#include <thread>
#include <iterator>
#include <vector>

#include <iostream>

#include "libcargo/simulator.h"
#include "libcargo/types.h"
#include "libcargo/file.h"
#include "libcargo/options.h"
#include "gtree/gtree.h"

namespace cargo {

using opts::Options;
using file::ReadNodes;
using file::ReadEdges;
using file::ReadProblemInstance;

// --------------------------------------------------------
// Simulator
// --------------------------------------------------------

Simulator::Simulator()
    : status_(SimulatorStatus::RUNNING), t_(0), count_active_(0) {}

void Simulator::SetOptions(Options opts) {
    opts_ = opts;
}

void Simulator::Initialize() {
    // Loads the road network and the problem instance.
    ReadNodes(opts_.RoadNetworkPath, nodes_);
    ReadEdges(opts_.EdgeFilePath, edges_);
    ReadProblemInstance(opts_.ProblemInstancePath, pi_);

    // Sets the gtree.
    GTree::load(opts_.GTreePath);
    gtree_ = GTree::get();

    // Sets the minimim simulation time to ensure all trips will be broadcasted.
    tmin_ = pi_.trips.rbegin()->first;

    // Sets the sleep time based on the time scale option.
    sleep_ = std::round((float)1000/opts_.Scale);
    std::cout << "finished initialization" << std::endl;
}

void Simulator::InsertVehicle(const Trip &trip) {
    // (1) Inserts the vehicle's route into the routes_ table.
    // Uses GTree to find the vehicle's initial route from origin to
    // destination. The route returned by GTree is a vector of ints
    // corresponding to node ID's; the distance is an int summing the edge
    // lengths. So, the vector is walked in order to find the corresponding
    // Nodes.
    std::vector<int> rt_raw;
    gtree_.find_path(trip.oid, trip.did, rt_raw);
    Route rt;
    for (auto id : rt_raw)
        rt.push_back(nodes_.at(id));
    routes_[trip.id] = rt;

    // (2) Inserts the vehicle's schedule into the schedules_ table.
    // The initial schedule is the vehicle's origin and destination.
    Schedule sch;
    sch.push_back({trip.id, trip.oid, StopType::VEHICLE_ORIGIN});
    sch.push_back({trip.id, trip.did, StopType::VEHICLE_DESTINATION});
    schedules_[trip.id] = sch;

    // (3) Inserts the vehicle's position into the positions_ table.
    // The initial position is the head of the route.
    positions_[trip.id] = rt.begin();

    // (4) Inserts the vehicle's residual into the residuals_ table.  Computes
    // the distance to the next node in the route. nx is guaranteed to exist
    // because the trip must have at least two nodes, origin and destination.
    auto nx = std::next(rt.begin());
    residuals_[trip.id] = edges_.at(trip.oid).at(nx->id);

    // (5) Inserts the vehicle's capacity into the capacities_ table.
    // The capacity is equal to the trip demand.
    capacities_[trip.id] = trip.demand;

    // (6) Increment the count of active vehicles.
    count_active_++;
}

void Simulator::AdvanceSimulationState() {
    // Sets new residuals_, positions_, capacities_ based on
    // vehicle movement.
    for (const auto &kv : residuals_) {
        TripId tid = kv.first;
        Distance res = kv.second;

        // Only move vehicles where next position is not route.end()
        if (std::next(positions_.at(tid)) != routes_.at(tid).end()) {
            // speed is m/s; each t_ corresponds to 1 real second
            res -= opts_.VehicleSpeed;
            if (res <= 0) {
                // (1) Increment the current position
                positions_.at(tid)++;
                // (2) Compute the residual to the next position
                auto nx = std::next(positions_.at(tid));
                residuals_.at(tid) = edges_.at(positions_.at(tid)->id).at(nx->id);
                // TODO: check if is a stop
            } else {
                residuals_[tid] = res;
            }
        }

    } // for
}

void Simulator::Run() {
    // This process will run (and block anything downstream) until two
    // conditions are met: t_ > tmin_, and no more active vehicles.
    while (true) {
        // Start the clock for this interval
        auto t_start = std::chrono::high_resolution_clock::now();

        if (t_ > 0)
            AdvanceSimulationState();

        // All trips broadcasted, and no more active vehicles?
        if (t_ > tmin_ && count_active_ == 0)
            break;

        // Broadcast new trips
        // Loop through the TripGroup corresponding to the current SimTime t_
        // and broadcast based on whether the trip is a customer or vehicle
        if (pi_.trips.find(t_) != pi_.trips.end()) {
            for (auto trip : pi_.trips[t_]) {
                if (trip.demand < 0) {
                    InsertVehicle(trip);
                    // TODO: broadcast a new vehicle
                } else {
                    // TODO: customer_online() embark
                }
            }
        }

        // If the elapsed time is too long, the simulator's run thread is too
        // slow to fit inside real-time. Reset the Scale option to be 1. If the
        // thread is still too slow, reduce the scale even further; but then,
        // the simulation won't be real-time anymore; it will be slowed down.
        auto t_end = std::chrono::high_resolution_clock::now();
        auto elapsed = std::round(
            std::chrono::duration<double, std::milli>(t_end - t_start).count());
        if (elapsed > sleep_) {
            std::cerr << "Scale too big, exiting" << std::endl;
            exit(0);
        }

        // Sleep until the next time interval
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ - elapsed));

        // Advance the simulation time
        t_++;
    }
}

} // namespace cargo
