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
#include <algorithm>

#include "libcargo/simulator.h"
#include "libcargo/types.h"
#include "libcargo/file.h"
#include "libcargo/options.h"
#include "libcargo/message.h"
#include "gtree/gtree.h"

namespace cargo {

using opts::Options;
using file::ReadNodes;
using file::ReadEdges;
using file::ReadProblemInstance;
using  msg::Message;
using  msg::MessageType;

// --------------------------------------------------------
// Simulator
// --------------------------------------------------------

Simulator::Simulator()
    : status_(SimulatorStatus::RUNNING), t_(0), count_active_(0) {}

void Simulator::SetOptions(Options opts) {
    opts_ = opts;
}

void Simulator::Initialize() {
    ReadNodes(opts_.RoadNetworkPath, nodes_);
    ReadEdges(opts_.EdgeFilePath, edges_);
    ReadProblemInstance(opts_.ProblemInstancePath, pi_);

    GTree::load(opts_.GTreePath);
    gtree_ = GTree::get();

    // Sets the minimim simulation time to ensure all trips will be broadcasted.
    tmin_ = pi_.trips.rbegin()->first;

    // Sets the sleep time based on the time scale option.
    sleep_ = std::round((float)1000/opts_.Scale);

    Message info(MessageType::INFO);
    info << "finished initialization" << std::endl;
}

void Simulator::InsertVehicle(const Vehicle &veh) {
    // Uses GTree to find the vehicle's initial route from origin to
    // destination. The route returned by GTree is a vector of ints
    // corresponding to node ID's; the distance is an int summing the edge
    // lengths. So, the vector is walked in order to find the corresponding
    // Nodes.
    std::vector<int> rt_raw;
    gtree_.find_path(veh.oid, veh.did, rt_raw);
    Route rt;
    for (auto node_id : rt_raw)
        rt.push_back(nodes_.at(node_id));
    routes_[veh.id] = rt;

    // The initial schedule is the vehicle's origin and destination.
    Schedule sch;
    sch.push_back({veh.id, veh.oid, StopType::VEHICLE_ORIGIN});
    sch.push_back({veh.id, veh.did, StopType::VEHICLE_DESTINATION});
    schedules_[veh.id] = sch;

    // The initial position is the head of the route.
    positions_[veh.id] = rt.begin();

    // Compute the distance to the next node in the route. nx is guaranteed to
    // exist because the trip must have at least two nodes, origin and
    // destination.
    auto nx = std::next(rt.begin());
    residuals_[veh.id] = edges_.at(veh.oid).at(nx->node_id);

    // The capacity is equal to the trip demand.
    capacities_[veh.id] = veh.demand;

    count_active_++;
}

void Simulator::NextVehicleState(const VehicleId &vid) {
    // Remove the vehicle from residuals_ table if there is no more route.
    if (std::next(positions_.at(vid)) == routes_.at(vid).end()) {
        residuals_.erase(vid);
        return;
    }

    // Reduce the residual following one time step. Speed is in m/s, so we
    // just need to deduct the speed.
    Distance res = residuals_.at(vid);
    res -= opts_.VehicleSpeed;

    // Update position of the vehicle if the residual is < 0, otherwise
    // just update the residual
    if (res <= 0) {
        positions_[vid]++;
        // Compute the residual to the next position
        auto nx = std::next(positions_.at(vid));
        residuals_[vid] =
            edges_.at(positions_.at(vid)->node_id).at(nx->node_id);

        // Handle stops

    } else {
        residuals_[vid] = res;
    }
}

bool Simulator::IsStopped(const VehicleId &vid) {
    NodeId x = positions_.at(vid)->node_id;
    Schedule _s = schedules_.at(vid);
    for (auto s : _s)
        if (s.did == x)
            return true;
    return false;
}

void Simulator::SynchronizeSchedule(const VehicleId &vid) {
    // Extract the vehicle's remaining route
    const Route &r = routes_.at(vid);
    const Node &x = *(positions_.at(vid));
    Route rt_rem;
    auto i = std::find(r.begin(), r.end(), x);
    std::copy(i, r.end(), std::back_inserter(rt_rem));

    // Check each stop in the vehicle's schedule to see if it is in the
    // remaining route. If so, keep the stop.
    Schedule s_new;
    const Schedule &s = schedules_.at(vid);
    for (auto stop : s)
        for (auto j = rt_rem.begin(); j != rt_rem.end(); ++j)
            if (stop.did == j->node_id)
                s_new.push_back(stop);

    // Commit the synchronize schedule
    schedules_[vid] = s_new;
}

void Simulator::Run() {
    // This process will run (and block anything downstream) until two
    // conditions are met: t_ > tmin_, and no more active vehicles.
    while (true) {
        // Start the clock for this interval
        auto t_start = std::chrono::high_resolution_clock::now();

        // Move all the vehicles
        if (t_ > 0) {
            for (const auto &kv : residuals_)
                NextVehicleState(kv.first);
        }

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
        // thread is still too slow, increase the scale; but then, the
        // simulation won't be real-time anymore; it will be slowed down.
        auto t_end = std::chrono::high_resolution_clock::now();
        int elapsed = std::round(
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
