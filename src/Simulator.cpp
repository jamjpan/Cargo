#include "Simulator.h"

#include <chrono>
#include <thread>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <iterator>
#include <vector>

#include "base/basic_types.h"
#include "base/ridesharing_types.h"
#include "base/file.h"
#include "gtree/GTree.h"

namespace cargo {

// --------------------------------------------------------
// Simulator
// --------------------------------------------------------

Simulator::Simulator()
    : status_(SimulatorStatus::RUNNING), t_(0), count_active_(0) {}

// Not quite sure what the better way is to set all these parameters!!
Filepath &Simulator::ParamRoadNetworkPath() { return path_rn_; }
Filepath &Simulator::ParamGTreePath() { return path_gtree_; }
Filepath &Simulator::ParamEdgePath() { return path_edges_; }
Filepath &Simulator::ParamProblemInstancePath() { return path_trips_; }
std::string &Simulator::ParamProblemInstanceName() { return name_instance_; }
std::string &Simulator::ParamRoadNetworkName() { return name_rn_; }
size_t &Simulator::ParamNumberOfVehicles() { return count_vehicles_; }
size_t &Simulator::ParamNumberOfCustomers() { return count_customers_; }
size_t &Simulator::ParamNumberOfNodes() { return count_nodes_; }
size_t &Simulator::ParamNumberOfEdges() { return count_edges_; }
Speed &Simulator::ParamVehicleSpeed() { return speed_; }
float &Simulator::ParamSimTimeScale() { return scale_; }

void Simulator::Initialize() {
    file::ReadNodes(path_rn_, nodes_);
    file::ReadEdges(path_edges_, edges_);
    file::ReadProblemInstance(path_trips_, pi_);
    GTree::load(path_gtree_);
    gtree_ = GTree::get();

    // Compute the minimum simulation time (to allow all trips to be
    // broadcasted)
    tmin_ = pi_.trips.rbegin()->first;

    // Compute the sleep interval (ms) based on scale_
    sleep_ = std::round((float)1000/scale_);
}

void Simulator::InsertVehicle(const Trip &trip) {
    // Insert into routes_
    // Use GTree to find the vehicle's initial route from origin to
    // destination. The route returned by GTree is a vector of ints
    // corresponding to node ID's; the distance is an int summing the edge
    // lengths.
    std::vector<int> rt_raw;
    gtree_.find_path(trip.oid, trip.did, rt_raw);
    // Build the Route by walking through the raw route to get the nodes
    Route rt;
    for (auto id : rt_raw)
        rt.push_back(nodes_.at(id));
    routes_[trip.id] = rt;

    Schedule sch;
    sch.push_back({trip.id, trip.oid, StopType::VEHICLE_ORIGIN});
    sch.push_back({trip.id, trip.did, StopType::VEHICLE_DESTINATION});
    schedules_[trip.id] = sch;

    positions_[trip.id] = rt.begin();

    // Insert into residuals_
    // Compute the distance to the next node in the route and store in the
    // residuals. nx is guaranteed to exist because the trip must have at least
    // two nodes, origin and destination.
    auto nx = std::next(rt.begin());
    residuals_[trip.id] = edges_.at(trip.oid).at(nx->id);

    capacities_[trip.id] = trip.demand;

    count_active_++;
}

void Simulator::AdvanceSimulationState() {
    // Move the existing vehicles
    // Loop through residuals_, and update them based on how far each
    // vehicle has moved. If the residual becomes negative, a vehicle has
    // arrived at its next node; update the position and check
    //     (1) does the node belong to a stop in the vehicle's schedule?
    //         if so, update the capacity if it is a DESTINATION type
    //     (2) is the node the VEHICLE_DESTINATION?
    //         if so, decrement active vehicles
    for (const auto &kv : residuals_) {
        TripId tid = kv.first;
        Distance res = kv.second;

        // Only move vehicles where next position is not route.end()
        if (std::next(positions_.at(tid)) != routes_.at(tid).end()) {
            res -= speed_; // speed is m/s; each t_ corresponds to 1 real second
            if (res <= 0) {
                positions_.at(tid)++;
                auto nx = std::next(positions_.at(tid));
                residuals_.at(tid) = edges_.at(positions_.at(tid)->id).at(nx->id);
                // TODO: check if is a stop
            }
        }
    }
}

void Simulator::UpdateRoute(const TripId &tid, const Route) {

}

void Simulator::UpdateSchedule(const TripId &tid, const Schedule) {

}

void Simulator::UpdatePosition(const TripId &tid, Route::const_iterator x) {

}

void Simulator::UpdateResidual(const TripId &tid, const Distance d) {

}

void Simulator::UpdateCapacity(const TripId &tid, const Demand q) {

}

void Simulator::Run() {
    // This process will run (and block anything downstream) until two
    // conditions are met: t_ > tmin_, and no more active vehicles.
    while (true) {
        // Start the clock for this interval
        // auto t_start = std::chrono::high_resolution_clock::now();

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

        // update vehicle status
        // auto t_end = std::chrono::high_resolution_clock::now();
        // auto elapsed = std::round(std::chrono::duration<double, std::milli>(t_end - t_start).count());
        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_));

        // Advance the simulation time
        t_++;
    }
}

} // namespace cargo
