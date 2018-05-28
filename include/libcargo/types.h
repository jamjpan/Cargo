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
#ifndef CARGO_INCLUDE_TYPES_H_
#define CARGO_INCLUDE_TYPES_H_

#include <chrono>
#include <limits>
#include <list>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace cargo {

// We use many "logical" numerical types such as node IDs, edge IDs,
// trip IDs, etc. Unfortunately the possibility exists for these types to
// get "mingled" in the code. For example, consider, where TripId and NodeId
// are int types:
//     TripId tid;
//     NodeId nid;
//
// The following assignment is allowed by C++ even though the two vars are
// logically different:
//     nid = tid;
//     tid = nid;
//
// There are other similar issues in type conversion.
//
// Google or-tools has a nice template class in int_type.h to prevent these
// kinds of issues. We can consider using their template IntType class in the
// future. But for now, using typedefs to at least provide some semantic
// difference is better than nothing.

// ints are guaranteed at least 32-bits ~> 2 billion values.
typedef int NodeId;

// these are part of the same "type-class", and are interchangeable.
typedef int TripId;
typedef int VehicleId;
typedef int CustomerId;

// No need for double precision because these will never be operated on. Float
// gives us 7 decimal digits. For lng/lat coordinates, the 6th digit corresponds
// to a precision of roughly 110 centimeters(!), so 7 is more than enough.
typedef float Longitude;
typedef float Latitude;

// Spatial data type.
struct Point {
    Longitude lng;
    Latitude lat;
};

// Used as the internal simulation time; one SimTime is roughly equivalent to
// one real second. Time windows are expressed as SimTime, with 0 being the
// start of the simulation. Travel time is also expressed as SimTime, computed
// as the real (haversine) distance divided by the real speed, in m/s.
typedef int SimTime;

enum class StopType {
    CUSTOMER_ORIGIN,
    CUSTOMER_DEST,
    VEHICLE_ORIGIN,
    VEHICLE_DEST,
};

// visit_time defaults to -1 and is populated when a vehicle visits the stop.
struct Stop {
    TripId trip_id;
    NodeId node_id;
    StopType type;
    SimTime visit_time;
    // Time limit for a stop, if type == ORIGIN, it's early time, otherwise late
    // time
    // @James addition variable
    SimTime time_limit;

    // bool operator==(const Stop& a) {
    //     return (a.trip_id == trip_id && a.node_id == node_id && a.type ==
    //     type
    //             && a.time_limit == time_limit);
    // }
};

typedef std::vector<NodeId> Route;
typedef std::vector<Stop> Schedule;
typedef std::list<NodeId> Routel;
typedef std::list<Stop> Schedulel;

// Trip represents shared properties between customers and vehicles.
struct Trip {
    TripId id;
    NodeId oid;
    NodeId did;

    // The time window is expressed as SimTimes. early tells the simulator when
    // to broadcast this trip. late tells the solver the constraint for when
    // the trip should arrive at destination.
    SimTime early;
    SimTime late;

    // Positive corresponds to a customer request; negative
    // corresponds to vehicle capacity.
    int demand;
};

// nnd is the next-node distance (negative, to indicate remaining distance).
// lv_node and lv_stop are the indices to the last-visited node and stop. These
// are advanced as the vehicle moves along its route.
struct Vehicle : public Trip {
    int load; // mutate this instead of vehicle.demand
    int nnd;
    Route route;
    Schedule sched;
    size_t lv_node;
    size_t lv_stop;
    bool is_active;

    // Copy the properties of t into this vehicle.
    Vehicle(const Trip &t)
        : Trip(t), load(t.demand), nnd(0), lv_node(0), lv_stop(0),
          is_active(true){};

    // @James add default constructor for map use
    Vehicle()
        : Trip(Trip{-1, -1, -1, -1, -1, 0}), load(0), nnd(0), lv_node(0),
          lv_stop(0), is_active(false){};
};

// Lookup nodes.
typedef std::unordered_map<NodeId, Point> KeyValueNodes;

// Lookup edges.  The key-value store is "undirected"; that is, from-to and
// to-from key combinations both exist in the store. Usage:
//     EdgeMap em_;
//     em[from_id][to_id] = weight;
typedef std::unordered_map<NodeId, std::unordered_map<NodeId, double>>
    KeyValueEdges;

// Lookup vehicles.
typedef std::unordered_map<VehicleId, Vehicle> KeyValueVehicles;

// Store assignments.
typedef std::unordered_map<VehicleId, CustomerId> KeyValueAssignments;

// Request broadcast time map
typedef std::unordered_map<
    CustomerId, std::chrono::time_point<std::chrono::high_resolution_clock>>
    KeyValueBroadcastTime;

// A problem instance is the set of trips keyed by their early time. When the
// simulator time reaches SimTime, all the trips in the group are broadcasted.
struct ProblemInstance {
    std::string name;
    std::string road_network;
    std::unordered_map<SimTime, std::vector<Trip>> trips;
};

// Simulator status flags
// TODO: These might not be necessary?
enum class SimulatorStatus {
    RUNNING,
    FINISHED,
};

// Vehicle speed, in m/s
typedef float Speed;

// Filepath
typedef std::string Filepath;

// Infinity
const int kIntInfinity = std::numeric_limits<int>::max();
const double kDblInfinity = std::numeric_limits<double>::infinity();

// Math PI
const double kPI = 3.141592653589793238462643383279502884L;

} // namespace cargo

#endif // CARGO_INCLUDE_TYPES_H_
