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
#ifndef CARGO_INCLUDE_LIBCARGO_CLASSES_H_
#define CARGO_INCLUDE_LIBCARGO_CLASSES_H_

#include <string>
#include <unordered_map>
#include <vector>

#include "types.h"

// Properties of these classes are immutable, and they are accesed via the
// "inspector method"; see https://isocpp.org/wiki/faq/const-correctness
//
// Why the weird indentation? To make it easier to see the interface.
namespace cargo {

// A stop is a customer (vehicle) origin or destination and correspond to
// Cordeau's "nodes".
class Stop {
public:
    Stop(TripId, NodeId, StopType, EarlyTime, LateTime, SimTime);
    Stop(TripId, NodeId, StopType, EarlyTime, LateTime); // visitedAt gets -1
    const TripId&               owner()                 const;
    const NodeId&               location()              const;
    const StopType&             type()                  const;
    const EarlyTime&            early()                 const;
    const LateTime&             late()                  const;
    const SimTime&              visitedAt()             const;

private:
    TripId owner_;
    NodeId location_;
    StopType type_;
    EarlyTime early_;
    LateTime late_;
    SimTime visitedAt_;
};

// Schedules are meant to be attached to vehicles. If you need to manipulate a
// schedule, for example construct one, use std::vector<Stop>
class Schedule {
public:
    Schedule(VehicleId, std::vector<Stop>);
    const VehicleId&            owner()                 const;
    const std::vector<Stop>&    data()                  const;
    size_t                      size()                  const;
    const Stop&                 at(ScheduleIndex)       const;
    const Stop&                 front()                 const;
    void                        print()                 const;

private:
    VehicleId owner_;
    std::vector<Stop> data_;
};

// Routes are meant to be attached to vehicles. If you need to manipulate a
// route, for example construct one from segments, use std::vector<NodeId>
class Route {
public:
    Route(VehicleId, std::vector<Waypoint>);
    const VehicleId&            owner()                 const;
    const std::vector<Waypoint>& data()                 const;
    size_t                      size()                  const;
    NodeId                      node_at(RouteIndex)     const;
    DistanceInt                 dist_at(RouteIndex)     const;
    Waypoint                    at(RouteIndex)          const;
    void                        print()                 const;

private:
    VehicleId owner_;
    std::vector<Waypoint> data_;
};

// A Trip is the base class for a customer or vehicle.
class Trip {
public:
    Trip(TripId, OriginId, DestinationId, EarlyTime, LateTime, Load);
    const TripId&               id()                    const;
    const OriginId&             origin()                const;
    const DestinationId&        destination()           const;
    const EarlyTime&            early()                 const;
    const LateTime&             late()                  const;
    Load                        load()                  const;

protected:
    TripId id_;
    OriginId origin_;
    DestinationId destination_;
    EarlyTime early_;
    LateTime late_;
    Load load_;
};

// A Customer represents a new ride request
class Customer : public Trip {
public:
    Customer(CustomerId, OriginId, DestinationId, EarlyTime, LateTime, Load,
             CustomerStatus);
    Customer(CustomerId, OriginId, DestinationId, EarlyTime, LateTime, Load,
             CustomerStatus, VehicleId);
    CustomerStatus              status()                const;
    VehicleId                   assignedTo()            const;
    bool                        assigned()              const;

private:
    CustomerStatus status_;
    VehicleId assignedTo_;

};

// Don't bother trying to modify a vehicle. Vehicle objects are meant to be
// ephemeral anyway. If you want to modify a vehicle, modify the ground truth,
// stored in the database.
class Vehicle : public Trip {
public:
    Vehicle(VehicleId, OriginId, DestinationId, EarlyTime, LateTime, Load,
            DistanceInt, Route, Schedule, RouteIndex, VehicleStatus);
    DistanceInt                 next_node_distance()    const;
    const Route&                route()                 const;
    const Schedule&             schedule()              const;
    RouteIndex                  idx_last_visited_node() const;
    NodeId                      last_visited_node()     const;
    VehicleStatus               status()                const;
    void                        print()                 const;

private:
    DistanceInt next_node_distance_;
    Route route_;
    Schedule schedule_;
    RouteIndex idx_last_visited_node_;
    VehicleStatus status_;

};

// A problem is the set of trips keyed by their early time. When the
// simulator time reaches SimTime, all the trips in the group are broadcasted.
class ProblemSet {
public:
    ProblemSet();
    std::string&                name();
    std::string&                road_network();
    void                        set_trips(const std::unordered_map<EarlyTime,
                                    std::vector<Trip>> &);
    const std::unordered_map<EarlyTime, std::vector<Trip>>&
                                trips()                 const;

private:
    std::string name_;
    std::string road_network_;
    std::unordered_map<SimTime, std::vector<Trip>> trips_;
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_CLASSES_H_

