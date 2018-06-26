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
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "libcargo/classes.h"
#include "libcargo/functions.h"
#include "libcargo/message.h"
#include "libcargo/types.h"
#include "gtree/gtree.h"

namespace cargo {

Stop::Stop(TripId o, NodeId loc, StopType t, EarlyTime e, LateTime l, SimTime v)
    : Stop(o, loc, t, e, l)
{
    visitedAt_ = v;
}

Stop::Stop(TripId o, NodeId loc, StopType t, EarlyTime e, LateTime l)
{
    owner_ = o;
    location_ = loc;
    type_ = t;
    early_ = e;
    late_ = l;
    visitedAt_ = -1; // indicates "unvisited"
}
const TripId& Stop::owner() const { return owner_; }
const NodeId& Stop::location() const { return location_; }
const StopType& Stop::type() const { return type_; }
const EarlyTime& Stop::early() const { return early_; }
const LateTime& Stop::late() const { return late_; }
const SimTime& Stop::visitedAt() const { return visitedAt_; }

Schedule::Schedule(VehicleId o, std::vector<Stop> n)
{
    owner_ = o;
    data_ = n;
}
const VehicleId& Schedule::owner() const { return owner_; }
const std::vector<Stop>& Schedule::data() const { return data_; }
size_t Schedule::size() const { return data_.size(); }
const Stop& Schedule::at(size_t i) const { return data_.at(i); }
const Stop& Schedule::front() const { return data_.front(); }
void Schedule::print() const
{
    for (const auto& stop : data_)
        std::cout << stop.location() << " ";
    std::cout << std::endl;
}

Route::Route(VehicleId o, std::vector<Waypoint> n)
{
    owner_ = o;
    data_ = n;
}
const VehicleId& Route::owner() const { return owner_; }
const std::vector<Waypoint>& Route::data() const { return data_; }
size_t Route::size() const { return data_.size(); }
NodeId Route::node_at(size_t i) const { return data_.at(i).second; }
DistanceInt Route::dist_at(size_t i) const { return data_.at(i).first; }
Waypoint Route::at(size_t i) const { return data_.at(i); }
void Route::print() const
{
    for (const auto& wp : data_)
        std::cout << "(" << wp.first << "|" << wp.second << ") ";
    std::cout << std::endl;
}

Trip::Trip(TripId o, NodeId oid, NodeId did, EarlyTime e, LateTime l, Load ld)
{
    id_ = o;
    origin_ = oid;
    destination_ = did;
    early_ = e;
    late_ = l;
    load_ = ld;
}
const TripId& Trip::id() const { return id_; }
const OriginId& Trip::origin() const { return origin_; }
const DestinationId& Trip::destination() const { return destination_; }
const EarlyTime& Trip::early() const { return early_; }
const LateTime& Trip::late() const { return late_; }
Load Trip::load() const { return load_; }

Customer::Customer(CustomerId o, OriginId oid, DestinationId did, EarlyTime e,
                   LateTime l, Load ld, CustomerStatus f)
    : Trip(o, oid, did, e, l, ld)
{
    status_ = f;
    assignedTo_ = -1;
}
Customer::Customer(CustomerId o, OriginId oid, DestinationId did, EarlyTime e,
                   LateTime l, Load ld, CustomerStatus f, VehicleId a)
    : Trip(o, oid, did, e, l, ld)
{
    status_ = f;
    assignedTo_ = a;
}
CustomerStatus Customer::status() const { return status_; }
VehicleId Customer::assignedTo() const { return assignedTo_; }
bool Customer::assigned() const { return (assignedTo_ > 0); }
void Customer::print() const
{
    Message print_out;
    print_out
        << "Customer " << this->id() << ":\n"
        << "origin     \t" << this->origin() << "\n"
        << "destination\t" << this->destination() << "\n"
        << "early      \t" << this->early() << "\n"
        << "late       \t" << this->late() << "\n"
        << "status     \t" << (int)this->status() << "\n"
        << "assignedTo \t" << this->assignedTo() << "\n"
        << "assigned   \t" << this->assigned() << std::endl;
}

Vehicle::Vehicle(VehicleId vid, OriginId oid, DestinationId did, EarlyTime et,
        LateTime lt, Load load, GTree::G_Tree& gtree) : Trip(vid, oid, did, et, lt, load)
{
    Stop o(vid, oid, StopType::VehicleOrigin, et, lt, et);
    Stop d(vid, did, StopType::VehicleDest, et, lt);
    std::vector<Waypoint> route_data {};
    route_through({o, d}, route_data, gtree);
    Route route(vid, route_data);
    route_ = route;
    next_node_distance_ = route_.at(1).first;
    idx_last_visited_node_ = 0;
    Stop next_loc(vid, route_.at(1).second, StopType::VehicleOrigin, et, lt);
    Schedule schedule(vid, {next_loc, d});
    schedule_ = schedule;
    queued_ = 0;
    status_ = VehicleStatus::Enroute;
}

Vehicle::Vehicle(VehicleId o, OriginId oid, DestinationId did, EarlyTime e,
                 LateTime l, Load ld, Load qd, DistanceInt nnd, Route r, Schedule s,
                 RouteIndex ri, VehicleStatus f)
    : Trip(o, oid, did, e, l, ld), route_(r), schedule_(s)
{
    next_node_distance_ = nnd;
    idx_last_visited_node_ = ri;
    queued_ = qd;
    status_ = f;
}
DistanceInt Vehicle::next_node_distance() const { return next_node_distance_; }
const Route& Vehicle::route() const { return route_; }
const Schedule& Vehicle::schedule() const { return schedule_; }
RouteIndex Vehicle::idx_last_visited_node() const { return idx_last_visited_node_; }
NodeId Vehicle::last_visited_node() const { return route_.node_at(idx_last_visited_node_); }
Load Vehicle::queued() const { return queued_; }
Load Vehicle::capacity() const { return -load_; } // Convenience function
VehicleStatus Vehicle::status() const { return status_; }
void Vehicle::print() const
{
    Message print_out;
    print_out
        << "Vehicle " << this->id() << ":\n"
        << "origin     \t" << this->origin() << "\n"
        << "destination\t" << this->destination() << "\n"
        << "early      \t" << this->early() << "\n"
        << "late       \t" << this->late() << "\n"
        << "load       \t" << this->load() << "\n"
        << "nnd        \t" << this->next_node_distance() << "\n"
        << "route      \t";
    for (const auto& i : this->route().data())
        print_out << "(" << i.first << "|" << i.second << ") ";
    print_out << "\n";
    print_out
        << "schedule   \t";
    for (const auto& i : this->schedule().data())
        print_out << i.location() << " ";
    print_out << "\n";
    print_out
        << "idx_lvn    \t" << this->idx_last_visited_node() << "\n"
        << "status     \t" << (int)this->status() << std::endl;
}
MutableVehicle::MutableVehicle(const Vehicle& veh) : Vehicle(veh) {}
void MutableVehicle::set_route(const std::vector<Waypoint>& r)
{
    Route route(this->id_, r);
    set_route(route);
}
void MutableVehicle::set_route(const Route& route) { this->route_ = route; }
void MutableVehicle::set_schedule(const std::vector<Stop>& s)
{
    Schedule schedule(this->id_, s);
    set_schedule(schedule);
}
void MutableVehicle::set_schedule(const Schedule& schedule) { this->schedule_ = schedule; }
void MutableVehicle::reset_lvn() { this->idx_last_visited_node_ = 0; }
void MutableVehicle::incr_queued() { this->queued_++; }

ProblemSet::ProblemSet() {}
std::string& ProblemSet::name() { return name_; }
std::string& ProblemSet::road_network() { return road_network_; }
void ProblemSet::set_trips(
    const std::unordered_map<EarlyTime, std::vector<Trip>>& trips)
{
    trips_ = trips;
}
const std::unordered_map<EarlyTime, std::vector<Trip>>&
ProblemSet::trips() const
{
    return trips_;
}

} // namespace cargo

