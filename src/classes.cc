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
#include "libcargo/classes.h"

namespace cargo {

Stop::Stop(TripId o, NodeId loc, StopType t, EarlyTime e, LateTime l, SimTime v)
{
    owner_ = o;
    location_ = loc;
    type_ = t;
    early_ = e;
    late_ = l;
    visitedAt_ = v;
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

Route::Route(VehicleId o, std::vector<NodeId> n)
{
    owner_ = o;
    data_ = n;
}
const VehicleId& Route::owner() const { return owner_; }
const std::vector<NodeId>& Route::data() const { return data_; }
size_t Route::size() const { return data_.size(); }
NodeId Route::at(size_t i) const { return data_.at(i); }

Trip::Trip(TripId o, NodeId oid, NodeId did, EarlyTime e, LateTime l, Load ld)
{
    owner_ = o;
    origin_ = oid;
    destination_ = did;
    early_ = e;
    late_ = l;
    load_ = ld;
}
const TripId& Trip::owner() const { return owner_; }
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
}
CustomerStatus Customer::status() const { return status_; }

Vehicle::Vehicle(VehicleId o, OriginId oid, DestinationId did, EarlyTime e,
                 LateTime l, Load ld, DistanceInt nnd, Route r, Schedule s,
                 RouteIndex ri, ScheduleIndex si, VehicleStatus f)
    : Trip(o, oid, did, e, l, ld), route_(r), schedule_(s)
{
    next_node_distance_ = nnd;
    idx_last_visited_node_ = ri;
    idx_last_visited_stop_ = si;
    status_ = f;
}
DistanceInt Vehicle::next_node_distance() const { return next_node_distance_; }
const Route& Vehicle::route() const { return route_; }
const Schedule& Vehicle::schedule() const { return schedule_; }
RouteIndex Vehicle::idx_last_visited_node() const
{
    return idx_last_visited_node_;
}
ScheduleIndex Vehicle::idx_last_visited_stop() const
{
    return idx_last_visited_stop_;
}
NodeId Vehicle::last_visited_node() const
{
    return route_.at(idx_last_visited_node_);
}
Stop Vehicle::last_visited_stop() const
{
    return schedule_.at(idx_last_visited_stop_);
}
VehicleStatus Vehicle::status() const { return status_; }

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

