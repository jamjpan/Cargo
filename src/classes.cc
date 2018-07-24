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

Stop::Stop(TripId owner, NodeId loc, StopType type,
        ErlyTime e, LateTime l, SimlTime v) {
  this->owner_ = owner;
  this->location_ = loc;
  this->type_ = type;
  this->early_ = e;
  this->late_ = l;
  this->visitedAt_ = v;
}

const TripId    & Stop::owner()     const { return owner_; }
const NodeId    & Stop::loc()       const { return location_; }
const StopType  & Stop::type()      const { return type_; }
const ErlyTime  & Stop::early()     const { return early_; }
const LateTime  & Stop::late()      const { return late_; }
const SimlTime  & Stop::visitedAt() const { return visitedAt_; }

Schedule::Schedule(VehlId owner, std::vector<Stop> data) {
  this->owner_ = owner;
  this->data_ = data;
}

const VehlId            & Schedule::owner()      const { return owner_; }
const std::vector<Stop> & Schedule::data()       const { return data_; }
const Stop              & Schedule::at(SchIdx i) const { return data_.at(i); }
const Stop              & Schedule::front()      const { return data_.front(); }
      size_t              Schedule::size()       const { return data_.size(); }

void Schedule::print() const {
  for (const auto& stop : data_) std::cout << stop.loc() << " ";
  std::cout << std::endl;
}

Route::Route(VehlId owner, std::vector<Wayp> data) {
  this->owner_ = owner;
  this->data_ = data;
}

const VehlId            & Route::owner()           const { return owner_; }
const std::vector<Wayp> & Route::data()            const { return data_; }
const NodeId            & Route::node_at(RteIdx i) const { return data_.at(i).second; }
const DistInt           & Route::dist_at(RteIdx i) const { return data_.at(i).first; }
const Wayp              & Route::at(RteIdx i)      const { return data_.at(i); }
      size_t              Route::size()            const { return data_.size(); }

void Route::print() const {
  for (const auto& wp : data_) std::cout << "(" << wp.first << "|" << wp.second << ") ";
  std::cout << std::endl;
}

Trip::Trip(TripId owner, NodeId oid, NodeId did, ErlyTime e, LateTime l,
           Load ld) {
  this->id_ = owner;
  this->orig_ = oid;
  this->dest_ = did;
  this->early_ = e;
  this->late_ = l;
  this->load_ = ld;
}

const TripId   & Trip::id()    const { return id_; }
const OrigId   & Trip::orig()  const { return orig_; }
const DestId   & Trip::dest()  const { return dest_; }
const ErlyTime & Trip::early() const { return early_; }
const LateTime & Trip::late()  const { return late_; }
      Load       Trip::load()  const { return load_; }

Customer::Customer(CustId owner, OrigId oid, DestId did, ErlyTime e, LateTime l,
                   Load ld, CustStatus f, VehlId a)
    : Trip(owner, oid, did, e, l, ld) {
  this->status_ = f;
  this->assignedTo_ = a;
}

const CustStatus & Customer::status()     const { return status_; }
const VehlId     & Customer::assignedTo() const { return assignedTo_; }
      bool         Customer::assigned()   const { return (assignedTo_ > 0); }

void Customer::print() const {
    std::cout << "Customer " << this->id() << ":\n"
              << "origin     \t" << this->orig() << "\n"
              << "destination\t" << this->dest() << "\n"
              << "early      \t" << this->early() << "\n"
              << "late       \t" << this->late() << "\n"
              << "status     \t" << (int)this->status() << "\n"
              << "assignedTo \t" << this->assignedTo() << "\n"
              << "assigned   \t" << this->assigned() << std::endl;
}

Vehicle::Vehicle(VehlId vid, OrigId oid, DestId did, ErlyTime et, LateTime lt,
                 Load load, GTree::G_Tree& gtree)
    : Trip(vid, oid, did, et, lt, load) {
  /* Initialize default route */
  Stop o(vid, oid, StopType::VehlOrig, et, lt, et);
  Stop d(vid, did, StopType::VehlDest, et, lt);
  std::vector<Wayp> route_data{};
  route_through({o, d}, route_data, gtree);
  Route rte(vid, route_data);

  /* Initialize default schedule */
  Stop next_loc(vid, rte.at(1).second, StopType::VehlOrig, et, lt);
  Schedule sch(vid, {next_loc, d});

  this->route_ = rte;
  this->next_node_distance_ = rte.at(1).first;
  this->idx_last_visited_node_ = 0;
  this->schedule_ = sch;
  this->queued_ = 0;
  this->status_ = VehlStatus::Enroute;
}

Vehicle::Vehicle(VehlId vid, OrigId oid, DestId did, ErlyTime e, LateTime l,
                 Load ld, Load qd, DistInt nnd, Route r, Schedule s, RteIdx ri,
                 VehlStatus f)
    : Trip(vid, oid, did, e, l, ld), route_(r), schedule_(s) {
  this->next_node_distance_ = nnd;
  this->idx_last_visited_node_ = ri;
  this->queued_ = qd;
  this->status_ = f;
}

const DistInt    & Vehicle::next_node_distance()    const { return next_node_distance_; }
const Route      & Vehicle::route()                 const { return route_; }
const Schedule   & Vehicle::schedule()              const { return schedule_; }
const RteIdx     & Vehicle::idx_last_visited_node() const { return idx_last_visited_node_; }
const NodeId     & Vehicle::last_visited_node()     const { return route_.node_at(idx_last_visited_node_); }
const VehlStatus & Vehicle::status()                const { return status_; }
      Load         Vehicle::queued()                const { return queued_; }
      Load         Vehicle::capacity()              const { return -load_; }
void Vehicle::print() const {
    std::cout << "Vehicle " << this->id() << ":\n"
              << "origin     \t" << this->orig() << "\n"
              << "destination\t" << this->dest() << "\n"
              << "early      \t" << this->early() << "\n"
              << "late       \t" << this->late() << "\n"
              << "load       \t" << this->load() << "\n"
              << "nnd        \t" << this->next_node_distance() << "\n"
              << "route      \t";
  for (const auto& i : this->route().data())
    std::cout << "(" << i.first << "|" << i.second << ") ";
  std::cout << "\n";
  std::cout << "schedule   \t";
  for (const auto& i : this->schedule().data())
    std::cout << i.loc() << " ";
  std::cout << "\n";
  std::cout << "idx_lvn    \t" << this->idx_last_visited_node() << "\n"
            << "status     \t" << (int)this->status() << std::endl;
}
MutableVehicle::MutableVehicle(const Vehicle& veh) : Vehicle(veh) {}
void MutableVehicle::set_rte(const std::vector<Wayp>& r) {
  Route route(this->id_, r);
  set_rte(route);
}
void MutableVehicle::set_rte(const Route& route) { this->route_ = route; }
void MutableVehicle::set_sch(const std::vector<Stop>& s) {
  Schedule schedule(this->id_, s);
  set_sch(schedule);
}
void MutableVehicle::set_sch(const Schedule& schedule) {
  this->schedule_ = schedule;
}
void MutableVehicle::set_nnd(const DistInt& sync_nnd) {
  this->next_node_distance_ = sync_nnd;
}
void MutableVehicle::reset_lvn()    { this->idx_last_visited_node_ = 0; }
void MutableVehicle::incr_queued()  { this->queued_++; }

ProblemSet::ProblemSet() {}
std::string & ProblemSet::name()            { return name_; }
std::string & ProblemSet::road_network()    { return road_network_; }
void ProblemSet::set_trips(
    const std::unordered_map<ErlyTime, std::vector<Trip>>& trips) {
  trips_ = trips;
}
const std::unordered_map<ErlyTime, std::vector<Trip>>& ProblemSet::trips()
    const {
  return trips_;
}

} // namespace cargo

