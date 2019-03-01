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
#include <algorithm>
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

/* Stop ----------------------------------------------------------------------*/
Stop::Stop(
  TripId owner,
  NodeId loc,
  StopType type,
  ErlyTime e,
  LateTime l,
  SimlTime v)
{
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


/* Schedule ------------------------------------------------------------------*/
Schedule::Schedule(
  VehlId owner,
  vec_t<Stop> data)
{
  this->owner_ = owner;
  this->data_ = data;
}

const VehlId            & Schedule::owner()      const { return owner_; }
const vec_t<Stop>       & Schedule::data()       const { return data_; }
const Stop              & Schedule::at(SchIdx i) const { return data_.at(i); }
const Stop              & Schedule::front()      const { return data_.front(); }
const Stop              & Schedule::back()       const { return data_.back(); }
      size_t              Schedule::size()       const { return data_.size(); }

void Schedule::print() const {
  for (const auto & stop : data_)
    std::cout << stop.loc() << " ";
  std::cout << std::endl;
}


/* Route ---------------------------------------------------------------------*/
Route::Route(
  VehlId owner,
  vec_t<Wayp> data)
{
  this->owner_ = owner;
  this->data_ = data;
}

const VehlId      & Route::owner()           const { return owner_; }
const vec_t<Wayp> & Route::data()            const { return data_; }
const NodeId      & Route::node_at(RteIdx i) const { return data_.at(i).second; }
const DistInt     & Route::dist_at(RteIdx i) const { return data_.at(i).first; }
const DistInt     & Route::cost()            const { return data_.back().first; }
const Wayp        & Route::at(RteIdx i)      const { return data_.at(i); }
      size_t        Route::size()            const { return data_.size(); }

void Route::print() const {
  for (const auto & wp : data_)
    std::cout << "(" << wp.first << "|" << wp.second << ") ";
  std::cout << std::endl;
}


/* Trip ----------------------------------------------------------------------*/
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


/* Customer ------------------------------------------------------------------*/
Customer::Customer(
  CustId owner,
  OrigId oid,
  DestId did,
  ErlyTime e,
  LateTime l,
  Load ld,
  CustStatus f,
  VehlId a)
    : Trip(owner, oid, did, e, l, ld)
{
  this->status_ = f;
  this->assignedTo_ = a;
}

const CustStatus & Customer::status()     const { return status_; }
const VehlId     & Customer::assignedTo() const { return assignedTo_; }
      bool         Customer::assigned()   const { return (assignedTo_ > 0); }

void Customer::print() const {
  std::cout
    << "Customer "       << this->id()          << ":\n"
    << "  origin     \t" << this->orig()        << "\n"
    << "  destination\t" << this->dest()        << "\n"
    << "  early      \t" << this->early()       << "\n"
    << "  late       \t" << this->late()        << "\n"
    << "  status     \t" << (int)this->status() << "\n"
    << "  assignedTo \t" << this->assignedTo()  << "\n"
    << "  assigned   \t" << this->assigned()    << std::endl;
}


/* Vehicle -------------------------------------------------------------------*/
Vehicle::Vehicle(
  VehlId vid,
  OrigId oid,
  DestId did,
  ErlyTime et,
  LateTime lt,
  Load load,
  GTree::G_Tree & gtree)
    : Trip(vid, oid, did, et, lt, load)
{
  /* Initialize default route */
  Stop o(vid, oid, StopType::VehlOrig, et, lt, et);  // create origin
  Stop d(vid, did, StopType::VehlDest, et, lt);      // create destination
  vec_t<Wayp> route_data{};                          // container for route
  route_through({o, d}, route_data, gtree);          // compute route
  Route rte(vid, route_data);                        // construct Route

  /* Initialize default schedule
   * (first stop equals vehicle's next location in route) */
  Stop next_loc(vid, rte.at(1).second, StopType::VehlOrig, et, lt);
  Schedule sch(vid, {next_loc, d});                  // construct Schedule

  this->route_ = rte;                                // set route
  this->next_node_distance_ = rte.at(1).first;       // set dist to next node
  this->idx_last_visited_node_ = 0;                  // set last-visited node
  this->schedule_ = sch;                             // set schedule
  this->queued_ = 0;                                 // set queued (none)
  this->status_ = VehlStatus::Enroute;               // set status (Enroute)
}

Vehicle::Vehicle(
  VehlId vid,
  OrigId oid,
  DestId did,
  ErlyTime e,
  LateTime l,
  Load ld,
  Load qd,
  DistInt nnd,
  Route r,
  Schedule s,
  RteIdx ri,
  VehlStatus f)
    : Trip(vid, oid, did, e, l, ld), route_(r), schedule_(s)
{
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
      DistInt      Vehicle::remaining()             const { return this->route().cost() - this->traveled(); }

DistInt Vehicle::traveled() const {  // is this ever used??
  DistInt traveled = this->route_.dist_at(this->idx_last_visited_node_);
  NodeId lvn = this->route_.node_at(this->idx_last_visited_node_);
  NodeId next = this->route_.node_at(this->idx_last_visited_node_+1);
  DistInt base = Cargo::edgew(lvn, next);
  return traveled + (base - this->next_node_distance_);
}

void Vehicle::print() const {
  std::cout
    << "Vehicle "      << this->id()      << ":\n"
    << "origin     \t" << this->orig()    << "\n"
    << "destination\t" << this->dest()    << "\n"
    << "early      \t" << this->early()   << "\n"
    << "late       \t" << this->late()    << "\n"
    << "load       \t" << this->load()    << "\n"
    << "nnd        \t" << this->next_node_distance() << "\n"
    << "route      \t";
  for (const auto & i : this->route().data())
    std::cout << "(" << i.first << "|" << i.second << ") ";
  std::cout << "\n";
  std::cout
    << "schedule   \t";
  for (const auto & i : this->schedule().data())
    std::cout << i.loc() << " ";
  std::cout << "\n";
  std::cout
    << "idx_lvn    \t" << this->idx_last_visited_node() << "\n"
    << "status     \t" << (int)this->status() << std::endl;
}


/* MutableVehicle ------------------------------------------------------------*/
MutableVehicle::MutableVehicle(const Vehicle & veh) : Vehicle(veh) {}  // copy cstor

void MutableVehicle::set_rte(const vec_t<Wayp> & r) {
  Route route(this->id_, r);
  set_rte(route);
}

void MutableVehicle::set_rte(const Route & route) {
  this->route_ = route;
}

void MutableVehicle::set_sch(const vec_t<Stop> & s) {
  Schedule schedule(this->id_, s);
  set_sch(schedule);
}

void MutableVehicle::set_sch(const Schedule & schedule) {
  this->schedule_ = schedule;
}

void MutableVehicle::set_nnd(const DistInt & sync_nnd) {
  this->next_node_distance_ = sync_nnd;
}

void MutableVehicle::set_lvn(const RteIdx & lvn) {
  this->idx_last_visited_node_ = lvn;
}

void MutableVehicle::reset_lvn()    { this->idx_last_visited_node_ = 0; }
void MutableVehicle::incr_queued()  { this->queued_++; }  // when is "queued" used??
void MutableVehicle::decr_queued()  { this->queued_--; }


/* ProblemSet ----------------------------------------------------------------*/
ProblemSet::ProblemSet() {}
std::string & ProblemSet::name()            { return name_; }
std::string & ProblemSet::road_network()    { return road_network_; }

void ProblemSet::set_trips(const std::unordered_map<ErlyTime, vec_t<Trip>> & trips) {
  trips_ = trips;
}

const std::unordered_map<ErlyTime,vec_t<Trip>> & ProblemSet::trips() const {
  return trips_;
}


/*****************************************************************************/
std::ostream& operator<<(std::ostream& os, const Wayp& wp) {
  return os << "(" << wp.first << "," << wp.second << ")";
}

std::ostream& operator<<(std::ostream& os, const vec_t<Wayp>& route) {
  std::ostringstream oss;
  for (size_t i = 0; i < route.size(); ++i)
    oss << route.at(i) << (i == route.size() - 1 ? "" : " ");
  return os << oss.str();
}

std::ostream& operator<<(std::ostream& os, const Route& route) {
  return os << route.data();
}

std::ostream& operator<<(std::ostream& os, const Stop& sp) {
  return os << "(" << sp.owner() << "," << sp.loc() << "," << (int)sp.type() << ","
            << sp.early() << "," << sp.late() << "," << sp.visitedAt() << ")";
}

std::ostream& operator<<(std::ostream& os, const vec_t<Stop>& sched) {
  std::ostringstream oss;
  for (size_t i = 0; i < sched.size(); ++i)
    oss << sched.at(i) << (i == sched.size() - 1 ? "" : " ");
  return os << oss.str();
}

std::ostream& operator<<(std::ostream& os, const Schedule& sched) {
  return os << sched.data();
}

} // namespace cargo

