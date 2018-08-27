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

#include "../gtree/gtree.h"

// Properties of these classes are immutable, and they are accesed via the
// "inspector method"; see https://isocpp.org/wiki/faq/const-correctness
namespace cargo {

// A stop is a customer (vehicle) origin or destination and correspond to
// Cordeau's "nodes".
class Stop {
 public:
  /* Constructor
   *   param1: id of the stop owner (VehlId or CustId)
   *   param2: id of the node (NodeId)
   *   param3: stop type
   *   param4: time window early
   *   param5: time window late
   *   param6: visited time (-1=unvisited) */
  Stop(TripId, NodeId, StopType, ErlyTime, LateTime, SimlTime v = -1);
  const TripId   & owner()     const;
  const NodeId   & loc()       const;
  const StopType & type()      const;
  const ErlyTime & early()     const;
  const LateTime & late()      const;
  const SimlTime & visitedAt() const;

  bool operator==(const Stop& rhs) const {
    return owner_ == rhs.owner_ && location_ == rhs.location_;
  }

 private:
  TripId owner_;
  NodeId location_;
  StopType type_;
  ErlyTime early_;
  LateTime late_;
  SimlTime visitedAt_;
};

// Schedules are meant to be attached to vehicles. If you need to manipulate a
// schedule, for example construct one, use std::vector<Stop>
class Schedule {
 public:
  Schedule() = default;
  Schedule(VehlId, std::vector<Stop>);
  const VehlId            & owner()    const;
  const std::vector<Stop> & data()     const;
  const Stop              & at(SchIdx) const;
  const Stop              & front()    const;
        size_t              size()     const;
        void                print()    const;

 private:
  VehlId owner_;
  std::vector<Stop> data_;
};

// Routes are meant to be attached to vehicles. If you need to manipulate a
// route, for example construct one from segments, use std::vector<Wayp>
class Route {
 public:
  Route() = default;
  Route(VehlId, std::vector<Wayp>);
  const VehlId            & owner()         const;
  const std::vector<Wayp> & data()          const;
  const NodeId            & node_at(RteIdx) const;
  const DistInt           & dist_at(RteIdx) const;
  const DistInt           & cost()          const;
  const Wayp              & at(RteIdx)      const;
        size_t              size()          const;
        void                print()         const;

 private:
  VehlId owner_;
  std::vector<Wayp> data_;
};

// A Trip is the base class for a customer or vehicle.
class Trip {
 public:
  /* Constructor
   *   param1: id
   *   param2: origin id
   *   param3: destination id
   *   param4: time window early
   *   param5: time window late
   *   param6: load */
  Trip() = default;
  Trip(TripId, OrigId, DestId, ErlyTime, LateTime, Load);
  const TripId            & id()    const;
  const OrigId            & orig()  const;
  const DestId            & dest()  const;
  const ErlyTime          & early() const;
  const LateTime          & late()  const;
        Load                load()  const;

 protected:
  TripId id_;
  OrigId orig_;
  DestId dest_;
  ErlyTime early_;
  LateTime late_;
  Load load_;
};

// A Customer represents a new ride request
class Customer : public Trip {
 public:
  /* Constructor
   *   param1: customer id
   *   param2: origin id
   *   param3: destination id
   *   param4: time window early
   *   param5: time window late
   *   param6: load
   *   param7: customer status
   *   param8: assigned to (-1=unassigned) */
  Customer(CustId, OrigId, DestId, ErlyTime, LateTime, Load, CustStatus, VehlId a = -1);
  const CustStatus & status()     const;
  const VehlId     & assignedTo() const;
        bool         assigned()   const;
        void         print()      const;

  bool operator==(const Customer& rhs) const { return id_ == rhs.id_; }

 private:
  CustStatus status_;
  VehlId assignedTo_;
};

// Immutable Vehicle class, use for reading and processing
class Vehicle : public Trip {
 public:
  Vehicle() = default;
  Vehicle(const Vehicle&) = default;
  /* Constructor 1
   *   param1: id
   *   param2: origin id
   *   param3: destination id
   *   param4: time window early
   *   param5: time window late
   *   param6: load
   *   param7: gtree for finding default route */
  Vehicle(VehlId, OrigId, DestId, ErlyTime, LateTime, Load, GTree::G_Tree&);
  /* Constructor 2
   *   param1-6: same as above
   *   param7:   queued (assigned but not yet picked up)
   *   param8:   next-node distance
   *   param9:   route
   *   param10:  schedule
   *   param11:  index of last visited node
   *   param12:  vehicle status */
  Vehicle(VehlId, OrigId, DestId, ErlyTime, LateTime, Load, Load,
          DistInt, Route, Schedule, RteIdx, VehlStatus);
  const DistInt    & next_node_distance()    const;
  const Route      & route()                 const;
  const Schedule   & schedule()              const;
  const RteIdx     & idx_last_visited_node() const;
  const NodeId     & last_visited_node()     const;
  const VehlStatus & status()                const;
        Load         queued()                const;
        Load         capacity()              const;
  void print()                               const;

  bool operator==(const Vehicle& rhs) const { return id_ == rhs.id_; }

 protected:
  DistInt next_node_distance_;
  Route route_;
  Schedule schedule_;
  RteIdx idx_last_visited_node_;
  Load queued_;
  VehlStatus status_;
};

// Mutable Vehicle class, use when local vehicle needs modification. You are
// responsible for synchronizing any MutableVehicles with the db.
class MutableVehicle : public Vehicle {
 public:
  MutableVehicle() = default;
  MutableVehicle(const Vehicle&);
  void set_rte(const std::vector<Wayp>&);
  void set_rte(const Route&);
  void set_sch(const std::vector<Stop>&);
  void set_sch(const Schedule&);
  void set_nnd(const DistInt&);
  void set_lvn(const RteIdx&);
  void reset_lvn();
  void incr_queued();
  void decr_queued();
};

// A problem is the set of trips keyed by their early time. When the
// simulator time reaches SimlTime, all the trips in the group are broadcasted.
class ProblemSet {
 public:
  ProblemSet();
  const std::unordered_map<ErlyTime, std::vector<Trip>>& trips() const;
  std::string& name();
  std::string& road_network();
  void set_trips(const std::unordered_map<ErlyTime, std::vector<Trip>>&);

 private:
  std::string name_;
  std::string road_network_;
  std::unordered_map<SimlTime, std::vector<Trip>> trips_;
};

}  // namespace cargo

/* For using Vehicle, Customer as custom keys in an unordered_map
 * (http://en.cppreference.com/w/cpp/container/unordered_map/unordered_map) */
namespace std {

template <> struct hash<cargo::Vehicle> {
  std::size_t operator()(const cargo::Vehicle& vehl) const {
    return std::hash<int>{}(vehl.id());
  }
};

template <> struct hash<cargo::Customer> {
  std::size_t operator()(const cargo::Customer& cust) const {
    return std::hash<int>{}(cust.id());
  }
};

}  // namespace std

#endif  // CARGO_INCLUDE_LIBCARGO_CLASSES_H_

