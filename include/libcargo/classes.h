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

#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "types.h"

#include "../gtree/gtree.h"

/* -------
 * SUMMARY
 * -------
 * This file contains definitions for major classes used in Cargo ridesharing
 * simulator. These classes are:
 *   - Stop
 *   - Schedule
 *   - Route
 *   - Trip
 *   - Customer
 *   - Vehicle
 *   - MutableVehicle
 *   - ProblemSet
 * At the bottom are hash functions and << overloads
 */

namespace cargo {

/* Stop is an origin or destination belonging to a customer or vehicle owner. */
class Stop {
 public:
  /* Constructor */
  Stop(
    TripId,            // param1: id of owner (VehlId or CustId)
    NodeId,            // param2: id of stop location node (NodeId)
    StopType,          // param3: type of stop (cust/vehl origin or destination)
    ErlyTime,          // param4: early time window bound (e_i)
    LateTime,          // param5: late time window bound (l_i)
    SimlTime v = -1    // param6: visited time (-1 means not visited)
  );
  const TripId   & owner()     const;  // return owner
  const NodeId   & loc()       const;  // return location
  const StopType & type()      const;  // return type
  const ErlyTime & early()     const;  // return early bound
  const LateTime & late()      const;  // return late bound
  const SimlTime & visitedAt() const;  // return visited time

  /* Equality comparator
   * Stop A and B are equal if owner and location are the same. */
  bool operator==(const Stop & rhs) const {
    return owner_ == rhs.owner_ && location_ == rhs.location_;
  }

 private:
  TripId   owner_;
  NodeId   location_;
  StopType type_;
  ErlyTime early_;
  LateTime late_;
  SimlTime visitedAt_;
};

/* Schedule is sequence of Stops for Vehicle to follow. ----------------------*/
class Schedule {
 public:
  /* Constructors */
  Schedule() = default;
  Schedule(
    VehlId,      // param1: id of owner
    vec_t<Stop>  // param2: raw sequence of Stops
  );
  const VehlId       & owner()    const;  // return owner
  const vec_t<Stop>  & data()     const;  // return raw sequence of Stops
  const Stop         & at(SchIdx) const;  // return particular Stop
  const Stop         & front()    const;  // return first Stop
  const Stop         & back()     const;  // return last Stop
        size_t         size()     const;  // return Schedule size
        void           print()    const;  // print to standard out

 private:
  VehlId owner_;
  vec_t<Stop> data_;
};

/* Route is sequence of Waypoints for Vehicle to follow. ---------------------*/
class Route {
 public:
  /* Constructors */
  Route() = default;
  Route(
    VehlId,      // param1: id of owner
    vec_t<Wayp>  // param2: raw sequence of Waypoints
  );
  const VehlId       & owner()         const;  // return owner
  const vec_t<Wayp>  & data()          const;  // return raw sequence
  const NodeId       & node_at(RteIdx) const;  // return particular node
  const DistInt      & dist_at(RteIdx) const;  // return distance to
  const DistInt      & cost()          const;  // return total distance
  const Wayp         & at(RteIdx)      const;  // return particular Wayp
        size_t         size()          const;  // return Route size
        void           print()         const;  // print to standard out

 private:
  VehlId owner_;
  vec_t<Wayp> data_;
};

/* Base class for Customers and Vehicles. ------------------------------------*/
class Trip {
 public:
  /* Constructors */
  Trip() = default;
  Trip(
    TripId,    // id of the trip (CustId or VehlId)
    OrigId,    // id of origin (NodeId)
    DestId,    // id of destination (NodeId)
    ErlyTime,  // early time window bound (e_i)
    LateTime,  // late time window bound (l_i)
    Load       // load (negative indicates vehicle capacity)
  );
  const TripId       & id()    const;  // return id
  const OrigId       & orig()  const;  // return origin id
  const DestId       & dest()  const;  // return destination id
  const ErlyTime     & early() const;  // return early bound
  const LateTime     & late()  const;  // return late bound
        Load           load()  const;  // return load (NOT current load)

 protected:
  TripId id_;
  OrigId orig_;
  DestId dest_;
  ErlyTime early_;
  LateTime late_;
  Load load_;
};

/* Ridesharing customer. -----------------------------------------------------*/
class Customer : public Trip {
 public:
  /* Constructor */
  Customer() = default;
  Customer(
    CustId,        // id of the Customer
    OrigId,        // id of origin (NodeId)
    DestId,        // id of destination (NodeId)
    ErlyTime,      // early time window bound (e_i)
    LateTime,      // late time window bound (l_i)
    Load,          // load (always positive for customer)
    CustStatus,    // status (see types.h)
    VehlId a = -1  // assigned vehicle (-1 means not assigned)
  );
  const CustStatus & status()     const;  // return status (see types.h)
  const VehlId     & assignedTo() const;  // return assigned vehicle
        bool         assigned()   const;  // return TRUE if assigned
        void         print()      const;  // print properties to standard out

  bool operator==(const Customer & rhs) const { return id_ == rhs.id_; }
  bool operator<(const Customer & rhs) const { return id_ < rhs.id_; }

 private:
  CustStatus status_;
  VehlId assignedTo_;
};

/* Ridesharing Vehicle. ------------------------------------------------------*/
class Vehicle : public Trip {
 public:
  /* Copy constructor */
  Vehicle(const Vehicle &) = default;
  /* Constructors */
  Vehicle() = default;
  Vehicle( /* Basic constructor */
    VehlId,          // id of the Vehicle
    OrigId,          // id of origin (NodeId)
    DestId,          // id of destination (NodeId)
    ErlyTime,        // early time window bound (e_i)
    LateTime,        // late time window bound (l_i)
    Load,            // load (always negative for vehicle)
    GTree::G_Tree &  // Specific G-tree to use for construction
  );
  Vehicle( /* Fine-detail constructor */
    VehlId,
    OrigId,
    DestId,
    ErlyTime,
    LateTime,
    Load,
    Load,       // num queued (customers assigned but not yet picked up)
    DistInt,    // distance to next node in Route
    Route,      // vehicle's route
    Schedule,   // vehicle's schedule
    RteIdx,     // index of current node in Route
    VehlStatus  // status (see types.h)
  );
  const DistInt    & next_node_distance()    const;  // return next node dist.
  const Route      & route()                 const;  // return Route
  const Schedule   & schedule()              const;  // return Schedule
  const RteIdx     & idx_last_visited_node() const;  // index to last-visit node
  const NodeId     & last_visited_node()     const;  // return last-visit node
  const VehlStatus & status()                const;  // return status
        DistInt      traveled()              const;  // return distance traveled
        DistInt      remaining()             const;  // return distance remaining
        Load         queued()                const;  // return number queued
        Load         capacity()              const;  // return REMAINING capacity
  void print()                               const;  // print to standard out

  bool operator==(const Vehicle & rhs) const { return id_ == rhs.id_; }
  bool operator<(const Vehicle & rhs) const { return id_ < rhs.id_; }

 protected:
  DistInt next_node_distance_;
  Route route_;
  Schedule schedule_;
  RteIdx idx_last_visited_node_;
  Load queued_;
  VehlStatus status_;
};

/* Mutable Vehicle. ----------------------------------------------------------*/
class MutableVehicle : public Vehicle {
 public:
  /* Copy constructor
   * Create a MutableVehicle from an ordinary Vehicle */
  MutableVehicle(const Vehicle &);
  /* Constructors */
  MutableVehicle() = default;

  void set_rte(const vec_t<Wayp> &);  // set raw route
  void set_rte(const Route &);        // set Route
  void set_sch(const vec_t<Stop> &);  // set raw schedule
  void set_sch(const Schedule &);     // set Schedule
  void set_nnd(const DistInt &);      // set distance to next node
  void set_lvn(const RteIdx &);       // set last-visited node index
  void reset_lvn();                   // set last-visited node index to 0
  void incr_queued();                 // increase value of queued by 1
  void decr_queued();                 // decrease value of queued by 1

  bool operator==(const MutableVehicle & rhs) const { return id_ == rhs.id_; }
  bool operator<(const MutableVehicle & rhs) const { return id_ < rhs.id_; }
};
typedef std::shared_ptr<MutableVehicle> MutableVehicleSptr;

/* Ridesharing problem instance.
 * Corresponds to .instance file in data/benchmarks. -------------------------*/
class ProblemSet {
 public:
  /* Constructor */
  ProblemSet();

  const dict<ErlyTime, vec_t<Trip>> & trips() const;
    // return all trips

  std::string & name();          // get/set instance name
  std::string & road_network();  // get/set road network

  /* Trips are grouped by their early time (time of appearance) */
  void set_trips(const dict<ErlyTime, vec_t<Trip>> &);
    // store trips

 private:
  std::string name_;
  std::string road_network_;
  dict<SimlTime, vec_t<Trip>> trips_;
};

std::ostream& operator<<(std::ostream& os, const Wayp &);
std::ostream& operator<<(std::ostream& os, const vec_t<Wayp> &);
std::ostream& operator<<(std::ostream& os, const Route &);
std::ostream& operator<<(std::ostream& os, const Stop &);
std::ostream& operator<<(std::ostream& os, const vec_t<Stop> &);
std::ostream& operator<<(std::ostream& os, const Schedule &);

}  // namespace cargo


/*****************************************************************************/
namespace std {

/* Hash vehicles by their id */
template <> struct hash<cargo::Vehicle>  {
  std::size_t operator()(const cargo::Vehicle  & vehl) const
  { return std::hash<int>{}(vehl.id()); }
};

template <> struct hash<cargo::MutableVehicle>  {
  std::size_t operator()(const cargo::MutableVehicle  & vehl) const
  { return std::hash<int>{}(vehl.id()); }
};

/* Hash customers by their id */
template <> struct hash<cargo::Customer> {
  std::size_t operator()(const cargo::Customer & cust) const
  { return std::hash<int>{}(cust.id()); }
};

}  // namespace std

#endif  // CARGO_INCLUDE_LIBCARGO_CLASSES_H_

