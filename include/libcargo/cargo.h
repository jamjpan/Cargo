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
#ifndef CARGO_INCLUDE_LIBCARGO_CARGO_H_
#define CARGO_INCLUDE_LIBCARGO_CARGO_H_
#include <fstream>
#include <map>
#include <mutex>
#include <random>

#include "classes.h"
#include "file.h"
#include "functions.h"
#include "message.h"
#include "options.h"
#include "rsalgorithm.h"
#include "types.h"

#include "../gtree/gtree.h"
#include "../lrucache/lrucache.hpp"
#include "../sqlite3/sqlite3.h"

namespace cargo {

class Cargo {
 public:
  Cargo();
  Cargo(const Options &);
  ~Cargo();

  /* Accessors */
  const std::string    & name();
  const std::string    & road_network();
  static DistInt         edgew(const NodeId& u, const NodeId& v) {
                                                     return edges_.at(u).at(v); }
  static Point           node2pt(const NodeId& i)  { return nodes_.at(i); }
  static DistInt         basecost(const TripId& i) { return trip_costs_.at(i); }
  static Customer        basecust(const TripId& i) { return customers_.at(i); }  // INITIAL customer (never gets updated)
  static BoundingBox     bbox()                    { return bbox_; }
  static Speed         & vspeed()                  { return speed_; }
  static SimlTime        now()                     { return t_; }
  static GTree::G_Tree & gtree()                   { return gtree_; }
  static sqlite3       * db()                      { return db_; }
  static bool          & paused()                  { return paused_; }
  static int           & count_sp()                { return count_sp_; }

  void start();                             // start simulation
  void start(RSAlgorithm &);                // start simulation
  int step(int &);                          // move the vehicles

  static vec_t<NodeId>                      // get from spcache
  spget(const NodeId& u, const NodeId& v) {
    return spcache_.get(std::to_string(u)+"|"+std::to_string(v));
  }

  static void spput(                        // put into spcache
  const NodeId& u, const NodeId& v, vec_t<NodeId>& path) {
    spcache_.put(std::to_string(u)+"|"+std::to_string(v), path);
  }

  static bool                               // check if exists
  spexist(const NodeId& u, const NodeId& v) {
    return spcache_.exists(std::to_string(u)+"|"+std::to_string(v));
  }

  static DistInt
  scget(const NodeId& u, const NodeId& v) {
    return sccache_.get(std::to_string(u)+"|"+std::to_string(v));
  }

  static void scput(
  const NodeId& u, const NodeId& v, const DistInt& cost) {
    sccache_.put(std::to_string(u)+"|"+std::to_string(v), cost);
  }

  static bool
  scexist(const NodeId& u, const NodeId& v) {
    return sccache_.exists(std::to_string(u)+"|"+std::to_string(v));
  }

  static std::mutex dbmx;                   // protect the db
  static std::mutex spmx;                   // protect the shortest-paths cache
  static std::mutex scmx;
  static std::mutex ofmx;                   // static mode mutex
  static std::mutex pause_mx;
  static std::condition_variable pause_cv;
  static bool static_mode;                  // static mode
  static bool strict_mode;

 private:
  Message print;

  ProblemSet probset_;

  SimlTime tmin_;                           // max trip.early
  SimlTime tmax_;                           // max vehicle.late
  SimlTime matp_;                           // matching pd. (customer timeout)

  bool full_sim_;                           // set true for full simulation

  size_t total_vehicles_;
  size_t total_customers_;
  size_t active_vehicles_;
  size_t base_cost_;
  int sleep_interval_;                      // 1 sec/time_multiplier

  /* Global vars */
  static KVNodes nodes_;                    // nodes_[u] = Point
  static KVEdges edges_;                    // edges_[u][v] = w
  static dict<TripId, Customer> customers_;
  static BoundingBox bbox_;
  static GTree::G_Tree gtree_;
  static sqlite3* db_;
  static Speed speed_;
  static SimlTime t_;                       // current sim time
  static bool paused_;
  static dict<TripId, DistInt> trip_costs_;
  static cache::lru_cache<std::string, vec_t<NodeId>> spcache_;
  static cache::lru_cache<std::string, DistInt>       sccache_;
  static int count_sp_;                     // number of sp computations

  Speed original_speed_; // hack

  /* Solution file */
  void total_solution_cost();
  long int total_traveled_;
  long int total_penalty_;
  SimlDur avg_pickup_delay();               // (time-to-pickup) - cust.early()
  SimlDur avg_trip_delay();                 // (dropoff - pickup) - base cost

  /* Logger containers */
  std::map<VehlId, vec_t<std::pair<NodeId, DistInt>>>  log_v_;
  vec_t<CustId> log_p_, log_d_, log_t_;
  vec_t<VehlId> log_a_, log_l_;

  /* Save Database */
  Filepath database_file_;

  /* SQL statements */
  SqliteReturnCode rc;
  SqliteErrorMessage err;
  sqlite3_stmt* tim_stmt;                   // timeout customers
  sqlite3_stmt* sac_stmt;                   // select all customers
  sqlite3_stmt* sar_stmt;                   // select all routes
  sqlite3_stmt* ssv_stmt;                   // select step vehicles
  sqlite3_stmt* ucs_stmt;                   // update cust status
  sqlite3_stmt* uro_stmt;                   // update route, lvn, nnd
  sqlite3_stmt* sch_stmt;                   // update schedule
  sqlite3_stmt* dav_stmt;                   // deactivate vehicle
  sqlite3_stmt* pup_stmt;                   // pickup
  sqlite3_stmt* drp_stmt;                   // dropoff
  sqlite3_stmt* vis_stmt;                   // visitedAt
  sqlite3_stmt* lvn_stmt;                   // last-visited node
  sqlite3_stmt* nnd_stmt;                   // nearest-node dist
  sqlite3_stmt* stc_stmt;                   // select timed-out customers
  sqlite3_stmt* mov_stmt;                   // bulk-move vehicles
  sqlite3_stmt* usc_stmt;                   // update schedule, lvn, nnd
  sqlite3_stmt* cwc_stmt;                   // count waiting customers

  void construct(const Options &);
  void initialize(const Options &);

  std::mt19937 rng;
  NodeId random_node();
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_CARGO_H_
