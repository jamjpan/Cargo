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
#include <unordered_map>

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

/* Cargo is a simulator and requests generator. Its two functionalities are to
 * simulate the movement of vehicles, and to generate customer requests and
 * vehicles in "real time". These are generated from a "problem instance", a
 * special text file that lists customers, vehicles, and their properties.
 *
 * Vehicle movement is simulated based on vehicle speed, a road network,
 * vehicle routes, and time. Speed determines how much distance is covered at
 * every simulation step. Routes are sequences of nodes through the road
 * network. By default, all vehicles travel along the shortest path from their
 * origin to their destination.
 *
 * A vehicle with a destination of -1 is a "permanent taxi" These will coast
 * forever, until all customers have appeared and been dropped off or timed out.
 * The coast strategy is to move to a random node; other coast strategies might
 * be implemented in the future.
 *
 * Cargo will poll an internal sqlite3 database for new vehicle routes. These
 * routes can be updated through an RSAlgorithm. */
namespace cargo {

class Cargo {
 public:
  Cargo(const Options &);
  ~Cargo();
  const std::string & name();          // e.g. rs-lg-5
  const std::string & road_network();  // e.g. mny, cd1, bj5

  /* Starts a dynamic simulation */
  void start();
  void start(RSAlgorithm &);

  /* Returns number of stepped vehicles.
   * Outputs number of deactivated vehicles (param1) */
  int step(int &);

  /* Accessors */
  static DistInt         edgew(const NodeId& u, const NodeId& v) { return edges_.at(u).at(v); }
  static Point           node2pt(const NodeId& i)  { return nodes_.at(i); }
  static DistInt         basecost(const TripId& i) { return trip_costs_.at(i); }
  static BoundingBox     bbox()                    { return bbox_; }
  static Speed         & vspeed()                  { return speed_; }
  static SimlTime        now()                     { return t_; }
  static GTree::G_Tree & gtree()                   { return gtree_; }
  static sqlite3       * db()                      { return db_; }

  /* Access the shortest-paths cache */
  static std::vector<NodeId> spget(const NodeId& u, const NodeId& v) {
    std::string k = std::to_string(u)+"|"+std::to_string(v);
    return spcache_.get(k);
  }

  static void spput(const NodeId& u, const NodeId& v, std::vector<NodeId>& path) {
    std::string k = std::to_string(u)+"|"+std::to_string(v);
    spcache_.put(k, path);
  }

  static bool spexist(const NodeId& u, const NodeId& v) {
    std::string k = std::to_string(u)+"|"+std::to_string(v);
    return spcache_.exists(k);
  }

  static std::mutex dbmx;  // protect the db
  static std::mutex spmx;  // protect the shortest-paths cache
  static std::mutex ofmx;  // offline mutex
  static bool OFFLINE; // offline mode

 private:
  Message print;

  ProblemSet probset_;

  SimlTime tmin_;  // max trip.early
  SimlTime tmax_;  // max vehicle.late
  SimlTime matp_;  // matching pd. (customer timeout)

  bool full_sim_;

  size_t total_vehicles_;
  size_t total_customers_;
  size_t active_vehicles_;
  size_t base_cost_;      // total base cost
  int sleep_interval_;    // 1 sec/time_multiplier

  /* Global vars */
  static KVNodes nodes_;  // nodes_[u] = Point
  static KVEdges edges_;  // edges_[u][v] = w
  static BoundingBox bbox_;
  static GTree::G_Tree gtree_;
  static sqlite3* db_;
  static Speed speed_;
  static SimlTime t_;     // current sim time
  static std::unordered_map<TripId, DistInt> trip_costs_; // indiv. base costs
  /* Shortest-paths cache
   * (the key is a linear combination of two node IDs for from and to) */
  static cache::lru_cache<std::string, std::vector<NodeId>> spcache_;

  /* Solution file */
  Filepath solution_file_;
  Filepath dataout_file_;
  long int total_route_cost();
  SimlDur avg_pickup_delay(); // (time-to-pickup) - cust.early()
  SimlDur avg_trip_delay();   // (dropoff - pickup) - base cost

  /* Logger containers */
  std::map<VehlId, NodeId>  log_v_;
  std::vector<CustId>       log_p_, log_d_, log_t_;
  std::vector<VehlId>       log_a_;

  /* Save Database */
  Filepath database_file_;

  /* SQL statements */
  SqliteReturnCode rc;
  SqliteErrorMessage err;
  sqlite3_stmt* tim_stmt;  // timeout customers
  sqlite3_stmt* sac_stmt;  // select all customers
  sqlite3_stmt* sar_stmt;  // select all routes
  sqlite3_stmt* ssv_stmt;  // select step vehicles
  sqlite3_stmt* ucs_stmt;  // update cust status
  sqlite3_stmt* uro_stmt;  // update route, lvn, nnd
  sqlite3_stmt* sch_stmt;  // update schedule
  sqlite3_stmt* dav_stmt;  // deactivate vehicle
  sqlite3_stmt* pup_stmt;  // pickup
  sqlite3_stmt* drp_stmt;  // dropoff
  sqlite3_stmt* vis_stmt;  // visitedAt
  sqlite3_stmt* lvn_stmt;  // last-visited node
  sqlite3_stmt* nnd_stmt;  // nearest-node dist
  sqlite3_stmt* stc_stmt;  // select timed-out customers
  sqlite3_stmt* mov_stmt;  // bulk-move vehicles
  sqlite3_stmt* usc_stmt;  // update schedule, lvn, nnd

  void initialize(const Options &);

  std::mt19937 rng;
  NodeId random_node();
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_CARGO_H_
