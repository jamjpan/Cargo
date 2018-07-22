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
#ifndef CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_
#define CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "classes.h"
#include "message.h"
#include "types.h"

#include "../sqlite3/sqlite3.h"

namespace cargo {

// The abstract interface for ridesharing algorithms. Users can implement the
// handle_customer(), handle_vehicle(), match(), end(), and listen() methods.
//
// Only listen() has a default behavior. The behavior is to select all active
// vehicles into vehicles_ (retrievable with vehicles()), select all waiting
// customers into waiting_customers_ (retrievable with waiting_customers()),
// then sleep for batch_time_ (settable with batch_time()). The method is
// called continuously inside Cargo::step().
class RSAlgorithm {
 public:
  // Pass along a name string to your RSAlgorithm
  RSAlgorithm(const std::string& name = "noname", bool fifo = false);
  ~RSAlgorithm();

  virtual void handle_customer(const Customer&);
  virtual void handle_vehicle(const Vehicle&);
  virtual void match();
  virtual void end();
  virtual void listen();

  const std::string & name() const;
  const bool        & done() const;
        int         & batch_time();  // <-- set to 0 for streaming
        void          kill();        // <-- sets done_ to true

  // Call these to populate customers_ and vehicles_
  void select_matchable_vehicles();
  void select_waiting_customers();

  // Call these to retrieve customers_ and vehicles_
  // Customers and vehicles are refreshed whenever listen() is called
  std::vector<Customer> & customers();
  std::vector<Vehicle>  & vehicles();

  // Call these to return all customer/vehicle objects.
  std::vector<Customer> get_all_customers();
  std::vector<Vehicle> get_all_vehicles();

  /* Function assign(...) will try to add customers (param1) into vehicle (param3)
   * using the route and schedule (params4,5). If the route cannot be synchronized,
   * then the function will attempt to compute a new route using the vehicle's
   * current position going through the schedule (param5). If this new route
   * meets constraints, then the assignment is accepted. */
  bool assign(
              const std::vector<Customer>&, // custs to add
              const std::vector<CustId>&,   // custs to del
                    MutableVehicle&,        // vehicle to assign to
                    bool strict = false);

  bool assign_strict(
              const std::vector<Customer>&, // custs to add
              const std::vector<CustId>&,   // custs to del
                    MutableVehicle&);       // vehicle to assign to

  bool assign_test(
              const std::vector<Customer>&, // custs to add
              const std::vector<CustId>&,   // custs to del
                    MutableVehicle&,        // vehicle to assign to
                    bool strict = false);

  Message print;

 private:
  std::string name_;
  bool done_;
  int batch_time_;  // seconds

  std::vector<Customer> customers_;
  std::vector<Vehicle>  vehicles_;

  SqliteReturnCode rc;
  sqlite3_stmt* ssr_stmt;  // select route
  sqlite3_stmt* sss_stmt;  // select schedule
  sqlite3_stmt* uro_stmt;  // update route
  sqlite3_stmt* sch_stmt;  // update sched
  sqlite3_stmt* qud_stmt;  // increase queued
  sqlite3_stmt* com_stmt;  // assign cust to veh
  sqlite3_stmt* smv_stmt;  // select matchable vehicles
  sqlite3_stmt* sav_stmt;  // select all vehicles
  sqlite3_stmt* swc_stmt;  // select waiting customers
  sqlite3_stmt* sac_stmt;  // select all customers

  typedef enum { SUCCESS, CURLOC_MISMATCH, PREFIX_MISMATCH, CADD_SYNC_FAIL, CDEL_SYNC_FAIL }
  SyncResult;

  SyncResult sync(const std::vector<Wayp>     & new_rte,
                  const std::vector<Wayp>     & cur_rte,
                  const RteIdx                & idx_lvn,
                  const std::vector<Stop>     & new_sch,
                  const std::vector<Stop>     & cur_sch,
                  const std::vector<CustId>   & cadd,
                  const std::vector<CustId>   & cdel,
                        std::vector<Wayp>     & out_rte,
                        std::vector<Stop>     & out_sch);


  /* Returns false if sync is impossible */
  bool sync_route(const std::vector<Wayp> &,     // new route
                  const std::vector<Wayp> &,     // current route
                  const RteIdx &,                // current last-visited-node idx
                  const std::vector<Customer> &, // new customers in the route
                        std::vector<Wayp> &);    // synchronized output

  /* Removes from new schedule any stops not in current schedule or custs;
   * Sets first stop in new schedule to be the same as first stop in cur sch */
  bool sync_schedule(const std::vector<Stop> &,     // new schedule
                     const std::vector<Stop> &,     // current schedule
                     const std::vector<Wayp> &,     // synced route
                     const std::vector<Customer> &, // new customers in the sch
                           std::vector<Stop> &);    // synchronized output
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

