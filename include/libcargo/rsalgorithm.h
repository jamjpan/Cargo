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
  RSAlgorithm(const std::string&);
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

  // Customers and vehicles are refreshed whenever listen() is called
  std::vector<Customer> & customers();
  std::vector<Vehicle>  & vehicles();

  // Write assignment to the db
  bool commit(
          const std::vector<Customer>&, // custs
          const Vehicle&,               // vehicle
          const std::vector<Wayp>&,     // new route
          const std::vector<Stop>&,     // new schedule
                std::vector<Wayp>&,     // out (after sync) route
                std::vector<Stop>&,     // out (after sync) schedule
                DistInt&);              // out (after sync) nnd

  bool commit( // use MutableVehicle
          const std::vector<Customer>&,
          const std::shared_ptr<MutableVehicle>&,
          const std::vector<Wayp>&,     // new route
          const std::vector<Stop>&,     // new schedule
                std::vector<Wayp>&,     // out route
                std::vector<Stop>&,     // out schedule
                DistInt&);              // out nnd

  // Non-outputting
  bool commit(
          const std::vector<Customer>&,
          const Vehicle&,
          const std::vector<Wayp>&,
          const std::vector<Stop>&);

  bool commit( // use MutableVehicle
          const std::vector<Customer>&,
          const std::shared_ptr<MutableVehicle>&,
          const std::vector<Wayp>&,
          const std::vector<Stop>&);

  Message print_out;
  Message print_info;
  Message print_warning;
  Message print_error;
  Message print_success;

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
  sqlite3_stmt* swc_stmt;  // select waiting customers

  void select_matchable_vehicles();
  void select_waiting_customers();

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

