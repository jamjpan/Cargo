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

typedef std::chrono::duration<double, std::milli> dur_milli;
typedef std::chrono::milliseconds milli;

// The class for ridesharing algorithms. Users can implement the
// handle_customer(), handle_vehicle(), match(), end(), and listen() methods.
//
// Only listen() has a default behavior. The behavior is to select all active
// vehicles into vehicles_ (retrievable with vehicles()), select all waiting
// customers into waiting_customers_ (retrievable with waiting_customers()),
// then sleep for batch_time_ (settable with batch_time()). The method is
// called continuously inside Cargo::start().
class RSAlgorithm {
 public:
  // Pass along a name string to RSAlgorithm
  // Set fifo if want RSAlgorithm messages in a separate stream (not stdout)
  RSAlgorithm(const std::string& name = "noname", bool fifo = false);
  ~RSAlgorithm();

  // Overrideables
  virtual void handle_customer(const Customer&);
  virtual void handle_vehicle(const Vehicle&);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

  // Setters/getters
  const std::string & name()        const;  // e.g. "greedy_insertion"
  const bool        & done()        const;  // true if done
  const int         & matches()     const;  // # matches
  const int         & rejected()    const;  // # rejected due to out of sync
        int         & batch_time();         // set to 1 for streaming
        void          kill();               // sets done_ to true
        float         avg_cust_ht();        // avg. cust handling time

  // Populates customers_ and vehicles_
  void select_matchable_vehicles();
  void select_waiting_customers();

  // Returns customers_ and vehicles_
  std::vector<Customer> & customers();
  std::vector<Vehicle>  & vehicles();

  // Return all customer/vehicle objects
  std::vector<Customer> get_all_customers();
  std::vector<Vehicle> get_all_vehicles();

  /* Function assign(...) will try to add customers (param1) into vehicle (param5)
   * using the route and schedule (params3,4). If the route cannot be synchronized,
   * then the function will attempt to compute a new route using the vehicle's
   * current position going through the schedule (param4). If this new route
   * meets constraints, then the assignment is accepted. */
  bool assign(const std::vector<CustId>&,  // custs to add
              const std::vector<CustId>&,  // custs to del
              const std::vector<Wayp>  &,  // new route
              const std::vector<Stop>  &,  // new schedule
                    MutableVehicle&,       // vehicle to assign to
                    bool strict = false);  // set if do not want re-routing

  /* Return true if customer delay is less than RETRY_ */
  bool delay(const CustId &);

  /* Begin/end delaying a customer */
  void beg_delay(const CustId &);
  void end_delay(const CustId &);

  /* Begin/end measuring handling time */
  void beg_ht();
  void end_ht();

  /* Timeout long-running executions (pass a start clock) */
  bool timeout(std::chrono::time_point<std::chrono::high_resolution_clock> &);

  Message print;

 protected:
  int nmat_;  // number matched
  int nrej_;  // number rejectd

  std::vector<float> handling_times_;

  /* If a customer doesn't get matched right away, put it here */
  std::unordered_map<CustId, SimlTime> delay_;

  int retry_;  // try again after RETRY secs
  int timeout_; // timeout long-running executions (millisecs)
  tick_t batch_0, batch_1;
  tick_t ht_0, ht_1;

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
  sqlite3_stmt* svs_stmt;  // select vehicle status
  sqlite3_stmt* sov_stmt;  // select one vehicle
  sqlite3_stmt* sva_stmt;  // select stop visitedAt

  typedef enum {
    SUCCESS,
    INVALID_VEHICLE,
    CURLOC_MISMATCH,
    PREFIX_MISMATCH,
    CADD_SYNC_FAIL,
    CDEL_SYNC_FAIL } SyncResult;

  SyncResult sync(const std::vector<Wayp>     & new_rte,
                  const std::vector<Wayp>     & cur_rte,
                  const RteIdx                & idx_lvn,
                  const std::vector<Stop>     & new_sch,
                  const std::vector<Stop>     & cur_sch,
                  const std::vector<CustId>   & cadd,
                  const std::vector<CustId>   & cdel,
                        std::vector<Wayp>     & out_rte,
                        std::vector<Stop>     & out_sch);
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

