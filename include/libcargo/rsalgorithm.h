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

class RSAlgorithm {
 public:
  RSAlgorithm(const std::string& name = "noname", bool fifo = false);
  ~RSAlgorithm();

  /* Overrideables */
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

  /* Setters/getters */
  const std::string & name()        const;  // e.g. "greedy_insertion"
  const bool        & done()        const;  // true if done
  const int         & matches()     const;  // # matches
  const int         & rejected()    const;  // # rejected due to out of sync
        int         & batch_time();         // (set to 1 for streaming)
        void          kill();               // set done_ to true
        float         avg_cust_ht();        // return avg. cust handling time

  void select_matchable_vehicles();         // populate vehicles_
  void select_waiting_customers();          // populate customers_
  std::vector<Vehicle>& vehicles();         // return vehicles_
  std::vector<Customer>& customers();       // return customers_
  std::vector<Vehicle> get_all_vehicles();  // populate & return ALL vehicles
  std::vector<Customer> get_all_customers();// populate & return ALL customers
  bool delay(const CustId &);               // true if customer under delay
  bool timeout(tick_t &);                   // true if tick_t > timeout_

  /* Commit to db */
  bool assign(
    const std::vector<CustId>&,             // custs to add
    const std::vector<CustId>&,             // custs to del
    const std::vector<Wayp>  &,             // new route
    const std::vector<Stop>  &,             // new schedule
          MutableVehicle&,                  // vehicle to assign to
          bool strict = false);             // set if do not want re-routing

  void beg_delay(const CustId &);           // begin delaying a customer
  void end_delay(const CustId &);           // end delaying a customer

  void beg_ht();                            // begin measure handling time
  void end_ht();                            // end measure handling time

  Message print;                            // print stream

 protected:
  int nmat_;                                // number matched
  int nrej_;                                // number rejectd

  std::vector<float> handling_times_;       // handling times container

  std::unordered_map<CustId, SimlTime>      // delays container;
    delay_;

  int retry_;                               // delay interval
  int timeout_;                             // timeout limit (ms)
  tick_t batch_0;                           // batch time tick
  tick_t batch_1;
  tick_t ht_0;                              // handling time tick
  tick_t ht_1;

 private:
  std::string name_;                        // get with name()
  bool done_;                               // get with done(), set with kill()
  int batch_time_;                          // get/set with batch_time()

  std::vector<Customer> customers_;         // get with customers()
  std::vector<Vehicle>  vehicles_;          // get with vehicles()

  SqliteReturnCode rc;
  sqlite3_stmt* ssr_stmt;                   // select route
  sqlite3_stmt* sss_stmt;                   // select schedule
  sqlite3_stmt* uro_stmt;                   // update route
  sqlite3_stmt* sch_stmt;                   // update sched
  sqlite3_stmt* qud_stmt;                   // increase queued
  sqlite3_stmt* com_stmt;                   // assign cust to veh
  sqlite3_stmt* smv_stmt;                   // select matchable vehicles
  sqlite3_stmt* sav_stmt;                   // select all vehicles
  sqlite3_stmt* swc_stmt;                   // select waiting customers
  sqlite3_stmt* sac_stmt;                   // select all customers
  sqlite3_stmt* svs_stmt;                   // select vehicle status
  sqlite3_stmt* sov_stmt;                   // select one vehicle
  sqlite3_stmt* sva_stmt;                   // select stop visitedAt

  typedef enum {                            // used interally for sync()
    SUCCESS,
    INVALID_VEHICLE,
    CURLOC_MISMATCH,
    PREFIX_MISMATCH,
    CADD_SYNC_FAIL,
    CDEL_SYNC_FAIL
  } SyncResult;

  SyncResult sync(                          // internal function
    const std::vector<Wayp>   & new_rte,
    const std::vector<Wayp>   & cur_rte,
    const RteIdx              & idx_lvn,
    const std::vector<Stop>   & new_sch,
    const std::vector<Stop>   & cur_sch,
    const std::vector<CustId> & cadd,
    const std::vector<CustId> & cdel,
          std::vector<Wayp>   & out_rte,
          std::vector<Stop>   & out_sch);
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

