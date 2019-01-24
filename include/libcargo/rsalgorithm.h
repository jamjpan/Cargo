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
        std::string & name();        // e.g. "greedy_insertion"
        int         & batch_time();  // (set to 1 for streaming)
  const bool        & done()                    const;  // true if done
  const int         & matches()                 const;  // # matches
  const int         & rejected()                const;  // # rejected due to out of sync
  const float       & avg_handle_customer_dur() const;
  const float       & avg_handle_vehicle_dur()  const;
  const float       & avg_match_dur()           const;
  const float       & avg_listen_dur()          const;
  const float       & avg_num_cust_per_batch()  const;
  const float       & avg_num_vehl_per_batch()  const;
        void          kill();        // set done_ to true

        void          select_waiting_customers(bool skip_assigned = true, bool skip_delayed = true); // populate customers_
        void          select_matchable_vehicles(); // populate vehicles_

  vec_t<Vehicle>    & vehicles();           // return vehicles_
  vec_t<Customer>   & customers();          // return customers_
  vec_t<Vehicle>      get_all_vehicles();   // populate & return ALL vehicles
  vec_t<Customer>     get_all_customers();  // populate & return ALL customers
        bool          delay(const CustId &); // true if customer under delay
        bool          timeout(const tick_t &); // true if tick_t > timeout_

  /* Control */
        void          pause();              // pauses until user input
        void          pause(const int &);   // pauses for int seconds

  /* Commit to db */
  bool assign(
    const vec_t<CustId> &,                  // custs to add
    const vec_t<CustId> &,                  // custs to del
    const vec_t<Wayp>   &,                  // new route
    const vec_t<Stop>   &,                  // new schedule
          MutableVehicle &);                // vehicle to assign to
//        bool strict = false);             // set if do not want re-routing

  bool assign_or_delay(
    const vec_t<CustId> &,
    const vec_t<CustId> &,
    const vec_t<Wayp>   &,
    const vec_t<Stop>   &,
          MutableVehicle &);
//        bool strict = false);

  void beg_delay(const CustId &);           // begin delaying a customer
  void end_delay(const CustId &);           // end delaying a customer

  Message print;                            // print stream

  void print_statistics();                  // print statistics
  void print_rte(const vec_t<Wayp> &);
  void print_sch(const vec_t<Stop> &);

 protected:
  int   nmat_;                              // number matched
  int   nrej_;                              // number rejectd
  float avg_handle_customer_dur_;
  float avg_handle_vehicle_dur_;
  float avg_match_dur_;
  float avg_listen_dur_;
  float avg_num_cust_per_batch_;
  float avg_num_vehl_per_batch_;

  vec_t<int> dur_handle_customer_;          // running times handle_customer()
  vec_t<int> dur_handle_vehicle_;           // running times handle_vehicle()
  vec_t<int> dur_match_;                    // running times match()
  vec_t<int> dur_listen_;                   // running times listen()

  vec_t<int> n_cust_per_batch_;
  vec_t<int> n_vehl_per_batch_;

  dict<CustId, SimlTime> delay_;

  int retry_;                               // delay interval
  int timeout_;                             // timeout limit (ms)

  tick_t t_handle_customer_0, t_handle_customer_1;
  tick_t t_handle_vehicle_0,  t_handle_vehicle_1;
  tick_t t_match_0,           t_match_1;
  tick_t t_listen_0,          t_listen_1;

 private:
  std::string name_;                        // get with name()
  bool done_;                               // get with done(), set with kill()
  int batch_time_;                          // get/set with batch_time()
  int nsize_;                               // number of customers per batch

  vec_t<Customer> customers_;               // get with customers()
  vec_t<Vehicle>  vehicles_;                // get with vehicles()

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

  int duration(const tick_t& t_0, const tick_t& t_1) {
    return std::round(dur_milli(t_1 - t_0).count());
  }

  SyncResult sync(                          // internal function
    const vec_t<Wayp>   & new_rte,
    const vec_t<Wayp>   & cur_rte,
    const RteIdx              & idx_lvn,
    const vec_t<Stop>   & new_sch,
    const vec_t<Stop>   & cur_sch,
    const vec_t<CustId> & cadd,
    const vec_t<CustId> & cdel,
          vec_t<Wayp>   & out_rte,
          vec_t<Stop>   & out_sch);
};

}  // namespace cargo

#endif  // CARGO_INCLUDE_LIBCARGO_RSALGORITHM_H_

