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
#include <algorithm> /* std::find, std::find_if */
#include <chrono>
#include <cmath>
#include <iterator> /* std::back_inserter */
#include <mutex>
#include <numeric>
#include <thread>

#include "libcargo/cargo.h"
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/dbsql.h"
#include "libcargo/file.h"
#include "libcargo/message.h"
#include "libcargo/rsalgorithm.h"
#include "libcargo/types.h"

namespace cargo {

RSAlgorithm::RSAlgorithm(const std::string& name, bool fifo)
    : print(name, fifo) {
  this->name_ = name;
  this->done_ = false;
  this->batch_time_ = 1;
  this->nmat_ = 0;
  this->nrej_ = 0;
  this->delay_ = {};
  this->retry_ = 0;
  this->timeout_ = 1;
  prepare_stmt(sql::ssr_stmt, &ssr_stmt);
  prepare_stmt(sql::sss_stmt, &sss_stmt);
  prepare_stmt(sql::uro_stmt, &uro_stmt);
  prepare_stmt(sql::sch_stmt, &sch_stmt);
  prepare_stmt(sql::qud_stmt, &qud_stmt);
  prepare_stmt(sql::com_stmt, &com_stmt);
  prepare_stmt(sql::smv_stmt, &smv_stmt);
  prepare_stmt(sql::sac_stmt, &sac_stmt);
  prepare_stmt(sql::sav_stmt, &sav_stmt);
  prepare_stmt(sql::svs_stmt, &svs_stmt);
  prepare_stmt(sql::swc_stmt, &swc_stmt);
  prepare_stmt(sql::sov_stmt, &sov_stmt);
  prepare_stmt(sql::sva_stmt, &sva_stmt);
}

RSAlgorithm::~RSAlgorithm() {
  sqlite3_finalize(ssr_stmt);
  sqlite3_finalize(sss_stmt);
  sqlite3_finalize(uro_stmt);
  sqlite3_finalize(sch_stmt);
  sqlite3_finalize(qud_stmt);
  sqlite3_finalize(com_stmt);
  sqlite3_finalize(smv_stmt);
  sqlite3_finalize(swc_stmt);
  sqlite3_finalize(sac_stmt);
  sqlite3_finalize(sav_stmt);
  sqlite3_finalize(svs_stmt);
  sqlite3_finalize(sov_stmt);
  sqlite3_finalize(sva_stmt);
}

const bool        & RSAlgorithm::done()                     const { return done_; }
const int         & RSAlgorithm::matches()                  const { return nmat_; }
const int         & RSAlgorithm::rejected()                 const { return nrej_; }
const float       & RSAlgorithm::avg_handle_customer_dur()  const { return avg_handle_customer_dur_; }
const float       & RSAlgorithm::avg_handle_vehicle_dur()   const { return avg_handle_vehicle_dur_; }
const float       & RSAlgorithm::avg_match_dur()            const { return avg_match_dur_; }
const float       & RSAlgorithm::avg_listen_dur()           const { return avg_listen_dur_; }
const float       & RSAlgorithm::avg_num_cust_per_batch()   const { return avg_num_cust_per_batch_; }
const float       & RSAlgorithm::avg_num_vehl_per_batch()   const { return avg_num_vehl_per_batch_; }
      std::string & RSAlgorithm::name()                           { return name_; }
      int         & RSAlgorithm::batch_time()                     { return batch_time_; }
      void          RSAlgorithm::kill()                           { done_ = true; }

void RSAlgorithm::pause() {
  Cargo::paused() = true;
  std::string cmd;
  std::cout << "Press ENTER "/*or type command (e.g. help)" */<< std::endl;
  std::cin.clear();
  std::getline(std::cin, cmd);
  // if (!cmd.empty()) {
  //   std::cout << "Got command " << cmd << std::endl;
  // } else {
  //   std::cout << "Continue" << std::endl;
  // }
}

void RSAlgorithm::pause(const int& t) {
  Cargo::paused() = true;
  std::this_thread::sleep_for(milli(t*1000));
}

vec_t<Customer> & RSAlgorithm::customers() { return customers_; }
vec_t<Vehicle>  & RSAlgorithm::vehicles()  { return vehicles_; }

bool RSAlgorithm::assign(
  const vec_t<CustId> & custs_to_add,
  const vec_t<CustId> & custs_to_del,
  const vec_t<Wayp>   & new_rte,
  const vec_t<Stop>   & new_sch,
        MutableVehicle      & vehl) {
//      bool                strict) {
  //if (custs_to_add.empty() && custs_to_del.empty())
  //  return true;
  bool strict = Cargo::strict_mode;
  std::lock_guard<std::mutex> dblock(Cargo::dbmx);

  /* Get current vehicle properties */
  sqlite3_clear_bindings(sov_stmt);
  sqlite3_reset(sov_stmt);
  sqlite3_bind_int(sov_stmt, 1, vehl.id());
  if (sqlite3_step(sov_stmt) != SQLITE_ROW) {
    vehl.print();
    throw std::runtime_error("sov_stmt returned no rows.");
  }

  /* Check status */
  if (static_cast<VehlStatus>(sqlite3_column_int(sov_stmt, 7))
          == VehlStatus::Arrived) {
    this->nrej_++;
    return false;
  }

  /* Get current capacity */
  const int curcap = sqlite3_column_int(sov_stmt, 5)*(-1);

  /* Get current schedule */
  const Stop* schbuf =
    static_cast<const Stop*>(sqlite3_column_blob(sov_stmt, 11));
  vec_t<Stop> cur_sch(
    schbuf, schbuf + sqlite3_column_bytes(sov_stmt, 11) / sizeof(Stop));

  /* Get current route */
  const Wayp* rtebuf
    = static_cast<const Wayp*>(sqlite3_column_blob(sov_stmt, 8));
  vec_t<Wayp> cur_rte(
    rtebuf, rtebuf + sqlite3_column_bytes(sov_stmt, 8) / sizeof(Wayp));
  RteIdx  cur_lvn = sqlite3_column_int(sov_stmt, 9);
  DistInt cur_nnd = sqlite3_column_int(sov_stmt, 10);

  if (sqlite3_step(sov_stmt) != SQLITE_DONE) {
    vehl.print();
    throw std::runtime_error("sov_stmt returned multiple rows.");
  }

  /* Attempt synchronization */
  vec_t<CustId> cdel = custs_to_del;
  vec_t<CustId> cadd = custs_to_add;
  vec_t<Wayp> out_rte {};  // container for synced route
  vec_t<Stop> out_sch {};  // container for synced schedule

  SyncResult synced = sync(
    new_rte, cur_rte, cur_lvn, new_sch, cur_sch, cadd, cdel, out_rte, out_sch);
  if (synced == SUCCESS) {
    DEBUG(3, { print(MessageType::Info) << "sync succeeded." << std::endl; });
  } else {
    if (strict) {
      DEBUG(3, {
        print(MessageType::Error)
          << "assign() strict enabled; done." << std::endl; });
      this->nrej_++;
      return false;
    }
    if (synced == CDEL_SYNC_FAIL) {
      DEBUG(3, {
        print(MessageType::Error) << "assign() failed due to cdel-sync."
        << std::endl; });
      this->nrej_++;
      return false;
    }
    DEBUG(3, {
      print(MessageType::Info) << "assign() recomputing..." << std::endl; });
    /* The recomputed route must include the vehicle's current location
     * and the next node in the current route (the vehicle must arrive at
     * its next node; it cannot change direction mid-edge) */
    vec_t<Stop> re_sch;
    re_sch.push_back(Stop(vehl.id(), cur_rte.at(cur_lvn).second,
                          StopType::VehlOrig, vehl.early(), vehl.late()));
    re_sch.push_back(Stop(vehl.id(), cur_rte.at(cur_lvn + 1).second,
                          StopType::VehlOrig, vehl.early(), vehl.late()));

    /* Add the original schedule, minus the first stop (old next node) */
    std::copy(new_sch.begin()+1, new_sch.end(), std::back_inserter(re_sch));

    /* Synchronize existing stops (remove picked-up stops) */
    re_sch.erase(
      std::remove_if(re_sch.begin()+2, re_sch.end()-1, [&](const Stop& a) {
      /* If stop does not belong to cadd,
       * remove the stop if its not found in cur_sch */
        if (std::find(cadd.begin(), cadd.end(), a.owner()) == cadd.end()) {
          if (std::find_if(cur_sch.begin()+1, cur_sch.end()-1,
              [&](const Stop& b) {
            return a.owner() == b.owner() && a.loc() == b.loc(); })
                  == cur_sch.end()-1) {
            return true;
          }
        }
        return false; }),
      re_sch.end()-1
    );

    DEBUG(3, { print << "assign() created re_sch:"; print_sch(re_sch); });

    if (!chkcap(curcap, re_sch)) {
      DEBUG(3, {
        print(MessageType::Error)
          << "assign() re_sch failed capacity check"
          << std::endl; });
      this->nrej_++;
      return false;
    }

    /* Re-compute the route and add the traveled distance to each new node */
    vec_t<Wayp> re_rte;
    route_through(re_sch, re_rte);
    for (auto& wp : re_rte)
      wp.first += cur_rte.at(cur_lvn).first;

    DEBUG(3, { print << "assign() re-computed sync_rte:"; print_rte(re_rte); });

    /* After got the route, can delete the current location from re-sch */
    re_sch.erase(re_sch.begin());

    if (!chktw(re_sch, re_rte)) {
      DEBUG(3, {
        print(MessageType::Error)
          << "assign() re-route failed due to time window"
          << std::endl; });
      this->nrej_++;
      return false;
    }

    DEBUG(3, {
      print(MessageType::Info)
        << "assign() re-route complete."
        << std::endl; });
    out_rte = re_rte;
    out_sch = re_sch;
  }

  /* TODO Bug: some customers can be accepted even if late window not met
   * Cause: some algorithms issue assign() at end of batch. Even if the algorithm
   * checks time windows inside the batch, the assignment may no longer be
   * valid at end of batch. This function assign() should check before committing
   * to db. It checks in the case of re-route, but when there is no re-route,
   * it incorrectly skips the check. */

  /* Output synchronized vehicle */
  vehl.set_rte(out_rte);
  vehl.set_nnd(cur_nnd);
  vehl.set_sch(out_sch);
  vehl.reset_lvn();
  vehl.incr_queued();
  for (size_t i = 0; i < cadd.size(); ++i) {
    vehl.incr_queued();
    nmat_++;
  }
  for (size_t i = 0; i < cdel.size(); ++i) {
    vehl.decr_queued();
    nmat_--;
  }

  /* Commit the synchronized route */
  sqlite3_bind_blob(uro_stmt, 1, static_cast<void const*>(out_rte.data()),
                    out_rte.size() * sizeof(Wayp), SQLITE_TRANSIENT);
  sqlite3_bind_int(uro_stmt, 2, 0);
  sqlite3_bind_int(uro_stmt, 3, cur_nnd);
  sqlite3_bind_int(uro_stmt, 4, vehl.id());
  if ((rc = sqlite3_step(uro_stmt)) != SQLITE_DONE) {
    print << "Error in commit new route " << rc << std::endl;
    throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  }
  sqlite3_clear_bindings(uro_stmt);
  sqlite3_reset(uro_stmt);

  /* Commit the synchronized schedule */
  sqlite3_bind_blob(sch_stmt, 1, static_cast<void const*>(out_sch.data()),
                    out_sch.size() * sizeof(Stop), SQLITE_TRANSIENT);
  sqlite3_bind_int(sch_stmt, 2, vehl.id());
  if ((rc = sqlite3_step(sch_stmt)) != SQLITE_DONE) {
    print << "Error in commit new schedule " << rc << std::endl;
    throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  }
  sqlite3_clear_bindings(sch_stmt);
  sqlite3_reset(sch_stmt);

  /* Increase queued */
  sqlite3_bind_int(qud_stmt, 1, (int)(custs_to_add.size()-custs_to_del.size()));
  sqlite3_bind_int(qud_stmt, 2, vehl.id());
  if ((rc = sqlite3_step(qud_stmt)) != SQLITE_DONE) {
    print << "Error in qud " << rc << std::endl;
    throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  }
  sqlite3_clear_bindings(qud_stmt);
  sqlite3_reset(qud_stmt);

  /* Commit the assignment */
  for (const auto& cust_id : custs_to_add) {
    sqlite3_bind_int(com_stmt, 1, vehl.id());
    sqlite3_bind_int(com_stmt, 2, cust_id);
    if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
      print << "Error in commit assignment " << rc << std::endl;
      throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(com_stmt);
    sqlite3_reset(com_stmt);
  }

  /* Commit un-assignments */
  for (const auto& cust_id : custs_to_del) {
    sqlite3_bind_null(com_stmt, 1);
    sqlite3_bind_int(com_stmt, 2, cust_id);
    if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
      print << "Error in commit assignment " << rc << std::endl;
      throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(com_stmt);
    sqlite3_reset(com_stmt);
  }

  /* Log the route and match events */
  Logger::put_r_message(out_rte, vehl);
  Logger::put_m_message(custs_to_add, custs_to_del, vehl.id());

  return true;
}

bool RSAlgorithm::assign_or_delay(
  const vec_t<CustId> & custs_to_add,
  const vec_t<CustId> & custs_to_del,
  const vec_t<Wayp>   & new_rte,
  const vec_t<Stop>   & new_sch,
        MutableVehicle      & vehl) {
//      bool                strict) {
    bool success = false;
    if (this->assign(
            custs_to_add, custs_to_del, new_rte, new_sch, vehl/*, strict*/)) {
      for (const CustId& cust_id : custs_to_add) {
        this->end_delay(cust_id);
      }
      success = true;
    } else {
      for (const CustId& cust_id : custs_to_add) {
        this->beg_delay(cust_id);
      }
    }
    return success;
}

bool RSAlgorithm::delay(const CustId& cust_id) {
  return (delay_.count(cust_id) && delay_.at(cust_id) >= Cargo::now() - retry_)
    ? true : false;
}

void RSAlgorithm::beg_delay(const CustId& cust_id) {
  delay_[cust_id] = Cargo::now();
}

void RSAlgorithm::end_delay(const CustId& cust_id) {
  if (delay_.count(cust_id)) delay_.erase(cust_id);
}

bool RSAlgorithm::timeout(const tick_t& start) {
  auto end = hiclock::now();
  int dur = std::round(dur_milli(end-start).count());
  // print << "Timeout called (" << dur << " > " << timeout_ << "?)" << std::endl;
  return (dur >= timeout_ || this->done()) ? true : false;
}

void RSAlgorithm::print_statistics() {
  print(MessageType::Success)
    << "Matched:  " << this->nmat_ << '\n'
    << "Rejected: " << this->nrej_ << '\n'
    << "Avg. handle_customer dur (ms): " << this->avg_handle_customer_dur_ << '\n'
    << "Avg. handle_vehicle  dur (ms): " << this->avg_handle_vehicle_dur_  << '\n'
    << "Avg. match           dur (ms): " << this->avg_match_dur_           << '\n'
    << "Avg. listen          dur (ms): " << this->avg_listen_dur_          << '\n'
    << "Avg. num. customers per batch: " << this->avg_num_cust_per_batch_  << '\n'
    << "Avg. num. vehicles  per batch: " << this->avg_num_vehl_per_batch_  << '\n'
    << "Count shortest-path comps: " << Cargo::count_sp()
    << std::endl;
}

void RSAlgorithm::print_rte(const vec_t<Wayp>& rte) {
  for (const auto& wp : rte)
    print << " (" << wp.first << "|" << wp.second << ")";
  print << std::endl;
}

void RSAlgorithm::print_sch(const vec_t<Stop>& sch) {
  for (const auto& sp : sch)
    print << " (" << sp.owner() << "|" << sp.loc() << "|" << sp.early()
          <<  "|" << sp.late()  << "|" << (int)sp.type()  << ")";
  print << std::endl;
}

RSAlgorithm::SyncResult
RSAlgorithm::sync(const vec_t<Wayp>   & new_rte,
                  const vec_t<Wayp>   & cur_rte,
                  const RteIdx        & idx_lvn,
                  const vec_t<Stop>   & new_sch,
                  const vec_t<Stop>   & cur_sch,
                  const vec_t<CustId> & cadd,
                  const vec_t<CustId> & cdel,
                        vec_t<Wayp>   & out_rte,
                        vec_t<Stop>   & out_sch) {
  // HACKY BUT WORKS -- ALWAYS RETURN TRUE IF:
  //   1. cur_sch LOOKS LIKE IT IS A STANDBY-TAXI
  bool looks_like_a_taxi = (cur_sch.size() == 2 && cur_sch.front().loc() == cur_sch.back().loc() && cur_sch.back().late() == -1);
  //   2. vehicle LOOKS LIKE IT HAS NOT MOVED
  bool looks_like_hasnt_moved = (cur_rte.at(idx_lvn).second == cur_sch.front().loc());
  if (looks_like_a_taxi || looks_like_hasnt_moved) {
    out_sch = new_sch;
    out_rte = new_rte;
    return SUCCESS;
  }

  out_rte = {};
  out_sch = {};

  DEBUG(3, {
    print << "sync(9) got new_rte: "; print_rte(new_rte);
    print << "sync(9) got cur_rte: "; print_rte(cur_rte);
    print << "sync(9) got new_sch: "; print_sch(new_sch);
    print << "sync(9) got cur_sch: "; print_sch(cur_sch);
  });

  /* VALIDATE CUSTOMERS
   * If any customer stops are visited already, sync fails
   * TODO: If customer already timed out, sync fails
   */
  for (const CustId& cid : cadd) {
    sqlite3_bind_int(sva_stmt, 1, cid);
    sqlite3_bind_int(sva_stmt, 2, (int)StopType::CustOrig);
    if (sqlite3_step(sva_stmt) != SQLITE_ROW) {
      print(MessageType::Error) << "sva_stmt failed" << std::endl;
      throw;
    }
    if (sqlite3_step(sva_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "sva_stmt multiple rows" << std::endl;
      throw;
    }
    if (sqlite3_column_int(sva_stmt, 0) == -1)
      return CADD_SYNC_FAIL;
    sqlite3_clear_bindings(sva_stmt);
    sqlite3_reset(sva_stmt);
    sqlite3_bind_int(sva_stmt, 1, cid);
    sqlite3_bind_int(sva_stmt, 2, (int)StopType::CustDest);
    if (sqlite3_step(sva_stmt) != SQLITE_ROW) {
      print(MessageType::Error) << "sva_stmt failed" << std::endl;
      throw;
    }
    if (sqlite3_step(sva_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "sva_stmt multiple rows" << std::endl;
      throw;
    }
    if (sqlite3_column_int(sva_stmt, 0) == -1)
      return CADD_SYNC_FAIL;
    sqlite3_clear_bindings(sva_stmt);
    sqlite3_reset(sva_stmt);
  }
  for (const CustId& cid : cdel) {
    sqlite3_bind_int(sva_stmt, 1, cid);
    sqlite3_bind_int(sva_stmt, 2, (int)StopType::CustOrig);
    if (sqlite3_step(sva_stmt) != SQLITE_ROW) {
      print(MessageType::Error) << "sva_stmt failed" << std::endl;
      throw;
    }
    if (sqlite3_step(sva_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "sva_stmt multiple rows" << std::endl;
      throw;
    }
    if (sqlite3_column_int(sva_stmt, 0) == -1)
      return CDEL_SYNC_FAIL;
    sqlite3_clear_bindings(sva_stmt);
    sqlite3_reset(sva_stmt);
    sqlite3_bind_int(sva_stmt, 1, cid);
    sqlite3_bind_int(sva_stmt, 2, (int)StopType::CustDest);
    if (sqlite3_step(sva_stmt) != SQLITE_ROW) {
      print(MessageType::Error) << "sva_stmt failed" << std::endl;
      throw;
    }
    if (sqlite3_step(sva_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "sva_stmt multiple rows" << std::endl;
      throw;
    }
    if (sqlite3_column_int(sva_stmt, 0) == -1)
      return CDEL_SYNC_FAIL;
    sqlite3_clear_bindings(sva_stmt);
    sqlite3_reset(sva_stmt);
  }

  /* COMPARE PREFIXES
   * If mismatch, sync fails. If match, proceed with checking stops */
  const NodeId& curloc = cur_rte.at(idx_lvn).second;
  const NodeId& nxtloc = cur_rte.at(idx_lvn+1).second;
  DEBUG(3, {
    print << "sync(9) got curloc=" << curloc << std::endl;
    print << "sync(9) got nxtloc=" << nxtloc << std::endl;
  });
  auto x = std::find_if(new_rte.begin(), new_rte.end(), [&](const Wayp& a) {
      return a.second == curloc; });
  if (x == new_rte.end() || (x+1)->second != nxtloc) {
      DEBUG(3, {
        if (x == new_rte.end())      print << "sync(9) curloc not in new_rte" << std::endl;
        if ((x+1)->second != nxtloc) print << "sync(9) nxtloc not in new_rte" << std::endl;
      });
      return CURLOC_MISMATCH;
  }

  /* SYNCHRONIZE ROUTE */
  /* new_rte prefix must match what the vehicle's traveled */
  auto i = x + 2;
  auto j = cur_rte.begin() + idx_lvn + 2;
  DEBUG(3, { print << "Checking prefix: "; });
  while (i != new_rte.begin()) {
    i--; j--;
    DEBUG(3, { print << " " << i->second << "==" << j->second; });
    /* Return false if j has reached head of the route */
    if (i != new_rte.begin() && j == cur_rte.begin()) {
      DEBUG(3, { print << "sync(9) new_rte prefix exceeds cur_rte" << std::endl; });
      return PREFIX_MISMATCH;
    }
    /* Return false if i, j mismatch */
    if (i->second != j->second) {
      DEBUG(3, { print << "sync(9) prefix mismatch" << std::endl; });
      return PREFIX_MISMATCH;
    }
  }
  DEBUG(3, { print << " new_rte done." << std::endl; });

  /* Synchronize existing stops */
  auto k = new_sch.begin() + 1;
  for (auto pr = new_rte.begin(); pr <= x; pr++) {
    DEBUG(3, { print << pr->second << ";" << k->loc() << std::endl; });
    while (pr->second == k->loc())
      k++;
  }

  /* Synchronize fails if any stops in the prefix belong to cadd
   * (both stops must be in the remaining schedule) */
  for (const CustId& cust_id : cadd) {
    auto g = std::find_if(k, new_sch.end(), [&](const Stop& a) {
            return a.owner() == cust_id && a.type() == StopType::CustOrig; });
    if (g == new_sch.end()) {
      DEBUG(3, { print
        << "sync(9) bad add(owner=" << cust_id << ")"
        << std::endl;
      });
      return CADD_SYNC_FAIL;
    }
  }

  /* Return remaining portion of new route and last-visited node */
  out_rte.push_back(cur_rte.at(idx_lvn));
  std::copy(x + 1, new_rte.end(), std::back_inserter(out_rte));

  /* Return remaining portion of new schedule, with synchronized location */
  out_sch.push_back(cur_sch.at(0));
  std::copy(k, new_sch.end(), std::back_inserter(out_sch));

  /* HACK! I think the section above, Synchronize existing stops, is aggressively
   * leaving out the last stop of taxis (the fake destination) if the vehicle
   * hasn't moved. So just put it back in here. The issue only seems to happen
   * for taxis. */
  if (out_sch.size() == 1 && out_sch.front().late() == -1) {
    const Stop& stop = out_sch.front();
    Stop fake_dest(stop.owner(), stop.loc(), StopType::VehlDest, stop.early(), stop.late());
    out_sch.push_back(fake_dest);
  }

  DEBUG(3, {
    print << "sync(9) out_rte: "; print_rte(out_rte);
    print << "sync(9) out_sch: "; print_sch(out_sch);
  });
  return SUCCESS;
}

void RSAlgorithm::select_matchable_vehicles() {
  vehicles_.clear();
  sqlite3_bind_int(smv_stmt, 1, Cargo::now());
  sqlite3_bind_int(smv_stmt, 2, (int)VehlStatus::Arrived);
  while ((rc = sqlite3_step(smv_stmt)) == SQLITE_ROW) {
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(smv_stmt,  8));
    const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(smv_stmt, 11));
    vec_t<Wayp> raw_rte(rtebuf, rtebuf + sqlite3_column_bytes(smv_stmt,  8) / sizeof(Wayp));
    vec_t<Stop> raw_sch(schbuf, schbuf + sqlite3_column_bytes(smv_stmt, 11) / sizeof(Stop));
    Route route(sqlite3_column_int(smv_stmt, 0), raw_rte);
    Schedule schedule(sqlite3_column_int(smv_stmt, 0), raw_sch);

    SimlTime vlt = sqlite3_column_int(smv_stmt, 4);

    /* Vehicles approaching their destination cannot be matchable UNLESS it is
     * a taxi. */
    if (vlt != -1 &&
       (schedule.data().at(0).loc() == schedule.data().at(1).loc() &&
        schedule.data().at(1).type() == StopType::VehlDest))
      continue;

    /* Construct vehicle object
     * If permanent taxi, set vehl.dest() to the last wp in the route */
    NodeId vehl_dest = sqlite3_column_int(smv_stmt, 2);
    if (vlt == -1) vehl_dest = raw_rte.back().second;
    Vehicle vehicle(
        sqlite3_column_int(smv_stmt, 0), // vehl id
        sqlite3_column_int(smv_stmt, 1), // orig id
        vehl_dest,                       // dest id
        sqlite3_column_int(smv_stmt, 3), // early
        vlt,                             // late
        sqlite3_column_int(smv_stmt, 5), // load
        sqlite3_column_int(smv_stmt, 6), // queued
        sqlite3_column_int(smv_stmt, 10),// nnd
        route,                           // route
        schedule,                        // schedule
        sqlite3_column_int(smv_stmt, 9), // lvn
        static_cast<VehlStatus>(sqlite3_column_int(smv_stmt, 7))); // status
    vehicles_.push_back(vehicle);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(smv_stmt);
  sqlite3_reset(smv_stmt);
}

void RSAlgorithm::select_waiting_customers(
    bool skip_assigned, bool skip_delayed) {
  customers_.clear();
  sqlite3_bind_int(swc_stmt, 1, (int)CustStatus::Waiting);
  sqlite3_bind_int(swc_stmt, 2, Cargo::now());
  while ((rc = sqlite3_step(swc_stmt)) == SQLITE_ROW) {
    Customer customer(
        sqlite3_column_int(swc_stmt, 0), sqlite3_column_int(swc_stmt, 1),
        sqlite3_column_int(swc_stmt, 2), sqlite3_column_int(swc_stmt, 3),
        sqlite3_column_int(swc_stmt, 4), sqlite3_column_int(swc_stmt, 5),
        static_cast<CustStatus>(sqlite3_column_int(swc_stmt, 6)),
        sqlite3_column_int(swc_stmt, 7));
    if (customer.assigned() && skip_assigned) {
      ; // do nothing
    } else if (this->delay(customer.id()) && skip_delayed) {
      ; // do nothing
    } else {
      customers_.push_back(customer);
    }
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(swc_stmt);
  sqlite3_reset(swc_stmt);
}

vec_t<Customer> RSAlgorithm::get_all_customers() {
  vec_t<Customer> custs;
  while ((rc = sqlite3_step(sac_stmt)) == SQLITE_ROW) {
    Customer customer(
        sqlite3_column_int(sac_stmt, 0), sqlite3_column_int(sac_stmt, 1),
        sqlite3_column_int(sac_stmt, 2), sqlite3_column_int(sac_stmt, 3),
        sqlite3_column_int(sac_stmt, 4), sqlite3_column_int(sac_stmt, 5),
        static_cast<CustStatus>(sqlite3_column_int(sac_stmt, 6)),
        sqlite3_column_int(sac_stmt, 7));
    custs.push_back(customer);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_reset(sac_stmt);
  return custs;
}

vec_t<Vehicle> RSAlgorithm::get_all_vehicles() {
  vec_t<Vehicle> vehls;
  while ((rc = sqlite3_step(sav_stmt)) == SQLITE_ROW) {
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(sav_stmt,  8));
    const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(sav_stmt, 11));
    vec_t<Wayp> raw_rte(rtebuf, rtebuf + sqlite3_column_bytes(sav_stmt,  8) / sizeof(Wayp));
    vec_t<Stop> raw_sch(schbuf, schbuf + sqlite3_column_bytes(sav_stmt, 11) / sizeof(Stop));
    Route route(sqlite3_column_int(sav_stmt, 0), raw_rte);
    Schedule schedule(sqlite3_column_int(sav_stmt, 0), raw_sch);

    /* Construct vehicle object */
    Vehicle vehicle(
        sqlite3_column_int(sav_stmt, 0), // vehl id
        sqlite3_column_int(sav_stmt, 1), // orig id
        sqlite3_column_int(sav_stmt, 2), // dest id
        sqlite3_column_int(sav_stmt, 3), // early
        sqlite3_column_int(sav_stmt, 4), // late
        sqlite3_column_int(sav_stmt, 5), // load
        sqlite3_column_int(sav_stmt, 6), // queued
        sqlite3_column_int(sav_stmt, 10),// nnd
        route,                           // route
        schedule,                        // schedule
        sqlite3_column_int(sav_stmt, 9), // lvn
        static_cast<VehlStatus>(sqlite3_column_int(sav_stmt, 7))); // status
    vehls.push_back(vehicle);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_reset(sav_stmt);
  return vehls;
}

/* Overrideables */
void RSAlgorithm::handle_customer(const Customer&) {
  /* For streaming-matching or other customer processing. */
}

void RSAlgorithm::handle_vehicle(const Vehicle&) {
  /* For vehicle processing (e.g. add to a spatial index) */
}

void RSAlgorithm::match() {
  /* For bulk-matching. Access current customers and vehicles using
   * customers() and vehicles() */
}

void RSAlgorithm::end() {
  /* Executes after the simulation finishes. */
  this->avg_handle_customer_dur_ = (float) std::accumulate(dur_handle_customer_.begin(), dur_handle_customer_.end(), 0)/dur_handle_customer_.size();
  this->avg_handle_vehicle_dur_  = (float) std::accumulate(dur_handle_vehicle_ .begin(), dur_handle_vehicle_ .end(), 0)/dur_handle_vehicle_ .size();
  this->avg_match_dur_           = (float) std::accumulate(dur_match_          .begin(), dur_match_          .end(), 0)/dur_match_          .size();
  this->avg_listen_dur_          = (float) std::accumulate(dur_listen_         .begin(), dur_listen_         .end(), 0)/dur_listen_         .size();
  this->avg_num_cust_per_batch_  = (float) std::accumulate(n_cust_per_batch_   .begin(), n_cust_per_batch_   .end(), 0)/n_cust_per_batch_   .size();
  this->avg_num_vehl_per_batch_  = (float) std::accumulate(n_vehl_per_batch_   .begin(), n_vehl_per_batch_   .end(), 0)/n_vehl_per_batch_   .size();
  this->print_statistics();
  
}

void RSAlgorithm::listen(bool skip_assigned, bool skip_delayed) {
  if (Cargo::static_mode)
    Cargo::ofmx.lock();

  // Start timing -------------------------------
  this->t_listen_0 = hiclock::now();

  this->select_matchable_vehicles();
  int num_vehicles = this->vehicles_.size();
  this->n_vehl_per_batch_.push_back(num_vehicles);
  for (const auto& vehicle : this->vehicles_) {
    this->t_handle_vehicle_0 = hiclock::now();
    this->handle_vehicle(vehicle);
    this->t_handle_vehicle_1 = hiclock::now();
    this->dur_handle_vehicle_.push_back(
      duration(t_handle_vehicle_0, t_handle_vehicle_1));
  }

  this->select_waiting_customers(skip_assigned, skip_delayed);
  int num_customers = this->customers_.size();
  this->n_cust_per_batch_.push_back(num_customers);
  // Set default timeout (per customer)
  this->timeout_ = (Cargo::static_mode ? InfInt : 30000);
  for (const auto& customer : this->customers_) {
    this->t_handle_customer_0 = hiclock::now();
    this->handle_customer(customer);
    this->t_handle_customer_1 = hiclock::now();
    this->dur_handle_customer_.push_back(
      duration(t_handle_customer_0, t_handle_customer_1));
  }
  // Set default timeout (per batch)
  this->timeout_ = (Cargo::static_mode ? InfInt : 30000);
  this->t_match_0 = hiclock::now();
  this->match();
  this->t_match_1 = hiclock::now();
  this->dur_match_.push_back(duration(t_match_0, t_match_1));

  this->t_listen_1 = hiclock::now();
  // Stop timing --------------------------------

  int dur = this->duration(t_listen_0, t_listen_1);
  this->dur_listen_.push_back(dur);

  Cargo::paused() = false;
  Cargo::pause_cv.notify_one();

  if (Cargo::static_mode) {
    Cargo::ofmx.unlock();
    // std::this_thread::sleep_for(milli(100)); // timing hack
    std::this_thread::sleep_for(milli(this->batch_time_ * 1000));
  } else {
    // Don't sleep if time exceeds batch time
    // int dur = std::round(dur_milli(batch_1-batch_0).count());
    if (dur > this->batch_time_ * 1000) {
      print(MessageType::Warning)
          << "listen() (" << dur << " ms) "
          << "exceeds batch time ("
          << this->batch_time_ * 1000 << " ms) for "
          << num_vehicles << " vehls and " << num_customers << " custs"
          << std::endl;
    } else {
      // print
      //     << "listen() handled "
      //     << this->vehicles_.size() << " vehls and "
      //     << this->customers_.size() << " custs " << "in " << dur << " ms"
      //     << std::endl;
      std::this_thread::sleep_for(milli(this->batch_time_ * 1000 - dur));
    }
  }
}

}  // namespace cargo

