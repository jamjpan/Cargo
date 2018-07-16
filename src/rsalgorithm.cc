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
#include <algorithm> /* std::find */
#include <chrono>
#include <mutex>
#include <thread>

#include "libcargo/cargo.h" /* now(), gtree(), db() */
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/dbsql.h"
#include "libcargo/message.h"
#include "libcargo/rsalgorithm.h"
#include "libcargo/types.h"

namespace cargo {

RSAlgorithm::RSAlgorithm(const std::string& name) : print(name) {
  name_ = name;
  done_ = false;
  batch_time_ = 1;
  if (sqlite3_prepare_v2(Cargo::db(), sql::ssr_stmt, -1, &ssr_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::sss_stmt, -1, &sss_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::uro_stmt, -1, &uro_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::sch_stmt, -1, &sch_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::qud_stmt, -1, &qud_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::com_stmt, -1, &com_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::smv_stmt, -1, &smv_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::sac_stmt, -1, &sac_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::sav_stmt, -1, &sav_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(Cargo::db(), sql::swc_stmt, -1, &swc_stmt, NULL) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create rsalg stmts). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  }
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
}

const std::string & RSAlgorithm::name() const { return name_; }
const bool        & RSAlgorithm::done() const { return done_; }
      int         & RSAlgorithm::batch_time() { return batch_time_; }
      void          RSAlgorithm::kill()       { done_ = true; }

std::vector<Customer> & RSAlgorithm::customers() { return customers_; }
std::vector<Vehicle>  & RSAlgorithm::vehicles()  { return vehicles_; }

/* Assignment comes in two flavors: assign and assign_strict.
 * Function assign will try to do a strict-assign. If the new route
 * cannot be synchronized to the vehicle due to match latency, then
 * assign will poll the vehicle's position and re-compute new route
 * using this position. If the recomputed route fails time window check,
 * the assign returns false. Otherwise it commits this recomputed route
 * to the database.
 *
 * Returns false if:
 *   custs_to_del contains a customer already picked up due to match latency
 *   match lat. cause recompute new_rte, and recomputed route fails time cons.
 *
 * */
bool RSAlgorithm::assign(
        const std::vector<Customer> & custs_to_add,
        const std::vector<CustId>   & custs_to_del,
              MutableVehicle        & vehl,
              bool                    strict) {
  std::lock_guard<std::mutex> dblock(Cargo::dbmx);  // Lock acquired

  const std::vector<Wayp>& new_rte = vehl.route().data();
  const std::vector<Stop>& new_sch = vehl.schedule().data();

  DEBUG(3, { std::cout << "assign() called with new_rte:"; print_rte(new_rte); });
  DEBUG(3, { std::cout << "assign() called with new_sch:"; print_sch(new_sch); });

  /* Get current schedule */
  std::vector<Stop> cur_sch;
  sqlite3_bind_int(sss_stmt, 1, vehl.id());
  while ((rc = sqlite3_step(sss_stmt)) == SQLITE_ROW) {
    const Stop* buf = static_cast<const Stop*>(sqlite3_column_blob(sss_stmt, 1));
    std::vector<Stop> raw_sch(buf, buf + sqlite3_column_bytes(sss_stmt, 1) / sizeof(Stop));
    cur_sch = raw_sch;
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(sss_stmt);
  sqlite3_reset(sss_stmt);

  DEBUG(3, { std::cout << "assign() found cur_sch:"; print_sch(cur_sch); });

  /* If stops belong to custs_to_del are no longer in cur_sch, the custs
   * cannot be deleted (because they've already been picked up or dropped
   * off. Reject the commit. */
  for (const auto& cust_id : custs_to_del) {
    auto i = std::find_if(cur_sch.begin(), cur_sch.end(), [&](const Stop& a) { return a.owner() == cust_id; });
    if (i == cur_sch.end()) return false;
    auto j = std::find_if(i, cur_sch.end(), [&](const Stop& a) { return a.owner() == cust_id; });
    if (j == cur_sch.end()) return false;
  }

  DEBUG(3, { std::cout << "assign() found no problems with cust_del" << std::endl; });

  /* Get current route */
  RteIdx cur_lvn = 0;
  DistInt cur_nnd = 0;
  std::vector<Wayp> cur_rte;
  sqlite3_bind_int(ssr_stmt, 1, vehl.id());
  while ((rc = sqlite3_step(ssr_stmt)) == SQLITE_ROW) {
    const Wayp* buf = static_cast<const Wayp*>(sqlite3_column_blob(ssr_stmt, 1));
    std::vector<Wayp> raw_rte(buf, buf + sqlite3_column_bytes(ssr_stmt, 1) / sizeof(Wayp));
    cur_rte = raw_rte;
    cur_lvn = sqlite3_column_int(ssr_stmt, 2);
    cur_nnd = sqlite3_column_int(ssr_stmt, 3);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(ssr_stmt);
  sqlite3_reset(ssr_stmt);

  DEBUG(3, { std::cout << "assign() found cur_rte:"; print_rte(cur_rte); });

  /* Attempt synchronization */
  std::vector<Wayp> sync_rte;
  std::vector<Stop> re_sch = new_sch;
  if (!sync_route(new_rte, cur_rte, cur_lvn, custs_to_add, sync_rte)) {
    DEBUG(3, { std::cout << "assign() sync_route failed; recomputing..." << std::endl; });

    if (strict) return false;

    /* The recomputed schedule must include the vehicle's current location
     * and the next node in the current route (the vehicle must arrive at
     * its next node; it cannot change direction mid-edge) */
    re_sch[0] = Stop(vehl.id(), cur_rte.at(cur_lvn).second, StopType::VehlOrig, vehl.early(), vehl.late());
    re_sch.insert(re_sch.begin()+1, Stop(vehl.id(), cur_rte.at(cur_lvn+1).second, StopType::VehlOrig, vehl.early(), vehl.late()));

    /* If the next node is a stop, move the stop to the beginning of re_sch */
    if (cur_sch.at(0).loc() == cur_sch.at(1).loc() &&
        cur_sch.at(1).type() != StopType::VehlDest) {
      auto i = std::find_if(re_sch.begin(), re_sch.end(), [&](const Stop& a) {
              return a.loc() == cur_sch.at(1).loc() && a.type() == cur_sch.at(1).type(); });
      Stop tmp = *i;
      re_sch.erase(i);
      re_sch.insert(re_sch.begin()+2, tmp);
    }

    DEBUG(3, { std::cout << "assign() created re_sch:"; print_sch(re_sch); });

    /* Re-compute the route and add the traveled distance to each of the new
     * nodes */
    route_through(re_sch, sync_rte);
    for (auto& wp : sync_rte) wp.first += cur_rte.at(cur_lvn).first;

    DEBUG(3, { std::cout << "assign() re-computed sync_rte:"; print_rte(sync_rte); });

    /* After got the route, can delete the current location from re-sch */
    re_sch.erase(re_sch.begin());

    if (!chktw(re_sch, sync_rte)) return false;

    DEBUG(3, { std::cout << "assign() found no problems with timewindow" << std::endl; });
  }

  std::vector<Stop> sync_sch;
  if (sync_schedule(re_sch, cur_sch, sync_rte, custs_to_add, sync_sch)) {
    DEBUG(3, { std::cout << "assign() sync_sch:"; print_sch(sync_sch); });

    /* Output synchronized vehicle */
    vehl.set_rte(sync_rte);
    vehl.set_nnd(cur_nnd);
    vehl.set_sch(sync_sch);
    vehl.reset_lvn();

    // Commit the synchronized route
    sqlite3_bind_blob(uro_stmt, 1, static_cast<void const*>(sync_rte.data()),
                      sync_rte.size() * sizeof(Wayp), SQLITE_TRANSIENT);
    sqlite3_bind_int(uro_stmt, 2, 0);
    sqlite3_bind_int(uro_stmt, 3, cur_nnd);
    sqlite3_bind_int(uro_stmt, 4, vehl.id());
    if ((rc = sqlite3_step(uro_stmt)) != SQLITE_DONE) {
      std::cout << "Error in commit new route " << rc << std::endl;
      throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(uro_stmt);
    sqlite3_reset(uro_stmt);

    // Commit the synchronized schedule
    sqlite3_bind_blob(sch_stmt, 1, static_cast<void const*>(sync_sch.data()),
                      sync_sch.size() * sizeof(Stop), SQLITE_TRANSIENT);
    sqlite3_bind_int(sch_stmt, 2, vehl.id());
    if ((rc = sqlite3_step(sch_stmt)) != SQLITE_DONE) {
      std::cout << "Error in commit new schedule " << rc << std::endl;
      throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(sch_stmt);
    sqlite3_reset(sch_stmt);

    // Increase queued
    sqlite3_bind_int(qud_stmt, 1, (int)(custs_to_add.size()-custs_to_del.size()));
    sqlite3_bind_int(qud_stmt, 2, vehl.id());
    if ((rc = sqlite3_step(qud_stmt)) != SQLITE_DONE) {
      std::cout << "Error in qud " << rc << std::endl;
      throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(qud_stmt);
    sqlite3_reset(qud_stmt);

    // Commit the assignment
    for (const auto& cust : custs_to_add) {
      sqlite3_bind_int(com_stmt, 1, vehl.id());
      sqlite3_bind_int(com_stmt, 2, cust.id());
      if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
      }
      sqlite3_clear_bindings(com_stmt);
      sqlite3_reset(com_stmt);
    }

    // Commit un-assignments
    for (const auto& cust_id : custs_to_del) {
      sqlite3_bind_null(com_stmt, 1);
      sqlite3_bind_int(com_stmt, 2, cust_id);
      if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
      }
      sqlite3_clear_bindings(com_stmt);
      sqlite3_reset(com_stmt);
    }
    return true;
  }
  DEBUG(3, { std::cout << "rsalgorithm() sync_sch failed" << std::endl; });
  return false;
}

bool RSAlgorithm::assign_strict(
        const std::vector<Customer> & custs_to_add,
        const std::vector<CustId>   & custs_to_del,
              MutableVehicle        & vehl) {
  return assign(custs_to_add, custs_to_del, vehl, true);
}

void RSAlgorithm::select_matchable_vehicles() {
  vehicles_.clear();
  sqlite3_bind_int(smv_stmt, 1, Cargo::now());
  sqlite3_bind_int(smv_stmt, 2, (int)VehlStatus::Arrived);
  while ((rc = sqlite3_step(smv_stmt)) == SQLITE_ROW) {
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(smv_stmt,  9));
    const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(smv_stmt, 13));
    std::vector<Wayp> raw_rte(rtebuf, rtebuf + sqlite3_column_bytes(smv_stmt,  9) / sizeof(Wayp));
    std::vector<Stop> raw_sch(schbuf, schbuf + sqlite3_column_bytes(smv_stmt, 13) / sizeof(Stop));
    Route route(sqlite3_column_int(smv_stmt, 0), raw_rte);
    Schedule schedule(sqlite3_column_int(smv_stmt, 0), raw_sch);

    /* Construct vehicle object
     * If permanent taxi, set vehl.dest() to the last wp in the route */
    SimlTime vlt = sqlite3_column_int(smv_stmt, 4);
    NodeId vehl_dest = sqlite3_column_int(smv_stmt, 2);
    if (vlt == -1) vehl_dest = raw_rte.back().second;
    Vehicle vehicle(
        sqlite3_column_int(smv_stmt, 0), sqlite3_column_int(smv_stmt, 1),
        vehl_dest, sqlite3_column_int(smv_stmt, 3), vlt,
        sqlite3_column_int(smv_stmt, 5), sqlite3_column_int(smv_stmt, 6),
        sqlite3_column_int(smv_stmt, 11), route, schedule,
        sqlite3_column_int(smv_stmt, 10),
        static_cast<VehlStatus>(sqlite3_column_int(smv_stmt, 7)));
    vehicles_.push_back(vehicle);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(smv_stmt);
  sqlite3_reset(smv_stmt);
}

void RSAlgorithm::select_waiting_customers() {
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
    customers_.push_back(customer);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_clear_bindings(swc_stmt);
  sqlite3_reset(swc_stmt);
}

std::vector<Customer> RSAlgorithm::get_all_customers() {
  std::vector<Customer> custs;
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

std::vector<Vehicle> RSAlgorithm::get_all_vehicles() {
  std::vector<Vehicle> vehls;
  while ((rc = sqlite3_step(sav_stmt)) == SQLITE_ROW) {
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(sav_stmt,  9));
    const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(sav_stmt, 13));
    std::vector<Wayp> raw_rte(rtebuf, rtebuf + sqlite3_column_bytes(sav_stmt,  9) / sizeof(Wayp));
    std::vector<Stop> raw_sch(schbuf, schbuf + sqlite3_column_bytes(sav_stmt, 13) / sizeof(Stop));
    Route route(sqlite3_column_int(sav_stmt, 0), raw_rte);
    Schedule schedule(sqlite3_column_int(sav_stmt, 0), raw_sch);

    /* Construct vehicle object */
    Vehicle vehicle(
        sqlite3_column_int(sav_stmt, 0), sqlite3_column_int(sav_stmt, 1),
        sqlite3_column_int(sav_stmt, 2), sqlite3_column_int(sav_stmt, 3),
        sqlite3_column_int(sav_stmt, 4), sqlite3_column_int(sav_stmt, 5),
        sqlite3_column_int(sav_stmt, 6), sqlite3_column_int(sav_stmt, 11),
        route, schedule, sqlite3_column_int(sav_stmt, 10),
        static_cast<VehlStatus>(sqlite3_column_int(sav_stmt, 7)));
    vehls.push_back(vehicle);
  }
  if (rc != SQLITE_DONE) throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
  sqlite3_reset(sav_stmt);
  return vehls;
}

bool RSAlgorithm::sync_route(const std::vector<Wayp>& new_rte,
                             const std::vector<Wayp>& cur_rte,
                             const RteIdx& cur_lvn,
                             const std::vector<Customer>& custs,
                             std::vector<Wayp>& sync_rte) {
  /* Strategy to sync routes
   * Get the current route/lvn/nnd. Find the current waypoint in the
   * new route. If not found, the new route is invalid, reject the commit.
   * If found, move back one in the current route, move back one in the
   * new route, and check if equal. If not equal, reject the commit.
   * Continue moving back one waypoint, checking if equal, until no more
   * waypoints in the new route. */

  sync_rte = new_rte;

  /* If vehicle hasn't moved, nothing to do */
  if (cur_lvn == 0) return true;

  /* Get current location in the new route */
  auto x = std::find_if(sync_rte.begin(), sync_rte.end(), [&](const Wayp& a) {
    return a.second == cur_rte.at(cur_lvn).second;
  });

  /* If couldn't find current lvn in the new route, sync is not possible */
  if (x == sync_rte.end()) return false;

  /* Get next node in the new route */
  auto y = std::find_if(sync_rte.begin(), sync_rte.end(), [&](const Wayp& a) {
    return a.second == cur_rte.at(cur_lvn+1).second;
  });

  /* If couldn't find next node, sync is not possible
   * (the vehicle cannot make a u-turn along an edge) */
  if (y == sync_rte.end()) return false;

  /* Sync is possible only if all custs are in the remaining route */
  for (const auto& cust : custs) {
    auto o_itr = std::find_if(y, sync_rte.end(), [&](const Wayp& a) {
            return a.second == cust.orig(); });
    if (o_itr == sync_rte.end()) return false;
    auto d_itr = std::find_if(o_itr, sync_rte.end(), [&](const Wayp& a) {
            return a.second == cust.dest(); });
    if (d_itr == sync_rte.end()) return false;
  }

  /* If found current loc but vehicle has moved, test the routes */
  int j = cur_lvn+1;
  auto i = y;
  while (true) {
    /* If any waypoint in the prefix does not match, or
     * if current route is finished but new_rte still has waypoints,
     * the sync is impossible. */
    if ((i->second != cur_rte.at(j).second) ||
        (i > sync_rte.begin() && j == 0)) {
      return false;
      break;
    }
    /* If neither route is finished yet, decrement and do again */
    if (i > sync_rte.begin() && j > 0) {
      j--;
      i--;
      /* When new_rte is finished, finish */
    } else if (i == sync_rte.begin()) {
      break;
    }
  }

  sync_rte.erase(sync_rte.begin(), x);
  return true;
}

bool RSAlgorithm::sync_schedule(const std::vector<Stop>& new_sch,
                                const std::vector<Stop>& cur_sch,
                                const std::vector<Wayp>& sync_rte,
                                const std::vector<Customer>& custs,
                                std::vector<Stop>& sync_sch) {
  DEBUG(3, { std::cout << "sync_sch() got new_sch"; print_sch(new_sch); });
  DEBUG(3, { std::cout << "sync_sch() got cur_sch"; print_sch(cur_sch); });
  DEBUG(3, { std::cout << "sync_sch() got sync_rte"; print_rte(sync_rte); });

  /* Ugly hack:
   * Sometimes we get a cur_sch like {a a b c} (a is the next node and is also
   * a stop). Due to matching latency, we can get a new_sch like {x y a a b c}
   * (x is already passed by the vehicle, hence it is not in cur_sch; y is newly
   * added to due match). This new_sch should be invalid. sync_route catches
   * most of these cases because it looks to see each customer origin is after
   * the next node; i.e. y.orig should occur after a. But in some cases, the
   * current route does have y.orig after the next node, but the schedule has it
   * before. For example current route is {a a b y c}, the new schedule is
   * {x y a a b c} with route {x y a a b y c}, but the first y is when the pickup
   * gets made. sync_route will not catch these case. We try to catch these
   * cases here by checking if the first and second stops in cur_sch are
   * equal; if so, then we know the vehicle is moving toward a stop. In this case,
   * we check if every cust has a stop that is after this stop in the new route.
   */
  if (cur_sch.at(0).loc() == cur_sch.at(1).loc()) {
    NodeId next_id = cur_sch.at(1).loc();
    StopType next_type = cur_sch.at(1).type();
    auto x = std::find_if(new_sch.begin(), new_sch.end(), [&](const Stop& a) {
      return (a.loc() == next_id && a.type() == next_type); });
    for (const auto& cust : custs) {
        auto y = std::find_if(x, new_sch.end(), [&](const Stop& a) {
          return (a.loc() == cust.orig() && a.type() == StopType::CustOrig); });
        if (y == new_sch.end()) return false;
        auto z = std::find_if(y, new_sch.end(), [&](const Stop& a) {
          return (a.loc() == cust.dest() && a.type() == StopType::CustDest); });
        if (z == new_sch.end()) return false;
    }
  }
  /* Strategy to sync schedules
   * The first stop in the schedule takes from cur_sch. All subsequent
   * stops, check if in cur_sch, or if equals orig/dest of the custs
   * being committed. If not, then discard. */
  sync_sch = {};
  sync_sch.push_back(cur_sch.at(0));
  for (size_t i = 1; i < new_sch.size(); ++i) {
    const Stop& stop = new_sch.at(i);
    bool in_cur = (std::find(cur_sch.begin(), cur_sch.end(), stop) != cur_sch.end());
    if (in_cur)
        sync_sch.push_back(stop);
    else {
      for (const auto& cust : custs) {
        bool is_orig = (cust.id() == stop.owner() && cust.orig() == stop.loc());
        if (is_orig) {
          sync_sch.push_back(stop);
          break;
        }
        bool is_dest = (cust.id() == stop.owner() && cust.dest() == stop.loc());
        if (is_dest) {
          sync_sch.push_back(stop);
          break;
        }
      }
    }
  }
  /* Here we do a final check to make sure every stop is after the vehicle's
   * cur location, and all stops are in order as they appear in the route. */
  for (const auto& stop : sync_sch) {
    auto x = sync_rte.begin()+1; // <-- use +1 to skip last-visited node
    auto y = std::find_if(x, sync_rte.end(), [&](const Wayp& a) {
      return (a.second == stop.loc()); });
    if (y == sync_rte.end()) return false;
    auto z = std::find_if(y, sync_rte.end(), [&](const Wayp& a) {
      return (a.second == stop.loc()); });
    if (z == sync_rte.end()) return false;
  }

  return true;
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

void RSAlgorithm::end() { /* Executes after the simulation finishes. */ }

void RSAlgorithm::listen() {
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

  // Start timing -------------------------------
  t0 = std::chrono::high_resolution_clock::now();

  select_matchable_vehicles();
  for (const auto& vehicle : vehicles_) handle_vehicle(vehicle);

  select_waiting_customers();
  for (const auto& customer : customers_) handle_customer(customer);

  match();
  t1 = std::chrono::high_resolution_clock::now();
  // Stop timing --------------------------------

  // Don't sleep if time exceeds batch time
  int dur = std::round(std::chrono::duration<double, std::milli>(t1-t0).count());
  if (dur > batch_time_ * 1000)
    print(MessageType::Warning) << "listen() (" << dur << " ms) exceeds batch time ("
                  << batch_time_ * 1000 << " ms) for " << vehicles_.size()
                  << " vehls and " << customers_.size() << " custs"
                  << std::endl;
  else {
    print << "listen() handled " << vehicles_.size() << " vehls and "
               << customers_.size() << " custs in " << dur << " ms"
               << std::endl;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(batch_time_ * 1000 - dur));
  }
}

}  // namespace cargo

