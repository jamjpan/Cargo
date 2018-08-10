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
#include <chrono>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <string>
#include <thread>

#include "libcargo/cargo.h"
#include "libcargo/classes.h"
#include "libcargo/debug.h"
#include "libcargo/dbsql.h"
#include "libcargo/file.h"
#include "libcargo/functions.h"
#include "libcargo/message.h"
#include "libcargo/options.h"
#include "libcargo/rsalgorithm.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"
#include "sqlite3/sqlite3.h"

namespace cargo {

const int LRU_CACHE_SIZE = 10000;

/* Initialize global vars */
KVNodes Cargo::nodes_ = {};
KVEdges Cargo::edges_ = {};
std::unordered_map<TripId, DistInt> Cargo::trip_costs_ = {};
BoundingBox Cargo::bbox_ = {{}, {}};
GTree::G_Tree Cargo::gtree_ = GTree::get();
sqlite3* Cargo::db_ = nullptr;
Speed Cargo::speed_ = 0;
SimlTime Cargo::t_ = 0;
std::mutex Cargo::dbmx;
std::mutex Cargo::spmx;
cache::lru_cache<std::string, std::vector<NodeId>> Cargo::spcache_(LRU_CACHE_SIZE);

Cargo::Cargo(const Options& opt) : print("cargo") {
  print << "Initializing Cargo\n";
  rng.seed(std::random_device()());
  this->initialize(opt);  // <-- loads db
  if (sqlite3_prepare_v2(db_, sql::tim_stmt, -1, &tim_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sac_stmt, -1, &sac_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sar_stmt2,-1, &sar_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::ssv_stmt2,-1, &ssv_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::ucs_stmt, -1, &ucs_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::uro_stmt2,-1, &uro_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sch_stmt2,-1, &sch_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::dav_stmt, -1, &dav_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::pup_stmt, -1, &pup_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::drp_stmt, -1, &drp_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::vis_stmt, -1, &vis_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::lvn_stmt2,-1, &lvn_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::stc_stmt, -1, &stc_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::nnd_stmt2,-1, &nnd_stmt2,NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::mov_stmt, -1, &mov_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::usc_stmt, -1, &usc_stmt, NULL) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create stmts). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_bind_int(mov_stmt, 1, speed_);
  print(MessageType::Success) << "Cargo initialized!" << std::endl;
}

Cargo::~Cargo() {
  sqlite3_finalize(tim_stmt);
  sqlite3_finalize(sac_stmt);
  sqlite3_finalize(sar_stmt2);
  sqlite3_finalize(ssv_stmt2);
  sqlite3_finalize(ucs_stmt);
  sqlite3_finalize(uro_stmt2);
  sqlite3_finalize(sch_stmt2);
  sqlite3_finalize(dav_stmt);
  sqlite3_finalize(pup_stmt);
  sqlite3_finalize(drp_stmt);
  sqlite3_finalize(vis_stmt);
  sqlite3_finalize(lvn_stmt2);
  sqlite3_finalize(nnd_stmt2);
  sqlite3_finalize(stc_stmt);
  sqlite3_finalize(mov_stmt);
  sqlite3_finalize(usc_stmt);
  if (err != NULL) sqlite3_free(err);
  sqlite3_close(db_);  // <-- calls std::terminate on failure
  print << "Database closed." << std::endl;
}

const std::string & Cargo::name()         { return probset_.name(); }
const std::string & Cargo::road_network() { return probset_.road_network(); }

int Cargo::step(int& ndeact) {
  int nrows = ndeact = 0;

  /* Prepare logger containers */
  log_v_.clear();
  log_p_.clear();
  log_d_.clear();
  log_a_.clear();

  std::lock_guard<std::mutex> dblock(dbmx);  // Lock acquired

  /* Bulk-update next-node distance (bulk-move the vehicles) */
  if (sqlite3_step(mov_stmt) != SQLITE_DONE) {
    print(MessageType::Error) << "mov_stmt failed; reason:" << std::endl;
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  /* Update move events (pickup, dropoff, etc.)
   * (select vehicles where bulk-move resulted in negative nnd) */
  sqlite3_bind_int(ssv_stmt2, 1, t_);
  sqlite3_bind_int(ssv_stmt2, 2, (int)VehlStatus::Arrived);
  while ((rc = sqlite3_step(ssv_stmt2)) == SQLITE_ROW) {
    nrows++;

    sqlite3_exec(db_, "BEGIN", NULL, NULL, &err);

    /* Extract */
    const VehlId vid = sqlite3_column_int(ssv_stmt2, 0);    // id
    const SimlTime vet = sqlite3_column_int(ssv_stmt2, 3);  // early
    const SimlTime vlt = sqlite3_column_int(ssv_stmt2, 4);  // late
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(ssv_stmt2,  8));
    const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(ssv_stmt2, 11));
    const std::vector<Wayp> rte(rtebuf, rtebuf + sqlite3_column_bytes(ssv_stmt2,  8) / sizeof(Wayp));
    const std::vector<Stop> sch(schbuf, schbuf + sqlite3_column_bytes(ssv_stmt2, 11) / sizeof(Stop));
    std::vector<Stop> new_sch = sch;                        // mutable copy
    RteIdx  lvn = sqlite3_column_int(ssv_stmt2, 9);         // last-visited node
    DistInt nnd = sqlite3_column_int(ssv_stmt2, 10);        // next-node dist

    DEBUG(2, {  // Print vehicle info
      print << "t=" << t_ << std::endl;
      print << "Vehicle " << vid << "\n\tearly:\t" << vet << "\n\tlate:\t" << vlt << "\n\tnnd:\t" << nnd << "\n\tlvn:\t" << lvn << "\n";
      print << "\tsched:"; print_sch(sch);
      print << "\troute:"; print_rte(rte);
    });

    bool active = true;  // all vehicles selected by ssv_stmt are active
    int nstops = 0;

    while (nnd <= 0 && active) {  // O(|schedule|)
      lvn++;
      /* Log position */
      log_v_[vid] = rte.at(lvn).second;

      /* Did vehicle move to a stop?
       * (schedule[0] gives the node the vehicle is currently traveling to.
       * Vehicle has moved to it already because nnd <= 0; hence use
       * schedule[1+nstops] to get the next node.) */
      while (active && rte.at(lvn).second == sch.at(1 + nstops).loc()) {
        const Stop& stop = sch.at(1 + nstops);
        nstops++;

        /* Ridesharing vehicle arrives at destination; OR
         * Permanent taxi arrives at destination and no more customers remain */
        if (stop.type() == StopType::VehlDest &&
           (stop.late() != -1 || t_ >= tmin_)) {
          sqlite3_bind_int(dav_stmt, 1, (int)VehlStatus::Arrived);
          sqlite3_bind_int(dav_stmt, 2, vid);
          if (sqlite3_step(dav_stmt) != SQLITE_DONE) {
            print(MessageType::Error) << "Failed (deactivate vehicle " << vid << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else
            DEBUG(1, { print(MessageType::Info) << "Vehicle " << vid << " arrived." << std::endl; });
          sqlite3_clear_bindings(dav_stmt);
          sqlite3_reset(dav_stmt);
          active = false;  // <-- stops the while loops
          ndeact++;
          /* Log arrival */
          log_a_.push_back(stop.owner());

        /* Permanent taxi arrived at "destination"
         * (essentially recreate the taxi) */
        } else if (stop.type() == StopType::VehlDest && stop.late() == -1) {
          NodeId new_dest = random_node();
          Stop a(stop.owner(), stop.loc(), StopType::VehlOrig, stop.early(), -1);
          Stop b(stop.owner(), new_dest  , StopType::VehlDest, stop.early(), -1);
          std::vector<Wayp> route;
          route_through({a, b}, route);

          /* Don't subtract nnd here; the "extra" distance the taxi travels
           * past its destination is lost. It doesn't have a next wp anyway. */
          int new_nnd = route.at(1).first;

          /* Insert the new route */
          sqlite3_bind_blob(uro_stmt2, 1, static_cast<void const*>(route.data()),
                            route.size()*sizeof(Wayp), SQLITE_TRANSIENT);
          sqlite3_bind_int(uro_stmt2, 2, 0);        // lvn
          sqlite3_bind_int(uro_stmt2, 3, new_nnd);  // nnd
          sqlite3_bind_int(uro_stmt2, 4, stop.owner());
          if (sqlite3_step(uro_stmt2) != SQLITE_DONE) {
            print(MessageType::Error) << "Failure at route " << stop.owner() << "\n";
            print(MessageType::Error) << "Failed (insert route). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          }
          sqlite3_clear_bindings(uro_stmt2);
          sqlite3_reset(uro_stmt2);

          /* Insert schedule */
          Stop next_loc(stop.owner(), route.at(1).second, StopType::VehlOrig, stop.early(), -1);
          std::vector<Stop> sch{next_loc, b};
          sqlite3_bind_blob(sch_stmt2, 1, static_cast<void const*>(sch.data()),
                            sch.size() * sizeof(Stop), SQLITE_TRANSIENT);
          sqlite3_bind_int(sch_stmt2, 2, stop.owner());
          if (sqlite3_step(sch_stmt2) != SQLITE_DONE) {
            print(MessageType::Error) << "Failure at schedule " << stop.owner() << "\n";
            print(MessageType::Error) << "Failed (insert schedule). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          }
          sqlite3_clear_bindings(sch_stmt2);
          sqlite3_reset(sch_stmt2);

          active = false; // stop the loop

        /* Vehicle arrives at a pickup */
        } else if (stop.type() == StopType::CustOrig) {
          sqlite3_bind_int(pup_stmt, 1, vid);
          sqlite3_bind_int(ucs_stmt, 1, (int)CustStatus::Onboard);
          sqlite3_bind_int(ucs_stmt, 2, stop.owner());
          if (sqlite3_step(pup_stmt) != SQLITE_DONE ||
              sqlite3_step(ucs_stmt) != SQLITE_DONE) {
            print(MessageType::Error) << "Failed (veh" << vid << " pickup " << stop.owner() << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else {
            /* Log pickup */
            log_p_.push_back(stop.owner());
            DEBUG(1, { print(MessageType::Info)
              << "Vehicle " << vid << " picked up Customer "
              << stop.owner() << "(" << stop.loc() << ")" << std::endl; });
          }
          sqlite3_clear_bindings(pup_stmt);
          sqlite3_clear_bindings(ucs_stmt);
          sqlite3_reset(pup_stmt);
          sqlite3_reset(ucs_stmt);

        /* Vehicle arrived at dropoff */
        } else if (stop.type() == StopType::CustDest) {
          sqlite3_bind_int(drp_stmt, 1, vid);
          sqlite3_bind_int(ucs_stmt, 1, (int)CustStatus::Arrived);
          sqlite3_bind_int(ucs_stmt, 2, stop.owner());
          if (sqlite3_step(drp_stmt) != SQLITE_DONE ||
              sqlite3_step(ucs_stmt) != SQLITE_DONE) {
            print(MessageType::Error) << "Failed (veh" << vid << " dropoff " << stop.owner() << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else {
            /* Log dropoff */
            log_d_.push_back(stop.owner());
            DEBUG(1, { print(MessageType::Info)
              << "Vehicle " << vid << " dropped off Customer "
              << stop.owner() << "(" << stop.loc() << ")" << std::endl; });
          }
          sqlite3_clear_bindings(drp_stmt);
          sqlite3_clear_bindings(ucs_stmt);
          sqlite3_reset(drp_stmt);
          sqlite3_reset(ucs_stmt);
        }
        /* Update visitedAt (used for avg. delay statistics) */
        sqlite3_bind_int(vis_stmt, 1, t_);
        sqlite3_bind_int(vis_stmt, 2, stop.owner());
        sqlite3_bind_int(vis_stmt, 3, stop.loc());
        if (sqlite3_step(vis_stmt) != SQLITE_DONE) {
          print(MessageType::Error) << "Failed (update visitedAt for stop " << stop.owner() << " at " << stop.loc() << "). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(vis_stmt);
        sqlite3_reset(vis_stmt);
      }  // end inner while
      if (active) nnd += (rte.at(lvn + 1).first - rte.at(lvn).first);
    }  // end outer while

    if (active) {
      if (nstops > 0) {
        /* Update schedule (remove the just-visited stops) */
        new_sch.erase(new_sch.begin() + 1, new_sch.begin() + 1 + nstops);
      }
      /* Update schedule[0] (set to be the next node in the route) */
      new_sch[0] = Stop(vid, rte.at(lvn + 1).second, StopType::VehlOrig, vet, vlt, t_);

      /* Commit the schedule, lvn, and nnd after motion */
      sqlite3_bind_blob(usc_stmt, 1,
        static_cast<void const*>(new_sch.data()),new_sch.size()*sizeof(Stop),SQLITE_TRANSIENT);
      sqlite3_bind_int(usc_stmt, 2, lvn);
      sqlite3_bind_int(usc_stmt, 3, nnd);
      sqlite3_bind_int(usc_stmt, 4, vid);
      if (sqlite3_step(usc_stmt) != SQLITE_DONE) {
        print(MessageType::Error) << "Failed (update schedule for vehicle " << vid << "). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }

      /* Kill permanent taxis after all customers have appeared and
       * taxi has no more dropoffs to make */
      if (vlt == -1 && new_sch.size() == 2 && t_ >= tmin_) {
        sqlite3_bind_int(dav_stmt, 1, (int)VehlStatus::Arrived);
        sqlite3_bind_int(dav_stmt, 2, vid);
        if (sqlite3_step(dav_stmt) != SQLITE_DONE) {
          print(MessageType::Error) << "Failed (deactivate vehicle " << vid << "). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        } else
          DEBUG(1, { print(MessageType::Info) << "Vehicle " << vid << " arrived." << std::endl; });
        sqlite3_clear_bindings(dav_stmt);
        sqlite3_reset(dav_stmt);
        ndeact++;
      }
    }  // end active
    sqlite3_exec(db_, "END", NULL, NULL, &err);
  }    // end SQLITE_ROW
  if (rc != SQLITE_DONE) {
    print(MessageType::Error) << "Failure in select step vehicles. Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_clear_bindings(ssv_stmt2);
  sqlite3_reset(ssv_stmt2);

  /* Record events */
  if (!log_p_.empty()) Logger::put_p_message(log_p_);
  if (!log_d_.empty()) Logger::put_d_message(log_d_);
  if (!log_v_.empty()) Logger::put_v_message(log_v_);
  if (!log_a_.empty()) Logger::put_a_message(log_a_);

  return nrows;
}  // release dblock

/* Returns cost of all vehicle routes, plus the base cost for each
 * unassigned customer trip */
DistInt Cargo::total_route_cost() {
  DistInt cst = 0;
  while ((rc = sqlite3_step(sar_stmt2)) == SQLITE_ROW) {
    const Wayp* rtebuf = static_cast<const Wayp*>(sqlite3_column_blob(sar_stmt2, 0));
    const std::vector<Wayp> route(rtebuf, rtebuf + sqlite3_column_bytes(sar_stmt2, 0) / sizeof(Wayp));
    cst += route.back().first;
  }
  if (rc != SQLITE_DONE) {
    print(MessageType::Error) << "Failure in select all routes. Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(sar_stmt2);

  while ((rc = sqlite3_step(sac_stmt)) == SQLITE_ROW) {
    const CustId cust_id = sqlite3_column_int(sac_stmt, 0);
    const VehlId assigned_to = sqlite3_column_int(sac_stmt, 7);
    if (assigned_to == 0) cst += trip_costs_.at(cust_id);
  }
  return cst;
}

SimlDur Cargo::avg_pickup_delay() {
  /* Get all origin stops belonging to assigned customers
   * to find (visitedAt - early) */
  SimlDur pdelay = 0;
  int count = 0;
  std::string querystr = "select * from stops where type = "
      + std::to_string((int)StopType::CustOrig) + " and "
      + "exists (select id from customers "
      + "where customers.id = stops.owner and customers.assignedTo not null);";
  sqlite3_stmt* stmt;
  sqlite3_prepare_v2(db_, querystr.c_str(), -1, &stmt, NULL);
  while ((rc = sqlite3_step(stmt)) == SQLITE_ROW) {
    const SimlTime early = sqlite3_column_int(stmt, 3);
    const SimlTime visitedAt = sqlite3_column_int(stmt, 5);
    pdelay += (visitedAt - early);
    count++;
  }
  if (rc != SQLITE_DONE) {
    print(MessageType::Error) << "Failure in avg_pickup_delay(). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(stmt);
  sqlite3_finalize(stmt);

  return count == 0 ? -1 : pdelay/count; // int
}

SimlDur Cargo::avg_trip_delay() {
  /* Get all orig, dest stops belonging to assigned customers
   * to find (dest.visitedAt - orig.visitedAt - base cost) */
  SimlDur tdelay = 0;
  std::unordered_map<CustId, SimlTime> orig_t;
  std::unordered_map<CustId, SimlTime> dest_t;
  std::vector<CustId> keys;
  std::string querystr = "select * from stops where type = "
      + std::to_string((int)StopType::CustOrig) + " and " // <-- origs
      + "exists (select id from customers "
      + "where customers.id = stops.owner and customers.assignedTo not null);";
  sqlite3_stmt* stmt1;
  sqlite3_prepare_v2(db_, querystr.c_str(), -1, &stmt1, NULL);
  while ((rc = sqlite3_step(stmt1)) == SQLITE_ROW) {
    const CustId cust_id = sqlite3_column_int(stmt1, 0);
    const SimlTime visitedAt = sqlite3_column_int(stmt1, 5);
    orig_t[cust_id] = visitedAt;
    keys.push_back(cust_id);
  }
  if (rc != SQLITE_DONE) {
    print(MessageType::Error) << "Failure in avg_trip_delay(). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(stmt1);
  sqlite3_finalize(stmt1);
  querystr = "select * from stops where type = "
      + std::to_string((int)StopType::CustDest) + " and " // <-- dests
      + "exists (select id from customers "
      + "where customers.id = stops.owner and customers.assignedTo not null);";
  sqlite3_stmt* stmt2;
  sqlite3_prepare_v2(db_, querystr.c_str(), -1, &stmt2, NULL);
  while ((rc = sqlite3_step(stmt2)) == SQLITE_ROW) {
    const CustId cust_id = sqlite3_column_int(stmt2, 0);
    const SimlTime visitedAt = sqlite3_column_int(stmt2, 5);
    dest_t[cust_id] = visitedAt;
  }
  if (rc != SQLITE_DONE) {
    print(MessageType::Error) << "Failure in avg_trip_delay(). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(stmt2);
  sqlite3_finalize(stmt2);

  for (const CustId& cust_id : keys)
    tdelay += ((dest_t.at(cust_id) - orig_t.at(cust_id)))
              - trip_costs_.at(cust_id)/speed_;

  return keys.size() == 0 ? -1 : tdelay/keys.size(); // int
}

NodeId Cargo::random_node() {
  int bucket, bucket_size;
  do {
    std::uniform_int_distribution<> dis1(0,nodes_.bucket_count()-1);
    bucket = dis1(rng);
  } while ((bucket_size = nodes_.bucket_size(bucket)) == 0);

  std::uniform_int_distribution<> dis2(0,bucket_size-1);
  NodeId res = -1;
  do res = std::next(nodes_.begin(bucket), dis2(rng))->first;
  while (res == -1);
  return res;
}

/* Start Cargo with the default (blank) RSAlgorithm */
void Cargo::start() {
  RSAlgorithm _;
  start(_);
}

void Cargo::start(RSAlgorithm& rsalg) {
  print << "Starting Cargo\n";
  print << "Starting algorithm " << rsalg.name() << "\n";

  /* Algorithm thread */
  std::thread thread_rsalg([&rsalg]() { while (!rsalg.done()) {
      rsalg.listen();
  }});

  /* Logger thread */
  Logger logger(dataout_file_);
  std::thread logger_thread([&logger]() { logger.run(); });

  /* Cargo thread
   * (Don't call any rsalg methods here) */
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  int ndeact, nstepped, dur;
  while (active_vehicles_ > 0 || t_ <= tmin_) {
    t0 = std::chrono::high_resolution_clock::now();

    /* Timeout customers where t_ > early + matching_period_ */
    // Log timed-out customers
    std::vector<CustId> timeout;
    sqlite3_bind_int(stc_stmt, 1, t_);
    sqlite3_bind_int(stc_stmt, 2, matp_);
    sqlite3_bind_int(stc_stmt, 3, (int)CustStatus::Canceled);
    while ((rc = sqlite3_step(stc_stmt)) == SQLITE_ROW) {
      timeout.push_back(sqlite3_column_int(stc_stmt, 0));
    }
    sqlite3_clear_bindings(stc_stmt);
    sqlite3_reset(stc_stmt);
    if (timeout.size() != 0) Logger::put_t_message(timeout);

    // Update timeout customers
    sqlite3_bind_int(tim_stmt, 1, (int)CustStatus::Canceled);
    sqlite3_bind_int(tim_stmt, 2, t_);
    sqlite3_bind_int(tim_stmt, 3, matp_);
    if (sqlite3_step(tim_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "Failed to timeout customers. Reason:\n";
      throw std::runtime_error(sqlite3_errmsg(db_));
    }
    DEBUG(1, { print << "(Timed out " << sqlite3_changes(db_) << " customers)\n"; });
    sqlite3_clear_bindings(tim_stmt);
    sqlite3_reset(tim_stmt);

    /* Step the vehicles */
    nstepped = step(ndeact);
    active_vehicles_ -= ndeact;
    print << "t=" << t_ << "; stepped " << nstepped
          << " vehicles; remaining=" << active_vehicles_ << ";" << std::endl;

    t1 = std::chrono::high_resolution_clock::now();

    dur = std::round(std::chrono::duration<double, std::milli>(t1 - t0).count());
    if (dur > sleep_interval_)
      print(MessageType::Warning) << "step() (" << dur << " ms) exceeds interval ("
                    << sleep_interval_ << " ms)\n";
    else
      std::this_thread::sleep_for(
          std::chrono::milliseconds(sleep_interval_ - dur));

    /* Increment the time step */
    t_ += 1;
  }  // end Cargo thread

  rsalg.kill();
  rsalg.end();  // <-- user-defined
  thread_rsalg.join();
  print << "Finished algorithm " << rsalg.name() << std::endl;

  /* Stop logger */
  print << "Logger stopped" << std::endl;
  logger.stop();
  logger_thread.join();

  avg_pickup_delay();

  /* Print solution
   * (e.g. # matches, avg. delay statistics, other statistics) */
  std::ofstream f_sol_(solution_file_, std::ios::out);
  f_sol_ << name() << '\n'
         << road_network() << '\n'
         << "VEHICLES " << total_vehicles_ << '\n'
         << "CUSTOMERS " << total_customers_ << '\n'
         << "base cost " << base_cost_ << '\n'
         << "solution cost " << total_route_cost() << '\n'
         << "avg. pickup delay " << avg_pickup_delay() << '\n'
         << "avg. trip delay " << avg_trip_delay() << '\n';
  f_sol_.close();

  print << "Finished Cargo" << std::endl;
}

void Cargo::initialize(const Options& opt) {
  total_customers_ = total_vehicles_ = base_cost_ = 0;

  print << "Starting initialization sequence\n";
  print << "Reading nodes (" << opt.path_to_roadnet << ")... ";
  const size_t nnodes = read_nodes(opt.path_to_roadnet, nodes_, bbox_);
  nodes_[-1] = {-1, -1}; // special "no destination" node
  print << nnodes << "\n";
  print << "\tBounding box: "
        << "(" << bbox().lower_left.lng << "," << bbox().lower_left.lat  << "), "
        << "(" << bbox().upper_right.lng << ","<< bbox().upper_right.lat << ")\n";

  print << "Reading edges (" << opt.path_to_edges << ")... ";
  const size_t nedges = read_edges(opt.path_to_edges, edges_);
  print << nedges << "\n";

  print << "Reading gtree (" << opt.path_to_gtree << ")... ";
  GTree::load(opt.path_to_gtree);
  gtree_ = GTree::get();
  print << "Done\n";

  print << "Reading problem (" << opt.path_to_problem << ")... ";
  const size_t ntrips = read_problem(opt.path_to_problem, probset_);
  if (ntrips == 0) {
    print(MessageType::Error) << "Problem file has no trips!\n";
    throw std::runtime_error("bad");
  }
  print << ntrips << "\n";
  print << "\t" << name() << " on " << road_network() << "\n";

  tmin_ = tmax_ = 0;
  matp_ = opt.matching_period;
  sleep_interval_ = std::round((float)1000 / opt.time_multiplier);
  speed_ = opt.vehicle_speed;

  print << "Creating in-memory database...\n";
  if (sqlite3_open(":memory:", &db_) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create db). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  /* Enable foreign keys */
  if (sqlite3_db_config(db_, SQLITE_DBCONFIG_ENABLE_FKEY, 1, NULL) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (enable foreign keys). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  /* Performance enhancements */
  sqlite3_exec(db_, "PRAGMA synchronous = OFF", NULL, NULL, &err);
  sqlite3_exec(db_, "PRAGMA journal_mode = OFF", NULL, NULL, &err);
  sqlite3_exec(db_, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, &err);

  print << "\t Creating Cargo tables...";
  if (sqlite3_exec(db_, sql::create_cargo_tables, NULL, NULL, &err) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create cargo tables). Reason: " << err << "\n";
    print << sql::create_cargo_tables << "\n";
    throw std::runtime_error("create cargo tables failed.");
  }
  print << "Done\n";

  print << "\t Inserting nodes...";
  sqlite3_exec(db_, "BEGIN", NULL, NULL, &err);
  sqlite3_stmt* insert_node_stmt;
  if (sqlite3_prepare_v2(db_, "insert into nodes values(?, ?, ?);", -1, &insert_node_stmt, NULL) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create insert node stmt). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  for (const auto& kv : nodes_) {
    sqlite3_bind_int(insert_node_stmt, 1, kv.first);
    sqlite3_bind_double(insert_node_stmt, 2, kv.second.lng);
    sqlite3_bind_double(insert_node_stmt, 3, kv.second.lat);
    if (sqlite3_step(insert_node_stmt) != SQLITE_DONE) {
      print(MessageType::Error) << "Failure at node " << kv.first << "\n";
      print(MessageType::Error) << "Failed (insert nodes). Reason:\n";
      throw std::runtime_error(sqlite3_errmsg(db_));
    }
    sqlite3_clear_bindings(insert_node_stmt);
    sqlite3_reset(insert_node_stmt);
  }
  sqlite3_finalize(insert_node_stmt);
  sqlite3_exec(db_, "END", NULL, NULL, &err);
  print << "Done\n";

  print << "\t Inserting trips...";
  sqlite3_stmt* insert_vehicle_stmt;
  sqlite3_stmt* insert_customer_stmt;
  sqlite3_stmt* insert_stop_stmt;
  if (sqlite3_prepare_v2(db_, "insert into vehicles values(?,?,?,?,?,?,?,?,?,?,?,?);", -1, &insert_vehicle_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, "insert into customers values(?, ?, ?, ?, ?, ?, ?, ?);", -1, &insert_customer_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, "insert into stops values(?, ?, ?, ?, ?, ?);", -1, &insert_stop_stmt, NULL) != SQLITE_OK) {
    print(MessageType::Error) << "Failed (create insert trip stmts). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  for (const auto& kv : probset_.trips()) {
    for (const auto& trip : kv.second) {
      StopType stop_type = StopType::CustOrig;  // default

      /* Insert vehicle (negative load) */
      if (trip.load() < 0) {
        total_vehicles_++;
        stop_type = StopType::VehlOrig;

        /* Sanity check */
        if (trip.dest() == -1 && trip.late() != -1) {
          print(MessageType::Error) << "Vehicle " << trip.id() << " has a late window with no destination\n";
          throw std::runtime_error("bad vehicle");
        }

        /* Compute initial route */
        NodeId trip_dest = (trip.dest() == -1 ? random_node() : trip.dest());
        Stop a(trip.id(), trip.orig(), StopType::VehlOrig, trip.early(), trip.late(), trip.early());
        Stop b(trip.id(), trip_dest  , StopType::VehlDest, trip.early(), trip.late());
        std::vector<Wayp> rte;
        DistInt cost = route_through({a,b}, rte);

        /* Initialize vehicle schedule */
        Stop next_loc(trip.id(), rte.at(1).second, StopType::VehlOrig, trip.early(), trip.late());
        std::vector<Stop> sch{next_loc,b};

        /* Insert to database */
        sqlite3_bind_int(insert_vehicle_stmt, 1, trip.id());
        sqlite3_bind_int(insert_vehicle_stmt, 2, trip.orig());
        sqlite3_bind_int(insert_vehicle_stmt, 3, trip.dest());
        sqlite3_bind_int(insert_vehicle_stmt, 4, trip.early());
        sqlite3_bind_int(insert_vehicle_stmt, 5, trip.late());
        sqlite3_bind_int(insert_vehicle_stmt, 6, trip.load());
        sqlite3_bind_int(insert_vehicle_stmt, 7, 0);
        sqlite3_bind_int(insert_vehicle_stmt, 8, (int)VehlStatus::Enroute);
        sqlite3_bind_blob(insert_vehicle_stmt, 9,
          static_cast<void const*>(rte.data()),rte.size()*sizeof(Wayp),SQLITE_TRANSIENT);
        sqlite3_bind_int(insert_vehicle_stmt,10, 0);
        sqlite3_bind_int(insert_vehicle_stmt,11, rte.at(1).first);
        sqlite3_bind_blob(insert_vehicle_stmt,12,
          static_cast<void const*>(sch.data()),sch.size()*sizeof(Stop),SQLITE_TRANSIENT);
        if (sqlite3_step(insert_vehicle_stmt) != SQLITE_DONE) {
          print(MessageType::Error) << "Failure at vehicle " << trip.id() << "\n";
          print(MessageType::Error) << "Failed (insert vehicle). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(insert_vehicle_stmt);
        sqlite3_reset(insert_vehicle_stmt);

        /* Record base cost (for rs vehicles) */
        if (trip.dest() == -1) cost = 0;
        base_cost_ += cost;
        trip_costs_[trip.id()] = cost;

      /* Insert customer (positive load) */
      } else if (trip.load() > 0) {
        total_customers_++;

        /* Compute initial route */
        Stop a(trip.id(), trip.orig(), StopType::CustOrig, trip.early(), trip.late(), trip.early());
        Stop b(trip.id(), trip.dest(), StopType::CustDest, trip.early(), trip.late());
        std::vector<Wayp> rte;
        DistInt cost = route_through({a, b}, rte);

        /* Record base cost */
        base_cost_ += cost;
        trip_costs_[trip.id()] = cost;
        stop_type = StopType::CustOrig;

        /* Insert to database */
        sqlite3_bind_int(insert_customer_stmt, 1, trip.id());
        sqlite3_bind_int(insert_customer_stmt, 2, trip.orig());
        sqlite3_bind_int(insert_customer_stmt, 3, trip.dest());
        sqlite3_bind_int(insert_customer_stmt, 4, trip.early());
        sqlite3_bind_int(insert_customer_stmt, 5, trip.late());
        sqlite3_bind_int(insert_customer_stmt, 6, trip.load());
        sqlite3_bind_int(insert_customer_stmt, 7, (int)CustStatus::Waiting);
        sqlite3_bind_null(insert_customer_stmt, 8);
        if (sqlite3_step(insert_customer_stmt) != SQLITE_DONE) {
          print(MessageType::Error) << "Failure at customer " << trip.id() << "\n";
          print(MessageType::Error) << "Failed (insert customer). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(insert_customer_stmt);
        sqlite3_reset(insert_customer_stmt);

      /* Insert small "customers", e.g. mail, packages (zero load) */
      } else {
        // placeholder
        print(MessageType::Warning) << "Trip" << trip.id() << " load == 0\n";
      }

      /* Insert origin */
      sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
      sqlite3_bind_int(insert_stop_stmt, 2, trip.orig());
      sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type);
      sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
      sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
      sqlite3_bind_int(insert_stop_stmt, 6, trip.early());
      if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
        print(MessageType::Error) << "Failure at stop " << trip.orig() << "\n";
        print(MessageType::Error) << "Failed (insert stop). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      sqlite3_clear_bindings(insert_stop_stmt);
      sqlite3_reset(insert_stop_stmt);

      /* Insert destination */
      sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
      sqlite3_bind_int(insert_stop_stmt, 2, trip.dest());
      sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type + 1);
      sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
      sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
      sqlite3_bind_null(insert_stop_stmt, 6);
      if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
        print(MessageType::Error) << "Failure at stop " << trip.dest() << "\n";
        print(MessageType::Error) << "Failed (insert stop). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      sqlite3_clear_bindings(insert_stop_stmt);
      sqlite3_reset(insert_stop_stmt);

      /* Get tmin_, tmax_ */
      tmin_ = std::max(trip.early(), tmin_);
      tmax_ = std::max(trip.late(), tmax_);
    }
  }
  sqlite3_exec(db_, "END TRANSACTION;", NULL, NULL, &err);

  active_vehicles_ = total_vehicles_;

  // Minimum sim time equals time of last trip appearing, plus matching pd.
  tmin_ += matp_;

  sqlite3_finalize(insert_vehicle_stmt);
  sqlite3_finalize(insert_customer_stmt);
  sqlite3_finalize(insert_stop_stmt);

  solution_file_ = opt.path_to_solution;
  dataout_file_ = opt.path_to_dataout;

  t_ = 0;  // Ready to begin!
  print << "Done\n";
  print << "Finished initialization sequence\n";
}

}  // namespace cargo

