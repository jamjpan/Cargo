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
#include <cstdio> /* std::remove */
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "libcargo/cargo.h"
#include "libcargo/classes.h"
#include "libcargo/dbsql.h"
#include "libcargo/file.h"
#include "libcargo/functions.h"
#include "libcargo/logger.h"
#include "libcargo/message.h"
#include "libcargo/options.h"
#include "libcargo/rsalgorithm.h"
#include "libcargo/types.h"

#include "gtree/gtree.h"
#include "sqlite3/sqlite3.h"

/* Usage: DEBUG(int, stmt) */
#define DEBUG(level, x)                  \
  do {                                   \
    if (level && debug_flag >= level) x; \
  } while (0)

namespace cargo {

const int debug_flag = (int)DebugFlag::Level1;

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
std::mutex Message::mtx_;

// ENHANCEMENT: just one message object, allowing an option
//   e.g. Message print("cargo");              // construct the object
//        print << "text\n"                    // print default text
//        print(MessageType::Info) << "text\n" // same object, different type
Cargo::Cargo(const Options& opt)
    : print_out("cargo"),
      print_info(MessageType::Info, "cargo"),
      print_warning(MessageType::Warning, "cargo"),
      print_error(MessageType::Error, "cargo"),
      print_success(MessageType::Success, "cargo"),
      f_sol_temp_("sol.partial", std::ios::out)  // <-- needed for writing sol
{
  print_out << "Initializing Cargo\n";
  this->initialize(opt);  // <-- load db
  if (sqlite3_prepare_v2(db_, sql::tim_stmt, -1, &tim_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sac_stmt, -1, &sac_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sar_stmt, -1, &sar_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::ssv_stmt, -1, &ssv_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::ucs_stmt, -1, &ucs_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::dav_stmt, -1, &dav_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::pup_stmt, -1, &pup_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::drp_stmt, -1, &drp_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::vis_stmt, -1, &vis_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::sch_stmt, -1, &sch_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::lvn_stmt, -1, &lvn_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::nnd_stmt, -1, &nnd_stmt, NULL) !=
          SQLITE_OK ||
      sqlite3_prepare_v2(db_, sql::select_timeout_stmt, -1,
                         &select_timeout_stmt, NULL) != SQLITE_OK) {
    print_error << "Failed (create step stmts). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  print_success << "Cargo initialized!" << std::endl;
}

Cargo::~Cargo() {
  sqlite3_finalize(tim_stmt);
  sqlite3_finalize(sac_stmt);
  sqlite3_finalize(sar_stmt);
  sqlite3_finalize(ssv_stmt);
  sqlite3_finalize(ucs_stmt);
  sqlite3_finalize(dav_stmt);
  sqlite3_finalize(pup_stmt);
  sqlite3_finalize(drp_stmt);
  sqlite3_finalize(vis_stmt);
  sqlite3_finalize(sch_stmt);
  sqlite3_finalize(lvn_stmt);
  sqlite3_finalize(nnd_stmt);
  sqlite3_finalize(select_timeout_stmt);
  if (err != NULL) sqlite3_free(err);
  sqlite3_close(db_);  // <-- calls std::terminate on failure
  print_out << "Database closed." << std::endl;
  std::remove("sol.partial");
}

const std::string& Cargo::name() { return probset_.name(); }
const std::string& Cargo::road_network() { return probset_.road_network(); }

int Cargo::step(int& ndeact) {
  int nrows = ndeact = 0;

  // Picked up customers for logger
  std::vector<CustId> picked;
  // Dropped off customers for logger
  std::vector<CustId> dropped;
  // Vehicle postion update for logger
  std::map<VehlId, NodeId> vehicle_position;
  // Vehicle arrived update for logger
  std::vector<VehlId> arrived;

  std::lock_guard<std::mutex> dblock(dbmx);  // Lock acquired

  sqlite3_bind_int(ssv_stmt, 1, t_);
  sqlite3_bind_int(ssv_stmt, 2, (int)VehlStatus::Arrived);

  while ((rc = sqlite3_step(ssv_stmt)) == SQLITE_ROW) {  // O(|vehicles|)
    DEBUG(3, {                                           // Print column headers
      for (int i = 0; i < sqlite3_column_count(ssv_stmt); ++i)
        print_info << "[" << i << "] " << sqlite3_column_name(ssv_stmt, i)
                   << "\n";
    });

    nrows++;

    /* Extract */
    const VehlId vid = sqlite3_column_int(ssv_stmt, 0);    // id
    const SimlTime vet = sqlite3_column_int(ssv_stmt, 3);  // early
    const SimlTime vlt = sqlite3_column_int(ssv_stmt, 4);  // late
    const Wayp* rtebuf =
        static_cast<const Wayp*>(sqlite3_column_blob(ssv_stmt, 9));
    const Stop* schbuf =
        static_cast<const Stop*>(sqlite3_column_blob(ssv_stmt, 13));
    const std::vector<Wayp> rte(
        rtebuf, rtebuf + sqlite3_column_bytes(ssv_stmt, 9) / sizeof(Wayp));
    const std::vector<Stop> sch(
        schbuf, schbuf + sqlite3_column_bytes(ssv_stmt, 13) / sizeof(Stop));

    std::vector<Stop> new_sch = sch;                // mutable
    RteIdx lvn = sqlite3_column_int(ssv_stmt, 10);  // last-visited node
    DistInt nnd = sqlite3_column_int(ssv_stmt, 11) - speed_;

    DEBUG(2, {  // Print vehicle info
      print_out << "Vehicle " << vid << "\n\tearly:\t" << vet << "\n\tlate:\t"
                << vlt << "\n\tnnd:\t" << nnd << "\n\tlvn:\t" << lvn;
      print_out << "\n\tsched:";
      for (const Stop& sp : sch) print_out << " " << sp.loc();
      print_out << "\n\troute:";
      for (const Wayp& wp : rte)
        print_out << " (" << wp.first << "|" << wp.second << ")";
      print_out << std::endl;
    });

    bool active = true;  // all vehicles selected by ssv_stmt are active
    bool moved = (nnd <= 0) ? true : false;
    int nstops = 0;

    while (nnd <= 0 && active) {  // O(|schedule|)
      lvn++;
      // Set new position
      vehicle_position[vid] = rte.at(lvn).second;

      /* Did vehicle move to a stop?
       * (schedule[0] gives the node the vehicle is currently traveling
       * to. Vehicle has moved to it already because nnd <= 0; hence use
       * schedule[1+nstops] to get the next node.) */
      while (active && rte.at(lvn).second == sch.at(1 + nstops).loc()) {
        const Stop& stop = sch.at(1 + nstops);
        nstops++;
        if (stop.type() == StopType::VehlDest) {  // At dest
          sqlite3_bind_int(dav_stmt, 1, (int)VehlStatus::Arrived);
          sqlite3_bind_int(dav_stmt, 2, vid);
          if (sqlite3_step(dav_stmt) != SQLITE_DONE) {
            print_error << "Failed (deactivate vehicle " << vid
                        << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else {
            // Add arrived vehicles
            arrived.push_back(vid);

            DEBUG(1, {
              print_info << "Vehicle " << vid << " arrived." << std::endl;
            });
          }
          sqlite3_clear_bindings(dav_stmt);
          sqlite3_reset(dav_stmt);
          active = false;  // <-- stops the while loops
          ndeact++;
          sol_routes_[vid] = rte.at(lvn).second;  // Record

        } else if (stop.type() == StopType::CustOrig) {  // At pickup
          sqlite3_bind_int(pup_stmt, 1, vid);
          sqlite3_bind_int(ucs_stmt, 1, (int)CustStatus::Onboard);
          sqlite3_bind_int(ucs_stmt, 2, stop.owner());
          if (sqlite3_step(pup_stmt) != SQLITE_DONE ||
              sqlite3_step(ucs_stmt) != SQLITE_DONE) {
            print_error << "Failed (veh" << vid << " pickup " << stop.owner()
                        << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else {
            // Add picked up customers
            picked.push_back(stop.owner());

            DEBUG(1, {
              print_info << "Vehicle " << vid << " picked up Customer "
                         << stop.owner() << "(" << stop.loc() << ")"
                         << std::endl;
            });
          }
          sqlite3_clear_bindings(pup_stmt);
          sqlite3_clear_bindings(ucs_stmt);
          sqlite3_reset(pup_stmt);
          sqlite3_reset(ucs_stmt);

        } else if (stop.type() == StopType::CustDest) {  // At dropoff
          sqlite3_bind_int(drp_stmt, 1, vid);
          sqlite3_bind_int(ucs_stmt, 1, (int)CustStatus::Arrived);
          sqlite3_bind_int(ucs_stmt, 2, stop.owner());
          if (sqlite3_step(drp_stmt) != SQLITE_DONE ||
              sqlite3_step(ucs_stmt) != SQLITE_DONE) {
            print_error << "Failed (veh" << vid << " dropoff " << stop.owner()
                        << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
          } else {
            // Add dropped off customers
            dropped.push_back(stop.owner());

            DEBUG(1, {
              print_info << "Vehicle " << vid << " dropped off Customer "
                         << stop.owner() << "(" << stop.loc() << ")"
                         << std::endl;
            });
          }
          sqlite3_clear_bindings(drp_stmt);
          sqlite3_clear_bindings(ucs_stmt);
          sqlite3_reset(drp_stmt);
          sqlite3_reset(ucs_stmt);
        }
        /* Update visitedAt
         * (is visitedAt ever used??) */
        sqlite3_bind_int(vis_stmt, 1, t_);
        sqlite3_bind_int(vis_stmt, 2, stop.owner());
        sqlite3_bind_int(vis_stmt, 3, stop.loc());
        if (sqlite3_step(vis_stmt) != SQLITE_DONE) {
          print_error << "Failed (update visitedAt for stop " << stop.owner()
                      << " at " << stop.loc() << "). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(vis_stmt);
        sqlite3_reset(vis_stmt);
      }  // end inner while
      // DEBUGGGGG
      // if (active) {
      // try {
      //   int x = rte.at(lvn+1).first;
      //   int y = rte.at(lvn).first;
      // } catch (...) {
      //   print_out << std::endl;
      //   print_error << "rte.at(lvn+1) / rte.at(lvn) failed" << std::endl;

      //   print_out << "Vehicle " << vid << "\n\tearly:\t" << vet <<
      //   "\n\tlate:\t"
      //             << vlt << "\n\tnnd:\t" << nnd << "\n\tlvn:\t" << lvn;
      //   print_out << "\n\tsched:";
      //   for (const Stop& sp : sch) print_out << " (" << sp.loc() << "|" <<
      //   (int)sp.type() << ")"; print_out << "\n\troute:"; for (const Wayp& wp
      //   : rte)
      //     print_out << " (" << wp.first << "|" << wp.second << ")";
      //   print_out << std::endl;
      //   throw std::runtime_error("bad");
      // }
      // }
      if (active) nnd += (rte.at(lvn + 1).first - rte.at(lvn).first);
    }  // end outer while

    if (active) {
      if (nstops > 0) {
        /* Update schedule
         * (remove the just-visited stops) */
        new_sch.erase(new_sch.begin() + 1, new_sch.begin() + 1 + nstops);
      }
      if (moved) {
        /* Update schedule[0]
         * (set to be the next node in the route) */
        new_sch[0] =
            Stop(vid, rte.at(lvn + 1).second, StopType::VehlOrig, vet, vlt, t_);

        /* Commit the new schedule */
        sqlite3_bind_blob(sch_stmt, 1, static_cast<void const*>(new_sch.data()),
                          new_sch.size() * sizeof(Stop), SQLITE_TRANSIENT);
        sqlite3_bind_int(sch_stmt, 2, vid);
        if (sqlite3_step(sch_stmt) != SQLITE_DONE) {
          print_error << "Failed (update schedule for vehicle " << vid
                      << "). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(sch_stmt);
        sqlite3_reset(sch_stmt);

        /* Update idx_last_visited_node */
        sqlite3_bind_int(lvn_stmt, 1, lvn);
        sqlite3_bind_int(lvn_stmt, 2, vid);
        if (sqlite3_step(lvn_stmt) != SQLITE_DONE) {
          print_error << "Failed (update idx lvn for vehicle " << vid
                      << "). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sqlite3_clear_bindings(lvn_stmt);
        sqlite3_reset(lvn_stmt);
      }  // end moved
      /* Update next_node_distance */
      sqlite3_bind_int(nnd_stmt, 1, nnd);
      sqlite3_bind_int(nnd_stmt, 2, vid);
      if (sqlite3_step(nnd_stmt) != SQLITE_DONE) {
        print_error << "Failed (update nnd for vehicle " << vid
                    << "). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      sqlite3_clear_bindings(nnd_stmt);
      sqlite3_reset(nnd_stmt);

      /* Record vehicle status
       * (taken at the end of t_) */
      sol_routes_[vid] = rte.at(lvn).second;
      // Test logger: route index
      // logger_routes_[vid] = lvn;

    }  // end active
  }    // end SQLITE_ROW
  if (rc != SQLITE_DONE) {
    print_error << "Failure in select step vehicles. Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_clear_bindings(ssv_stmt);
  sqlite3_reset(ssv_stmt);

  // Log pickup and dropoff
  if (picked.size() != 0) Logger::put_p_message(picked);
  if (dropped.size() != 0) Logger::put_d_message(dropped);
  if (!vehicle_position.empty()) Logger::put_v_message(vehicle_position);
  if (arrived.size() != 0) Logger::put_a_message(arrived);

  return nrows;
}  // release dblock

void Cargo::record_customer_statuses() {
  while ((rc = sqlite3_step(sac_stmt)) == SQLITE_ROW) {
    std::lock_guard<std::mutex> dblock(dbmx);  // Lock acquired
    const CustId cust_id = sqlite3_column_int(sac_stmt, 0);
    const CustStatus cust_status =
        static_cast<CustStatus>(sqlite3_column_int(sac_stmt, 6));
    const VehlId cust_assignedTo = sqlite3_column_int(sac_stmt, 7);
    sol_statuses_[cust_id] = {cust_status, cust_assignedTo};
  }
  if (rc != SQLITE_DONE) {
    print_error << "Failure in select all customers. Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(sac_stmt);
}

/* Returns cost of all vehicle routes, plus the base cost for each
 * unassigned customer trip */
DistInt Cargo::total_route_cost() {
  DistInt cst = 0;

  while ((rc = sqlite3_step(sar_stmt)) == SQLITE_ROW) {
    const Wayp* rtebuf =
        static_cast<const Wayp*>(sqlite3_column_blob(sar_stmt, 1));
    const std::vector<Wayp> route(
        rtebuf, rtebuf + sqlite3_column_bytes(sar_stmt, 1) / sizeof(Wayp));
    cst += route.back().first;
  }
  if (rc != SQLITE_DONE) {
    print_error << "Failure in select all routes. Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  sqlite3_reset(sar_stmt);

  while ((rc = sqlite3_step(sac_stmt)) == SQLITE_ROW) {
    const CustId cust_id = sqlite3_column_int(sac_stmt, 0);
    const VehlId assigned_to = sqlite3_column_int(sac_stmt, 7);
    if (assigned_to == 0) cst += trip_costs_.at(cust_id);
  }
  return cst;
}

/* Start Cargo with the default (blank) RSAlgorithm */
void Cargo::start() {
  RSAlgorithm _("noalg");
  start(_);
}

void Cargo::start(RSAlgorithm& rsalg) {
  print_out << "Starting Cargo\n";
  print_out << "Starting algorithm " << rsalg.name() << "\n";

  /* Algorithm thread */
  std::thread thread_rsalg([&rsalg]() {
    while (!rsalg.done()) {
      rsalg.listen();
    }
  });

  // Logger thread
  Logger logger("a.log");
  std::thread logger_thread([&logger]() { logger.run(); });

  /* Cargo thread
   * (Don't call any rsalg methods here) */
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  int ndeact, nstepped, dur;
  while (active_vehicles_ > 0 || t_ <= tmin_) {
    t0 = std::chrono::high_resolution_clock::now();

    /* Timeout customers where t_ > early + matching_period_ */
    // Select timeouts and log them
    std::vector<CustId> timeout;
    sqlite3_bind_int(select_timeout_stmt, 1, t_);
    sqlite3_bind_int(select_timeout_stmt, 2, matching_period_);
    sqlite3_bind_int(select_timeout_stmt, 3, (int)CustStatus::Canceled);
    while ((rc = sqlite3_step(select_timeout_stmt)) == SQLITE_ROW) {
      timeout.push_back(sqlite3_column_int(select_timeout_stmt, 0));
    }
    sqlite3_clear_bindings(select_timeout_stmt);
    sqlite3_reset(select_timeout_stmt);
    if (timeout.size() != 0) Logger::put_t_message(timeout);

    // Update timeout customers
    sqlite3_bind_int(tim_stmt, 1, (int)CustStatus::Canceled);
    sqlite3_bind_int(tim_stmt, 2, t_);
    sqlite3_bind_int(tim_stmt, 3, matching_period_);
    if (sqlite3_step(tim_stmt) != SQLITE_DONE) {
      print_error << "Failed to timeout customers. Reason:\n";
      throw std::runtime_error(sqlite3_errmsg(db_));
    }
    DEBUG(1, {
      print_out << "(Timed out " << sqlite3_changes(db_) << " customers)\n";
    });
    sqlite3_clear_bindings(tim_stmt);
    sqlite3_reset(tim_stmt);

    /* Prepare this step's solution tables */
    sol_routes_.clear();
    sol_statuses_.clear();

    /* Step the vehicles */
    nstepped = step(ndeact);
    active_vehicles_ -= ndeact;
    print_out << "t=" << t_ << "; stepped " << nstepped
              << " vehicles; remaining=" << active_vehicles_ << ";"
              << std::endl;

    /* Log the customers */
    record_customer_statuses();

    // Write vehicle position update log
    // Logger::put_v_message(sol_routes_);

    /* Write the partial solution
     * (sol_routes_, sol_statuses_ are now filled) */
    f_sol_temp_ << t_;
    for (const auto& veh_id : sol_veh_cols_)
      f_sol_temp_ << '\t'  // line below is to put a 0 if vehicle is done
                  << (sol_routes_.find(veh_id) != sol_routes_.end()
                          ? sol_routes_.at(veh_id)
                          : -1);
    for (const auto& kv : sol_statuses_)
      f_sol_temp_ << '\t' << (int)kv.second.first << '\t' << kv.second.second;
    f_sol_temp_ << '\n';

    t1 = std::chrono::high_resolution_clock::now();

    dur =
        std::round(std::chrono::duration<double, std::milli>(t1 - t0).count());
    if (dur > sleep_interval_)
      print_warning << "step() (" << dur << " ms) exceeds interval ("
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
  print_out << "Finished algorithm " << rsalg.name() << "\n";

  f_sol_temp_.close();

  // Stop logger
  print_out << "Logger stop"
            << "\n";
  logger.stop();
  logger_thread.join();

  /* Print solution headers
   * (we do this last so we can get total_route_cost()) */
  std::ifstream ifs("sol.partial");
  std::ofstream f_sol_(solution_file_, std::ios::out);
  f_sol_ << name() << '\n'
         << road_network() << '\n'
         << "VEHICLES " << total_vehicles_ << '\n'
         << "CUSTOMERS " << total_customers_ << '\n'
         << "base cost " << base_cost_ << '\n'
         << "solution cost " << total_route_cost() << '\n'
         << '\n'
         << "T";
  for (const auto& veh_id : sol_veh_cols_)  // veh col headers
    f_sol_ << "\tVEH" << veh_id;
  for (const auto& kv : sol_statuses_)  // cust col headers
    f_sol_ << "\tC" << kv.first << "_ST\tC" << kv.first << "_AT";
  f_sol_ << '\n';
  f_sol_ << ifs.rdbuf();  // copy sol.partial
  ifs.close();
  f_sol_.close();

  print_out << "Finished Cargo" << std::endl;
}

void Cargo::initialize(const Options& opt) {
  total_customers_ = total_vehicles_ = base_cost_ = 0;

  print_out << "Starting initialization sequence\n";
  print_out << "Reading nodes...";
  const size_t nnodes = read_nodes(opt.path_to_roadnet, nodes_, bbox_);
  print_out << nnodes << "\n";
  print_out << "\tBounding box: (" << bbox().lower_left.lng << ","
            << bbox().lower_left.lat << "), (" << bbox().upper_right.lng << ","
            << bbox().upper_right.lat << ")\n";

  print_out << "Reading edges...";
  const size_t nedges = read_edges(opt.path_to_edges, edges_);
  print_out << nedges << "\n";

  print_out << "Reading gtree...";
  GTree::load(opt.path_to_gtree);
  gtree_ = GTree::get();
  print_out << "Done\n";

  print_out << "Reading problem...";
  const size_t ntrips = read_problem(opt.path_to_problem, probset_);
  print_out << ntrips << "\n";
  print_out << "\t" << name() << " on " << road_network() << "\n";

  tmin_ = tmax_ = 0;
  matching_period_ = opt.matching_period;
  sleep_interval_ = std::round((float)1000 / opt.time_multiplier);
  speed_ = opt.vehicle_speed;

  print_out << "Creating in-memory database...\n";
  if (sqlite3_open(":memory:", &db_) != SQLITE_OK) {
    print_error << "Failed (create db). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  /* Enable foreign keys */
  if (sqlite3_db_config(db_, SQLITE_DBCONFIG_ENABLE_FKEY, 1, NULL) !=
      SQLITE_OK) {
    print_error << "Failed (enable foreign keys). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  /* Performance enhancements */
  sqlite3_exec(db_, "PRAGMA synchronous = OFF", NULL, NULL, &err);
  sqlite3_exec(db_, "PRAGMA journal_mode = OFF", NULL, NULL, &err);
  sqlite3_exec(db_, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, &err);

  print_out << "\t Creating Cargo tables...";
  if (sqlite3_exec(db_, sql::create_cargo_tables, NULL, NULL, &err) !=
      SQLITE_OK) {
    print_error << "Failed (create cargo tables). Reason: " << err << "\n";
    print_out << sql::create_cargo_tables << "\n";
    throw std::runtime_error("create cargo tables failed.");
  }
  print_out << "Done\n";

  print_out << "\t Inserting nodes...";
  sqlite3_stmt* insert_node_stmt;
  if (sqlite3_prepare_v2(db_, "insert into nodes values(?, ?, ?);", -1,
                         &insert_node_stmt, NULL) != SQLITE_OK) {
    print_error << "Failed (create insert node stmt). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }
  for (const auto& kv : nodes_) {
    sqlite3_reset(insert_node_stmt);
    sqlite3_bind_int(insert_node_stmt, 1, kv.first);
    sqlite3_bind_double(insert_node_stmt, 2, kv.second.lng);
    sqlite3_bind_double(insert_node_stmt, 3, kv.second.lat);
    if (sqlite3_step(insert_node_stmt) != SQLITE_DONE) {
      print_error << "Failure at node " << kv.first << "\n";
      print_error << "Failed (insert nodes). Reason:\n";
      throw std::runtime_error(sqlite3_errmsg(db_));
    }
  }
  sqlite3_finalize(insert_node_stmt);
  print_out << "Done\n";

  print_out << "\t Inserting trips...";
  sqlite3_stmt* insert_vehicle_stmt;
  sqlite3_stmt* insert_customer_stmt;
  sqlite3_stmt* insert_stop_stmt;
  sqlite3_stmt* insert_schedule_stmt;
  sqlite3_stmt* insert_route_stmt;
  if (sqlite3_prepare_v2(db_,
                         "insert into vehicles values(?, ?, ?, ?, ?, ?, ?, ?);",
                         -1, &insert_vehicle_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(
          db_, "insert into customers values(?, ?, ?, ?, ?, ?, ?, ?);", -1,
          &insert_customer_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, "insert into stops values(?, ?, ?, ?, ?, ?);", -1,
                         &insert_stop_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, "insert into schedules values(?, ?);", -1,
                         &insert_schedule_stmt, NULL) != SQLITE_OK ||
      sqlite3_prepare_v2(db_, "insert into routes values(?, ?, ?, ?);", -1,
                         &insert_route_stmt, NULL) != SQLITE_OK) {
    print_error << "Failed (create insert trip stmts). Reason:\n";
    throw std::runtime_error(sqlite3_errmsg(db_));
  }

  for (const auto& kv : probset_.trips()) {
    for (const auto& trip : kv.second) {
      StopType stop_type = StopType::CustOrig;  // default
      if (trip.load() < 0) {                    // Insert vehicle
        total_vehicles_++;
        stop_type = StopType::VehlOrig;
        sqlite3_reset(insert_vehicle_stmt);
        sqlite3_bind_int(insert_vehicle_stmt, 1, trip.id());
        sqlite3_bind_int(insert_vehicle_stmt, 2, trip.orig());
        sqlite3_bind_int(insert_vehicle_stmt, 3, trip.dest());
        sqlite3_bind_int(insert_vehicle_stmt, 4, trip.early());
        sqlite3_bind_int(insert_vehicle_stmt, 5, trip.late());
        sqlite3_bind_int(insert_vehicle_stmt, 6, trip.load());
        sqlite3_bind_int(insert_vehicle_stmt, 7, 0);
        sqlite3_bind_int(insert_vehicle_stmt, 8, (int)VehlStatus::Enroute);
        if (sqlite3_step(insert_vehicle_stmt) != SQLITE_DONE) {
          print_error << "Failure at vehicle " << trip.id() << "\n";
          print_error << "Failed (insert vehicle). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sol_routes_[trip.id()] = trip.orig();

        /* Insert route */
        Stop a(trip.id(), trip.orig(), StopType::VehlOrig, trip.early(),
               trip.late(), trip.early());
        Stop b(trip.id(), trip.dest(), StopType::VehlDest, trip.early(),
               trip.late());
        std::vector<Wayp> route;
        Schedule s(trip.id(), {a, b});
        DistInt cost = route_through(s, route);
        base_cost_ += cost;
        trip_costs_[trip.id()] = cost;
        int nnd = route.at(1).first;
        sqlite3_reset(insert_route_stmt);
        sqlite3_bind_int(insert_route_stmt, 1, trip.id());
        sqlite3_bind_blob(insert_route_stmt, 2,
                          static_cast<void const*>(route.data()),
                          route.size() * sizeof(Wayp), SQLITE_TRANSIENT);
        sqlite3_bind_int(insert_route_stmt, 3, 0);
        sqlite3_bind_int(insert_route_stmt, 4, nnd);
        if (sqlite3_step(insert_route_stmt) != SQLITE_DONE) {
          print_error << "Failure at route " << trip.id() << "\n";
          print_error << "Failed (insert route). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        /* Insert schedule */
        Stop next_loc(trip.id(), route.at(1).second, StopType::VehlOrig,
                      trip.early(), trip.late());
        std::vector<Stop> sch{next_loc, b};
        sqlite3_reset(insert_schedule_stmt);
        sqlite3_bind_int(insert_schedule_stmt, 1, trip.id());
        sqlite3_bind_blob(insert_schedule_stmt, 2,
                          static_cast<void const*>(sch.data()),
                          sch.size() * sizeof(Stop), SQLITE_TRANSIENT);
        if (sqlite3_step(insert_schedule_stmt) != SQLITE_DONE) {
          print_error << "Failure at schedule " << trip.id() << "\n";
          print_error << "Failed (insert schedule). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
      } else if (trip.load() > 0) {  // Insert customer
        total_customers_++;
        Stop a(trip.id(), trip.orig(), StopType::CustOrig, trip.early(),
               trip.late(), trip.early());
        Stop b(trip.id(), trip.dest(), StopType::CustDest, trip.early(),
               trip.late());
        std::vector<Wayp> route;
        Schedule s(trip.id(), {a, b});
        DistInt cost = route_through(s, route);
        base_cost_ += cost;
        trip_costs_[trip.id()] = cost;
        stop_type = StopType::CustOrig;
        sqlite3_reset(insert_customer_stmt);
        sqlite3_bind_int(insert_customer_stmt, 1, trip.id());
        sqlite3_bind_int(insert_customer_stmt, 2, trip.orig());
        sqlite3_bind_int(insert_customer_stmt, 3, trip.dest());
        sqlite3_bind_int(insert_customer_stmt, 4, trip.early());
        sqlite3_bind_int(insert_customer_stmt, 5, trip.late());
        sqlite3_bind_int(insert_customer_stmt, 6, trip.load());
        sqlite3_bind_int(insert_customer_stmt, 7, (int)CustStatus::Waiting);
        sqlite3_bind_null(insert_customer_stmt, 8);
        if (sqlite3_step(insert_customer_stmt) != SQLITE_DONE) {
          print_error << "Failure at customer " << trip.id() << "\n";
          print_error << "Failed (insert customer). Reason:\n";
          throw std::runtime_error(sqlite3_errmsg(db_));
        }
        sol_statuses_[trip.id()] = {CustStatus::Waiting, 0};
      } else  // e.g. mail, packages
        print_warning << "Trip" << trip.id() << " load == 0\n";

      /* Insert origin */
      sqlite3_reset(insert_stop_stmt);
      sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
      sqlite3_bind_int(insert_stop_stmt, 2, trip.orig());
      sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type);
      sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
      sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
      sqlite3_bind_int(insert_stop_stmt, 6, trip.early());
      if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
        print_error << "Failure at stop " << trip.orig() << "\n";
        print_error << "Failed (insert stop). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      /* Insert destination */
      sqlite3_reset(insert_stop_stmt);
      sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
      sqlite3_bind_int(insert_stop_stmt, 2, trip.dest());
      sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type + 1);
      sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
      sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
      sqlite3_bind_null(insert_stop_stmt, 6);
      if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
        print_error << "Failure at stop " << trip.dest() << "\n";
        print_error << "Failed (insert stop). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
      }
      /* Get tmin_, tmax_ */
      tmin_ = std::max(trip.early(), tmin_);
      tmax_ = std::max(trip.late(), tmax_);
    }
  }

  active_vehicles_ = total_vehicles_;

  // Minimum sim time equals time of last trip appearing, plus matching pd.
  tmin_ += matching_period_;

  sqlite3_finalize(insert_vehicle_stmt);
  sqlite3_finalize(insert_customer_stmt);
  sqlite3_finalize(insert_stop_stmt);
  sqlite3_finalize(insert_schedule_stmt);
  sqlite3_finalize(insert_route_stmt);

  solution_file_ = opt.path_to_solution;

  /* Store vehicle column headers
   * (sol_veh_cols_ will be ordered) */
  for (const auto& kv : sol_routes_) sol_veh_cols_.push_back(kv.first);

  t_ = 0;  // Ready to begin!
  print_out << "Done\n";
  print_out << "Finished initialization sequence\n";
}

}  // namespace cargo
