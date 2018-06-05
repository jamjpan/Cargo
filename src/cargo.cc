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
#include <iostream>
#include <string>
#include <thread>

#include "libcargo/cargo.h"
#include "libcargo/classes.h"
#include "libcargo/dbutils.h"
#include "libcargo/file.h"
#include "libcargo/functions.h"
#include "libcargo/message.h"
#include "libcargo/options.h"
#include "libcargo/rsalgorithm.h"
#include "libcargo/types.h"
#include "gtree/gtree.h"
#include "sqlite3/sqlite3.h"

namespace cargo {

// Initializes to an empty tree. The Cargo constructor will call
// Cargo::initialize(), which will load gtree based on the passed Options.
GTree::G_Tree Cargo::gtree_ = GTree::get();

// Initializes an empty sqlite3*.
sqlite3* Cargo::db_ = nullptr;

// Initializes vehicle speed.
Speed Cargo::speed_ = 0;

// Initializes start of the sim time.
SimTime Cargo::t_ = 0;

// Initialize the Messages mutex
std::mutex Message::mtx_;

Cargo::Cargo(const Options& opt)
    : print_out("cargo"),
      print_info(MessageType::Info, "cargo"),
      print_warning(MessageType::Warning, "cargo"),
      print_error(MessageType::Error, "cargo"),
      print_success(MessageType::Success, "cargo")
{
    print_out << "Initializing Cargo\n";
    initialize(opt);
    print_success << "Cargo initialized!" << std::endl;
}

Cargo::~Cargo()
{
    sqlite3_close(db_); // Calls std::terminate if fails
    print_out << "Database closed." << std::endl;
}

const BoundingBox& Cargo::bbox() const { return bbox_; }
const SimTime& Cargo::final_request_time() const { return tmin_; }
const SimTime& Cargo::final_arrival_time() const { return tmax_; }
size_t Cargo::active_vehicles() const { return active_vehicles_; }
const std::string& Cargo::name() { return probset_.name(); }
const std::string& Cargo::road_network()
{
    return probset_.road_network();
}

int Cargo::step(int& ndeact)
{
    ndeact = 0;
    int nrows = 0;

    sqlite3_stmt* ssv_stmt;
    if (sqlite3_prepare_v2(db_,
                sql::select_step_vehicles,
                -1, &ssv_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create select step vehicles stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }
    sqlite3_bind_int(ssv_stmt, 1, t_);
    sqlite3_bind_int(ssv_stmt, 2, (int)VehicleStatus::Arrived);

    SqliteReturnCode rc;
    // In the worst case, this loop runs |vehicles| times.
    while ((rc = sqlite3_step(ssv_stmt)) == SQLITE_ROW) {
        nrows++;
        // Print columns
        // for (int i = 0; i < sqlite3_column_count(ssv_stmt); ++i)
        //     print_info << "["<<i<<"] "<< sqlite3_column_name(ssv_stmt, i) << "\n";
        Route route(
                sqlite3_column_int(ssv_stmt, 0),
                deserialize_route(stringify(sqlite3_column_text(ssv_stmt, 8))));
        Schedule schedule(
                sqlite3_column_int(ssv_stmt, 0),
                deserialize_schedule(stringify(sqlite3_column_text(ssv_stmt, 12))));
        std::vector<Stop> new_schedule_data = schedule.data(); // <-- mutable
        Vehicle vehicle(
                sqlite3_column_int(ssv_stmt, 0),
                sqlite3_column_int(ssv_stmt, 1),
                sqlite3_column_int(ssv_stmt, 2),
                sqlite3_column_int(ssv_stmt, 3),
                sqlite3_column_int(ssv_stmt, 4),
                sqlite3_column_int(ssv_stmt, 5),
                sqlite3_column_int(ssv_stmt, 10),
                route,
                schedule,
                sqlite3_column_int(ssv_stmt, 9),
                static_cast<VehicleStatus>(sqlite3_column_int(ssv_stmt, 6)));
        DistanceInt nnd = vehicle.next_node_distance() - speed_; // step the veh
        RouteIndex lvn = vehicle.idx_last_visited_node();
        bool active = true; // all vehicles selected by ssv_stmt are active
        bool moved = (nnd <= 0) ? true : false;
        // Print vehicles
        // vehicle.print();
        // print_out << "new nnd: " << nnd << "\n";

        // In the worst case, this loop runs |schedule| times.
        // while (active && route.dist_at(lvn) - route.dist_at(lvn+1) >= nnd) {
        while (active && nnd <= 0) {
            lvn += 1;
            // The first stop in schedule is always the vehicle's current location.
            // The next stop is always in the 1 position.
            if (route.node_at(lvn) == schedule.at(1).location()) {
                const Stop& stop = schedule.at(1);
                // Vehicle is at own destination
                if (stop.type() == StopType::VehicleDest) {
                    if (sql::deactivate_vehicle(vehicle.id()) != SQLITE_OK) {
                        print_error << "Failed (deactivate vehicle" << vehicle.id() << "). Reason:\n";
                        throw std::runtime_error(sqlite3_errmsg(db_));
                    } else
                        print_info << "Vehicle " << vehicle.id() << " arrived." << std::endl;
                    active = false; // <-- stops the while loop
                    ndeact += 1;
                }
                // Vehicle is at customer pickup
                if (stop.type() == StopType::CustomerOrigin) {
                    if (sql::pickup_customer(vehicle.id(), stop.owner()) != SQLITE_OK) {
                        print_error << "Failed (veh" << vehicle.id() << " pickup " << stop.owner() << "). Reason:\n";
                        throw std::runtime_error(sqlite3_errmsg(db_));
                    } else
                        print_info << "Vehicle " << vehicle.id() << " picked up Customer "
                            << stop.owner() << "(" << stop.location() << ")" << std::endl;
                }
                // Vehicle is at customer dropoff
                if (stop.type() == StopType::CustomerDest) {
                    if (sql::dropoff_customer(vehicle.id(), stop.owner()) != SQLITE_OK) {
                        print_error << "Failed (veh" << vehicle.id() << " dropoff " << stop.owner() << "). Reason:\n";
                        throw std::runtime_error(sqlite3_errmsg(db_));
                    } else
                        print_info << "Vehicle " << vehicle.id() << " dropped off Customer "
                            << stop.owner() << "(" << stop.location() << ")" << std::endl;
                }
                // Update visitedAt
                if (sql::update_visitedAt(stop.owner(), stop.location(), t_) != SQLITE_OK) {
                    print_error << "Failed (update visitedAt for stop " << stop.owner() << " at " << stop.location() << "). Reason:\n";
                    throw std::runtime_error(sqlite3_errmsg(db_));
                }
                // Update schedule (remove the just-visited stop) O(|schedule|)
                new_schedule_data.erase(new_schedule_data.begin()+1);
            }
            if (active)
                nnd += (route.dist_at(lvn+1) - route.dist_at(lvn));
        }
        if (moved) {
            // Update schedule (update the vehicle's current location) O(|schedule|)
            Stop curr_loc(vehicle.id(), route.at(lvn).second, StopType::VehicleOrigin, vehicle.early(), vehicle.late(), t_);
            new_schedule_data.erase(new_schedule_data.begin());
            new_schedule_data.insert(new_schedule_data.begin(), curr_loc);
            Schedule new_schedule(vehicle.id(), new_schedule_data);
            // Commit the new schedule
            if (sql::update_schedule(vehicle.id(), new_schedule) != SQLITE_OK) {
                print_error << "Failed (update schedule for vehicle " << vehicle.id() << "). Reason:\n";
                throw std::runtime_error(sqlite3_errmsg(db_));
            }
            // Update idx_last_visited_node
            if (sql::update_idx_last_visited_node(vehicle.id(), lvn) != SQLITE_OK) {
                print_error << "Failed (update idx lvn for vehicle " << vehicle.id() << "). Reason:\n";
                throw std::runtime_error(sqlite3_errmsg(db_));
            }
        }
        // Update next_node_distance
        if (sql::update_next_node_distance(vehicle.id(), nnd) != SQLITE_OK) {
            print_error << "Failed (update nnd for vehicle " << vehicle.id() << "). Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
        }
    } // end SQLITE_ROW
    if (rc != SQLITE_DONE) {
        print_error << "Failure in select step vehicles. Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }
    sqlite3_finalize(ssv_stmt);
    return nrows;
}

void Cargo::start()
{
    RSAlgorithm no_algorithm;
    start(no_algorithm);
}

void Cargo::start(RSAlgorithm& rsalg)
{
    print_out << "Starting Cargo\n";
    print_out << "Starting algorithm " << rsalg.name() << "\n";

    // Ridesharing algorithm thread
    std::thread thread_rsalg([&rsalg]() { while (!rsalg.done()) {
        // The rsalg should poll for data in its listen() method. The
        // polling frequency is up to the algorithm.
        rsalg.listen();
    }});

    // Cargo thread
    // Don't call any rsalg methods here because then they would run in this
    // thread instead of their own thread.
    std::chrono::time_point<std::chrono::high_resolution_clock> t_start;
    std::chrono::time_point<std::chrono::high_resolution_clock> t_end;
    int ndeact, nstepped, elapsed;
    while (active_vehicles_ > 0 || t_ <= tmin_) {
        t_start = std::chrono::high_resolution_clock::now();

        // Timeout customers where t_ > early + matching_period_
        if (sql::timeout_customers(t_, matching_period_) != SQLITE_OK) {
            print_error << "Failed to timeout customers. Reason:\n";
            throw std::runtime_error(sqlite3_errmsg(db_));
        }
        print_out <<"("<< sqlite3_changes(db_) <<" customers have timed out)\n";

        // Function step() is in O(|vehicles|*|schedules|^2). The first term
        // dominates. On my machine, every 1,000 stepping vehicles takes about
        // 100 ms to loop through. The limit for "real-time" is then about
        // 10,000 stepping vehicles.
        nstepped = step(ndeact);
        active_vehicles_ -= ndeact;
        print_out << "t=" << t_ << "; stepped " << nstepped
                   << " vehicles; remaining=" << active_vehicles_ << ";"
                   << std::endl;

        t_end = std::chrono::high_resolution_clock::now();

        elapsed = std::round(
            std::chrono::duration<double, std::milli>(t_end - t_start).count());
        if (elapsed > sleep_interval_)
            print_warning << "Elapsed (" << elapsed << " ms) exceeds interval ("
                          << sleep_interval_ << " ms)\n";
        else
            std::this_thread::sleep_for(
                std::chrono::milliseconds(sleep_interval_ - elapsed));
        t_ += 1;
    }
    rsalg.kill();
    thread_rsalg.join();

    print_out << "Finished algorithm " << rsalg.name() << "\n";
    print_out << "Finished Cargo" << std::endl;
}

void Cargo::initialize(const Options& opt)
{
    print_out << "Starting initialization sequence\n";
    print_out << "Reading nodes...";
    size_t nnodes = read_nodes(opt.path_to_roadnet, nodes_, bbox_);
    print_out << nnodes << "\n";
    print_out << "\tBounding box: ("
        << bbox().lower_left.lng << "," << bbox().lower_left.lat << "), ("
        << bbox().upper_right.lng << "," << bbox().upper_right.lat << ")\n";

    print_out << "Reading edges...";
    size_t nedges = read_edges(opt.path_to_edges, edges_);
    print_out << nedges << "\n";

    print_out << "Reading gtree...";
    GTree::load(opt.path_to_gtree);
    gtree_ = GTree::get();
    print_out << "Done\n";

    print_out << "Reading problem...";
    size_t ntrips = read_problem(opt.path_to_problem, probset_);
    print_out << ntrips << "\n";
    print_out << "\t" << name() << " on " << road_network() << "\n";

    tmin_ = 0;
    tmax_ = 0;
    matching_period_ = opt.matching_period;
    sleep_interval_ = std::round((float)1000 / opt.time_multiplier);
    speed_ = opt.vehicle_speed;

    // Enhancement: add a --persist option to enable disk-based db
    //   Potential issue: inserting nodes takes a long time, ~16 min for CD1
    //   on my machine. Other performance issues will occur.
    print_out << "Creating in-memory database...\n";
    if (sqlite3_open(":memory:", &db_) != SQLITE_OK) {
        print_error << "Failed (create db). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    if (sqlite3_db_config(db_, SQLITE_DBCONFIG_ENABLE_FKEY, 1, NULL) != SQLITE_OK) {
        print_error << "Failed (enable foreign keys). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    print_out << "\t Creating Cargo tables...";
    SqliteErrorMessage err;
    if (sqlite3_exec(db_, sql::create_cargo_tables, NULL, NULL, &err) != SQLITE_OK) {
        print_error << "Failed (create cargo tables). Reason: " << err << "\n";
        // sqlite3_free(&err); -- throws invalid pointer?
        print_out << sql::create_cargo_tables << "\n";
        throw std::runtime_error("create cargo tables failed.");
    }
    print_out << "Done\n";

    print_out << "\t Inserting nodes...";
    sqlite3_stmt* insert_node_stmt;
    if (sqlite3_prepare_v2(db_, "insert into nodes values(?, ?, ?);",
            -1, &insert_node_stmt, NULL) != SQLITE_OK) {
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
    if (sqlite3_prepare_v2(
            db_, "insert into vehicles values(?, ?, ?, ?, ?, ?, ?);", -1,
            &insert_vehicle_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create insert_vehicle_stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    sqlite3_stmt* insert_customer_stmt;
    if (sqlite3_prepare_v2(
            db_, "insert into customers values(?, ?, ?, ?, ?, ?, ?, ?);", -1,
            &insert_customer_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create insert_customer_stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    sqlite3_stmt* insert_stop_stmt;
    if (sqlite3_prepare_v2(
            db_, "insert into stops values(?, ?, ?, ?, ?, ?);", -1,
            &insert_stop_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create insert_stop_stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    sqlite3_stmt* insert_schedule_stmt;
    if (sqlite3_prepare_v2(
            db_, "insert into schedules values(?, ?);", -1,
            &insert_schedule_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create insert_schedule_stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    sqlite3_stmt* insert_route_stmt;
    if (sqlite3_prepare_v2(
            db_, "insert into routes values(?, ?, ?, ?);", -1,
            &insert_route_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create insert_route_stmt). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(db_));
    }

    active_vehicles_ = 0;
    for (const auto& kv : probset_.trips()) {
        auto& this_trips = kv.second;
        for (const auto& trip : this_trips) {
            StopType stop_type;
            if (trip.load() < 0) {
                // Insert vehicle
                active_vehicles_++;
                stop_type = StopType::VehicleOrigin;
                sqlite3_reset(insert_vehicle_stmt);
                sqlite3_bind_int(insert_vehicle_stmt, 1, trip.id());
                sqlite3_bind_int(insert_vehicle_stmt, 2, trip.origin());
                sqlite3_bind_int(insert_vehicle_stmt, 3, trip.destination());
                sqlite3_bind_int(insert_vehicle_stmt, 4, trip.early());
                sqlite3_bind_int(insert_vehicle_stmt, 5, trip.late());
                sqlite3_bind_int(insert_vehicle_stmt, 6, trip.load());
                sqlite3_bind_int(insert_vehicle_stmt, 7, (int)VehicleStatus::Enroute);
                if (sqlite3_step(insert_vehicle_stmt) != SQLITE_DONE) {
                    print_error << "Failure at vehicle " << trip.id() << "\n";
                    print_error << "Failed (insert vehicle). Reason:\n";
                    throw std::runtime_error(sqlite3_errmsg(db_));
                }
                // Insert schedule
                Stop a(trip.id(), trip.origin(), StopType::VehicleOrigin,
                       trip.early(), trip.late(), trip.early());
                Stop b(trip.id(), trip.destination(),
                       StopType::VehicleDest, trip.early(), trip.late());
                std::string sch = serialize_schedule({a, b});
                sqlite3_reset(insert_schedule_stmt);
                sqlite3_bind_int(insert_schedule_stmt, 1, trip.id());
                sqlite3_bind_text(insert_schedule_stmt, 2, sch.c_str(),
                                  sch.length(), SQLITE_STATIC);
                if (sqlite3_step(insert_schedule_stmt) != SQLITE_DONE) {
                    print_error << "Failure at schedule " << trip.id() << "\n";
                    print_error << "Failed (insert schedule). Reason:\n";
                    throw std::runtime_error(sqlite3_errmsg(db_));
                }
                // Insert route
                std::vector<Waypoint> route;
                Schedule s(trip.id(), {a, b});
                route_through(s, route);
                std::string rtstr = serialize_route(route);
                int nnd = edges_.at(route.at(0).second).at(route.at(1).second);
                sqlite3_reset(insert_route_stmt);
                sqlite3_bind_int(insert_route_stmt, 1, trip.id());
                sqlite3_bind_text(insert_route_stmt, 2, rtstr.c_str(),
                        rtstr.length(), SQLITE_STATIC);
                sqlite3_bind_int(insert_route_stmt, 3, 0);
                sqlite3_bind_int(insert_route_stmt, 4, nnd);
                if (sqlite3_step(insert_route_stmt) != SQLITE_DONE) {
                    print_error << "Failure at route " << trip.id() << "\n";
                    print_error << "Failed (insert route). Reason:\n";
                    throw std::runtime_error(sqlite3_errmsg(db_));
                }
            }
            else if (trip.load() > 0) {
                // Insert customer
                stop_type = StopType::CustomerOrigin;
                sqlite3_reset(insert_customer_stmt);
                sqlite3_bind_int(insert_customer_stmt, 1, trip.id());
                sqlite3_bind_int(insert_customer_stmt, 2, trip.origin());
                sqlite3_bind_int(insert_customer_stmt, 3, trip.destination());
                sqlite3_bind_int(insert_customer_stmt, 4, trip.early());
                sqlite3_bind_int(insert_customer_stmt, 5, trip.late());
                sqlite3_bind_int(insert_customer_stmt, 6, trip.load());
                sqlite3_bind_int(insert_customer_stmt, 7, (int)CustomerStatus::Waiting);
                sqlite3_bind_null(insert_customer_stmt, 8);
                if (sqlite3_step(insert_customer_stmt) != SQLITE_DONE) {
                    print_error << "Failure at customer " << trip.id() << "\n";
                    print_error << "Failed (insert customer). Reason:\n";
                    throw std::runtime_error(sqlite3_errmsg(db_));
                }
            }
            else {
                print_error << "Failed (trip" << trip.id() << " load == 0).\n";
                throw std::runtime_error("trip load == 0");
            }
            // Insert origin
            sqlite3_reset(insert_stop_stmt);
            sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
            sqlite3_bind_int(insert_stop_stmt, 2, trip.origin());
            sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type);
            sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
            sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
            sqlite3_bind_int(insert_stop_stmt, 6, trip.early());
            if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
                print_error << "Failure at stop " << trip.origin() << "\n";
                print_error << "Failed (insert stop). Reason:\n";
                throw std::runtime_error(sqlite3_errmsg(db_));
            }
            // Insert destination
            sqlite3_reset(insert_stop_stmt);
            sqlite3_bind_int(insert_stop_stmt, 1, trip.id());
            sqlite3_bind_int(insert_stop_stmt, 2, trip.destination());
            sqlite3_bind_int(insert_stop_stmt, 3, (int)stop_type+1);
            sqlite3_bind_int(insert_stop_stmt, 4, trip.early());
            sqlite3_bind_int(insert_stop_stmt, 5, trip.late());
            sqlite3_bind_null(insert_stop_stmt, 6);
            if (sqlite3_step(insert_stop_stmt) != SQLITE_DONE) {
                print_error << "Failure at stop " << trip.destination() << "\n";
                print_error << "Failed (insert stop). Reason:\n";
                throw std::runtime_error(sqlite3_errmsg(db_));
            }
            // Get tmin_, tmax_
            tmin_ = std::max(trip.early(), tmin_);
            tmax_ = std::max(trip.late(), tmax_);
        }
    }
    sqlite3_finalize(insert_vehicle_stmt);
    sqlite3_finalize(insert_customer_stmt);
    sqlite3_finalize(insert_stop_stmt);
    sqlite3_finalize(insert_schedule_stmt);
    sqlite3_finalize(insert_route_stmt);
    print_out << "Done\n";

    t_ = 0; // Ready to begin
    print_out << "Finished initialization sequence\n";
}

// it's just like inform the vehicle who is the customer
//bool Cargo::RequestMatched(const Trip &customer, const VehicleId &vid,
//                               const Schedule &schedule, const Route &route) {
//    // TODO: validate the route
//    Route &old_route = vehicles_[vid].route;
//    int end = vehicles_[vid].lv_node;
//    // validity check !important
//    for (int i = 0; i <= end; ++i) {
//        // if the old route doesn't exist in the new route or the oid is in the
//        // old route
//        if (route[i] != old_route[i] || customer.oid == old_route[i]) {
//            ERROR << "req " << customer.id << " to veh " << vid << " REFUSED"
//                  << std::endl;
//            total_refuse_++;
//            return false;
//        }
//    }
//
//    // Get the runtime on the request
//    auto t_end = std::chrono::high_resolution_clock::now();
//    int elpased = std::round(std::chrono::duration<double, std::milli>(
//                                 t_end - broadcast_time_[customer.id])
//                                 .count());
//#if SPBUG
//    if (vid == SP) {
//        ERROR << "request " << customer.id << " matched to veh " << vid
//              << " in " << elpased << " ms" << std::endl;
//        for (auto &item : route)
//            INFO << item << " ";
//        INFO << std::endl;
//        for (auto &item : schedule)
//            INFO << item.node_id << " ";
//        INFO << std::endl;
//    }
//#endif
//#if !SPBUG
//    ERROR << "request " << customer.id << " matched to veh " << vid << " in "
//          << elpased << " ms" << std::endl;
//#endif
//    total_time_ += elpased;
//    total_match_++;
//
//    try {
//        vehicles_[vid].sched.clear();
//        vehicles_[vid].route.clear();
//        std::copy(schedule.begin(), schedule.end(),
//                  std::back_inserter(vehicles_[vid].sched));
//        std::copy(route.begin(), route.end(),
//                  std::back_inserter(vehicles_[vid].route));
//        vehicles_[vid].load++;
//        return true;
//    } catch (std::exception &e) {
//        ERROR << "IIIIIIIIIIIIII'm here" << std::endl;
//        return true;
//    }
//}

} // namespace cargo

