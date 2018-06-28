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

#include "libcargo/rsalgorithm.h"
#include "libcargo/cargo.h" /* now(), gtree(), db() */
#include "libcargo/classes.h"
#include "libcargo/dbsql.h"
#include "libcargo/message.h"
#include "libcargo/types.h"

namespace cargo {

RSAlgorithm::RSAlgorithm(const std::string& name)
    : print_out(name), print_info(MessageType::Info, name),
      print_warning(MessageType::Warning, name),
      print_error(MessageType::Error, name),
      print_success(MessageType::Success, name) {
    name_ = name;
    done_ = false;
    batch_time_ = 1;
    if (sqlite3_prepare_v2(Cargo::db(), sql::ssr_stmt, -1, &ssr_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::uro_stmt, -1, &uro_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::sch_stmt, -1, &sch_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::qud_stmt, -1, &qud_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::com_stmt, -1, &com_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::smv_stmt, -1, &smv_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::swc_stmt, -1, &swc_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create rsalg stmts). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
}

RSAlgorithm::~RSAlgorithm() {
    sqlite3_finalize(ssr_stmt);
    sqlite3_finalize(uro_stmt);
    sqlite3_finalize(sch_stmt);
    sqlite3_finalize(qud_stmt);
    sqlite3_finalize(com_stmt);
    sqlite3_finalize(smv_stmt);
    sqlite3_finalize(swc_stmt);
}

const std::string& RSAlgorithm::name() const { return name_; }
bool RSAlgorithm::done() const { return done_; }
bool RSAlgorithm::commit(const std::vector<Customer>           & custs,
                         const std::shared_ptr<MutableVehicle> & mvehptr,
                         const std::vector<Waypoint>           & new_route,
                         const std::vector<Stop>               & new_schedule,
                         std::vector<Waypoint>                 & sync_rte,
                         DistanceInt                           & sync_nnd) {
    return commit(custs, *mvehptr, new_route, new_schedule, sync_rte, sync_nnd);
}
bool RSAlgorithm::commit(const std::vector<Customer>           & custs,
                         const Vehicle                         & veh,
                         const std::vector<Waypoint>           & new_route,
                         const std::vector<Stop>               & new_schedule,
                         std::vector<Waypoint>                 & sync_rte,
                         DistanceInt                           & sync_nnd) {
    std::lock_guard<std::mutex>
        dblock(Cargo::dbmx); // Lock acquired

    /* Synchronize
     * The vehicle may have moved since new_route/new_schedule were computed.
     * The new_route should be synchronized with the current route. In other
     * words, new_route may not be possible anymore due to the vehicle
     * movement. For example,
     *     new_route may start with {a, x, y, d, e},
     *     the old route may be     {a, b, c, d, e},
     * the vehicle may already be at d, hence moving to x, y is no longer
     * possible. The commit should be rejected in this case. */
    std::vector<Wayp>   cur_route;
    RouteIndex          cur_lvn = 0;
    DistanceInt         cur_nnd = 0;
    sqlite3_bind_int(ssr_stmt, 1, veh.id());
    while ((rc = sqlite3_step(ssr_stmt)) == SQLITE_ROW) {
        const Wayp* buf = static_cast<const Wayp*>(sqlite3_column_blob(ssr_stmt, 1));
        std::vector<Wayp> raw_route(buf, buf + sqlite3_column_bytes(ssr_stmt, 1)/sizeof(Wayp));
        cur_route = raw_route;
        cur_lvn = sqlite3_column_int(ssr_stmt, 2);
        cur_nnd = sqlite3_column_int(ssr_stmt, 3);
    }
    if (rc != SQLITE_DONE)
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    sqlite3_clear_bindings(ssr_stmt);
    sqlite3_reset(ssr_stmt);
     /* Strategy
     * Get the current route/lvn/nnd. Find the current waypoint in the
     * new route. If not found, the new route is invalid, reject the commit.
     * If found, move back one in the current route, move back one in the
     * new route, and check if equal. If not equal, reject the commit.
     * Continue moving back one waypoint, checking if equal, until no more
     * waypoints in the new route. */
    sync_rte = new_route;
    auto x = std::find_if(sync_rte.begin(), sync_rte.end(), [&](const Wayp& a) {
            return a.second == cur_route.at(cur_lvn).second; });

    std::cout << "cur_route.at(cur_lvn): " << cur_route.at(cur_lvn).second << std::endl;
    std::cout << "x: " << x->second << std::endl;

    bool ok = true;
    if (x == sync_rte.end())    ok = false;
    else if (cur_lvn == 0)      ok = true;
    else {
        int itr = cur_lvn;
        auto i = x;
        while (itr >= 0 && i >= sync_rte.begin()) {
            std::cout << "itr(" << cur_route.at(itr).second << ") ";
            std::cout << "  i(" << i->second << ")" << std::endl;
            if (i->second != cur_route.at(itr).second) {
                ok = false;
                break;
            }
            itr--;
            i--;
        }
    }

    if (!ok) {
        std::cout << "cur_route: ";
        for (const auto& wp : cur_route)
            std::cout << " (" << wp.first << "|" << wp.second << ")";
        std::cout << std::endl;
        std::cout << "new_route: ";
        for (const auto& wp : new_route)
            std::cout << " (" << wp.first << "|" << wp.second << ")";
        std::cout << std::endl;
    }

    if (ok) {
    // Commit the new route (synchronized for vehicle movement)
    //std::cout << "new_route before synch: " << std::endl;
    //for (const auto& wp : sync_rte)
    //    std::cout << " (" << wp.first << "|" << wp.second << ")";
    //std::cout << std::endl;

    sync_rte.erase(sync_rte.begin(), x);
    sync_nnd = cur_nnd;

    //std::cout << "new_route after synch: " << std::endl;
    //for (const auto& wp : sync_rte)
    //    std::cout << " (" << wp.first << "|" << wp.second << ")";
    //std::cout << std::endl;

    sqlite3_bind_blob(uro_stmt, 1, static_cast<void const *>(sync_rte.data()),
            sync_rte.size()*sizeof(Wayp), SQLITE_TRANSIENT);
    sqlite3_bind_int(uro_stmt, 2, 0);
    sqlite3_bind_int(uro_stmt, 3, cur_nnd);
    sqlite3_bind_int(uro_stmt, 4, veh.id());
    if ((rc = sqlite3_step(uro_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new route " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(uro_stmt);
    sqlite3_reset(uro_stmt);

    // Commit the new schedule
    sqlite3_bind_blob(sch_stmt, 1, static_cast<void const*>(new_schedule.data()),
            new_schedule.size()*sizeof(Stop), SQLITE_TRANSIENT);
    sqlite3_bind_int(sch_stmt, 2, veh.id());
    if ((rc = sqlite3_step(sch_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new schedule " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(sch_stmt);
    sqlite3_reset(sch_stmt);

    // Increase queued
    sqlite3_bind_int(qud_stmt, 1, veh.id());
    if ((rc = sqlite3_step(qud_stmt)) != SQLITE_DONE) {
        std::cout << "Error in qud " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(qud_stmt);
    sqlite3_reset(qud_stmt);

    // Commit the assignment
    for (const auto& cust : custs) {
    sqlite3_bind_int(com_stmt, 1, veh.id());
    sqlite3_bind_int(com_stmt, 2, cust.id());
    if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(com_stmt);
    sqlite3_reset(com_stmt);
    } // end for

    } // end if ok

    return ok;
}
void RSAlgorithm::kill() { done_ = true; }
int& RSAlgorithm::batch_time() { return batch_time_; }
std::vector<Customer>& RSAlgorithm::waiting_customers() { return waiting_customers_; }
std::vector<Vehicle>& RSAlgorithm::vehicles() { return vehicles_; }

void RSAlgorithm::select_matchable_vehicles()
{
    vehicles_.clear();
    sqlite3_bind_int(smv_stmt, 1, Cargo::now());
    sqlite3_bind_int(smv_stmt, 2, (int)VehicleStatus::Arrived);
    while ((rc = sqlite3_step(smv_stmt)) == SQLITE_ROW) {
        const Waypoint* buf = static_cast<const Waypoint*>(sqlite3_column_blob(smv_stmt, 9));
        std::vector<Waypoint> raw_route(buf, buf + sqlite3_column_bytes(smv_stmt, 9)/sizeof(Waypoint));
        Route route(sqlite3_column_int(smv_stmt, 0), raw_route);

        const Stop* schbuf = static_cast<const Stop*>(sqlite3_column_blob(smv_stmt, 13));
        std::vector<Stop> raw_sch(schbuf, schbuf + sqlite3_column_bytes(smv_stmt, 13)/sizeof(Stop));
        Schedule schedule(sqlite3_column_int(smv_stmt, 0), raw_sch);

        /* Construct vehicle object */
        Vehicle vehicle(
                sqlite3_column_int(smv_stmt, 0),
                sqlite3_column_int(smv_stmt, 1),
                sqlite3_column_int(smv_stmt, 2),
                sqlite3_column_int(smv_stmt, 3),
                sqlite3_column_int(smv_stmt, 4),
                sqlite3_column_int(smv_stmt, 5),
                sqlite3_column_int(smv_stmt, 6),
                sqlite3_column_int(smv_stmt, 11),
                route,
                schedule,
                sqlite3_column_int(smv_stmt, 10),
                static_cast<VehicleStatus>(sqlite3_column_int(smv_stmt, 7)));
        vehicles_.push_back(vehicle);
    }
    if (rc != SQLITE_DONE)
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    sqlite3_clear_bindings(smv_stmt);
    sqlite3_reset(smv_stmt);
}

void RSAlgorithm::select_waiting_customers()
{
    waiting_customers_.clear();
    sqlite3_bind_int(swc_stmt, 1, (int)CustomerStatus::Waiting);
    sqlite3_bind_int(swc_stmt, 2, Cargo::now());
    while ((rc = sqlite3_step(swc_stmt)) == SQLITE_ROW) {
        // Print columns
        // for (int i = 0; i < sqlite3_column_count(stmt); ++i)
        //    std::cout << "["<<i<<"] "<< sqlite3_column_name(stmt, i) << "\n";
        Customer customer(
            sqlite3_column_int(swc_stmt, 0),
            sqlite3_column_int(swc_stmt, 1),
            sqlite3_column_int(swc_stmt, 2),
            sqlite3_column_int(swc_stmt, 3),
            sqlite3_column_int(swc_stmt, 4),
            sqlite3_column_int(swc_stmt, 5),
            static_cast<CustomerStatus>(sqlite3_column_int(swc_stmt, 6)),
            sqlite3_column_int(swc_stmt, 7));
        waiting_customers_.push_back(customer);
    }
    if (rc != SQLITE_DONE)
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    sqlite3_clear_bindings(swc_stmt);
    sqlite3_reset(swc_stmt);

}

/* Overrideables */
void RSAlgorithm::handle_customer(const Customer &)
{
    /* For streaming-matching or other customer processing. */
}

void RSAlgorithm::handle_vehicle(const Vehicle &)
{
    /* For vehicle processing (e.g. add to a spatial index) */
}

void RSAlgorithm::match()
{
    /* For bulk-matching. Access current customers and vehicles using
     * waiting_customers() and vehicles() */
}

void RSAlgorithm::end()
{
    /* Executes after the simulation finishes. */
}

void RSAlgorithm::listen()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

    // Start timing -------------------------------
    t0 = std::chrono::high_resolution_clock::now();

    select_matchable_vehicles();
    for (const auto& vehicle : vehicles_)
        handle_vehicle(vehicle);

    select_waiting_customers();
    for (const auto& customer : waiting_customers_)
        handle_customer(customer);

    match();
    t1 = std::chrono::high_resolution_clock::now();
    // Stop timing --------------------------------

    // Don't sleep if time exceeds batch time
    int dur = std::round(std::chrono::duration<double, std::milli>(t1-t0).count());
    if (dur > batch_time_*1000)
        print_warning << "listen() ("<<dur<<" ms) exceeds batch time ("<<batch_time_*1000<<" ms) for "
            << vehicles_.size() << " vehs and " << waiting_customers_.size() << " custs" << std::endl;
    else {
        print_info << "listen() handled " << vehicles_.size() << " vehs and " << waiting_customers_.size()
            << " custs in " << dur << " ms" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(batch_time_*1000-dur));
    }
}

} // namespace cargo

