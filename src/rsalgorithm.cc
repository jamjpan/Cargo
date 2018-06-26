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
    if (sqlite3_prepare_v2(Cargo::db(), sql::uro_stmt, -1, &uro_stmt, NULL) != SQLITE_OK
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
    sqlite3_finalize(uro_stmt);
    sqlite3_finalize(sch_stmt);
    sqlite3_finalize(qud_stmt);
    sqlite3_finalize(com_stmt);
    sqlite3_finalize(smv_stmt);
    sqlite3_finalize(swc_stmt);
}

const std::string& RSAlgorithm::name() const { return name_; }
bool RSAlgorithm::done() const { return done_; }
void RSAlgorithm::commit(const std::vector<Customer>    & custs,
                         const MutableVehicle           & mveh) {
    commit(custs, mveh, mveh.route().data(), mveh.schedule().data());
}
void RSAlgorithm::commit(const std::vector<Customer>           & custs,
                         const std::shared_ptr<MutableVehicle> & mvehptr,
                         const std::vector<Waypoint>           & new_route,
                         const std::vector<Stop>               & new_schedule) {
    commit(custs, *mvehptr, new_route, new_schedule);
}
void RSAlgorithm::commit(const std::vector<Customer>    & custs,
                         const Vehicle                  & veh,
                         const std::vector<Waypoint>    & new_route,
                         const std::vector<Stop>        & new_schedule) {
    std::lock_guard<std::mutex>
        dblock(Cargo::dbmx); // Lock acquired

    // ENHANCEMENT check if assignment is valid
    //   If the vehicle has moved a lot during RSAlgorithm::match(), then the
    //   lvn will be different in the db compared to what it is here (because
    //   here was selected before the vehicle moved). How to handle this case?
    //   Reject the assignment, or "roll back" the vehicle?
    //
    //   We roll back the vehicle for now, consider adding the check and rejceting
    //   the assignment in the future.
    //
    // Commit the new route (current_node_idx, nnd are unchanged)
    sqlite3_bind_blob(uro_stmt, 1, static_cast<void const *>(new_route.data()),
            new_route.size()*sizeof(Waypoint), SQLITE_TRANSIENT);
    sqlite3_bind_int(uro_stmt, 2, 0);
    sqlite3_bind_int(uro_stmt, 3, veh.next_node_distance());
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

