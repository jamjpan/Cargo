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
#include "libcargo/cargo.h" /* static now(), gtree(), db() */
#include "libcargo/classes.h"
#include "libcargo/dbutils.h"
#include "libcargo/message.h"
#include "libcargo/types.h"

namespace cargo {

RSAlgorithm::RSAlgorithm(const std::string& name)
    : print_out(name), print_info(MessageType::Info, name),
      print_warning(MessageType::Warning, name),
      print_error(MessageType::Error, name),
      print_success(MessageType::Success, name)
{
    name_ = name;
    done_ = false;
    batch_time_ = 1;
    if (sqlite3_prepare_v2(Cargo::db(), sql::uro_stmt, -1, &uro_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::sch_stmt, -1, &sch_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::nnd_stmt, -1, &nnd_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::lvn_stmt, -1, &lvn_stmt, NULL) != SQLITE_OK
     || sqlite3_prepare_v2(Cargo::db(), sql::com_stmt, -1, &com_stmt, NULL) != SQLITE_OK) {
        print_error << "Failed (create rsalg stmts). Reason:\n";
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
}

RSAlgorithm::~RSAlgorithm()
{
    sqlite3_finalize(uro_stmt);
    sqlite3_finalize(sch_stmt);
    sqlite3_finalize(nnd_stmt);
    sqlite3_finalize(lvn_stmt);
    sqlite3_finalize(com_stmt);
}

const std::string& RSAlgorithm::name() const { return name_; }
bool RSAlgorithm::done() const { return done_; }
void RSAlgorithm::commit(const Customer& cust, const MutableVehicle& mveh)
{
    commit(cust, mveh, mveh.route().data(), mveh.schedule().data());
}
void RSAlgorithm::commit(const Customer& cust, const Vehicle& veh,
        const std::vector<Waypoint>& new_route,
        const std::vector<Stop>& new_schedule)
{
    std::lock_guard<std::mutex> dblock(Cargo::dbmx);

    // Commit the new route (current_node_idx, nnd are unchanged)
    std::string new_route_str = serialize_route(new_route);
    sqlite3_bind_text(uro_stmt, 1, new_route_str.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(uro_stmt, 2, veh.id());
    if ((rc = sqlite3_step(uro_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new route " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(uro_stmt);
    sqlite3_reset(uro_stmt);

    // ENHANCEMENT check if assignment is valid
    //   If the vehicle has moved a lot during RSAlgorithm::match(), then the
    //   lvn will be different in the db compared to what it is here (because
    //   here was selected before the vehicle moved). How to handle this case?
    //   Reject the assignment, or "roll back" the vehicle?
    //
    //   We roll back the vehicle for now, consider adding the check and rejceting
    //   the assignment in the future.

    // Re-commit the nnd
    sqlite3_bind_int(nnd_stmt, 1, veh.next_node_distance());
    sqlite3_bind_int(nnd_stmt, 2, veh.id());
    if ((rc = sqlite3_step(nnd_stmt)) != SQLITE_DONE) {
        std::cout << "Error in nnd " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(nnd_stmt);
    sqlite3_reset(nnd_stmt);

    // Reset the lvn
    sqlite3_bind_int(lvn_stmt, 1, 0);
    sqlite3_bind_int(lvn_stmt, 2, veh.id());
    if ((rc = sqlite3_step(lvn_stmt)) != SQLITE_DONE) {
        std::cout << "Error in lvn " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(lvn_stmt);
    sqlite3_reset(lvn_stmt);

    sqlite3_bind_text(sch_stmt, 1, serialize_schedule(new_schedule).c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(sch_stmt, 2, veh.id());
    if ((rc = sqlite3_step(sch_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit new schedule " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(sch_stmt);
    sqlite3_reset(sch_stmt);

    // Commit the assignment
    sqlite3_bind_int(com_stmt, 1, veh.id());
    sqlite3_bind_int(com_stmt, 2, cust.id());
    if ((rc = sqlite3_step(com_stmt)) != SQLITE_DONE) {
        std::cout << "Error in commit assignment " << rc << std::endl;
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    sqlite3_clear_bindings(com_stmt);
    sqlite3_reset(com_stmt);
}
void RSAlgorithm::kill() { done_ = true; }
int& RSAlgorithm::batch_time() { return batch_time_; }
std::vector<Customer>& RSAlgorithm::waiting_customers() { return waiting_customers_; }
std::vector<Vehicle>& RSAlgorithm::vehicles() { return vehicles_; }

void RSAlgorithm::handle_customer(const Customer &)
{
    /* Use handle_customer() for streaming-matching, or other necessary
     * customer processing. */
}

void RSAlgorithm::handle_vehicle(const Vehicle &)
{
    /* Use handle_vehicle() to add new vehicles to a spatial index
     * or other kind of vehicle processing. */
}

void RSAlgorithm::match()
{
    /* Use match() for bulk-matching. waiting_customers() and vehicles()
     * provide access to current customers and vehicles. */
}

void RSAlgorithm::end()
{
    /* Stuff here executes after the simulation finishes. */
}

void RSAlgorithm::listen()
{
    std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

    // Start timing -------------------------------
    t0 = std::chrono::high_resolution_clock::now();

    vehicles_.clear();
    sql::select_matchable_vehicles(vehicles_, Cargo::now());
    for (const auto& vehicle : vehicles_)
        handle_vehicle(vehicle);

    waiting_customers_.clear();
    sql::select_waiting_customers(waiting_customers_, Cargo::now());
    for (const auto& customer : waiting_customers_)
        handle_customer(customer);

    match();
    t1 = std::chrono::high_resolution_clock::now();
    // Stop timing --------------------------------

    // Don't sleep if time exceeds batch time
    int dur = std::round(std::chrono::duration<double, std::milli>(t1-t0).count());
    if (dur > batch_time_*1000)
        print_warning << "listen() ("<<dur<<" ms) exceeds batch time ("<<batch_time_*1000<<" ms)\n";
    else
        std::this_thread::sleep_for(std::chrono::milliseconds(batch_time_*1000-dur));
}

} // namespace cargo

