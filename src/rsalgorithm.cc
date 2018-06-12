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
#include <thread>

#include "libcargo/rsalgorithm.h"
#include "libcargo/cargo.h" /* static now(), gtree(), db() */
#include "libcargo/classes.h"
#include "libcargo/dbutils.h"
#include "libcargo/message.h"
#include "libcargo/types.h"

namespace cargo {

bool RSAlgorithm::committing_ = false;

RSAlgorithm::RSAlgorithm(const std::string& name)
    : print_out(name), print_info(MessageType::Info, name),
      print_warning(MessageType::Warning, name),
      print_error(MessageType::Error, name),
      print_success(MessageType::Success, name)
{
    name_ = name;
    done_ = false;
    batch_time_ = 1;
}

const std::string& RSAlgorithm::name() const { return name_; }
bool RSAlgorithm::done() const { return done_; }
void RSAlgorithm::commit(const Customer cust, const Vehicle veh,
        std::vector<cargo::Waypoint> new_route,
        const std::vector<cargo::Stop> new_schedule) const
{
    committing_ = true;         // lock
    while (Cargo::stepping())   // wait for lock
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (cargo::sql::commit_assignment(cust, veh, new_route, new_schedule) != SQLITE_OK) {
        std::cerr << "Failed commit " << cust.id() << "to " << veh.id() << "\n";
        throw std::runtime_error(sqlite3_errmsg(Cargo::db()));
    }
    committing_ = false;        // unlock
}
void RSAlgorithm::kill() { done_ = true; }
int& RSAlgorithm::batch_time() { return batch_time_; }
std::vector<Customer>& RSAlgorithm::waiting_customers() { return waiting_customers_; }
std::vector<Vehicle>& RSAlgorithm::vehicles() { return vehicles_; }

void RSAlgorithm::handle_customer(const Customer customer)
{
    /* Use handle_customer() for streaming-matching, or other necessary
     * customer processing. */
}

void RSAlgorithm::handle_vehicle(const Vehicle vehicle)
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
    int dur = std::round(std::chrono::duration<double, std::milli>(t1-t0).count());
    if (dur > batch_time_*1000)
        print_warning << "listen() ("<<dur<<" ms) exceeds batch time ("<<batch_time_*1000<<" ms)\n";
    else
        std::this_thread::sleep_for(std::chrono::milliseconds(batch_time_*1000-dur));
}

} // namespace cargo

