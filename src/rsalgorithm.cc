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

RSAlgorithm::RSAlgorithm()
    : print_out("rs"), print_info(MessageType::Info, "rs"),
      print_warning(MessageType::Warning, "rs"),
      print_error(MessageType::Error, "rs"),
      print_success(MessageType::Success, "rs")
{
    name_ = "noname";
    done_ = false;
    batch_time_ = 1;
}
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
void RSAlgorithm::kill() { done_ = true; }
int& RSAlgorithm::batch_time() { return batch_time_; }
std::vector<Customer>& RSAlgorithm::waiting_customers() { return waiting_customers_; }
std::vector<Vehicle>& RSAlgorithm::vehicles() { return vehicles_; }

void RSAlgorithm::handle_customer(const Customer& customer) {
    // print_info << "Customer " << customer.id() << " request received.\n";
    match();
}

void RSAlgorithm::handle_vehicle(const Vehicle& vehicle) {
    // print_info << "Vehicle " << vehicle.id() << " appeared.\n";
    // e.g. add vehicle to a local index
}

void RSAlgorithm::match() {
    // print_info << "Match called, but I don't know how to match.\n";
}

void RSAlgorithm::listen() {
    // O(n); we have to update all the local vehicles
    vehicles_.clear();
    sql::select_matchable_vehicles(vehicles_, Cargo::now());
    for (const auto& vehicle : vehicles_)
        if (appeared_vehicles_.insert(vehicle.id()).second == true)
            handle_vehicle(vehicle);

    waiting_customers_.clear();
    sql::selectall_waiting_customers(waiting_customers_, Cargo::now());
    for (const auto& customer : waiting_customers_)
        handle_customer(customer);

    // Adjust the batch time here. Set to 0 if you want streaming.
    // But beware, your CPU fans will spin.
    print_info << "Going to sleep now for " << batch_time_ << " seconds.\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(batch_time_*1000));
}

} // namespace cargo

