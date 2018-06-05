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
#include <exception>
#include <iostream> /* std::endl */
#include <vector>

#include "greedy_insertion.h"
#include "libcargo.h"

GreedyInsertion::GreedyInsertion() : RSAlgorithm("greedy_insertion")
{
    // Initialize stuff here
    batch_time() = 5;
    nmatches = 0;
}

void GreedyInsertion::match() {
    for (const auto& customer : waiting_customers()) {

        // NULL converts to 0 when retrieved from the database. In the
        // benchmarks, there is no vehicle ID = 0 for this reason.
        // If assignedTo() == 0, then the customer is not yet assigned.
        if (customer.assignedTo() > 0)
            continue;

        cargo::DistanceInt cost;
        cargo::DistanceInt best_cost = cargo::InfinityInt;
        std::vector<cargo::Stop> schedule, best_schedule;
        std::vector<cargo::Waypoint> route, best_route;
        cargo::VehicleId best_vehicle = 0;
        // TODO: use index to narrow the candidates
        for (const auto& vehicle : vehicles()) {
            cost = cargo::sop_insert(
                    vehicle.schedule(), customer, true, true, schedule, route);
            if (cost < best_cost
                    && cargo::check_timewindow_constr(schedule, route)) {
                best_schedule = schedule;
                best_route = route;
                best_vehicle = vehicle.id();
            }
        }
        if (best_vehicle > 0) {
            // TODO: simplify into one command
            if (cargo::sql::assign_customer_to(customer.id(), best_vehicle) != SQLITE_OK) {
                print_error << "Failed assign " << customer.id() << "to " << best_vehicle << "\n";
                throw std::runtime_error(sqlite3_errmsg(cargo::Cargo::db()));
            }
            if (cargo::sql::replace_route(best_vehicle, best_route) != SQLITE_OK) {
                print_error << "Failed update route for vehicle " << best_vehicle << "\n";
                throw std::runtime_error(sqlite3_errmsg(cargo::Cargo::db()));
            }
            if (cargo::sql::update_schedule(best_vehicle, best_schedule) != SQLITE_OK) {
                print_error << "Failed update schedule for vehicle " << best_vehicle << "\n";
                throw std::runtime_error(sqlite3_errmsg(cargo::Cargo::db()));
            }
            print_success << "Match (Customer " << customer.id() << ", Vehicle "
                << best_vehicle << ")" << std::endl;
            nmatches++;
        }
    }
    print_out << "Matches: " << nmatches << std::endl;
}

int main() {
    cargo::Options opt;
    opt.path_to_roadnet = "../../data/roadnetwork/tiny.rnet";
    opt.path_to_gtree = "../../data/roadnetwork/tiny.gtree";
    opt.path_to_edges = "../../data/roadnetwork/tiny.edges";
    opt.path_to_problem = "../../data/benchmark/tiny/tiny-n1m2.instance";
    opt.time_multiplier = 1;
    opt.vehicle_speed = 1;
    opt.matching_period = 60;

    GreedyInsertion gr;
    cargo::Cargo cargo(opt);
    cargo.start(gr);
}

