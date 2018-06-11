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
    this->batch_time() = 0; // Inherited from RSAlgorithm
    this->nmatches = 0;     // Example of a custom variable
}

void GreedyInsertion::match()
{
    for (const auto& cust : waiting_customers()) {
        if (cust.assigned())
            continue; // <-- skip assigned (but not yet picked up)

        cargo::DistanceInt cost;
        cargo::DistanceInt best_cost = cargo::InfinityInt;
        std::vector<cargo::Stop> schedule;
        std::vector<cargo::Stop> best_schedule;
        std::vector<cargo::Waypoint> route;
        std::vector<cargo::Waypoint> best_route;
        cargo::VehicleId best_vehicle = 0;
        bool matched = false;

        // TODO: use index to narrow the candidates
        // BUG wrong times in the route?
        //   The schedule returned by veh.schedule() only includes future
        //   stops. The route returned by sop_insert sets the time at the
        //   first stop in this schedule to be 0, and all subsequent stop
        //   distances are relative to this first stop. But in reality, the
        //   vehicle has already spent some time to get to this first stop.
        //   This time should be added to all the waypoints in the route.
        for (const auto& veh : vehicles()) {
            cost = cargo::sop_insert(veh, cust, schedule, route);
            if (cost < best_cost
                    && cargo::check_timewindow_constr(schedule, route)) {
                best_schedule = schedule;
                best_route = route;
                best_vehicle = veh.id();
                best_cost = cost;
                matched = true;
            }
        }
        if (matched) {
            std::cerr << "GreedyInsertion found a match, calling commit" << std::endl;
            commit(cust.id(), best_vehicle, best_route, best_schedule);
            nmatches++;
            print_success << "Match (Customer " << cust.id() << ", Vehicle "
                          << best_vehicle << ")" << std::endl;
            print_warning << "best cost: " << best_cost << "\n";
            print_warning << "best route: ";
            for (const auto& wp : best_route)
                std::cout << "(" << wp.first << "|" << wp.second << ") ";
            std::cout << std::endl;
            print_warning << "best schedule: ";
            for (const auto& s : best_schedule)
                std::cout << s.location() << " ";
            std::cout << std::endl;
        }
    }
}

void GreedyInsertion::listen()
{
    RSAlgorithm::listen(); // call base listen()
    /* Custom stuff here */
}

int main()
{
    cargo::Options opt;
    opt.path_to_roadnet = "../../data/roadnetwork/tiny.rnet";
    opt.path_to_gtree   = "../../data/roadnetwork/tiny.gtree";
    opt.path_to_edges   = "../../data/roadnetwork/tiny.edges";
    opt.path_to_problem = "../../data/benchmark/tiny/tiny-n1m2.instance";
    opt.time_multiplier = 2;
    opt.vehicle_speed   = 1;
    opt.matching_period = 10;

    GreedyInsertion gr;
    cargo::Cargo cargo(opt);
    cargo.start(gr);
}

