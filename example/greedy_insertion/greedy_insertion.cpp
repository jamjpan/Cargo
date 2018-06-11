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
    batch_time() = 1; // Setting batch to 1 second
    nmatches = 0;
}

void GreedyInsertion::handle_customer(const cargo::Customer cust)
{
    print_warning << "GreedyInsertion handling customer " << cust.id() << "\n";
    if (cust.assigned())
        return; // <-- skip assigned (but not yet picked up)

    cargo::DistanceInt cost;
    cargo::DistanceInt best_cost = cargo::InfinityInt;
    std::vector<cargo::Stop> schedule;
    std::vector<cargo::Stop> best_schedule;
    std::vector<cargo::Waypoint> route;
    std::vector<cargo::Waypoint> best_route;
    cargo::Vehicle best_vehicle;
    bool matched = false;

    // TODO: use index to narrow the candidates
    for (const auto& veh : vehicles()) {
        print_warning << "\tevaluating vehicle " << veh.id() << "\n";
        cost = cargo::sop_insert(veh, cust, schedule, route);
        for (const auto& wp : route)
            std::cout << "(" << wp.first << "|" << wp.second << ")";
        std::cout << std::endl;
        bool within_time = cargo::check_timewindow_constr(schedule, route);
        print_warning << "\t\tcost: " << cost << " in-time: " << within_time << "\n";
        if ((cost < best_cost) && within_time) {
            best_schedule = schedule;
            best_route = route;
            best_vehicle = veh;
            best_cost = cost;
            matched = true;
        }
    }
    if (matched) {
        commit(cust, best_vehicle, best_route, best_schedule);
        print_success << "Match (cust" << cust.id() << ", veh" << best_vehicle.id() << ")\n";
        nmatches++;
    }
}

void GreedyInsertion::end()
{
    print_success << "Matches: "<<nmatches<<std::endl;
}

void GreedyInsertion::listen()
{
    RSAlgorithm::listen(); // call base listen()
    print_info << "Idled "<< batch_time() <<" seconds." << std::endl;
}

int main()
{
    cargo::Options op;
    op.path_to_roadnet = "../../data/roadnetwork/tiny.rnet";
    op.path_to_gtree   = "../../data/roadnetwork/tiny.gtree";
    op.path_to_edges   = "../../data/roadnetwork/tiny.edges";
    op.path_to_problem = "../../data/benchmark/tiny/tiny-n1m2.instance";
    op.time_multiplier = 2;
    op.vehicle_speed   = 1;
    op.matching_period = 10;

    cargo::Cargo cargo(op);

    GreedyInsertion gr;
    cargo.start(gr);
}

