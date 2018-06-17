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

#include "libcargo.h"
#include "greedy_insertion.h"

GreedyInsertion::GreedyInsertion() : RSAlgorithm("greedy_insertion"),
    grid_(100) // <-- initialize my 100x100 grid
{
    batch_time() = 1; // Set batch to 1 second
    nmatches = 0; // Initialize my private counter
}

void GreedyInsertion::handle_customer(const cargo::Customer& cust)
{
    /* Some customers are assigned, but not yet picked up. We will decide
     * to skip these when we encounter them. */
    if (cust.assigned())
        return;

    /* Containers for storing outputs */
    cargo::DistanceInt cost;
    cargo::DistanceInt best_cost = cargo::InfinityInt;
    std::vector<cargo::Stop> schedule, best_schedule;
    std::vector<cargo::Waypoint> route, best_route;

    /* best_vehicle will point to an underlying MutableVehicle in our grid */
    std::shared_ptr<cargo::MutableVehicle> best_vehicle;
    bool matched = false;

    /* Get candidates from the local grid index
     * (the grid is refreshed during listen()) */
    cargo::DistanceInt range = cargo::pickup_range(cust, cargo::Cargo::now());
    auto candidates = grid_.within_about(range, cust.origin());

    /* Grid::within_about() returns a vector of pointers to the underlying
     * MutableVehicles. Loop through them and check which is the greedy match */
    for (const auto& cand : candidates) {
        cost = cargo::sop_insert(*cand, cust, schedule, route); // TODO make sop_insert accept a pointer as 1st arg
        bool within_time = cargo::check_timewindow_constr(schedule, route);
        if ((cost < best_cost) && within_time) {
            best_cost = cost;
            best_schedule = schedule;
            best_route = route;
            best_vehicle = cand; // copy the pointer
            matched = true;
        }
    }

    /* Commit match to the db. Also refresh our local grid index, so data is
     * fresh for other handle_customers that occur before the next listen(). */
    if (matched) {
        grid_.refresh(best_vehicle, best_route, best_schedule); // <-- update local
        commit(cust, *best_vehicle, best_route, best_schedule); // <-- write to the db TODO make commit accept pointer as 2nd arg
        print_success << "Match (cust" << cust.id() << ", veh" << best_vehicle->id() << ")\n";
        nmatches++;
    }
}

void GreedyInsertion::handle_vehicle(const cargo::Vehicle& veh)
{
    /* Insert vehicles into my grid */
    grid_.insert(veh);
}

void GreedyInsertion::end()
{
    /* Print an informative message */
    print_success << "Matches: "<<nmatches<<std::endl;
}

void GreedyInsertion::listen()
{
    /* Clear the index, then call base listen() */
    grid_.clear();
    RSAlgorithm::listen();
}

int main()
{
    cargo::Options op;
    op.path_to_roadnet = "../../data/roadnetwork/mny.rnet";
    op.path_to_gtree   = "../../data/roadnetwork/mny.gtree";
    op.path_to_edges   = "../../data/roadnetwork/mny.edges";
    //op.path_to_problem = "../../data/benchmark/rs-sm-4.instance";
    op.path_to_problem = "../../data/benchmark/rs-lg-5.instance";
    op.path_to_solution= "a.sol";
    op.time_multiplier = 10;
    op.vehicle_speed   = 10;
    op.matching_period = 60;

    // op.path_to_roadnet = "../../data/roadnetwork/tiny.rnet";
    // op.path_to_gtree   = "../../data/roadnetwork/tiny.gtree";
    // op.path_to_edges   = "../../data/roadnetwork/tiny.edges";
    // op.path_to_problem = "../../data/benchmark/tiny/tiny-n1m2.instance";
    // op.time_multiplier = 1;
    // op.vehicle_speed   = 1;
    // op.matching_period = 10;

    cargo::Cargo cargo(op);

    GreedyInsertion gr;
    cargo.start(gr);
}

