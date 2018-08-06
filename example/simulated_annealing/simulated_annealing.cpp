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
#include <iostream> /* std::endl */
#include <unordered_map>
#include <vector>

#include "simulated_annealing.h"
#include "libcargo.h"

using namespace cargo;

/* GreedyInsertion extends RSAlgorithm by implementing matching functionality.
 * The base RSAlgorithm is initialized with name (param1) and bool (param2).
 * Setting bool=true causes this algorithm's messages to be output to a named
 * pipe on the filesystem, while Cargo simulator messages are output to
 * standard out. Setting bool=false causes all messages to be output to
 * standard out. */
GreedyInsertion::GreedyInsertion()
    : RSAlgorithm("greedy_insertion"),
      grid_(100) /* <-- Initialize my 100x100 grid (see grid.h) */ {
  batch_time() = 1;  // Set batch to 1 second
  nmat_ = 0;         // Initialize my private counter
  delay_ = {};
}

void GreedyInsertion::handle_customer(const Customer& cust) {

  /* Don't consider customers that are assigned but not yet picked up */
  if (cust.assigned()) return;

  /* Don't consider customers already looked at within the last 10 seconds */
  if (delay_.count(cust.id()) && delay_.at(cust.id()) >= Cargo::now() - 10) return;

  /* Containers for storing outputs */
  DistInt cst, best_cst = InfInt;
  std::vector<Stop> sch, best_sch;
  std::vector<Wayp> rte, best_rte;

  /* The grid stores MutableVehicles. During one simulation step, a vehicle
   * may be matched, altering its schedule and route, then become a candidate
   * for another customer in the same step. During a match, the database
   * copy of the vehicle is updated. The local copy in the grid also needs to
   * be updated. Hence the grid returns pointers to vehicles to give access to
   * the local vehicles. */
  std::shared_ptr<MutableVehicle> best_vehl;
  bool matched = false;

  /* Get candidates from the local grid index */
  DistInt rng = pickup_range(cust, Cargo::now());
  auto candidates = grid_.within_about(rng, cust.orig());

  /* Loop through candidates and check which is the greedy match */
  for (const auto& cand : candidates) {
    if (cand->queued() == cand->capacity())
      continue;  // don't consider vehs already queued to capacity

    cst = sop_insert(cand, cust, sch, rte);  // (functions.h)
    bool within_time = chktw(sch, rte);      // (functions.h)
    if ((cst < best_cst) && within_time) {
      best_cst = cst;
      best_sch = sch;
      best_rte = rte;
      best_vehl = cand;  // copy the pointer
      matched = true;
    }
  }

  /* Commit match to the db. */
  if (matched) {
    auto old_rte = best_vehl->route().data();
    auto old_sch = best_vehl->schedule().data();
    best_vehl->set_rte(best_rte);
    best_vehl->set_sch(best_sch);
    /* Function assign(3) will synchronize the vehicle (param3) before
     * committing it the database. Synchronize is necessary due to "match
     * latency". The vehicle may have moved since the match began computing,
     * hence the version of the vehicle used in the match computation is stale.
     * Synchronization trims the traveled part of the vehicle's route and the
     * visited stops that occurred during the latency, and will also re-route
     * the vehicle if it's missed an intersection that the match instructs it
     * to make. */
    if (assign({cust.id()}, {}, *best_vehl)) {
      print(MessageType::Success) << "Match "
          << "(cust" << cust.id() << ", veh" << best_vehl->id() << ")"
          << std::endl;
      nmat_++;
    } else {
      best_vehl->set_rte(old_rte);
      best_vehl->set_sch(old_sch);
    }
    if (delay_.count(cust.id())) delay_.erase(cust.id());
  } else {
    delay_[cust.id()] = Cargo::now();
  }
}

void GreedyInsertion::handle_vehicle(const Vehicle& vehl) {
  /* Insert each vehicle into the grid */
  grid_.insert(vehl);
}

void GreedyInsertion::end() {
  /* Print the total matches */
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
}

void GreedyInsertion::listen() {
  /* Clear the index, then call listen() */
  grid_.clear();
  RSAlgorithm::listen();
}

int main() {
  /* Set the options */
  Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/mny.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/mny.gtree";
  op.path_to_edges    = "../../data/roadnetwork/mny.edges";
  // op.path_to_problem  = "../../data/benchmark/rs-mny-small.instance";
  op.path_to_problem  = "../../data/benchmark/rs-lg-5.instance";
  op.path_to_solution = "a.sol";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 10;
  op.matching_period  = 60;

  Cargo cargo(op);

  /* Initialize a new greedy */
  GreedyInsertion gr;

  /* Start Cargo */
  cargo.start(gr);
}

