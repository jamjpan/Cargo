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

#include "greedy_insertion.h"
#include "libcargo.h"

using namespace cargo;

const int RETRY = 15; // seconds

GreedyInsertion::GreedyInsertion() : RSAlgorithm("greedy_insertion"),
      grid_(100) {   // (grid.h)
  batch_time() = 1;  // (rsalgorithm.h) set batch to 1 second ("streaming")

  /* Private vars */
  nmat_  = 0;  // match counter
  nrej_  = 0;  // number rejected due to out-of-sync
  delay_ = {}; // delay container
}

void GreedyInsertion::handle_customer(const Customer& cust) {
  /* Skip customers already assigned (but not yet picked up) */
  if (cust.assigned())
    return;

  /* Skip customers looked at within the last RETRY seconds */
  if (delay_.count(cust.id()) && delay_.at(cust.id()) >= Cargo::now() - RETRY)
    return;

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
  std::shared_ptr<MutableVehicle> best_vehl = nullptr;
  bool matched = false;

  /* Get candidates from the local grid index */
  DistInt rng = /* pickup_range(cust, Cargo::now()); */ 2000;
  auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

  /* Increment number-of-custs counter */
  ncust_++;

  /* Loop through candidates and check which is the greedy match */
  for (const auto& cand : candidates) {
    /* Skip vehicles queued to capacity (queued = number of customer assigned
     * but may or may not be picked up yet) */
    if (cand->queued() == cand->capacity())
      continue;

    /* Increment number-of-candidates counter */
    ncand_++;

    cst = sop_insert(cand, cust, sch, rte);  // (functions.h)
    bool within_time = chktw(sch, rte);      // (functions.h)
    if ((cst < best_cst) && within_time) {
      best_cst = cst;
      best_sch = sch;
      best_rte = rte;
      best_vehl = cand;  // copy the pointer
      matched = true;
    }
  }  // end for cand : candidates

  /* Commit match to the db */
  bool add_to_delay = true;
  if (matched) {
    /* Function assign(3) will synchronize the vehicle (param3) before
     * committing it the database. Synchronize is necessary due to "match
     * latency". The vehicle may have moved since the match began computing,
     * hence the version of the vehicle used in the match computation is stale.
     * Synchronization trims the traveled part of the vehicle's route and the
     * visited stops that occurred during the latency, and will also re-route
     * the vehicle if it's missed an intersection that the match instructs it
     * to make. */
    if (assign({cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      print(MessageType::Success)
        << "Match " << "(cust" << cust.id() << ", veh" << best_vehl->id() << ")"
        << std::endl;
      nmat_++;  // increment matched counter
      /* Remove customer from delay storage */
      if (delay_.count(cust.id())) delay_.erase(cust.id());
      add_to_delay = false;
    } else
      nrej_++;  // increment rejected counter
  }
  if (add_to_delay) {
    /* Add customer to delay storage */
    delay_[cust.id()] = Cargo::now();
  }
}

void GreedyInsertion::handle_vehicle(const Vehicle& vehl) {
  /* Insert each vehicle into the grid */
  grid_.insert(vehl);
}

void GreedyInsertion::end() {
  /* Print the statistics */
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
}

void GreedyInsertion::listen() {
  /* Clear counters */
  ncand_ = 0;  // candidates counter
  ncust_ = 0;  // customers counter

  /* Clear the index, then call base listen */
  grid_.clear();
  RSAlgorithm::listen();

  /* Report number of candidates per customer */
  print(MessageType::Info)
    << "Cand per cust: " << (float)ncand_/ncust_ << std::endl;
}

int main() {
  /* Set options */
  Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  op.path_to_solution = "greedy_insertion.sol";
  op.path_to_dataout  = "greedy_insertion.dat";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  /* Construct Cargo */
  Cargo cargo(op);

  /* Initialize algorithm */
  GreedyInsertion gr;

  /* Start the simulation */
  cargo.start(gr);

  return 0;
}

