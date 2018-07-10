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
#include <vector>
#include <climits>

#include "greedy_insertion.h"
#include "libcargo.h"

GreedyInsertion::GreedyInsertion()
    : RSAlgorithm("greedy_insertion"),
      grid_(100)  /* <-- Initialize my 100x100 grid (see grid.h) */ {
  batch_time() = 1;  // Set batch to 1 second
  nmat_ = 0;      // Initialize my private counter
}

void GreedyInsertion::handle_customer(const cargo::Customer& cust) {
  /* Don't consider customers that are assigned but not yet picked up */
  if (cust.assigned()) return;

  /* Containers for storing outputs */
  cargo::DistInt cst, best_cst = cargo::InfInt;
  std::vector<cargo::Stop> sch, best_sch;
  std::vector<cargo::Wayp> rte, best_rte;

  /* best_vehl will point to an underlying MutableVehicle in our grid */
  std::shared_ptr<cargo::MutableVehicle> best_vehl;
  bool matched = false;

  /* Get candidates from the local grid index
   * (the grid is refreshed during listen()) */
  cargo::DistInt rng = cargo::pickup_range(cust, cargo::Cargo::now());
  auto candidates = grid_.within_about(rng, cust.orig());

  /* Loop through candidates and check which is the greedy match */
  for (const auto& cand : candidates) {
    if (cand->queued() == cand->capacity())
      continue;  // don't consider vehs already queued to capacity

    cst = cargo::sop_insert(cand, cust, sch, rte);  // <-- functions.h
    bool within_time = cargo::check_timewindow_constr(sch, rte);
    if ((cst < best_cst) && within_time) {
      best_cst = cst;
      best_sch = sch;
      best_rte = rte;
      best_vehl = cand;  // copy the pointer
      matched = true;
    }
  }

  /* Commit match to the db. Also refresh our local grid index, so data is
   * fresh for other handle_customers that occur before the next listen(). */
  if (matched) {
    std::vector<cargo::Wayp> sync_rte;
    std::vector<cargo::Stop> sync_sch;
    cargo::DistInt sync_nnd;
    if (commit({cust}, {}, best_vehl, best_rte, best_sch, sync_rte, sync_sch, sync_nnd)) {
      grid_.commit(best_vehl, sync_rte, sync_sch, sync_nnd);
      print_success << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")\n";
      nmat_++;
    }
  }
}

void GreedyInsertion::handle_vehicle(const cargo::Vehicle& vehl) {
  grid_.insert(vehl);  // Insert into my grid
}

void GreedyInsertion::end() {
  print_success << "Matches: " << nmat_ << std::endl;  // Print a msg
}

void GreedyInsertion::listen() {
  grid_.clear();          // Clear the index...
  RSAlgorithm::listen();  // ...then call listen()
}

int main() {
  /* Set the options */
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/mny.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/mny.gtree";
  op.path_to_edges    = "../../data/roadnetwork/mny.edges";
  op.path_to_problem  = "../../data/benchmark/rs-lg-5.instance";
  op.path_to_solution = "a.sol";
  op.time_multiplier  = 5;
  op.vehicle_speed    = 10;
  op.matching_period  = 60;

  cargo::Cargo cargo(op);

  /* Initialize a new greedy */
  GreedyInsertion gr;

  /* Start Cargo */
  cargo.start(gr);
}

