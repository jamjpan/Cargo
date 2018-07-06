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
#include <algorithm> /* std::random_shuffle, std::remove_if, std::find_if */
#include <exception>
#include <iostream> /* std::endl */
#include <thread>
#include <vector>

#include "bilateral_arrangement.h"
#include "libcargo.h"

using namespace cargo;

BilateralArrangement::BilateralArrangement()
    : RSAlgorithm("ba"),
      grid_(100)  /* <-- Initialize my 100x100 grid (see grid.h) */ {
  batch_time() = 1;  // Set batch to 1 second
  nmat_ = 0;      // Initialize my private counter
}

void BilateralArrangement::handle_vehicle(const cargo::Vehicle& vehl) {
  grid_.insert(vehl);  // Insert into my grid
}

void BilateralArrangement::match() {
  std::vector<Customer> custs = customers();  // <-- get a local copy
  std::random_shuffle(custs.begin(), custs.end()); // <-- random it

  /* Extract riders one at a time */
  while (!custs.empty()) {
    Customer cust = custs.back();
    custs.pop_back();
    if (cust.assigned()) continue; // <-- skip already assigned

    /* Containers for storing outputs */
    cargo::DistInt cst, best_cst = cargo::InfInt;
    std::vector<cargo::Stop> sch, best_sch;
    std::vector<cargo::Wayp> rte, best_rte;
    CustId removed_cust = -1;

    /* best_vehl will point to an underlying MutableVehicle in our grid */
    std::shared_ptr<cargo::MutableVehicle> best_vehl = nullptr;
    bool matched = false;

    /* Get candidates from the grid */
    DistInt rng = pickup_range(cust, Cargo::now());
    auto candidates = grid_.within_about(rng, cust.orig());

    while (!candidates.empty()) {
      /* Loop through and get the greedy match */;
      for (const auto& cand : candidates) {
        cst = sop_insert(cand, cust, sch, rte); // <-- doesn't check time/cap constraints
        if (cst < best_cst) {
          best_cst = cst;
          best_sch = sch;
          best_rte = rte;
          best_vehl = cand;  // copy the pointer
        }
      }
      if (best_vehl == nullptr) break; // should never happen but just to be safe
      candidates.erase(std::find(candidates.begin(), candidates.end(), best_vehl));
      bool within_time = check_timewindow_constr(best_sch, best_rte);
      bool within_cap = (best_vehl->queued() < best_vehl->capacity());
      if (within_time && within_cap) {
        matched = true;
        break;
      } else {
        // print_info << "best candidate is infeasible" << std::endl;
        /* Remove some random not-picked-up customer from cand and try the
         * insertion again. If it meets constraints, then accept. */
        std::vector<Stop> randomized_schedule(best_vehl->schedule().data().begin()+1,
                                              best_vehl->schedule().data().end()-1);

        std::random_shuffle(randomized_schedule.begin(), randomized_schedule.end());
        /* Look for the first pair of stops with the same owner */
        CustId remove_me = -1;
        for (auto i = randomized_schedule.begin(); i != randomized_schedule.end(); ++i) {
          auto j = std::find_if(i, randomized_schedule.end(), [&](const Stop& a) { return a.owner() == i->owner(); });
          if (j != randomized_schedule.end()) {
            remove_me = i->owner();
            break;
          }
        }
        if (remove_me != -1) {
          // print_info << "trying to remove " << remove_me << std::endl;
          // print_info << "before remove:";
          // for (const auto& stop : best_vehl->schedule().data())
          //   print_info << " " << stop.loc();
          // print_info << std::endl;
          std::vector<Stop> new_sch = best_vehl->schedule().data();
          new_sch.erase(std::remove_if(new_sch.begin(), new_sch.end(), [&](const Stop& a)
                      { return a.owner() == remove_me; }), new_sch.end());
          best_vehl->set_schedule(new_sch);
          // print_info << "after remove:";
          // for (const auto& stop : best_vehl->schedule().data())
          //   print_info << " " << stop.loc();
          // print_info << std::endl;
          std::vector<Stop> new_best_sch;
          std::vector<Wayp> new_best_rte;
          sop_insert(best_vehl, cust, new_best_sch, new_best_rte);
          // print_info << "after insert:";
          // for (const auto& stop : new_best_sch)
          //   print_info << " " << stop.loc();
          // print_info << std::endl;
          if (check_timewindow_constr(new_best_sch, new_best_rte)) {
            print_info << "feasible after remove " << std::endl;
            best_sch = new_best_sch;
            best_rte = new_best_rte;
            matched = true;
            removed_cust = remove_me;
            break;
          } else {
            // print_info << "still infeasible" << std::endl;
          }
        }
      }
    } // end while !candidates.empty()

    /* Commit */
    if (matched) {
      std::vector<cargo::Wayp> sync_rte;
      std::vector<cargo::Stop> sync_sch;
      cargo::DistInt sync_nnd;
      std::vector<CustId> cust_to_del {};
      if (removed_cust != -1) cust_to_del.push_back(removed_cust);
      if (commit({cust}, {cust_to_del}, best_vehl, best_rte, best_sch, sync_rte, sync_sch, sync_nnd)) {
        grid_.commit(best_vehl, sync_rte, sync_sch, sync_nnd);
        print_success << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")\n";
        nmat_++;
      } else {
        print_warning << "Commit rejected" << std::endl;
      }
    }

    // TODO: Add the removed cust back to waiting_customers_!

  } // end while !custs.empty()
}

void BilateralArrangement::end() {
  print_success << "Matches: " << nmat_ << std::endl;  // Print a msg
}

void BilateralArrangement::listen() {
  grid_.clear();          // Clear the index...
  RSAlgorithm::listen();  // ...then call listen()
}

int main() {
  /* Set the options */
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-1.instance";
  op.path_to_solution = "a.sol";
  op.time_multiplier  = 5;
  op.vehicle_speed    = 10;
  op.matching_period  = 60;

  cargo::Cargo cargo(op);

  /* Initialize a new bilateral-arrangement alg */
  BilateralArrangement ba;

  /* Start Cargo */
  cargo.start(ba);
}

