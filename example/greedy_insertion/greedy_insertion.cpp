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
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

#include "greedy_insertion.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

GreedyInsertion::GreedyInsertion()
    : RSAlgorithm("greedy_insertion", true), grid_(100) {
  this->batch_time() = BATCH;
}

void GreedyInsertion::handle_customer(const Customer& cust) {
  print << "Handling cust " << cust.id() << std::endl;
  this->beg_ht();
  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  DistInt best_cst = InfInt;

  float ss = 0;
  for (const auto& cand : this->candidates)
    ss += cand->schedule().data().size()-2;

  print << "\tGot " << this->candidates.size()
        << " candidates (range=" << pickup_range(cust) << ", s.avg=" << ss/this->candidates.size() << ")" << std::endl;

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed-up heuristics:
    //   1) Try only if vehicle has capacity at this point in time
    //   2) Try only if vehicle's current schedule len < 8 customer stops
    // if (cand->capacity() > 1 && cand->schedule().data().size() < 10) {
    if (cand->schedule().data().size() < 10) {
      DistInt new_cst = sop_insert(*cand, cust, sch, rte); // TODO consider having sop_insert return the detour cost?
      // DistInt cst = new_cst - cand->route().cost() + cand->next_node_distance();
      DistInt cst = new_cst - cand->route().cost();
      if (cst < 0) {
        print(MessageType::Error) << "Got negative detour!" << std::endl;
        print << cand->id() << std::endl;
        print << cst << " (" << new_cst << "-" << cand->route().cost() << ")" << std::endl;
        print << "Current schedule: ";
        for (const Stop& sp : cand->schedule().data())
          print << sp.loc() << " ";
        print << std::endl;
        print << "nnd: " << cand->next_node_distance() << std::endl;
        print << "New schedule: ";
        for (const Stop& sp : sch)
          print << sp.loc() << " ";
        print << std::endl;
        throw;
      }
      // print << "\t\tVehl " << cand->id() << ": " << cst << std::endl;
      if (cst < best_cst) {
        if (chkcap(cand->capacity(), sch)
         && chktw(sch, rte)) {
          best_vehl = cand;
          best_sch  = sch;
          best_rte  = rte;
          best_cst  = cst;
          // print << "sch: ";
          // for (const Stop& a : best_sch)
          //   print << a.loc() << " ";
          // print << std::endl;
        }
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (best_vehl != nullptr)
    matched = true;

  /* Attempt commit to db */
  if (matched) {
    print << "Matched " << cust.id() << " with " << best_vehl->id() << std::endl;
    this->assign_or_delay(
        {cust.id()}, {}, best_rte, best_sch, *best_vehl);
  } else {
    this->beg_delay(cust.id());
  }

  this->end_ht();
}

void GreedyInsertion::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void GreedyInsertion::end() {
  this->print_statistics();
}

void GreedyInsertion::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void GreedyInsertion::reset_workspace() {
  this->sch = this->best_sch = {};
  this->rte = this->best_rte = {};
  this->candidates = {};
  this->matched = false;
  this->best_vehl = nullptr;
  this->timeout_0 = hiclock::now();
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_solution = "greedy_insertion.sol";
  option.path_to_dataout  = "greedy_insertion.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode = true;
  Cargo cargo(option);
  GreedyInsertion gr;
  cargo.start(gr);

  return 0;
}

