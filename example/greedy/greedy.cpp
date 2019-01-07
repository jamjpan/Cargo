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

#include "greedy.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

GreedyInsertion::GreedyInsertion()
    : RSAlgorithm("greedy", false), grid_(100) {
  this->batch_time() = BATCH;
}

void GreedyInsertion::handle_customer(const Customer& cust) {
  this->beg_ht();
  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed heuristic: try only if vehicle's current schedule has < 8 customer stops
    if (cand->schedule().data().size() < 10) {
      DistInt cost = sop_insert(*cand, cust, sch, rte, Cargo::gtree()) - cand->route().cost();
      if (cost < this->best_cost) {
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          this->best_vehl = cand;
          this->best_sch = sch;
          this->best_rte = rte;
          this->best_cost = cost;
        }
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (this->best_vehl != nullptr)
    this->matched = true;

  this->end_ht();

  if (this->matched) {
    print << "Matched " << cust.id() << " with " << this->best_vehl->id() << std::endl;
    this->assign_or_delay(
      {cust.id()}, {}, this->best_rte, this->best_sch, *(this->best_vehl));
  } else {
    this->beg_delay(cust.id());
  }
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
  this->best_cost = InfInt;
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
  option.path_to_problem  = "../../data/benchmark/rs-m5k-c3.instance";
  option.path_to_solution = "greedy.sol";
  option.path_to_dataout  = "greedy.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode = false;
  option.static_mode = true;
  Cargo cargo(option);
  GreedyInsertion gr;
  cargo.start(gr);

  return 0;
}

