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
  if (this->first) {
    print << "batchsize=" << this->customers().size() << std::endl;
    print << "timeout=" << this->timeout_ << std::endl;
    this->first=false;
  }
  this->beg_ht();
  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  print << "Handling cust " << cust.id() << " (" << this->candidates.size() << " candidates)" << std::endl;

  DistInt best_cost = InfInt;

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed heuristic: try only if vehicle's current schedule len < 8 customer stops
    if (cand->schedule().data().size() < 10) {
      DistInt cost = sop_insert(*cand, cust, sch, rte) - cand->route().cost();
      if (cost < best_cost) {
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          best_vehl = cand;
          best_sch = sch;
          best_rte = rte;
          best_cost = cost;
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
    this->assign_or_delay({cust.id()}, {}, best_rte, best_sch, *best_vehl, true);
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
  this->first = true;
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
  option.path_to_roadnet  = "../../data/roadnetwork/cd1.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/cd1.gtree";
  option.path_to_edges    = "../../data/roadnetwork/cd1.edges";
  // option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_problem  = "../../tool/rspgen/a.instance";
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

