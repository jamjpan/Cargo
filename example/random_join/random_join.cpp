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
#include <algorithm>
#include <iostream>
#include <queue>
#include <random>

#include "libcargo.h"
#include "random_join.h"

using namespace cargo;

const int BATCH = 30;

RandomJoin::RandomJoin()
    : RSAlgorithm("random_join", false), grid_(100) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

void RandomJoin::match() {
  this->beg_batch_ht();
  this->reset_workspace();

  for (const Customer& cust : this->customers()) {
    this->candidates = this->grid_.within(pickup_range(cust), cust.orig());
    std::shuffle(candidates.begin(), candidates.end(), this->gen);
    for (const MutableVehicleSptr& cand : this->candidates) {
      sop_insert(cand, cust, this->sch, this->rte);
      if (chktw(this->sch, this->rte) && chkcap(cand->capacity(), this->sch)) {
        this->end_ht();
        this->assign_or_delay({cust.id()}, {}, this->rte, this->sch, *cand);
        break;
      }
      if (this->timeout(this->timeout_0))
        return;
    }
  }

  this->end_batch_ht();
}

void RandomJoin::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void RandomJoin::end() {
  this->print_statistics();
}

void RandomJoin::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

void RandomJoin::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->timeout_0 = hiclock::now();
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-m5k-c3.instance";
  option.path_to_solution = "random_join.sol";
  option.path_to_dataout  = "random_join.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode = false;
  option.static_mode = true;
  Cargo cargo(option);
  RandomJoin rj;
  cargo.start(rj);

  return 0;
}

