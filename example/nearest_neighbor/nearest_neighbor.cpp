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
#include <vector>

#include "libcargo.h"
#include "nearest_neighbor.h"

using namespace cargo;

const int BATCH = 1;                        // batch time, seconds
const int RANGE = 2000;                     // meters

auto cmp = [](rank_cand left, rank_cand right) {
  return std::get<0>(left) > std::get<0>(right);
};

NearestNeighbor::NearestNeighbor()
    : RSAlgorithm("nearest_neighbor"), grid_(100) {
  this->batch_time() = BATCH;               // (rsalgorithm.h)
}

void NearestNeighbor::handle_customer(const Customer& cust) {
  this->beg_ht();                           // begin timing (rsalgorithm.h)
  this->reset_workspace();                  // reset workspace variables
  this->candidates =                        // collect candidates
    this->grid_.within(RANGE, cust.orig()); // (grid.h)

  /* Rank candidates (timeout) */
  std::priority_queue<rank_cand, std::vector<rank_cand>, decltype(cmp)>
    my_q(cmp);                              // rank by nearest
  for (const MutableVehicleSptr& cand : this->candidates) {
    if (cand->queued() < cand->capacity()) {
      DistDbl cst = haversine(cand->last_visited_node(), cust.orig());
      rank_cand rc = {cst, cand};
      my_q.push(rc);                        // O(log(|my_q|))
    }
    if(this->timeout(this->timeout_0))
      break;
  }

  /* Accept nearest valid (timeout) */
  while (!my_q.empty() && !matched) {
    rank_cand rc = my_q.top();              // access order by nearest
    my_q.pop();                             // remove from queue
    best_vehl = std::get<1>(rc);
    sop_insert(best_vehl, cust, sch, rte);  // (functions.h)
    if (chktw(sch, rte))                    // check time window (functions.h)
      matched = true;                       // accept
    if (this->timeout(this->timeout_0))     // (rsalgorithm.h)
      break;
  }

  /* Attempt commit to db */
  if (matched) {
    if (this->assign(                       // (rsalgorithm.h)
      {cust.id()}, {}, rte, sch, *best_vehl)) {
      this->end_delay(cust.id());           // (rsalgorithm.h)
    } else {
      this->nrej_++;                        // increment # rejected counter
      this->beg_delay(cust.id());           // rsalgorithm.h
    }
  }
  if (!matched)                             // add to delay
    this->beg_delay(cust.id());

  this->end_ht();                           // end timing
}

void NearestNeighbor::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);                 // (grid.h)
}

void NearestNeighbor::end() {
  this->print_statistics();                 // (rsalgorithm.h)
}

void NearestNeighbor::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();                      // (grid.h)
  RSAlgorithm::listen(                      // (rsalgorithm.h)
    skip_assigned, skip_delayed);
}

void NearestNeighbor::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->matched = false;
  this->best_vehl = nullptr;
  this->timeout_0 = hiclock::now();
}

int main() {
  Options option;                           // (options.h)
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_solution = "nearest_neighbor.sol";
  option.path_to_dataout  = "nearest_neighbor.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 20;
  option.matching_period  = 60;
  option.static_mode = false;
  Cargo cargo(option);                      // (cargo.h)
  NearestNeighbor nn;
  cargo.start(nn);

  return 0;
}

