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
#include <chrono>
#include <ctime>
#include <iostream> /* std::endl */
#include <unordered_map>
#include <vector>

#include "greedy_insertion.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH     = 1;  // seconds
const int RANGE     = 1500; // meters
const int TIMEOUT   = 1;  // timeout customers take > TIMEOUT sec

std::vector<int> avg_dur {};

typedef std::chrono::duration<double, std::milli> dur_milli;
typedef std::chrono::milliseconds milli;

/* Define ordering of rank_cands */
auto cmp = [](rank_cand left, rank_cand right) {
  return std::get<0>(left) > std::get<0>(right); };

bool GreedyInsertion::timeout(clock_t& start) {
  clock_t end = std::clock();
  double elapsed = double(end-start)/CLOCKS_PER_SEC;
  if ((elapsed >= TIMEOUT) || this->done()) {
    print << "timeout() triggered." << std::endl;
    return true;
  }
  return false;
}

GreedyInsertion::GreedyInsertion() : RSAlgorithm("greedy_insertion"),
      grid_(100) {       // (grid.h)
  batch_time() = BATCH;  // (rsalgorithm.h) set batch to 1 second ("streaming")
  nmat_  = 0;  // match counter
  nrej_  = 0;  // number rejected due to out-of-sync
}

void GreedyInsertion::handle_customer(const Customer& cust) {
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;

  // Start timing -------------------------------
  t0 = std::chrono::high_resolution_clock::now();
  clock_t start = std::clock();

  /* Skip customers already assigned (but not yet picked up) */
  if (cust.assigned())
    return;

  int ncust = 1;

  /* Containers for storing outputs */
  DistInt cst = InfInt;
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

  /* Increment number-of-custs counter */
  ncust_++;

  /* Get candidates from the local grid index */
  DistInt rng = /* pickup_range(cust, Cargo::now()); */ RANGE;
  auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

  /* Container to rank candidates by least cost */
  std::priority_queue<rank_cand, std::vector<rank_cand>, decltype(cmp)> q(cmp);

  /* Loop through and rank each candidate */
  for (const auto& cand : candidates) {
    /* Increment number-of-candidates counter */
    ncand_++;
    cst = sop_insert(*cand, cust, sch, rte); // doesn't check time/cap constraints
    q.push({cst, cand, sch, rte});
  }

  /* Loop through candidates and check which is the greedy match */
  while (!q.empty() && !matched) {
    /* Get and unpack the best candidate */
    auto cand = q.top(); q.pop();
    best_vehl = std::get<1>(cand);
    best_sch  = std::get<2>(cand);
    best_rte  = std::get<3>(cand);

    /* If best vehicle is within constraints... */
    bool within_time = chktw(best_sch, best_rte);
    bool within_cap = (best_vehl->queued() < best_vehl->capacity());
    if (within_time && within_cap) {
      /* ... accept the match */
      matched = true;
    }
    if (timeout(start))
      break;
  }

  /* Commit match to the db */
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
    } else
      nrej_++;  // increment rejected counter
  }
  t1 = std::chrono::high_resolution_clock::now();
  // Stop timing --------------------------------
  avg_dur.push_back(std::round(dur_milli(t1-t0).count())/float(ncust));
}

void GreedyInsertion::handle_vehicle(const Vehicle& vehl) {
  /* Insert each vehicle into the grid */
  grid_.insert(vehl);
}

void GreedyInsertion::end() {
  /* Print the statistics */
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
  int sum_avg = 0; for (auto& n : avg_dur) sum_avg += n;
  print(MessageType::Success) << "Avg-cust-handle: " << (float)sum_avg/avg_dur.size() << "ms" << std::endl;
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

