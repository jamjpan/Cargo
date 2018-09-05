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
#include <iostream> /* std::endl */
#include <vector>

#include "nearest_neighbor.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 1;  // seconds
const int RANGE = 1500; // meters
const int K_NN  = 10; // how many nearest candidates to evaluate before give up

std::vector<int> avg_dur {};

typedef std::chrono::duration<double, std::milli> dur_milli;
typedef std::chrono::milliseconds milli;

NearestNeighbor::NearestNeighbor()
    : RSAlgorithm("nearest_neighbor"),
      grid_(100)  /* <-- Initialize my 100x100 grid (see grid.h) */ {
  batch_time() = BATCH;
  nmat_ = 0;
}

void NearestNeighbor::handle_customer(const cargo::Customer& cust) {
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  this->timeout_ = std::ceil((float)BATCH/customers().size()*(1000.0));

  // Start timing -------------------------------
  t0 = std::chrono::high_resolution_clock::now();
  auto start = t0;

  /* Skip customers already assigned (but not yet picked up) */
  if (cust.assigned())
    return;
  /* Skip customers under delay */
  if (delay(cust.id()))
    return;

  int ncust = 1;

  /* Containers for storing outputs */
  std::vector<cargo::Stop> sch, best_sch;
  std::vector<cargo::Wayp> rte, best_rte;

  /* best_vehl will point to an underlying MutableVehicle in our grid */
  std::shared_ptr<cargo::MutableVehicle> best_vehl;
  bool matched = false;

  /* Get candidates from the local grid index
   * (the grid is refreshed during listen()) */
  DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
  auto candidates = grid_.within_about(rng, cust.orig());

  /* Find K_NN nearest candidates
   * Complexity: O(K_NN*|vehicles|)
   * (Timeout the K_NN loop) */
  std::vector<DistDbl> best_euc(K_NN, InfDbl);
  std::vector<std::shared_ptr<MutableVehicle>> nnv(K_NN, nullptr);
  for (int i = 0; i < K_NN; ++i) {  // O(K_NN)
    for (const auto& cand : candidates) {  // O(|vehicles|)
      DistDbl dist = haversine(cand->last_visited_node(), cust.orig()); // <-- libcargo/distance.h
      bool min_dist = (dist < best_euc[i]);
      if (i > 0) min_dist = (min_dist && (dist > best_euc[i-1]));
      if (min_dist) {
        best_euc[i] = dist;
        nnv[i] = cand;
      }
    }
    if (timeout(start))
      break;
  }

  /* Loop through candidates in order of nearest first
   * (Timeout) */
  for (const auto& cand : nnv) {
    if (cand == nullptr) break;  // no more candidates
    if (cand->queued() == cand->capacity())
      continue;  // don't consider vehs already queued to capacity
    cargo::sop_insert(cand, cust, sch, rte);  // <-- functions.h
    if (cargo::chktw(sch, rte)) {
      best_sch = sch;
      best_rte = rte;
      best_vehl = cand;  // copy the pointer
      matched = true;
      break;
    }
    if (timeout(start))
      break;
  }

  /* Commit match to the db. Also refresh our local grid index, so data is
   * fresh for other handle_customers that occur before the next listen(). */
  if (matched) {
    if (assign({cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      print(MessageType::Success) << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")\n";
      nmat_++;
      end_delay(cust.id());  // (rsalgorithm.h)
    }
    else
      nrej_++;
  } else
    beg_delay(cust.id());
  t1 = std::chrono::high_resolution_clock::now();
  // Stop timing --------------------------------
  if (ncust > 0)
    avg_dur.push_back(std::round(dur_milli(t1-t0).count())/float(ncust));
}

void NearestNeighbor::handle_vehicle(const cargo::Vehicle& vehl) {
  grid_.insert(vehl);  // Insert into my grid
}

void NearestNeighbor::end() {
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;  // Print a msg
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
  int sum_avg = 0; for (auto& n : avg_dur) sum_avg += n;
  this->avg_cust_ht_ = sum_avg/avg_dur.size();
  print(MessageType::Success) << "Avg-cust-handle: " << avg_cust_ht_ << "ms" << std::endl;
}

void NearestNeighbor::listen() {
  grid_.clear();          // Clear the index...
  RSAlgorithm::listen();  // ...then call listen()
}

int main() {
  /* Set the options */
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-sm-1.instance";
  op.path_to_solution = "nearest_neighbor.sol";
  op.path_to_dataout  = "nearest_neighbor.dat";
  op.path_to_save = "nearest_neighbor.sqlite3.backup";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  cargo::Cargo cargo(op);

  /* Initialize a new nn alg */
  NearestNeighbor nn;

  /* Start Cargo */
  cargo.start(nn);
}

