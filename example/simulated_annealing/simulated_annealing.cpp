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
#include <algorithm> /* std::shuffle */
#include <cmath> /* std::exp */
#include <iostream> /* std::endl */
#include <random>
#include <unordered_map>
#include <vector>

#include "simulated_annealing.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 1;  // seconds
const int PERT  = 10; // how many solutions to generate per iter?
const int K_NN  = 10; // how many candidates per customer?
const int RETRY = 15;

SimulatedAnnealing::SimulatedAnnealing() : RSAlgorithm("simulated_annealing"),
    grid_(100), d(0,1) {
  batch_time() = BATCH;
  nmat_ = 0;
  nclimbs_ = 0;
  std::random_device rd;
  gen.seed(rd());
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  grid_.insert(vehl);
}

void SimulatedAnnealing::match() {
  /* Containers for storing outputs */
  std::vector<Stop> sch, best_sch;
  std::vector<Wayp> rte, best_rte;
  Sol sol;

  /* Construct solutions via random-nearest-neighbor */
  for (const Customer& cust : customers()) {
    /* Skip customers already assigned (but not yet picked up) */
    if (cust.assigned())
      continue;

    std::shared_ptr<MutableVehicle> best_vehl = nullptr;

    /* Get candidates from the local grid index */
    DistInt rng = /* pickup_range(cust, Cargo::now()); */ 1200;
    auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

    /* Find K_NN nearest candidates
     * Complexity: O(K_NN*|vehicles|) */
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
    }

    /* Generate solutions */
    for (int i = 0; i < PERT; ++i) {
      std::shuffle(nnv.begin(), nnv.end(), gen);
      /* Loop through candidates */
      for (const auto& cand : nnv) {
        if (cand == nullptr) break;  // no more candidates
        if (cand->queued() == cand->capacity())
          continue;  // don't consider vehs already queued to capacity
        DistInt cst = cargo::sop_insert(cand, cust, sch, rte);  // <-- functions.h
        if (cargo::chktw(sch, rte)) {
          /* Decide to keep this assignment or a previous one */
          if (sol.count(cust.id()) == 0)
            sol[cust.id()] = {cst, cand, sch, rte};
          else if (hillclimb(i)) {
            std::get<1>(sol.at(cust.id()))->decr_queued();
            sol[cust.id()] = {cst, cand, sch, rte};
            nclimbs_++;
          }
          else if (cst < std::get<0>(sol.at(cust.id()))) {
            std::get<1>(sol.at(cust.id()))->decr_queued();
            sol[cust.id()] = {cst, cand, sch, rte};
          }
          cand->incr_queued();
          break;
        }
      }
    }
  } // end for cust : customers

  /* Commit the solution */
  for (const auto& kv : sol) {
    auto cust_id = kv.first;
    auto best_vehl = std::get<1>(kv.second);
    auto& best_sch = std::get<2>(kv.second);
    auto& best_rte = std::get<3>(kv.second);
    if (assign({cust_id}, {}, best_rte, best_sch, *best_vehl)) {
      print(MessageType::Success)
        << "Match " << "(cust" << cust_id << ", veh" << best_vehl->id() << ")"
        << std::endl;
      nmat_++;  // increment matched counter
    } else
      nrej_++;  // increment rejected counter
  }
}

void SimulatedAnnealing::end() {
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Climbs: " << nclimbs_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
}

void SimulatedAnnealing::listen() {
  grid_.clear();
  RSAlgorithm::listen();
}

bool SimulatedAnnealing::hillclimb(int& T) {
  return d(gen) <= std::exp((-1)*T/2.5) ? true : false;
}

int main() {
  /* Set the options */
  Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  op.path_to_solution = "simulated_annealing.sol";
  op.path_to_dataout  = "simulated_annealing.dat";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  Cargo cargo(op);
  SimulatedAnnealing sa;
  cargo.start(sa);
}

