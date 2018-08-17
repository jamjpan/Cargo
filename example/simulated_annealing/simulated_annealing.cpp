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
const int RANGE = 1500; // meters
const int PERT  = 12; // how many solutions to generate per iter?

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
  std::unordered_map<Customer, std::vector<std::shared_ptr<MutableVehicle>>> candstab;
  std::vector<Sol> sol {};
  Sol best_sol;
  DistInt best_solcst = InfInt;

  /* Initialize sol containers */
  for (int i = 0; i < PERT; ++i)
    sol.push_back({});

  /* Get candidates per customer */
  for (const Customer& cust : customers()) {
    /* Skip customers already assigned (but not yet picked up) */
    if (cust.assigned())
      continue;
    /* Get candidates from the local grid index */
    DistInt rng = /* pickup_range(cust, Cargo::now()); */ RANGE;
    candstab[cust] = grid_.within_about(rng, cust.orig()); // (grid.h)
  }

  /* Generate a solution:
   * Assign each customer to a random one of its candidates */
  for (int i = 0; i < PERT; ++i) {
    for (const auto& kv : candstab) {
      const Customer& cust = kv.first;
      auto& candidates = candstab.at(cust);
      if (candidates.size() == 0)
        continue;
      std::shuffle(candidates.begin(), candidates.end(), gen);
      auto& cand = candidates.front();
      DistInt cst = cargo::sop_insert(cand, cust, sch, rte);
      if (cargo::chktw(sch, rte)) {
        sol[i][cust.id()] = {cst, cand, sch, rte};
      }
    }
    DistInt solcst = 0;
    for (const auto& kv : sol[i])
      solcst += std::get<0>(kv.second);
    if (i == 0 || hillclimb(i) || solcst < best_solcst) {
      best_sol = sol[i];
      best_solcst = solcst;
    }
  }

  /* Commit the solution */
  for (const auto& kv : best_sol) {
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
  return d(gen) <= std::exp((-1)*T/1.5) ? true : false;
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

