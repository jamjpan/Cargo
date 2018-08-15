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

const int ITER = 5;  // how many iterations per batch?
const int PERT = 2;  // how many perturbed solutions to generate?
const int RETRY = 15;

SimulatedAnnealing::SimulatedAnnealing() : RSAlgorithm("simulated_annealing"),
    grid_(100), d(0,1) {
  batch_time() = 1;  // Set batch to 1 second
  nmat_ = 0;         // Initialize my private counter
  delay_ = {};

  std::random_device rd;
  gen.seed(rd());
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  grid_.insert(vehl);
}

void SimulatedAnnealing::match() {
  /* Containers for storing outputs */
  DistInt cst, best_cst = InfInt;
  std::vector<Stop> sch, best_sch;
  std::vector<Wayp> rte, best_rte;
  Sol sol, best_sol;

  T_ = 0;  // reset "temperature"
  while (T_++ < ITER) {
    auto lcl_custs = customers(); // make a local copy for shuffling

    /* Construct solutions via greedy */
    for (int i = 0; i < PERT; ++i) {
        std::shuffle(lcl_custs.begin(), lcl_custs.end(), gen);
    for (const Customer& cust : lcl_custs) {
      /* Skip customers already assigned (but not yet picked up) */
      if (cust.assigned())
        continue;

      /* Skip customers looked at within the last RETRY seconds */
      if (delay_.count(cust.id()) && delay_.at(cust.id()) >= Cargo::now() - RETRY)
        continue;

      std::shared_ptr<MutableVehicle> best_vehl = nullptr;
      bool matched = false;

      /* Get candidates from the local grid index */
      DistInt rng = /* pickup_range(cust, Cargo::now()); */ 2000;
      auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

      /* Loop through candidates and check which is the greedy match */
      for (const auto& cand : candidates) {
        if (cand->queued() == cand->capacity())
          continue;

        cst = sop_insert(cand, cust, sch, rte);  // (functions.h)
        bool within_time = chktw(sch, rte);      // (functions.h)
        if ((cst < best_cst) && within_time) {
          best_cst = cst;
          best_sch = sch;
          best_rte = rte;
          best_vehl = cand;  // copy the pointer
          matched = true;
        }
      }  // end for cand : candidates

      /* Add match to solution */
      if (matched)
        sol[cust.id()] = {best_vehl, best_sch, best_rte};
      else
        unassigned.push_back(cust.id());
    } // end for cust : customers
    if (best_sol.size() == 0)
      best_sol = sol;
    else if (hillclimb(T_))
      best_sol = sol;
    else if (cost(sol) < cost(best_sol))
      best_sol = sol;
    } // end for i
  } // end while T_ < ITER

  /* Commit the best solution */
  for (const auto& kv : best_sol) {
    auto cust_id = kv.first;
    auto best_vehl = std::get<0>(kv.second);
    auto& best_sch = std::get<1>(kv.second);
    auto& best_rte = std::get<2>(kv.second);
    if (assign({cust_id}, {}, best_rte, best_sch, *best_vehl)) {
      print(MessageType::Success)
        << "Match " << "(cust" << cust_id << ", veh" << best_vehl->id() << ")"
        << std::endl;
      nmat_++;  // increment matched counter
      /* Remove customer from delay storage */
      if (delay_.count(cust_id)) delay_.erase(cust_id);
    } else
      nrej_++;  // increment rejected counter
  }
  for (const CustId& cust_id : unassigned) {
    /* Add customer to delay storage */
    delay_[cust_id] = Cargo::now();
  }
}

void SimulatedAnnealing::end() {
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
}

void SimulatedAnnealing::listen() {
  grid_.clear();
  RSAlgorithm::listen();
}

bool SimulatedAnnealing::hillclimb(int& T) {
  return d(gen) <= std::exp((-1)*T) ? true : false;
}

int SimulatedAnnealing::cost(const Sol& sol) {
  int cost = 0;
  /* Get unassigned penalty */
  for (const CustId& cust_id : unassigned)
    cost += Cargo::basecost(cust_id);
  /* Get assignment costs */
  for (const auto& kv : sol)
    cost += std::get<2>(kv.second).back().first;
  return cost;
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

