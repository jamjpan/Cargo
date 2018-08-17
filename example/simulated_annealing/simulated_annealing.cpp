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
#include <chrono>
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
const int PERT  = 30; // how many solutions to generate per iter?
const int T_MAX = 5; // maximum "temperature"

std::vector<int> avg_dur {};

typedef std::chrono::duration<double, std::milli> dur_milli;
typedef std::chrono::milliseconds milli;

SimulatedAnnealing::SimulatedAnnealing() : RSAlgorithm("simulated_annealing"),
    grid_(100), d(0,1) {
  batch_time() = BATCH;
  nmat_ = 0;
  nclimbs_ = 0;
  timeout_ = std::ceil((float)BATCH/T_MAX*(1000.0));
  std::random_device rd;
  gen.seed(rd());
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  grid_.insert(vehl);
}

void SimulatedAnnealing::match() {
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  int ncust = 0;

  // Start timing -------------------------------
  t0 = std::chrono::high_resolution_clock::now();
  auto start = std::chrono::high_resolution_clock::now();

  /* Initialize is_matched to false for all customers */
  std::unordered_map<CustId, bool> is_matched {};
  for (const Customer& cust : customers())
    is_matched[cust.id()] = false;

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
    /* Skip customers under delay */
    if (delay(cust.id()))
      continue;
    ncust++;
    /* Get candidates from the local grid index */
    DistInt rng = /* pickup_range(cust, Cargo::now()); */ RANGE;
    candstab[cust] = grid_.within_about(rng, cust.orig()); // (grid.h)
  }

  /* Generate a solution:
   * Assign each customer to a random one of its candidates
   * Timeouts: cand generation; perturbing; temperature loop */
  for (int T = 0; T < T_MAX; ++T) {
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
        if (timeout(start))
          break;
      }
      DistInt solcst = 0;
      for (const auto& kv : sol[i])
        solcst += std::get<0>(kv.second);
      if (i == 0) {
        best_sol = sol[i];
        best_solcst = solcst;
      }
      else if (hillclimb(T)) {
        best_sol = sol[i];
        best_solcst = solcst;
        nclimbs_++;
      }
      else if (solcst < best_solcst) {
        best_sol = sol[i];
        best_solcst = solcst;
      }
      if (timeout(start))
        break;
    }
    if (timeout(start))
      break;
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
      is_matched.at(cust_id) = true;
      end_delay(cust_id);  // (rsalgorithm.h)
    } else
      nrej_++;  // increment rejected counter
  }
  for (const auto& kv : is_matched)
    if (!kv.second) beg_delay(kv.first);
  t1 = std::chrono::high_resolution_clock::now();
  // Stop timing --------------------------------
  avg_dur.push_back(std::round(dur_milli(t1-t0).count())/float(ncust));
}

void SimulatedAnnealing::end() {
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Climbs: " << nclimbs_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
  int sum_avg = 0; for (auto& n : avg_dur) sum_avg += n;
  this->avg_cust_ht_ = sum_avg/avg_dur.size();
  print(MessageType::Success) << "Avg-cust-handle: " << avg_cust_ht_ << "ms" << std::endl;
}

void SimulatedAnnealing::listen() {
  grid_.clear();
  RSAlgorithm::listen();
}

bool SimulatedAnnealing::hillclimb(int& T) {
  return d(gen) <= std::exp((-1)*T/1) ? true : false;
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

