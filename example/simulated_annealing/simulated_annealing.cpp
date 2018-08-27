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

const int BATCH = 30;  // seconds
const int RANGE = 2000; // meters
const int PERT  = 100000; // how many solutions to generate per iter?
const int T_MAX = 10; // maximum "temperature"

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
  print(MessageType::Info) << "match called" << std::endl;
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
  std::vector<Stop> sch;
  std::vector<Wayp> rte;

  /* Generate a solution:
  * Assign each customer to a random one of its candidates */
  std::vector<std::tuple<Customer, MutableVehicle, DistInt>> best_sol = {};
  DistInt best_solcst = InfInt;
  /* Don't want to modify the properties of the grid vehicles, so store
   * the queued count here */
  std::unordered_map<VehlId, std::pair<int, int>> qcount, qcount_backup;
  for (const Vehicle& vehl : vehicles()) {
    qcount[vehl.id()] = {vehl.queued(), vehl.capacity()};
  }
  qcount_backup = qcount;
  for (int T = 0; T < T_MAX; ++T) {
    /* Each temperature starts with an initial random solution */
    std::vector<std::tuple<Customer, MutableVehicle, DistInt>> sol = {};
    DistInt this_solcst = 0;
    qcount = qcount_backup;
    for (const Customer& cust : customers()) {
      /* Skip customers already assigned (but not yet picked up) */
      if (cust.assigned())
        continue;
      /* Skip customers under delay */
      if (delay(cust.id()))
        continue;

      if (T == 0) ncust++; // don't double count

      /* Get candidates from the local grid index
       * (the grid is refreshed during listen()) */
      DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
      auto candidates = grid_.within_about(rng, cust.orig());
      if (candidates.empty())
        continue;

      /* Assign to a random candidate */
      std::shuffle(candidates.begin(), candidates.end(), gen);
      auto& cand = candidates.front();
      if (qcount.at(cand->id()).first < qcount.at(cand->id()).second) {
        // DistInt cst = haversine(cand->last_visited_node(), cust.orig());
        std::vector<Wayp> unused_rte;
        std::vector<Stop> unused_sch;
        DistInt cst = sop_insert(*cand, cust, unused_sch, unused_rte);
        sol.push_back({cust, *cand, cst});
        this_solcst += cst;
        (qcount.at(cand->id()).first)++;
      }
    } // finished all customers
    best_sol = sol;
    best_solcst = this_solcst;
    /* Perturb the solution; pick a random assigned customer; remove it from
     * current assignment; try assign to another one of its candidates; accept
     * with probability. */
    if (sol.empty()) break;
    std::uniform_int_distribution<> n(0,sol.size()-1);
    for (int i = 0; i < PERT; ++i) {
      auto itr = sol.begin();
      std::advance(itr, n(gen));
      DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
      Customer& cust = std::get<0>(*itr);
      DistInt cst = 0;
      auto candidates = grid_.within_about(rng, cust.orig());
      if (candidates.empty())
        continue;
      std::shuffle(candidates.begin(), candidates.end(), gen);
      auto cand = candidates.front();
      if (qcount.at(cand->id()).first < qcount.at(cand->id()).second) {
          std::get<1>(*itr) = *cand;
        std::vector<Wayp> unused_rte;
        std::vector<Stop> unused_sch;
        cst = sop_insert(*cand, cust, unused_sch, unused_rte);
        // this_solcst += haversine(cand->last_visited_node(), cust.orig());
        this_solcst -= std::get<2>(*itr);
        this_solcst += cst;
      }
      if (!sol.empty() && hillclimb(T)) {
        (qcount.at(std::get<1>(*itr).id()).first)--;
        (qcount.at(cand->id()).first)++;
        best_sol = sol;
        best_solcst = this_solcst;
        nclimbs_++;
      } else if (!sol.empty() && this_solcst < best_solcst) {
        (qcount.at(std::get<1>(*itr).id()).first)--;
        (qcount.at(cand->id()).first)++;
        best_sol = sol;
        best_solcst = this_solcst;
      } else {
        this_solcst += std::get<2>(*itr);
        this_solcst -= cst;
      }
      if (timeout(start))
        break;
    } // end perturbation
  } // end temperature

  /* Commit the solution:
   * Add each customer to the vehicle one by one, in order of appearance. */
  std::unordered_map<VehlId, std::vector<Customer>> commits;
  for (const auto& assignment : best_sol) {
    commits[std::get<1>(assignment).id()] = {};
  }
  for (const auto& assignment : best_sol) {
    auto& cust = std::get<0>(assignment);
    auto& cand_id = std::get<1>(assignment).id();
    commits[cand_id].push_back(cust);
  }
  print << "starting to commit..." << std::endl;
  for (const auto& kv : commits) {
    auto& custs = kv.second;
    auto& cand_id = kv.first;
    auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
            [&](const Vehicle& a){ return a.id() == cand_id; }));
    std::vector<CustId> custs_to_add = {};
    print << "Commiting " << custs.size() << " custs..." << std::endl;
    /* The customers are inserted one at a time in no particular order...
     * here might be a reason for poor performance */
    for (const auto& cust : custs) {
      cargo::sop_insert(cand, cust, sch, rte);
      if (cargo::chktw(sch, rte)) { // skip if time window fails
        cand.set_sch(sch);
        cand.set_rte(rte);
        cand.reset_lvn();
        custs_to_add.push_back(cust.id());
      }
    }
    if (assign(custs_to_add, {}, rte, sch, cand)) {
      for (const auto& cust_id : custs_to_add) {
        print(MessageType::Success)
          << "Match " << "(cust" << cust_id << ", veh" << cand.id() << ")"
          << std::endl;
        nmat_++;  // increment matched counter
        is_matched.at(cust_id) = true;
        end_delay(cust_id);  // (rsalgorithm.h)
      }
    } else
      nrej_++;  // increment rejected counter
  }
  print << "Ncust: " << ncust << std::endl;
  print << "Done." << std::endl;
  for (const auto& kv : is_matched)
    if (!kv.second) beg_delay(kv.first);
  t1 = std::chrono::high_resolution_clock::now();
  // Stop timing --------------------------------
  if (ncust > 0)
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
  return d(gen) <= std::exp((-1)*T/1.0) ? true : false;
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

