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
const int PERT  = 10000; // how many solutions to generate per iter?
const int T_MAX = 10; // maximum "temperature"

std::vector<int> avg_dur {};

typedef std::chrono::duration<double, std::milli> dur_milli;
typedef std::chrono::milliseconds milli;

SimulatedAnnealing::SimulatedAnnealing() : RSAlgorithm("simulated_annealing"),
    grid_(100), d(0,1) {
  batch_time() = BATCH;
  nmat_ = 0;
  nclimbs_ = 0;
  ndrops_ = 0;
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
  std::vector<std::tuple<Customer, MutableVehicle, DistInt>> best_sol = {};
  DistInt best_solcst = 0;

  /* Generate an initial solution */
  Grid lcl_grid(this->grid_); // make a local copy
  for (const Customer& cust : customers()) {
    /* Skip customers already assigned (but not yet picked up) */
    if (cust.assigned())
      continue;
    /* Skip customers under delay */
    if (delay(cust.id()))
      continue;
    /* Increment the customer counter */
    ncust++;

    /* Get candidates from the local grid index */
    DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
    auto candidates = lcl_grid.within_about(rng, cust.orig());
    if (candidates.empty())
      continue;

    /* Try assign to a random candidate */
    auto& cand = candidates.front();

    bool within_cap = (cand->queued() < cand->capacity());
    if (!within_cap)
      continue;
    std::vector<Wayp> try_rte;
    std::vector<Stop> try_sch;
    DistInt cst = sop_insert(*cand, cust, try_sch, try_rte);
    bool within_time = chktw(try_sch, try_rte);
    if (within_time) {
      cand->set_rte(try_rte);
      cand->set_sch(try_sch);
      cand->reset_lvn();
      cand->incr_queued();
      std::tuple<Customer, MutableVehicle, DistInt> temp_tuple(cust, *cand, cst);
      best_sol.push_back(temp_tuple);  // cand has new rte/sch now
      // best_sol.push_back({cust, *cand, cst});  // cand has new rte/sch now
      best_solcst += cst;
    }
    // Vehicles in lcl_grid are now different from this->grid_ (ground-truth)
  } // finished all customers
  std::cout << "best_solcst: " << best_solcst << std::endl;

  /* Perturb the solution; pick a random assigned customer; try assign to
   * another one of its candidates; accept with probability. */
  if (best_sol.empty()) return;
  for (int T = 0; T < T_MAX; ++T) {
    std::uniform_int_distribution<> n(0,best_sol.size()-1);
    for (int i = 0; i < PERT; ++i) {

      /* Pick a random customer to perturb */
      auto sol = best_sol;
      auto cust_itr = sol.begin();
      std::advance(cust_itr, n(gen));
      DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
      Customer& cust = std::get<0>(*cust_itr);

      /* Pick a random candidate to assign to */
      auto candidates = lcl_grid.within_about(rng, cust.orig());
      if (candidates.empty())
        continue;
      std::uniform_int_distribution<> m(0,candidates.size()-1);
      auto vehl_itr = candidates.begin();
      std::advance(vehl_itr, m(gen));
      auto cand = *vehl_itr;

      /* Try find the cost */
      bool is_same = (cand->id() == std::get<1>(*cust_itr).id());
      if (is_same)
        continue;
      bool within_cap = (cand->queued() < cand->capacity());
      if (!within_cap)
        continue;
      std::vector<Wayp> alt_rte;
      std::vector<Stop> alt_sch;
      DistInt cst = sop_insert(*cand, cust, alt_sch, alt_rte);
      bool within_time = chktw(alt_sch, alt_rte);
      if (within_time) {
        /* If cost is less than solution's cost OR hillclimb, accept it */
        bool climb = hillclimb(T);
        if (cst < std::get<2>(*cust_itr) || climb) {
          if (climb) {
            nclimbs_++;
            ndrops_++;
          }
          else
            ndrops_++;
          /* Modify the candidate in the grid */
          cand->set_rte(alt_rte);
          cand->set_sch(alt_sch);
          cand->reset_lvn();
          cand->incr_queued();

          /* Modify the current vehicle in the grid */
          auto sptr = lcl_grid.select(std::get<1>(*cust_itr).id());
          if (sptr == nullptr) {
            print(MessageType::Error) << "Could not find vehicle in local grid" << std::endl;
            throw;
          }
          auto less_sch = sptr->schedule().data();
          opdel(less_sch, cust.id());
          std::vector<Wayp> less_rte {};
          route_through(less_sch, less_rte);
          sptr->set_sch(less_sch);
          sptr->set_rte(less_rte);
          sptr->decr_queued();

          /* Store the new solution */
          std::get<1>(*cust_itr) = *cand; // modify sol with new cand
          std::get<2>(*cust_itr) = cst;   // modify sol with new cost
          DistInt solcst = cst;
          for (auto &assignment : sol) {
            if (std::get<1>(assignment).id() == sptr->id())
              std::get<1>(assignment) = *sptr;  // modify old assignment
            else
              solcst += std::get<2>(assignment);
            if (std::get<1>(assignment).id() == cand->id())
              std::get<1>(assignment) = *cand;  // modify old assignment
          }
          best_sol = sol;
          best_solcst = solcst;
        }
      }
      if (timeout(start))
        break;
    } // end perturbation
    // print << "Best temperature cost: " << best_solcst << std::endl;
  } // end temperature
  // print << "Final cost: " << best_solcst << std::endl;
  // print << nclimbs_ << "/" << ndrops_ << std::endl;

  /* Commit the solution:
   * Add each customer to the vehicle one by one, in order of appearance. */
  std::unordered_map<VehlId, std::vector<Customer>> commit_cadd;
  std::unordered_map<VehlId, std::vector<Wayp>> commit_rte;
  std::unordered_map<VehlId, std::vector<Stop>> commit_sch;
  for (const auto& assignment : best_sol) {
    auto &vehl_id = std::get<1>(assignment).id();
    commit_cadd[vehl_id] = {};
    if (commit_rte.count(vehl_id) == 0)
      commit_rte[vehl_id] = std::get<1>(assignment).route().data();
    if (commit_sch.count(vehl_id) == 0)
      commit_sch[vehl_id] = std::get<1>(assignment).schedule().data();
  }
  for (const auto& assignment : best_sol) {
    auto& cust = std::get<0>(assignment);
    auto& cand_id = std::get<1>(assignment).id();
    commit_cadd[cand_id].push_back(cust);
    // std::get<1>(assignment).print();
  }
  print << "starting to commit..." << std::endl;
  for (const auto& kv : commit_cadd) {
    auto& custs = kv.second;
    auto& cand_id = kv.first;
    auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
            [&](const Vehicle& a){ return a.id() == cand_id; }));
    std::vector<CustId> custs_to_add = {};
    print << "Commiting " << custs.size() << " custs..." << std::endl;
    for (const auto& cust : custs)
      custs_to_add.push_back(cust.id());
    if (assign(custs_to_add, {}, commit_rte.at(cand_id), commit_sch.at(cand_id), cand)) {
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
  double temp_time = std::round(dur_milli(t1-t0).count());
  print << "Time Cost: " << temp_time << std::endl;
  if (ncust > 0)
    avg_dur.push_back(temp_time / ncust);
  /*
  if (ncust > 0)
    avg_dur.push_back(std::round(dur_milli(t1-t0).count())/float(ncust));
  */
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
  op.path_to_roadnet  = "../../data/roadnetwork/mny.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/mny.gtree";
  op.path_to_edges    = "../../data/roadnetwork/mny.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-9.instance";
  op.path_to_solution = "simulated_annealing.sol";
  op.path_to_dataout  = "simulated_annealing.dat";
  op.path_to_save = "simulated_annealing.db";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  Cargo cargo(op);
  Cargo::OFFLINE = true;
  SimulatedAnnealing sa;
  cargo.start(sa);
}

