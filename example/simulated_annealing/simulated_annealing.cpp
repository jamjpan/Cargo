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

#include "libcargo.h"
#include "simulated_annealing.h"

using namespace cargo;

const int BATCH = 30;
const int RANGE = 2000;
const int PERT  = 6;
const int T_MAX = 5;

SimulatedAnnealing::SimulatedAnnealing()
    : RSAlgorithm("simulated_annealing", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ndrops_ = 0;
  std::random_device rd;
  this->gen.seed(rd());
}

void SimulatedAnnealing::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : customers())
    is_matched[cust.id()] = false;

  DistInt best_solcst = 0;

  /* Generate an initial solution */
  Grid lcl_grid(this->grid_); // make a local copy
  for (const Customer& cust : customers()) {
    bool initial = false;
    this->candidates = lcl_grid.within(RANGE, cust.orig());
    while (!this->candidates.empty() && initial == false) {
      MutableVehicleSptr& cand = this->candidates.back();
      candidates.pop_back();
      if (cand->queued() < cand->capacity()) {
        DistInt cst = sop_insert(*cand, cust, sch, rte);
        if (chktw(sch, rte)) {
          cand->set_sch(sch);
          cand->set_rte(rte);
          cand->reset_lvn();
          cand->incr_queued();
          std::tuple<Customer, MutableVehicle, DistInt>
            assignment(cust, *cand, cst);
          this->best_sol.push_back(assignment);
          best_solcst += cst;
          initial = true;
        }
      }
    }
    if (!initial)
      this->beg_delay(cust.id());
  }

  /* AT THIS POINT, vehicles in lcl_grid are different from grid_ because
   * candidates have different routes/schedules now. */

  if (this->best_sol.empty())
    return;

  /* Perturb the solution */
  std::uniform_int_distribution<> n(0, this->best_sol.size() - 1);
  for (int T = 0; T < T_MAX; ++T) {
    print << "T=" << T << std::endl;
    for (int i = 0; i < PERT; ++i) {
      print << "P=" << i << std::endl;
      auto sol = best_sol;
      auto cust_itr = sol.begin();          // pick random customer
      std::advance(cust_itr, n(this->gen));
      Customer& cust = std::get<0>(*cust_itr);

      auto candidates = lcl_grid.within(RANGE, cust.orig());
      if (!candidates.empty()) {
        std::uniform_int_distribution<> m(0, candidates.size() - 1);
        auto vehl_itr = candidates.begin(); // pick random candidate
        std::advance(vehl_itr, m(gen));
        auto cand = *vehl_itr;

        bool is_same = (cand->id() == std::get<1>(*cust_itr).id());
        if (!is_same && cand->queued() < cand->capacity()) {
          DistInt cst = sop_insert(*cand, cust, sch, rte);
          if (chktw(sch, rte)) {
            bool climb = hillclimb(T);
            bool accept = false;
            if (cst < std::get<2>(*cust_itr)) {
              accept = true;
              this->ndrops_++;
            } else if (climb) {
              accept = true;
              this->nclimbs_++;
            }
            if (accept) {
              cand->set_rte(rte);
              cand->set_sch(sch);
              cand->reset_lvn();
              cand->incr_queued();

              auto sptr = lcl_grid.select(std::get<1>(*cust_itr).id());
              if (sptr == nullptr) {
                print(MessageType::Error)
                  << "Could not find vehicle in local grid" << std::endl;
                throw;
              }
              auto less_sch = sptr->schedule().data();
              opdel(less_sch, cust.id());
              std::vector<Wayp> less_rte {};
              route_through(less_sch, less_rte);
              sptr->set_sch(less_sch);
              sptr->set_rte(less_rte);
              sptr->reset_lvn();
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
        }
      }
    } // end perturbation
    if (this->timeout(this->timeout_0)) {
      print << "Timed out" << std::endl;
      break;
    }
  } // end temperature

  /* Commit the solution */
  for (const auto& assignment : this->best_sol) {
    auto& vehl_id = std::get<1>(assignment).id();
    commit_cadd[vehl_id] = {};
    if (commit_rte.count(vehl_id) == 0)
      commit_rte[vehl_id] = std::get<1>(assignment).route().data();
    if (commit_sch.count(vehl_id) == 0)
      commit_sch[vehl_id] = std::get<1>(assignment).schedule().data();
  }
  for (const auto& assignment : this->best_sol) {
    auto& cust = std::get<0>(assignment);
    auto& cand_id = std::get<1>(assignment).id();
    commit_cadd[cand_id].push_back(cust);
  }
  for (const auto& kv : commit_cadd) {
    auto& custs = kv.second;
    auto& cand_id = kv.first;
    auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
            [&](const Vehicle& a){ return a.id() == cand_id; }));
    std::vector<CustId> custs_to_add = {};
    for (const auto& cust : custs)
      custs_to_add.push_back(cust.id());
    if (assign(custs_to_add, {}, commit_rte.at(cand_id), commit_sch.at(cand_id), cand)) {
      for (const auto& cust_id : custs_to_add) {
        is_matched.at(cust_id) = true;
        this->end_delay(cust_id);
      }
    } else {
      for (size_t i = 0; i < custs_to_add.size(); ++i) {
        this->nrej_++;
        this->beg_delay(custs_to_add.at(i));
      }
    }
  }

  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  this->end_ht();
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void SimulatedAnnealing::end() {
  print(MessageType::Info)
    << "climbs: " << nclimbs_ << '\n'
    << "descents: " << ndrops_ << std::endl;
  this->print_statistics();
}

void SimulatedAnnealing::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

bool SimulatedAnnealing::hillclimb(int& T) {
  return this->d(this->gen) <= std::exp((-1)*T/1.0) ? true : false;
}

void SimulatedAnnealing::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->timeout_0 = hiclock::now();
  this->timeout_ = BATCH*1000;
  this->is_matched = {};
  this->best_sol = {};
  this->commit_cadd = {};
  this->commit_rte = {};
  this->commit_sch = {};
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-md-18.instance";
  option.path_to_solution = "simulated_annealing.sol";
  option.path_to_dataout  = "simulated_annealing.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 20;
  option.matching_period  = 60;
  option.static_mode = false;
  Cargo cargo(option);
  SimulatedAnnealing sa;
  cargo.start(sa);

  return 0;
}

