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
#include<algorithm>
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

#include "grasp.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;
const int RANGE = 2000;
const int MAX_RESTART = 10;
const int MAX_ITER = 1000;

Grasp::Grasp()
    : RSAlgorithm("grasp", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

void Grasp::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : customers())
    is_matched[cust.id()] = false;

  DistInt best_solcst = InfInt;

  /* Difference between Grasp (GP) and Simulated Annealing (SA) is that GP does
   * multi-starts at each "temperature" whereas SA refines a single solution. */
  for (int i = 0; i < MAX_RESTART; ++i) {
    /* Generate an initial random solution
     * (same method as in SA) */
    std::random_shuffle(customers().begin(), customers().end());
    std::vector<std::tuple<Customer, MutableVehicle, DistInt>> sol = {};
    DistInt solcst = 0;
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
            sol.push_back(assignment);
            solcst += cst;
            initial = true;
          }
        }
      }
    }

    /* AT THIS POINT, vehicles in lcl_grid are different from grid_ because
     * candidates have different routes/schedules now. */

    if (sol.empty())
      return;

    /* Local search (same as SA) */
    std::uniform_int_distribution<> n(0, sol.size() - 1);
    for (int i = 0; i < MAX_ITER; ++i) {
      auto lcl_sol = sol;
      auto cust_itr = lcl_sol.begin();
      std::advance(cust_itr, n(this->gen));
      Customer& cust = std::get<0>(*cust_itr);

      auto candidates = lcl_grid.within(RANGE, cust.orig());
      if (!candidates.empty()) {
        std::uniform_int_distribution<> m(0, candidates.size() - 1);
        auto vehl_itr = candidates.begin();
        std::advance(vehl_itr, m(gen));
        auto cand = *vehl_itr;

        bool is_same = (cand->id() == std::get<1>(*cust_itr).id());
        if (!is_same && cand->queued() < cand->capacity()) {
          DistInt cst = sop_insert(*cand, cust, sch, rte);
          if (chktw(sch, rte) && cst < std::get<2>(*cust_itr)) {
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
            std::get<1>(*cust_itr) = *cand; // modify lcl_sol with new cand
            std::get<2>(*cust_itr) = cst;   // modify lcl_sol with new cost
            DistInt lcl_solcst = cst;
            for (auto &assignment : lcl_sol) {
              if (std::get<1>(assignment).id() == sptr->id())
                std::get<1>(assignment) = *sptr;  // modify old assignment
              else
                lcl_solcst += std::get<2>(assignment);
              if (std::get<1>(assignment).id() == cand->id())
                std::get<1>(assignment) = *cand;  // modify old assignment
            }
            sol = lcl_sol;
            solcst = lcl_solcst;
            // sol should be modified now!!!???
          }
        }
      }
    } // end perturbation
    if (this->timeout(this->timeout_0)) {
      break;
    }

    /* AT THIS POINT, sol has finished local search. Accept it as best_sol
     * if it beats the others. */

    if (solcst < best_solcst) {
      best_sol = sol;
      best_solcst = solcst;
    }
  } // end restarts

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
        this->beg_delay(custs_to_add.at(i));
      }
    }
  }

  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  this->end_ht();
}

void Grasp::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void Grasp::end() {
  this->print_statistics();
}

void Grasp::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

void Grasp::reset_workspace() {
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
  option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_solution = "grasp.sol";
  option.path_to_dataout  = "grasp.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 20;
  option.matching_period  = 60;
  option.static_mode = false;
  Cargo cargo(option);
  Grasp gp;
  cargo.start(gp);

  return 0;
}

