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
#include <map>
#include <unordered_map>
#include <vector>

#include "libcargo.h"
#include "grasp.h"

using namespace cargo;

const int BATCH = 30;
const int RANGE = 2000;
const int PERT  = 1000;
const int T_MAX = 10;

GRASP::GRASP()
    : RSAlgorithm("grasp", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

void GRASP::match() {
  if (this->customers().size() == 0)
    return;
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : this->customers())
    this->is_matched[cust.id()] = false;

  this->best_solcst = 0;

  // PRE-Initialize: use candidates filtering to find
  // which customers certain vehicles are candidates for.
  Grid lcl_grid(this->grid_);  // local copy to store local changes
  this->candidates_list.clear();
  for (const Customer& cust : this->customers()) {
    vec_t<MutableVehicleSptr> candidates = lcl_grid.within(RANGE, cust.orig());
    for (const MutableVehicleSptr& mv : candidates) {
      if (candidates_list.count(mv) == 0)
        candidates_list[mv] = {};
      candidates_list.at(mv).push_back(cust);
    }
  }

  /* Generate an initial solution
   * ---------------------------- */
  dict<VehlId, vec_t<CustId>> solution;
  vec_t<CustId> assigned = {};
  // Loop through the vehicles; for each vehicle, list all the
  // new route costs of inserting each customer into the vehicle.
  for (auto& kv : candidates_list) {
    MutableVehicleSptr cand = kv.first;  // points to mv in lcl_grid
    const vec_t<Customer>& custs = kv.second;
    vec_t<rank_cust> rank;
    dict<CustId, vec_t<Stop>> new_sch;
    dict<CustId, vec_t<Wayp>> new_rte;
    for (const Customer& cust : custs) {
      if (std::find(assigned.begin(), assigned.end(), cust.id()) == assigned.end()) {
        vec_t<Stop> sch;
        vec_t<Wayp> rte;
        DistInt cost =
          sop_insert(*cand, cust, sch, rte) - cand->route().cost() + cand->next_node_distance();
        rank.push_back({cost, cust.id()});
        new_sch[cust.id()] = sch;
        new_rte[cust.id()] = rte;
      }
    }
    // Select a customer with probability proportional to cost
    // (see https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
    DistInt max = max_rank(rank);
    std::uniform_int_distribution<> n(0, rank.size() - 1);
    std::uniform_real_distribution<> m(0, 1);
    size_t it = 0;
    while (true) {
      it = n(this->gen);
      float threshold = m(this->gen);
      if ((float)rank.at(it).first/max < threshold)  // small ranks are more likely to pass
        break;
    }
    CustId cadd = rank.at(it).second;
    sch = new_sch.at(cadd);
    rte = new_rte.at(cadd);
    if (chkcap(cand->capacity(), sch)
     && chktw(sch, rte)) {
      cand->set_sch(sch);
      cand->set_rte(rte);
      cand->reset_lvn();
      cand->incr_queued();
      if (solution.count(cand->id()) == 0)
        solution[cand->id()] = {};
      solution.at(cand->id()).push_back(cadd);
      assigned.push_back(cadd);
    }
  }

  // AT THIS POINT, vehicles in lcl_grid are different from grid_ because
  // candidates have different routes/schedules now. */

  /* Improve
   * ------- */
  bool has_improvement = false;
  vec_t<Stop> best_sch;
  vec_t<Wayp> best_rte;
  // 1. Replace an assigned with an unassigned from some random vehicle.
  bool replace = false;
  std::uniform_int_distribution<> p(0, candidates_list.size() - 1);
  auto j = candidates_list.begin();
  std::advance(j, p(gen));
  MutableVehicleSptr mv = j->first;
  auto k = j->second.begin();
  for (; k != j->second.end(); ++k) {
    if (std::find(assigned.begin(), assigned.end(), k->id()) == assigned.end())
      break;
  }
  CustId remove_me = randcust(mv->schedule().data());
  if (remove_me != -1) {
    vec_t<Stop> old_sch = mv->schedule().data();
    vec_t<Stop> rep_sch;
    vec_t<Wayp> rep_rte;
    sop_replace(mv, remove_me, *k, rep_sch, rep_rte);
    if (chkcap(mv->capacity(), rep_sch)
     && chktw(rep_sch, rep_rte)
     && rep_rte.back().first < mv->route().cost()) {
      replace = true;
      best_sch = rep_sch;
      best_rte = rep_rte;
      has_improvement = true;
      // don't forget to change assigned map
    }
  }
  // 2. Swap assignments from two random vehicles.
  bool swap = false;


  /* Perturb the solution */
  std::uniform_int_distribution<> n(0, this->best_sol.size() - 1);
  for (int T = 0; T < T_MAX; ++T) {
    for (int i = 0; i < PERT; ++i) {
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
            } else if (climb) {
              accept = true;
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
        this->beg_delay(custs_to_add.at(i));
      }
    }
  }

  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  this->end_ht();
}

void GRASP::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void GRASP::end() {
  this->print_statistics();
}

void GRASP::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

bool GRASP::hillclimb(int& T) {
  return this->d(this->gen) <= std::exp((-1)*T/1.0) ? true : false;
}

DistInt GRASP::max_rank(const vec_t<rank_cust>& rank) {
  DistInt max = 0;
  for (const rank_cust& rc : rank)
    if (rc.first > max)
      max = rc.first;
  return max;
}

void GRASP::reset_workspace() {
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
  option.static_mode = true;
  Cargo cargo(option);
  GRASP grasp;
  cargo.start(grasp);

  return 0;
}

