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
#include "sa.h"

using namespace cargo;

const int BATCH = 30;
const int T_MAX = 5;
const int P_MAX = 15000;

SimulatedAnnealing::SimulatedAnnealing()
    : RSAlgorithm("sa", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ndrops_ = 0;
  std::random_device rd;
  this->gen.seed(rd());
}

void SimulatedAnnealing::match() {
  this->beg_batch_ht();
  this->reset_workspace();

  Grid local_grid(this->grid_);  // make a deep copy

  // auto init_0 = hiclock::now();
  this->initialize(local_grid);
  // auto init_1 = hiclock::now();
  // print << "init: " << std::round(dur_milli(init_1-init_0).count()) << std::endl;

  if (!this->sol.empty()) {
    std::uniform_int_distribution<>::param_type sol_size_range(1, this->sol.size());
    this->n.param(sol_size_range);

    // auto pert_0 = hiclock::now();
    this->anneal(T_MAX, P_MAX);
    // auto pert_1 = hiclock::now();
    // print << "pert: " << std::round(dur_milli(pert_1-pert_0).count()) << std::endl;

    this->end_batch_ht();
    this->commit();
  }
}

void SimulatedAnnealing::initialize(Grid& local_grid) {
  // Assign each customer to a random candidate
  for (const Customer& cust : this->customers()) {
    bool initial = false;
    vec_t<MutableVehicleSptr>& candidates
      = this->candidates_list[cust.id()]
      = local_grid.within(pickup_range(cust), cust.orig());

    // Add to local vehicle lookup (we need it during perturb)
    for (const MutableVehicleSptr cand : candidates)
      this->vehicle_lookup[cand->id()] = cand;

    // Randomize the candidates order
    std::shuffle(candidates.begin(), candidates.end(), this->gen);

    // Try until got a valid one
    while (!candidates.empty() && initial == false) {
      MutableVehicleSptr& cand = candidates.back();
      candidates.pop_back();
      // Speed-up heuristic!
      // Try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < 10) {
        sop_insert(*cand, cust, sch, rte);
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          cand->set_sch(sch);  // update grid version of the candidate
          cand->set_rte(rte);
          cand->reset_lvn();
          if (this->sol.count(cand->id()) == 0) {
            vec_t<Customer> assignments = {};
            this->sol[cand->id()] = std::make_pair(*cand, assignments);
          }
          this->sol.at(cand->id()).first = *cand;  // update our copy of the candidate
          this->sol.at(cand->id()).second.push_back(cust);
          initial = true;
        }
      }
    }
  }
}

Solution SimulatedAnnealing::perturb(const Solution& sol,
                                     const int& temperature) {
  auto i = sol.cbegin();                    // 1. Select random vehicle
  std::advance(i, this->n(this->gen)-1);
  if (i->second.second.empty()) return sol;
  MutableVehicle  k_old = i->second.first;
  vec_t<Customer> k_old_assignments = i->second.second;

  auto j = k_old_assignments.cbegin();      // 2. Select random assigned customer
  std::uniform_int_distribution<> m(1, k_old_assignments.size());
  std::advance(j, m(this->gen)-1);
  Customer cust_to_move = *j;

  vec_t<MutableVehicleSptr>& candidates = this->candidates_list.at(cust_to_move.id());
  if (candidates.empty()) return sol;
  auto k = candidates.cbegin();             // 3. Select new candidate
  std::uniform_int_distribution<> p(1, candidates.size());
  std::advance(k, p(this->gen)-1);

  while ((*k)->id() == k_old.id() && k != candidates.end()) k++;
  if (k == candidates.end()) return sol;

  MutableVehicle k_new = **k;
  vec_t<Customer> k_new_assignments = {};
  if (sol.find(k_new.id()) != sol.end())
    k_new_assignments = sol.at(k_new.id()).second;

  DistInt current_cost =                    // 4. Do the move
    k_old.route().cost() + k_new.route().cost();

  //   a. Remove cust from k_old
  this->sch_after_rem = k_old.schedule().data();
  opdel(this->sch_after_rem, cust_to_move.id());
  route_through(this->sch_after_rem, this->rte_after_rem);

  //   b. Add cust to k_new (bottleneck)
  sop_insert(k_new, cust_to_move, this->sch_after_add, this->rte_after_add);

  //   c. Accept or reject
  if (chkcap(k_new.capacity(), this->sch_after_add)
   && chktw(this->sch_after_add, this->rte_after_add)) {
    DistInt new_cost = this->rte_after_add.back().first + rte_after_rem.back().first;
    bool climb = false;
    if (new_cost >= current_cost) {
      climb = hillclimb(temperature);
      if (climb) this->nclimbs_++;
    }
    if (new_cost < current_cost || climb) {
      // Update grid
      vehicle_lookup.at(k_old.id())->set_sch(sch_after_rem);
      vehicle_lookup.at(k_old.id())->set_rte(rte_after_rem);
      vehicle_lookup.at(k_old.id())->reset_lvn();
      vehicle_lookup.at(k_new.id())->set_sch(sch_after_add);
      vehicle_lookup.at(k_new.id())->set_rte(rte_after_add);
      vehicle_lookup.at(k_new.id())->reset_lvn();

      // Update solution
      k_old.set_sch(sch_after_rem);
      k_old.set_rte(rte_after_rem);
      k_old.reset_lvn();
      k_old_assignments.erase(j);
      k_new.set_sch(sch_after_add);
      k_new.set_rte(rte_after_add);
      k_new.reset_lvn();
      k_new_assignments.push_back(cust_to_move);
      Solution improved_sol = sol;
      improved_sol[k_old.id()] = std::make_pair(k_old, k_old_assignments);
      improved_sol[k_new.id()] = std::make_pair(k_new, k_new_assignments);
      return improved_sol;
    }
  }
  return sol;
}

void SimulatedAnnealing::commit() {
  for (const auto& kv : this->sol) {
    MutableVehicle cand = kv.second.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second.second) cadd.push_back(cust.id());
    this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand);
  }
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
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void SimulatedAnnealing::reset_workspace() {
  this->sol = {};
  this->sch = this->sch_after_rem = this->sch_after_add = {};
  this->rte = this->rte_after_rem = this->rte_after_add = {};
  this->candidates_list = {};
  this->vehicle_lookup = {};
  this->timeout_0 = hiclock::now();
  this->timeout_ = this->timeout_/T_MAX;
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
  option.path_to_problem  = "../../data/benchmark/rs-m5k-c3.instance";
  option.path_to_solution = "sa.sol";
  option.path_to_dataout  = "sa.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode = false;
  option.static_mode = true;
  Cargo cargo(option);
  SimulatedAnnealing sa;
  cargo.start(sa);

  return 0;
}

