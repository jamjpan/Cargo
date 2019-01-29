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
const int SCHED_MAX = 10;

SimulatedAnnealing::SimulatedAnnealing()
    : RSAlgorithm("sa50", true), grid_(100), d(0,1) {
  this->construct(50);
}

SimulatedAnnealing::SimulatedAnnealing(const int& f)
    : RSAlgorithm("sa"+std::to_string(f), true), grid_(100), d(0,1) {
  if (f < 1 && f > 100) {
    print(MessageType::Warning) << "f less than 1 or greater than 100; set to default (50)" << std::endl;
    this->construct(50);
  } else
    this->construct(f);
}

void SimulatedAnnealing::construct(const int& f) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->t_max = 5;
  this->p_max = 5000;
  this->f_ = static_cast<float>(f/100.0);
  print << "Set f to " << this->f_ << std::endl;
  std::random_device rd;
  this->gen.seed(rd());
}

void SimulatedAnnealing::match() {
  this->reset_workspace();

  Grid local_grid(this->grid_);  // make a deep copy

  this->initialize(local_grid);
  print << "initial cost: " << this->sol_cost(this->sol) << std::endl;

  if (!this->sol.empty()) {
    std::uniform_int_distribution<>::param_type sol_size_range(1, this->sol.size());
    this->n.param(sol_size_range);
    this->anneal(t_max, p_max);
    print << "after anneal: " << this->sol_cost(this->sol) << std::endl;
    this->commit();
  }
}

void SimulatedAnnealing::initialize(Grid& local_grid) {
  // Assign each customer to a random candidate
  for (const Customer& cust : this->customers()) {
    print << "initialize() working on cust " << cust.id() << std::endl;
    bool initial = false;
    //this->tried[cust.id()] = {};
    vec_t<MutableVehicleSptr> candidates
      = this->candidates_list[cust.id()]
      = local_grid.within(pickup_range(cust), cust.orig());

    // Add to local vehicle lookup (we need it during perturb)
    for (const MutableVehicleSptr cand : candidates)
      this->vehicle_lookup[cand->id()] = cand;

    // Randomize the candidates order
    //std::shuffle(candidates.begin(), candidates.end(), this->gen);

    print << "  got " << candidates.size() << " cands" << std::endl;
    // Try until got a valid one
    while (!candidates.empty() && initial == false) {
      auto k = candidates.begin();
      std::uniform_int_distribution<> m(0, candidates.size()-1);
      std::advance(k, m(this->gen));
      MutableVehicleSptr cand = *k;
      candidates.erase(k);
      //MutableVehicleSptr cand = candidates.back();
      //candidates.pop_back();
      // Speed-up heuristic!
      // Try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < SCHED_MAX) {
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
          print << "    initialized cust " << cust.id() << " to cand " << cand->id() << "; assignments=";
          for (const auto& c : this->sol.at(cand->id()).second) print << c.id() << ", ";
          print << std::endl;
          initial = true;
        }
      }
    }
  }
}

SASol SimulatedAnnealing::perturb(const SASol& sol,
                                     const int& temperature) {
  auto i = sol.cbegin();                    // 1. Select random vehicle
  std::advance(i, this->n(this->gen)-1);
  if (i->second.second.empty()) return sol;
  MutableVehicle  k_old = i->second.first;
  vec_t<Customer> k_old_assignments = i->second.second;
  print << "perturb got vehl " << k_old.id() << std::endl;

  auto j = k_old_assignments.cbegin();      // 2. Select random assigned customer
  std::uniform_int_distribution<> m(1, k_old_assignments.size());
  std::advance(j, m(this->gen)-1);
  Customer cust_to_move = *j;
  print << "perturb got cust " << cust_to_move.id() << std::endl;

  vec_t<MutableVehicleSptr> candidates = this->candidates_list.at(cust_to_move.id());
  if (candidates.empty()) {
    print << "  no candidates; returning" << std::endl;
    return sol;
  }

  auto k = candidates.cbegin();             // 3. Select new candidate
  std::uniform_int_distribution<> p(1, candidates.size());
  std::advance(k, p(this->gen)-1);
  print << " perturb got new cand " << (*k)->id() << std::endl;

  //while (((*k)->id() == k_old.id() || this->tried.at(cust_to_move.id()).count((*k)->id()) == 1)
  //    && !candidates.empty()) {
  while ((*k)->id() == k_old.id()) {
    candidates.erase(k);
    if (candidates.empty()) {
      print << "  no untried candidates; returning" << std::endl;
      return sol;
    }
    k = candidates.cbegin();
    std::uniform_int_distribution<> p(0, candidates.size()-1);
    std::advance(k, p(this->gen));
    print << " perturb got new cand " << (*k)->id() << std::endl;
  }
  //if (k == candidates.end()) return sol;

  //this->tried[cust_to_move.id()].insert((*k)->id());
  MutableVehicle k_new = **k;
  // We don't want to try this candidate again in later perturbs, so just remove
  // it from the candidates list
  //candidates.erase(k);

  vec_t<Customer> k_new_assignments = {};
  if (sol.find(k_new.id()) != sol.end())
    k_new_assignments = sol.at(k_new.id()).second;

  // 4. Do the move
  DistInt current_cost = k_old.route().cost() + k_new.route().cost();

  //   a. Add cust to k_new (--bottleneck--)
  sop_insert(k_new, cust_to_move, this->sch_after_add, this->rte_after_add);
  print << "  move " << cust_to_move.id() << " from " << k_old.id() << " to " << k_new.id() << std::endl;

  //   b. Accept or reject
  if (chkcap(k_new.capacity(), this->sch_after_add)
   && chktw(this->sch_after_add, this->rte_after_add)) {
    //   c. Remove cust from k_old
    this->sch_after_rem = k_old.schedule().data();
    opdel(this->sch_after_rem, cust_to_move.id());
    route_through(this->sch_after_rem, this->rte_after_rem);

    //   d. Compare costs
    DistInt new_cost = this->rte_after_add.back().first + rte_after_rem.back().first;
    bool climb = false;
    print << "    is " << new_cost << " < " << current_cost << "?" << std::endl;
    // Quality heuristic: don't do any climbs on the last temperature
    if (new_cost >= current_cost && temperature != 1) {
      climb = hillclimb(temperature);
      if (climb) this->nclimbs_++;
    }
    if (new_cost < current_cost || climb) {
      print << (new_cost < current_cost ? "  accept" : " accept due to climb") << std::endl;
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
      SASol improved_sol = sol;
      improved_sol[k_old.id()] = std::make_pair(k_old, k_old_assignments);
      improved_sol[k_new.id()] = std::make_pair(k_new, k_new_assignments);
      return improved_sol;
    }
  } else {
    print << "    not valid" << std::endl;
  }
  return sol;
}

void SimulatedAnnealing::commit() {
  for (const auto& kv : this->sol) {
    MutableVehicle cand = kv.second.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second.second) cadd.push_back(cust.id());
    if (this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand))
      for (const CustId& cid : cadd) print << "Matched " << cid << " with " << cand.id() << std::endl;
  }
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void SimulatedAnnealing::end() {
  print(MessageType::Info) << "climbs: " << nclimbs_ << std::endl;
  RSAlgorithm::end();
}

void SimulatedAnnealing::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void SimulatedAnnealing::reset_workspace() {
  this->sol = {};
  //this->tried = {};
  this->sch = this->sch_after_rem = this->sch_after_add = {};
  this->rte = this->rte_after_rem = this->rte_after_add = {};
  this->candidates_list = {};
  this->vehicle_lookup = {};
  this->timeout_0 = hiclock::now();
  this->best_sol = {};
  this->commit_cadd = {};
  this->commit_rte = {};
  this->commit_sch = {};
}

