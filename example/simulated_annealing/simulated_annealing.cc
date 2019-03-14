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

SimulatedAnnealing::SimulatedAnnealing(const std::string& name,
    const int& f, const int& tmax, const int& pmax)
    : RSAlgorithm(name, false), grid_(100), d(0,1) {
  if (f < 1 && f > 100)
    print(MessageType::Warning) << "f less than 1 or greater than 100; set to default (50)" << std::endl;
  if (tmax < 0)
    print(MessageType::Warning) << "tmax less than 0; set to default (5)" << std::endl;
  if (pmax < 0)
    print(MessageType::Warning) << "pmax less than 0; set to default (5000)" << std::endl;
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ntries_ = 0;
  this->nanneals_ = 0;
  this->ntimeouts_ = 0;
  this->ncap_ = 0;
  this->ntw_ = 0;
  this->t_max = tmax;
  this->p_max = pmax;
  this->f_ = static_cast<float>(f/100.0);
  print << "Set f to " << this->f_ << std::endl;
  print << "Set tmax to " << this->t_max << std::endl;
  print << "Set pmax to " << this->p_max << std::endl;
  std::random_device rd;
  this->gen.seed(rd());
}

void SimulatedAnnealing::match() {
  this->reset_workspace();

  Grid local_grid(this->grid_);  // make a deep copy

  // #########################
  auto t_init_0 = hiclock::now();
  // #########################

  this->initialize(local_grid);

  // #########################
  auto t_init_1 = hiclock::now();
  // print_sol(this->sol);
  // print(MessageType::Info) << "initial cost: " << this->sol_cost(this->sol) << std::endl;
  print(MessageType::Info) << "initial time: " << std::round(dur_milli(t_init_1-t_init_0).count()) << std::endl;
  // #########################

  if (!this->sol.empty()) {

    // #########################
    auto t_anneal_0 = hiclock::now();
    // #########################

    this->anneal(t_max, p_max);

    // #########################
    auto t_anneal_1 = hiclock::now();
    // print(MessageType::Info) << "after anneal: " << this->sol_cost(this->sol) << std::endl;
    print(MessageType::Info) << "anneal time: " << std::round(dur_milli(t_anneal_1-t_anneal_0).count()) << std::endl;
    // print_sol(this->sol);
    // for (const auto& kv : this->sol) {
    //   print << "(t=" << Cargo::now() << ") " << kv.first << " rte| ";
    //   print << kv.second.first.route();
    //   print << std::endl;
    //   print << "(t=" << Cargo::now() << ") " << kv.first << " sch| ";
    //   print << kv.second.first.schedule();
    //   print << std::endl;
    // }
    // #########################
    this->commit();
  } else {
    print(MessageType::Warning) << "empty initial solution" << std::endl;
  }
}

void SimulatedAnnealing::initialize(Grid& local_grid) {
  // Assign each customer to a random candidate
  for (const Customer& cust : this->customers()) {
    // print << "initialize() working on cust " << cust.id() << std::endl;
    bool initial = false;
    //this->tried[cust.id()] = {};
    vec_t<MutableVehicleSptr> candidates
      = this->candidates_list[cust.id()]
      = local_grid.within(pickup_range(cust), cust.orig());

    // Add to local vehicle lookup (we need it during perturb)
    for (const MutableVehicleSptr cand : candidates)
      this->vehicle_lookup[cand->id()] = cand;

    // print << "  got " << candidates.size() << " cands" << std::endl;

    // Randomize the candidates order
    //std::shuffle(candidates.begin(), candidates.end(), this->gen);

    // Try until got a valid one
    while (!candidates.empty() && initial == false) {
      auto k = candidates.begin();
      std::uniform_int_distribution<> m(0, candidates.size()-1);
      std::advance(k, m(this->gen));
      MutableVehicleSptr cand = *k;
      candidates.erase(k);
      //MutableVehicleSptr cand = candidates.back();
      //candidates.pop_back();

      // print << "    trying vehl " << cand->id() << std::endl;

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
          // print << "    initialized cust " << cust.id() << " to cand " << cand->id() << std::endl;

          // #########
          // for (const auto& kv : this->sol) {
          //   print << "(t=" << Cargo::now() << ") " << kv.first << " rte| ";
          //   print << kv.second.first.route();
          //   print << std::endl;
          //   print << "(t=" << Cargo::now() << ") " << kv.first << " sch| ";
          //   print << kv.second.first.schedule();
          //   print << std::endl;
          // }
          // for (const auto& c : this->sol.at(cand->id()).second) print << c.id() << ", ";
          // print << std::endl;
          // print << "    sched: " << cand->schedule() << std::endl;
          // #########
          initial = true;
        } else {
          // print << "      skipping due to infeasible" << std::endl;
        }
      } else {
        // print << "      skipping due to sched_max" << std::endl;
      }
    }
  }
}

SASol SimulatedAnnealing::perturb(const SASol& sol,
                                     const int& temperature) {
  // Due to structure of SASol, when we "randomly select a customer" we
  // actually randomly select a vehicle, then choose a customer assigned to
  // that vehicle. We then need to make sure our solution never has "empty"
  // vehicles with no assignments, otherwise some perturbs will be wasted.

  std::uniform_int_distribution<>::param_type sol_size_range(1, sol.size());
  this->n.param(sol_size_range);
  auto i = sol.cbegin();                    // 1. Select random vehicle
  std::advance(i, this->n(this->gen)-1);

  // if (i->second.second.empty()) {  // this should never happen
  //   print << "perturb got vehicle " << i->second.first.id() << " with no assignments; returning" << std::endl;
  //   print_sol(sol);
  //   pause();
  //   return sol;
  // }

  MutableVehicle  k_old = i->second.first;
  vec_t<Customer> k_old_assignments = i->second.second;
  // print << "perturb got vehl " << k_old.id() << std::endl;

  auto j = k_old_assignments.cbegin();      // 2. Select random assigned customer
  std::uniform_int_distribution<> m(1, k_old_assignments.size());
  std::advance(j, m(this->gen)-1);
  Customer cust_to_move = *j;
  // print << "  perturb got cust " << cust_to_move.id() << std::endl;

  vec_t<MutableVehicleSptr> candidates = this->candidates_list.at(cust_to_move.id());
  if (candidates.empty()) {
    // print << "  no candidates; returning" << std::endl;
    return sol;
  }

  auto k = candidates.cbegin();             // 3. Select new candidate
  std::uniform_int_distribution<> p(1, candidates.size());
  std::advance(k, p(this->gen)-1);
  // print << "  perturb got new cand " << (*k)->id() << std::endl;

  //while (((*k)->id() == k_old.id() || this->tried.at(cust_to_move.id()).count((*k)->id()) == 1)
  //    && !candidates.empty()) {
  while ((*k)->id() == k_old.id()) {
    candidates.erase(k);
    if (candidates.empty()) {
      // print << "  no untried candidates; returning" << std::endl;
      return sol;
    }
    k = candidates.cbegin();
    std::uniform_int_distribution<> p(0, candidates.size()-1);
    std::advance(k, p(this->gen));
    // print << " perturb got new cand " << (*k)->id() << std::endl;
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
  // print << "  move " << cust_to_move.id() << " from " << k_old.id() << " to " << k_new.id() << std::endl;

  // #############
  // print << "BEFORE MOVE" << std::endl;
  // print << k_old.id() << " route| ";
  // print << k_old.route();
  // print << std::endl;
  // print << k_new.id() << " route| ";
  // print << k_new.route();
  // print << std::endl;
  // print << "AFTER MOVE: ";
  // print << rte_after_add;
  // print << std::endl;
  // #############

  //   b. Accept or reject
  if (chkcap(k_new.capacity(), this->sch_after_add)) {
    this->ncap_++;
    if (chktw(this->sch_after_add, this->rte_after_add)) {
      this->ntw_++;
      //   c. Remove cust from k_old
      this->sch_after_rem = k_old.schedule().data();
      opdel(this->sch_after_rem, cust_to_move.id());
      route_through(this->sch_after_rem, this->rte_after_rem);

      // Add the traveled distance to rte_after_rem <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
      for (Wayp& wp : this->rte_after_rem)
        wp.first += k_old.route().dist_at(k_old.idx_last_visited_node());
      // ##########
      // print << "AFTER REMOVE: ";
      // print << rte_after_rem;
      // print << std::endl;
      // ##########

      //   d. Compare costs
      DistInt new_cost = this->rte_after_add.back().first + rte_after_rem.back().first;
      bool climb = false;
      // print << "    is " << new_cost << " < " << current_cost << "?" << std::endl;
      this->ntries_++;
      // Quality heuristic: don't do any climbs on the last temperature
      if (new_cost >= current_cost && temperature != 1) {
        climb = hillclimb(temperature);
        if (climb) this->nclimbs_++;
      }
      if (new_cost < current_cost || climb) {
        // print << (new_cost < current_cost ? "  accept" : " accept due to climb") << std::endl;
        // print << "  sched for " << k_old.id() << ": " << sch_after_rem << std::endl;
        // print << "  sched for " << k_new.id() << ": " << sch_after_add << std::endl;
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
        improved_sol[k_new.id()] = std::make_pair(k_new, k_new_assignments);
        if (k_old_assignments.empty())
          improved_sol.erase(k_old.id());  // erase by key
        else
          improved_sol[k_old.id()] = std::make_pair(k_old, k_old_assignments);
        return improved_sol;
      } else {
        // print << "    reject (greater cost)" << std::endl;
      }
    } else {
      // print << "    reject (failed chktw)" << std::endl;
    }
  } else {
    // print << "    reject (failed chkcap)" << std::endl;
  }
  return sol;
}

void SimulatedAnnealing::commit() {
  for (const auto& kv : this->sol) {
    MutableVehicle cand = kv.second.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second.second) cadd.push_back(cust.id());
    if (this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand)) {
      // for (const CustId& cid : cadd)
      //   print << "Matched " << cid << " with " << cand.id() << std::endl;
    } // else {
      // for (const CustId& cid : cadd)
      //   print(MessageType::Warning) << "Rejected due to sync " << cid << " with " << cand.id() << std::endl;
    // }
  }
}

void SimulatedAnnealing::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void SimulatedAnnealing::end() {
  print(MessageType::Info) << "climbs/tries: " << nclimbs_ << "/" << ntries_ << std::endl;
  print(MessageType::Info) << "anneals: " << nanneals_ << std::endl;
  print(MessageType::Info) << "timeouts: " << ntimeouts_ << std::endl;
  print(MessageType::Info) << "ncap: " << ncap_ << std::endl;
  print(MessageType::Info) << "ntw: " << ntw_ << std::endl;
  RSAlgorithm::end();
}

void SimulatedAnnealing::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void SimulatedAnnealing::print_sol(const SASol& sol) {
  for (const auto& kv : sol) {
    print << "Vehl " << kv.first << " | ";
    for (const auto& cust : kv.second.second) {
      print << cust.id() << " ";
    }
    print << std::endl;
  }
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

