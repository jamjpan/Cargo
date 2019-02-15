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
#include <algorithm>
#include <iostream>
#include <random>
#include <unordered_set>
#include <utility>
#include <vector>

#include "libcargo.h"
#include "grasp.h"

using namespace cargo;

const int BATCH = 30;
const int TOP_K = 10;
const int SCHED_MAX = 10;

GRASP::GRASP(const std::string& name)
    : RSAlgorithm(name, false), grid_(100), d(0,1) {
  this->construct(4);
}

GRASP::GRASP(const std::string& name, const int& i)
    : RSAlgorithm(name, false), grid_(100), d(0,1) {
  if (i < 1) {
    print(MessageType::Warning) << "max_iter less than 1; set to default (4)" << std::endl;
    this->construct(4);
  } else
    this->construct(i);
}

void GRASP::construct(const int& i) {
  this->batch_time() = BATCH;
  this->max_iter = i;
  print << "Set max_iter to " << i << std::endl;
  std::random_device rd;
  this->gen.seed(rd());
  this->nswap_ = this->nreplace_ = this->nrearrange_ = this->nnoimprov_ = 0;
}

void GRASP::match() {
  this->timeout_0 = hiclock::now();

  this->best_solution = {};
  this->best_cost = InfInt;

  for (int count = 0; count < max_iter; ++count) {
    this->reset_workspace();
    Grid local_grid(this->grid_);  // deep copy
    vec_t<Customer> local_customers = this->customers();

    // bottleneck
    Solution initial_solution = this->initialize(local_grid, local_customers);
    //print << "Got initial solution:" << std::endl;
    //print_sol(initial_solution);

    DistInt local_cost = this->cost(initial_solution);
    bool has_improvement = false;
    Solution local_best = initial_solution;
    //print << "Initialized (cost=" << local_cost << ")" << std::endl;
    do {
      has_improvement = false;
      bool has_replace = false;
      vec_t<Customer> unassigned = local_customers;
      Solution sol_replace      = this->replace(local_best, unassigned);
      //print << "Got replace:" << std::endl;
      //print_sol(sol_replace);
      Solution sol_swap         = this->swap(local_best);
      //print << "Got swap:" << std::endl;
      //print_sol(sol_swap);
      Solution sol_rearrange    = this->rearrange(local_best);
      //print << "Got rearrange:" << std::endl;
      //print_sol(sol_rearrange);
      DistInt replace_cost      = this->cost(sol_replace);
      DistInt swap_cost         = this->cost(sol_swap);
      DistInt rearrange_cost    = this->cost(sol_rearrange);
      if (replace_cost < local_cost) {
        //print << "\tAccepted replace (" << replace_cost << ")" << std::endl;
        local_best = sol_replace;
        local_cost = replace_cost;
        has_improvement = true;
        has_replace = true;
        this->nreplace_++;
      }
      if (swap_cost < local_cost) {
        //print << "\tAccepted swap (" << swap_cost << ")" << std::endl;
        local_best = sol_swap;
        local_cost = swap_cost;
        has_improvement = true;
        has_replace = false;
        this->nswap_++;
      }
      if (rearrange_cost < local_cost) {
        //print << "\tAccepted rearrange (" << rearrange_cost << ")" << std::endl;
        local_best = sol_rearrange;
        local_cost = rearrange_cost;
        has_improvement = true;
        has_replace = false;
        this->nrearrange_++;
      }
      //print << "  has improvement? " << has_improvement << std::endl;
      if (has_replace == true)
        local_customers = unassigned;
      //print << "  local best: " << std::endl;
      //print_sol(local_best);

      // Update candidates_index
      if (has_improvement) {
        dict<VehlId, MutableVehicleSptr> temp = {};
        for (const auto& kv : local_best) {
          MutableVehicleSptr copy = std::make_shared<MutableVehicle>(*kv.first);
          temp[copy->id()] = copy;
        }
        for (auto& kv : this->candidates_index) {
          for (auto& cand : kv.second) {
            if (temp.count(cand->id()) == 1) {
              *cand = *(temp.at(cand->id()));
            }
          }
        }
      }

    } while (has_improvement);

    // print << "Local best (" << local_cost << ")" << std::endl;
    // print_sol(local_best);
    if (local_cost <= this->best_cost) {
      // print << "\tnew best solution (" << local_cost << " < " << this->best_cost << ")" << std::endl;
      this->best_solution = local_best;
      this->best_cost = local_cost;
    } //else
      //print << "kept incumbent (" << local_cost << " not better than " << this->best_cost << ")" << std::endl;
  }
  this->commit(this->best_solution);
}

Solution GRASP::initialize(Grid& grid, vec_t<Customer>& customers) {
  Solution initial_solution = {};
  // print << "initialize solution" << std::endl;

  // 1. List all the candidates for each customer (0-500 ms)
  int cands_per_cust = 0;
  for (const Customer& cust : customers) {
    this->candidates_index[cust] = {};
    vec_t<MutableVehicleSptr> candidates = grid.within(pickup_range(cust), cust.orig());
    cands_per_cust += candidates.size();
    for (const MutableVehicleSptr& cand : candidates) {
      //vehicles.insert(cand);
      this->candidates_index.at(cust).insert(cand);
      if (this->candidates_list.count(cand) == 0)
        this->candidates_list[cand] = {};
      this->candidates_list[cand].push_back(cust);
    }
  }
  //print << "  cands/cust: " << (float)cands_per_cust/customers.size() << std::endl;

  // 1-2. Speedup heuristic: only keep top x customers per candidate based on
  // nearest distance
  for (auto& kv : this->candidates_list) {
      std::sort(kv.second.begin(), kv.second.end(),
        [&](const Customer& a, const Customer& b) {
          return haversine(Cargo::node2pt(a.orig()), Cargo::node2pt(kv.first->last_visited_node()))
               < haversine(Cargo::node2pt(b.orig()), Cargo::node2pt(kv.first->last_visited_node()));
      });
      if (kv.second.size() > TOP_K) kv.second.resize(TOP_K);
  }

  // 2. Greedily Randomly Adaptively assign!
  //print << "  " << this->candidates_list.size() << " vehicles" << std::endl;
  //print << "  " << customers.size() << " customers" << std::endl;

  vec_t<MutableVehicleSptr> keys = {};
  for (const auto& kv : this->candidates_list)
    keys.push_back(kv.first);

  std::shuffle(keys.begin(), keys.end(), this->gen);

  int count = 0;
  int sch_size = 0;
  int cand_size = 0;
  for (const auto& kv : this->candidates_list)
    cand_size += kv.second.size();
  // for (auto i = this->candidates_list.begin(); i != this->candidates_list.end(); ++i) {
    // MutableVehicleSptr cand = i->first;
  for (MutableVehicleSptr cand : keys) {
    // Speed-up heuristic!
    // Try only if vehicle's current schedule len < SCHED_MAX customer stops
    if (cand->schedule().data().size() >= SCHED_MAX) continue;
    bool cand_done = false;
    bool initial_fitness = true;
    //for (const Customer& cust : this->candidates_list.at(cand))
    //  print << cust.id() << " ";
    //print << std::endl;
    while (!cand_done && !(this->candidates_list.at(cand).empty())
        && cand->schedule().size() < SCHED_MAX) {
      // bottleneck -- recompute fitness for every customer!
      // Instead, just compute fitness ONCE
      if (initial_fitness) {
        for (const Customer& cust : this->candidates_list.at(cand)) {
          if (this->timeout(this->timeout_0))
            return initial_solution;
          this->fitness[cand][cust] = sop_insert(cand, cust, sch, rte);
          this->schedules[cand][cust] = std::move(sch);
          this->routes[cand][cust] = std::move(rte);
        }
        initial_fitness = false;
      }
      Customer cust_to_add = this->roulette(this->fitness.at(cand));
      // print << "\tRolling roulette wheel" << std::endl;
      // for (const auto& kv : this->fitness.at(cand))
      //   print << "\t  cust " << kv.first.id() << ": " << kv.second << std::endl;
      // print << "\troulette returned " << cust_to_add.id() << std::endl;
      if (initial_fitness) {
        sch = std::move(this->schedules.at(cand).at(cust_to_add));
        rte = std::move(this->routes.at(cand).at(cust_to_add));
      } else
        sop_insert(cand, cust_to_add, sch, rte);
      if (chktw(sch, rte) && chkcap(cand->capacity(), sch)) {
        // print << "added " << cust_to_add.id() << " to " << cand->id() << std::endl;
        count++;
        sch_size += sch.size();
        cand->set_sch(sch);
        cand->set_rte(rte);
        cand->reset_lvn();
        if (initial_solution.count(cand) == 0) {
          initial_solution[cand].first = {cust_to_add.id()};
          initial_solution[cand].second = {};
        }
        else
          (initial_solution[cand].first).push_back(cust_to_add.id());
        //print << "added " << cust_to_add.id() << " to vehl " << cand->id() << std::endl;
        //print << "sched: ";
        //print_sch(sch);

        auto in_customers = std::find_if(customers.begin(), customers.end(),
          [&](const Customer& a) { return a.id() == cust_to_add.id(); });
        customers.erase(in_customers);
        if (customers.empty()) {
          //print << "  count: " << count << std::endl;
          //print << "  size: " << (float)sch_size/count << std::endl;
          //print << "  cand size: " << (float)cand_size/vehicles.size() << std::endl;
          //print << "  cand size: " << (float)cand_size/this->candidates_list.size() << std::endl;
          return initial_solution;
        }

        for (auto& kv : this->candidates_list) {
          auto in_list = std::find_if(kv.second.begin(), kv.second.end(),
            [&](const Customer& a) { return a.id() == cust_to_add.id(); });
          if (in_list != kv.second.end()) kv.second.erase(in_list);
        }

        { auto in_fitness = std::find_if( this->fitness.at(cand).begin(), this->fitness.at(cand).end(),
            [&](const std::pair<Customer, DistInt>& kv) {
              return kv.first.id() == cust_to_add.id(); });
          if (in_fitness != this->fitness.at(cand).end()) this->fitness.at(cand).erase(in_fitness);
        }
      } else {
        // print << " failed " << cust_to_add.id() << " with " << cand->id() << std::endl;
        cand_done = true;
      }
    }
    if (this->timeout(this->timeout_0))
      return initial_solution;
  }
  //print << "  count: " << count << std::endl;
  //print << "  size: " << (float)sch_size/count << std::endl;
  //print << "  cand size: " << (float)cand_size/vehicles.size() << std::endl;
  //print << "  cand size: " << (float)cand_size/this->candidates_list.size() << std::endl;
  return initial_solution;
}

Solution GRASP::replace(const Solution& solution, vec_t<Customer>& customers) {
  if (customers.empty() || solution.empty()) return solution;

  Solution sol_replace = {};

  // Deep copy into new solution
  for (const auto& kv : solution) {
    MutableVehicleSptr copy = std::make_shared<MutableVehicle>(*(kv.first));
    sol_replace[copy] = kv.second;
  }


  //std::shuffle(customers.begin(), customers.end(), this->gen);
  auto cust = customers.begin();  // select a replacement
  std::uniform_int_distribution<> n(0, customers.size() - 1);
  std::advance(cust, n(this->gen));
  Customer replacement = *cust;

  // for (auto cust = customers.begin(); cust != customers.end(); ++cust) {
    // Customer replacement = *cust;
    std::unordered_set<MutableVehicleSptr>& candidates = candidates_index.at(replacement);
    if (candidates.size() == 0) {
      return solution;
      // continue;
    }

    // Copy a candidate
    auto j = candidates.begin();
    std::uniform_int_distribution<> m(0, candidates.size() - 1);
    std::advance(j, m(this->gen));
    MutableVehicle cand = **j;

    // Select a customer to replace
    CustId to_replace = randcust(cand.schedule().data());
    if (to_replace == -1) {
      return solution;
      // continue;
    }

    // Do the replace
    vec_t<Stop> new_sch;
    vec_t<Wayp> new_rte;
    sop_replace(cand, to_replace, replacement, new_sch, new_rte);
    if (chkcap(cand.capacity(), new_sch) && chktw(new_sch, new_rte)) {
      cand.set_sch(new_sch);
      cand.set_rte(new_rte);
      cand.reset_lvn();

      // We need the following hacky gymnastics to
      // update sol_replace  with the updated cand
      MutableVehicleSptr copy_cand = std::make_shared<MutableVehicle>(cand);
      auto i = std::find_if(sol_replace.begin(), sol_replace.end(),
        [&](const std::pair<MutableVehicleSptr, std::pair<vec_t<CustId>, vec_t<CustId>>>& kv) {
          return kv.first->id() == cand.id(); });
      if (i != sol_replace.end()) {
        sol_replace[copy_cand] = i->second;
        sol_replace.erase(i);
      }

      // Sometimes cand is not part of the
      // current solution so add it manually
      if (i == sol_replace.end()) {
        vec_t<CustId> to_assign = {replacement.id()};
        vec_t<CustId> to_unassign = {to_replace};
        sol_replace[copy_cand] = std::make_pair(to_assign, to_unassign);
      } else {
        // Now we need to add replacement into assignments, and REMOVE to_replace
        // from assignments OR add it to unassignments
        vec_t<CustId>& to_assign = sol_replace.at(copy_cand).first;
        vec_t<CustId>& to_unassign = sol_replace.at(copy_cand).second;
        to_assign.push_back(replacement.id());
        auto in_assignments = std::find(to_assign.begin(), to_assign.end(), to_replace);
        if (in_assignments != to_assign.end())
          to_assign.erase(in_assignments);
        else
          to_unassign.push_back(to_replace);
      }
      customers.erase(cust);
      return sol_replace;
    } else {
      return solution;
      //continue;
    }
  //}
  return solution;
}

Solution GRASP::swap(const Solution& solution) {
  if (solution.size() < 2) return solution;

  auto k1 = solution.begin();
  auto k2 = solution.begin();
  std::uniform_int_distribution<> n(1, solution.size());
  std::advance(k1, n(this->gen)-1);
  do {
    k2 = solution.begin();
    std::advance(k2, n(this->gen)-1);
  }
  while (k2->first->id() == k1->first->id());

  vec_t<CustId> k1_assignments = k1->second.first;
  vec_t<CustId> k2_assignments = k2->second.first;

  // vec_t<MutableVehicleSptr> keys = {};
  // for (const auto& kv : solution)
  //   keys.push_back(kv.first);

  // std::shuffle(keys.begin(), keys.end(), this->gen);

  // for (auto k1 : keys) {
    // for (auto k2 : keys) {
      // if (k2->id() == k1->id()) continue;
      // vec_t<CustId> k1_assignments = solution.at(k1).first;
      // vec_t<CustId> k2_assignments = solution.at(k2).first;

      if (k1_assignments.empty() || k2_assignments.empty()) {
        return solution;
        // continue;
      }

      // MutableVehicle vehl_1 = *k1;
      // MutableVehicle vehl_2 = *k2;
      MutableVehicle vehl_1 = *k1->first;
      MutableVehicle vehl_2 = *k2->first;

      auto from_1 = k1_assignments.begin();
      auto from_2 = k2_assignments.begin();
      std::uniform_int_distribution<> m1(0, k1_assignments.size() - 1);
      std::uniform_int_distribution<> m2(0, k2_assignments.size() - 1);
      std::advance(from_1, m1(this->gen));
      std::advance(from_2, m2(this->gen));

      // for (auto from_1 = k1_assignments.begin(); from_1 != k1_assignments.end(); ++from_1) {
        // for (auto from_2 = k2_assignments.begin(); from_2 != k2_assignments.end(); ++from_2) {
          Customer cust_from_1 =
            *(std::find_if(this->customers().begin(), this->customers().end(),
              [&](const Customer& a) { return a.id() == *from_1; }));

          Customer cust_from_2 =
            *(std::find_if(this->customers().begin(), this->customers().end(),
              [&](const Customer& a) { return a.id() == *from_2; }));

          vec_t<Stop> sch_1, sch_2;
          vec_t<Wayp> rte_1, rte_2;
          sop_replace(vehl_1, *from_1, cust_from_2, sch_1, rte_1);
          sop_replace(vehl_2, *from_2, cust_from_1, sch_2, rte_2);
          if (chkcap(vehl_1.capacity(), sch_1) && chktw(sch_1, rte_1)
           && chkcap(vehl_2.capacity(), sch_2) && chktw(sch_2, rte_2)) {
            //print << "\tSwap feasible" << std::endl;
            vehl_1.set_sch(sch_1);
            vehl_2.set_sch(sch_2);
            vehl_1.set_rte(rte_1);
            vehl_2.set_rte(rte_2);
            vehl_1.reset_lvn();
            vehl_2.reset_lvn();
            k1_assignments.push_back(cust_from_2.id());
            k2_assignments.push_back(cust_from_1.id());
            auto in_k1 = std::find(k1_assignments.begin(), k1_assignments.end(), cust_from_1.id());
            auto in_k2 = std::find(k2_assignments.begin(), k2_assignments.end(), cust_from_2.id());
            k1_assignments.erase(in_k1);
            k2_assignments.erase(in_k2);

            Solution sol_swap = {};
            for (const auto& kv : solution) {  // deep copy into new objects
              if (kv.first->id() != vehl_1.id() && kv.first->id() != vehl_2.id()) {
                MutableVehicleSptr copy = std::make_shared<MutableVehicle>(*(kv.first));
                sol_swap[copy] = kv.second;
              }
            }
            MutableVehicleSptr cand_1 = std::make_shared<MutableVehicle>(vehl_1);
            MutableVehicleSptr cand_2 = std::make_shared<MutableVehicle>(vehl_2);
            // auto pair1 = std::make_pair(k1_assignments, solution.at(k1).second);
            // auto pair2 = std::make_pair(k2_assignments, solution.at(k2).second);
            auto pair1 = std::make_pair(k1_assignments, k1->second.second);
            auto pair2 = std::make_pair(k2_assignments, k2->second.second);
            sol_swap[cand_1] = pair1;
            sol_swap[cand_2] = pair2;
            return sol_swap;
          }
        // }  // end for assignments
      // }
    // }  // end for keys
  // }
  return solution;
}

Solution GRASP::rearrange(const Solution& solution) {
  if (solution.empty()) {
    //print << "\tRearrange found empty solution" << std::endl;
    return solution;
  }

  auto i = solution.begin();
  std::uniform_int_distribution<> n(0, solution.size() - 1);
  std::advance(i, n(this->gen));
  MutableVehicle vehl = *(i->first);

  //print << "\tRearranging vehl " << vehl.id() << std::endl;

  vec_t<Stop> schedule = vehl.schedule().data();
  if (schedule.size() < 5) {
    //print << "\tNot enough to rearrange" << std::endl;
    return solution;
  }

  auto j = schedule.begin();
  std::uniform_int_distribution<> m(1, schedule.size() - 2);
  std::advance(j, m(this->gen));

  // Only swap if j and its neighbor have different owners and both are customer stops
  if (j->owner() != (j+1)->owner()
  && (    j->type() != StopType::VehlOrig &&     j->type() != StopType::VehlDest)
  && ((j+1)->type() != StopType::VehlOrig && (j+1)->type() != StopType::VehlDest)) {
    std::iter_swap(j,j+1);
    vec_t<Wayp> route;
    route_through(schedule, route);
    if (chkcap(vehl.capacity(), schedule) && chktw(schedule, route)) {
      //print << "\tRearrange feasible" << std::endl;
      vehl.set_sch(schedule);
      vehl.set_rte(route);
      vehl.reset_lvn();

      Solution sol_rearrange = {};
      for (const auto& kv : solution) {  // deep copy into new objects
        if (kv.first->id() != vehl.id()) {
          MutableVehicleSptr copy = std::make_shared<MutableVehicle>(*(kv.first));
          sol_rearrange[copy] = kv.second;
        }
      }
      MutableVehicleSptr cand = std::make_shared<MutableVehicle>(vehl);
      auto pair = std::make_pair(i->second.first, i->second.second);
      sol_rearrange[cand] = pair;
      return sol_rearrange;
    }
  }
  return solution;
}

Customer GRASP::roulette(const dict<Customer, DistInt>& fitness) {
  if (fitness.size() == 0) {
    print(MessageType::Error) << "roulette called with empty fitness!" << std::endl;
    throw;
  }
  // 0. Get cost max
  int cost_max = 0;
  for (const auto& kv : fitness)
    cost_max = std::max(cost_max, kv.second);

  // 1. Get the max rank
  float max = 0;
  for (const auto& kv : fitness)
    max = std::max(max, ((float)cost_max - kv.second));

  // 2. Roulette-wheel selection
  // (https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
  std::uniform_int_distribution<>  n(0, fitness.size() - 1);
  std::uniform_real_distribution<> m(0, 1);
  auto i = fitness.begin();
  while (true) {
    i = fitness.begin();
    std::advance(i, n(this->gen));
    float per = (max == 0 ? 0 : (cost_max - i->second)/max);
    float threshold = m(this->gen);
    if (max == 0 || per > threshold)
      break;
  }
  return i->first;
}

DistInt GRASP::cost(const Solution& solution) {
  DistInt sum = 0;
  vec_t<CustId> matched = {};
  for (const auto& kv : solution) {
    for (const CustId& cid : kv.second.first)
      matched.push_back(cid);
    sum += kv.first->route().cost();
  }
  for (const Customer& cust : this->customers()) {
    bool is_matched = (std::find(matched.begin(), matched.end(), cust.id()) != matched.end());
    if (!is_matched)
      sum += Cargo::basecost(cust.id());
  }

  return sum;
}

void GRASP::print_sol(const Solution& sol) {
  for (const auto& kv : sol) {
    print << "  vehl " << kv.first->id() << ": {";
    for (const auto& cid : kv.second.first) print << cid << " ";
    print << "}, {";
    for (const auto& cid : kv.second.second) print << cid << ", ";
    print << "}" << std::endl;
    //print_sch(kv.first->schedule().data());
  }
}

void GRASP::commit(const Solution& solution) {
  for (auto& kv : solution) {
    MutableVehicleSptr cand = kv.first;
    const vec_t<CustId>& cadd = kv.second.first;
    const vec_t<CustId>& cdel = kv.second.second;
    cand->reset_lvn();
    this->assign_or_delay(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand);
    for (const CustId& cid : cadd) print << "Matched " << cid << " to vehl " << cand->id() << std::endl;
    for (const CustId& cid : cdel) print << "Removed " << cid << " from vehl " << cand->id() << std::endl;
  }
}

void GRASP::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void GRASP::end() {
  print << "nswap: " << nswap_ << std::endl;
  print << "nreplace: " << nreplace_ << std::endl;
  print << "nrearrange: " << nrearrange_ << std::endl;
  print << "nnoimprov: " << nnoimprov_ << std::endl;
  RSAlgorithm::end();
}

void GRASP::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

