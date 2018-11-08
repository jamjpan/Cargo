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
#include <vector>

#include "libcargo.h"
#include "grasp.h"

using namespace cargo;

const int BATCH = 30;
const int MAX_ITER = 2;

GRASP::GRASP()
    : RSAlgorithm("grasp", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
  this->nswap_ = this->nreplace_ = this->nrearrange_ = this->nnoimprov_ = 0;
}

void GRASP::match() {
  this->beg_batch_ht();
  this->timeout_0 = hiclock::now();
  Solution best_solution = {};
  DistInt best_cost = InfInt;

  for (int count = 0; count < MAX_ITER; ++count) {
    auto pert_0 = hiclock::now();
    Grid local_grid(this->grid_);  // deep copy
    vec_t<Customer> local_customers = this->customers();

    // bottleneck
    auto init_0 = hiclock::now();
    Solution initial_solution = this->initialize(local_grid, local_customers);
    auto init_1 = hiclock::now();
    print << "init: " << std::round(dur_milli(init_1-init_0).count()) << std::endl;

    DistInt local_cost = this->cost(initial_solution);
    bool has_improvement = false;
    Solution local_best = initial_solution;
    //print << "Initialized (cost=" << local_cost << ")" << std::endl;
    do {
      has_improvement = false;
      auto repl_0 = hiclock::now();
      Solution sol_replace      = this->replace(local_best, local_customers);
      auto repl_1 = hiclock::now();
      print << "repl: " << std::round(dur_milli(repl_1-repl_0).count()) << std::endl;
      auto swap_0 = hiclock::now();
      Solution sol_swap         = this->swap(local_best);
      auto swap_1 = hiclock::now();
      print << "swap: " << std::round(dur_milli(swap_1-swap_0).count()) << std::endl;
      auto arrg_0 = hiclock::now();
      Solution sol_rearrange    = this->rearrange(local_best);
      auto arrg_1 = hiclock::now();
      print << "arrg: " << std::round(dur_milli(arrg_1-arrg_0).count()) << std::endl;
      DistInt replace_cost      = this->cost(sol_replace);
      DistInt swap_cost         = this->cost(sol_swap);
      DistInt rearrange_cost    = this->cost(sol_rearrange);
      if (replace_cost < local_cost) {
        //print << "\tAccepted replace (" << replace_cost << ")" << std::endl;
        local_best = sol_replace;
        local_cost = replace_cost;
        has_improvement = true;
        this->nreplace_++;
      }
      if (swap_cost < local_cost) {
        //print << "\tAccepted swap (" << swap_cost << ")" << std::endl;
        local_best = sol_swap;
        local_cost = swap_cost;
        has_improvement = true;
        this->nswap_++;
      }
      if (rearrange_cost < local_cost) {
        //print << "\tAccepted rearrange (" << rearrange_cost << ")" << std::endl;
        local_best = sol_rearrange;
        local_cost = rearrange_cost;
        has_improvement = true;
        this->nrearrange_++;
      }
    } while (has_improvement);
    if (local_cost < best_cost) {
      best_solution = local_best;
      best_cost = local_cost;
    }
    auto pert_1 = hiclock::now();
    print << "  pert: " << std::round(dur_milli(pert_1-pert_0).count()) << std::endl;
  }
  // best_solution = initial_solution;  // for testing
  this->commit(best_solution);
  this->end_batch_ht();
}

Solution GRASP::initialize(Grid& grid, vec_t<Customer>& customers) {
  Solution initial_solution = {};
  this->candidates_list = {};

  // 1. List all the candidates for each customer (0-500 ms)
  auto cand_0 = hiclock::now();
  vec_t<MutableVehicleSptr> vehicles = {};
  for (const Customer& cust : customers) {
    this->candidates_index[cust] = {};
    vec_t<MutableVehicleSptr> candidates = grid.within(pickup_range(cust), cust.orig());
    for (const MutableVehicleSptr& cand : candidates) {
      vehicles.push_back(cand);
      this->candidates_index.at(cust).push_back(cand);
      if (this->candidates_list.count(cand) == 0)
        this->candidates_list[cand] = {};
      this->candidates_list[cand].push_back(cust);
    }
  }
  auto cand_1 = hiclock::now();
  print << "  cand: " << std::round(dur_milli(cand_1-cand_0).count()) << std::endl;

  // 2. Greedily Randomly Adaptively assign!
  while (!vehicles.empty() && !customers.empty()) {
    MutableVehicleSptr cand = vehicles.back();
    vehicles.pop_back();
    if (this->candidates_list.at(cand).size() == 0)
      continue;
    bool cand_done = false;
    while (!cand_done && this->candidates_list.at(cand).size() > 0) {
      vec_t<Stop> sch = {};
      vec_t<Wayp> rte = {};
      this->fitness[cand] = {};
      this->schedules[cand] = {};
      this->routes[cand] = {};
      // bottleneck -- recompute fitness for every customer!
      for (const Customer& cust : this->candidates_list.at(cand)) {
        if (this->timeout(this->timeout_0))
          return initial_solution;
        this->fitness[cand][cust] = sop_insert(cand, cust, sch, rte);
        this->schedules[cand][cust] = sch;
        this->routes[cand][cust] = rte;
      }
      Customer cust_to_add = this->roulette(this->fitness.at(cand));
      sch = this->schedules.at(cand).at(cust_to_add);
      rte = this->routes.at(cand).at(cust_to_add);
      if (chktw(sch, rte) && chkcap(cand->capacity(), sch)) {
        cand->set_sch(schedules.at(cand).at(cust_to_add));  // modify the deep copy
        cand->set_rte(routes.at(cand).at(cust_to_add));     // modify the deep copy
        cand->reset_lvn();
        if (initial_solution.count(cand) == 0)
          initial_solution[cand].first = {};
        (initial_solution[cand].first).push_back(cust_to_add.id());
        auto in_customers = std::find_if(customers.begin(), customers.end(),
          [&](const Customer& a) {
            return a.id() == cust_to_add.id();
        });
        customers.erase(in_customers);
        for (auto& kv : this->candidates_list) {
          auto in_list = std::find_if(kv.second.begin(), kv.second.end(),
            [&](const Customer& a) {
              return a.id() == cust_to_add.id();
          });
          if (in_list != kv.second.end())
            kv.second.erase(in_list);
        }
      } else {
        cand_done = true;
      }
    }
    if (this->timeout(this->timeout_0))
      return initial_solution;
  }
  return initial_solution;
}

Solution GRASP::replace(const Solution& solution, vec_t<Customer>& customers) {
  if (customers.size() == 0) {
    //print << "\tReplace found no unassigned" << std::endl;
    return solution;
  }
  if (solution.size() == 0) {
    //print << "\tReplace found no assigned" << std::endl;
    return solution;
  }

  // Only initialize(...) requires MutableVehicleSptr POINTERS because each
  // physical vehicle can get updated during the initialize loop. WE COULD make
  // a second Solution type that uses PHYSICAL VEHICLES, OR we could gymnastics
  // the existing Solution type by using deep copies. BOTH METHODS are ugly.
  // CAN WE DO BETTER????
  Solution sol_replace = {};
  for (const auto& kv : solution) {  // deep copy into new objects
    MutableVehicleSptr copy = std::make_shared<MutableVehicle>(*(kv.first));
    sol_replace[copy] = kv.second;
  }

  auto i = customers.begin();  // select a replacement
  std::uniform_int_distribution<> n(0, customers.size() - 1);
  std::advance(i, n(this->gen));
  Customer replacement = *i;
  //print << "\tReplace got " << replacement.id() << " as replacement" << std::endl;

  vec_t<MutableVehicleSptr>& candidates = candidates_index.at(replacement);
  if (candidates.size() == 0) {
    //print << "\tReplacement " << replacement.id() << " has no candidates!" << std::endl;
    return solution;
  }
  auto j = candidates.begin();
  std::uniform_int_distribution<> m(0, candidates.size() - 1);
  std::advance(j, m(this->gen));
  MutableVehicle cand = **j;  // copy the physical vehicle

  CustId to_replace = randcust(cand.schedule().data());
  if (to_replace == -1) {
    //print << "\tVehicle " << cand.id() << " has no replaceables!" << std::endl;
    return solution;
  }
  vec_t<Stop> new_sch;
  vec_t<Wayp> new_rte;
  sop_replace(cand, to_replace, replacement, new_sch, new_rte);
  if (chkcap(cand.capacity(), new_sch) && chktw(new_sch, new_rte)) {
    //print << "\tReplacement feasible" << std::endl;
    cand.set_sch(new_sch);
    cand.set_rte(new_rte);
    cand.reset_lvn();
    // We need the following hacky gymnastics to update sol_replace
    // with the updated cand
    MutableVehicleSptr copy_cand = std::make_shared<MutableVehicle>(cand);
    auto i = sol_replace.begin();
    for (; i != sol_replace.end(); ++i) {
      if (i->first->id() == cand.id()) {
        sol_replace[copy_cand] = i->second;
        sol_replace.erase(i);
        break;
      }
    }
    // Sometimes cand is NOT PART OF THE CURRENT SOLUTION.
    // Just add it manually.
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
    return sol_replace;
  } else {
    //print << "\tReplacement not feasible" << std::endl;
    return solution;
  }
}

Solution GRASP::swap(const Solution& solution) {
  if (solution.size() < 2) {
    //print << "\tSwap found not enough assignments" << std::endl;
    return solution;
  }

  auto k1 = solution.begin();
  auto k2 = solution.begin();
  std::uniform_int_distribution<> n(0, solution.size() - 1);
  while (k1 == k2) {
    std::advance(k1, n(this->gen));
    std::advance(k2, n(this->gen));
  }

  vec_t<CustId> k1_assignments = k1->second.first;
  vec_t<CustId> k2_assignments = k2->second.first;
  if (k1_assignments.empty() || k2_assignments.empty()) {
    //print << "\tSwap giving up" << std::endl;
    return solution;
  }
  MutableVehicle vehl_1 = *k1->first;
  MutableVehicle vehl_2 = *k2->first;

  auto from_1 = k1_assignments.begin();
  auto from_2 = k2_assignments.begin();
  std::uniform_int_distribution<> m1(0, k1_assignments.size() - 1);
  std::uniform_int_distribution<> m2(0, k2_assignments.size() - 1);
  std::advance(from_1, m1(this->gen));
  std::advance(from_2, m2(this->gen));

  Customer cust_from_1 =
    *(std::find_if(this->customers().begin(), this->customers().end(),
      [&](const Customer& a) { return a.id() == *from_1; }));

  Customer cust_from_2 =
    *(std::find_if(this->customers().begin(), this->customers().end(),
      [&](const Customer& a) { return a.id() == *from_2; }));

  //print << "\tSwapping " << cust_from_1.id() << " from " << vehl_1.id()
  //      << " with " << cust_from_2.id() << " from " << vehl_2.id() << std::endl;

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
    auto pair1 = std::make_pair(k1_assignments, k1->second.second);
    auto pair2 = std::make_pair(k2_assignments, k2->second.second);
    sol_swap[cand_1] = pair1;
    sol_swap[cand_2] = pair2;
    return sol_swap;
  } else {
    //print << "\tSwap not feasible" << std::endl;
    return solution;
  }
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
  }

  auto j = schedule.begin();
  std::uniform_int_distribution<> m(1, schedule.size() - 2);
  std::advance(j, m(this->gen));

  if (j->owner() == (j+1)->owner()) {
    if (j->type() == StopType::CustOrig && (j+1)->type() == StopType::CustDest)
      std::iter_swap(j+1,j+2);
    else {
      print(MessageType::Error) << "adjacent stops not in order!?" << std::endl;
      for (const Stop& stop : schedule)
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
      throw;
    }
  }
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
  } else {
    //print << "\tRearrange not feasible" << std::endl;
    return solution;
  }
}

Customer GRASP::roulette(const dict<Customer, DistInt>& fitness) {
  if (fitness.size() == 0) {
    print(MessageType::Error) << "roulette called with empty fitness!" << std::endl;
    throw;
  }
  // 1. Get the max rank
  int max = 0;
  for (const auto& kv : fitness) max += kv.second;

  // 2. Roulette-wheel selection
  // (https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
  std::uniform_int_distribution<>  n(0, fitness.size() - 1);
  std::uniform_real_distribution<> m(0, 1);
  auto i = fitness.begin();
  while (true) {
    i = fitness.begin();
    std::advance(i, n(this->gen));
    float threshold = m(this->gen);
    float rankratio = (max = 0 ? 0 : 1 - (float)i->second/max);
    if (rankratio == 0 || rankratio > threshold)
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

void GRASP::commit(const Solution& solution) {
  for (auto& kv : solution) {
    MutableVehicleSptr cand = kv.first;
    const vec_t<CustId>& cadd = kv.second.first;
    const vec_t<CustId>& cdel = kv.second.second;
    cand->reset_lvn();
    this->assign_or_delay(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand);
    //for (const CustId& cid : cadd) print << "Matched " << cid << " to vehl " << cand->id() << std::endl;
    //for (const CustId& cid : cdel) {
    //  print << "Removed " << cid << " from vehl " << cand->id() << std::endl;
    //  this->beg_delay(cid);
    //}
  }
}

void GRASP::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void GRASP::end() {
  this->print_statistics();
  print << "nswap: " << nswap_ << std::endl;
  print << "nreplace: " << nreplace_ << std::endl;
  print << "nrearrange: " << nrearrange_ << std::endl;
  print << "nnoimprov: " << nnoimprov_ << std::endl;
}

void GRASP::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-m5k-c3.instance";
  option.path_to_solution = "grasp.sol";
  option.path_to_dataout  = "grasp.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode = false;
  option.static_mode = true;
  Cargo cargo(option);
  GRASP grasp;
  cargo.start(grasp);

  return 0;
}

