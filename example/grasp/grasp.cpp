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
const int MAX_ITER = 100;
const int MAX_C = 10;

GRASP::GRASP()
    : RSAlgorithm("grasp", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
  this->nswap_ = 0;
  this->nreplace_ = 0;
  this->nrearrange_ = 0;
  this->nnoimprov_ = 0;
}

void GRASP::match() {
  this->beg_ht();
  this->timeout_ = this->timeout_/MAX_ITER/2;

  Solution best = {};
  DistInt cost = InfInt;
  int stat_improv = -1;
  for (int iter_count = 0; iter_count < MAX_ITER; ++iter_count) {
    print << "Initializing sol_0" << std::endl;
    Grid local_grid = this->grid_;
    Solution sol_0 = this->initialize(local_grid);  // <-- bottleneck
    print << "Done initialize" << std::endl;
    this->verify(sol_0);
    print << "Passed verification" << std::endl;
    DistInt sol_0_cost = this->solcost(sol_0);
    best = sol_0;
    cost = sol_0_cost;

    this->timeout_0 = hiclock::now();
    do {
      print << "Searching for improvement" << std::endl;
      stat_improv = -1;

      Solution sol_1 = replace(sol_0, local_grid);
      this->verify(sol_1);
      DistInt sol_1_cost = this->solcost(sol_1);
      print << "Replace cost: " << sol_1_cost << " (incumbent: " << cost << ")" << std::endl;
      if (sol_1_cost < cost) {
        print << "Acceped replace" << std::endl;
        best = sol_1;
        cost = sol_1_cost;
        stat_improv = 1;
      }

      Solution sol_2 = swap(sol_0, local_grid);
      this->verify(sol_2);
      DistInt sol_2_cost = this->solcost(sol_2);
      print << "Swap cost: " << sol_2_cost << " (incumbent: " << cost << ")" << std::endl;
      if (sol_2_cost < cost) {
        print << "Accepted swap" << std::endl;
        best = sol_2;
        cost = sol_2_cost;
        stat_improv = 2;
      }

      Solution sol_3 = rearrange(sol_0);
      this->verify(sol_3);
      DistInt sol_3_cost = this->solcost(sol_3);
      print << "Rearrange cost: " << sol_3_cost << " (incumbent: " << cost << ")" << std::endl;
      if (sol_3_cost < cost) {
        print << "Accepted rearrange" << std::endl;
        best = sol_3;
        cost = sol_3_cost;
        stat_improv = 3;
      }

      if (stat_improv == -1)
        this->nnoimprov_++;
      else if (stat_improv == 1)
        this->nreplace_++;
      else if (stat_improv == 2)
        this->nswap_++;
      else if (stat_improv == 3)
        this->nrearrange_++;
      // Have to update the grid in order to use replace/swap
      for (const auto& kv : best) {
        this->vehicles_lookup.at(kv.first.id())->set_sch(kv.first.schedule().data());
        this->vehicles_lookup.at(kv.first.id())->set_rte(kv.first.route().data());
        this->vehicles_lookup.at(kv.first.id())->reset_lvn();
        // don't bother with incr/decr_queued, it's not used
      }
      // Get ready for the next iteration
      sol_0 = best;
      print << "Done improvement" << std::endl;
      if (this->timeout(this->timeout_0)) {
        print << "hit timeout" << std::endl;
        break;
      }
    } while (stat_improv != -1);
  }

  this->commit(best);
  this->end_ht();
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

Solution GRASP::initialize(Grid& local_grid) {
  this->timeout_1 = hiclock::now();
  Solution sol_0 = {};
  // THIS SOLDEX THING IS HORRIBLE! GET RID OF IT?
  dict<VehlId, std::pair<MutableVehicle, std::pair<vec_t<Customer>, vec_t<Customer>>>> soldex;

  vec_t<Customer> local_customers = this->customers();
  vec_t<MutableVehicleSptr> local_vehicles = {};

  // Get candidates list of vehicles and customers
  print << "\tBuilding candidates list" << std::endl;
  this->candidates_list = {};
  this->range = {};
  for (const Customer& cust : this->customers()) {
    vec_t<MutableVehicleSptr> cands = local_grid.within(pickup_range(cust), cust.orig());
    for (MutableVehicleSptr& cand : cands) {
      this->vehicles_lookup[cand->id()] = cand;  // need for later
      if (this->candidates_list.count(cand->id()) == 0) {
        this->candidates_list[cand->id()] = {};
        local_vehicles.push_back(cand);
      }
      this->candidates_list.at(cand->id()).push_back(cust);
      this->range[(cust.id())] = haversine(cand->last_visited_node(), cust.orig());
    }
  }
  // Speed-up Heuristic! Keep the top MAX_C NEAREST customers per vehicle
  for (auto& kv : this->candidates_list) {
    if (kv.second.size() > MAX_C) {
      vec_t<std::pair<DistInt, Customer>> top = {};
      for (const Customer& cust : kv.second) {
        auto rank = std::make_pair(range.at(cust.id()), cust);
        top.push_back(rank);
      }
      std::sort(top.begin(), top.end(), [](std::pair<DistInt, Customer>& a, std::pair<DistInt, Customer>& b) {
          return a.first < b.first; });
      vec_t<Customer> new_list = {};
      for (auto i = top.begin(); i < top.begin()+MAX_C || i != top.end(); ++i)
        new_list.push_back(i->second);
      this->candidates_list.at(kv.first) = new_list;
    }
  }
  print << "\tDone candidates list" << std::endl;

  std::shuffle(local_vehicles.begin(), local_vehicles.end(), this->gen);

  print << "\tAssigning customers to vehicles" << std::endl;
  while (!local_vehicles.empty() && !local_customers.empty()) {
    // 1. Select random vehicle (we've shuffled the list)
    MutableVehicleSptr cand = local_vehicles.back();
    local_vehicles.pop_back();

    print << "\t\tSelected Vehl " << cand->id() << std::endl;

    dict<Customer, int> fitness = {};
    dict<Customer, vec_t<Stop>> schedule = {};
    dict<Customer, vec_t<Wayp>> route = {};

    int count = 0;
    bool has_change = false;
    while (!this->candidates_list.at(cand->id()).empty()) {
      // 2. (Re)-compute fitness <-- bottleneck
      print << "\t\t\t(Re)-computing fitness" << std::endl;
      vec_t<Stop> sch;
      vec_t<Wayp> rte;
      if (count == 0 || has_change) {
        for (const Customer& cust : this->candidates_list.at(cand->id())) {
          int score = sop_insert(*cand, cust, sch, rte) - cand->route().cost();
          fitness[cust] = score;
          schedule[cust] = sch;
          route[cust] = rte;
          print << "\t\t\t\tCust " << cust.id() << ": " << score << std::endl;
          // if (this->timeout(this->timeout_1)) {
          //   print << "hit timeout" << std::endl;
          //   break;
          // }
        }
      } else {
        print << "\t\t\tNo change to fitness" << std::endl;
      }

      // 3. Roulette-select one customer to add to the vehicle
      print << "\t\t\tRolling the roulette wheel" << std::endl;
      Customer cust_to_add = roulette_select(fitness);
      sch = schedule.at(cust_to_add);
      rte = route.at(cust_to_add);
      print << "\t\t\tConfirming cust " << cust_to_add.id() << std::endl;
      if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {  // <-- often fails
        cand->set_sch(sch);
        cand->set_rte(rte);
        cand->reset_lvn();
        cand->incr_queued();
        if (soldex.count(cand->id()) == 0) {
          vec_t<Customer> assignments = {};
          vec_t<Customer> unassign = {};
          auto data = std::make_pair(assignments, unassign);
          soldex[cand->id()] = std::make_pair(*cand, data);
        }
        soldex.at(cand->id()).first = *cand;
        soldex.at(cand->id()).second.first.push_back(cust_to_add);
        print << "\t\t\tAdded cust " << cust_to_add.id() << std::endl;
        // Clean up customer from stores
        for (auto& kv : this->candidates_list) {
          vec_t<Customer>& custs = kv.second;
          auto i = std::find(custs.begin(), custs.end(), cust_to_add);
          if (i != custs.end())
            custs.erase(i);
        }
        local_customers.erase(std::find(local_customers.begin(), local_customers.end(), cust_to_add));
        fitness.erase(cust_to_add);
        schedule.erase(cust_to_add);
        route.erase(cust_to_add);
        has_change = true;
      } else {
        print << "\t\t\tInvalid." << std::endl;
        vec_t<Customer>& possibles = this->candidates_list.at(cand->id());
        possibles.erase(std::find(possibles.begin(), possibles.end(), cust_to_add));
        fitness.erase(cust_to_add);
        schedule.erase(cust_to_add);
        route.erase(cust_to_add);
        has_change = false;
      }
      if (this->timeout(this->timeout_1)) {
        print << "hit timeout" << std::endl;
        break;
      }
      count++;
    }
  }
  print << "\tDone assignment" << std::endl;
  for (const auto& kv : soldex) {
    print << kv.first << " (" << kv.second.second.first.size() << ")" << std::endl;
    sol_0[kv.second.first] = kv.second.second;
  }
  return sol_0;
}

Solution GRASP::replace(const Solution& sol, Grid& local_grid) {
  if (sol.empty()) {
    print << "Replace returning empty solution" << std::endl;
    return sol;
  }

  // 1. Get unassigned
  vec_t<Customer> customers = this->customers();  // copy
  vec_t<Customer> assigned = {};
  for (const auto& kv : sol)
    for (const Customer& cust : kv.second.first)
      assigned.push_back(cust);  // copy

  vec_t<Customer> unassigned;
  std::sort(customers.begin(), customers.end());
  std::sort(assigned.begin(), assigned.end());
  std::set_difference(customers.begin(), customers.end(), assigned.begin(), assigned.end(),
    std::inserter(unassigned, unassigned.begin()));

  print << "Replace got " << unassigned.size() << " unassigned." << std::endl;
  if (unassigned.size() == 0)
    return sol;

  // 2. Select random unassigned
  std::uniform_int_distribution<> n(0, unassigned.size() - 1);
  auto replace_by = unassigned.begin();
  std::advance(replace_by, n(this->gen));

  print << "\tSelected " << replace_by->id() << " for replace by" << std::endl;

  // 3. Replace
  vec_t<MutableVehicleSptr> candidates = local_grid.within(pickup_range(*replace_by), replace_by->orig());
  std::shuffle(candidates.begin(), candidates.end(), this->gen);
  CustId replace_me = -1;
  auto candptr = candidates.begin();
  while (replace_me == -1 && candptr != candidates.end()) {
    replace_me = randcust((*candptr)->schedule().data());
    if (replace_me == -1)
      candptr++;
  }
  if (replace_me == -1) {
    print << "\tNo replaceable customers" << std::endl;
    return sol;
  }
  if (candptr == candidates.end()) {
    print << "\tNo vehicles have replaceable customer" << std::endl;
    return sol;  // return now, or try again?
  } else {
    MutableVehicle cand = **candptr;
    vec_t<Stop> sch;
    vec_t<Wayp> rte;
    sop_replace(cand, replace_me, *replace_by, sch, rte);
    Customer to_replace = Cargo::basecust(replace_me);
    if (chkcap(cand.capacity(), sch) && chktw(sch, rte)) {
      cand.set_sch(sch);
      cand.set_rte(rte);
      cand.reset_lvn();

      // 4. Update the solution
      auto improved_sol = sol;
      vec_t<Customer> new_assignments, unassignments;
      for (auto itr = improved_sol.begin(); itr != improved_sol.end(); ++itr) {
        if (itr->first.id() == cand.id()) {
          new_assignments = itr->second.first;
          unassignments = itr->second.second;
          improved_sol.erase(itr);
          break;
        }
      }
      new_assignments.push_back(*replace_by);
      // If replace_me is an existing customer, it needs to be added to unassignments
      auto i = std::find_if(new_assignments.begin(), new_assignments.end(), [&](const Customer& a) {
                  return a.id() == replace_me; });
      if (i == new_assignments.end())  // cannot find in THIS ITERATION'S assignments; it must be a prior
        unassignments.push_back(to_replace);
      else  // else, remove from assignments
        new_assignments.erase(i);
      improved_sol[cand] = {new_assignments, unassignments};
      return improved_sol;
    } else {
      print << "\tReplace " << replace_me << " with " << replace_by->id() << " on vehl " << cand.id() << " not feasible" << std::endl;
      return sol;
    }
  }
}

Solution GRASP::swap(const Solution& sol, Grid& local_grid) {
  if (sol.empty()) {
    print << "Swap returning empty solution" << std::endl;
    return sol;
  }

  print << "Swap" << std::endl;

  // 1. Select random vehicle
  std::uniform_int_distribution<> n(0, sol.size() - 1);
  auto k1ptr = sol.begin();
  std::advance(k1ptr, n(this->gen));
  MutableVehicle k1 = k1ptr->first;

  // 2. Select random customer assigned to k1
  vec_t<Customer> assignments = k1ptr->second.first;
  std::uniform_int_distribution<> m(0, assignments.size() - 1);
  auto cptr = assignments.begin();
  std::advance(cptr, m(this->gen));
  Customer from_k1 = *cptr;

  print << "\tSelected " << from_k1.id() << " from " << k1.id() << " for swap" << std::endl;

  vec_t<MutableVehicleSptr> candidates = local_grid.within(pickup_range(from_k1), from_k1.orig());

  // 3. Select random candidate
  std::shuffle(candidates.begin(), candidates.end(), this->gen);
  CustId replace_me = -1;
  auto candptr = candidates.begin();
  while (replace_me == -1 || replace_me == from_k1.id()) {
    if ((*candptr)->id() == k1.id())
      candptr++;  // skip it
    else {
      replace_me = randcust((*candptr)->schedule().data());
      if (replace_me == -1 || replace_me == from_k1.id()) {
        candptr++;
      }
    }
    if(candptr == candidates.end())
      break;
  }
  if (replace_me == -1) {
    print << "\tNo replaceable customers" << std::endl;
    return sol;
  }
  if (candptr == candidates.end()) {
    print << "\tNo vehicles have swappable customer for " << from_k1.id() << std::endl;
    return sol;  // return now, or try again?
  } else {
    MutableVehicle k2 = **candptr;
    Customer from_k2 = Cargo::basecust(replace_me);

    print << "\tSelected " << from_k2.id() << " from " << k2.id() << " for swap" << std::endl;

    vec_t<Stop> sch1, sch2;
    vec_t<Wayp> rte1, rte2;
    sop_replace(k1, from_k1.id(), from_k2, sch1, rte1);
    sop_replace(k2, from_k2.id(), from_k1, sch2, rte2);
    if (chkcap(k1.capacity(), sch1) && chktw(sch1, rte1)
     && chkcap(k2.capacity(), sch2) && chktw(sch2, rte2)) {
      k1.set_sch(sch1);
      k1.set_rte(rte1);
      k1.reset_lvn();
      k2.set_sch(sch2);
      k2.set_rte(rte2);
      k2.reset_lvn();

      // 4. Update the solution
      auto improved_sol = sol;
      vec_t<Customer> new_assignments1, unassignments1, new_assignments2, unassignments2;

      // Get k1
      for (auto itr = improved_sol.begin(); itr != improved_sol.end(); ++itr) {
        if (itr->first.id() == k1.id()) {
          new_assignments1 = itr->second.first;
          unassignments1 = itr->second.second;
          improved_sol.erase(itr);
          break;
        }
      }
      new_assignments1.push_back(from_k2);

      // Get k2
      for (auto itr = improved_sol.begin(); itr != improved_sol.end(); ++itr) {
        if (itr->first.id() == k2.id()) {
          new_assignments2 = itr->second.first;
          unassignments2 = itr->second.second;
          improved_sol.erase(itr);
          break;
        }
      }
      new_assignments2.push_back(from_k1);

      // from_k1 needs to be removed from k1's assignments or added to k1's unassignments
      { auto i = std::find_if(new_assignments1.begin(), new_assignments1.end(), [&](const Customer& a) {
                  return a.id() == from_k1.id(); });
      if (i == new_assignments1.end())  // cannot find in THIS ITERATION'S assignments; it must be a prior
        unassignments2.push_back(from_k1);
      else  // else, remove from assignments
        new_assignments1.erase(i);
      }

      // from_k2 needs to be removed from k2's assignments or added to k2's unassignments
      {auto i = std::find_if(new_assignments2.begin(), new_assignments2.end(), [&](const Customer& a) {
                  return a.id() == from_k2.id(); });
      if (i == new_assignments2.end())  // cannot find in THIS ITERATION'S assignments; it must be a prior
        unassignments2.push_back(from_k2);
      else  // else, remove from assignments
        new_assignments2.erase(i);
      }

      improved_sol[k1] = {new_assignments1, unassignments1};
      improved_sol[k2] = {new_assignments2, unassignments2};
      return improved_sol;
    } else {
      print << "\tSwap not feasible" << std::endl;
      return sol;
    }
  }
}

Solution GRASP::rearrange(const Solution& sol) {
  if (sol.empty()) {
    print << "Rearrange returning empty solution" << std::endl;
    return sol;
  }

  print << "Rearrange" << std::endl;

  // 1. Randomly choose a vehicle in sol
  auto improved_sol = sol;
  std::uniform_int_distribution<>  n(0, improved_sol.size() - 1);
  auto i = improved_sol.begin();
  std::advance(i, n(this->gen));
  MutableVehicle cand = i->first;
  MutableVehicle improved_cand = cand;

  // 2. Randomly choose a stop in vehicle schedule
  vec_t<Stop> schedule = cand.schedule().data();
  if (schedule.size() < 5) {
    print << "\tNot enough stops, cannot rearrange" << std::endl;
    return sol;
  }
  std::uniform_int_distribution<>  m(1, schedule.size() - 2);
  auto j = schedule.begin();
  std::advance(j, m(this->gen));

  print << "\tSelected " << j->loc() << " from " << cand.id() << " for rearrange" << std::endl;
  for (const Stop& stop : schedule)
    print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
  print << std::endl;

  // 3. If it is an origin, check if neighbor is same dest
  //   a. if so, choose the dest and swap it down
  if (j->owner() == (j+1)->owner()) {
    if (j->type() == StopType::CustOrig && (j+1)->type() == StopType::CustDest)
      std::iter_swap(j+1,j+2);
    else {
      print(MessageType::Error) << "adjacent stops not in order!?" << std::endl;
      for (const Stop& stop : schedule)
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
    }
  }
  //   b. if not, swap it down
  std::iter_swap(j,j+1);
  for (const Stop& stop : schedule)
    print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
  print << std::endl;

  vec_t<Wayp> route;
  route_through(schedule, route);
  if (chkcap(cand.capacity(), schedule) && chktw(schedule, route)) {
    improved_sol.erase(cand);
    improved_cand.set_sch(schedule);
    improved_cand.set_rte(route);
    improved_sol[improved_cand] = sol.at(cand);
    return improved_sol;
  } else {
    print << "Rearrange not feasible" << std::endl;
    return sol;
  }

}

Customer GRASP::roulette_select(const dict<Customer, int>& fitness) {
  if (fitness.size() == 0) {
    print(MessageType::Error)
        << "roulette_select called with empty fitness!" << std::endl;
    throw;
  }
  // 1. Get the max rank
  int max = 0;
  for (const auto& kv : fitness)
    max += kv.second;

  // 2. Roulette-wheel selection
  // (see https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
  std::uniform_int_distribution<>  n(0, fitness.size() - 1);
  std::uniform_real_distribution<> m(0, 1);
  auto i = fitness.begin();
  while (true) {
    i = fitness.begin();
    std::advance(i, n(this->gen));
    float threshold = m(this->gen);
    float rankratio = (max = 0 ? 0 : 1 - (float)i->second/max);
    if (rankratio == 0 || rankratio > threshold) {
      break;
    } else {
      print << "Roulette failed to hit " << rankratio << " not > " << threshold << std::endl;
    }
  }
  return i->first;
}

void GRASP::commit(const Solution& sol) {
  for (const auto& kv : sol) {
    MutableVehicle cand = kv.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : (kv.second).first) {
      cadd.push_back(cust.id());
      print << "Matched " << cust.id() << " with " << cand.id() << std::endl;
    }
    for (const Customer& cust : (kv.second).second)
      cdel.push_back(cust.id());
    this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand, false /*true*/);
  }
}

DistInt GRASP::solcost(const Solution& sol) {
  vec_t<CustId> assigned = {};
  DistInt sum = 0;
  for (const auto& kv : sol) {
    sum += (kv.first).route().cost();
    for (const Customer& cust : (kv.second).first)
      assigned.push_back(cust.id());
  }
  for (const Customer& cust : this->customers())
    if (std::find(assigned.begin(), assigned.end(), cust.id()) == assigned.end())
      sum += Cargo::basecost(cust.id());
  return sum;
}

bool GRASP::verify(const Solution& sol) {
  for (auto& kv : sol) {
    const MutableVehicle& vehl = kv.first;
    const vec_t<Stop>& schedule = vehl.schedule().data();
    const vec_t<Customer>& assigned = kv.second.first;
    const vec_t<Customer>& unassign = kv.second.second;
    for (const Customer& cust : assigned) {
      bool has_origin = (std::find_if(schedule.begin(), schedule.end(), [&](const Stop& stop) {
        return stop.owner() == cust.id() && stop.type() == StopType::CustOrig;
              }) != schedule.end());
      bool has_destination = (std::find_if(schedule.begin(), schedule.end(), [&](const Stop& stop) {
        return stop.owner() == cust.id() && stop.type() == StopType::CustDest;
              }) != schedule.end());
      if (!(has_origin && has_destination)) {
        print(MessageType::Error) << "Solution assigns " << cust.id() << " to " << vehl.id() << " but schedule incomplete!" << std::endl;
        for (const Stop& stop : schedule)
          print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
        print << std::endl;
        throw;
      }
    }
    for (const Customer& cust : unassign) {
      bool has_origin = (std::find_if(schedule.begin(), schedule.end(), [&](const Stop& stop) {
        return stop.owner() == cust.id() && stop.type() == StopType::CustOrig;
              }) != schedule.end());
      bool has_destination = (std::find_if(schedule.begin(), schedule.end(), [&](const Stop& stop) {
        return stop.owner() == cust.id() && stop.type() == StopType::CustDest;
              }) != schedule.end());
      if (has_origin || has_destination) {
        print(MessageType::Error) << "Solution unassigns " << cust.id() << " from " << vehl.id() << " but schedule malformed!" << std::endl;
        for (const Stop& stop : schedule)
          print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
        print << std::endl;
        throw;
      }
    }
  }
  return true;
}

void GRASP::print_sol(const Solution& sol) {
  for (const auto& kv : sol) {
    print << "Vehl " << kv.first.id() << std::endl;
    print << "\tAssigned to ";
    for (const Customer& cust : (kv.second).first)
      print << cust.id() << " ";
    print << std::endl;
    print << "\tUnassigned from ";
    for (const Customer& cust : (kv.second).second)
      print << cust.id() << " ";
    print << std::endl;
    print << "\tRoute: (omitted)" << std::endl;
    print << "\tSchedule: ";
    for (const Stop& stop : (kv.first).schedule().data())
      print << stop.loc() << " ";
    print << std::endl;
  }
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
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode = true;
  Cargo cargo(option);
  GRASP grasp;
  cargo.start(grasp);

  return 0;
}

