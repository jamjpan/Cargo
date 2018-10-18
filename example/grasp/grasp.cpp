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
const int MAX_ITER = 1;

GRASP::GRASP()
    : RSAlgorithm("grasp", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

Solution GRASP::initialize() {
  Solution sol_0 = {};

  vec_t<Customer> local_customers = this->customers();
  vec_t<MutableVehicleSptr> local_vehicles = {};

  // Get candidates list of vehicles and customers
  print << "\tBuilding candidates list" << std::endl;
  dict<VehlId, vec_t<Customer>> candidates_list = {};
  for (const Customer& cust : this->customers()) {
    vec_t<MutableVehicleSptr> cands = this->grid_.within(pickup_range(cust), cust.orig());
    for (MutableVehicleSptr& cand : cands) {
      if (candidates_list.count(cand->id()) == 0) {
        candidates_list[cand->id()] = {};
        local_vehicles.push_back(cand);
      }
      candidates_list.at(cand->id()).push_back(cust);
    }
  }

  auto rng = std::default_random_engine {};
  std::shuffle(std::begin(local_vehicles), std::end(local_vehicles), rng);

  print << "\tAssigning customers to vehicles" << std::endl;
  while (!local_vehicles.empty() && !local_customers.empty()) {
    // 1. Select random vehicle (we've shuffled the list)
    MutableVehicleSptr cand = local_vehicles.back();
    local_vehicles.pop_back();

    print << "\t\tSelected Vehl " << cand->id() << std::endl;

    dict<Customer, int> fitness = {};
    dict<Customer, vec_t<Stop>> schedule = {};
    dict<Customer, vec_t<Wayp>> route = {};

    while (true) {
      // 2. (Re)-compute fitness
      print << "\t\t\t(Re)-computing fitness" << std::endl;
      for (const Customer& cust : candidates_list.at(cand->id())) {
        vec_t<Stop> sch;
        vec_t<Wayp> rte;
        int score = sop_insert(*cand, cust, sch, rte) - cand->route().cost();
        fitness[cust] = score;
        schedule[cust] = sch;
        route[cust] = rte;
        print << "\t\t\tCust " << cust.id() << ": " << score << std::endl;
      }

      print << "\t\t\tRolling the roulette wheel" << std::endl;

      // 3. Roulette-select one customer to add to the vehicle
      Customer cust_to_add = roulette_select(fitness);
      vec_t<Stop> sch = schedule.at(cust_to_add);
      vec_t<Wayp> rte = route.at(cust_to_add);
      print << "\t\t\tConfirming cust " << cust_to_add.id() << std::endl;
      if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
        cand->set_sch(sch);
        cand->set_rte(rte);
        cand->reset_lvn();
        cand->incr_queued();
        print << "\t\t\tAdded cust " << cust_to_add.id() << std::endl;
        // Remove the customer from candidates list, local_customers
        for (auto& kv : candidates_list) {
          vec_t<Customer>& custs = kv.second;
          custs.erase(std::find(custs.begin(), custs.end(), cust_to_add));
        }
        local_customers.erase(std::find(local_customers.begin(), local_customers.end(), cust_to_add));
      } else {
        print << "\t\t\tInvalid." << std::endl;
        break;  // this vehicle is now done
      }
      // 4. Stop if no more candidates
      if (candidates_list.at(cand->id()).empty())
        break;
    }
  }

  print << "\tDone." << std::endl;
  return sol_0;
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
    float rankratio = 1 - (float)i->second/max;
    if (rankratio == 0 || rankratio > threshold)
      break;
  }
  return i->first;
}

void GRASP::print_sol(const Solution& sol) {
  for (const auto& kv : sol) {
    print << "Vehl " << kv.first->id() << std::endl;
    print << "\tAssigned to ";
    for (const Customer& cust : std::get<0>(kv.second))
      print << cust.id() << " ";
    print << std::endl;
    print << "\tUnassigned from ";
    for (const Customer& cust : std::get<1>(kv.second))
      print << cust.id() << " ";
    print << std::endl;
    print << "\tRoute: (omitted)" << std::endl;
    print << "\tSchedule: ";
    for (const Stop& stop : std::get<3>(kv.second))
      print << stop.loc() << " ";
    print << std::endl;
  }
}

void GRASP::match() {
  this->beg_ht();
  this->timeout_0 = hiclock::now();

  Solution best = {};
  for (int iter_count = 0; iter_count < MAX_ITER; ++iter_count) {
    print << "Intializing sol_0" << std::endl;
    Solution sol_0 = this->initialize();
    print_sol(sol_0);
    // - Solution sol_1 = replace(sol_0, ...);
    // - Solution sol_2 = swap(sol_0, ...);
    // - Solution sol_3 = rearrange(sol_0, ...);
    // - Solution sol_f = get_best(sol_replace, sol_swap, sol_rearrange);
    // - if (cost_of(sol_f) < cost_of(best)) {
    // -   best = sol_f;
    // - }
  }
  // - Commit best_sol;
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
  RSAlgorithm::listen(skip_assigned, skip_delayed);
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

