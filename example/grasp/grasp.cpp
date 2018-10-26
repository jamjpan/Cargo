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
  this->nswap_ = this->nreplace_ = this->nrearrange_ = this->nnoimprov_ = 0;
}

void GRASP::match() {
  this->beg_batch_ht();
  Solution best_solution = {};

  // Every solution gets its own grid and customers to make local changes.
  for (int count = 0; count < MAX_ITER; ++count) {
    Grid local_grid(this->grid_);  // deep copy
    vec_t<Customer> local_customers = this->customers();

    Solution initial_solution = this->initialize(local_grid, local_customers);
    best_solution = initial_solution;  // for testing
  }
  this->commit(best_solution);
  this->end_batch_ht();
}

Solution GRASP::initialize(Grid& grid, vec_t<Customer> customers) {
  Solution initial_solution = {};
  this->candidates_list = {};

  // 1. List all the candidates for each customer
  vec_t<MutableVehicleSptr> vehicles = {};
  for (const Customer& cust : customers) {
    vec_t<MutableVehicleSptr> candidates = grid.within(pickup_range(cust), cust.orig());
    for (const MutableVehicleSptr& cand : candidates) {
      vehicles.push_back(cand);
      if (this->candidates_list.count(cand) == 0)
        this->candidates_list[cand] = {};
      this->candidates_list[cand].push_back(cust);
    }
  }

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
      for (const Customer& cust : this->candidates_list.at(cand)) {
        this->fitness[cand][cust] = sop_insert(cand, cust, sch, rte);
        this->schedules[cand][cust] = sch;
        this->routes[cand][cust] = rte;
      }
      Customer cust_to_add = this->roulette(this->fitness.at(cand));
      print << "roulette returned " << cust_to_add.id() << std::endl;
      print << "all custs: ";
      for (const auto& kv : this->fitness.at(cand))
        print << kv.first.id() << std::endl;
      print << std::endl;
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
  }
  return initial_solution;
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

void GRASP::commit(const Solution& solution) {
  for (auto& kv : solution) {
    MutableVehicleSptr cand = kv.first;
    const vec_t<CustId>& cadd = kv.second.first;
    const vec_t<CustId>& cdel = kv.second.second;
    cand->reset_lvn();
    this->assign_or_delay(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand, false/*true*/);
    for (const CustId& cid : cadd) print << "Matched " << cid << " to vehl " << cand->id() << std::endl;
    for (const CustId& cid : cdel) {
      print << "Removed " << cid << " from vehl " << cand->id() << std::endl;
      this->beg_delay(cid);
    }
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

