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
const int T_MAX = 5;
const int P_MAX = 100000;

SimulatedAnnealing::SimulatedAnnealing()
    : RSAlgorithm("simulated_annealing", true), grid_(100), d(0,1) {
//    : RSAlgorithm("sa5-100000", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ndrops_ = 0;
  std::random_device rd;
  this->gen.seed(rd());
}

void SimulatedAnnealing::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : customers())
    this->is_matched[cust.id()] = false;

  /* Generate an initial solution */
  print << "Initializing solution" << std::endl;
  Grid local_grid(this->grid_);  // make a local copy
  Solution sol = this->initialize(local_grid);

  /* AT THIS POINT, vehicles in lcl_grid are different from grid_ because
   * candidates have different routes/schedules now. */

  if (sol.empty()) return;

  /* Perturb the solution */
  for (int temperature = 1; temperature <= T_MAX; ++temperature) {
    for (int perturbation = 1; perturbation <= P_MAX; ++perturbation) {
      print << "\tPerturb(T=" << temperature << ";P=" << perturbation << ")" << std::endl;
      sol = this->perturb(sol, temperature);
      if (this->timeout(this->timeout_0))
        break;
    }
  }

  this->commit(sol);
  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  this->end_ht();
}

Solution SimulatedAnnealing::initialize(Grid& local_grid) {
  Solution sol = {};

  // Assign each customer to a random candidate
  for (const Customer& cust : customers()) {
    bool initial = false;
    this->candidates_list[cust.id()] = local_grid.within(pickup_range(cust), cust.orig());

    // Add to local vehicle lookup (we need it during perturb)
    for (const MutableVehicleSptr cand : this->candidates_list.at(cust.id()))
      this->vehicle_lookup[cand->id()] = cand;

    // Randomize the candidates order
    vec_t<MutableVehicleSptr>& candidates = this->candidates_list.at(cust.id());
    std::shuffle(candidates.begin(), candidates.end(), this->gen);

    // Try until got a valid one
    while (!candidates.empty() && initial == false) {
      MutableVehicleSptr& cand = candidates.back();
      candidates.pop_back();
      // Speed-up heuristic!
      // Try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < 10) {
        vec_t<Stop> schedule;
        vec_t<Wayp> route;
        DistInt cst = sop_insert(*cand, cust, schedule, route) - cand->route().cost();
        if (chkcap(cand->capacity(), schedule) && chktw(schedule, route)) {
          cand->set_sch(schedule);  // update grid version of the candidate
          cand->set_rte(route);
          cand->reset_lvn();
          cand->incr_queued();
          if (sol.count(cand->id()) == 0) {
            vec_t<Customer> assignments = {};
            sol[cand->id()] = std::make_pair(*cand, assignments);
          }
          sol.at(cand->id()).first = *cand;  // update our copy of the candidate
          sol.at(cand->id()).second.push_back(cust);
          initial = true;
          print << "Initial match " << cust.id() << " with " << cand->id() << "; cost: " << cst << std::endl;
        }
      }
    }
    if (!initial)
      this->beg_delay(cust.id());
  }
  return sol;
}

Solution SimulatedAnnealing::perturb(const Solution& sol, const Temperature& temperature) {
  // 1. Random-select a vehicle from sol
  auto i = sol.begin();
  if (sol.size() > 1) {
    std::uniform_int_distribution<> n(0, sol.size() - 1);
    std::advance(i, n(gen));
  }
  MutableVehicle k_old = i->second.first;

  // 2. Random-select an assigned customer
  vec_t<Customer> k_old_assignments = i->second.second;
  if (k_old_assignments.size() == 0) {
    print << "\t\tPerturb found no assignments on vehl " << k_old.id() << std::endl;
    return sol;
  }
  auto j = k_old_assignments.begin();
  if (k_old_assignments.size() > 1) {
    std::uniform_int_distribution<> m(0, k_old_assignments.size() - 1);
    std::advance(j, m(gen));
  }
  Customer cust_to_move = *j;

  // 3. Random-select new candidate for the customer
  vec_t<MutableVehicleSptr>& candidates = this->candidates_list.at(cust_to_move.id());
  if (candidates.size() == 0) {
    print << "\t\tPerturb found no candidates for cust " << cust_to_move.id() << std::endl;
    return sol;
  }
  auto k = candidates.begin();
  if (candidates.size() > 1) {
    std::uniform_int_distribution<> p(0, candidates.size() - 1);
    std::advance(k, p(gen));
  }
  while ((*k)->id() == k_old.id() && k != candidates.end())
    k++;
  if (k == candidates.end()) {
    print << "\t\tPerturb found no eligible candidates for cust " << cust_to_move.id() << std::endl;
    return sol;
  }
  MutableVehicle k_new = **k;
  vec_t<Customer> k_new_assignments = {};
  if (sol.count(k_new.id()) == 1)
    k_new_assignments = sol.at(k_new.id()).second;

  // 4. Compute the current cost. Do the move, then recompute the cost.
  DistInt current_cost = k_old.route().cost() + k_new.route().cost();
  //   a. Remove cust from k_old
  vec_t<Stop> schedule_after_remove = k_old.schedule().data();
  vec_t<Wayp> route_after_remove = {};
  opdel(schedule_after_remove, cust_to_move.id());
  route_through(schedule_after_remove, route_after_remove);
  //   b. Add cust to k_new
  vec_t<Stop> schedule_after_add = {};
  vec_t<Wayp> route_after_add = {};
  DistInt cost_after_add = sop_insert(k_new, cust_to_move, schedule_after_add, route_after_add);
  //   c. (Verify)
  if (cost_after_add < k_new.route().cost()) {
    print(MessageType::Error) << "Got negative detour!!!" << std::endl;
    throw;
  }
  if (chkcap(k_new.capacity(), schedule_after_add) && chktw(schedule_after_add, route_after_add)) {
    DistInt new_cost = route_after_add.back().first + route_after_remove.back().first;
    bool climb = false;
    if (new_cost >= current_cost) {
      climb = hillclimb(temperature);
      if (climb) this->nclimbs_++;
    }
    if (new_cost < current_cost || climb) {
      print << "\t\tPerturbed " << cust_to_move.id() << " from " << k_old.id() << " to " << k_new.id()
            << " (old cost: " << current_cost << "; new cost: " << new_cost << ") " << (climb ? " (climb) " : "") << std::endl;
      print << "\t\tk_old current sch: ";
      for (const Stop& stop : k_old.schedule().data())
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
      print << "\t\tk_old new sch: ";
      for (const Stop& stop : schedule_after_remove)
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
      print << "\t\tk_new old sch: ";
      for (const Stop& stop : k_new.schedule().data())
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
      print << "\t\tk_new new sch: ";
      for (const Stop& stop : schedule_after_add)
        print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
      print << std::endl;
      // THERE'S PROBABLY A BETTER WAY TO DO THIS
      // Update grid
      vehicle_lookup.at(k_old.id())->set_sch(schedule_after_remove);
      vehicle_lookup.at(k_old.id())->set_rte(route_after_remove);
      vehicle_lookup.at(k_old.id())->reset_lvn();
      vehicle_lookup.at(k_old.id())->decr_queued();
      vehicle_lookup.at(k_new.id())->set_sch(schedule_after_add);
      vehicle_lookup.at(k_new.id())->set_rte(route_after_add);
      vehicle_lookup.at(k_new.id())->reset_lvn();
      vehicle_lookup.at(k_new.id())->incr_queued();

      // Update solution
      k_old.set_sch(schedule_after_remove);
      k_old.set_rte(route_after_remove);
      k_old.reset_lvn();
      k_old.decr_queued();
      k_old_assignments.erase(j);
      k_new.set_sch(schedule_after_add);
      k_new.set_rte(route_after_add);
      k_new.reset_lvn();
      k_new.incr_queued();
      k_new_assignments.push_back(cust_to_move);
      Solution improved_sol = sol;
      improved_sol[k_old.id()] = std::make_pair(k_old, k_old_assignments);
      improved_sol[k_new.id()] = std::make_pair(k_new, k_new_assignments);
      return improved_sol;
    } else {
      print << "\t\tPerturb failed " << cust_to_move.id() << " from " << k_old.id() << " to " << k_new.id()
            << " (old cost: " << current_cost << "; new cost: " << new_cost << ") " << (climb ? " (climb) " : "") << std::endl;
      return sol;
    }
  } else {
    print << "\t\tPerturb infeasible" << std::endl;
    return sol;
  }
}

void SimulatedAnnealing::commit(const Solution& sol) {
  for (const auto& kv : sol) {
    MutableVehicle cand = kv.second.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second.second) {
      cadd.push_back(cust.id());
      this->is_matched[cust.id()] = true;
      print << "Matched " << cust.id() << " with " << cand.id() << std::endl;
    }
    // print << "\tCommit schedule for vehl " << cand.id() << ": ";
    // for (const Stop& stop : cand.schedule().data())
    //   print << "(" << stop.owner() << "|" << stop.loc() << "|" << (int)stop.type() << ") ";
    // print << std::endl;
    this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand, false /*true*/);
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
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

bool SimulatedAnnealing::hillclimb(const int& T) {
  return this->d(this->gen) < std::exp(-1.2*(float)T);
}

void SimulatedAnnealing::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates_list = {};
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
  option.path_to_solution = "simulated_annealing.sol";
  option.path_to_dataout  = "simulated_annealing.dat";
//  option.path_to_solution = "sa5-100000.sol";
//  option.path_to_dataout  = "sa5-100000.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode = true;
  Cargo cargo(option);
  SimulatedAnnealing sa;
  cargo.start(sa);

  return 0;
}

