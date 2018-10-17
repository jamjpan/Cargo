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

GRASP::GRASP()
    : RSAlgorithm("grasp", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

void GRASP::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : this->customers())
    this->is_matched[cust.id()] = false;

  // EVERYTHING below here needs to be wrapped up in a loop

  /* Generate an initial solution
   * ---------------------------- */
  print << "Initializing solution" << std::endl;
  Grid lcl_grid(this->grid_);                               // local grid
  vec_t<Customer> lcl_custs = this->customers();            // local customers
  dict<VehlId, vec_t<Customer>> lcl_sol = {};               // local solution
  dict<VehlId, vec_t<Customer>> lcl_removed = {};           // removed assignments
  dict<MutableVehicleSptr, vec_t<Customer>> lcl_cands = {}; // local cands table

  for (const Customer& cust : lcl_custs) {
    vec_t<MutableVehicleSptr> cands = lcl_grid.within(pickup_range(cust), cust.orig());
    for (const MutableVehicleSptr& cand : cands) {
      if (lcl_cands.count(cand) == 0) lcl_cands[cand] = {};
      lcl_cands.at(cand).push_back(cust);
    }
  }

  initialize(lcl_custs, lcl_cands, lcl_sol);

  // Vehicles in lcl_grid are now modified with new routes and schedules
  // corresponding to the assignments in lcl_sol
  print << "Sol initialized (" << lcl_sol.size() << ")" << std::endl;


  /* Commit Solution
   * --------------- */


  /* Improve
   * ------- */
  print << "Improving..." << std::endl;

  // 1. Replace an assigned with an unassigned from some random vehicle.
  print << "\tReplace" << std::endl;
  DistInt replace_improvement = 0;
  vec_t<Stop> replace_sch;
  vec_t<Wayp> replace_rte;
  CustId replace_me = -1;
  CustId replace_by = -1;
  MutableVehicleSptr cand_for_replace =
      this->replace(solution, unassigned, replace_improvement, replace_sch,
                    replace_rte, replace_me, replace_by);
  print << "\t\tVehl " << cand_for_replace->id() << " replace " << replace_me
        << " with " << replace_by << " gets " << replace_improvement
        << std::endl;

  // 2. Swap assignments from two random vehicles.
  print << "\tSwap" << std::endl;
  DistInt swap_improvement = 0;
  vec_t<Stop> sch1, sch2;
  vec_t<Wayp> rte1, rte2;
  CustId a, b;
  auto swaps =
      this->swap(solution, swap_improvement, sch1, rte1, sch2, rte2, b, a);
  print << "\t\tSwap improvement: " << swap_improvement << std::endl;

  // 3. Rearrange two stops from a random vehicle.
  print << "\tRearrange" << std::endl;
  DistInt rearrange_improvement = 0;
  vec_t<Stop> rearrange_sch;
  vec_t<Wayp> rearrange_rte;
  MutableVehicleSptr cand_for_rearrange = this->rearrange(
      solution, rearrange_improvement, rearrange_sch, rearrange_rte);
  print << "\t\tRearrange improvement: " << rearrange_improvement << std::endl;

  int max_improvement = std::max(replace_improvement, std::max(swap_improvement, rearrange_improvement));
  if (max_improvement > 0) {
    if (replace_improvement == max_improvement) {
    vec_t<Stop> replace_sch;
    vec_t<Wayp> replace_rte;
    CustId replace_me = -1;
    CustId replace_by = -1;
      cand_for_replace->set_sch(replace_sch);
      cand_for_replace->set_rte(replace_rte);
      unassigned.erase(std::remove_if(unassigned.begin(), unassigned.end(), [&](const Customer& cust){ return cust.id() == replace_me; }), unassigned.end());
      //unassigned.push_back(replace_me);


    } else if (swap_improvement == max_improvement) {

    } else if (rearrange_improvement == max_improvement) {

    }
  }

  /* Commit the solution */
  // for (const auto& assignment : this->best_sol) {
  //   auto& vehl_id = std::get<1>(assignment).id();
  //   commit_cadd[vehl_id] = {};
  //   if (commit_rte.count(vehl_id) == 0)
  //     commit_rte[vehl_id] = std::get<1>(assignment).route().data();
  //   if (commit_sch.count(vehl_id) == 0)
  //     commit_sch[vehl_id] = std::get<1>(assignment).schedule().data();
  // }
  // for (const auto& assignment : this->best_sol) {
  //   auto& cust = std::get<0>(assignment);
  //   auto& cand_id = std::get<1>(assignment).id();
  //   commit_cadd[cand_id].push_back(cust);
  // }
  // for (const auto& kv : commit_cadd) {
  //   auto& custs = kv.second;
  //   auto& cand_id = kv.first;
  //   auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
  //           [&](const Vehicle& a){ return a.id() == cand_id; }));
  //   std::vector<CustId> custs_to_add = {};
  //   for (const auto& cust : custs)
  //     custs_to_add.push_back(cust.id());
  //   if (assign(custs_to_add, {}, commit_rte.at(cand_id), commit_sch.at(cand_id), cand)) {
  //     for (const auto& cust_id : custs_to_add) {
  //       is_matched.at(cust_id) = true;
  //       this->end_delay(cust_id);
  //     }
  //   } else {
  //     for (size_t i = 0; i < custs_to_add.size(); ++i) {
  //       this->beg_delay(custs_to_add.at(i));
  //     }
  //   }
  // }

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

void GRASP::initialize(vec_t<Customer>& lcl_custs,
                       dict<MutableVehicleSptr, vec_t<Customer>>& lcl_cands,
                       dict<VehlId, vec_t<Customer>>& lcl_sol) {
  // Loop through the vehicles; for each vehicle, list all the
  // new route costs of inserting each customer into the vehicle.
  for (auto& kv : lcl_cands) {
    // Extract key-value pair
    MutableVehicleSptr     cand  = kv.first;  // points to mv in lcl_grid
    const vec_t<Customer>& custs = kv.second;
    print << "\tSelect vehl " << cand->id() << " (" << custs.size() << " custs)" << std::endl;

    // Initialize rank list, sch, rte containers
    vec_t<std::pair<DistInt, Customer>> init_rank;
    dict<CustId, vec_t<Stop>>           new_sch;
    dict<CustId, vec_t<Wayp>>           new_rte;

    // Initially rank each candidate customer
    for (const Customer& cust : custs) {
      vec_t<Stop> sch;
      vec_t<Wayp> rte;
      DistInt cost =
        sop_insert(*cand, cust, sch, rte) - cand->route().cost(); // + cand->next_node_distance();
      init_rank.push_back({cost, cust});
      new_sch[cust.id()] = sch;
      new_rte[cust.id()] = rte;
    }
    if (init_rank.size() == 0) {
      print << "\tNo ranks; skip" << std::endl;
      continue;
    }
    // Do until no more assignments are possible
    auto rank = init_rank;
    while (rank.size() > 0) {
      // Select and assign one ranked customer
      print << "\tSelecting customer to assign..." << std::endl;
      DistInt max = max_rank(rank);
      std::uniform_int_distribution<>  n(0, rank.size() - 1);
      std::uniform_real_distribution<> m(0, 1);
      size_t it = 0;
      if (rank.size() > 1) {
        // Roulette-wheel selection
        // (see https://en.wikipedia.org/wiki/Fitness_proportionate_selection)
        while (true) {
          it = n(this->gen);
          float threshold = m(this->gen);
          float rankratio = 1-(float)rank.at(it).first/max;  // smaller ranks are better
          if (rankratio == 0 || rankratio > threshold)
            break;
        }
      }
      const Customer& cadd = rank.at(it).second;
      bool pass_cap = chkcap(cand->capacity(), new_sch.at(cadd.id()));
      bool pass_tw  = chktw(new_sch.at(cadd.id()), new_rte.at(cadd.id()));
      if (pass_cap && pass_tw) {
        cand->set_sch(new_sch.at(cadd.id()));
        cand->set_rte(new_rte.at(cadd.id()));
        cand->reset_lvn();
        cand->incr_queued();
        if (solution.count(cand) == 0)
          solution[cand] = {};
        solution.at(cand).push_back(cadd);
        assigned.push_back(cadd.id());
        print << "\tAssigned " << cadd.id() << " to " << cand->id() << std::endl;
      }
      rank.erase(rank.begin()+it);

      // Re-rank the remaining customers
      for (const auto& kv : rank) {
        const Customer cust = kv.second;
        // print << "\t\tRanking cust " << cust.id() << std::endl;
        bool isAssigned = (std::find(assigned.begin(), assigned.end(), cust.id()) != assigned.end());
        if (!isAssigned) {
          vec_t<Stop> sch;
          vec_t<Wayp> rte;
          DistInt cost =
            sop_insert(*cand, cust, sch, rte) - cand->route().cost(); // + cand->next_node_distance();
          init_rank.push_back({cost, cust});
          new_sch[cust.id()] = sch;
          new_rte[cust.id()] = rte;
          // print << "\t\t\tRanked (" << cost << ")" << std::endl;
        }
      }
    }
  }
}

MutableVehicleSptr GRASP::replace(
    const dict<MutableVehicleSptr, vec_t<Customer>>& solution,
    const vec_t<Customer>& unassigned,
    DistInt& improvement,
    vec_t<Stop>& replace_sch,
    vec_t<Wayp>& replace_rte,
    CustId& replaced_cust,
    CustId& now_assigned_cust) {
  if (unassigned.size() == 0) {
    print << "\t\tNothing to replace" << std::endl;
    improvement = 0;
    return nullptr;
  }

  // Select random vehicle from the solution
  std::uniform_int_distribution<> p(0, solution.size() - 1);
  auto j = solution.begin();
  std::advance(j, p(gen));

  // Select random unassigned customer
  std::uniform_int_distribution<> q(0, unassigned.size()-1);
  auto k = unassigned.begin();
  std::advance(k, q(gen));

  // Replace random assigned customer with *k
  MutableVehicleSptr cand_for_replace = j->first;
  improvement = 0;
  CustId remove_me = randcust(cand_for_replace->schedule().data());
  if (remove_me != -1) {
    vec_t<Stop> old_sch = cand_for_replace->schedule().data();
    vec_t<Stop> sch;
    vec_t<Wayp> rte;
    sop_replace(cand_for_replace, remove_me, *k, sch, rte);
    improvement = cand_for_replace->route().cost() - rte.back().first;
    if (improvement > 0 && chkcap(cand_for_replace->capacity(), sch)
     && chktw(sch, rte)) {
      replace_sch = sch;
      replace_rte = rte;
      replaced_cust = remove_me;
      now_assigned_cust = k->id();
      print << "\t\tReplaced " << remove_me << " with " << k->id()
            << " (" << rte.back().first << " < " << cand_for_replace->route().cost() << ")" << std::endl;
    } else {
      if (improvement <= 0)
        print << "\t\tReplace not improving ("
              << rte.back().first << " < " << cand_for_replace->route().cost() << ")" << std::endl;
      else
        print << "\t\tReplace " << remove_me << " not feasible" << std::endl;
    }
  } else {
    print << "\t\tCould not find a customer to remove." << std::endl;
  }
  return cand_for_replace;
}

std::pair<MutableVehicleSptr, MutableVehicleSptr> GRASP::swap(
    const dict<MutableVehicleSptr, vec_t<Customer>>& solution,
    DistInt& improvement,
    vec_t<Stop>& swap_sch1,
    vec_t<Wayp>& swap_rte1,
    vec_t<Stop>& swap_sch2,
    vec_t<Wayp>& swap_rte2,
    CustId& swapped_to_1,
    CustId& swapped_to_2) {
  // Select two random vehicles from the solution
  std::uniform_int_distribution<> p(0, solution.size() - 1);
  auto j = solution.begin();
  auto k = solution.begin();
  std::advance(j, p(gen));
  do {
    k = solution.begin();
    std::advance(k, p(gen));
  } while (k == j);

  // Select customers to swap
  std::uniform_int_distribution<> q(0, j->second.size()-1);
  std::uniform_int_distribution<> r(0, k->second.size()-1);
  auto a = j->second.begin();
  auto b = k->second.begin();
  std::advance(a, q(gen));
  std::advance(b, r(gen));

  // Swap the customers
  MutableVehicleSptr cand1 = j->first;
  MutableVehicleSptr cand2 = k->first;
  improvement = 0;
  vec_t<Stop> sch1, sch2;
  vec_t<Wayp> rte1, rte2;
  sop_replace(cand1, a->id(), *b, sch1, rte1);
  sop_replace(cand2, b->id(), *a, sch2, rte2);
  improvement = cand1->route().cost() - rte1.back().first;
  improvement+= cand2->route().cost() - rte2.back().first;
  if (improvement > 0 && chkcap(cand1->capacity(), sch1) && chkcap(cand2->capacity(), sch2)
   && chktw(sch1, rte1) && chktw(sch2, rte2)) {
    swap_sch1 = sch1;
    swap_rte1 = rte1;
    swap_sch2 = sch2;
    swap_rte2 = rte2;
    swapped_to_1 = b->id();
    swapped_to_2 = a->id();
    print << "\t\tSwapped " << a->id() << ", " << b->id() << " from vehls " << cand1->id() << ", " << cand2->id() << std::endl;
  } else {
    if (improvement <= 0)
      print << "\t\tSwap not improving (" << cand1->route().cost() << " - " << rte1.back().first << "; "
            << cand2->route().cost() << " - " << rte2.back().first << "; " << improvement << ")" << std::endl;
    else
      print << "\t\tSwap " << a->id() << ", " << b->id() << " not feasible." << std::endl;
  }
  return {cand1, cand2};
}

MutableVehicleSptr GRASP::rearrange(
    const dict<MutableVehicleSptr, vec_t<Customer>>& solution,
    DistInt& improvement,
    vec_t<Stop>& sch,
    vec_t<Wayp>& rte) {
  // Select random vehicle from the solution
  std::uniform_int_distribution<> p(0, solution.size() - 1);
  auto j = solution.begin();
  std::advance(j, p(gen));

  MutableVehicleSptr cand = j->first;

  if (cand->schedule().size() < 4) {
    improvement = 0;
    print << "\t\tRearrange not feasible (schedule < 4 stops)" << std::endl;
  } else {
    // Select random stop
    std::uniform_int_distribution<> n(1, cand->schedule().size() - 2);
    sch = cand->schedule().data();
    auto i = sch.begin();
    std::advance(i, n(gen));
    std::iter_swap(i,i+1);
    if (!chkpc(sch)) {
      print << "\t\tRearrange not feasible (new schedule fails precedence)" << std::endl;
    } else {
      route_through(sch, rte);
      improvement = rte.back().first - cand->route().cost();
      if (improvement > 0 && chkcap(cand->capacity(), sch)
       && chktw(sch, rte)) {
      } else {
        print << "\t\tRearrange not improving or not feasible" << std::endl;
      }
    }
  }
  return cand;
}

DistInt GRASP::max_rank(const vec_t<std::pair<DistInt, Customer>>& rank) {
  DistInt max = 0;
  for (const auto& rc : rank)
    if (rc.first > max)
      max = rc.first;
  return max;
}

DistInt GRASP::max_rank(const vec_t<std::pair<DistInt, MutableVehicleSptr>>& rank) {
  DistInt max = 0;
  for (const auto& rc : rank)
    if (rc.first > max)
      max = rc.first;
  return max;
}

void GRASP::reset_workspace() {
  this->best_solcst = 0;
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->timeout_0 = hiclock::now();
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
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode = true;
  Cargo cargo(option);
  GRASP grasp;
  cargo.start(grasp);

  return 0;
}

