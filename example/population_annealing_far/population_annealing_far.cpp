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
#include "population_annealing_far.h"

using namespace cargo;

const int BATCH = 30;
const int T_MAX = 5;
const int NSOL = 5;

PopulationAnnealingFar::PopulationAnnealingFar()
    : RSAlgorithm("population_annealing_far", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ndrops_ = 0;
  std::random_device rd;
  this->gen.seed(rd());
}

void PopulationAnnealingFar::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : customers()) {
    this->is_matched[cust.id()] = false;
    cand_used[cust.id()] = {};
  }

  DistInt best_solcst = InfInt;

  /* Generate initial solutions */
  print << "Initializing solution" << std::endl;
  for (int n_sol = 0; n_sol < NSOL; ++n_sol) {
    vec_t<std::tuple<Customer, MutableVehicle, DistInt>> sol = {};
    Grid lcl_grid(this->grid_); // make a local copy
    for (const Customer& cust : customers()) {
      bool initial = false;
      this->candidates = lcl_grid.within(pickup_range(cust), cust.orig());
      // Clear the memory if all candidates have been used up
      if (cand_used.at(cust.id()).size() == this->candidates.size())
        cand_used.at(cust.id()).clear();
      // Randomize access order
      std::random_shuffle(this->candidates.begin(), this->candidates.end());
      while (!this->candidates.empty() && initial == false) {
        MutableVehicleSptr& cand = this->candidates.back();
        candidates.pop_back();
        bool is_used = (std::find(cand_used.at(cust.id()).begin(), cand_used.at(cust.id()).end(), cand->id()) != cand_used.at(cust.id()).end());
        // Speed-up heuristic!
        // Try only if vehicle's current schedule len < 8 customer stops
        if (cand->schedule().data().size() < 10 && !is_used) {
          DistInt cst = sop_insert(*cand, cust, sch, rte) - cand->route().cost();
          if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
            cand->set_sch(sch);
            cand->set_rte(rte);
            cand->reset_lvn();
            cand->incr_queued();
            std::tuple<Customer, MutableVehicle, DistInt>
              assignment(cust, *cand, cst);
            sol.push_back(assignment);
            cand_used.at(cust.id()).push_back(cand->id());
            initial = true;
            print << "Initial match " << cust.id() << " with " << cand->id()
                  << "; cost: " << cst << std::endl;
          }
        }
      }
    }

    /* AT THIS POINT, vehicles in lcl_grid are different from grid_ because
     * candidates have different routes/schedules now. */

    if (sol.empty()) {
      print << "Empty sol; returning." << std::endl;
      return;
    }

    /* Perturb the solution */
    std::uniform_int_distribution<> n(0, sol.size() - 1);
    for (int T = 1; T <= T_MAX; ++T) {
      for (int i = 0; i < 200*T*T; ++i) {  // dynamic # of perts
        auto lcl_sol = sol;
        auto cust_itr = lcl_sol.begin();          // pick random customer
        std::advance(cust_itr, n(this->gen));
        Customer& cust = std::get<0>(*cust_itr);
        print << "Perturbing " << cust.id() << std::endl;

        auto candidates = lcl_grid.within(pickup_range(cust), cust.orig());
        if (!candidates.empty()) {
          std::uniform_int_distribution<> m(0, candidates.size() - 1);
          auto vehl_itr = candidates.begin(); // pick random candidate
          std::advance(vehl_itr, m(gen));
          auto cand = *vehl_itr;

          bool is_same = (cand->id() == std::get<1>(*cust_itr).id());
          if (!is_same && cand->schedule().data().size() < 10) {
          DistInt new_cst = sop_insert(*cand, cust, sch, rte);
          DistInt cst = new_cst - cand->route().cost();
          if (cst < 0) {
            print(MessageType::Error) << "Got negative detour!" << std::endl;
            print << cand->id() << std::endl;
            print << cst << " (" << new_cst << "-" << cand->route().cost() << ")" << std::endl;
            print << "Current schedule: ";
            for (const Stop& sp : cand->schedule().data())
              print << sp.loc() << " ";
            print << std::endl;
            print << "nnd: " << cand->next_node_distance() << std::endl;
            print << "New schedule: ";
            for (const Stop& sp : sch)
              print << sp.loc() << " ";
            print << std::endl;
            throw;
          }
          if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
              bool climb = false;
              bool accept = false;
              print << "\tOld cost: " << std::get<2>(*cust_itr) << "; new cost: " << cst << std::endl;
              if (cst < std::get<2>(*cust_itr)) {
                accept = true;
                this->ndrops_++;
              } else if (hillclimb(T) && T != T_MAX) {
                accept = true;
                climb = true;
                this->nclimbs_++;
              }
              if (accept) {
                cand->set_rte(rte);
                cand->set_sch(sch);
                cand->reset_lvn();
                cand->incr_queued();

                auto sptr = lcl_grid.select(std::get<1>(*cust_itr).id());
                if (sptr == nullptr) {
                  print(MessageType::Error)
                    << "Could not find vehicle in local grid" << std::endl;
                  throw;
                }
                auto less_sch = sptr->schedule().data();
                opdel(less_sch, cust.id());
                std::vector<Wayp> less_rte {};
                route_through(less_sch, less_rte);
                sptr->set_sch(less_sch);
                sptr->set_rte(less_rte);
                sptr->reset_lvn();
                sptr->decr_queued();

                /* Store the new solution */
                vec_t<CustId> assigned = {};
                std::get<1>(*cust_itr) = *cand; // modify sol with new cand
                std::get<2>(*cust_itr) = cst;   // modify sol with new cost
                for (auto &assignment : lcl_sol) {
                  if (std::get<1>(assignment).id() == sptr->id())
                    std::get<1>(assignment) = *sptr;  // modify old assignment
                  if (std::get<1>(assignment).id() == cand->id())
                    std::get<1>(assignment) = *cand;  // modify old assignment
                }
                sol = lcl_sol;
                print << "\tN=" << n_sol << "; T=" << T << "; P=" << i << "; Moved " << cust.id() << " to " << cand->id()
                      << "; new cost: " << cst << (climb ? " (climb)" : "") << std::endl;
                print << "New schedule for " << cand->id() << ":" << std::endl;
                for (const Stop& stop : sch)
                  print << stop.loc() << " ";
                print << std::endl;
                print << "New schedule for " << sptr->id() << ":" << std::endl;
                for (const Stop& stop : less_sch)
                  print << stop.loc() << " ";
                print << std::endl;
              }
            }
          }
        }
        if (this->timeout(this->timeout_0)) {
          break;
        }
      } // end perturbation
      if (this->timeout(this->timeout_0)) {
        break;
      }
    } // end temperature

    /* AT THIS POINT, sol has finished local search. Accept it as best_sol
     * if it beats the others. */
    DistInt solcst = 0;
    vec_t<CustId> assigned = {};
    for (const auto& assignment : sol) {
      solcst += std::get<2>(assignment);
      assigned.push_back(std::get<0>(assignment).id());
    }
    for (const Customer& cust : this->customers())
      if (std::find(assigned.begin(), assigned.end(), cust.id()) == assigned.end())
        solcst += Cargo::basecost(cust.id());

    print << "Solution cost: " << solcst << "; incumbent: " << best_solcst << std::endl;
    if (solcst < best_solcst) {
      print << "Better solution found (" << solcst << " < " << best_solcst << ")" << std::endl;
      best_sol = sol;
      best_solcst = solcst;
    }
  } // end all solutions

  /* Commit the solution */
  for (const auto& assignment : this->best_sol) {
    auto& vehl_id = std::get<1>(assignment).id();
    commit_cadd[vehl_id] = {};
    if (commit_rte.count(vehl_id) == 0)
      commit_rte[vehl_id] = std::get<1>(assignment).route().data();
    if (commit_sch.count(vehl_id) == 0)
      commit_sch[vehl_id] = std::get<1>(assignment).schedule().data();
  }
  for (const auto& assignment : this->best_sol) {
    auto& cust = std::get<0>(assignment);
    auto& cand_id = std::get<1>(assignment).id();
    commit_cadd[cand_id].push_back(cust);
  }
  for (const auto& kv : commit_cadd) {
    auto& custs = kv.second;
    auto& cand_id = kv.first;
    auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
            [&](const Vehicle& a){ return a.id() == cand_id; }));
    std::vector<CustId> custs_to_add = {};
    for (const auto& cust : custs) {
      custs_to_add.push_back(cust.id());
      print << "Matched " << cust.id() << " with " << cand_id << std::endl;
    }
    if (assign(custs_to_add, {}, commit_rte.at(cand_id), commit_sch.at(cand_id), cand)) {
      for (const auto& cust_id : custs_to_add) {
        is_matched.at(cust_id) = true;
        this->end_delay(cust_id);
      }
    } else {
      for (size_t i = 0; i < custs_to_add.size(); ++i) {
        this->beg_delay(custs_to_add.at(i));
      }
    }
  }

  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  this->end_ht();
}

void PopulationAnnealingFar::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void PopulationAnnealingFar::end() {
  print(MessageType::Info)
    << "climbs: " << nclimbs_ << '\n'
    << "descents: " << ndrops_ << std::endl;
  this->print_statistics();
}

void PopulationAnnealingFar::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

bool PopulationAnnealingFar::hillclimb(int& T) {
  return this->d(this->gen) < std::exp(-1.2*(float)T);
}

void PopulationAnnealingFar::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->timeout_0 = hiclock::now();
  this->timeout_ = BATCH*1000;
  this->is_matched = {};
  this->cand_used = {};
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
  option.path_to_solution = "population_annealing_far.sol";
  option.path_to_dataout  = "population_annealing_far.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode = true;
  Cargo cargo(option);
  PopulationAnnealingFar paf;
  cargo.start(paf);

  return 0;
}

