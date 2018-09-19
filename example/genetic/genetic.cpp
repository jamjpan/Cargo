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
#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include "genetic.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;
const int RANGE = 2000;
const int MAX_GENERATIONS = 1000;
const int MAX_CHROMOSOMES = 10;

Genetic::Genetic()
    : RSAlgorithm("genetic", false), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  std::random_device rd;
  this->gen.seed(rd());
}

void Genetic::match() {
  this->beg_ht();
  this->reset_workspace();
  for (const Customer& cust : customers())
    is_matched[cust.id()] = false;

  Population pop = {};

  // print << "Generating initial population." << std::endl;

  /* Generate initial populations (solutions) */
  for (int j = 0; j < MAX_CHROMOSOMES; ++j) {
    std::random_shuffle(customers().begin(), customers().end());
    std::set<MutableVehicleSptr> matches = {};
    Grid lcl_grid(this->grid_); // make a local copy
    for (const Customer& cust : customers()) {
      bool initial = false;
      this->candidates = lcl_grid.within(RANGE, cust.orig());
      while (!this->candidates.empty() && initial == false) {
        MutableVehicleSptr& cand = this->candidates.back();
        candidates.pop_back();
        if (cand->queued() < cand->capacity()) {
          sop_insert(*cand, cust, sch, rte);
          if (chktw(sch, rte)) {
            cand->set_sch(sch);
            cand->set_rte(rte);
            cand->reset_lvn();
            cand->incr_queued();
            matches.insert(cand);
            initial = true;
          }
        }
      }
    }
    Assignment a = {};
    for (const MutableVehicleSptr& i : matches)
      a.push_back(*i);
    if (!a.empty()) {
      DistInt cst = cost(a);
      std::pair<DistInt, Assignment> sol(cst, a);
      pop.push_back(sol);
    }
  }

  // print << "Generated initial population." << std::endl;
  // print_population(pop);

  if (pop.empty()) {
    // print << "Empty population." << std::endl;
    return;
  }

  if (pop.size() == 1) {
    // print << "Population size 1." << std::endl;
    /* Assign the one solution */
    return;
  }

  for (int generations = 0; generations < MAX_GENERATIONS; ++generations) {
    // print << "Generation " << generations << std::endl;
    /* Generate an offspring */
    Population offspring_pop = {};
    while (offspring_pop.size() < MAX_CHROMOSOMES-1) {
      Assignment A = extract_fit(pop);
      Assignment B = extract_fit(pop);
      // print << "\tBefore crossover:" << std::endl;
      // print_solution({cost(A),A});
      // print_solution({cost(B),B});
      crossover(A, B);  // now A,B have been modified
      // print << "\tBefore repair:" << std::endl;
      // print_solution({cost(A),A});
      // print_solution({cost(B),B});
      repair(A);
      repair(B);
      // print << "\tAfter repair:" << std::endl;
      // print_solution({cost(A),A});
      // print_solution({cost(B),B});
      offspring_pop.push_back({cost(A),A});
      offspring_pop.push_back({cost(B),B});
      // print << "Offspring: " << offspring_pop.size() << ", Max: " << MAX_CHROMOSOMES << std::endl;
    }
    pop = offspring_pop;
    // print << "Population set to new generation." << std::endl;
    if (this->timeout(this->timeout_0)) {
      break;
    }
  }

  // print << "Finished breeding." << std::endl;

  /* Select fittest offspring and commit */
  Assignment best_a = extract_fit(pop);

  // print << "Extracted fittest." << std::endl;

  for (const MutableVehicle& mutvehl : best_a) {
    this->commit_cadd[mutvehl.id()] = {};
    this->commit_rte[mutvehl.id()] = mutvehl.route().data();
    this->commit_sch[mutvehl.id()] = mutvehl.schedule().data();
    for (const Stop& stop : mutvehl.schedule().data())
      if (stop.type() == StopType::CustDest && is_matched.count(stop.owner()) == 1)
        this->commit_cadd.at(mutvehl.id()).push_back(stop.owner());
  }

  // print << "Committing." << std::endl;

  for (const auto& kv : commit_cadd) {
    auto& custs = kv.second;
    auto& cand_id = kv.first;
    auto cand = MutableVehicle(*std::find_if(vehicles().begin(), vehicles().end(),
            [&](const Vehicle& a){ return a.id() == cand_id; }));
    std::vector<CustId> custs_to_add = {};
    for (const auto& cust_id : custs)
      custs_to_add.push_back(cust_id);
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

  // print << "Done" << std::endl;

  this->end_ht();
}

void Genetic::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void Genetic::end() {
  this->print_statistics();
}

void Genetic::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

DistInt Genetic::cost(const Assignment& sol) {
  dict<CustId, bool> penalty = {};
  DistInt cost = 0;
  for (const Customer& cust : customers())
    penalty[cust.id()] = true;
  for (const MutableVehicle& mutvehl : sol) {
    cost += mutvehl.route().cost();
    for (const Stop& stop : mutvehl.schedule().data())
      penalty[stop.owner()] = false;
  }
  for (const auto& kv : penalty)
    if (kv.second)
      cost += Cargo::basecost(kv.first);
  return cost;
}

Assignment Genetic::extract_fit(Population& pop) {
  DistInt best_solcst = InfInt;
  auto best = pop.end();
  for (auto i = pop.begin(); i != pop.end(); ++i) {
    auto& sol = (*i);
    if (sol.first < best_solcst) {
      best_solcst = sol.first;
      best = i;
    }
  }
  if (best == pop.end()) {
    print(MessageType::Error) << "Bad extract!" << std::endl;
    throw;
  }
  Assignment s = (*best).second;
  pop.erase(best);
  return s;
}

void Genetic::crossover(Assignment& A, Assignment& B) {
  std::sort(A.begin(), A.end(), [](const MutableVehicle& a, const MutableVehicle& b) {
    return a.id() < b.id();
  });
  std::sort(B.begin(), B.end(), [](const MutableVehicle& a, const MutableVehicle& b) {
    return a.id() < b.id();
  });
  std::uniform_int_distribution<> n(0, std::min(A.size(),B.size())-1);
  int xpoint = n(this->gen);
  Assignment child1, child2;
  child1.insert(child1.begin(), A.begin(), A.begin()+xpoint);
  child1.insert(child1.end(), B.begin()+xpoint, B.end());
  child2.insert(child2.begin(), B.begin(), B.begin()+xpoint);
  child2.insert(child2.end(), A.begin()+xpoint, A.end());
  A = child1;
  B = child2;
}

void Genetic::repair(Assignment& A) {
  for (auto i = A.begin(); i != A.end(); ++i) {
    for (const Stop& stop1 : i->schedule().data()) {
      if (stop1.owner() == i->id())
        continue;
      /* Search schedules of each assignment */
      for (auto j = i+1; j != A.end(); ++j) {
        auto sch = j->schedule().data();
        if (std::find_if(sch.begin(), sch.end(), [&](const Stop& stop2) {
              return stop1.owner() == stop2.owner();
            }) != sch.end()) {
          opdel(sch, stop1.owner());
          vec_t<Wayp> rte = {};
          route_through(sch, rte);
          j->set_sch(sch);
          j->set_rte(rte);
          j->reset_lvn();
          j->decr_queued();
          break;
        }
      }
    }
  }
}

void Genetic::print_population(const Population& pop) {
  for (const auto& sol : pop)
    this->print_solution(sol);
}

void Genetic::print_solution(const std::pair<DistInt, Assignment>& sol) {
  print << "Solution: " << sol.first << std::endl;
  for (const MutableVehicle& mutvehl : sol.second) {
    print << "Vehicle " << mutvehl.id() << ", ";
    for (const Stop& stop : mutvehl.schedule().data())
      if (stop.type() == StopType::CustDest)
        print << stop.owner() << " ";
    print << std::endl;
  }
}

void Genetic::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->timeout_0 = hiclock::now();
  this->timeout_ = BATCH*1000;
  this->is_matched = {};
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
  option.path_to_solution = "genetic.sol";
  option.path_to_dataout  = "genetic.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 20;
  option.matching_period  = 60;
  option.static_mode = false;
  Cargo cargo(option);
  Genetic gp;
  cargo.start(gp);

  return 0;
}

