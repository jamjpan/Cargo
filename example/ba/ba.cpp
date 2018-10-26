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
#include <algorithm> /* std::random_shuffle, std::remove_if, std::find_if */
#include <iostream> /* std::endl */
#include <queue>
#include <tuple>
#include <vector>

#include "ba.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

BilateralArrangement::BilateralArrangement()
    : RSAlgorithm("ba", true), grid_(100) {
  this->batch_time() = BATCH;
  this->nswapped_ = 0;
}

void BilateralArrangement::match() {
  this->beg_batch_ht();
  this->timeout_0 = hiclock::now();

  print << "Prepare..." << std::endl;
  this->prepare();
  print << "Prepare done" << std::endl;
  // Now lookup, schedules, routes, modified are all populated

  for (const Customer& cust : this->customers()) {
    vec_t<rank_cand>& candidates = lookup.at(cust.id());
    print << "Handling cust " << cust.id() << " (" << candidates.size() << " candidates)" << std::endl;
    if (candidates.size() == 0) {
      print << "\tNo candidates" << std::endl;
      continue;
    }
    // Order the candidates in order to access by greedy
    std::sort(candidates.begin(), candidates.end(),
      [](const rank_cand& a, const rank_cand& b) {
        return a.first < b.first;
    });
    bool matched = false;
    auto i = candidates.begin();
    while (!matched && i != candidates.end()) {
      MutableVehicleSptr cand = i->second;
      print << "\tTrying vehl " << cand->id() << std::endl;
      if (!modified.at(cand)) {
        print << "\t\tNot modified, directly added" << std::endl;
        this->to_assign[cand].push_back(cust.id());
        this->modified[cand] = true;
        cand->set_sch(schedules.at(cust.id()).at(cand->id()));
        cand->set_rte(routes.at(cust.id()).at(cand->id()));
        cand->reset_lvn();
        matched = true;
      } else {
        vec_t<Stop> retry_sch;
        vec_t<Wayp> retry_rte;
        sop_insert(cand, cust, retry_sch, retry_rte);
        if (chktw(retry_sch, retry_rte) && chkcap(cand->capacity(), retry_sch)) {
          print << "\t\tModified but feasible, added" << std::endl;
          this->to_assign[cand].push_back(cust.id());
          this->modified[cand] = true;
          cand->set_sch(retry_sch);
          cand->set_rte(retry_rte);
          cand->reset_lvn();
          matched = true;
        } else {  // Replace procedure
          CustId cust_to_remove = randcust(cand->schedule().data());
          print << "\t\tModified and now infeasible, trying to replace" << std::endl;
          if (cust_to_remove != -1) {
            vec_t<Stop> replace_sch;
            vec_t<Wayp> replace_rte;
            sop_replace(cand, cust_to_remove, cust, replace_sch, replace_rte);
            if (chktw(replace_sch, replace_rte) && chkcap(cand->capacity(), replace_sch)) {
              print << "\t\t\tReplace " << cust_to_remove << " feasible, added" << std::endl;
              this->to_assign[cand].push_back(cust.id());
              this->modified[cand] = true;
              cand->set_sch(replace_sch);
              cand->set_rte(replace_rte);
              cand->reset_lvn();
              matched = true;
              auto remove_ptr = std::find(to_assign[cand].begin(), to_assign[cand].end(), cust_to_remove);
              if (remove_ptr != to_assign[cand].end())
                to_assign[cand].erase(remove_ptr);
              else
                to_unassign[cand].push_back(cust_to_remove);
              this->nswapped_++;
            } else {
              print << "\t\t\tReplace " << cust_to_remove << " still infeasible" << std::endl;
            }
          } else {
            print << "\t\t\tNothin to replace" << std::endl;
          }
        }
      }
      std::advance(i, 1);
    }
    if (!matched)
      this->beg_delay(cust.id());
    if (this->timeout(this->timeout_0))
      break;
  }

  // Batch-commit the solution
  for (const auto& kv : this->modified) {
    if (kv.second == true) {
      this->commit(kv.first);
    }
  }

  this->end_batch_ht();
}

void BilateralArrangement::handle_vehicle(const cargo::Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void BilateralArrangement::prepare() {
  this->clear();
  vec_t<Stop> temp_sch;
  vec_t<Wayp> temp_rte;
  for (const Customer& cust : this->customers()) {
    this->lookup[cust.id()] = {};
    vec_t<MutableVehicleSptr> cands = this->grid_.within(pickup_range(cust), cust.orig());
    for (const MutableVehicleSptr& cand : cands) {
      // Speed heuristic: try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < 10) {
        this->to_assign[cand] = {};
        this->to_unassign[cand] = {};
        DistInt cost = sop_insert(cand, cust, temp_sch, temp_rte) - cand->route().cost();
        if (chktw(temp_sch, temp_rte) && chkcap(cand->capacity(), temp_sch)) {
          auto rank = std::make_pair(cost, cand);
          this->lookup.at(cust.id()).push_back(rank);
          this->schedules[cust.id()][cand->id()] = temp_sch;  // store a copy
          this->routes[cust.id()][cand->id()] = temp_rte;     // store a copy
          this->modified[cand] = false;
        }
      }
    }
  }
}

void BilateralArrangement::clear() {
  this->lookup = {};
  this->schedules = {};
  this->routes = {};
  this->modified = {};
  this->to_assign = {};
  this->to_unassign = {};
}

void BilateralArrangement::commit(const MutableVehicleSptr& cand) {
  vec_t<CustId>& cadd = this->to_assign.at(cand);
  vec_t<CustId>& cdel = this->to_unassign.at(cand);
  cand->reset_lvn();
  this->assign_or_delay(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand, false/*true*/);
  for (const CustId& cid : cadd) print << "Matched " << cid << " to vehl " << cand->id() << std::endl;
  for (const CustId& cid : cdel) {
    print << "Removed " << cid << " from vehl " << cand->id() << std::endl;
    this->beg_delay(cid);
  }
}

void BilateralArrangement::end() {
  print(MessageType::Info) << "swaps: " << this->nswapped_ << std::endl;
  this->print_statistics();
}

void BilateralArrangement::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

int main() {
  /* Set options */
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_solution = "ba.sol";
  option.path_to_dataout  = "ba.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.static_mode=true;
  Cargo cargo(option);
  BilateralArrangement ba;
  cargo.start(ba);

  return 0;
}

