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
#include <utility>
#include <vector>

#include "bilateral+.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

BilateralPlus::BilateralPlus(const std::string& name)
    : RSAlgorithm(name, false), grid_(100) {
  this->batch_time() = BATCH;
  this->nswapped_ = 0;
}

void BilateralPlus::match() {
  this->clear();
  this->prepare();

  for (const Customer& cust : this->customers()) {
    vec_t<rank_cand>& candidates = lookup.at(cust.id());
    bool matched = false;
    auto i = candidates.cbegin();
    while (!matched && i != candidates.cend()) {
      MutableVehicleSptr cand = i->second;
      print << "cust " << cust.id() << " trying " << cand->id() << std::endl;
      if (!modified.at(cand)) {
        this->to_assign[cand].push_back(cust.id());
        this->modified[cand] = true;
        cand->set_sch(schedules.at(cust.id()).at(cand->id()));
        cand->set_rte(routes.at(cust.id()).at(cand->id()));
        cand->reset_lvn();
        matched = true;
        print << "  direct accept" << std::endl;
      } else {
        sop_insert(cand, cust, this->retry_sch, this->retry_rte);
        if (chktw(this->retry_sch, this->retry_rte)
         && chkcap(cand->capacity(), this->retry_sch)) {
          this->to_assign[cand].push_back(cust.id());
          this->modified[cand] = true;
          cand->set_sch(this->retry_sch);
          cand->set_rte(this->retry_rte);
          cand->reset_lvn();
          matched = true;
          print << "  feasible accept" << std::endl;
        } else {  // Replace procedure
          CustId cust_to_remove = randcust(cand->schedule().data());
          // Heuristic #1: Only try to replace if the replacement has longer base cost
          if (Cargo::basecost(cust_to_remove) < Cargo::basecost(cust.id())) {
            print << "  not feasible; replacing " << cust_to_remove << std::endl;
            if (cust_to_remove != -1) {
              DistInt old_cost = cand->route().cost();
              DistInt new_cost = sop_replace(cand, cust_to_remove, cust, this->replace_sch, this->replace_rte);
              // Heuristic #2: Only replace if the replacement reduces the cost
              if (new_cost < old_cost) {
                if (chktw(this->replace_sch, this->replace_rte)
                 && chkcap(cand->capacity(), this->replace_sch)) {
                  print << "  replace accept" << std::endl;
                  this->to_assign[cand].push_back(cust.id());
                  this->modified[cand] = true;
                  cand->set_sch(this->replace_sch);
                  cand->set_rte(this->replace_rte);
                  cand->reset_lvn();
                  matched = true;
                  auto remove_ptr = std::find(to_assign[cand].begin(), to_assign[cand].end(), cust_to_remove);
                  if (remove_ptr != to_assign[cand].end())
                    to_assign[cand].erase(remove_ptr);
                  else
                    to_unassign[cand].push_back(cust_to_remove);
                  this->nswapped_++;
                }
              }
            }
          }
        }
      }
      std::advance(i, 1);
    }
  }

  // Batch-commit the solution
  for (const auto& kv : this->modified) {
    if (kv.second == true) {
      this->commit(kv.first);
    }
  }
}

void BilateralPlus::handle_vehicle(const cargo::Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void BilateralPlus::prepare() {
  vec_t<Stop> sch;
  vec_t<Wayp> rte;

  auto add_or_remove = [&](const Customer& cust) {
    if (this->timeout(this->timeout_0)) return true;  // don't continue if no more time
    this->lookup[cust.id()] = {};
    vec_t<MutableVehicleSptr> cands = this->grid_.within(pickup_range(cust), cust.orig());
    for (const MutableVehicleSptr& cand : cands) {
      // Speed heuristic: try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < 10) {
        this->to_assign[cand] = {};
        this->to_unassign[cand] = {};
        DistInt cost = sop_insert(cand, cust, sch, rte) - cand->route().cost();
        if (chktw(sch, rte) && chkcap(cand->capacity(), sch)) {
          this->lookup.at(cust.id()).push_back(std::make_pair(cost, cand));
          this->schedules[cust.id()][cand->id()] = std::move(sch);
          this->routes[cust.id()][cand->id()] = std::move(rte);
          this->modified[cand] = false;
        }
      }
    }
    if (this->lookup.at(cust.id()).empty()) return true;
    else {
      // Order the candidates in order to access by greedy
      std::sort(this->lookup.at(cust.id()).begin(), this->lookup.at(cust.id()).end(),
        [](const rank_cand& a, const rank_cand& b) { return a.first < b.first; });
      return false;
    }
  };

  this->customers().erase(std::remove_if(this->customers().begin(), this->customers().end(),
    add_or_remove), this->customers().end());
}

void BilateralPlus::clear() {
  this->lookup = {};
  this->schedules = {};
  this->routes = {};
  this->modified = {};
  this->to_assign = {};
  this->to_unassign = {};
  this->timeout_0 = hiclock::now();
  this->retry_sch = this->replace_sch = {};
  this->retry_rte = this->replace_rte = {};
}

void BilateralPlus::commit(const MutableVehicleSptr& cand) {
  vec_t<CustId>& cadd = this->to_assign.at(cand);
  vec_t<CustId>& cdel = this->to_unassign.at(cand);
  this->assign_or_delay(cadd, cdel, cand->route().data(), cand->schedule().data(), *cand);
  for (const CustId& cid : cadd) print << "Matched " << cid << " with " << cand->id() << std::endl;
  for (const CustId& cid : cdel) print << "Removed " << cid << " from " << cand->id() << std::endl;
}

void BilateralPlus::end() {
  print(MessageType::Info) << "swaps: " << this->nswapped_ << std::endl;
  RSAlgorithm::end();
}

void BilateralPlus::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

