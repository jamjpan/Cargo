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
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

#include "greedy.h"
#include "libcargo.h"

using namespace cargo;

Greedy::Greedy(const std::string& name) : RSAlgorithm(name, false), grid_(100) {
  this->batch_time() = 30;
}

void Greedy::handle_customer(const Customer& cust) {
  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed heuristic: try only if vehicle's current schedule has < 8 customer stops
    if (cand->schedule().data().size() < 10) {
      DistInt cost = sop_insert(*cand, cust, sch, rte, Cargo::gtree()) - cand->route().cost();
      if (cost < this->best_cost) {
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          this->best_vehl = cand;
          this->best_sch = sch;
          this->best_rte = rte;
          this->best_cost = cost;
        }
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (this->best_vehl != nullptr)
    this->matched = true;

  if (this->matched) {
    print << "Matched " << cust.id() << " with " << this->best_vehl->id() << std::endl;
    this->assign(
      {cust.id()}, {}, this->best_rte, this->best_sch, *(this->best_vehl));
  }
}

void Greedy::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void Greedy::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void Greedy::reset_workspace() {
  this->best_cost = InfInt;
  this->sch = best_sch = {};
  this->rte = best_rte = {};
  this->best_vehl = nullptr;
  this->candidates = {};
  this->matched = false;
}

