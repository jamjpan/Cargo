// Portions Copyright (c) 2018 the Cargo authors
// Portions Copyright (c) uakfdotb
//
// The following applies only to portions copyright the Cargo authors:
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
#include <chrono>
#include <ctime>
#include <exception>
#include <iostream>
#include <thread>
#include <vector>

#include "kinetic_tree.h"
#include "treeTaxiPath.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

KineticTrees::KineticTrees(const std::string& name)
    : RSAlgorithm(name, false), grid_(100) {
  this->batch_time() = BATCH;
}

KineticTrees::~KineticTrees() {
  for (auto kv : kt_)
    delete kv.second;
}

void KineticTrees::handle_customer(const Customer& cust) {
  DistInt range = pickup_range(cust);
  this->reset_workspace();
  this->candidates = this->grid_.within(range, cust.orig());

  Stop cust_orig = Stop(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_dest = Stop(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  DistInt best_cst = InfInt;

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed-up heuristics: try only if vehicle has less than 8 stops
    if (cand->schedule().data().size() < 10) {

      DistInt new_cost = this->kt_.at(cand->id())->value(  // here is the bottleneck
        cust.orig(), cust.dest(), cust.id(), range/Cargo::vspeed()+Cargo::now(), cust.late());

      if (new_cost > -1) {
        // The new cost can be LESS THAN the old cost if the
        // current schedule was not fully optimized (e.g. due to timeout)
        DistInt cst = new_cost - (cand->route().cost() - cand->route().dist_at(cand->idx_last_visited_node()+1));
        if (cst < best_cst) {
          sch = this->kt2sch(cand, cust_orig, cust_dest);
          if (chkcap(cand->capacity(), sch)) {
            best_vehl = cand;
            best_sch  = sch;
            best_cst  = cst;
          } else this->kt_.at(cand->id())->cancel();
        } else this->kt_.at(cand->id())->cancel();
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (best_vehl != nullptr)
    matched = true;

  /* Attempt commit to db */
  if (matched) {
    route_through(best_sch, best_rte);
    for (auto& wp : best_rte) wp.first +=
      best_vehl->route().dist_at(best_vehl->idx_last_visited_node() + 1);
    if (this->assign({cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      this->kt_.at(best_vehl->id())->push();
      this->sync_kt(kt_.at(best_vehl->id()), best_vehl->schedule().data());
      this->end_delay(cust.id());
    } else
      this->beg_delay(cust.id());
  }
  if (!matched)
    this->beg_delay(cust.id());
}

void KineticTrees::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);

  // Create a kinetic tree for vehl if none exists
  if (this->kt_.count(vehl.id()) == 0) {
    NodeId dest = vehl.dest();
    if (vehl.late() == -1)
      dest = -1;  // special node for taxis
    this->kt_[vehl.id()] =
      new TreeTaxiPath(vehl.schedule().data().at(0).loc(), dest, vehl.id());
  }

  // Create last-modified entry if none exists
  if (this->last_modified_.count(vehl.id()) == 0)
    this->last_modified_[vehl.id()] = Cargo::now();

  // Synchronize kinetic tree stops with local schedule for vehl
    if (this->sched_.count(vehl.id()) != 0) {
      if (this->sched_.at(vehl.id()).at(0).loc() != vehl.dest()) {
        this->sync_kt(this->kt_.at(vehl.id()), vehl.schedule().data());
      }
    }
  // }

  // Update local schedule with ground-truth vehicle schedule
  this->sched_[vehl.id()] = vehl.schedule().data();

  // Synchronize kinetic tree distances with elapsed time
  int dur = Cargo::now() - last_modified_.at(vehl.id());
  if (dur > 0) {
    //this->kt_.at(vehl.id())->moved(dur*Cargo::vspeed());
    this->kt_.at(vehl.id())->moved(dur);
  }

  // Update last-modified
  this->last_modified_[vehl.id()] = Cargo::now();
}

void KineticTrees::end() {
  RSAlgorithm::end();
}

void KineticTrees::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void KineticTrees::sync_kt(TreeTaxiPath* kt, const std::vector<Stop>& cur_sch) {
  std::vector<std::tuple<NodeId, NodeId, bool>> seq;
  kt->printStopSequence(seq);
  NodeId i = std::get<1>(*(seq.begin()+1));
  NodeId j = (cur_sch.begin()+1)->loc();
  while (i != j && i != -1) {
    kt->step();
    i = kt->next();
  }
  kt->move(cur_sch.begin()->loc());
}

std::vector<Stop> KineticTrees::kt2sch(const MutableVehicleSptr& cand,
                                       const Stop& cust_orig,
                                       const Stop& cust_dest) {
  std::vector<std::tuple<NodeId, NodeId, bool>> schseq;
  this->kt_.at(cand->id())->printTempStopSequence(schseq);

  std::vector<Stop> sch {};
  sch.push_back(cand->schedule().data().front());
  for (auto i = schseq.begin() + 1; i != schseq.end(); ++i) {
    auto j = std::find_if(cand->schedule().data().begin(), cand->schedule().data().end(),
        [&](const Stop& a) {
          bool same_loc = (a.loc() == std::get<1>(*i));
          bool same_owner = (a.owner() == std::get<0>(*i));
          bool same_type;
          if ((a.type() == StopType::VehlOrig ||
               a.type() == StopType::CustOrig) &&
              std::get<2>(*i) == true)
            same_type = true;
          else if ((a.type() == StopType::VehlDest ||
                    a.type() == StopType::CustDest) &&
                   std::get<2>(*i) == false)
            same_type = true;
          else
            same_type = false;
          return (same_loc && same_owner && same_type);
        });
    if (j != cand->schedule().data().end()) {
      sch.push_back(*j);
    }
    else {
      if (std::get<1>(*i) == cust_orig.loc()) sch.push_back(cust_orig);
      if (std::get<1>(*i) == cust_dest.loc()) sch.push_back(cust_dest);
      //if (i->first == cand->dest()) sch.push_back(cand->schedule().data().back());
    }
  }
  // Push back fake destination if taxi
  if (cand->late() == -1) {
    Stop last = sch.back();
    Stop fake_dest(cand->id(), last.loc(), StopType::VehlDest, last.early(), -1, -1);
    sch.push_back(fake_dest);
  }

  return sch;
}

void KineticTrees::reset_workspace() {
  this->sch = this->best_sch = {};
  this->rte = this->best_rte = {};
  this->candidates = {};
  this->matched = false;
  this->best_vehl = nullptr;
  this->timeout_0 = hiclock::now();
}

void KineticTrees::print_kt(TreeTaxiPath* kt, bool temp) {
  std::vector<std::tuple<NodeId, NodeId, bool>> seq;
  if (!temp) kt->printStopSequence(seq);
  else       kt->printTempStopSequence(seq);
  for (const auto& tuple : seq)
    print << "(" << std::get<0>(tuple) << "|" << std::get<1>(tuple) << ") ";
  print << std::endl;
}

