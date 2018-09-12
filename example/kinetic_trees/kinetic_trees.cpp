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
#include <algorithm> /* std::find_if */
#include <chrono>
#include <ctime>
#include <exception>
#include <iostream> /* std::endl */
#include <thread>
#include <vector>

#include "kinetic_trees.h"
#include "treeTaxiPath.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 1;
const int RANGE = 2000;

KineticTrees::KineticTrees()
    : RSAlgorithm("kinetic_trees", false), grid_(100) {
  this->batch_time() = BATCH;
}

KineticTrees::~KineticTrees() {
  for (auto kv : kt_)
    delete kv.second;
}

void KineticTrees::handle_customer(const Customer& cust) {
  this->beg_ht();
  this->reset_workspace();
  this->candidates =
    this->grid_.within(RANGE, cust.orig());

  Stop cust_orig = Stop(
    cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_dest = Stop(
    cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  DistInt max_travel = (cust.late()-cust.early())*Cargo::vspeed();
  DistInt best_cst = InfInt;

  for (const MutableVehicleSptr& cand : this->candidates) {
    if (cand->queued() < cand->capacity()) {
      DistInt cst = this->kt_.at(cand->id())->value(
        cust.orig(), cust.dest(), RANGE, max_travel);
      if (cst != -1) {
        cst -= cand->route().cost();
        if (cst < best_cst) {
          best_sch = this->kt2sch(cand, cust_orig, cust_dest);
          best_vehl = cand;
          best_cst  = cst;
        } else {
          this->kt_.at(cand->id())->cancel();
        }
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (best_vehl != nullptr)
    matched = true;

  /* Attempt commit to db */
  if (matched) {
    DistInt head = best_vehl->route().dist_at(best_vehl->idx_last_visited_node() + 1);
    route_through(best_sch, best_rte);
    for (auto& wp : best_rte)
      wp.first += head;
    best_rte.insert(best_rte.begin(),
          best_vehl->route().at(best_vehl->idx_last_visited_node()));
    if (this->assign(
      {cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      this->kt_.at(best_vehl->id())->push();
      this->sync_kt(kt_.at(best_vehl->id()), best_vehl->schedule().data());
      this->end_delay(cust.id());
    } else {
      this->nrej_++;
      this->beg_delay(cust.id());
    }
  }
  if (!matched)
    this->beg_delay(cust.id());

  this->end_ht();
}

void KineticTrees::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);

  /* Synchronize kinetic tree to vehicle motion */
  if (this->kt_.count(vehl.id()) == 0) {
    this->kt_[vehl.id()] =
      new TreeTaxiPath(vehl.schedule().data().at(0).loc(), vehl.dest());
  }

  if (this->last_modified_.count(vehl.id()) == 0)
    this->last_modified_[vehl.id()] = Cargo::now();

  if (vehl.dest() != this->kt_.at(vehl.id())->get_dest()) {
    delete kt_.at(vehl.id());
    this->kt_[vehl.id()] =
      new TreeTaxiPath(vehl.orig(), vehl.dest());
  }
  else {
    if (this->sched_.count(vehl.id()) != 0) {
      if (this->sched_.at(vehl.id()).at(0).loc() != vehl.dest()) {
        this->sync_kt(this->kt_.at(vehl.id()), vehl.schedule().data());
      }
    }
  }

  this->sched_[vehl.id()] = vehl.schedule().data();

  int dur = Cargo::now() - last_modified_.at(vehl.id());
  if (dur > 0)
    this->kt_.at(vehl.id())->moved(dur * Cargo::vspeed());

  this->last_modified_[vehl.id()] = Cargo::now();
}

void KineticTrees::end() {
  this->print_statistics();
}

void KineticTrees::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

void KineticTrees::sync_kt(TreeTaxiPath* kt, const std::vector<Stop>& cur_sch) {
  std::vector<std::pair<NodeId, bool>> seq;
  kt->printStopSequence(seq);
  NodeId i = (seq.begin()+1)->first;
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
  std::vector<std::pair<NodeId, bool>> schseq;
  this->kt_.at(cand->id())->printTempStopSequence(schseq);

  std::vector<Stop> sch {};
  sch.push_back(cand->schedule().data().front());
  for (auto i = schseq.begin() + 1; i != schseq.end(); ++i) {
    auto j = std::find_if(
        cand->schedule().data().begin() + 1, cand->schedule().data().end(),
        [&](const Stop& a) {
          bool same_loc = (a.loc() == i->first);
          bool same_type;
          if ((a.type() == StopType::VehlOrig ||
               a.type() == StopType::CustOrig) &&
              i->second == true)
            same_type = true;
          else if ((a.type() == StopType::VehlDest ||
                    a.type() == StopType::CustDest) &&
                   i->second == false)
            same_type = true;
          else
            same_type = false;
          return (same_loc && same_type);
        });
    if (j != cand->schedule().data().end())
      sch.push_back(*j);
    else {
      if (i->first == cust_orig.loc()) sch.push_back(cust_orig);
      if (i->first == cust_dest.loc()) sch.push_back(cust_dest);
    }
  }

  return sch;
}

void KineticTrees::reset_workspace() {
  this->best_sch = {};
  this->best_rte = {};
  this->candidates = {};
  this->matched = false;
  this->best_vehl = nullptr;
  this->timeout_0 = hiclock::now();
}

void KineticTrees::print_kt(TreeTaxiPath* kt, bool temp) {
  std::vector<std::pair<NodeId, bool>> seq;
  if (!temp) kt->printStopSequence(seq);
  else       kt->printTempStopSequence(seq);
  for (const auto& pair : seq)
    print << pair.first << " ";
  print << std::endl;
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  option.path_to_solution = "kinetic_trees.sol";
  option.path_to_dataout  = "kinetic_trees.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 20;
  option.matching_period  = 60;
  option.static_mode = false;
  Cargo cargo(option);
  KineticTrees kt;
   cargo.start(kt);

  return 0;
}

