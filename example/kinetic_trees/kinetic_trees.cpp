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

const int BATCH = 30;

KineticTrees::KineticTrees()
    : RSAlgorithm("kinetic_trees", true), grid_(100) {
  this->batch_time() = BATCH;
}

KineticTrees::~KineticTrees() {
  for (auto kv : kt_)
    delete kv.second;
}

void KineticTrees::handle_customer(const Customer& cust) {
  print << "Handling cust " << cust.id() << std::endl;
  DistInt range = pickup_range(cust);
  this->beg_ht();
  this->reset_workspace();
  this->candidates =
    this->grid_.within(range, cust.orig());

  Stop cust_orig = Stop(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_dest = Stop(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  DistInt max_travel = (cust.late()-Cargo::now())*Cargo::vspeed();  // distance from root
  DistInt best_cst = InfInt;

  print << "\tGot " << this->candidates.size() << " candidates." << std::endl;

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed-up heuristics:
    //   1) Try only if vehicle has capacity at this point in time
    //   2) Try only if vehicle's current schedule len < 8 customer stops
    // if (cand->capacity() > 1 && cand->schedule().data().size() < 10) {
      print << "\t\tTrying " << cand->id() << std::endl;
      for (const auto& sp : cand->schedule().data())
        print << "\t\t\t(" << sp.owner() << "|" << sp.loc() << "|" << sp.early()
              << "|" << sp.late() << "|" << (int)sp.type() << ") " << std::endl;

      // Function value() returns cost from cand's next stop to end
      DistInt cst = this->kt_.at(cand->id())->value(  // here is the bottleneck
        cust.orig(), cust.dest(), range, max_travel);

      print << "\t\t\tGot new route cost " << cst << " and sch: ";
      print_kt(this->kt_.at(cand->id()), true);
      print << "\t\t\tRemaining route cost: " << cand->remaining() << std::endl;
      print << "\t\t\tNext-node distance: " << cand->next_node_distance() << std::endl;
      print << "\t\t\tCurrent sch: ";
      for (const Stop& a : cand->schedule().data())
        print << a.loc() << " ";
      print << std::endl;
      if (cst > -1) {
        cst += cand->next_node_distance();
        cst -= cand->remaining();
        print << "\t\tVehl " << cand->id() << ": " << cst << std::endl;
        if (cst < 0) {
          print(MessageType::Error) << "Got negative detour!" << std::endl;
          throw;
        }
        if (cst < best_cst) {
          print << "kt+: ";
          print_kt(this->kt_.at(cand->id()), true);
          sch = this->kt2sch(cand, cust_orig, cust_dest);
          print << "sch: ";
          for (const Stop& a : sch)
            print << a.loc() << " ";
          print << std::endl;
          route_through(sch, rte);
          if (chkcap(cand->capacity(), sch)
           && chktw(sch, rte)) {
            best_vehl = cand;
            best_sch  = sch;
            best_rte  = rte;
            best_cst  = cst;
          } else {
            this->kt_.at(cand->id())->cancel();
          }
        } else {
          this->kt_.at(cand->id())->cancel();
        }
      }
    // }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (best_vehl != nullptr)
    matched = true;

  /* Attempt commit to db */
  if (matched) {
    print << "\tMatched " << cust.id() << " with " << best_vehl->id() << std::endl;
    DistInt head = best_vehl->route().dist_at(best_vehl->idx_last_visited_node() + 1);
    for (auto& wp : best_rte)
      wp.first += head;
    best_rte.insert(best_rte.begin(),
          best_vehl->route().at(best_vehl->idx_last_visited_node()));
    if (this->assign(
      {cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      print << "\tAssigned." << std::endl;
      this->kt_.at(best_vehl->id())->push();
      print << "kt* (unsync): ";
      print_kt(this->kt_.at(best_vehl->id()), false);
      this->sync_kt(kt_.at(best_vehl->id()), best_vehl->schedule().data());
      print << "kt* (sync)  : ";
      print_kt(this->kt_.at(best_vehl->id()), false);
      this->end_delay(cust.id());
    } else {
      print << "\tRejected." << std::endl;
      this->beg_delay(cust.id());
    }
  }
  if (!matched) {
    print << "\tNot matched." << std::endl;
    this->beg_delay(cust.id());
  }

  this->end_ht();
}

void KineticTrees::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);

  // Create a kinetic tree for vehl if none exists
  if (this->kt_.count(vehl.id()) == 0) {
    NodeId dest = vehl.dest();
    if (vehl.late() == -1)
      dest = -1;  // special node for taxis
    this->kt_[vehl.id()] =
      new TreeTaxiPath(vehl.schedule().data().at(0).loc(), dest);
  }

  // Create last-modified entry if none exists
  if (this->last_modified_.count(vehl.id()) == 0)
    this->last_modified_[vehl.id()] = Cargo::now();

  // if (vehl.dest() != this->kt_.at(vehl.id())->get_dest()) {
  //   delete kt_.at(vehl.id());
  //   this->kt_[vehl.id()] =
  //     new TreeTaxiPath(vehl.orig(), vehl.dest());
  // }
  // else {

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
  if (dur > 0)
    this->kt_.at(vehl.id())->moved(dur * Cargo::vspeed());

  // Update last-modified
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
  // Why did I add +1 here? Answer: the first stop in the schedule is fixed?
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
  // Push back fake destination if taxi
  if (cand->late() == -1) {
    Stop last = sch.back();
    Stop fake_dest(last.owner(), last.loc(), StopType::VehlDest, last.early(), -1, -1);
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
  option.static_mode = true;
  Cargo cargo(option);
  KineticTrees kt;
   cargo.start(kt);

  return 0;
}

