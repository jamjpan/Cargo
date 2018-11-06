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

#include "kt.h"
#include "treeTaxiPath.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 30;

KineticTrees::KineticTrees()
    : RSAlgorithm("kt", true), grid_(100) {
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
  this->candidates = this->grid_.within(range, cust.orig());

  Stop cust_orig = Stop(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_dest = Stop(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  //DistInt max_travel = (cust.late()-Cargo::now())*Cargo::vspeed();  // distance from root
  DistInt best_cst = InfInt;

  //print << "At t=" << Cargo::now() << ", computed max_travel=" << max_travel << " for cust " << cust.id() << std::endl;

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed-up heuristics: try only if vehicle has less than 8 stops
    if (cand->schedule().data().size() < 10) {
      print << "\t\tTrying " << cand->id() << std::endl;
      print << "\t\tkt: ";
      print_kt(this->kt_.at(cand->id()), false);
      for (const auto& sp : cand->schedule().data())
        print << "\t\t\t(" << sp.owner() << "|" << sp.loc() << "|" << sp.early()
              << "|" << sp.late() << "|" << (int)sp.type() << ") " << std::endl;

      // Function value() returns cost from cand's next stop to end
      //DistInt new_cost = this->kt_.at(cand->id())->value(  // here is the bottleneck
      //  cust.orig(), cust.dest(), cust.id(), range/Cargo::vspeed(), max_travel/Cargo::vspeed());
      DistInt new_cost = this->kt_.at(cand->id())->value(  // here is the bottleneck
        cust.orig(), cust.dest(), cust.id(), range/Cargo::vspeed()+Cargo::now(), cust.late());

      print << "\t\t\tGot new route cost " << new_cost << " and sch: ";
      print_kt(this->kt_.at(cand->id()), true);
      // print << "\t\t\tRemaining route cost: " << cand->remaining() << std::endl;
      // print << "\t\t\tNext-node distance: " << cand->next_node_distance() << std::endl;
      print << "\t\t\tCurrent sch: ";
      for (const Stop& a : cand->schedule().data())
        print << a.loc() << " ";
      print << std::endl;
      if (new_cost > -1) {
        // The new cost can be LESS THAN the old cost if the
        // current schedule was not fully optimized due to timeout
        DistInt cst = new_cost - (cand->route().cost() - cand->route().dist_at(cand->idx_last_visited_node()+1));
        // print << "\t\tVehl " << cand->id() << ": " << cst << std::endl;
        // if (cst < -1) {  // 1 m buffer due to rounding error
        //   // HACK: recompute the current cost and subtract from new cost
        //   // vec_t<Wayp> hack_rte = {};
        //   // route_through(cand->schedule().data(), hack_rte);
        //   // cst = new_cost = hack_rte.back().first;
        //   print(MessageType::Error) << "Got negative detour!" << std::endl;
        //   print << "detour: " << cst << std::endl;
        //   print << "Vehicle " << cand->id() << std::endl;
        //   print << "new cost: " << new_cost << std::endl;
        //   print << "current cost: " << cand->route().cost() << std::endl;
        //   print << "last-visited node index: " << cand->idx_last_visited_node() << std::endl;
        //   print << "last-visited node: " << cand->last_visited_node() << std::endl;
        //   print << "dist at last-visited node+1: " << cand->route().dist_at(cand->idx_last_visited_node()+1) << std::endl;
        //   print << "next-node dist: " << cand->next_node_distance() << std::endl;
        //   throw;
        // }
        if (cst < best_cst) {
          print << "kt+: ";
          print_kt(this->kt_.at(cand->id()), true);
          sch = this->kt2sch(cand, cust_orig, cust_dest);
          print << "sch: ";
          for (const Stop& a : sch)
            print << a.loc() << " ";
          print << std::endl;
          // VALIDATE (remove in production)
          route_through(sch, rte);
          // if (!chktw(sch, rte)) {  // <-- might fail because the sch is computed at a time BEFORE match latency,
          //                          // but chktw is using the CURRENT TIME. Instead, chktw should be passed the
          //                          // adjusted sch that accounts for match latency. Let assign() do that work!
          //   print << "FAILS TIME CONSTRAINTS" << std::endl;
          //   print_sch(sch);
          //   print_rte(rte);
          //   throw;
          // }
          if (chkcap(cand->capacity(), sch)) {
           // && chktw(sch, rte)) {
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
    }
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
    // best_rte.insert(best_rte.begin(),
    //       best_vehl->route().at(best_vehl->idx_last_visited_node()));
    if (this->assign({cust.id()}, {}, best_rte, best_sch, *best_vehl)) {  // <-- best_vehl is synced now if assign succeeds
      // print << "\tAssigned." << std::endl;
      this->kt_.at(best_vehl->id())->push();
      print << "kt* (unsync): ";
      print_kt(this->kt_.at(best_vehl->id()), false);
      this->sync_kt(kt_.at(best_vehl->id()), best_vehl->schedule().data());
      print << "kt* (sync)  : ";
      print_kt(this->kt_.at(best_vehl->id()), false);
      this->end_delay(cust.id());
    } else {
      // print << "\tRejected." << std::endl;
      this->beg_delay(cust.id());
    }
  }
  if (!matched) {
    // print << "\tNot matched." << std::endl;
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
      new TreeTaxiPath(vehl.schedule().data().at(0).loc(), dest, vehl.id());
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
  if (dur > 0) {
    //this->kt_.at(vehl.id())->moved(dur*Cargo::vspeed());
    this->kt_.at(vehl.id())->moved(dur);
  }

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
  print << "sync_kt() got kt:" << std::endl;
  print_kt(kt, false);
  print << "sync_kt() got cur_sch:" << std::endl;
  for (const auto& sp : cur_sch)
    print << "(" << sp.owner() << "|" << sp.loc() << "|" << sp.early()
          << "|" << sp.late() << "|" << (int)sp.type() << ") " << std::endl;
  std::vector<std::tuple<NodeId, NodeId, bool>> seq;
  kt->printStopSequence(seq);
  // Why did I add +1 here? Answer: the first stop in the schedule is fixed?
  NodeId i = std::get<1>(*(seq.begin()+1));
  NodeId j = (cur_sch.begin()+1)->loc();
  while (i != j && i != -1) {
    kt->step();
    i = kt->next();
  }
  kt->move(cur_sch.begin()->loc());
  print << "sync_kt() got new kt:" << std::endl;
  print_kt(kt, false);
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

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-m5k-c3.instance";
  option.path_to_solution = "kt.sol";
  option.path_to_dataout  = "kt.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode = false;
  option.static_mode = true;
  Cargo cargo(option);
  KineticTrees kt;
  cargo.start(kt);

  return 0;
}

