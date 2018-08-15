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
#include <ctime>
#include <exception>
#include <iostream> /* std::endl */
#include <thread>
#include <vector>

#include "libcargo.h"

#include "kinetic_trees.h"
#include "treeTaxiPath.h"  // (uakfdotb/tsharesim)

using namespace cargo;

const int RETRY   = 15; // retry unmatched cust after RETRY sec
const int TIMEOUT = 30; // timeout customers take > TIMEOUT sec

bool KineticTrees::timeout(clock_t& start) {
  clock_t end = std::clock();
  return (double(start-end)/CLOCKS_PER_SEC >= TIMEOUT)
      || this->done();  // (rsalgorithm.h)
}

KineticTrees::KineticTrees() : RSAlgorithm("kinetic_trees"),
      grid_(100) {   // (grid.h)
  batch_time() = 1;  // (rsalgorithm.h)

  /* Private vars */
  nmat_  = 0;  // match counter
  nrej_  = 0;  // number rejected due to out-of-sync
  delay_ = {}; // delay container
}

KineticTrees::~KineticTrees() {
  for (auto kv : kt_)
    delete kv.second;  // cleanup
}

void KineticTrees::handle_customer(const Customer& cust) {
  clock_t start = std::clock();

  /* Skip customers already assigned (but not yet picked up) */
  if (cust.assigned())
    return;

  /* Skip customers looked at within the last RETRY seconds */
  if (delay_.count(cust.id()) && delay_.at(cust.id()) >= Cargo::now() - RETRY)
    return;

  /* Containers for storing outputs */
  DistInt cst, best_cst = InfInt;
  std::vector<Stop> sch, best_sch;
  std::vector<Wayp> rte, best_rte;

  /* best_vehl will point to an underlying MutableVehicle in our grid */
  std::shared_ptr<MutableVehicle> best_vehl = nullptr;
  bool matched = false;

  /* Get candidates from the local grid index */
  DistInt rng = pickup_range(cust, Cargo::now()); /* 2000; */
  auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

  /* Create the customer stops to insert into best vehicle. These will
   * be used in the kt.value() method to do tree insertion. */
  Stop cust_orig = Stop(cust.id(), cust.orig(), StopType::CustOrig, cust.early(), cust.late());
  Stop cust_dest = Stop(cust.id(), cust.dest(), StopType::CustDest, cust.early(), cust.late());

  /* Loop through candidates and check which is the greedy match */
  for (const auto& cand : candidates) {
    /* Skip vehicles queued to capacity (queued = number of customer assigned
     * but may or may not be picked up yet) */
    if (cand->queued() == cand->capacity())
      continue;

    // print << "handle_customer(" << cust.id() << ") to vehl" << cand->id() << std::endl;
    // print << "\tins. into kt: "; print_kt(kt_.at(cand->id())); print.flush();

    /* Define the maximum travel bound based on customer's time window */
    DistInt max_travel = (cust.late()-cust.early())*Cargo::vspeed();

    /* Find the least-cost schedule using kinetic tree */
    cst = kt_.at(cand->id())->value(  // (treeTaxiPath.h)
            cust.orig(), cust.dest(), rng, max_travel);

    /* Skip if vehicle is infeasible */
    if (cst == -1)
      continue;

    // print << "\taug. kt: "; print_kt(kt_.at(cand->id()), true);
    // print.flush();

    /* Retrieve the new schedule */
    std::vector<std::pair<NodeId, bool>> schseq;  // container
    kt_.at(cand->id())->printTempStopSequence(schseq);

    /* Convert the sequence of nodes returned from the tree into a vector of
     * stops for committing into the db */
    sch.clear();
    sch.push_back(cand->schedule().data().front());
    for (auto i = schseq.begin()+1; i != schseq.end(); ++i) {
      auto j = std::find_if(cand->schedule().data().begin()+1, cand->schedule().data().end(),
        [&](const Stop& a) {
          bool same_loc = (a.loc() == i->first);
          bool same_type;
          if ((a.type() == StopType::VehlOrig || a.type() == StopType::CustOrig)
            && i->second == true) same_type = true;
          else if ((a.type() == StopType::VehlDest || a.type() == StopType::CustDest)
            && i->second == false) same_type = true;
          else
            same_type = false;
          return (same_loc && same_type);
        });
      if (j != cand->schedule().data().end()) sch.push_back(*j);
      else {
        if (i->first == cust_orig.loc()) sch.push_back(cust_orig);
        if (i->first == cust_dest.loc()) sch.push_back(cust_dest);
      }
    }
    // print << "\taug.sch: "; print_sch(sch);
    // print.flush();

    /* Route through the least-cost schedule */
    // The distances to the nodes in the routes found by route_through need to
    // be corrected.  sch passed here contains only un-visited stops. The first
    // stop in the schedule is the vehicle's next node.  route_through will
    // give this stop a distance of 0.  The distances to other stops in the
    // augmented schedule passed to route_through will be relative to this
    // first stop. The already-traveled distance (the head) should be added.
    DistInt head = cand->route().dist_at(cand->idx_last_visited_node() + 1);
    rte.clear();
    route_through(sch, rte);
    // Add head to the new nodes in the route
    for (auto& wp : rte) wp.first += head;
    rte.insert(rte.begin(), cand->route().at(cand->idx_last_visited_node()));

    bool within_time = chktw(sch, rte);
    if ((cst < best_cst) && within_time) {
      if (best_vehl != nullptr)
        kt_.at(best_vehl->id())->cancel(); // <-- cancel current best
      best_cst = cst;
      best_sch = sch;
      best_rte = rte;
      best_vehl = cand;  // copy the pointer
      matched = true;
    } else
      kt_.at(cand->id())->cancel();
    if (timeout(start))
      break;
  }

  /* Commit match to the db */
  bool add_to_delay = true;
  if (matched) {
    if (assign({cust.id()}, {}, best_rte, best_sch, *best_vehl)) {
      /* Accept the new kinetic tree, and sync it up to sync_sch */
      kt_.at(best_vehl->id())->push();
      // print << "handle_customer(" << cust.id() << ") accepted vehl" << best_vehl->id() << std::endl;
      // print << "\tsync sch: "; print_sch(best_vehl->schedule().data());
      // print.flush();
      // print << "\tnew kt: "; print_kt(kt_.at(best_vehl->id()));
      // print.flush();
      sync_kt(kt_.at(best_vehl->id()), best_vehl->schedule().data());
      // print << "\tmod kt: "; print_kt(kt_.at(best_vehl->id()));
      // print.flush();
      print(MessageType::Success)
        << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")" << std::endl;
      nmat_++;
      /* Remove customer from delay storage */
      if (delay_.count(cust.id())) delay_.erase(cust.id());
      add_to_delay = false;
    } else
      nrej_++;
  }
  if (add_to_delay) {
    /* Add customer to delay storage */
    delay_[cust.id()] = Cargo::now();
  }
}

void KineticTrees::handle_vehicle(const Vehicle& vehl) {
  // print << "handle_vehicle(" << vehl.id() << ")... " << std::endl;
  /* Insert into grid */
  grid_.insert(vehl);

  /* Create kinetic tree (treeTaxiPath.h) */
  if (kt_.count(vehl.id()) == 0) {
    kt_[vehl.id()] = new TreeTaxiPath(vehl.schedule().data().at(0).loc(), vehl.dest());
    // print(MessageType::Info) << "handle_vehicle(" << vehl.id() << ") created kt" << std::endl;
  }

  /* Add to last-modified */
  if (last_modified_.count(vehl.id()) == 0)
    last_modified_[vehl.id()] = Cargo::now();

  /* Update dest if it's changed (permanent taxi) */
  if (vehl.dest() != kt_.at(vehl.id())->get_dest()) {
    delete kt_.at(vehl.id());
    kt_[vehl.id()] = new TreeTaxiPath(vehl.orig(), vehl.dest());
    // print(MessageType::Info) << "handle_vehicle(" << vehl.id() << ") re-created kt" << std::endl;
  }
  else {
    /* Step the kinetic tree */
    if (sched_.count(vehl.id()) != 0) {
      /* The vehicle could have moved a lot since the last handle_vehicle to it.
       * We check its current schedule against our memory of its schedule and
       * adjust the kinetic tree to match. */
      if (sched_.at(vehl.id()).at(0).loc() != vehl.dest()) {
        // print << "\tcur sched: ";
        // print_sch(vehl.schedule().data());
        // print.flush();
        // print << "\tcached sched: ";
        // print_sch(sched_.at(vehl.id()));
        // print.flush();
        // print << "\tcached kt: "; print_kt(kt_.at(vehl.id()));
        // for (int i = 0; i < visited_stops; ++i)
        //   kt_.at(vehl.id())->step();  // (treeTaxiPath.h)
        sync_kt(kt_.at(vehl.id()), vehl.schedule().data());
        // print << "\tmod. kt: ";
        // print_kt(kt_.at(vehl.id()));
        // print.flush();
      }
    }
  }

  /* Create/Update vehicle schedules */
  sched_[vehl.id()] = vehl.schedule().data();

  /* Update kinetic tree node times */
  int dur = Cargo::now() - last_modified_.at(vehl.id());
  if (dur > 0) kt_.at(vehl.id())->moved(dur * Cargo::vspeed());  // (treeTaxiPath.h)

  /* Update last modified */
  last_modified_[vehl.id()] = Cargo::now();

  /* DEBUG */
  // print << "handle_vehicle(" << vehl.id() << ") final cached sched: ";
  // print << "(" << vehl.last_visited_node() << ")";
  // print_sch(sched_.at(vehl.id()));
  // print.flush();
  // print << "handle_vehicle(" << vehl.id() << ") final kt: ";
  // print_kt(kt_.at(vehl.id()));
}

void KineticTrees::end() {
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
}

void KineticTrees::listen() {
  /* Clear the index, then call base listen */
  grid_.clear();
  RSAlgorithm::listen();
}

/* The main challenge is keeping the local kinetic trees (kt_) in sync with
 * the ground-truth state of the vehicles (in Cargo::db()). Method sync_kt
 * performs synchronization between a local kt and a ground-truth vehicle
 * schedule. Let the root of the kt always be the vehicle's next node. */
void KineticTrees::sync_kt(TreeTaxiPath* kt, const std::vector<Stop>& cur_sch) {
  std::vector<std::pair<NodeId, bool>> seq;
  kt->printStopSequence(seq);
  NodeId i = (seq.begin()+1)->first;
  NodeId j = (cur_sch.begin()+1)->loc();
  while (i != j && i != -1) {
    // print << "sync_kt: " << i << " != " << j << std::endl;
    kt->step();
    // print << "sync_kt: step to "; print_kt(kt);
    i = kt->next();
  }
  // print << "sync_kt: " << i << " == " << j << std::endl;
  kt->move(cur_sch.begin()->loc());
  // print << "sync_kt: move to "; print_kt(kt);
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
  /* Set options */
  Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  op.path_to_solution = "kinetic_trees.sol";
  op.path_to_dataout  = "kinetic_trees.dat";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  /* Construct Cargo */
  Cargo cargo(op);

  /* Initialize algorithm */
  KineticTrees kt;

  /* Start the simulation */
   cargo.start(kt);

  return 0;
}

