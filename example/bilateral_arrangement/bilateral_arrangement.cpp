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
#include <thread>
#include <tuple>
#include <vector>

#include "bilateral_arrangement.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH     = 1;  // seconds
const int RETRY     = 15; // seconds
const int TIMEOUT   = 1;  // timeout customers take > TIMEOUT sec

/* Define ordering of rank_cands */
auto cmp = [](rank_cand left, rank_cand right) {
  return std::get<0>(left) > std::get<0>(right); };

bool BilateralArrangement::timeout(clock_t& start) {
  clock_t end = std::clock();
  double elapsed = double(end-start)/CLOCKS_PER_SEC;
  if ((elapsed >= TIMEOUT) || this->done()) {
    print << "timeout() triggered." << std::endl;
    return true;
  }
  return false;
}

BilateralArrangement::BilateralArrangement() : RSAlgorithm("bilateral_arrangement"),
      grid_(100) {   // (grid.h)
  batch_time() = BATCH;  // (rsalgorithm.h)
  nswapped_ = 0;  // number swapped and accepted
  nrej_     = 0;  // number rejected due to out-of-sync
  delay_    = {}; // delay container
}

void BilateralArrangement::handle_vehicle(const cargo::Vehicle& vehl) {
  /* Insert each vehicle into the grid */
  grid_.insert(vehl);
}

void BilateralArrangement::match() {
  /* BilateralArrangement is a random algorithm due to this shuffle. */
  std::random_shuffle(customers().begin(), customers().end());

  /* Extract riders one at a time */
  while (!customers().empty()) {
    clock_t start = std::clock();
    Customer cust = customers().back();
    customers().pop_back();

    /* Skip customers already assigned (but not yet picked up) */
    if (cust.assigned())
      continue;

    /* Skip customers looked at within the last RETRY seconds */
    if (delay_.count(cust.id()) && delay_.at(cust.id()) >= Cargo::now() - RETRY)
      continue;

    /* Containers for storing outputs */
    cargo::DistInt cst = cargo::InfInt;
    std::vector<cargo::Stop> sch, best_sch;
    std::vector<cargo::Wayp> rte, best_rte;
    CustId removed_cust = -1;

    /* best_vehl will point to an underlying MutableVehicle in our grid */
    std::shared_ptr<MutableVehicle> best_vehl = nullptr;
    bool matched = false;

    /* Get candidates from the local grid index */
    DistInt rng = /* pickup_range(cust, Cargo::now()); */ 1200;
    auto candidates = grid_.within_about(rng, cust.orig());  // (grid.h)

    /* Container to rank candidates by least cost */
    std::priority_queue<rank_cand, std::vector<rank_cand>, decltype(cmp)> q(cmp);

    /* Loop through and rank each candidate */;
    for (const auto& cand : candidates) {
      cst = sop_insert(*cand, cust, sch, rte); // doesn't check time/cap constraints
      q.push({cst, cand, sch, rte});
    }

    /* Process each candidate, starting from least-cost, to see if can accept
     * the customer within constraints. If cannot, try to replace an existing
     * customer in the candidate's schedule with this customer, and try again. */
    while (!q.empty() && !matched) {
      /* Get and unpack the best candidate */
      auto cand = q.top(); q.pop();
      best_vehl = std::get<1>(cand);
      best_sch  = std::get<2>(cand);
      best_rte  = std::get<3>(cand);

      /* If best vehicle is within constraints... */
      bool within_time = chktw(best_sch, best_rte);
      bool within_cap = (best_vehl->queued() < best_vehl->capacity());
      if (within_time && within_cap) {
        /* ... accept the match */
        matched = true;
      } else {
        /* ... otherwise, remove some random not-picked-up customer from cand
         * and try the insertion again. If it meets constraints, then accept. */
        CustId remove_me = randcust(best_vehl->schedule().data());
        if (remove_me != -1) {
          std::vector<Stop> old_sch = best_vehl->schedule().data(); // make a backup
          std::vector<Stop> new_sch;
          std::vector<Wayp> new_rte;
          /* Replace remove_me with cust to produce new_sch, new_rte */
          sop_replace(best_vehl, remove_me, cust, new_sch, new_rte);
          /* If replacement is within constraints... */
          if (chktw(new_sch, new_rte)) {
            /* ... accept the match */
            print(MessageType::Info)
              << "Vehicle " << best_vehl->id()
              << " feasible after remove " << remove_me << std::endl;
            nswapped_++;  // increment counter
            best_sch = new_sch;
            best_rte = new_rte;
            matched = true;
            removed_cust = remove_me;
            matched_[remove_me] = false;
          } else {
            /* ... restore the backup */
            best_vehl->set_sch(old_sch);  // restore the backup
          }
        }
      }
      if (timeout(start))
        break;
    } // end while !q.empty()

    /* Commit to db */
    bool add_to_delay = true;
    if (matched) {
      std::vector<CustId> cust_to_del {};
      if (removed_cust != -1) cust_to_del.push_back(removed_cust);

      /* assign() will modify best_vehl with the synchronized version to
       * account for match latency */
      if (assign({cust.id()}, cust_to_del, best_rte, best_sch, *best_vehl)) {
        print(MessageType::Success)
          << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")"
          << std::endl;
        matched_[cust.id()] = true;
        /* Remove customer from delay storage */
        if (delay_.count(cust.id())) delay_.erase(cust.id());
        add_to_delay = false;
      } else
        nrej_++;  // increment rejected counter
    }
    if (add_to_delay) {
      /* Add customer to delay storage */
      delay_[cust.id()] = Cargo::now();
    }
  } // end while !customers().empty()
}

void BilateralArrangement::end() {
  /* Print the statistics */
  nmat_ = 0;  // (rsalgorithm.h)
  for (const auto& kv : matched_)
    if (kv.second == true) nmat_++;
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Swapped: " << nswapped_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
}

void BilateralArrangement::listen() {
  /* Clear the index, then call base listen */
  grid_.clear();
  RSAlgorithm::listen();
}

int main() {
  /* Set options */
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  op.path_to_solution = "bilateral_arrangement.sol";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;

  /* Construct Cargo */
  cargo::Cargo cargo(op);

  /* Initialize algorithm */
  BilateralArrangement ba;

  /* Start the simulation */
  cargo.start(ba);

  return 0;
}

