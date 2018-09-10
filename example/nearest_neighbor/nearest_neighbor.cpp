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
#include <chrono>
#include <iostream>
#include <iterator>
#include <vector>

#include "nearest_neighbor.h"
#include "libcargo.h"

using namespace cargo;

const int BATCH = 1;  // seconds
const int RANGE = 1500; // meters

bool NearestNeighbor::tried(
  const VehlId & vehl_id)
{ return std::find(tried_.begin(), tried_.end(), vehl_id) != tried_.end(); }

MutableVehicleSptr NearestNeighbor::find_nearest(
  const std::vector<MutableVehicleSptr> & cands,
  const Customer & cust)
{
  auto t = hiclock::now();
  DistDbl d_nearest = InfDbl;
  MutableVehicleSptr cand_nearest = nullptr;
  for (const MutableVehicleSptr & cand : cands) {
    TIMEOUT(t)
    DistDbl d = haversine(cand->last_visited_node(), cust.orig());
    if (d < d_nearest
       && cand->queued() < cand->capacity()
       && !tried(cand->id())) {
      d_nearest = d;
      cand_nearest = cand;
    }
  }
  return cand_nearest;
}

NearestNeighbor::NearestNeighbor()
    : RSAlgorithm("nearest_neighbor", false), grid_(100)
{
  batch_time() = BATCH;
  nmat_ = 0;
}

void NearestNeighbor::handle_customer(
  const cargo::Customer& cust)
{
  start_timing();
  sch = {};
  rte = {};
  tried_ = {};
  best_vehl = nullptr;

  DistInt rng = /* cargo::pickup_range(cust, cargo::Cargo::now()); */ RANGE;
  auto candidates = grid_.within_about(rng, cust.orig());
  if (candidates.size() == 0)
    RETURN_FAIL

  while (tried_.size() != candidates.size()) {
    best_vehl = find_nearest(candidates, cust);
    if (best_vehl == nullptr)
      RETURN_FAIL

    sop_insert(best_vehl, cust, sch, rte);
    if (!chktw(sch, rte)) {
      tried_.push_back(best_vehl->id());
    } else {
      if (assign({cust.id()}, {}, rte, sch, *best_vehl)) {
        print(MessageType::Success)
          << "Match (cust" << cust.id() << ", veh" << best_vehl->id() << ")\n";
        nmat_++;
        RETURN_SUCCESS
      } else {
        tried_.push_back(best_vehl->id());
      }
    }
  }
  RETURN_FAIL
}

void NearestNeighbor::handle_vehicle(
  const cargo::Vehicle& vehl)
{ grid_.insert(vehl); }

void NearestNeighbor::end()
{
  print(MessageType::Success) << "Matches: " << nmat_ << std::endl;
  print(MessageType::Success) << "Out-of-sync rejected: " << nrej_ << std::endl;
  print(MessageType::Success) << "Avg-cust-handle: " << avg_cust_ht() << "ms" << std::endl;
}

void NearestNeighbor::listen()
{
  grid_.clear();
  RSAlgorithm::listen();
}

int main()
{
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  op.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  op.path_to_problem  = "../../data/benchmark/rs-md-7.instance";
  op.path_to_solution = "nearest_neighbor.sol";
  op.path_to_dataout  = "nearest_neighbor.dat";
  op.path_to_save     = "nearest_neighbor.bak";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 20;
  op.matching_period  = 60;
  op.static_mode      = false;

  cargo::Cargo cargo(op);
  NearestNeighbor nn;
  cargo.start(nn);
}

