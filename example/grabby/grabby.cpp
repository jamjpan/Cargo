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
#include <iostream>
#include <memory>
#include <tuple>
#include <vector>

#include "grabby.h"
#include "libcargo.h"

using namespace cargo;

Grabby::Grabby() : RSAlgorithm("grabby", false) {
  this->batch_time() = 30;
  this->k = 10;
}

void Grabby::handle_customer(const Customer& cust) {
  vec_t<Stop> temp_sched, greedy_sched;
  vec_t<Wayp> temp_route, greedy_route;
  MutableVehicleSptr greedy_cand = nullptr;

  // <1. Get top-k veihcles>
  // a. Initialize top-k
  vec_t<std::pair<DistInt, size_t>> top;
  for (size_t i = 0; i < k; i++) {
    NodeId u = cust.orig();
    NodeId v = this->vehicles().at(i).last_visited_node();
    top.push_back(std::make_pair(haversine(u, v), i));
  }
  // b. Check remaining vehicles
  for (size_t i = k; i < this->vehicles().size(); i++) {
    auto kth = std::max_element(top.begin(), top.end());
    NodeId u = cust.orig();
    NodeId v = this->vehicles().at(i).last_visited_node();
    DistInt dist = haversine(u, v);
    if (dist < kth->first)
      *kth = std::make_pair(dist, i);
  }

  // <2. Select the greedy vehicle>
  DistInt cost_min = InfInt;
  for (const auto& pair : top) {
    MutableVehicle cand(this->vehicles().at(pair.second));
    DistInt cost_old = cand.route().cost();
    DistInt cost_new = sop_insert(
      cand, cust, temp_sched, temp_route);
    DistInt cost = cost_new - cost_old;
    if (cost < cost_min && chkcap(cand.capacity(), temp_sched)
     && chktw(temp_sched, temp_route)) {
      cost_min = cost;
      greedy_cand  = std::make_shared<MutableVehicle>(cand);
      greedy_sched = std::move(temp_sched);
      greedy_route = std::move(temp_route);
    }
  }

  if (greedy_cand)
    this->assign(
      {cust.id()}, {}, greedy_route, greedy_sched, *greedy_cand);
}

int main() {
  Options option;
  option.path_to_roadnet  = "../../data/roadnetwork/bj5.rnet";
  option.path_to_gtree    = "../../data/roadnetwork/bj5.gtree";
  option.path_to_edges    = "../../data/roadnetwork/bj5.edges";
  option.path_to_problem  = "../../data/benchmark/rs-bj5-m5k-c3-d6-s10-x1.0.instance";
  option.path_to_solution = "grabby.sol";
  option.path_to_dataout  = "grabby.dat";
  option.time_multiplier  = 1;
  option.vehicle_speed    = 10;
  option.matching_period  = 60;
  option.strict_mode      = false;
  option.static_mode      = false;
  Cargo cargo(option);
  Grabby gr;
  cargo.start(gr);
  return 0;
}

