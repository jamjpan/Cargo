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

#include "grabby.h"
#include "libcargo.h"

using namespace cargo;

Grabby::Grabby(const std::string& name) : RSAlgorithm(name, false) {
  this->batch_time() = 30;
  this->k = 10;
}

void Grabby::handle_customer(const Customer& cust) {
  print << "Handling cust " << cust.id() << std::endl;
  vec_t<Stop> temp_sched, greedy_sched;
  vec_t<Wayp> temp_route, greedy_route;
  MutableVehicleSptr greedy_cand = nullptr;

  // <1. Get top-k veihcles>
  print << "\tRanking top-k..." << std::endl;
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
  print << "\tComputing greedy..." << std::endl;
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

  if (greedy_cand) {
    print << "\tMatched with vehl " << greedy_cand->id() << std::endl;
    this->assign(
      {cust.id()}, {}, greedy_route, greedy_sched, *greedy_cand);
  }
}

