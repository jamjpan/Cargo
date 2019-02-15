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
#include <memory>
#include <tuple>
#include <unordered_map>
#include <random>

#include "libcargo.h"

using namespace cargo;

typedef dict<MutableVehicleSptr, std::pair<vec_t<CustId>, vec_t<CustId>>>
Solution;

class GRASP : public RSAlgorithm {
 public:
  GRASP(const std::string &);
  GRASP(const std::string &, const int &);

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  int max_iter;

  std::mt19937 gen;
  std::uniform_real_distribution<> d;

  int nswap_;
  int nreplace_;
  int nrearrange_;
  int nnoimprov_;

  tick_t timeout_0;

  Solution best_solution;
  DistInt  best_cost;

  dict<Customer, std::unordered_set<MutableVehicleSptr>> candidates_index;
  dict<MutableVehicleSptr, vec_t<Customer>> candidates_list;
  dict<MutableVehicleSptr, dict<Customer, DistInt>> fitness;
  dict<MutableVehicleSptr, dict<Customer, vec_t<Stop>>> schedules;
  dict<MutableVehicleSptr, dict<Customer, vec_t<Wayp>>> routes;
  vec_t<Stop> sch;
  vec_t<Wayp> rte;

  Solution initialize(Grid &, vec_t<Customer> &);
  Solution replace(const Solution &, vec_t<Customer> &);
  Solution swap(const Solution &);
  Solution rearrange(const Solution &);
  Customer roulette(const dict<Customer, DistInt> &);
  DistInt cost(const Solution &);

  void construct(const int &);

  void print_sol(const Solution &);

  void commit(const Solution &);

  void reset_workspace() {
    this->candidates_index = {};
    this->candidates_list = {};
    this->fitness = {};
    this->schedules = {};
    this->routes = {};
    this->sch = {};
    this->rte = {};
  }

};

