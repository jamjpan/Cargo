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

// A Solution is a table keyed by vehicle containing new assignment data.
// The first vector in the pair is list of assignments; the second vector is
// list of unassignments (due to swap, replace).
typedef dict<MutableVehicle, std::pair<vec_t<Customer>, vec_t<Customer>>>
    Solution;

class GRASP : public RSAlgorithm {
 public:
  GRASP();

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  std::mt19937 gen;
  std::uniform_real_distribution<> d;

  int nswap_;
  int nreplace_;
  int nrearrange_;
  int nnoimprov_;

  /* Workspace variables */
  dict<VehlId, vec_t<Customer>> candidates_list;
  dict<CustId, DistInt> range;
  dict<VehlId, MutableVehicleSptr> vehicles_lookup;

  tick_t timeout_0, timeout_1;

  dict<CustId, bool> is_matched;

  Solution initialize(Grid &);
  Solution replace(const Solution &, Grid &);
  Solution swap(const Solution &, Grid &);
  Solution rearrange(const Solution &);
  Customer roulette_select(const dict<Customer, int> &);
  void commit(const Solution &);

  DistInt solcost(const Solution &);
  bool verify(const Solution &);

  void print_sol(const Solution &);
};

