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
#include "libcargo.h"

using namespace cargo;

typedef vec_t<MutableVehicle> Assignment;
typedef vec_t<std::pair<DistInt, Assignment>> Population;

class Genetic : public RSAlgorithm {
 public:
  Genetic();

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  std::mt19937 gen;
  std::uniform_real_distribution<> d;

  /* Workspace variables */
  vec_t<Stop> sch;
  vec_t<Wayp> rte;
  vec_t<MutableVehicleSptr> candidates;
  tick_t timeout_0;
  dict<CustId, bool> is_matched;
  std::pair<DistInt, Assignment> best_sol;
  dict<VehlId, std::vector<CustId>> commit_cadd;
  dict<VehlId, std::vector<Wayp>> commit_rte;
  dict<VehlId, std::vector<Stop>> commit_sch;

  DistInt cost(const Assignment &);
  Assignment extract_fit(Population &);
  void crossover(Assignment &, Assignment &);
  void repair(Assignment &);

  void print_population(const Population &);
  void print_solution(const std::pair<DistInt, Assignment> &);

  void reset_workspace();
};

