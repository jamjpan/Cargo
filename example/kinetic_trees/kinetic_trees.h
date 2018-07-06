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
#include <unordered_map>

#include "libcargo.h"

#include "treeTaxiPath.h"

using namespace cargo;

// Implements greedy assignment via kinetic trees
// Citation:
//   Y. Huang et al. "Large Scale Real-time Ridesharing with Service Guarantee
//   on Road Networks". VLDB 2014.
// Adapted from the author code:
// https://github.com/uakfdotb/tsharesim
class KineticTrees : public RSAlgorithm {
 public:
  KineticTrees();

  /* My Overrides */
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void end();
  virtual void listen();

 private:
  /* My Custom Variables */
  int nmat_;
  cargo::Grid grid_;

  std::unordered_map<VehlId, TreeTaxiPath*>     kt_;
  std::unordered_map<VehlId, std::vector<Stop>> sched_;
  std::unordered_map<VehlId, SimlTime>          last_modified_;

  int compute_steps(const std::vector<Stop> &, const std::vector<Stop> &);
};

