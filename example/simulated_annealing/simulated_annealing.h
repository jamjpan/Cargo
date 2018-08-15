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
#include <random>

#include "libcargo.h"

using namespace cargo;

/* Annealing a metal is the process of repeatedly heating/cooling it
 * in order to get rid of impurities. For Simulated Annealing, we first
 * generate a solution out of the unmatched customers and vehicles; then
 * perturb the solution; depending on the "temperature", we accept perturbed
 * solutions of higher or lower cost than the original solution.
 *
 * Here we use "random reassign" as our perturbation. We generate an
 * initial solution by looping through all customers and greedily assigning
 * them; we generate a "perturbed" solution by shuffling the customers, then
 * again greedily assign. */
class SimulatedAnnealing : public RSAlgorithm {
 public:
  SimulatedAnnealing();

  /* My Overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen();

 private:
  /* My Custom Variables */
  int nmat_;
  int nrej_;
  Grid grid_;

  int T_;
  std::mt19937 gen;
  std::uniform_real_distribution<> d;

  typedef std::tuple<std::shared_ptr<MutableVehicle>, std::vector<Stop>, std::vector<Wayp>> Assignment;
  typedef std::unordered_map<CustId, Assignment> Sol;
  std::vector<CustId> unassigned;

  /* Probability of accepting "hill-climbing" solution */
  bool hillclimb(int &);

  /* Compute solution cost to evaluate which sol is better */
  int cost(const Sol &);

  /* If a customer doesn't get matched right away, try again after 5 seconds. */
  std::unordered_map<CustId, SimlTime> delay_;
};

