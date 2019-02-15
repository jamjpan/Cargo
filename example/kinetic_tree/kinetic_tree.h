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
#include "libcargo.h"
#include "treeTaxiPath.h"

/* Citation:
 *   Y. Huang et al. "Large Scale Real-time Ridesharing with Service Guarantee
 *   on Road Networks". VLDB 2014.
 * Author code: https://github.com/uakfdotb/tsharesim
 */
using namespace cargo;

class KineticTrees : public RSAlgorithm {
 public:
  KineticTrees(const std::string &);
  ~KineticTrees();

  /* My overrides */
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  cargo::Grid grid_;

  /* Workspace variables */
  std::vector<Stop> sch, best_sch;
  std::vector<Wayp> rte, best_rte;
  MutableVehicleSptr best_vehl;
  std::vector<MutableVehicleSptr> candidates;
  bool matched;
  tick_t timeout_0;

  /* Local containers for vehicle state */
  std::unordered_map<VehlId, TreeTaxiPath*>     kt_;
  std::unordered_map<VehlId, std::vector<Stop>> sched_;
  std::unordered_map<VehlId, SimlTime>          last_modified_;

  void sync_kt(TreeTaxiPath *, const std::vector<Stop> &);

  std::vector<Stop> kt2sch(const MutableVehicleSptr &, const Stop &,
                           const Stop &);

  void reset_workspace();
  void print_kt(TreeTaxiPath *, bool temp = false);
};

