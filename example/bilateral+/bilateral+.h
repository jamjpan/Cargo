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

typedef std::pair<DistInt, MutableVehicleSptr> rank_cand;

class BilateralPlus : public RSAlgorithm {
 public:
  BilateralPlus(const std::string &);

  /* My overrides */
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;
  int nswapped_;

  dict<CustId, vec_t<rank_cand>> lookup;
  dict<CustId, dict<VehlId, vec_t<Stop>>> schedules;
  dict<CustId, dict<VehlId, vec_t<Wayp>>> routes;
  dict<MutableVehicleSptr, bool> modified;
  dict<MutableVehicleSptr, vec_t<CustId>> to_assign;
  dict<MutableVehicleSptr, vec_t<CustId>> to_unassign;
  vec_t<Stop> retry_sch, replace_sch;
  vec_t<Wayp> retry_rte, replace_rte;

  /* Workspace variables */
  tick_t timeout_0;

  void prepare();
  void clear();
  void commit(const MutableVehicleSptr &);
};

