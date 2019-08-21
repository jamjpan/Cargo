#include <algorithm>
#include <iostream>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

#include "libcargo.h"

using namespace cargo;

typedef std::pair<DistInt, MutableVehicleSptr> ba_cand;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;
  int nswapped_;

  dict<CustId, vec_t<ba_cand>> lookup;
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

