#include <algorithm>
#include <iostream>
#include <queue>
#include <tuple>
#include <vector>

#include "libcargo.h"

using namespace cargo;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void listen(bool skip_assigned = true, bool skip_delayed = true);

 private:
  Grid grid_;

  /* Workspace variables */
  DistInt best_cost;
  vec_t<Stop> sch, best_sch;
  vec_t<Wayp> rte, best_rte;
  MutableVehicleSptr best_vehl;
  vec_t<MutableVehicleSptr> candidates;
  bool matched;
  tick_t timeout_0;

  void reset_workspace();
};

