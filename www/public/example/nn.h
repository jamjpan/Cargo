// cargoweb.h
#include "libcargo.h"
// Add all includes here
#include <iostream>  // std::endl
// -----------------------------
using namespace cargo;

typedef std::pair<DistDbl, MutableVehicleSptr> rank_cand;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();
  virtual void end();

// YOUR CODE BELOW
  // Add additional public vars + functions here
 private:
  // Add private vars + functions here
  Grid grid_;
  /* Workspace variables */
  vec_t<Stop> sch;
  vec_t<Wayp> rte;
  MutableVehicleSptr best_vehl;
  vec_t<MutableVehicleSptr> candidates;
  bool matched;
  tick_t timeout_0;

  void reset_workspace();
};
