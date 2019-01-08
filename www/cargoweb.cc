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
// cargoweb.cc
// Notes:
// - No need to implement main(). It will be composed automatically.
// - Do not include cargoweb.h. It will be squeezed with cargoweb.cc into one file.
// - No need to put any includes here for the reason above
// - The names "CargoWeb" and "cargoweb" cannot be changed for now, sorry!!
const int PERIOD = 30;  // call the alg every PERIOD seconds

// YOUR CODE BELOW
auto cmp = [](rank_cand left, rank_cand right) {
  return std::get<0>(left) > std::get<0>(right);
};
/* Algorithm constructor */
CargoWeb::CargoWeb() : RSAlgorithm("cargoweb", true)
  // Initialize other vars
  , grid_(100)
    {
  this->batch_time() = PERIOD;
}

/* Called every PERIOD on every waiting, non-assigned customer */
void CargoWeb::handle_customer(const Customer& cust) {
  // CustId cid = cust.id();
  // NodeId oid = cust.orig();
  // NodeId did = cust.dest();
  // print << "Cust " << cid << " (" << oid << ", " << did << ") appeared" << std::endl;

  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  std::priority_queue<rank_cand, vec_t<rank_cand>, decltype(cmp)> my_q(cmp);
  for (const MutableVehicleSptr& cand : this->candidates) {
    DistDbl cost = haversine(cand->last_visited_node(), cust.orig());
    rank_cand rc = std::make_pair(cost, cand);
    my_q.push(rc);
  }

  while (!my_q.empty() && !matched) {
    rank_cand rc = my_q.top();
    my_q.pop();
    best_vehl = std::get<1>(rc);
    sop_insert(best_vehl, cust, sch, rte);
    if (chktw(sch, rte)
     && chkcap(best_vehl->capacity(), sch))
      matched = true;
  }

  if (matched) {
    this->assign({cust.id()}, {}, rte, sch, *best_vehl);
  }
}

/* Called every PERIOD on every active vehicle */
void CargoWeb::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
  // VehlId vid = vehl.id();
  // NodeId loc = vehl.last_visited_node();
  // print << "Vehl " << vid << " is at " << loc << std::endl;
}

/* Called every PERIOD */
void CargoWeb::match() {
  // vec_t<Customer> customers = this->customers();
  // vec_t<Vehicle> vehicles = this->vehicles();
  // print << "(t=" << Cargo::now() << ")"
  //       << " There are " << customers.size() << " custs waiting to be matched"
  //       << " and " << vehicles.size() << " active vehicles" << std::endl;
}

/* Called at end of simulation */
void CargoWeb::end() {
  this->print_statistics();
}

// BEGIN CUSTOM FUNCTIONS
// void CargoWeb::foo() { ... }
void CargoWeb::reset_workspace() {
  this->sch = {};
  this->rte = {};
  this->candidates = {};
  this->matched = false;
  this->best_vehl = nullptr;
}
// END CUSTOM FUNCTIONS

//
        int main() {
          Options option;
          option.path_to_roadnet    = "data/bj5.rnet";
          option.path_to_gtree      = "data/bj5.gtree";
          option.path_to_edges      = "data/bj5.edges";
          option.path_to_problem    = "data/rs-bj5-m5k-c3-d6-s10-x1.0.instance";
          option.path_to_solution   = "cargoweb.sol";
          option.path_to_dataout    = "cargoweb.dat";
          option.time_multiplier    = 1;
          option.vehicle_speed      = 10;
          option.matching_period    = 60;
          option.strict_mode        = true;
          option.static_mode        = false;
          Cargo cargo(option);
          CargoWeb cw;
          cargo.start(cw);
          return 0;
        }
        