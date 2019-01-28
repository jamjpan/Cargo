var h_template =
`// cargoweb.h
#include "libcargo.h"
// Add all includes here
#include <iostream>  // std::endl
// -----------------------------
using namespace cargo;

class CargoWeb : public RSAlgorithm {
 public:
  CargoWeb();
  virtual void handle_customer(const Customer &);
  virtual void handle_vehicle(const Vehicle &);
  virtual void match();

// YOUR CODE BELOW
  // Add additional public vars + functions here
 private:
  // Add private vars + functions here
  // Grid grid_;
};
`

var cc_template =
`// cargoweb.cc
// Notes:
// - No need to implement main(). It will be composed automatically.
// - Do not include cargoweb.h. It will be squeezed with cargoweb.cc into one file.
// - No need to put any includes here for the reason above
// - The names "CargoWeb" and "cargoweb" cannot be changed for now, sorry!!
const int PERIOD = 30;  // call the alg every PERIOD seconds

// YOUR CODE BELOW
/* Algorithm constructor */
CargoWeb::CargoWeb() : RSAlgorithm("cargoweb", true)
  // Initialize other vars
  // , grid_(100)
    {
  this->batch_time() = PERIOD;
}

/* Called every PERIOD on every waiting, non-assigned customer */
void CargoWeb::handle_customer(const Customer& cust) {
  CustId cid = cust.id();
  NodeId oid = cust.orig();
  NodeId did = cust.dest();
  // print << "Cust " << cid << " (" << oid << ", " << did << ") appeared" << std::endl;
}

/* Called every PERIOD on every active vehicle */
void CargoWeb::handle_vehicle(const Vehicle& vehl) {
  // this->grid_.insert(vehl);
  VehlId vid = vehl.id();
  NodeId loc = vehl.last_visited_node();
  // print << "Vehl " << vid << " is at " << loc << std::endl;
}

/* Called every PERIOD */
void CargoWeb::match() {
  vec_t<Customer> customers = this->customers();
  vec_t<Vehicle> vehicles = this->vehicles();
  print << "(t=" << Cargo::now() << ")"
        << " There are " << customers.size() << " custs waiting to be matched"
        << " and " << vehicles.size() << " active vehicles" << std::endl;
}

// BEGIN CUSTOM FUNCTIONS
// void CargoWeb::foo() { ... }
// END CUSTOM FUNCTIONS
`
