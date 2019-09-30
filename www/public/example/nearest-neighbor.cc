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
  CustId cid = cust.id();
  NodeId oid = cust.orig();
  NodeId did = cust.dest();
  print << "Cust " << cid << " (" << oid << ", " << did << ") appeared" << std::endl;

  // Move camera to center on the customer
  gui::center(cust.orig());

  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  // Visualize candidates
  for (const MutableVehicleSptr& cand : this->candidates)
    gui::vhi(cand->id());
  pause();

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
    // Visualize nearest match
    gui::clinev(cust.id(), best_vehl->id());
    pause();
  }

  gui::reset();
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

