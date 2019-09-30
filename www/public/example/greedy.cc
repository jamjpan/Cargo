CargoWeb::CargoWeb() : RSAlgorithm("cargoweb", true), grid_(100) {
  this->batch_time() = 30;
}

void CargoWeb::handle_customer(const Customer& cust) {
  this->reset_workspace();
  this->candidates = this->grid_.within(pickup_range(cust), cust.orig());

  for (const MutableVehicleSptr& cand : this->candidates) {
    // Speed heuristic: try only if vehicle's current schedule has < 8 customer stops
    if (cand->schedule().data().size() < 10) {
      DistInt cost = sop_insert(*cand, cust, sch, rte, Cargo::gtree()) - cand->route().cost();
      if (cost < this->best_cost) {
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          this->best_vehl = cand;
          this->best_sch = sch;
          this->best_rte = rte;
          this->best_cost = cost;
        }
      }
    }
    if (this->timeout(this->timeout_0))
      break;
  }
  if (this->best_vehl != nullptr)
    this->matched = true;

  if (this->matched) {
    // print << "Matched " << cust.id() << " with " << this->best_vehl->id() << std::endl;
    // Visualize match
    gui::center(cust.orig());
    for (const MutableVehicleSptr& cand : this->candidates)
      gui::vhi(cand->id());
    gui::clinev(cust.id(), this->best_vehl->id());
    gui::curroute(this->best_vehl->route());
    gui::newroute(this->best_rte);
    pause();

    this->assign(
      {cust.id()}, {}, this->best_rte, this->best_sch, *(this->best_vehl));
  }

  gui::reset();
}

void CargoWeb::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void CargoWeb::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void CargoWeb::reset_workspace() {
  this->best_cost = InfInt;
  this->sch = best_sch = {};
  this->rte = best_rte = {};
  this->best_vehl = nullptr;
  this->candidates = {};
  this->matched = false;
}

