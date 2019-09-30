
CargoWeb::CargoWeb()
    : RSAlgorithm("cargoweb", true), grid_(100), d(0,1) {
  this->batch_time() = BATCH;
  this->nclimbs_ = 0;
  this->ntries_ = 0;
  this->nanneals_ = 0;
  this->ntimeouts_ = 0;
  this->ncap_ = 0;
  this->ntw_ = 0;
  this->t_max = tmax;
  this->p_max = pmax;
  this->f_ = static_cast<float>(f/100.0);
  print << "Set f to " << this->f_ << std::endl;
  print << "Set tmax to " << this->t_max << std::endl;
  print << "Set pmax to " << this->p_max << std::endl;
  std::random_device rd;
  this->gen.seed(rd());
}

void CargoWeb::match() {
  this->reset_workspace();

  Grid local_grid(this->grid_);  // make a deep copy

  this->initialize(local_grid);

  // Visualize initial assignments
  for (const auto& kv1 : this->sol) {
    for (const Customer& cust : kv1.second.second) {
      gui::chi(cust.orig());
      gui::clinev(cust.id(), kv1.first);
      // gui::newroute(kv1.second.first.route());
    }
  }
  pause();


  if (!this->sol.empty()) {
    this->anneal(t_max, p_max);
    // print(MessageType::Info) << "after anneal: " << this->sol_cost(this->sol) << std::endl;
    this->commit();
  } else {
    // print(MessageType::Warning) << "empty initial solution" << std::endl;
  }

  gui::reset();
}

void CargoWeb::initialize(Grid& local_grid) {
  // Assign each customer to a random candidate
  for (const Customer& cust : this->customers()) {
    // print << "initialize() working on cust " << cust.id() << std::endl;
    bool initial = false;
    //this->tried[cust.id()] = {};
    vec_t<MutableVehicleSptr> candidates
      = this->candidates_list[cust.id()]
      = local_grid.within(pickup_range(cust), cust.orig());

    // Add to local vehicle lookup (we need it during perturb)
    for (const MutableVehicleSptr cand : candidates)
      this->vehicle_lookup[cand->id()] = cand;

    // print << "  got " << candidates.size() << " cands" << std::endl;

    // Randomize the candidates order
    //std::shuffle(candidates.begin(), candidates.end(), this->gen);

    // Try until got a valid one
    while (!candidates.empty() && initial == false) {
      auto k = candidates.begin();
      std::uniform_int_distribution<> m(0, candidates.size()-1);
      std::advance(k, m(this->gen));
      MutableVehicleSptr cand = *k;
      candidates.erase(k);
      //MutableVehicleSptr cand = candidates.back();
      //candidates.pop_back();

      // print << "    trying vehl " << cand->id() << std::endl;

      // Speed-up heuristic!
      // Try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < SCHED_MAX) {
        sop_insert(*cand, cust, sch, rte);
        if (chkcap(cand->capacity(), sch) && chktw(sch, rte)) {
          cand->set_sch(sch);  // update grid version of the candidate
          cand->set_rte(rte);
          cand->reset_lvn();
          if (this->sol.count(cand->id()) == 0) {
            vec_t<Customer> assignments = {};
            this->sol[cand->id()] = std::make_pair(*cand, assignments);
          }
          this->sol.at(cand->id()).first = *cand;  // update our copy of the candidate
          this->sol.at(cand->id()).second.push_back(cust);
          // print << "    initialized cust " << cust.id() << " to cand " << cand->id() << "; assignments=";
          // for (const auto& c : this->sol.at(cand->id()).second) print << c.id() << ", ";
          // print << std::endl;
          // print << "    sched: " << cand->schedule() << std::endl;
          initial = true;
        } else {
          // print << "      skipping due to infeasible" << std::endl;
        }
      } else {
        // print << "      skipping due to sched_max" << std::endl;
      }
    }
  }
}

SASol CargoWeb::perturb(const SASol& sol,
                                     const int& temperature) {
  // Due to structure of SASol, when we "randomly select a customer" we
  // actually randomly select a vehicle, then choose a customer assigned to
  // that vehicle. We then need to make sure our solution never has "empty"
  // vehicles with no assignments, otherwise some perturbs will be wasted.

  std::uniform_int_distribution<>::param_type sol_size_range(1, sol.size());
  this->n.param(sol_size_range);
  auto i = sol.cbegin();                    // 1. Select random vehicle
  std::advance(i, this->n(this->gen)-1);
  if (i->second.second.empty()) {  // this should never happen
    // print << "perturb got vehicle " << i->second.first.id() << " with no assignments; returning" << std::endl;
    // print_sol(sol);
    pause();
    return sol;
  }
  MutableVehicle  k_old = i->second.first;
  vec_t<Customer> k_old_assignments = i->second.second;
  // print << "perturb got vehl " << k_old.id() << std::endl;

  auto j = k_old_assignments.cbegin();      // 2. Select random assigned customer
  std::uniform_int_distribution<> m(1, k_old_assignments.size());
  std::advance(j, m(this->gen)-1);
  Customer cust_to_move = *j;
  // print << "  perturb got cust " << cust_to_move.id() << std::endl;

  vec_t<MutableVehicleSptr> candidates = this->candidates_list.at(cust_to_move.id());
  if (candidates.empty()) {
    // print << "  no candidates; returning" << std::endl;
    return sol;
  }

  auto k = candidates.cbegin();             // 3. Select new candidate
  std::uniform_int_distribution<> p(1, candidates.size());
  std::advance(k, p(this->gen)-1);
  // print << "  perturb got new cand " << (*k)->id() << std::endl;

  //while (((*k)->id() == k_old.id() || this->tried.at(cust_to_move.id()).count((*k)->id()) == 1)
  //    && !candidates.empty()) {
  while ((*k)->id() == k_old.id()) {
    candidates.erase(k);
    if (candidates.empty()) {
      // print << "  no untried candidates; returning" << std::endl;
      return sol;
    }
    k = candidates.cbegin();
    std::uniform_int_distribution<> p(0, candidates.size()-1);
    std::advance(k, p(this->gen));
    // print << " perturb got new cand " << (*k)->id() << std::endl;
  }
  //if (k == candidates.end()) return sol;

  //this->tried[cust_to_move.id()].insert((*k)->id());
  MutableVehicle k_new = **k;
  // We don't want to try this candidate again in later perturbs, so just remove
  // it from the candidates list
  //candidates.erase(k);

  vec_t<Customer> k_new_assignments = {};
  if (sol.find(k_new.id()) != sol.end())
    k_new_assignments = sol.at(k_new.id()).second;

  // 4. Do the move
  DistInt current_cost = k_old.route().cost() + k_new.route().cost();

  //   a. Add cust to k_new (--bottleneck--)
  sop_insert(k_new, cust_to_move, this->sch_after_add, this->rte_after_add);
  // print << "  move " << cust_to_move.id() << " from " << k_old.id() << " to " << k_new.id() << std::endl;

  //   b. Accept or reject
  if (chkcap(k_new.capacity(), this->sch_after_add)) {
    this->ncap_++;
    if (chktw(this->sch_after_add, this->rte_after_add)) {
      this->ntw_++;
      //   c. Remove cust from k_old
      this->sch_after_rem = k_old.schedule().data();
      opdel(this->sch_after_rem, cust_to_move.id());
      route_through(this->sch_after_rem, this->rte_after_rem);

      //   d. Compare costs
      DistInt new_cost = this->rte_after_add.back().first + rte_after_rem.back().first;
      bool climb = false;
      // print << "    is " << new_cost << " < " << current_cost << "?" << std::endl;
      this->ntries_++;
      // Quality heuristic: don't do any climbs on the last temperature
      if (new_cost >= current_cost && temperature != 1) {
        climb = hillclimb(temperature);
        if (climb) this->nclimbs_++;
      }
      if (new_cost < current_cost || climb) {
        // print << (new_cost < current_cost ? "  accept" : " accept due to climb") << std::endl;
        // print << "  sched for " << k_old.id() << ": " << sch_after_rem << std::endl;
        // print << "  sched for " << k_new.id() << ": " << sch_after_add << std::endl;
        // Update grid
        vehicle_lookup.at(k_old.id())->set_sch(sch_after_rem);
        vehicle_lookup.at(k_old.id())->set_rte(rte_after_rem);
        vehicle_lookup.at(k_old.id())->reset_lvn();
        vehicle_lookup.at(k_new.id())->set_sch(sch_after_add);
        vehicle_lookup.at(k_new.id())->set_rte(rte_after_add);
        vehicle_lookup.at(k_new.id())->reset_lvn();

        // Update solution
        k_old.set_sch(sch_after_rem);
        k_old.set_rte(rte_after_rem);
        k_old.reset_lvn();
        k_old_assignments.erase(j);
        k_new.set_sch(sch_after_add);
        k_new.set_rte(rte_after_add);
        k_new.reset_lvn();
        k_new_assignments.push_back(cust_to_move);
        SASol improved_sol = sol;
        improved_sol[k_new.id()] = std::make_pair(k_new, k_new_assignments);
        if (k_old_assignments.empty())
          improved_sol.erase(k_old.id());  // erase by key
        else
          improved_sol[k_old.id()] = std::make_pair(k_old, k_old_assignments);
        return improved_sol;
      } else {
        // print << "    reject (greater cost)" << std::endl;
      }
    } else {
      // print << "    reject (failed chktw)" << std::endl;
    }
  } else {
    // print << "    reject (failed chkcap)" << std::endl;
  }
  return sol;
}

void CargoWeb::commit() {
  for (const auto& kv : this->sol) {
    MutableVehicle cand = kv.second.first;
    vec_t<CustId> cadd = {};
    vec_t<CustId> cdel = {};
    for (const Customer& cust : kv.second.second) cadd.push_back(cust.id());
    if (this->assign(cadd, cdel, cand.route().data(), cand.schedule().data(), cand)) {
      // for (const CustId& cid : cadd)
       // print << "Matched " << cid << " with " << cand.id() << std::endl;
    } else {
      // for (const CustId& cid : cadd)
        // print(MessageType::Warning) << "Rejected due to sync " << cid << " with " << cand.id() << std::endl;
    }
  }
}

void CargoWeb::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void CargoWeb::end() {
  print(MessageType::Info) << "climbs/tries: " << nclimbs_ << "/" << ntries_ << std::endl;
  print(MessageType::Info) << "anneals: " << nanneals_ << std::endl;
  print(MessageType::Info) << "timeouts: " << ntimeouts_ << std::endl;
  print(MessageType::Info) << "ncap: " << ncap_ << std::endl;
  print(MessageType::Info) << "ntw: " << ntw_ << std::endl;
  RSAlgorithm::end();
}

void CargoWeb::listen(bool skip_assigned, bool skip_delayed) {
  this->grid_.clear();
  RSAlgorithm::listen(skip_assigned, skip_delayed);
}

void CargoWeb::print_sol(const SASol& sol) {
  for (const auto& kv : sol) {
    print << "Vehl " << kv.first << " | ";
    for (const auto& cust : kv.second.second) {
      print << cust.id() << " ";
    }
    print << std::endl;
  }
}

void CargoWeb::reset_workspace() {
  this->sol = {};
  //this->tried = {};
  this->sch = this->sch_after_rem = this->sch_after_add = {};
  this->rte = this->rte_after_rem = this->rte_after_add = {};
  this->candidates_list = {};
  this->vehicle_lookup = {};
  this->timeout_0 = hiclock::now();
  this->best_sol = {};
  this->commit_cadd = {};
  this->commit_rte = {};
  this->commit_sch = {};
}

