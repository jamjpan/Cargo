const int PERIOD = 3;  // T (sec)
const int K = 10;  // top-k

CargoWeb::CargoWeb() : RSAlgorithm("cargoweb", true) {
  this->batch_time() = PERIOD;
  this->k = K;
}

void CargoWeb::handle_customer(const Customer& cust) {
  print << "Handling cust " << cust.id() << std::endl;

  // Move camera to center on the customer
  gui::center(cust.orig());

  vec_t<Stop> temp_sched, greedy_sched;
  vec_t<Wayp> temp_route, greedy_route;
  MutableVehicleSptr greedy_cand = nullptr;

  // <1. Get top-k veihcles>
  print << "\tRanking top-k..." << std::endl;
  // a. Initialize top-k
  vec_t<std::pair<DistInt, size_t>> top;
  for (size_t i = 0; i < k; i++) {
    NodeId u = cust.orig();
    NodeId v = this->vehicles().at(i).last_visited_node();
    top.push_back(std::make_pair(haversine(u, v), i));
  }
  // b. Check remaining vehicles
  for (size_t i = k; i < this->vehicles().size(); i++) {
    auto kth = std::max_element(top.begin(), top.end());
    NodeId u = cust.orig();
    NodeId v = this->vehicles().at(i).last_visited_node();
    DistInt dist = haversine(u, v);
    if (dist < kth->first)
      *kth = std::make_pair(dist, i);
  }

  // Visualize top-k
  for (const auto& pair : top)
    gui::clinev(cust.id(), this->vehicles().at(pair.second).id());
  pause();

  // <2. Select the greedy vehicle>
  print << "\tComputing greedy..." << std::endl;
  DistInt cost_min = InfInt;
  for (const auto& pair : top) {
    MutableVehicle cand(this->vehicles().at(pair.second));
    DistInt cost_old = cand.route().cost();
    DistInt cost_new = sop_insert(
      cand, cust, temp_sched, temp_route);
    DistInt cost = cost_new - cost_old;
    if (cost < cost_min && chkcap(cand.capacity(), temp_sched)
     && chktw(temp_sched, temp_route)) {
      cost_min = cost;
      greedy_cand  = std::make_shared<MutableVehicle>(cand);
      greedy_sched = std::move(temp_sched);
      greedy_route = std::move(temp_route);
    }
  }

  if (greedy_cand) {
    // Visualize the vehicle
    gui::vhi(greedy_cand->id());
    gui::curroute(greedy_cand->route());
    gui::newroute(greedy_route);
    pause();

    print << "\tMatched with vehl " << greedy_cand->id() << std::endl;
    this->assign(
      {cust.id()}, {}, greedy_route, greedy_sched, *greedy_cand);
  } else
    print << "\tNo suitable match" << std::endl;

  // Clear lines and highlights
  gui::reset();
}

