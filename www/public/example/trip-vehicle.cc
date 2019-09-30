CargoWeb::CargoWeb()
    : RSAlgorithm("cargoweb", true), grid_(100) {
  batch_time() = BATCH;
  gtre_.push_back(GTree::get());
}

void CargoWeb::match() {
  //print << "Match called." << std::endl;
  this->reset_workspace();
  this->timeout_rv_0 = hiclock::now();
  for (const Customer& cust : customers())
    is_matched[cust.id()] = false;

  // Workspace variables
  std::vector<CustId> matchable_custs {};
  dict<Vehicle, dict<Customer, std::vector<Stop>>>    rv_sch;  // schedule store
  dict<Vehicle, dict<Customer, std::vector<Wayp>>>    rv_rte;  // route store
  dict<Vehicle, dict<Customer, DistInt>>              rv_cst;
  dict<Customer, std::vector<Customer>>               rvgrph_rr_;
  dict<Vehicle, std::vector<Customer>>                rvgrph_rv_;
  dict<VehlId, dict<SharedTripId, std::vector<Stop>>> vt_sch;
  dict<VehlId, dict<SharedTripId, std::vector<Wayp>>> vt_rte;
  dict<VehlId, Vehicle>                               vehmap;
  dict<VehlId, dict<SharedTripId, DistInt>>           vted_;  // vehl-trip edges

  /* Generate rv-graph */
  { std::vector<Customer> lcl_cust = customers();
  //print << "Generating rv-graph..." << std::endl;
  GTree::G_Tree& lcl_gtre = gtre_.at(0);
  Grid lcl_grid = grid_;
  for (auto ptcust = lcl_cust.begin(); ptcust < lcl_cust.end(); ++ptcust) {
    { matchable_custs.push_back(ptcust->id()); }
    const Customer& cust_a = *ptcust;
    //print << "\tWorking on cust " << ptcust->id() << std::endl;

    /* Build rv edges
     * ----------------- */
    //print << "\tBuilding rv-edges..." << std::endl;
    auto cands = lcl_grid.within(pickup_range(cust_a), cust_a.orig());
    for (const auto& cand : cands) {
      // Speed-up heuristic: try only if vehicle's current schedule len < 8 customer stops
      if (cand->schedule().data().size() < 10) {
        DistInt cstout = 0;
        std::vector<Stop> schout;
        std::vector<Wayp> rteout;
        if (travel(*cand, {cust_a}, cstout, schout, rteout, lcl_gtre)) {
          { rv_cst[*cand][cust_a] = cstout;
            rv_sch[*cand][cust_a] = schout;
            rv_rte[*cand][cust_a] = rteout; }
        } else {
            // print << "  travel not valid; reject" << std::endl;
        }
      } else {
        // print << "  no capacity; reject" << std::endl;
      }
    }
     if (cands.size() == 0) {
       continue;
       // print << "No candidates." << std::endl;
     }
    if (this->timeout(this->timeout_rv_0)) {
      //print << "\t  Timed out" << std::endl;
      // break;
    }

    /* Build rr edges
     * ----------------- */
    //print << "\tBuilding rr-edges..." << std::endl;
    Vehicle vtvehl(cust_a.id(), cust_a.orig(), cust_a.dest(),
                   cust_a.early(), cust_a.late(), -3, lcl_gtre);
    /* ...then compare against all other cust_b */
    for (const Customer& cust_b : customers()) {
      if (cust_a == cust_b) continue;
      // print << "Cust " << cust_a.id() << " join Cust " << cust_b.id() << " ? " << std::endl;
      /* Euclidean filters */
      if (haversine(cust_a.orig(), cust_b.orig()) > pickup_range(cust_b)) {
        // print << "  origins not in range; reject" << std::endl;
        continue;
      }
      // does this make sense?
      // if (haversine(cust_a.dest(), cust_b.dest()) > pickup_range(cust_b)) {
      //   // print << "  destinations not in range; reject" << std::endl;
      //   continue;
      // }
      DistInt cstout = 0;
      std::vector<Stop> schout;
      std::vector<Wayp> rteout;
      if (travel(vtvehl, {cust_b}, cstout, schout, rteout, lcl_gtre)) {
        // print << "  accept" << std::endl;
        { rvgrph_rr_[cust_a].push_back(cust_b); }
      } else {
        // print << "  travel not valid; reject" << std::endl;
      }
    }
    // print << "Total rr-edges for cust " << ptcust->id() << std::endl;
    // for (const auto& i : rvgrph_rr_) {
    //   print << "Cust " << i.first.id() << ": Custs ";
    //   for (const auto& j : i.second)
    //     print << j.id() << " ";
    //   print << std::endl;
    // }
    // print << "done" << std::endl;
  }  // end for
  }  // end generate rv-graph


  // print << "R-R Edges:" << std::endl;
  // for (const auto& kv1 : rvgrph_rr_)
  //   for (const Customer& cust : kv1.second)
  //     print << "\t{ " << kv1.first.id() << " " << cust.id() << " }" << std::endl;

  // Visualize r-r edges
  for (const auto& kv : rvgrph_rr_)
    for (const auto& cust : kv.second)
      gui::clinec(kv.first.id(), cust.id());
  pause();
  gui::reset();

  // Visualize r-v edges
  for (const auto& kv1 : rv_cst) {
    gui::vhi(kv1.first.id());
    for (const auto& kv2 : kv1.second) {
      gui::clinev(kv2.first.id(), kv1.first.id());
    }
  }
  pause();
  gui::reset();


  { /* Heuristic: keep only the lowest k customers per vehicle */
    size_t topk = TOP_CUST;
    for (const auto& kv : rv_cst) {
      const Vehicle& vehl = kv.first;
      if (kv.second.size() > topk) {
        std::vector<std::pair<Customer, DistInt>> cc;
        for (const auto& kv2 : kv.second) cc.push_back({kv2.first, kv2.second});
        std::nth_element(cc.begin(), cc.begin() + topk, cc.end(),
                         [](const std::pair<Customer, DistInt>& a,
                            const std::pair<Customer, DistInt>& b) {
                           return a.second < b.second; });
        for (size_t i = 0; i < topk && i < kv.second.size(); ++i)
          rvgrph_rv_[vehl].push_back(cc.at(i).first);
      } else {
        for (const auto& kv2 : kv.second)
          rvgrph_rv_[vehl].push_back(kv2.first);
      }
    }
  }
  // print << "Total rv-edges:" << std::endl;
  // for (const auto& i : rvgrph_rr_) {
  //   print << "Vehl " << i.first.id() << ": Custs ";
  //   for (const auto& j : i.second)
  //     print << j.id() << " ";
  //   print << std::endl;
  // }
  // print << "done" << std::endl;

  if (this->done())
    return;

  /* Generate rtv-graph */
  this->timeout_rtv_0 = hiclock::now();
  int nvted = 0;
  //print << "Generating rtv-graph" << std::endl;
  { std::vector<Vehicle> lcl_vehl = vehicles();
  { /* Each thread adds edges to a local vtedges; these are combined when
     * all threads have completed */
  dict<VehlId, dict<SharedTripId, DistInt>> lcl_vted = {};
  dict<SharedTripId, SharedTrip>            lcl_trip = {};
  GTree::G_Tree                           & lcl_gtre = gtre_.at(0);
  for (auto ptvehl = lcl_vehl.begin(); ptvehl < lcl_vehl.end(); ++ptvehl) {
    const Vehicle& vehl = *ptvehl;
    // Speed-up heuristic: try only if vehicle's current schedule len < 8 customer stops
    if (vehl.schedule().data().size() > 10)
      continue; // skip assigned to full
    { vehmap[vehl.id()] = vehl; }

    /* Store trips of size_t for this vehicle */
    dict<size_t, std::vector<SharedTripId>> tripk;

    /* Trips of size 1
     * ---------------
     *  Add a trip of size 1 for every rv-pair involving this vehl */
    //print << "\tBuilding trips size 1 for vehl " << vehl.id() << "..." << std::endl;
    if (rvgrph_rv_.count(vehl) > 0) {
      for (const Customer& cust : rvgrph_rv_.at(vehl)) {
        SharedTripId stid;
        { stid = add_trip({cust});
          vt_sch[vehl.id()][stid] = rv_sch[vehl][cust];
          vt_rte[vehl.id()][stid] = rv_rte[vehl][cust]; }
        lcl_vted[vehl.id()][stid] = rv_cst[vehl][cust];
        lcl_trip[stid] = {cust};
        tripk[1].push_back(stid);
      }
    } else
      continue;  // <-- if no rv-pairs, skip to the next vehl
    if (this->timeout(this->timeout_rtv_0)) {
      //print << "Timed out" << std::endl;
      // break;
    }

    /* Trips of size 2
     * ---------------
     * Add trips of size 1 that can be combined, and add any rr-pairs
     * that can be served by this vehl */
    if (vehl.capacity() > 1) {  // <-- only vehls with capac
    //print << "\tBuilding trips size 2 for vehl " << vehl.id() << "..." << std::endl;
    /* 1) Check if any size 1 trips can be combined */
    // for (const auto& id_a : tripk.at(1)) {
    //   SharedTrip sh_base(lcl_trip.at(id_a));
    //   for (const auto& id_b : tripk.at(1)) {
    //     if (id_a == id_b) continue;
    //     SharedTrip shtrip = sh_base;
    //     shtrip.insert(shtrip.end(), lcl_trip.at(id_b).begin(), lcl_trip.at(id_b).end());
    //     // print << "Formed shtrip: ";
    //     // for (const Customer& cust : shtrip)
    //     //   print << cust.id() << " ";
    //     // print << std::endl;
    //     /* Euclidean filter */
    //     if (haversine(vehl.last_visited_node(), shtrip.at(0).orig()) > pickup_range(shtrip.at(0)))
    //       continue;
    //     DistInt cstout = 0;
    //     std::vector<Stop> schout;
    //     std::vector<Wayp> rteout;
    //     if (travel(vehl, shtrip, cstout, schout, rteout, lcl_gtre)) {
    //       SharedTripId stid;
    //       { stid = add_trip(shtrip);
    //         vt_sch[vehl.id()][stid] = schout;
    //         vt_rte[vehl.id()][stid] = rteout; }
    //       lcl_vted[vehl.id()][stid] = cstout;
    //       lcl_trip[stid] = shtrip;
    //       tripk[2].push_back(stid);
    //     }
    //     if (this->timeout(this->timeout_rtv_0)) {
    //       //print << "Timed out" << std::endl;
    //       // break;
    //     }
    //   }
    //   if (this->timeout(this->timeout_rtv_0)) {
    //     //print << "Timed out" << std::endl;
    //     // break;
    //   }
    // }

    /* 2) Check if any rr pairs can be served by this vehicle */
    for (const auto& kv : rvgrph_rr_) {
      const Customer& cust_a = kv.first;
      /* Euclidean filter */
      if (haversine(vehl.last_visited_node(), cust_a.orig()) > pickup_range(cust_a)) continue;
      for (const Customer& cust_b : kv.second) {
        DistInt cstout = 0;
        std::vector<Stop> schout;
        std::vector<Wayp> rteout;
        SharedTrip shtrip = {cust_a, cust_b};
        if (travel(vehl, shtrip, cstout, schout, rteout, lcl_gtre)) {
          SharedTripId stid;
          { stid = add_trip(shtrip);
            vt_sch[vehl.id()][stid] = schout;
            vt_rte[vehl.id()][stid] = rteout; }
          lcl_vted[vehl.id()][stid] = cstout;
          lcl_trip[stid] = shtrip;
          tripk[2].push_back(stid);
        }
        if (this->timeout(this->timeout_rtv_0)) {
          //print << "Timed out" << std::endl;
          // break;
        }
      }
      if (this->timeout(this->timeout_rtv_0)) {
        //print << "Timed out" << std::endl;
        // break;
      }
    }
    if (this->timeout(this->timeout_rtv_0)) {
      //print << "Timed out" << std::endl;
      // break;
    }

    /* Trips of size >= 3
     * ------------------ */
    size_t k = 3;
    while (vehl.capacity() >= (int)k && tripk.count(k-1) > 0) {
      //print << "\tBuilding trips size " << k << " for vehl " << vehl.id() << "..." << std::endl;
      /* Loop through trips of size k-1 */
      for (SharedTripId id_a : tripk.at(k - 1)) {
        const SharedTrip& trip_a = lcl_trip.at(id_a);
        /* Compare against remaining trips of size k-1 */
        for (SharedTripId id_b : tripk.at(k - 1)) {
          if (id_a == id_b) continue;
          const SharedTrip& trip_b = lcl_trip.at(id_b);
          /* Join trip_a and trip_b into new trip (no dups) */
          SharedTrip shtrip(trip_a);
          for (const Customer& cust : trip_b)
            if (std::find(shtrip.begin(), shtrip.end(), cust) == shtrip.end())
              shtrip.push_back(cust);
          /* If shtrip is size k... */
          if (shtrip.size() == k) {
            bool all_ok = true;
            /* ... check each subtrip if it is already a trip */
            for (size_t p = 0; p < shtrip.size(); ++p) {
              bool subtrip_ok = false;
              SharedTrip subtrip(shtrip);
              subtrip.erase(subtrip.begin() + p);
              for (SharedTripId id_q : tripk.at(k - 1))
                if (subtrip == lcl_trip.at(id_q)) {
                  subtrip_ok = true;
                  break;
                }
              if (!subtrip_ok) {
                all_ok = false;
                break;
              }
            }
            if (all_ok) {
              DistInt cstout = 0;
              std::vector<Stop> schout;
              std::vector<Wayp> rteout;
              if (travel(vehl, shtrip, cstout, schout, rteout, lcl_gtre)) {
                SharedTripId stid;
                { stid = add_trip(shtrip);
                  vt_sch[vehl.id()][stid] = schout;
                  vt_rte[vehl.id()][stid] = rteout; }
                lcl_vted[vehl.id()][stid] = cstout;
                lcl_trip[stid] = shtrip;
                tripk[k].push_back(stid);
              }
            }
          }  // end if shtrip.size() == k
        }  // end inner for
        if (this->timeout(this->timeout_rtv_0)) {
          //print << "Timed out" << std::endl;
          // break;
        }
      } // end outer for
      k++;
    } // end while
    } // end if vehls with capacity
    if (this->timeout(timeout_rtv_0)) {
      //print << "Timed out" << std::endl;
      // break;
    }
  } // end pragma omp for

  /* Combine lcl_vted */
  {
    for (const auto& kv : lcl_vted) {
    if (vted_.count(kv.first) == 0) {
      vted_[kv.first] = kv.second;
      nvted += kv.second.size();
      /* Heuristic: keep up to TRIP_MAX trips */
      if (nvted > TRIP_MAX) break;
    } else {
      for (const auto& kv2 : kv.second) {
        vted_[kv.first][kv2.first] = kv2.second;
        nvted++;
        if (nvted > TRIP_MAX) break;
       }
    }
    if (nvted > TRIP_MAX) break;
    }  // end for
  }    // end pragma omp critical

  }  // end pragma omp parallel
  }  // end generate rtv-graph

  // for (const auto& kv1 : vted_) {
  //   print << "Vehl " << kv1.first << " has " << kv1.second.size() << " trips:" << std::endl;
  //   for (const auto& kv2 : kv1.second) {
  //     print << "\tTrip " << kv2.first
  //       << " (size=" << trip_[kv2.first].size() << ";"
  //       << " Custs={ ";
  //     for (const Customer& cust : trip_[kv2.first]) {
  //       print << cust.id() << " ";
  //     }
  //     print << "}; cost=" << kv2.second << std::endl;
  //   }
  // }

  // Visualize rtv-graph
  if (Cargo::now() > 10) {
  for (const auto& kv1 : vted_) {
    for (const auto& kv2 : kv1.second) {
      gui::vhi(kv1.first);
      for (const auto& cust : trip_[kv2.first]) {
        gui::clinev(cust.id(), kv1.first);
      }
      pause();
      gui::reset();
    }
  }
  }

  if (this->done())
    return;

  /* Generating the mip */
  //print << "Got " << nvted << " trips, " << vted_.size() << " vehicles" << std::endl;
  if (vted_.size() > 0) {
  /* Objective: c11x11 + c12x12 + ... + cijxij + y1 + y2 + ... + yn
   *     For cijxij, ij are from vted_
   *     For yn, n is from waiting_customers()
   * Constraints:
   *     1) Each vehicle serves 0 or 1 trips
   *         xi1 + xi2 + ... + xin <= 1, for all i
   *     2) Each customer is either served or unserved
   *         yn + xij == 1, for all n, where j are trips containing n, and
   *                                         i are the vehicles serve them
   * Properties:
   *     - Total terms in the objective == number of columns
   *       == all VehicleId, SharedTripId pairs in vted_, plus |customers| */
  glp_prob* mip;
  mip = glp_create_prob();
  glp_set_prob_name(mip, ("mip (t=" + std::to_string(Cargo::now())).c_str());
  glp_set_obj_dir(mip, GLP_MIN);
  glp_term_out(GLP_OFF);

  int ncol = nvted + matchable_custs.size();
  glp_add_cols(mip, ncol);
  /*     - Total rows == # of vehicles (size of vted_), plus |customers| */
  int nrow = vted_.size() + matchable_custs.size();
  glp_add_rows(mip, nrow);

  /* Map col_idx to the vehicle/trip edge */
  dict<size_t, std::pair<TripId, SharedTripId>> colmap{};

  { /* Populate the coefficients in the objective */
  size_t col_idx = 0;
  for (const auto& kv : vted_) {
    const VehlId& vehl_id = kv.first;
    for (const auto& kv2 : kv.second) {
      const SharedTripId& shtrip_id = kv2.first;
      col_idx++;
      colmap[col_idx] = {vehl_id, shtrip_id};
      glp_set_obj_coef(mip, col_idx, kv2.second);
      glp_set_col_kind(mip, col_idx, GLP_BV);
      glp_set_col_name(mip, col_idx,
          ("x_" + std::to_string(vehl_id) + "_" + std::to_string(shtrip_id)).c_str());
    }
  }
  for (const auto& cust_id : matchable_custs) {
    col_idx++;
    colmap[col_idx] = {cust_id, -1};
    int penalty = (unassign_penalty > 0 ? unassign_penalty : Cargo::basecost(cust_id));
    glp_set_obj_coef(mip, col_idx, penalty);
    glp_set_col_kind(mip, col_idx, GLP_BV);
    glp_set_col_name(mip, col_idx,
                     ("y_" + std::to_string(cust_id)).c_str());
  }
  }  // end populate coefficients

  /* Populate cells of the constraints matrix */
  std::vector<int>    ia(ncol * nrow + 1);  // <-- 1-indexed
  std::vector<int>    ja(ncol * nrow + 1);
  std::vector<double> ar(ncol * nrow + 1);
  /* For constraint 1) each vehicle serves 0 or 1 trips:
   *     ia[cel_idx] = row_idx
   *     ja[cel_idx] = col_idx
   *     ar[cel_idx] = 1 if the i of col matches the vehl of row_idx,
   *                   0 otherwise */
  size_t row_idx = 0;
  size_t cel_idx = 0;
  for (const auto& kv : vted_) {
    row_idx++;
    glp_set_row_bnds(mip, row_idx, GLP_UP, 0.0, 1.0);
    glp_set_row_name(mip, row_idx, ("v" + std::to_string(kv.first)).c_str());
    for (const auto& kv2 : colmap) {
      cel_idx++;
      ia[cel_idx] = row_idx;
      ja[cel_idx] = kv2.first;
      ar[cel_idx] = (kv.first == kv2.second.first ? 1 : 0);
    }
  }
  /* For constraint 2) each customer is either served or unserved:
   *     ia[cel_idx] = row_idx
   *     ja[cel_idx] = col_idx
   *     ar[cel_idx] = 1 if the j of col contains the cust of row_idx,
   *                   1 if the i of col equals cust.id(),
   *                   0 otherwise */
  for (const auto& cust_id : matchable_custs) {
    row_idx++;
    glp_set_row_bnds(mip, row_idx, GLP_FX, 1.0, 1.0);
    glp_set_row_name(mip, row_idx, ("c" + std::to_string(cust_id)).c_str());
    for (const auto& kv2 : colmap) {
      cel_idx++;
      ia[cel_idx] = row_idx;
      ja[cel_idx] = kv2.first;
      if (kv2.second.first == cust_id) ar[cel_idx] = 1;
      else if (kv2.second.second == -1)  ar[cel_idx] = 0;
      else {                             ar[cel_idx] = 0;
        for (const auto& cust2 : trip_.at(kv2.second.second))
          if (cust2.id() == cust_id) { ar[cel_idx] = 1;
            break;
          }
      }
    }
  }
  // end populate constraints matrix

  glp_load_matrix(mip, cel_idx, ia.data(), ja.data(), ar.data());
  glp_iocp cparams;
  glp_init_iocp(&cparams);
  cparams.presolve = GLP_ON;

  /* Heuristics */
  cparams.tm_lim = 15 * 1000;  // set time limit to 15 sec
  cparams.mip_gap = 0.001;     // set optimality gap to 0.1%

  //print(MessageType::Info) << "Solving mip..." << std::endl;
  // int rc = glp_intopt(mip, &cparams);  // <-- solve!
  glp_intopt(mip, &cparams);  // <-- solve!

  // print << "mip return:" << std::endl;
  // switch (rc) {
  //   case 0: print << "good\n"; break;
  //   case GLP_EBOUND:  print(MessageType::Error) << "glp_ebound\n";  break;
  //   case GLP_EROOT:   print(MessageType::Error) << "glp_eroot\n";   break;
  //   case GLP_ENOPFS:  print(MessageType::Error) << "glp_enopfs\n";  break;
  //   case GLP_ENODFS:  print(MessageType::Error) << "glp_enodfs\n";  break;
  //   case GLP_EFAIL:   print(MessageType::Error) << "glp_efail\n";   break;
  //   case GLP_EMIPGAP: print(MessageType::Error) << "glp_emipgap\n"; break;
  //   case GLP_ETMLIM:  print(MessageType::Error) << "glp_etmlim\n";  break;
  //   case GLP_ESTOP:   print(MessageType::Error) << "glp_estop\n";   break;
  // }
  // int status = glp_mip_status(mip);
  // print << "mip status:" << std::endl;
  // switch (status) {
  //   case GLP_UNDEF:   print << "glp_undef\n";     break;
  //   case GLP_OPT:     print << "glp_opt\n";       break;
  //   case GLP_FEAS:    print << "glp_feas\n";      break;
  //   case GLP_NOFEAS:  print << "glp_nofeas\n";    break;
  // }

  // double z = glp_mip_obj_val(mip);
  // print << z << std::endl;

  if (this->done())
    return;

  /* Extract assignments from the results and commit to database */
  for (int i = 1; i <= ncol; ++i) {
    if (colmap[i].second == -1)
      continue;
    /* If the binary var for any other column is 1, then commit the
     * assignment. */
    if (glp_mip_col_val(mip, i) == 1) {
      MutableVehicle sync_vehl(vehmap.at(colmap[i].first));
      std::vector<Wayp>& new_rte = vt_rte.at(colmap[i].first).at(colmap[i].second);
      std::vector<Stop>& new_sch = vt_sch.at(colmap[i].first).at(colmap[i].second);
      std::vector<CustId> cadd {};
      for (const Customer& cust : trip_.at(colmap[i].second)) {
        cadd.push_back(cust.id());
        //print << "Matched " << cust.id() << " with " << sync_vehl.id() << std::endl;
      }
      if (this->assign(cadd, {}, new_rte, new_sch, sync_vehl)) {
        for (const auto& cust : trip_.at(colmap[i].second)) {
          is_matched.at(cust.id()) = true;
          this->end_delay(cust.id());
        }
      } else {
        for (size_t i = 0; i < cadd.size(); ++i) {
          this->beg_delay(cadd.at(i));
        }
      }
    }
  }
  for (const auto& kv : is_matched)
    if (!kv.second)
      this->beg_delay(kv.first);

  glp_delete_prob(mip);
  }  // end mip (if vted_.size() > 0)
}

void CargoWeb::handle_vehicle(const Vehicle& vehl) {
  this->grid_.insert(vehl);
}

void CargoWeb::listen(bool skip_assigned, bool skip_delayed) {
  grid_.clear();
  RSAlgorithm::listen(
    skip_assigned, skip_delayed);
}

bool CargoWeb::travel(const Vehicle& vehl,
                                 const std::vector<Customer>& custs,
                                 DistInt& cstout,
                                 std::vector<Stop>& schout,
                                 std::vector<Wayp>& rteout,
                                 GTree::G_Tree& gtree) {
  vec_t<Customer> to_insert = custs;
  std::sort(to_insert.begin(), to_insert.end(),
    [](const Customer& a, const Customer& b) { return a.id() < b.id(); });

  bool has_good = false;
  vec_t<Stop> sch, best_sch;
  vec_t<Wayp> rte, best_rte;
  DistInt best_cost = InfInt;
  do {
    // print << "\tTravel trying ";
    // for (const Customer& cust : to_insert)
    //   print << cust.id() << " ";
    // print << std::endl;
    MutableVehicle copy = vehl;
    sch = {}; rte = {};
    bool good = true;
    for (const Customer& cust : to_insert) {
      sop_insert(copy, cust, sch, rte, gtree);
      if (chktw(sch, rte) && chkcap(copy.capacity(), sch)) {
        copy.set_sch(sch);
        copy.set_rte(rte);
        copy.reset_lvn();
      } else {
        good = false;
        break;  // break the for loop
      }
    }
    if (good) {
      has_good = true;
      DistInt cost = copy.route().data().back().first - vehl.route().cost();
      if (cost < best_cost) {
        best_sch = copy.schedule().data();
        best_rte = copy.route().data();
        best_cost = cost;
      }
    }
  } while (std::next_permutation(to_insert.begin(), to_insert.end(),
    [](const Customer& a, const Customer& b) { return a.id() < b.id(); }));

  if (!has_good)
    return false;
  else {
    cstout = best_cost;
    schout = best_sch;
    rteout = best_rte;
    return true;
  }

  // /* Need mutable copy so that schedule/route can be updated as customers
  //  * are added */
  // MutableVehicle mtvehl = vehl;

  // /* Containers */
  // std::vector<Stop> schctr;
  // std::vector<Wayp> rtectr;

  // cstout = -1;

  // /* Insert customers one by one
  //  * (There is room to optimize?) */

  // DistInt cstsum = 0;
  // for (const Customer& cust : custs) {
  //   DistInt cst = sop_insert(mtvehl, cust, schctr, rtectr, gtre) - mtvehl.route().cost();
  //   if (chkcap(mtvehl.capacity(), schctr) && chktw(schctr, rtectr)) {
  //     cstsum += cst;
  //     mtvehl.set_sch(schctr);
  //     mtvehl.set_rte(rtectr);
  //     mtvehl.reset_lvn();
  //   } else  // <-- a customer failed; trip cannot be served
  //     return false;
  // }
  // cstout = cstsum - mtvehl.route().cost();
  // schout = schctr;
  // rteout = rtectr;
  // return true;
}

SharedTripId CargoWeb::add_trip(const SharedTrip& trip) {
  for (const auto& kv : trip_)
    if (kv.second == trip)
      return kv.first;
  stid_++;
  trip_[stid_] = trip;
  for (const auto& cust : trip)
    cted_[cust.id()].push_back(stid_);
  return stid_;
}

void CargoWeb::reset_workspace() {
  this->is_matched = {};
  if (Cargo::static_mode) this->timeout_=300*1000;
  this->timeout_ = this->timeout_/2;
  this->stid_ = 0;
  this->trip_ = {};
  this->cted_ = {};
}

