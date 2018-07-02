// Copyright (c) 2018 the Cargo authors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <omp.h>
#include <algorithm> /* std::find, std::nth_element */
#include <exception>
#include <iostream> /* std::endl */
#include <vector>

#include "libcargo.h"
#include "trip_vehicle_grouping.h"

#include "glpk/glpk.h" /* ILP solver */

using namespace cargo;

TripVehicleGrouping::TripVehicleGrouping()
    : RSAlgorithm("tvg"),
      grid_(100) /* <-- Initialize my 100x100 grid (see grid.h) */ {
  batch_time() = 30;  // Set batch to 30 real seconds
  nmat_ = 0;          // Initialize my private counter
  stid_ = 0;          // Initialize internal SharedTripId

  /* Initialize gtrees for parallel sp */
  for (int i = 0; i < omp_get_max_threads(); ++i) gtre_.push_back(GTree::get());
}

void TripVehicleGrouping::handle_vehicle(const cargo::Vehicle& vehl) {
  grid_.insert(vehl);
}

void TripVehicleGrouping::match() {
  print_info << "match called" << std::endl;

  /* Initialize the GLPK mixed-integer program */
  glp_prob* mip;
  mip = glp_create_prob();
  glp_set_prob_name(mip, ("mip (t=" + std::to_string(Cargo::now())).c_str());
  glp_set_obj_dir(mip, GLP_MIN);

  /* To avoid repeated calcuations, store cost, schedule, routes of
   * assigning vehicles to customers */
  dict<Vehicle, dict<Customer, std::vector<Stop>>> rv_sch;
  dict<Vehicle, dict<Customer, std::vector<Wayp>>> rv_rte;
  dict<Vehicle, dict<Customer, DistInt>>           rv_cst;

  print_info << "generating rv-graph..." << std::endl;
  { /* Generate rv-graph */
  std::vector<Customer> lcl_cust = customers();
  #pragma omp parallel shared(lcl_cust, rvgrph_rr_, rvgrph_rv_, \
          rv_cst, rv_sch, rv_rte)
  { /* Each thread gets a local gtree and a local grid to perform
     * sp-computations in parallel */
  GTree::G_Tree& lcl_gtre = gtre_.at(omp_get_thread_num());
  Grid lcl_grid = grid_;
  #pragma omp for
  for (auto ptcust = lcl_cust.begin(); ptcust < lcl_cust.end(); ++ptcust) {
    if (ptcust->assigned()) continue;  // <-- skip assigned not yet picked up

    /* 1) Build rr edges
     * -----------------
     * First create a virtual vehicle for cust_a... */
    const Customer& cust_a = *ptcust;
    Vehicle vtvehl(
            cust_a.id(),
            cust_a.orig(),
            cust_a.dest(),
            cust_a.early(),
            cust_a.late(),
            0,  // load
            lcl_gtre);
    /* ...then compare against all other cust_b */
    for (const Customer& cust_b : customers()) {
      if (cust_a == cust_b) continue;
      DistInt cstout = 0;
      std::vector<Stop> schout;
      std::vector<Wayp> rteout;
      if (travel(vtvehl, {cust_b}, cstout, schout, rteout, lcl_gtre)) {
        #pragma omp critical  // <-- one thread writes at a time
        { rvgrph_rr_[cust_a].push_back(cust_b); }
      }
    }

    /* 2) Build rv edges
     * -----------------
     * ...then compare against all vehicles */
    DistInt rng = pickup_range(cust_a, Cargo::now(), lcl_gtre);
    auto cands = lcl_grid.within_about(rng, cust_a.orig());
    for (const auto& cand : cands) {
      if (cand->queued() == cand->capacity())
        continue;  // <-- don't consider vehls queued to capacity
      DistInt cstout = 0;
      std::vector<Stop> schout;
      std::vector<Wayp> rteout;
      if (travel(*cand, {cust_a}, cstout, schout, rteout, lcl_gtre)) {
        #pragma omp critical // store the results
        { rv_cst[*cand][cust_a] = cstout;
          rv_sch[*cand][cust_a] = schout;
          rv_rte[*cand][cust_a] = rteout; }
      }
    }
  }  // end pragma omp for
  }  // end pragma omp parallel
  }  // end generate rv-graph

  { /* Heuristic: keep only the lowest k customers per vehicle */
    size_t topk = 30;
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
  }  // end heuristic

  /* Store vehicles, schedules, and routes so we can commit them
   * when vehicles are assigned */
  dict<VehlId, dict<SharedTripId, std::vector<Stop>>> vt_sch;
  dict<VehlId, dict<SharedTripId, std::vector<Wayp>>> vt_rte;
  dict<VehlId, Vehicle> vehmap;

  print_info << "generating rtv-graph..." << std::endl;
  int nvted = 0; // number of vehicle-trip edges
  { /* Generate rtv-graph */
  std::vector<Vehicle> lcl_vehl = vehicles();
  int timeout = 200;  // <-- Heuristic: stop after this time (ms)
  #pragma omp parallel shared(nvted, lcl_vehl, vt_sch, vt_rte, vehmap, \
          vted_, rvgrph_rr_, rvgrph_rv_, timeout)
  { /* Each thread adds edges to a local vtedges; these are combined when
     * all threads have completed */
  std::chrono::time_point<std::chrono::high_resolution_clock> t0, t1;
  dict<VehlId, dict<SharedTripId, DistInt>> lcl_vted = {};
  dict<SharedTripId, SharedTrip>            lcl_trip = {};
  GTree::G_Tree                           & lcl_gtre = gtre_.at(omp_get_thread_num());
  #pragma omp for
  for (auto ptvehl = lcl_vehl.begin(); ptvehl < lcl_vehl.end(); ++ptvehl) {
    t0 = std::chrono::high_resolution_clock::now();
    const Vehicle& vehl = *ptvehl;
    if (vehl.queued() == vehl.capacity()) continue; // skip assigned to full
    #pragma omp critical // store vehicle for later
    { vehmap[vehl.id()] = vehl; }

    bool timed_out = false;

    /* Store trips of size_t for this vehicle */
    dict<size_t, std::vector<SharedTripId>> tripk;

    /* Trips of size 1
     * ---------------
     *  Add a trip of size 1 for every rv-pair involving this vehl */
    if (rvgrph_rv_.count(vehl) > 0) {
      for (const Customer& cust : rvgrph_rv_.at(vehl)) {
        SharedTripId stid;
        #pragma omp critical
        { stid = add_trip({cust});
          vt_sch[vehl.id()][stid] = rv_sch[vehl][cust];
          vt_rte[vehl.id()][stid] = rv_rte[vehl][cust]; }
        lcl_vted[vehl.id()][stid] = rv_cst[vehl][cust];
        lcl_trip[stid] = {cust};
        tripk[1].push_back(stid);

        /* Heuristic: try for only 200 ms per vehicle */
        if (std::round(std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count()) > timeout) {
          timed_out = true;
          break;
        }
      }
    } else
      continue;  // <-- if no rv-pairs, skip to the next vehl

    /* Heuristic: try for only 200 ms per vehicle */
    if (timed_out) continue;  // <-- continue to the next vehicle

    /* Trips of size 2
     * ---------------
     * Add trips of size 1 that can be combined, and add any rr-pairs
     * that can be served by this vehl */
    if (vehl.capacity() - vehl.queued() > 1) {  // <-- only vehls with capac
    /* 1) Check if any size 1 trips can be combined */
    for (const auto& id_a : tripk.at(1)) {
      SharedTrip shtrip(lcl_trip.at(id_a));
      for (const auto& id_b : tripk.at(1)) {
        if (id_a == id_b) continue;
        shtrip.insert(shtrip.end(), lcl_trip.at(id_b).begin(), lcl_trip.at(id_b).end());
        DistInt cstout = 0;
        std::vector<Stop> schout;
        std::vector<Wayp> rteout;
        if (travel(vehl, shtrip, cstout, schout, rteout, lcl_gtre)) {
          SharedTripId stid;
          #pragma omp critical
          { stid = add_trip(shtrip);
            vt_sch[vehl.id()][stid] = schout;
            vt_rte[vehl.id()][stid] = rteout; }
          lcl_vted[vehl.id()][stid] = cstout;
          lcl_trip[stid] = shtrip;
          tripk[2].push_back(stid);
        }
        /* Heuristic: try for only 200 ms per vehicle */
        if (std::round(std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count()) > timeout) {
          timed_out = true;
          break;
        }
      }
      if (timed_out) break;
    }
    /* Heuristic: try for only 200 ms per vehicle */
    if (timed_out) continue;  // <-- continue to the next vehicle

    /* 2) Check if any rr pairs can be served by this vehicle */
    for (const auto& kv : rvgrph_rr_) {
      const Customer& cust_a = kv.first;
      for (const Customer& cust_b : kv.second) {
        DistInt cstout = 0;
        std::vector<Stop> schout;
        std::vector<Wayp> rteout;
        SharedTrip shtrip = {cust_a, cust_b};
        if (travel(vehl, shtrip, cstout, schout, rteout, lcl_gtre)) {
          SharedTripId stid;
          #pragma omp critical
          { stid = add_trip(shtrip);
            vt_sch[vehl.id()][stid] = schout;
            vt_rte[vehl.id()][stid] = rteout; }
          lcl_vted[vehl.id()][stid] = cstout;
          lcl_trip[stid] = shtrip;
          tripk[2].push_back(stid);
        }
        /* Heuristic: try for only 200 ms per vehicle */
        if (std::round(std::chrono::duration<double, std::milli>(
            std::chrono::high_resolution_clock::now() - t0).count()) > timeout) {
          timed_out = true;
          break;
        }
      }
      if (timed_out) break;
    }
    /* Heuristic: try for only 200 ms per vehicle */
    if (timed_out) continue;  // <-- continue to the next vehicle

    /* Trips of size >= 3
     * ------------------ */
    size_t k = 3;
    while ((size_t)(vehl.capacity() - vehl.queued()) >= k && tripk.count(k-1) > 0) {
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
                #pragma omp critical
                { stid = add_trip(shtrip);
                  vt_sch[vehl.id()][stid] = schout;
                  vt_rte[vehl.id()][stid] = rteout; }
                lcl_vted[vehl.id()][stid] = cstout;
                lcl_trip[stid] = shtrip;
                tripk[k].push_back(stid);
              }
            }
          }  // end if shtrip.size() == k
          /* Heuristic: try for only 200 ms per vehicle */
          if (std::round(std::chrono::duration<double, std::milli>(
              std::chrono::high_resolution_clock::now() - t0).count()) > timeout) {
            timed_out = true;
            break;
          }
        }  // end inner for
        if (timed_out) break;
      } // end outer for
      k++;
      /* Heuristic: try for only 200 ms per vehicle */
      if (timed_out) continue;  // <-- continue to the next vehicle
    } // end while
    } // end if vehls with capacity
  } // end pragma omp for

  /* Combine lcl_vted */
  #pragma omp critical
  { for (const auto& kv : lcl_vted) {
    if (vted_.count(kv.first) == 0) {
      vted_[kv.first] = kv.second;
      nvted += kv.second.size();
    } else {
      for (const auto& kv2 : kv.second) {
        vted_[kv.first][kv2.first] = kv2.second;
        nvted++;
       }
    }
    }  // end for
  }    // end pragma omp critical

  }  // end pragma omp parallel
  print_info << "ntrips=" << trip_.size() << std::endl;
  }  // end generate rtv-graph

  // print_info << "ctedges:" << std::endl;
  // for (const auto& kv : cted_) {
  //     print_info << kv.first << ":";
  //     for (const auto& stid : kv.second)
  //         print_info << " " << stid;
  //     print_info << std::endl;
  // }

  // print_info << "vted_:" << std::endl;
  // for (const auto& kv : vted_) {
  //     print_info << kv.first << ":";
  //     for (const auto& kv2 : kv.second)
  //         print_info << " " << kv2.first /*<< "(" << kv2.second << ")"*/;
  //     print_info << std::endl;
  // }

  // print_info << "trips:" << std::endl;
  // for (const auto& kv : trip_) {
  //     print_info << kv.first << ":";
  //     for (const auto& cust : kv.second)
  //         print_info << " " << cust.id();
  //     print_info << std::endl;
  // }

  print_info << "generating the mip..." << std::endl;
  /* Generating the mip */
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
  int ncol = nvted + customers().size();
  glp_add_cols(mip, ncol);
  /*     - Total rows == # of vehicles (size of vted_), plus |customers| */
  int nrow = vted_.size() + customers().size();
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
  for (const auto& cust : customers()) {
    col_idx++;
    colmap[col_idx] = {cust.id(), -1};
    glp_set_obj_coef(mip, col_idx, Cargo::basecost(cust.id()));
    glp_set_col_kind(mip, col_idx, GLP_BV);
    glp_set_col_name(mip, col_idx,
                     ("y_" + std::to_string(cust.id())).c_str());
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
  for (const auto& cust : customers()) {
    row_idx++;
    glp_set_row_bnds(mip, row_idx, GLP_FX, 1.0, 1.0);
    glp_set_row_name(mip, row_idx, ("c" + std::to_string(cust.id())).c_str());
    for (const auto& kv2 : colmap) {
      cel_idx++;
      ia[cel_idx] = row_idx;
      ja[cel_idx] = kv2.first;
      if (kv2.second.first == cust.id()) ar[cel_idx] = 1;
      else if (kv2.second.second == -1)  ar[cel_idx] = 0;
      else {                             ar[cel_idx] = 0;
        for (const auto& cust2 : trip_.at(kv2.second.second))
          if (cust2.id() == cust.id()) { ar[cel_idx] = 1;
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

  print_info << "solving..." << std::endl;
  int rc = glp_intopt(mip, &cparams);  // <-- solve!

  print_out << "mip return:" << std::endl;
  switch (rc) {
    case 0: print_out << "good\n"; break;
    case GLP_EBOUND:  print_error << "glp_ebound\n";  break;
    case GLP_EROOT:   print_error << "glp_eroot\n";   break;
    case GLP_ENOPFS:  print_error << "glp_enopfs\n";  break;
    case GLP_ENODFS:  print_error << "glp_enodfs\n";  break;
    case GLP_EFAIL:   print_error << "glp_efail\n";   break;
    case GLP_EMIPGAP: print_error << "glp_emipgap\n"; break;
    case GLP_ETMLIM:  print_error << "glp_etmlim\n";  break;
    case GLP_ESTOP:   print_error << "glp_estop\n";   break;
  }
  int status = glp_mip_status(mip);
  print_out << "mip status:" << std::endl;
  switch (status) {
    case GLP_UNDEF:   print_out << "glp_undef\n";     break;
    case GLP_OPT:     print_out << "glp_opt\n";       break;
    case GLP_FEAS:    print_out << "glp_feas\n";      break;
    case GLP_NOFEAS:  print_out << "glp_nofeas\n";    break;
  }

  double z = glp_mip_obj_val(mip);
  print_out << z << std::endl;

  /* Extract assignments from the results and commit to database */
  for (int i = 1; i <= ncol; ++i) {
    /* On line 416 we set colmap[i].second to -1 for the binary vars
     * for unassigned customer penalty. Skip those because, well,
     * they are unassigned. */
    if (colmap[i].second == -1) continue;
    /* If the binary var for any other column is 1, then commit the
     * assignment. */
    if (glp_mip_col_val(mip, i) == 1) {
      if (commit(trip_.at(colmap[i].second), vehmap.at(colmap[i].first),
             vt_rte.at(colmap[i].first).at(colmap[i].second),
             vt_sch.at(colmap[i].first).at(colmap[i].second))) {
        for (const auto& cust : trip_.at(colmap[i].second))
          print_success << "Match (cust" << cust.id() << ", vehl" << colmap[i].first << ")\n";
        nmat_++;
      } else {
        print_warning << "Sync failed vehl" << colmap[i].first << "\n";
      }
    }
  }

  glp_delete_prob(mip);
  }  // end mip (if vted_.size() > 0)

  /* Cleanup */
  rvgrph_rr_.clear();
  rvgrph_rv_.clear();
  trip_.clear();
  vted_.clear();
  cted_.clear();
}

void TripVehicleGrouping::end() {
  print_success << "Matches: " << nmat_ << std::endl;
}

void TripVehicleGrouping::listen() {
  grid_.clear();          // Clear the index...
  RSAlgorithm::listen();  // ...then call listen()
}

bool TripVehicleGrouping::travel(
        const Vehicle& vehl,
        const std::vector<Customer>& custs,
        DistInt& cstout,
        std::vector<Stop>& schout,
        std::vector<Wayp>& rteout,
        GTree::G_Tree& gtre) {
  /* Need mutable copy so that schedule/route can be updated as customers
   * are added */
  MutableVehicle mtvehl = vehl;

  /* Containers */
  std::vector<Stop> schctr;
  std::vector<Wayp> rtectr;

  cstout = -1;

  /* Insert customers one by one */
  DistInt cstsum = 0;
  for (const Customer& cust : custs) {
    DistInt cst = sop_insert(mtvehl, cust, schctr, rtectr, gtre);
    if (check_timewindow_constr(schctr, rtectr)) {
      cstsum += cst;
      mtvehl.set_schedule(schctr);
      mtvehl.set_route(rtectr);
      mtvehl.reset_lvn();
    } else  // <-- a customer failed; trip cannot be served
      return false;
  }
  cstout = cstsum - Cargo::basecost(vehl.id());
  schout = schctr;
  rteout = rtectr;
  return true;
}

SharedTripId TripVehicleGrouping::add_trip(const SharedTrip& trip) {
  for (const auto& kv : trip_)
    if (kv.second == trip) return kv.first;
  stid_++;
  trip_[stid_] = trip;
  for (const auto& cust : trip) cted_[cust.id()].push_back(stid_);
  return stid_;
}

int main() {
  /* Set the options */
  cargo::Options op;
  op.path_to_roadnet  = "../../data/roadnetwork/mny.rnet";
  op.path_to_gtree    = "../../data/roadnetwork/mny.gtree";
  op.path_to_edges    = "../../data/roadnetwork/mny.edges";
  op.path_to_problem  = "../../data/benchmark/rs-lg-5.instance";
  op.path_to_solution = "a.sol";
  op.time_multiplier  = 1;
  op.vehicle_speed    = 10;
  op.matching_period  = 60;

  cargo::Cargo cargo(op);

  /* Initialize a new tvg */
  TripVehicleGrouping tvg;

  /* Start Cargo */
  cargo.start(tvg);
}

