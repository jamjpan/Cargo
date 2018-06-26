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
#include <algorithm> /* std::find */
#include <cassert> /* assert */
#include <exception>
#include <iostream> /* std::endl */
#include <omp.h>
#include <vector>

#include "libcargo.h"
#include "trip_vehicle_grouping.h"

#include "glpk/glpk.h" /* ILP solver */

using namespace cargo;

TripVehicleGrouping::TripVehicleGrouping() : RSAlgorithm("tvg"),
    grid_(100) /* <-- Initialize my 100x100 grid (see grid.h) */ {
    batch_time() = 5;           // Set batch to 30 real seconds
    nmat_ = 0;                  // Initialize my private counter
    stid_ = 0;                  // Initialize internal SharedTripId
}

void TripVehicleGrouping::handle_customer(const cargo::Customer &) {
    /* null */
}

void TripVehicleGrouping::handle_vehicle(const cargo::Vehicle& veh) {
    grid_.insert(veh); // Insert into my grid
}

void TripVehicleGrouping::match() {
    glp_prob* mip_;
    mip_ = glp_create_prob();

    /* Set mip name */
    glp_set_prob_name(mip_, ("mip (t="+std::to_string(Cargo::now())).c_str());

    /* Set objective to minimize */
    glp_set_obj_dir(mip_, GLP_MIN);

    dict<VehicleId, dict<CustomerId, DistanceInt>>           rv_cst;
    dict<VehicleId, dict<CustomerId, std::vector<Stop>>>     rv_sch;
    dict<VehicleId, dict<CustomerId, std::vector<Waypoint>>> rv_rte;
    /* Generate rv-graph */ {
    int nedg = 0;
    /* All threads share a local snapshot of customers (not sure if needed?) */
    std::vector<Customer>   lcl_cust = waiting_customers();
    #pragma omp parallel shared(nedg, rvgrph_rr_, rvgrph_rv_, lcl_cust)
    { /* Each thread gets a local gtree and a local grid to perform
       * sp-computations in parallel */
    GTree::G_Tree           lcl_gtre = GTree::get();
    Grid                    lcl_grid = grid_;
    #pragma omp for
    for (auto itr = lcl_cust.begin(); itr < lcl_cust.end(); ++itr) {
        /* Don't consider customers that are assigned but not yet picked up */
        if (itr->assigned())
            continue;
        /* 1) Build rr edges
         *     a) Create a virtual vehicle for cust1 */
        const Customer& cust1 = *itr;
        Vehicle vrt_veh(cust1.id(), cust1.origin(), cust1.destination(),
                cust1.early(), cust1.late(), 0, lcl_gtre);
        /*     b) Compare the virtual vehicle with cust2 */
        for (const Customer& cust2 : waiting_customers()) {
            if (cust1 == cust2) continue;
            DistanceInt cstout = 0;
            std::vector<Stop> schout;
            std::vector<Waypoint> rteout;
            if (travel(vrt_veh, {cust2}, cstout, schout, rteout, lcl_gtre)) {
                #pragma omp critical // <-- one thread writes at a time
                { rvgrph_rr_[cust1].push_back(cust2);
                  nedg++; }
            }
        }
        /* 2) Build rv edges */
        DistanceInt rng = pickup_range(cust1, Cargo::now());
        auto cands = lcl_grid.within_about(rng, cust1.origin());
        for (const auto& cand : cands) {
            if (cand->queued() == cand->capacity())
                continue; // don't consider vehs already queued to capacity
            DistanceInt cstout = 0;
            std::vector<Stop> schout;
            std::vector<Waypoint> rteout;
            if (travel(*cand, {cust1}, cstout, schout, rteout, lcl_gtre)) {
                rv_cst[cand->id()][cust1.id()] = cstout;
                rv_sch[cand->id()][cust1.id()] = schout;
                rv_rte[cand->id()][cust1.id()] = rteout;
                #pragma omp critical
                { rvgrph_rv_[*cand].push_back(cust1);
                  nedg++; }
            }
        }
    } // end pragma omp for
    } // end pragma omp parallel
    print_info << "rvgrph_edges=" << nedg << std::endl;
    } // end generate rv-graph

    dict<VehicleId, Vehicle> vehmap;
    dict<VehicleId, dict<SharedTripId, std::vector<Stop>>> vt_sch;
    dict<VehicleId, dict<SharedTripId, std::vector<Waypoint>>> vt_rte;
    int nvtedg = 0;
    /* Generate rtv-graph */ {
    std::vector<Vehicle>    lcl_vehs = vehicles();
    #pragma omp parallel shared(vted_, nvtedg, \
            rvgrph_rr_, rvgrph_rv_, lcl_vehs, vehmap, vt_sch, vt_rte)
    { /* Each thread adds edges to a local vtedges; these are combined when
       * all threads have completed */
    GTree::G_Tree           lcl_gtre = GTree::get();
    dict<VehicleId, dict<SharedTripId, DistanceInt>>
                            lcl_vted = {};
    #pragma omp for
    for (auto itr = lcl_vehs.begin(); itr < lcl_vehs.end(); ++itr) {
        if (itr->queued() == itr->capacity())
            continue; // don't consider vehs already queued to capacity
        const Vehicle& veh = *itr;
        /* Store the vehicle in a lookup table (for later) */
        #pragma omp critical
        { vehmap[veh.id()] = veh; }

        /* Store trips of size_t for this vehicle */
        dict<size_t, std::vector<SharedTripId>> tripk;

        /* 1) Trips of size 1 */
        print_info << omp_get_thread_num() << " tripk1" << std::endl;
        /* Do only if a vehicle-customer pair exists for this vehicle */
        if (rvgrph_rv_.count(veh) > 0) {
        /* Add a trip of size 1 for each vehicle-customer pair for this veh */
        for (const Customer& cust : rvgrph_rv_.at(veh)) {
            SharedTripId stid;
            #pragma omp critical
            { stid = add_trip({cust});
              vt_sch[veh.id()][stid] = rv_sch[veh.id()][cust.id()];
              vt_rte[veh.id()][stid] = rv_rte[veh.id()][cust.id()]; }
            lcl_vted[veh.id()][stid] = rv_cst[veh.id()][cust.id()];
            tripk[1].push_back(stid);
        }
        } else
            continue;

        /* 2) Trips of size 2 */
        /* Only do if vehicle has capacity */
        if (veh.capacity() > 1) {
        print_info << omp_get_thread_num() << " tripk2" << std::endl;
        /* Check if size 1 trips can be combined */
        for (SharedTripId i : tripk.at(1)) {
            /* Initialize the possible trip to the cust in the size 1 trip */
            SharedTrip trip(trip_.at(i));
            /* Compare against all other size 1 trips */
            for (SharedTripId j : tripk.at(1)) {
                if (i == j) continue;
                trip.insert(trip.end(), trip_.at(j).begin(), trip_.at(j).end());
                DistanceInt cstout = 0;
                std::vector<Stop> schout;
                std::vector<Waypoint> rteout;
                if (travel(veh, trip, cstout, schout, rteout, lcl_gtre)) {
                    SharedTripId stid;
                    #pragma omp critical
                    { stid = add_trip({trip});
                      vt_sch[veh.id()][stid] = schout;
                      vt_rte[veh.id()][stid] = rteout; }
                    lcl_vted[veh.id()][stid] = cstout;
                    tripk[2].push_back(stid);
                }
            }
        }
        /* Check if any rr pairs can be served by this vehicle */
        for (const auto& kv : rvgrph_rr_) {
            const Customer& cust1 = kv.first;
            for (const Customer& cust2 : kv.second) {
                DistanceInt cstout = 0;
                std::vector<Stop> schout;
                std::vector<Waypoint> rteout;
                if (travel(veh, {cust1, cust2}, cstout, schout, rteout, lcl_gtre)) {
                    SharedTripId stid;
                    #pragma omp critical
                    { stid = add_trip({cust1, cust2});
                      vt_sch[veh.id()][stid] = schout;
                      vt_rte[veh.id()][stid] = rteout; }
                    lcl_vted[veh.id()][stid] = cstout;
                    tripk[2].push_back(stid);
                }
            }
        }

        /* 3) Trips of size > 2 */
        print_info << omp_get_thread_num() << " tripk>2" << std::endl;
        size_t k = 3;
        /* Only do if vehicle has capacity, and if trips of size k-1 exist */
        while ((size_t) veh.capacity() >= k && tripk.count(k-1) > 0) {
            /* Loop through trips of size k-1 */
            for (SharedTripId i : tripk.at(k-1)) {
                const SharedTrip& trip1 = trip_.at(i);
                /* Compare against remaining trips of size k-1 */
                for (SharedTripId j : tripk.at(k-1)) {
                    if (i == j) continue;
                    const SharedTrip& trip2 = trip_.at(j);
                    /* Join trip1 and trip2 into a new trip (no duplicates) */
                    SharedTrip trip(trip1);
                    for (const Customer& cust : trip2)
                        if (std::find(trip.begin(), trip.end(), cust) == trip.end())
                            trip.push_back(cust);
                    /* If trip1 JOIN trip2 (no duplicates) is size k... */
                    if (trip.size() == k) {
                        bool ok = false;
                        /* ... check each subtrip if it is already a trip */
                        for (size_t p = 0; p < trip.size(); ++p) {
                            SharedTrip subtrip(trip);
                            subtrip.erase(subtrip.begin() + p);
                            for (SharedTripId q : tripk.at(k-1))
                                if (subtrip == trip_.at(q))
                                    ok = true;
                        }
                        if (ok) {
                            DistanceInt cstout = 0;
                            std::vector<Stop> schout;
                            std::vector<Waypoint> rteout;
                            if (travel(veh, trip, cstout, schout, rteout, lcl_gtre)) {
                                SharedTripId stid;
                                #pragma omp critical
                                { stid = add_trip(trip);
                                  vt_sch[veh.id()][stid] = schout;
                                  vt_rte[veh.id()][stid] = rteout; }
                                lcl_vted[veh.id()][stid] = cstout;
                                tripk[k].push_back(stid);
                            }
                        }
                    }
                }
            }
            k++;
        } // end while
        } // end if vehicle.capacity() > 2
        print_info << omp_get_thread_num() << " done" << std::endl;
    } // end pragma omp for

    /* Combine lcl_vted */
    #pragma omp critical
    { /* Copy lcl_vted into global vted_ */
    for (const auto& kv : lcl_vted) {
        if (vted_.count(kv.first) == 0) {
            vted_[kv.first] = kv.second;
            nvtedg += kv.second.size();
        } else {
            for (const auto& kv2 : kv.second) {
                vted_[kv.first][kv2.first] = kv2.second;
                nvtedg++;
            }
        }
    } // end for
    } // end pragma omp critical

    } // end pragma omp parallel
    print_info << "ntrips=" << trip_.size() << std::endl;
    } // end generate rtv-graph

    print_info << "ctedges:" << std::endl;
    for (const auto& kv : cted_) {
        print_info << kv.first << ":";
        for (const auto& stid : kv.second)
            print_info << " " << stid;
        print_info << std::endl;
    }

    print_info << "vted_:" << std::endl;
    for (const auto& kv : vted_) {
        print_info << kv.first << ":";
        for (const auto& kv2 : kv.second)
            print_info << " " << kv2.first /*<< "(" << kv2.second << ")"*/;
        print_info << std::endl;
    }

    print_info << "trips:" << std::endl;
    for (const auto& kv : trip_) {
        print_info << kv.first << ":";
        for (const auto& cust : kv.second) 
            print_info << " " << cust.id();
        print_info << std::endl;
    }

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
    int ncol = nvtedg + waiting_customers().size();
    glp_add_cols(mip_, ncol);
    /*     - Total rows == # of vehicles (size of vted_), plus |customers| */
    int nrow = vted_.size() + waiting_customers().size();
    glp_add_rows(mip_, nrow);

    /* Map col_idx to the vehicle/trip edge */
    dict<size_t, std::pair<TripId, SharedTripId>> colmap {};

    /* Populate the coefficients in the objective */ {
    size_t col_idx = 0;
    for (const auto& kv : vted_) {
        const VehicleId& veh_id = kv.first;
        for (const auto& kv2 : kv.second) {
            const SharedTripId& st_id = kv2.first;
            col_idx++;
            colmap[col_idx] = {veh_id, st_id};
            glp_set_obj_coef(mip_, col_idx, kv2.second);
            glp_set_col_kind(mip_, col_idx, GLP_BV);
            glp_set_col_name(mip_, col_idx, ("x_"+std::to_string(veh_id)+"_"
                        +std::to_string(st_id)).c_str());
        }
    }
    for (const auto& cust : waiting_customers()) {
        col_idx++;
        colmap[col_idx] = {cust.id(), -1};
        glp_set_obj_coef(mip_, col_idx, Cargo::basecost(cust.id()));
        glp_set_col_kind(mip_, col_idx, GLP_BV);
        glp_set_col_name(mip_, col_idx, ("y_"+std::to_string(cust.id())).c_str());
    }
    assert(col_idx == (size_t) ncol);
    } // end populate coefficients in objective

    /* Populate cells of the constraints matrix */
    std::vector<int> ia(ncol*nrow+1); // <-- 1-indexed
    std::vector<int> ja(ncol*nrow+1);
    std::vector<double> ar(ncol*nrow+1);
    /* For constraint 1) each vehicle serves 0 or 1 trips:
     *     ia[cel_idx] = row_idx
     *     ja[cel_idx] = col_idx
     *     ar[cel_idx] = 1 if the i of col matches the veh of row_idx,
     *                   0 otherwise */
    size_t row_idx = 0;
    size_t cel_idx = 0;
    for (const auto& kv : vted_) {
        row_idx++;
        glp_set_row_name(mip_, row_idx, ("veh"+std::to_string(kv.first)).c_str());
        glp_set_row_bnds(mip_, row_idx, GLP_UP, 0.0, 1.0);
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
    for (const auto& cust : waiting_customers()) {
        row_idx++;
        glp_set_row_name(mip_, row_idx, ("cust"+std::to_string(cust.id())).c_str());
        glp_set_row_bnds(mip_, row_idx, GLP_FX, 1.0, 1.0);
        for (const auto& kv2 : colmap) {
            cel_idx++;
            ia[cel_idx] = row_idx;
            ja[cel_idx] = kv2.first;
            if (kv2.second.first == cust.id())
                ar[cel_idx] = 1;
            else if (kv2.second.second == -1)
                ar[cel_idx] = 0;
            else {
                ar[cel_idx] = 0;
                for (const auto& cust2 : trip_.at(kv2.second.second)) {
                    if (cust2.id() == cust.id()) {
                        ar[cel_idx] = 1;
                        break;
                    }
                }
            }
        }
    }
    assert(cel_idx == (size_t) ncol*nrow);


    // for (const auto& kv : colmap)
    //     print_out << " x_" << kv.second.first << "_" << kv.second.second;
    // print_out << std::endl;
    // for (int i = 1; i <= nrow; ++i) {
    //     for (int j = 1; j <= ncol; ++j) 
    //         print_out << ar[(i-1)*ncol+(j)];
    //     print_out << std::endl;
    // }

    glp_load_matrix(mip_, cel_idx, ia.data(), ja.data(), ar.data());
    glp_iocp cparams;
    glp_init_iocp(&cparams);
    cparams.presolve = GLP_ON;
    int rc = glp_intopt(mip_, &cparams); // <-- solve!

    print_out << "mip return:" << std::endl;
    switch (rc) {
        case 0: print_out << "good\n"; break;
        case GLP_EBOUND:    print_error << "glp_ebound\n";    break;
        case GLP_EROOT:     print_error << "glp_eroot\n";     break;
        case GLP_ENOPFS:    print_error << "glp_enopfs\n";    break;
        case GLP_ENODFS:    print_error << "glp_enodfs\n";    break;
        case GLP_EFAIL:     print_error << "glp_efail\n";     break;
        case GLP_EMIPGAP:   print_error << "glp_emipgap\n";   break;
        case GLP_ETMLIM:    print_error << "glp_etmlim\n";    break;
        case GLP_ESTOP:     print_error << "glp_estop\n";     break;
    }
    int status = glp_mip_status(mip_);
    print_out << "mip status:" << std::endl;
    switch (status) {
        case GLP_UNDEF: print_out << "glp_undef\n"; break;
        case GLP_OPT:   print_out << "glp_opt\n";   break;
        case GLP_FEAS:  print_out << "glp_feas\n";  break;
        case GLP_NOFEAS:print_out << "glp_nofeas\n";break;
    }

    double z = glp_mip_obj_val(mip_);
    print_out << z << std::endl;

    std::vector<int> res(ncol+1);
    for (int i = 1; i <= ncol; ++i)
        res[i] = glp_mip_col_val(mip_, i);

    for (int i = 1; i <= ncol; ++i) {
        print_out << "veh" << colmap[i].first << "-st" << colmap[i].second
            << ": " << res[i] << std::endl;
    }

    for (int i = 1; i <= ncol; ++i) {
        if (colmap[i].second == -1) // <-- -1=customer penalty
            continue; // <-- don't do anything for unassigned customers
        int res = glp_mip_col_val(mip_, i);
        if (res == 1) {
            commit(trip_.at(colmap[i].second),
                   vehmap.at(colmap[i].first),
                   vt_rte.at(colmap[i].first).at(colmap[i].second),
                   vt_sch.at(colmap[i].first).at(colmap[i].second));
            for (const auto& cust : trip_.at(colmap[i].second))
                print_success << "Match (cust" << cust.id()
                              << ", veh"       << colmap[i].first << ")\n";
            nmat_++;
        }
    }

    glp_delete_prob(mip_);

    } // end if vted_.size() > 0

    /* Cleanup */
    rvgrph_rr_.clear();
    rvgrph_rv_.clear();
    trip_.clear();
    vted_.clear();
    cted_.clear();
}

void TripVehicleGrouping::end() {
    print_success << "Matches: " << nmat_ << std::endl; // Print a msg
}

void TripVehicleGrouping::listen() {
    grid_.clear();          // Clear the index...
    RSAlgorithm::listen();  // ...then call listen()
}

/* Function travel
 * Returns true if the vehicle (param 1) can serve the set of requests (param 2)
 * with time constraints of all participants. Outputs the cost (param 3),
 * the new schedule (param 4), and the route (param 5). */
bool TripVehicleGrouping::travel(const Vehicle              & veh,
                                 const std::vector<Customer>& customers,
                                 DistanceInt                & cstout,
                                 std::vector<Stop>          & schout,
                                 std::vector<Waypoint>      & rteout,
                                 GTree::G_Tree              & gtree) {
    MutableVehicle mutveh = veh; // make a mutable copy
    DistanceInt total_cost = 0;
    std::vector<Stop> schout_;
    std::vector<Waypoint> rteout_;
    cstout = -1;

    /* Insert customers one by one */
    for (const Customer& cust : customers) {
        DistanceInt cost = sop_insert(mutveh, cust, schout_, rteout_, gtree);

        if (check_timewindow_constr(schout_, rteout_)) {
            total_cost += cost;
            mutveh.set_schedule(schout_);
            mutveh.set_route(rteout_);
            mutveh.reset_lvn();
        } else // <-- a customer failed to be inserted
            return false;
    }
    cstout = total_cost - Cargo::basecost(veh.id());
    schout = schout_;
    rteout = rteout_;
    return true;
}

SharedTripId TripVehicleGrouping::add_trip(const SharedTrip & trip) {
    for (const auto& kv : trip_)
        if (kv.second == trip)
            return kv.first;
    stid_++;
    trip_[stid_] = trip;
    for (const auto& cust : trip)
        cted_[cust.id()].push_back(stid_);
    return stid_;
}

int main() {
    /* Set the options */
    cargo::Options op;
    op.path_to_roadnet = "../../data/roadnetwork/mny.rnet";
    op.path_to_gtree   = "../../data/roadnetwork/mny.gtree";
    op.path_to_edges   = "../../data/roadnetwork/mny.edges";
    op.path_to_problem = "../../data/benchmark/rs-sm-4.instance";
    op.path_to_solution= "a.sol";
    op.time_multiplier = 1;
    op.vehicle_speed   = 10;
    op.matching_period = 60;

    cargo::Cargo cargo(op);

    /* Initialize a new tvg */
    TripVehicleGrouping tvg;

    /* Start Cargo */
    cargo.start(tvg);
}
