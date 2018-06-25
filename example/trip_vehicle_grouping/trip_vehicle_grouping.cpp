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
    nmatches = 0;               // Initialize my private counter
    stid_ = 0;                  // Initialize internal SharedTripId
    mip = glp_create_prob();    // Initialize my binary integer problem
}

void TripVehicleGrouping::handle_customer(const cargo::Customer &) {/* null */}

void TripVehicleGrouping::handle_vehicle(const cargo::Vehicle& veh) {
    grid_.insert(veh); // Insert into my grid
}

void TripVehicleGrouping::match() {
    /* Set mip name */
    std::string prob_name = "mip (t=" + std::to_string(Cargo::now());
    glp_set_prob_name(mip, prob_name.c_str());

    /* Set objective to minimize */
    glp_set_obj_dir(mip, GLP_MIN);

    /* Generate rv-graph */ {
    int num_edges = 0;
      /* All threads share a local snapshot of customers (not sure if needed?) */
    std::vector<Customer>   local_customers     = waiting_customers();
    #pragma omp parallel shared(num_edges, rvgrph_rr, rvgrph_rv, local_customers)
    { /* Each thread gets a local gtree and a local grid to perform
       * sp-computations in parallel */
    GTree::G_Tree           local_gtree         = GTree::get();
    Grid                    local_grid          = grid_;
    #pragma omp for
    for (auto itr = local_customers.begin(); itr < local_customers.end(); ++itr) {
        /* 1) Create a virtual vehicle for the first customer and compare
         * against the remaining customers */
        const Customer& cust1 = *itr;
        Vehicle vrtveh(cust1.id(), cust1.origin(), cust1.destination(), cust1.early(), cust1.late(), 0);
        for (const Customer& cust2 : waiting_customers()) {
            if (cust1 == cust2) continue;
            DistanceInt cstout = 0;
            if (travel(vrtveh, {cust2}, cstout, local_gtree)) {
                /* Only one thread can write at a time */
                #pragma omp critical
                { rvgrph_rr[cust1].push_back(cust2);
                  num_edges++; }
            }
        }
        /* 2) Compare the first customer against candidate vehicles */
        DistanceInt range = pickup_range(cust1, Cargo::now());
        auto candidates = local_grid.within_about(range, cust1.origin());
        for (const auto& mutvehptr : candidates) {
            Vehicle veh = *mutvehptr;
            DistanceInt cstout = 0;
            if (travel(veh, {cust1}, cstout, local_gtree)) {
                #pragma omp critical
                { rvgrph_rv[veh].push_back(cust1);
                  num_edges++; }
            }
        }
    } // end pragma omp for
    } // end pragma omp parallel
    print_info << "rvgrph_edges=" << num_edges << std::endl;
    } // end generate rv-graph

    int num_vtedges = 0;
    /* Generate rtv-graph */ {
    std::vector<Vehicle>    local_vehicles      = vehicles();
    #pragma omp parallel shared(vtedges, num_vtedges, rvgrph_rr, rvgrph_rv, local_vehicles)
    { /* Each thread adds edges to a local_vtedges, then these are combined when
       * all threads have completed */
    GTree::G_Tree           local_gtree         = GTree::get();
    dict<VehicleId, dict<SharedTripId, DistanceInt>>
                            local_vtedges       = {};
    #pragma omp for
    for (auto itr = local_vehicles.begin(); itr < local_vehicles.end(); ++itr) {
        const Vehicle& veh = *itr;

        /* Store trips of size_t for this vehicle */
        dict<size_t, std::vector<SharedTripId>> tripk;

        /* 1) Trips of size 1 */
        print_info << omp_get_thread_num() << " tripk1" << std::endl;
        /* Do only if a vehicle-customer pair exists for this vehicle */
        if (rvgrph_rv.count(veh) > 0) {
        /* Add a trip of size 1 for each vehicle-customer pair for this veh */
        for (const Customer& cust : rvgrph_rv.at(veh)) {
            DistanceInt cstout = 0;
            travel(veh, {cust}, cstout, local_gtree); // <-- repeated..!!
            /* Only one thread can add_trip at a time */
            SharedTripId stid;
            #pragma omp critical
            { stid = add_trip({cust}); }
            tripk[1].push_back(stid);
            local_vtedges[veh.id()][stid] = cstout;
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
            SharedTrip trip(trips.at(i));
            /* Compare against all other size 1 trips */
            for (SharedTripId j : tripk.at(1)) {
                if (i == j) continue;
                trip.insert(trip.end(), trips.at(j).begin(), trips.at(j).end());
                DistanceInt cstout = 0;
                if (travel(veh, trip, cstout, local_gtree)) {
                    SharedTripId stid;
                    #pragma omp critical
                    { stid = add_trip({trip}); }
                    tripk[2].push_back(stid);
                    local_vtedges[veh.id()][stid] = cstout;
                }
            }
        }
        /* Check if any rr pairs can be served by this vehicle */
        for (const auto& kv : rvgrph_rr) {
            const Customer& cust1 = kv.first;
            for (const Customer& cust2 : kv.second) {
                DistanceInt cstout = 0;
                if (travel(veh, {cust1, cust2}, cstout, local_gtree)) {
                    SharedTripId stid;
                    #pragma omp critical
                    { stid = add_trip({cust1, cust2}); }
                    tripk[2].push_back(stid);
                    local_vtedges[veh.id()][stid] = cstout;
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
                const SharedTrip& trip1 = trips.at(i);
                /* Compare against remaining trips of size k-1 */
                for (SharedTripId j : tripk.at(k-1)) {
                    if (i == j) continue;
                    const SharedTrip& trip2 = trips.at(j);
                    /* Join trip1 and trip2 into a new trip (no duplicates) */
                    SharedTrip trip(trip1);
                    for (const Customer& cust : trip2)
                        if (std::find(trip.begin(), trip.end(), cust) != trip.end())
                            trip.push_back(cust);
                    /* If trip1 JOIN trip2 (no duplicates) is size k... */
                    if (trip.size() == k) {
                        bool ok = true;
                        /* ... check each subtrip if it is already a trip */
                        for (size_t p = 0; p < trip.size(); ++p) {
                            SharedTrip subtrip(trip);
                            subtrip.erase(subtrip.begin() + p);
                            for (SharedTripId q : tripk.at(k-1))
                                if (subtrip != trips.at(q))
                                    ok = false;
                        }
                        /* Add the full trip if each subtrip is already a trip,
                         * and the full trip itself is valid */
                        if (ok) {
                            DistanceInt cstout = 0;
                            if (travel(veh, trip, cstout, local_gtree)) {
                                SharedTripId stid;
                                #pragma omp critical
                                { stid = add_trip(trip); }
                                tripk[k].push_back(stid);
                                local_vtedges[veh.id()][stid] = cstout;
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

    /* Combine local_vtedges */
    #pragma omp critical
    { /* Copy the local into the global */
        for (const auto& kv : local_vtedges) {
            if (vtedges.count(kv.first) == 0) {
                vtedges[kv.first] = kv.second;
                num_vtedges += kv.second.size();
            }
            else
                for (const auto& kv2 : kv.second) {
                    vtedges[kv.first][kv2.first] = kv2.second;
                    num_vtedges++;
                }
        }
    }

    } // end pragma omp parallel
    print_info << "ntrips=" << trips.size() << std::endl;
    } // end generate rtv-graph

    /* Populate penalties */
    for (const auto& cust : waiting_customers())
        penalties[cust.id()] = Cargo::basecost(cust.id());

    print_info << "ctedges:" << std::endl;
    for (const auto& kv : ctedges) {
        print_info << kv.first << ":";
        for (const auto& stid : kv.second)
            print_info << " " << stid;
        print_info << std::endl;
    }

    print_info << "vtedges:" << std::endl;
    for (const auto& kv : vtedges) {
        print_info << kv.first << ":";
        for (const auto& kv2 : kv.second)
            print_info << " " << kv2.first /*<< "(" << kv2.second << ")"*/;
        print_info << std::endl;
    }

    print_info << "trips:" << std::endl;
    for (const auto& kv : trips) {
        print_info << kv.first << ":";
        for (const auto& cust : kv.second) 
            print_info << " " << cust.id();
        print_info << std::endl;
    }

    /* Generating constraints
     * Total number of constraints (rows) = |trips| + |waiting_customers|
     * Total number of variables (cols) = vt-edges + |waiting_customers| */
//    int num_rows = trips.size() + waiting_customers().size();
//    int num_cols = num_vtedges + penalties.size();
//    int num_cons = num_rows * num_cols;
//
//    dict<size_t, std::pair<VehicleId, SharedTripId>> col_to_vtedge {};
//    dict<size_t, CustomerId> col_to_cust {}; // <-- for penalties
//    /* Add cols to mip */ {
//    size_t col_idx = 1;
//    glp_add_cols(mip, num_cols);
//    /* Add a column for each vtedge */
//    for (const auto& kv : vtedges)
//        for (const auto& kv2 : kv.second) {
//            glp_set_col_name(mip, col_idx, ("x_"+std::to_string(kv.first)+"_"+std::to_string(kv2.first)).c_str());
//            glp_set_col_kind(mip, col_idx, GLP_IV);
//            glp_set_obj_coef(mip, col_idx, kv2.second);
//            /* Add to mapping */
//            col_to_vtedge[col_idx] = {kv.first, kv2.first};
//            col_idx++;
//        }
//    /* Add a column for each customer (penalty) */
//    for (const auto& kv : penalties) {
//        glp_set_col_name(mip, col_idx, ("y_"+std::to_string(kv.first)).c_str());
//        glp_set_col_kind(mip, col_idx, GLP_IV);
//        glp_set_obj_coef(mip, col_idx, kv.second);
//        /* Add to mapping */
//        col_to_cust[col_idx] = kv.first;
//        col_idx++;
//    }
//    assert(col_idx-1 == (size_t) num_cols);
//    } // end add cols
//
//    std::vector<int> ia(num_cons+1); // <-- 1-indexed
//    std::vector<int> ja(num_cons+1);
//    std::vector<int> ar(num_cons+1);
//
//    dict<size_t, SharedTripId> row_to_trip {};
//    dict<size_t, CustomerId> row_to_cust {};
//    /* Add rows to mip
//     * There are two contraints.
//     *   1) A trip is served by at most one vehicle;
//     *   2) A customer is either serviced or not serviced;
//     * The first constraint is a set of |vtedge| */ {
//    size_t row_idx = 1;
//    size_t cel_idx = 1;
//    glp_add_rows(mip, num_rows);
//    /* Add a row for each trip */
//    for (const auto& kv : trips) {
//        SharedTripId stid = kv.first;
//        SharedTrip trid = kv.second;
//        glp_set_row_name(mip, row_idx, std::to_string(row_idx).c_str());
//        glp_set_row_bnds(mip, row_idx, GLP_UP, 0.0, 1.0);
//        /* Loop through vtedges. If edge from vehicle to trip, set coef to 1;
//         * otherwise set coef to 0. */
//        for (const auto& kv2 : col_to_vtedge) {
//            size_t col_idx = kv2.first;
//            std::pair<VehicleId, SharedTripId> edge = kv2.second;
//            if (edge.second == stid) {
//                ia[cel_idx] = row_idx;
//                ja[cel_idx] = col_idx;
//                ar[cel_idx] = 1;
//                cel_idx++;
//            } else {
//                ia[cel_idx] = row_idx;
//                ja[cel_idx] = col_idx;
//                ar[cel_idx] = 0;
//                cel_idx++;
//            }
//        }
//        for (const auto& kv : col_to_cust) {
//            ia[cel_idx] = row_idx;
//            ja[cel_idx] = kv.first;
//            ar[cel_idx] = 0;
//            cel_idx++;
//        }
//        row_idx++;
//    }
//    /* Add a row for each customer */
//    for (const auto& kv : col_to_cust) {
//        CustomerId cust_id = kv.second;
//        ia[cel_idx] = row_idx;
//        ja[cel_idx] = kv.first;
//        ar[cel_idx] = 1;
//        cel_idx++;
//        for (const auto& kv2 : col_to_vtedge) {
//            size_t col_idx = kv2.first;
//            SharedTripId stid = kv2.second.second;
//            const std::vector<SharedTripId>& edges = ctedges.at(cust_id);
//            if (std::find(edges.begin(), edges.end(), stid) != edges.end()) {
//                ia[cel_idx] = row_idx;
//                ja[cel_idx] = col_idx;
//                ar[cel_idx] = 1;
//                cel_idx++;
//            } else {
//                ia[cel_idx] = row_idx;
//                ja[cel_idx] = col_idx;
//                ar[cel_idx] = 0;
//                cel_idx++;
//            }
//        }
//        row_idx++;
//    }
//    } // end add rows
//
//    for (int i = 1; i <= num_rows; ++i) {
//        for (int j = 1; j <= num_cols; ++j) 
//            print_out << ar[(i-1)*num_cols+(j)];
//        print_out << std::endl;
//    }

    /* Cleanup */
    // glp_erase_prob(mip);
    rvgrph_rr.clear();
    rvgrph_rv.clear();
    trips.clear();
    penalties.clear();
    vtedges.clear();
    ctedges.clear();
}
    

void TripVehicleGrouping::end() {
    print_success << "Matches: " << nmatches << std::endl; // Print a msg
}

void TripVehicleGrouping::listen() {
    grid_.clear();          // Clear the index...
    RSAlgorithm::listen();  // ...then call listen()
}

/* Function travel
 * Returns true if the vehicle (param 1) can serve the set of requests (param 2)
 * with time constraints of all participants. Outputs the cost (param 3) */
bool TripVehicleGrouping::travel(const Vehicle              & veh,
                                 const std::vector<Customer>& customers,
                                 DistanceInt                & cstout,
                                 GTree::G_Tree              & gtree) {
    MutableVehicle mutveh = veh; // make a mutable copy
    DistanceInt total_cost = 0;
    cstout = -1;

    /* Insert customers one by one */
    for (const Customer& cust : customers) {
        std::vector<Stop> schout;
        std::vector<Waypoint> rteout;

        DistanceInt cost = sop_insert(mutveh, cust, schout, rteout, gtree);

        if (check_timewindow_constr(schout, rteout)) {
            total_cost += cost;
            mutveh.set_schedule(schout);
            mutveh.set_route(rteout);
            mutveh.reset_lvn();
        } else // <-- a customer failed to be inserted
            return false;
    }
    cstout = total_cost;
    return true;
}

SharedTripId TripVehicleGrouping::add_trip(const SharedTrip & trip) {
    for (const auto& kv : trips)
        if (kv.second == trip)
            return kv.first;
    stid_++;
    trips[stid_] = trip;
    for (const auto& cust : trip)
        ctedges[cust.id()].push_back(stid_);
    return stid_;
}

std::vector<VehicleId>
TripVehicleGrouping::select_vehicles(const SharedTripId& stid) {
    std::vector<VehicleId> res;
    for (const auto& kv : vtedges)
        for (const auto& kv2 : kv.second)
            if (kv2.first == stid) {
                res.push_back(kv.first);
                break;
            }
    return res;
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
