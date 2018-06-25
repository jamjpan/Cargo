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
#include "libcargo.h"

#include "glpk/glpk.h"

using namespace cargo;

// The Trip Assignment Problem
// ---------------------------
// minimize: sum(weight[i][j]*x[i][j]) + sum(base_cost[k]*y[k])
//     where i in vehicles, j in trips, k in customers,
//         weight is the cost of vehicle i serving trip j,
//         base_cost is sp(k.origin, k.destination) for customer k
//
// (x[i][j] only exists if there is an edge between vehicle i and trip j
//  in the RTV-graph)
//
// subject to:
//     Constraint 1 (a vehicle serves at most one trip)
//     sum(x[1][j]) <= 1, j in trips
//     sum(x[2][j]) <= 1, j in trips
//     ...
//     sum(x[i][j]) <= 1, j in trips
//         for all i
//
//     Constraint 2 (a vehicle serves a trip that includes k, or k is not served)
//     sum(x[1][j] + y[k]) = 1, j in trips that include customer k
//     sum(x[2][j] + y[k]) = 1, j in trips that include customer k
//     ...
//     sum(x[i][j] + y[k]) = 1, j in trips that include customer k
//         for all i, k
//
// Strategy
// --------
// unordered maps for weight, x; indices i and k will but VehicleId and CustomerIds;
// index j will be a SharedTripId to distinguish from Cargo's TripId.
//
// To take into account the penalty in the objective, write all the terms, then
// substitute a single binary variable to represent x's and y's. E.g.:
//
//     c[1][1]*x[1][1] + c[1][2]*x[1][2] + c[2][1]*x[2][1] + p[1]*y[1] + p[2]*y[2] + p[3]*y[3]
//
// becomes
//
//     C[1]*X[1] + C[2]*X[2] + C[3]*X[3] + C[4]*X[4] + C[5]*X[5] + C[6]*X[6]
//
// Then write constraints in terms of the new variable, e.g.
//
//     1*X[1] + 1*X[2] + 0*X[3] + 0*X[4] + 0*X[5] + 0*X[6] <= 1
//     (means vehicle 1 serving trip 1 or 2 (x[1][1], x[1][2]) can only serve
//     one or the other, but not both, or neither).
//
// The objective transforms into two vectors, C and X. The constraints transform
// into vectors of 1's and 0's of size |X|.
//
// To save typing we can typdef unordered_map to dict.
template <typename Key, typename Value>
using dict = std::unordered_map<Key, Value>;

typedef int SharedTripId;
typedef std::vector<Customer> SharedTrip;

class TripVehicleGrouping : public cargo::RSAlgorithm { // <-- inherit from the base
public:
    TripVehicleGrouping();

    /* My Overrides */
    virtual void handle_customer(const cargo::Customer &);
    virtual void handle_vehicle(const cargo::Vehicle &);
    virtual void match();
    virtual void end();
    virtual void listen();

private: // <-- custom variables and functions
    int nmatches;
    cargo::Grid grid_; // grid index

    /* GLPK program */
    glp_prob* mip;

    /* RV-graph */
    dict<Customer, std::vector<Customer>> rvgrph_rr;
    dict<Vehicle, std::vector<Customer>> rvgrph_rv;

    /* RTV-graph */
    SharedTripId stid_; // shared trip id
    dict<VehicleId, dict<SharedTripId, DistanceInt>> vtedges;
    dict<CustomerId, std::vector<SharedTripId>> ctedges;
    dict<SharedTripId, SharedTrip> trips;

    /* Unassigned requests */
    dict<CustomerId, DistanceInt> penalties;

    /* Function travel
     * In the paper, travel uses enumeration for vehicles with small capacity,
     * then the insertion method for each request above that capacity. We just
     * use the insertion method for all requests. */
    bool travel(const Vehicle &, const std::vector<Customer> &, DistanceInt &, GTree::G_Tree &);

    /* Function add_trip
     * Add SharedTrip into trips. If SharedTrip already exists in trips, return
     * its existing id. Otherwise, give it an id, add to trips, and return the id.
     * Also add to ctedges */
    SharedTripId add_trip(const SharedTrip &);

    /* Function select_vehicles
     * Given a SharedTripId, retrieve all vehicles with an edge to the trip */
    std::vector<VehicleId> select_vehicles(const SharedTripId &);

};

