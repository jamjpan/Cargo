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

/* The Trip Assignment Problem
 *
 * Citation:
 *
 * Javier Alonso-Mora, Samitha Samaranayake, Alex Wallar, Emilio Frazzoli, and
 * Daniela Rus. "On-demand high-capacity ride-sharing via dynamic trip-vehicle
 * assignment". PNAS 114(3), 2017, pp.462-467.
 *
 * The problem minimizes the cost of assigning vehicles to shared trips, plus a
 * penalty for each unserved customer. Given a set of trips, a cost c_ij gives
 * the detour cost of vehicle i serving trip j; a binary variable x_ij equals 1
 * if the vehicle serves the trip and 0 otherwise. A seperate cost C_k gives
 * the travel cost of customer k (equal to the shortest-path from the
 * customer's origin to destination), and a binary variable y_k equals 1 if the
 * customer is unserved by any trip and 0 otherwise. The objective function is:
 *
 *     min z = sum(c_ij*x_ij)        for all i,j
 *             + sum(C_k*y_k)        for all k
 *
 * The problem is constrained by two sets of equations. The first set retricts
 * vehicles to serve only 1 trip:
 *
 *     Given vehicle i, 0 <= sum over j (x_ij) <= 1 for trips j where vehicle i
 *     can serve j within window constraints
 *
 * The second set restricts customers to be served only once, or not served at
 * all:
 *
 *     Given customer k, y_k + sum over i,j (x_ij) == 1 for vehicle-trip pair
 *     ij where trip j includes customer k
 *
 * Trips are constructed iteratively using the steps in the paper. Briefly,
 * customer pairs are formed based on if the two trips are shareable; then
 * customer-vehicle pairs are formed; then trips of larger sizes (>2) are
 * formed iteratively.
 *
 * In this example, GLPK is used to solve the minimization problem. But the
 * performance of GLPK is quite poor compared with commercial options.
 */
 
/* To save typing typdef unordered_map to dict */
template <typename K, typename V> using dict = std::unordered_map<K, V>;

/* Semantics */
typedef int SharedTripId;
typedef std::vector<Customer> SharedTrip;

/* Strategy
 *
 * The default RSAlgorithm::listen() method calls match() at every batch_time().
 * Thus during match(), we will form trips from the unassigned customers and
 * active vehicles, then construct the binary integer linear program to solve
 * the assignment problem, then call glpk to do the actual solving. Vehicles
 * will be updated based on their assignments through calls to
 * RSAlgorithm::commit(). We will use a grid index to help construct the rv
 * pairs when building the rv-graph.
 */
class TripVehicleGrouping : public cargo::RSAlgorithm {
public:
    TripVehicleGrouping();

    /* My Overrides */
    virtual void handle_vehicle (const cargo::Vehicle &);
    virtual void match();
    virtual void end();
    virtual void listen();

private:
    int nmat_; // number of matches
    cargo::Grid grid_;

    /* RV-graph */
    dict<Customer, std::vector<Customer>> rvgrph_rr_;
    dict<Vehicle , std::vector<Customer>> rvgrph_rv_;

    /* RTV-graph */
    SharedTripId stid_;
    dict<VehicleId, dict<SharedTripId, DistanceInt>> vted_; // vehl-trip edges
    dict<CustomerId, std::vector<SharedTripId>>      cted_; // cust-trip edges
    dict<SharedTripId, SharedTrip>                   trip_; // trip lookup

    /* Function travel
     * In the paper, travel uses enumeration for vehicles with small capacity,
     * then the insertion method for each request above that capacity. We just
     * use the insertion method for all requests. The result is tradeoff
     * quality for speed. */
    bool travel(const Vehicle &,                    // Can this vehl...
                const std::vector<Customer> &,      // ...serve these custs?
                DistanceInt &,                      // cost of serving
                std::vector<Stop> &,                // resultant schedule
                std::vector<Waypoint> &,            // resultant route
                GTree::G_Tree &);                   // gtree to use for sp

    /* Function add_trip
     * Add SharedTrip into trip_. If SharedTrip already exists in trip_, return
     * its id. Also add to ctedges */
    SharedTripId add_trip(const SharedTrip &);

};

