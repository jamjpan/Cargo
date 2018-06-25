// MIT License
//
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
#ifndef CARGO_INCLUDE_LIBCARGO_CARGO_H_
#define CARGO_INCLUDE_LIBCARGO_CARGO_H_
#include <fstream>
#include <map>
#include <mutex>
#include <unordered_map>

#include "classes.h"
#include "file.h"
#include "functions.h"
#include "message.h"
#include "options.h"
#include "rsalgorithm.h"
#include "types.h"
#include "../gtree/gtree.h"
#include "../sqlite3/sqlite3.h"

// Cargo is a simulator and requests generator. Its two functionalities are to
// simulate the movement of vehicles, and to generate customer requests and
// vehicles in "real time". These are generated from a "problem instance", a
// special text file that lists customers, vehicles, and their properties.
//
// Vehicle movement is simulated based on vehicle speed, a road network,
// vehicle routes, and time. Speed determines how much distance is covered at
// every simulation timestep. Routes are sequences of nodes through the road
// network. By default, all vehicles travel along the shortest path from their
// origin to their destination.
//
// Cargo will poll an internal sqlite3 database for new vehicle routes. These
// routes can be updated through an RSAlgorithm.
namespace cargo {

class Cargo {
public:
    Cargo(const Options&);
    ~Cargo();
    const std::string&          name();                 // e.g. rs-lg-5
    const std::string&          road_network();         // e.g. mny, cd1, bj5
    void                        start(RSAlgorithm&);
    void                        start();
    /* Accessors */
    static Point                node2pt(const NodeId& i){ return nodes_.at(i); }
    static DistanceInt          edgeweight(const NodeId& u, const NodeId& v) { return edges_.at(u).at(v); }
    static DistanceInt          basecost(const TripId& i) { return trip_costs_.at(i); }
    static BoundingBox          bbox()                  { return bbox_; }
    static Speed&               vspeed()                { return speed_; }
    static SimTime              now()                   { return t_; }
    static GTree::G_Tree&       gtree()                 { return gtree_;}
    static sqlite3*             db()                    { return db_; }

    static std::mutex dbmx;                             // protect the db

private:
    Message print_out;
    Message print_info;
    Message print_warning;
    Message print_error;
    Message print_success;

    ProblemSet probset_;

    SimTime tmin_;                                      // max trip.early
    SimTime tmax_;                                      // max vehicle.late
    SimTime matching_period_;                           // customer timeout

    size_t total_vehicles_;
    size_t total_customers_;
    size_t active_vehicles_;
    int sleep_interval_;                                // 1 sec/time_multiplier

    size_t base_cost_;                                  // total base cost

    /* Global vars */
    static KeyValueNodes nodes_;                        // nodes_[u] = Point
    static KeyValueEdges edges_;                        // edges_[u][v] = w
    static BoundingBox bbox_;
    static GTree::G_Tree gtree_;
    static sqlite3* db_;
    static Speed speed_;
    static SimTime t_;                                  // current sim time
    static std::unordered_map<TripId, DistanceInt>      // indiv. base costs
        trip_costs_;

    /* Solution tables
     * (ordered, so we know exactly what to expect when iterating) */
    Filepath solution_file_;
    std::ofstream f_sol_temp_;                          // sol.partial
    std::vector<VehicleId> sol_veh_cols_;               // veh column headers
    std::map<VehicleId, NodeId> sol_routes_;
    std::map<CustomerId, std::pair<CustomerStatus, VehicleId>>
        sol_statuses_;

    /* SQL statements */
    SqliteReturnCode rc;
    SqliteErrorMessage err;
    sqlite3_stmt* tim_stmt;                             // timeout customers
    sqlite3_stmt* sac_stmt;                             // select all customers
    sqlite3_stmt* sar_stmt;                             // select all routes
    sqlite3_stmt* ssv_stmt;                             // select step vehicles
    sqlite3_stmt* ucs_stmt;                             // update cust status
    sqlite3_stmt* dav_stmt;                             // deactivate vehicle
    sqlite3_stmt* pup_stmt;                             // pickup
    sqlite3_stmt* drp_stmt;                             // dropoff
    sqlite3_stmt* vis_stmt;                             // visitedAt
    sqlite3_stmt* sch_stmt;                             // schedule
    sqlite3_stmt* lvn_stmt;                             // last-visited node
    sqlite3_stmt* nnd_stmt;                             // nearest-node dist

    void initialize(const Options &);

    /* Returns number of stepped vehicles.
     * Outputs number of deactivated vehicles. */
    int step(int&);

    void record_customer_statuses();
    DistanceInt total_route_cost();
};

} // namespace cargo

#endif // CARGO_INCLUDE_LIBCARGO_CARGO_H_

